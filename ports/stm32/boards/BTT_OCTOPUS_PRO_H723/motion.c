/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 Jay McClellan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/misc.h"
#include "py/objlist.h"

#include "motion.h"
#include "timer.h"
#include "pin.h"
#include "micromoco.h"

#define MOTION_CLOCK_HZ (1000000)

// Matches the driver-timing default moco_rig_init() seeds internally
// (moco_design.md §5.1); kept in sync by hand since the C layer doesn't
// expose it as a symbol.
#define MOTION_DEFAULT_TIMING_US ((moco_float)5)

// Seeded once per channel at construction, in raw steps (unit_scale is still
// 1.0 at that point), so a freshly built Rig can move without an explicit
// rates() call. Binding-level convenience only -- moco_rig_init() itself
// still leaves these at zero (moco_design.md §5.1). If a channel later gets
// real units via stepper(rotation_distance=...) or scale(), these numbers
// get reinterpreted in the new units and are almost certainly no longer
// sensible; call rates() explicitly whenever real units are in effect.
#define MOTION_DEFAULT_VMAX ((moco_float)1000) // steps/sec
#define MOTION_DEFAULT_AMAX ((moco_float)5000) // steps/sec^2

typedef struct _motion_rig_obj_t {
    mp_obj_base_t base;
    mp_int_t n_channels, n_segs;
    size_t mem_size;
    moco_rig *rig; // == mem once successfully initialized, else NULL
    intptr_t next_handle; // Next node in active rig list
    uint32_t vjump_explicit_mask; // bit per channel: vjump ever set via rates()
    // Per-channel queue-end target, for move()/segment()'s target= fill-forward.
    // There's no C-layer getter for this (moco_rig_position() is live position,
    // not queue-end), so the binding tracks it: 0 at construction, updated after
    // every successful move()/segment(), resynced from moco_rig_position() after
    // stop() (which truncates the queue back to the live position).
    moco_float last_target[MOCO_MAX_CHANNELS];
} motion_rig_obj_t;

static const mp_obj_type_t motion_rig_type;

// Singly-linked list of active Rig objects
static intptr_t motion_active_rigs_head;

// Linked list handles are stored with the 1 bit set to prevent GC from following them
static inline motion_rig_obj_t *motion_rig_ptr(intptr_t handle) {
    return (motion_rig_obj_t *)(handle & ~(intptr_t)1);
}

// Minimal custom exception surface, per MicroPython's "do a lot with a
// little": QueueFull/Busy exist because a caller plausibly catches and
// reacts to them differently (backpressure vs. wait-and-retry); every other
// failure -- including "not initialized" -- is a plain Error with a message,
// not its own type.
MP_DEFINE_EXCEPTION(MotionError, RuntimeError)
MP_DEFINE_EXCEPTION(MotionQueueFull, MotionError)
MP_DEFINE_EXCEPTION(MotionBusy, MotionError)

/******************************************************************************/
// SegResult: a plain, mutable, four-field container. Constructed with no
// args and passed as result= to move()/segment()/dwell() to be filled in
// place instead of allocating a fresh return value each call.

typedef struct _motion_segresult_obj_t {
    mp_obj_base_t base;
    mp_obj_t duration_s;
    mp_obj_t v_end;
    mp_obj_t clamped_v;
    mp_obj_t clamped_a;
} motion_segresult_obj_t;

static const mp_obj_type_t motion_segresult_type;

static mp_obj_t motion_segresult_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    motion_segresult_obj_t *self = mp_obj_malloc(motion_segresult_obj_t, type);
    self->duration_s = mp_obj_new_float_from_f(0);
    self->v_end = mp_obj_new_float_from_f(0);
    self->clamped_v = mp_obj_new_list(0, NULL);
    self->clamped_a = mp_obj_new_list(0, NULL);
    return MP_OBJ_FROM_PTR(self);
}

static void motion_segresult_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest) {
    motion_segresult_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_obj_t *field;
    if (attr == MP_QSTR_duration_s) {
        field = &self->duration_s;
    } else if (attr == MP_QSTR_v_end) {
        field = &self->v_end;
    } else if (attr == MP_QSTR_clamped_v) {
        field = &self->clamped_v;
    } else if (attr == MP_QSTR_clamped_a) {
        field = &self->clamped_a;
    } else {
        dest[1] = MP_OBJ_SENTINEL; // not ours; fall back to locals_dict
        return;
    }
    if (dest[0] == MP_OBJ_NULL) {
        dest[0] = *field; // load
    } else if (dest[1] != MP_OBJ_NULL) {
        *field = dest[1]; // store
        dest[0] = MP_OBJ_NULL; // signal success
    }
}

static MP_DEFINE_CONST_OBJ_TYPE(
    motion_segresult_type,
    MP_QSTR_SegResult,
    MP_TYPE_FLAG_NONE,
    make_new, motion_segresult_make_new,
    attr, &motion_segresult_attr
    );

// Builds a fresh 0..n_channels-1 list[bool] from a moco_seg_result clamp bitmask.
static mp_obj_t motion_clamped_list(mp_int_t n_channels, uint32_t bits) {
    mp_obj_t items[MOCO_MAX_CHANNELS];
    for (mp_int_t i = 0; i < n_channels; i++) {
        items[i] = mp_obj_new_bool((bits >> i) & 1);
    }
    return mp_obj_new_list(n_channels, items);
}

/******************************************************************************/
// Shared hardware timer. One TIM24 interrupt drives every active Rig's
// moco_rig_update() -- enabled when the first Rig initializes, disabled when
// the last one deinitializes (motion_rig_make_new()/motion_rig_deinit() below).

static int motion_timer_refcount;
static uint32_t motion_tick;

void motion_init(void) {
    motion_active_rigs_head = 0;
    motion_timer_refcount = 0;
    motion_tick = 0;
}

// TIM24 is unused by MicroPython elsewhere on this MCU, so it's free to
// drive directly. Toggling PE15 each tick gives a square wave on the pin,
// to watch the interrupt's actual timing on a logic analyzer during
// bring-up; kept until it's no longer needed for that purpose.
void TIM24_IRQHandler(void) {
    pin_E15->gpio->BSRR = pin_E15->pin_mask;
    TIM24->SR = ~TIM_SR_UIF;

    uint32_t now = ++motion_tick;
    for (motion_rig_obj_t *self = motion_rig_ptr(motion_active_rigs_head); self; self = motion_rig_ptr(self->next_handle)) {
        if (self->rig) {
            moco_rig_update(self->rig, now);
        }
    }

    pin_E15->gpio->BSRR = pin_E15->pin_mask << 16;
}

static void motion_timer_enable(void) {
    if (motion_timer_refcount++ > 0) {
        // Some other Rig already has the timer running.
        return;
    }

    mp_hal_pin_output(pin_E15);
    mp_hal_pin_low(pin_E15);
    mp_hal_pin_high(pin_E15);
    mp_hal_pin_low(pin_E15);

    __HAL_RCC_TIM24_CLK_ENABLE();

    // Run the counter at its full source rate and use ARR to set the
    // MOTION_CLOCK_HZ update period, rather than prescaling down and using
    // ARR=0 -- with ARR=0 the counter reloads every clock edge and never
    // holds a nonzero value, so CNT is useless as a liveness check. `now`
    // for moco_rig_update() comes from motion_tick above, not CNT: CNT
    // free-runs at the source clock and wraps every ARR ticks, so it isn't
    // itself a MOTION_CLOCK_HZ-rate counter.
    TIM24->PSC = 0;
    TIM24->ARR = (timer_get_source_freq(24) / MOTION_CLOCK_HZ) - 1;
    TIM24->EGR = TIM_EGR_UG; // load PSC/ARR, reset CNT
    TIM24->SR = ~TIM_SR_UIF; // clear UIF set by the forced update above
    TIM24->DIER = TIM_DIER_UIE;
    TIM24->CR1 = TIM_CR1_CEN;

    NVIC_SetPriority(TIM24_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 2, 0));
    HAL_NVIC_EnableIRQ(TIM24_IRQn);
}

static void motion_timer_disable(void) {
    if (motion_timer_refcount == 0 || --motion_timer_refcount > 0) {
        // Unbalanced call, or another Rig still needs the timer running.
        return;
    }

    HAL_NVIC_DisableIRQ(TIM24_IRQn);
    TIM24->CR1 = 0;
    TIM24->DIER = 0;
    __HAL_RCC_TIM24_CLK_DISABLE();
}

static void motion_rig_ensure_initialized(motion_rig_obj_t *self) {
    if (!self->rig) {
        mp_raise_msg(&mp_type_MotionError, MP_ERROR_TEXT("Rig is not initialized"));
    }
}

static void motion_channel_check(motion_rig_obj_t *self, mp_int_t channel) {
    if (!(0 <= channel && channel < self->n_channels)) {
        mp_raise_msg_varg(&mp_type_IndexError, MP_ERROR_TEXT("channel must be 0 to %d, got %d"), (int)self->n_channels - 1, (int)channel);
    }
}

static moco_float motion_get_float_or(mp_obj_t obj, moco_float default_value) {
    return obj == mp_const_none ? default_value : mp_obj_get_float_to_f(obj);
}

// Only apply the push-pull/no-pull/full-speed default to a pin that isn't
// already some flavor of output -- mp_hal_pin_output() would otherwise
// clobber a caller's own open-drain, pull, or drive-strength configuration.
static void motion_ensure_output(const machine_pin_obj_t *pin) {
    uint32_t mode = pin_get_mode(pin);
    if (mode != GPIO_MODE_OUTPUT_PP && mode != GPIO_MODE_OUTPUT_OD) {
        mp_hal_pin_output(pin);
    }
}

static void motion_check_status(moco_status status) {
    if (status == MOCO_OK) {
        return;
    }
    if (status == MOCO_ERR_BUSY) {
        mp_raise_msg(&mp_type_MotionBusy, MP_ERROR_TEXT("Rig is running"));
    }
    if (status == MOCO_ERR_FULL) {
        mp_raise_msg(&mp_type_MotionQueueFull, MP_ERROR_TEXT("No free queue slot"));
    }
    mp_raise_msg(&mp_type_MotionError, MP_ERROR_TEXT("Rejected by the rig"));
}

static void motion_rig_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "Rig(n_channels=%u, n_segs=%u, mem_size=%u)", self->n_channels, self->n_segs, self->mem_size);
}

static mp_obj_t motion_rig_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_n_channels, ARG_n_segs };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_n_channels, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_n_segs,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 4} },
    };
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, all_args + n_args);
    mp_arg_val_t parsed[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, all_args, &kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, parsed);

    mp_int_t n_channels = parsed[ARG_n_channels].u_int;
    mp_int_t n_segs = parsed[ARG_n_segs].u_int;
    if (!(1 <= n_channels && n_channels <= MOCO_MAX_CHANNELS)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("n_channels must be 1 to %d, got %d"), MOCO_MAX_CHANNELS, n_channels);
    }
    if (!(4 <= n_segs)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("n_segs must be at least 4, got %d"), n_segs);
    }

    // Everything above is fallible and raises before anything is allocated,
    // so there's never a half-constructed Rig floating around to clean up.
    const size_t mem_size = MOCO_RIG_SIZE(n_channels, n_segs);
    moco_rig *rig = (moco_rig *)m_malloc(mem_size); // Throws on failure

    moco_status status = moco_rig_init(rig, mem_size, n_channels, n_segs, MOTION_CLOCK_HZ);
    if (status != MOCO_OK) {
        m_free(rig);
        mp_raise_msg_varg(&mp_type_MotionError, MP_ERROR_TEXT("Initialization failed with n_channels=%d, n_segs=%d"), n_channels, n_segs);
    }
    for (mp_int_t i = 0; i < n_channels; i++) {
        moco_channel_set_limits(rig, i, MOTION_DEFAULT_VMAX, MOTION_DEFAULT_AMAX, MOTION_DEFAULT_VMAX);
    }

    motion_rig_obj_t *self = mp_obj_malloc_with_finaliser(motion_rig_obj_t, &motion_rig_type);
    self->mem_size = mem_size;
    self->n_channels = n_channels;
    self->n_segs = n_segs;
    self->rig = rig;
    self->vjump_explicit_mask = 0;
    for (mp_int_t i = 0; i < n_channels; i++) {
        self->last_target[i] = (moco_float)0;
    }

    // Append self to the ISR scan list
    self->next_handle = 0; // becomes the new tail
    intptr_t *link = &motion_active_rigs_head;
    while (motion_rig_ptr(*link)) {
        link = &motion_rig_ptr(*link)->next_handle;
    }
    MOCO_ENTER_CRITICAL();
    *link = ((intptr_t)self) | 1; // set 1 bit in handle so GC ignores it
    MOCO_EXIT_CRITICAL();

    motion_timer_enable();

    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t motion_rig_deinit(mp_obj_t self_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);

    // Removes self from the ISR scan list, if it's in there.
    intptr_t *link = &motion_active_rigs_head;
    while (motion_rig_ptr(*link)) {
        if (motion_rig_ptr(*link) == self) {
            MOCO_ENTER_CRITICAL();
            *link = self->next_handle;
            MOCO_EXIT_CRITICAL();
            self->next_handle = 0;
            break;
        }
        link = &motion_rig_ptr(*link)->next_handle;
    }

    moco_rig *rig = self->rig;
    if (rig) {
        self->rig = NULL;
        moco_rig_stop(rig);
        m_free(rig);
        motion_timer_disable();
    }
    self->mem_size = 0;
    self->n_channels = 0;
    self->n_segs = 0;

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(motion_rig_deinit_obj, motion_rig_deinit);

static void motion_rig_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest) {
    if (dest[0] != MP_OBJ_NULL) {
        // not load attribute
        return;
    }
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (attr == MP_QSTR_n_channels) {
        dest[0] = MP_OBJ_NEW_SMALL_INT(self->n_channels);
    } else if (attr == MP_QSTR_n_segs) {
        dest[0] = MP_OBJ_NEW_SMALL_INT(self->n_segs);
    } else if (attr == MP_QSTR_initialized) {
        dest[0] = mp_obj_new_bool(self->rig != NULL);
    } else if (attr == MP_QSTR_running) {
        dest[0] = mp_obj_new_bool(self->rig != NULL && moco_rig_is_running(self->rig));
    } else {
        // Not one of our special attributes; fall back to locals_dict
        dest[1] = MP_OBJ_SENTINEL;
    }
}

static mp_obj_t motion_rig_go(mp_obj_t self_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    motion_rig_ensure_initialized(self);
    moco_status status = moco_rig_go(self->rig);
    if (status != MOCO_OK) {
        mp_raise_msg(&mp_type_MotionError, MP_ERROR_TEXT("Rig configuration is invalid"));
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(motion_rig_go_obj, motion_rig_go);

static mp_obj_t motion_rig_stop(mp_obj_t self_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->rig) {
        moco_rig_stop(self->rig);
        // stop() truncates the queue back to the live position -- resync our
        // shadow of each channel's queue-end target to match (see last_target).
        moco_rig_position(self->rig, self->last_target);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(motion_rig_stop_obj, motion_rig_stop);

static mp_obj_t motion_rig_stepper(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {
        ARG_channel, ARG_step_pin, ARG_dir_pin, ARG_rotation_distance,
        ARG_microsteps, ARG_steps_per_rev, ARG_path_scale,
        ARG_pulse_us, ARG_low_min_us, ARG_dir_setup_us, ARG_dir_hold_us,
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_channel,           MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_step_pin,          MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_dir_pin,           MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_rotation_distance, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_microsteps,        MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 1} },
        { MP_QSTR_steps_per_rev,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 200} },
        { MP_QSTR_path_scale,        MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_pulse_us,          MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_low_min_us,        MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_dir_setup_us,      MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_dir_hold_us,       MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    motion_rig_ensure_initialized(self);
    mp_int_t channel = args[ARG_channel].u_int;
    motion_channel_check(self, channel);

    // set_timing() is HALTED-only, so this raises Busy before anything below
    // touches the channel's live moco_channel_data -- moco_channel_get_data()
    // itself has no such guard (moco_design.md §5.1), and a write there while
    // RUNNING could race the ISR mid-struct.
    moco_float pulse_us = motion_get_float_or(args[ARG_pulse_us].u_obj, MOTION_DEFAULT_TIMING_US);
    moco_float low_min_us = motion_get_float_or(args[ARG_low_min_us].u_obj, MOTION_DEFAULT_TIMING_US);
    moco_float dir_setup_us = motion_get_float_or(args[ARG_dir_setup_us].u_obj, MOTION_DEFAULT_TIMING_US);
    moco_float dir_hold_us = motion_get_float_or(args[ARG_dir_hold_us].u_obj, MOTION_DEFAULT_TIMING_US);
    motion_check_status(moco_channel_set_timing(self->rig, channel, pulse_us, low_min_us, dir_setup_us, dir_hold_us));

    const machine_pin_obj_t *step_pin = pin_find(args[ARG_step_pin].u_obj);
    const machine_pin_obj_t *dir_pin = pin_find(args[ARG_dir_pin].u_obj);
    motion_ensure_output(step_pin);
    motion_ensure_output(dir_pin);
    *moco_channel_get_data(self->rig, channel) = (moco_channel_data){
        .step_gpio = step_pin->gpio, .step_mask = step_pin->pin_mask,
        .dir_gpio = dir_pin->gpio, .dir_mask = dir_pin->pin_mask,
    };

    if (args[ARG_rotation_distance].u_obj != mp_const_none) {
        mp_int_t microsteps = args[ARG_microsteps].u_int;
        mp_int_t steps_per_rev = args[ARG_steps_per_rev].u_int;
        if (microsteps <= 0 || steps_per_rev <= 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("microsteps and steps_per_rev must be positive"));
        }
        moco_float rotation_distance = mp_obj_get_float_to_f(args[ARG_rotation_distance].u_obj);
        moco_float unit_scale = rotation_distance / (moco_float)(steps_per_rev * microsteps);
        moco_float path_scale = motion_get_float_or(args[ARG_path_scale].u_obj, (moco_float)1);
        motion_check_status(moco_channel_set_scale(self->rig, channel, unit_scale, path_scale));
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_stepper_obj, 1, motion_rig_stepper);

static mp_obj_t motion_rig_scale(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_channel, ARG_unit_scale, ARG_path_scale };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_channel,    MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_unit_scale, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_path_scale, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    motion_rig_ensure_initialized(self);
    mp_int_t channel = args[ARG_channel].u_int;
    motion_channel_check(self, channel);

    moco_float unit_scale, path_scale;
    moco_channel_get_scale(self->rig, channel, &unit_scale, &path_scale);

    bool changed = false;
    if (args[ARG_unit_scale].u_obj != mp_const_none) {
        unit_scale = mp_obj_get_float_to_f(args[ARG_unit_scale].u_obj);
        changed = true;
    }
    if (args[ARG_path_scale].u_obj != mp_const_none) {
        path_scale = mp_obj_get_float_to_f(args[ARG_path_scale].u_obj);
        changed = true;
    }
    if (changed) {
        motion_check_status(moco_channel_set_scale(self->rig, channel, unit_scale, path_scale));
    }

    mp_obj_t items[] = { mp_obj_new_float_from_f(unit_scale), mp_obj_new_float_from_f(path_scale) };
    return mp_obj_new_tuple(MP_ARRAY_SIZE(items), items);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_scale_obj, 1, motion_rig_scale);

static mp_obj_t motion_rig_rates(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_channel, ARG_vmax, ARG_amax, ARG_vjump };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_channel, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_vmax,    MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_amax,    MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_vjump,   MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    motion_rig_ensure_initialized(self);
    mp_int_t channel = args[ARG_channel].u_int;
    motion_channel_check(self, channel);

    moco_float vmax, amax, vjump;
    moco_channel_get_limits(self->rig, channel, &vmax, &amax, &vjump);

    bool changed = false;
    if (args[ARG_vmax].u_obj != mp_const_none) {
        vmax = mp_obj_get_float_to_f(args[ARG_vmax].u_obj);
        changed = true;
    }
    if (args[ARG_amax].u_obj != mp_const_none) {
        amax = mp_obj_get_float_to_f(args[ARG_amax].u_obj);
        changed = true;
    }
    uint32_t channel_bit = (uint32_t)1 << channel;
    if (args[ARG_vjump].u_obj != mp_const_none) {
        vjump = mp_obj_get_float_to_f(args[ARG_vjump].u_obj);
        self->vjump_explicit_mask |= channel_bit;
        changed = true;
    } else if (changed && !(self->vjump_explicit_mask & channel_bit)) {
        // vjump never explicitly set -- track vmax, the most permissive value.
        vjump = vmax;
    }
    if (changed) {
        motion_check_status(moco_channel_set_limits(self->rig, channel, vmax, amax, vjump));
    }

    mp_obj_t items[] = { mp_obj_new_float_from_f(vmax), mp_obj_new_float_from_f(amax), mp_obj_new_float_from_f(vjump) };
    return mp_obj_new_tuple(MP_ARRAY_SIZE(items), items);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_rates_obj, 1, motion_rig_rates);

// Parses target (a list/tuple of per-channel values, entries may be None or
// omitted) into a full n_channels-length array, filling forward from
// self->last_target for every omitted or None entry.
static void motion_parse_target(motion_rig_obj_t *self, mp_obj_t target_obj, moco_float *target) {
    size_t len;
    mp_obj_t *items;
    mp_obj_get_array(target_obj, &len, &items);
    if (len > (size_t)self->n_channels) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("target must have at most %d entries, got %d"), (int)self->n_channels, (int)len);
    }
    for (mp_int_t i = 0; i < self->n_channels; i++) {
        if ((size_t)i < len && items[i] != mp_const_none) {
            target[i] = mp_obj_get_float_to_f(items[i]);
        } else {
            target[i] = self->last_target[i];
        }
    }
}

static void motion_fill_segresult(motion_rig_obj_t *self, mp_obj_t result_obj, const moco_seg_result *c_result) {
    if (!mp_obj_is_type(result_obj, &motion_segresult_type)) {
        mp_raise_TypeError(MP_ERROR_TEXT("result must be a SegResult"));
    }
    motion_segresult_obj_t *result = MP_OBJ_TO_PTR(result_obj);
    result->duration_s = mp_obj_new_float_from_f(c_result->duration_s);
    result->v_end = mp_obj_new_float_from_f(c_result->v_end);
    result->clamped_v = motion_clamped_list(self->n_channels, c_result->clamped_v);
    result->clamped_a = motion_clamped_list(self->n_channels, c_result->clamped_a);
}

// Common result=/return-value contract for move()/segment()/dwell() (§3.4):
// result=None returns the plain duration; a SegResult is filled in place
// and returned instead.
static mp_obj_t motion_queue_return(motion_rig_obj_t *self, mp_obj_t result_obj, const moco_seg_result *c_result) {
    if (result_obj == mp_const_none) {
        return mp_obj_new_float_from_f(c_result->duration_s);
    }
    motion_fill_segresult(self, result_obj, c_result);
    return result_obj;
}

static mp_obj_t motion_rig_move(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_target, ARG_v_cruise, ARG_result, ARG_go };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_target,   MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_v_cruise, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_result,   MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_go,       MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    motion_rig_ensure_initialized(self);
    moco_float target[MOCO_MAX_CHANNELS];
    motion_parse_target(self, args[ARG_target].u_obj, target);
    moco_float v_cruise = motion_get_float_or(args[ARG_v_cruise].u_obj, (moco_float)0);
    moco_queue_flags flags = args[ARG_go].u_bool ? 0 : MOCO_QUEUE_WAIT;

    moco_seg_result c_result;
    motion_check_status(moco_rig_queue_move(self->rig, target, v_cruise, flags, &c_result));
    for (mp_int_t i = 0; i < self->n_channels; i++) {
        self->last_target[i] = target[i];
    }

    return motion_queue_return(self, args[ARG_result].u_obj, &c_result);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_move_obj, 1, motion_rig_move);

static mp_obj_t motion_rig_segment(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_target, ARG_v_end, ARG_result, ARG_go };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_target, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_v_end,  MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_result, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_go,     MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    motion_rig_ensure_initialized(self);
    moco_float target[MOCO_MAX_CHANNELS];
    motion_parse_target(self, args[ARG_target].u_obj, target);
    moco_float v_end = mp_obj_get_float_to_f(args[ARG_v_end].u_obj);
    moco_queue_flags flags = args[ARG_go].u_bool ? 0 : MOCO_QUEUE_WAIT;

    moco_seg_result c_result;
    motion_check_status(moco_rig_queue_seg(self->rig, target, v_end, flags, &c_result));
    for (mp_int_t i = 0; i < self->n_channels; i++) {
        self->last_target[i] = target[i];
    }

    return motion_queue_return(self, args[ARG_result].u_obj, &c_result);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_segment_obj, 1, motion_rig_segment);

static mp_obj_t motion_rig_dwell(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_duration_s, ARG_result, ARG_go };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_duration_s, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_result,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_go,         MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    motion_rig_ensure_initialized(self);
    moco_float duration_s = mp_obj_get_float_to_f(args[ARG_duration_s].u_obj);
    moco_queue_flags flags = args[ARG_go].u_bool ? 0 : MOCO_QUEUE_WAIT;

    moco_seg_result c_result;
    motion_check_status(moco_rig_queue_dwell(self->rig, duration_s, flags, &c_result));

    return motion_queue_return(self, args[ARG_result].u_obj, &c_result);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_dwell_obj, 1, motion_rig_dwell);

static mp_obj_t motion_rig_get_position(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_result };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_result, MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    motion_rig_ensure_initialized(self);
    moco_float pos[MOCO_MAX_CHANNELS];
    moco_rig_position(self->rig, pos);

    if (args[ARG_result].u_obj == mp_const_none) {
        mp_obj_t items[MOCO_MAX_CHANNELS];
        for (mp_int_t i = 0; i < self->n_channels; i++) {
            items[i] = mp_obj_new_float_from_f(pos[i]);
        }
        return mp_obj_new_list(self->n_channels, items);
    }

    if (!mp_obj_is_type(args[ARG_result].u_obj, &mp_type_list)) {
        mp_raise_TypeError(MP_ERROR_TEXT("result must be a list"));
    }
    size_t len;
    mp_obj_t *items;
    mp_obj_list_get(args[ARG_result].u_obj, &len, &items);
    if (len < (size_t)self->n_channels) {
        mp_raise_ValueError(MP_ERROR_TEXT("result list must have at least n_channels entries"));
    }
    for (mp_int_t i = 0; i < self->n_channels; i++) {
        items[i] = mp_obj_new_float_from_f(pos[i]);
    }
    return args[ARG_result].u_obj;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_get_position_obj, 1, motion_rig_get_position);

static mp_obj_t motion_rig_get_velocity(mp_obj_t self_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    motion_rig_ensure_initialized(self);
    return mp_obj_new_float_from_f(moco_rig_velocity(self->rig));
}
static MP_DEFINE_CONST_FUN_OBJ_1(motion_rig_get_velocity_obj, motion_rig_get_velocity);

static mp_obj_t motion_rig_get_queue_free(mp_obj_t self_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    motion_rig_ensure_initialized(self);
    return mp_obj_new_int(moco_rig_queue_free(self->rig));
}
static MP_DEFINE_CONST_FUN_OBJ_1(motion_rig_get_queue_free_obj, motion_rig_get_queue_free);

static const mp_rom_map_elem_t motion_rig_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&motion_rig_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&motion_rig_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_go), MP_ROM_PTR(&motion_rig_go_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&motion_rig_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_stepper), MP_ROM_PTR(&motion_rig_stepper_obj) },
    { MP_ROM_QSTR(MP_QSTR_scale), MP_ROM_PTR(&motion_rig_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_rates), MP_ROM_PTR(&motion_rig_rates_obj) },
    { MP_ROM_QSTR(MP_QSTR_move), MP_ROM_PTR(&motion_rig_move_obj) },
    { MP_ROM_QSTR(MP_QSTR_segment), MP_ROM_PTR(&motion_rig_segment_obj) },
    { MP_ROM_QSTR(MP_QSTR_dwell), MP_ROM_PTR(&motion_rig_dwell_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_position), MP_ROM_PTR(&motion_rig_get_position_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_velocity), MP_ROM_PTR(&motion_rig_get_velocity_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_queue_free), MP_ROM_PTR(&motion_rig_get_queue_free_obj) },
 };
static MP_DEFINE_CONST_DICT(motion_rig_locals_dict, motion_rig_locals_dict_table);

static MP_DEFINE_CONST_OBJ_TYPE(
    motion_rig_type,
    MP_QSTR_Rig,
    MP_TYPE_FLAG_NONE,
    make_new, motion_rig_make_new,
    print, motion_rig_print,
    attr, &motion_rig_attr,
    locals_dict, &motion_rig_locals_dict
    );

/******************************************************************************/
// The `motion` module.

static const mp_rom_map_elem_t motion_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_motion) },

    { MP_ROM_QSTR(MP_QSTR_Rig), MP_ROM_PTR(&motion_rig_type) },
    { MP_ROM_QSTR(MP_QSTR_SegResult), MP_ROM_PTR(&motion_segresult_type) },

    { MP_ROM_QSTR(MP_QSTR_Error), MP_ROM_PTR(&mp_type_MotionError) },
    { MP_ROM_QSTR(MP_QSTR_QueueFull), MP_ROM_PTR(&mp_type_MotionQueueFull) },
    { MP_ROM_QSTR(MP_QSTR_Busy), MP_ROM_PTR(&mp_type_MotionBusy) },
};
static MP_DEFINE_CONST_DICT(motion_module_globals, motion_module_globals_table);

const mp_obj_module_t motion_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&motion_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_motion, motion_module);

