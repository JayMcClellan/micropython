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

#include "motion.h"
#include "timer.h"
#include "micromoco.h"

// 100kHz for now, deliberately coarser than ideal for real pulse timing --
// easier to reason about while bringing up the timer/ISR plumbing itself.
// A faster rate (2MHz is the likely eventual choice for this board) is a
// later, board-specific tuning decision, not something to solve here.
#define MOTION_CLOCK_HZ (100000)

typedef struct _motion_rig_obj_t {
    mp_obj_base_t base;
    mp_int_t n_channels, n_segs;
    size_t mem_size;
    moco_rig *rig; // == mem once successfully initialized, else NULL
    intptr_t next_handle; // Next node in active rig list
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

    motion_rig_obj_t *self = mp_obj_malloc_with_finaliser(motion_rig_obj_t, &motion_rig_type);
    self->mem_size = mem_size;
    self->n_channels = n_channels;
    self->n_segs = n_segs;
    self->rig = rig;

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
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(motion_rig_stop_obj, motion_rig_stop);

static const mp_rom_map_elem_t motion_rig_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&motion_rig_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&motion_rig_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_go), MP_ROM_PTR(&motion_rig_go_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&motion_rig_stop_obj) },
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

