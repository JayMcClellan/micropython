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
#include "py/binary.h"

#include "motion.h"
#include "timer.h"
#include "pin.h"
#include "micromoco.h"

#define MOTION_CLOCK_HZ (2000000)

// Seeded once per channel at construction, in raw steps (unit_scale is still
// 1.0 at that point), so a freshly built Rig can move without an explicit
// constraints() call. Binding-level convenience only -- moco_rig_init() itself
// still leaves these at zero (moco_design.md §5.1). If a channel later gets
// real units via stepper(unit_scale=...) or scale(), these numbers
// get reinterpreted in the new units and are almost certainly no longer
// sensible; specify vmax/amax in stepper() or call constraints() explicitly
// whenever real units are in effect.
#define MOTION_DEFAULT_VMAX ((moco_float)1000) // steps/sec
#define MOTION_DEFAULT_AMAX ((moco_float)5000) // steps/sec^2

typedef struct _motion_rig_obj_t {
    mp_obj_base_t base;
    mp_int_t n_channels, q_depth;
    // Embedded by value, not a pointer: moco_rig_init() allocates the three
    // variable-sized arrays behind it, and moco_rig_initialized(&self->rig)
    // is the initialized flag. Living inside this GC-scanned object is also
    // what keeps those three blocks traced.
    moco_rig rig;
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

// Cycle-count instrumentation, reported through Rig.get_stats(). DWT->CYCCNT
// is core-private and costs a handful of cycles to read, so it perturbs little
// of what it measures -- and it is exact, unlike reading pulse widths off a
// screen. `isr` covers the whole handler; `update` covers only the
// moco_rig_update() calls within it, summed across every rig and every
// consolidation pass, so isr - update is the shell's own overhead.
// Cleared by Rig.clear_stats() along with the library's own counters.
static uint32_t motion_isr_cycles, motion_isr_cycles_max;
static uint32_t motion_update_cycles, motion_update_cycles_max;

/******************************************************************************/
// Stats: a fresh snapshot returned by Rig.get_stats(). The first block of
// fields mirrors moco_stats's counters directly; the second unpacks its
// MOCO_FLAG_* sticky bitmask into individually named booleans.

enum {
    STATS_updates, STATS_steps, STATS_seg_completed, STATS_slips, STATS_slip_ticks,
    STATS_max_late, STATS_floor_hits, STATS_errors,
    STATS_slip, STATS_floor_hit, STATS_clamp_v_flag, STATS_clamp_a_flag,
    STATS_corner_limited_flag, STATS_error, STATS_underrun,
    STATS_isr_cycles, STATS_isr_cycles_max,
    STATS_update_cycles, STATS_update_cycles_max,
    STATS_NUM_FIELDS,
};

static const uint16_t motion_stats_field_qstrs[STATS_NUM_FIELDS] = {
    MP_QSTR_updates, MP_QSTR_steps, MP_QSTR_seg_completed, MP_QSTR_slips, MP_QSTR_slip_ticks,
    MP_QSTR_max_late, MP_QSTR_floor_hits, MP_QSTR_errors,
    MP_QSTR_slip, MP_QSTR_floor_hit, MP_QSTR_clamp_v_flag, MP_QSTR_clamp_a_flag,
    MP_QSTR_corner_limited_flag, MP_QSTR_error, MP_QSTR_underrun,
    MP_QSTR_isr_cycles, MP_QSTR_isr_cycles_max,
    MP_QSTR_update_cycles, MP_QSTR_update_cycles_max,
};

typedef struct _motion_stats_obj_t {
    mp_obj_base_t base;
    mp_obj_t items[STATS_NUM_FIELDS];
} motion_stats_obj_t;

static const mp_obj_type_t motion_stats_type;

static mp_obj_t motion_stats_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    motion_stats_obj_t *self = mp_obj_malloc(motion_stats_obj_t, type);
    for (size_t i = 0; i < STATS_slip; i++) {
        self->items[i] = MP_OBJ_NEW_SMALL_INT(0);
    }
    for (size_t i = STATS_slip; i < STATS_isr_cycles; i++) {
        self->items[i] = mp_const_false;
    }
    for (size_t i = STATS_isr_cycles; i < STATS_NUM_FIELDS; i++) {
        self->items[i] = MP_OBJ_NEW_SMALL_INT(0);
    }
    return MP_OBJ_FROM_PTR(self);
}

static void motion_stats_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest) {
    motion_stats_obj_t *self = MP_OBJ_TO_PTR(self_in);
    for (size_t i = 0; i < STATS_NUM_FIELDS; i++) {
        if (attr != motion_stats_field_qstrs[i]) {
            continue;
        }
        if (dest[0] == MP_OBJ_NULL) {
            dest[0] = self->items[i]; // load
        } else if (dest[1] != MP_OBJ_NULL) {
            self->items[i] = dest[1]; // store
            dest[0] = MP_OBJ_NULL; // signal success
        }
        return;
    }
    dest[1] = MP_OBJ_SENTINEL; // not ours; fall back to locals_dict
}

// Reuses motion_stats_field_qstrs for the labels instead of a second literal
// per field, so this doesn't duplicate the field names' text in flash.
static void motion_stats_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    motion_stats_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "Stats(");
    for (size_t i = 0; i < STATS_NUM_FIELDS; i++) {
        mp_printf(print, "%s%s=", i ? ", " : "", qstr_str(motion_stats_field_qstrs[i]));
        mp_obj_print_helper(print, self->items[i], PRINT_REPR);
    }
    mp_printf(print, ")");
}

static MP_DEFINE_CONST_OBJ_TYPE(
    motion_stats_type,
    MP_QSTR_Stats,
    MP_TYPE_FLAG_NONE,
    make_new, motion_stats_make_new,
    print, motion_stats_print,
    attr, &motion_stats_attr
    );

// Smoke-test bridge to the redesigned micromoco (see the project's redesign
// plan): several moco_stats fields/flags this used to read no longer exist
// (phases, trapezoid clamps, backstop/underrun, schedule-slip correction all
// dissolved with the old trajectory model -- see the plan's "What this
// dissolves" table). Rather than change motion.c's own Python-facing Stats
// shape, those fields keep reporting -- always 0/False, since there is
// nothing left to report. seg_completed maps onto the closest surviving
// counter, moves_completed (a move is the whole unit of completion now,
// there being no more phases within one). Revisit once Stats itself gets a
// real pass (the redesign plan's Phase 5).
static void motion_stats_fill(motion_stats_obj_t *self, const moco_stats *c_stats) {
    self->items[STATS_updates] = mp_obj_new_int_from_uint(c_stats->updates);
    self->items[STATS_steps] = mp_obj_new_int_from_uint(c_stats->steps);
    self->items[STATS_seg_completed] = mp_obj_new_int_from_uint(c_stats->moves_completed);
    self->items[STATS_slips] = MP_OBJ_NEW_SMALL_INT(0);
    self->items[STATS_slip_ticks] = MP_OBJ_NEW_SMALL_INT(0);
    self->items[STATS_max_late] = mp_obj_new_int_from_uint(c_stats->max_late);
    self->items[STATS_floor_hits] = mp_obj_new_int_from_uint(c_stats->floor_hits);
    self->items[STATS_errors] = mp_obj_new_int_from_uint(c_stats->errors);
    self->items[STATS_slip] = mp_const_false;
    self->items[STATS_floor_hit] = mp_obj_new_bool(c_stats->flags & MOCO_FLAG_FLOOR_HIT);
    self->items[STATS_clamp_v_flag] = mp_const_false;
    self->items[STATS_clamp_a_flag] = mp_const_false;
    self->items[STATS_corner_limited_flag] = mp_const_false;
    self->items[STATS_error] = mp_obj_new_bool(c_stats->flags & MOCO_FLAG_ERROR);
    self->items[STATS_underrun] = mp_const_false;
    // Binding-level, not from moco_stats: see motion_isr_cycles above.
    self->items[STATS_isr_cycles] = mp_obj_new_int_from_uint(motion_isr_cycles);
    self->items[STATS_isr_cycles_max] = mp_obj_new_int_from_uint(motion_isr_cycles_max);
    self->items[STATS_update_cycles] = mp_obj_new_int_from_uint(motion_update_cycles);
    self->items[STATS_update_cycles_max] = mp_obj_new_int_from_uint(motion_update_cycles_max);
}

/******************************************************************************/
// Shared hardware timer. One TIM24 interrupt drives every active Rig's
// moco_rig_update() -- enabled when the first Rig initializes, disabled when
// the last one deinitializes (motion_rig_make_new()/motion_rig_deinit() below).
//
// CNT free-runs continuously at MOTION_CLOCK_HZ and is never reset; `now` for
// moco_rig_update() is always TIM24->CNT directly. The timer wakes on a CC1
// compare match programmed to the minimum of every active Rig's returned
// deadline (moco_design.md §7/§18), not a fixed period -- silent while idle,
// only fires when something is actually due.

static int motion_timer_refcount;

void motion_init(void) {
    motion_active_rigs_head = 0;
    motion_timer_refcount = 0;
}

// Trace pins (MOTION_TRACE_ISR/UPDATE_ON/OFF -- ADVANCE/MOVE/SERVO fire from
// inside micromoco.c itself) are all defined in micromoco_cfg.h, alongside
// the library's own move-lifecycle tracing, so the whole pin assignment
// lives in one place. PE8 brackets the whole interrupt, PE7 just the
// moco_rig_update() calls inside it, so the library's share is visible next
// to the step and dir edges rather than only as a number.

// TIM24 is unused by MicroPython elsewhere on this MCU, so it's free to
// drive directly.
static void motion_timer_service(void) {
    uint32_t isr_start = DWT->CYCCNT;
    uint32_t update_cycles = 0;
    uint32_t min_deadline = 0;
    bool any = false;
    MOTION_TRACE_ISR_ON();

    uint32_t now = TIM24->CNT;
    for (motion_rig_obj_t *self = motion_rig_ptr(motion_active_rigs_head); self; self = motion_rig_ptr(self->next_handle)) {
        if (moco_rig_initialized(&self->rig)) {
            uint32_t update_start = DWT->CYCCNT;
            MOTION_TRACE_UPDATE_ON();
            uint32_t deadline = moco_rig_update(&self->rig, now);
            MOTION_TRACE_UPDATE_OFF();
            update_cycles += DWT->CYCCNT - update_start;
            if (!any || (int32_t)(deadline - min_deadline) < 0) {
                min_deadline = deadline;
                any = true;
            }
        }
    }

    if (any) {
        now = TIM24->CNT;
        // Always leave one complete timer tick for the VM before re-entering
        // this ISR, even when motion already has work due. This batches any
        // overdue transitions into the next update instead of immediately
        // starving the application.
        uint32_t earliest = now + 2u;
        if ((int32_t)(min_deadline - earliest) < 0) {
            min_deadline = earliest;
        }
        TIM24->CCR1 = min_deadline;
    }

    motion_update_cycles = update_cycles;
    if (update_cycles > motion_update_cycles_max) {
        motion_update_cycles_max = update_cycles;
    }

    MOTION_TRACE_ISR_OFF();

    // Last, so the trace-pin write above is inside the measured span the same
    // way it is inside the analyzer's.
    motion_isr_cycles = DWT->CYCCNT - isr_start;
    if (motion_isr_cycles > motion_isr_cycles_max) {
        motion_isr_cycles_max = motion_isr_cycles;
    }
}

void TIM24_IRQHandler(void) {
    TIM24->SR = ~TIM_SR_CC1IF;
    motion_timer_service();
}

// Forces prompt re-evaluation after move()/segment()/dwell() may have
// started motion the currently-armed deadline doesn't know about yet (a
// freshly-idle Rig's own deadline can be ~18 minutes out by default). Never
// calls moco_rig_update() itself -- only the ISR does, matching its "call
// from exactly one context" contract -- this just makes that context run
// again promptly.
static void motion_timer_kick(void) {
    TIM24->EGR = TIM_EGR_CC1G;
}

static void motion_timer_enable(void) {
    if (motion_timer_refcount++ > 0) {
        // Some other Rig already has the timer running.
        return;
    }

    // Trace pins (MOTION_TRACE_* / moco_on_{advance,servo}_* in
    // micromoco_cfg.h) and the cycle counter behind motion_isr_cycles.
    // mp_hal_ticks_cpu_enable() is idempotent, so it costs nothing if the
    // application already started CYCCNT for its own use.
    mp_hal_pin_output(pin_E8);
    mp_hal_pin_low(pin_E8);
    mp_hal_pin_output(pin_E7);
    mp_hal_pin_low(pin_E7);
    mp_hal_pin_output(pin_E9);
    mp_hal_pin_low(pin_E9);
    mp_hal_pin_output(pin_E10);
    mp_hal_pin_low(pin_E10);
    mp_hal_pin_output(pin_E12);
    mp_hal_pin_low(pin_E12);
    mp_hal_pin_output(pin_E13);
    mp_hal_pin_low(pin_E13);
    mp_hal_ticks_cpu_enable();

    __HAL_RCC_TIM24_CLK_ENABLE();

    // CNT free-runs at MOTION_CLOCK_HZ across the full 32-bit range; ARR is
    // never meant to be reached (natural overflow is just wraparound, not
    // something we interrupt on -- DIER only enables the CC1 compare match).
    TIM24->PSC = (timer_get_source_freq(24) / MOTION_CLOCK_HZ) - 1;
    TIM24->ARR = 0xFFFFFFFF;
    TIM24->EGR = TIM_EGR_UG; // load PSC/ARR, reset CNT
    TIM24->SR = 0; // clear flags set by the forced update above
    TIM24->DIER = TIM_DIER_CC1IE;
    TIM24->CR1 = TIM_CR1_CEN;

    NVIC_SetPriority(TIM24_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 2, 0));
    HAL_NVIC_EnableIRQ(TIM24_IRQn);

    motion_timer_kick(); // establish the real first deadline via a service pass
}

static void motion_timer_disable(void) {
    if (motion_timer_refcount == 0 || --motion_timer_refcount > 0) {
        // Unbalanced call, or another Rig still needs the timer running.
        return;
    }

    HAL_NVIC_DisableIRQ(TIM24_IRQn);
    NVIC_ClearPendingIRQ(TIM24_IRQn);
    TIM24->CR1 = 0;
    TIM24->DIER = 0;
    __HAL_RCC_TIM24_CLK_DISABLE();
}

// Most methods need no explicit initialized check: every moco_* call already
// reports MOCO_ERR_CONFIG on a torn-down rig, which motion_check_status()
// turns into a MotionError. This exists for the few whose own binding-level
// work runs first and would otherwise misreport (a channel range of 0..-1) or
// read a buffer moco_rig_get_trajectory() declined to fill.
static void motion_rig_ensure_initialized(motion_rig_obj_t *self) {
    if (!moco_rig_initialized(&self->rig)) {
        mp_raise_msg(&mp_type_MotionError, MP_ERROR_TEXT("Rig is not initialized"));
    }
}

static void motion_channel_check(motion_rig_obj_t *self, mp_int_t channel) {
    motion_rig_ensure_initialized(self);
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
    // DEGENERATE/INFEASIBLE/CONFIG all land here -- the status code (see micromoco.h's
    // MOCO_ERR_* defines) is included since the message alone can't distinguish them.
    mp_raise_msg_varg(&mp_type_MotionError, MP_ERROR_TEXT("Rejected by the rig (status=%d)"), (int)status);
}

static void motion_rig_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "Rig(n_channels=%u, q_depth=%u)", self->n_channels, self->q_depth);
}

static mp_obj_t motion_rig_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_n_channels, ARG_q_depth };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_n_channels, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_q_depth,    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 4} },
    };
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, all_args + n_args);
    mp_arg_val_t parsed[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, all_args, &kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, parsed);

    mp_int_t n_channels = parsed[ARG_n_channels].u_int;
    mp_int_t q_depth = parsed[ARG_q_depth].u_int;
    if (!(1 <= n_channels && n_channels <= MOCO_MAX_CHANNELS)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("n_channels must be 1 to %d, got %d"), MOCO_MAX_CHANNELS, n_channels);
    }
    if (!(4 <= q_depth)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("q_depth must be at least 4, got %d"), q_depth);
    }

    // Everything above is fallible and raises before anything is allocated,
    // so there's never a half-constructed Rig floating around to clean up.
    // The object comes first now that the rig lives inside it; moco_rig_init()
    // frees its own partial allocations on failure, leaving only the object
    // itself for the GC to reclaim.
    motion_rig_obj_t *self = mp_obj_malloc_with_finaliser(motion_rig_obj_t, &motion_rig_type);
    // Everything the finaliser might touch is set before the one fallible
    // call below, since mp_obj_malloc() hands back dirty memory and a raise
    // from here on still leaves this object for the GC to finalise.
    self->next_handle = 0;
    self->rig = (moco_rig){0};
    self->n_channels = 0;
    self->q_depth = 0;

    moco_status status = moco_rig_init(&self->rig, n_channels, q_depth, MOTION_CLOCK_HZ);
    if (status == MOCO_ERR_NO_MEM) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("Out of memory"));
    }
    if (status != MOCO_OK) {
        mp_raise_msg_varg(&mp_type_MotionError, MP_ERROR_TEXT("Initialization failed with n_channels=%d, q_depth=%d"), n_channels, q_depth);
    }

    self->n_channels = n_channels;
    self->q_depth = q_depth;
    for (mp_int_t i = 0; i < n_channels; i++) {
        moco_channel_set_constraints(&self->rig, i, MOTION_DEFAULT_VMAX, MOTION_DEFAULT_AMAX);
    }

    // Append self to the ISR scan list (next_handle already 0: the new tail)
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

    // moco_rig_deinit() stops the rig, frees its three blocks and zeroes it,
    // so moco_rig_initialized() reads false from here on and every moco_*
    // call this object might still make returns a failure code.
    if (moco_rig_initialized(&self->rig)) {
        moco_rig_deinit(&self->rig);
        motion_timer_disable();
    }
    self->n_channels = 0;
    self->q_depth = 0;

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
    } else if (attr == MP_QSTR_q_depth) {
        dest[0] = MP_OBJ_NEW_SMALL_INT(self->q_depth);
    } else if (attr == MP_QSTR_initialized) {
        dest[0] = mp_obj_new_bool(moco_rig_initialized(&self->rig));
    } else {
        // Not one of our special attributes; fall back to locals_dict
        dest[1] = MP_OBJ_SENTINEL;
    }
}

static mp_obj_t motion_rig_stop(mp_obj_t self_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    // No-op on a deinitialized rig, so no guard is needed.
    (void)moco_rig_stop(&self->rig);
    motion_timer_kick();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(motion_rig_stop_obj, motion_rig_stop);

// feed_rate()/pause()/resume() -- see micromoco.h. The get/set-in-one shape
// matches constraints()/scale(): called bare it reports, called with a value it
// sets, and either way it returns the rate now in effect.
//
// Smoke-test bridge: feed_rate is deferred entirely under the redesign (see
// the redesign plan's "Pause, resume, and stop" section) -- there is no
// moco_rig_set_feed_rate()/moco_rig_get_feed_rate() anymore. A requested
// value/ramp is accepted (for API compatibility) but has no effect; this
// always reports the fixed rate of 1.0.
static mp_obj_t motion_rig_feed_rate(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_value, ARG_ramp };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_value, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_ramp,  MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    motion_rig_ensure_initialized(self);
    (void)args[ARG_value].u_obj;
    (void)args[ARG_ramp].u_obj;
    return mp_obj_new_float_from_f((moco_float)1);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_feed_rate_obj, 1, motion_rig_feed_rate);

static mp_obj_t motion_rig_pause(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_ramp };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_ramp, MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Smoke-test bridge: ramp is accepted (unused) for API compatibility --
    // moco_rig_pause() no longer takes one (see motion_rig_stop() above).
    motion_rig_ensure_initialized(self);
    (void)args[ARG_ramp].u_obj;
    motion_check_status(moco_rig_pause(&self->rig));
    motion_timer_kick();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_pause_obj, 1, motion_rig_pause);

static mp_obj_t motion_rig_resume(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_ramp };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_ramp, MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Smoke-test bridge: ramp is accepted (unused) for API compatibility --
    // moco_rig_resume() no longer takes one (see motion_rig_stop() above).
    motion_rig_ensure_initialized(self);
    (void)args[ARG_ramp].u_obj;
    motion_check_status(moco_rig_resume(&self->rig));
    motion_timer_kick();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_resume_obj, 1, motion_rig_resume);

// Resolves a stepper() pin argument, which is either a plain machine.Pin (active-high) or a
// (Pin, active_hi) 2-tuple/list -- a lightweight, no-object-of-its-own equivalent of
// machine.Signal's invert flag, chosen over a separate step_hi=/dir_hi= kwarg pair because
// the application builds these up as lists of (pin, polarity) per axis; passing each entry
// straight through as the pin argument avoids unzipping that list into parallel pin/polarity
// arguments at every call site.
static void motion_parse_pin_arg(mp_obj_t obj, const machine_pin_obj_t **pin_out, bool *active_hi_out) {
    if (mp_obj_is_type(obj, &mp_type_tuple) || mp_obj_is_type(obj, &mp_type_list)) {
        size_t len;
        mp_obj_t *items;
        mp_obj_get_array(obj, &len, &items);
        if (len != 2) {
            mp_raise_ValueError(MP_ERROR_TEXT("pin must be a Pin or a (pin, active_hi) pair"));
        }
        *pin_out = pin_find(items[0]);
        *active_hi_out = mp_obj_is_true(items[1]);
    } else {
        *pin_out = pin_find(obj);
        *active_hi_out = true;
    }
}

// Builds one moco_pin from a resolved machine.Pin + polarity -- both on_addr/off_addr point
// at the same BSRR register on this MCU (a single register does both set and reset, at
// different bit positions), unlike a port where "on" and "off" might be genuinely separate
// registers. active_hi=False (an active-low driver input) swaps which BSRR write is "on" vs
// "off" rather than swapping addresses -- moco_on_pos_change()/moco_on_dir_change() (§4.1)
// always mean "assert"/"deassert", not "drive high"/"drive low", so this is the one place
// that distinction gets resolved into actual register writes.
static moco_pin motion_make_pin(const machine_pin_obj_t *pin, bool active_hi) {
    uint32_t *bsrr = (uint32_t *)&pin->gpio->BSRR;
    uint32_t set_val = pin->pin_mask;
    uint32_t reset_val = (uint32_t)pin->pin_mask << 16;
    if (active_hi) {
        return (moco_pin){ .on_addr = bsrr, .on_val = set_val, .off_addr = bsrr, .off_val = reset_val };
    }
    return (moco_pin){ .on_addr = bsrr, .on_val = reset_val, .off_addr = bsrr, .off_val = set_val };
}

static mp_obj_t motion_rig_stepper(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {
        ARG_channel, ARG_step_pin, ARG_dir_pin, ARG_unit_scale,
        ARG_path_scale, ARG_vmax, ARG_amax,
        ARG_pulse_us, ARG_low_min_us, ARG_dir_setup_us, ARG_dir_hold_us,
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_channel,      MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_step_pin,     MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_dir_pin,      MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_unit_scale,   MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_path_scale,   MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_vmax,         MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_amax,         MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_pulse_us,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_low_min_us,   MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_dir_setup_us, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_dir_hold_us,  MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t channel = args[ARG_channel].u_int;
    motion_channel_check(self, channel);

    // set_timing() is HALTED-only, so this raises Busy before anything below
    // touches the channel's live moco_channel_data -- moco_channel_get_data()
    // itself has no such guard (moco_design.md §5.1), and a write there while
    // RUNNING could race the ISR mid-struct.
    moco_float pulse_us = motion_get_float_or(args[ARG_pulse_us].u_obj, MOCO_DEFAULT_PULSE_US);
    moco_float low_min_us = motion_get_float_or(args[ARG_low_min_us].u_obj, MOCO_DEFAULT_PULSE_US);
    moco_float dir_setup_us = motion_get_float_or(args[ARG_dir_setup_us].u_obj, MOCO_DEFAULT_DIR_US);
    moco_float dir_hold_us = motion_get_float_or(args[ARG_dir_hold_us].u_obj, MOCO_DEFAULT_DIR_US);
    motion_check_status(moco_channel_set_timing(&self->rig, channel, pulse_us, low_min_us, dir_setup_us, dir_hold_us));

    const machine_pin_obj_t *step_pin;
    bool step_hi;
    motion_parse_pin_arg(args[ARG_step_pin].u_obj, &step_pin, &step_hi);
    const machine_pin_obj_t *dir_pin;
    bool dir_hi;
    motion_parse_pin_arg(args[ARG_dir_pin].u_obj, &dir_pin, &dir_hi);
    motion_ensure_output(step_pin);
    motion_ensure_output(dir_pin);
    *moco_channel_get_data(&self->rig, channel) = (moco_channel_data){
        .step = motion_make_pin(step_pin, step_hi),
        .dir  = motion_make_pin(dir_pin, dir_hi),
    };

    if (args[ARG_unit_scale].u_obj != mp_const_none || args[ARG_path_scale].u_obj != mp_const_none) {
        moco_float unit_scale = motion_get_float_or(args[ARG_unit_scale].u_obj, (moco_float)1);
        moco_float path_scale = motion_get_float_or(args[ARG_path_scale].u_obj, (moco_float)1);
        motion_check_status(moco_channel_set_scale(&self->rig, channel, unit_scale, path_scale));
    }

    if (args[ARG_vmax].u_obj != mp_const_none || args[ARG_amax].u_obj != mp_const_none) {
        moco_float vmax = motion_get_float_or(args[ARG_vmax].u_obj, MOTION_DEFAULT_VMAX);
        moco_float amax = motion_get_float_or(args[ARG_amax].u_obj, MOTION_DEFAULT_AMAX);
        motion_check_status(moco_channel_set_constraints(&self->rig, channel, vmax, amax));
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

    mp_int_t channel = args[ARG_channel].u_int;
    motion_channel_check(self, channel);

    moco_float unit_scale, path_scale;
    moco_channel_get_scale(&self->rig, channel, &unit_scale, &path_scale);

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
        motion_check_status(moco_channel_set_scale(&self->rig, channel, unit_scale, path_scale));
    }

    mp_obj_t items[] = { mp_obj_new_float_from_f(unit_scale), mp_obj_new_float_from_f(path_scale) };
    return mp_obj_new_tuple(MP_ARRAY_SIZE(items), items);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_scale_obj, 1, motion_rig_scale);

static mp_obj_t motion_rig_constraints(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_channel, ARG_vmax, ARG_amax };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_channel, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_vmax,    MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_amax,    MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t channel = args[ARG_channel].u_int;
    motion_channel_check(self, channel);

    moco_float vmax, amax;
    moco_channel_get_constraints(&self->rig, channel, &vmax, &amax);

    bool changed = false;
    if (args[ARG_vmax].u_obj != mp_const_none) {
        vmax = mp_obj_get_float_to_f(args[ARG_vmax].u_obj);
        changed = true;
    }
    if (args[ARG_amax].u_obj != mp_const_none) {
        amax = mp_obj_get_float_to_f(args[ARG_amax].u_obj);
        changed = true;
    }
    if (changed) {
        motion_check_status(moco_channel_set_constraints(&self->rig, channel, vmax, amax));
    }

    mp_obj_t items[] = { mp_obj_new_float_from_f(vmax), mp_obj_new_float_from_f(amax) };
    return mp_obj_new_tuple(MP_ARRAY_SIZE(items), items);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_constraints_obj, 1, motion_rig_constraints);

static mp_obj_t motion_rig_set_channel_position(mp_obj_t self_in, mp_obj_t channel_in, mp_obj_t position_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_int_t channel = mp_obj_get_int(channel_in);
    motion_channel_check(self, channel);
    moco_float position = mp_obj_get_float_to_f(position_in);
    motion_check_status(moco_channel_set_position(&self->rig, channel, position));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(motion_rig_set_channel_position_obj, motion_rig_set_channel_position);

// Whole-rig position reset -- unlike set_channel_position() there is no
// scalar-broadcast form, since setting every channel to the same numeric
// value rarely makes sense (their unit_scale usually differs).
static mp_obj_t motion_rig_set_position(mp_obj_t self_in, mp_obj_t position_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    motion_rig_ensure_initialized(self);

    size_t len;
    mp_obj_t *items;
    mp_obj_get_array(position_in, &len, &items);
    if (len != (size_t)self->n_channels) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("position must have exactly %d entries, got %d"), (int)self->n_channels, (int)len);
    }
    moco_float position[MOCO_MAX_CHANNELS];
    for (mp_int_t i = 0; i < self->n_channels; i++) {
        position[i] = mp_obj_get_float_to_f(items[i]);
    }
    motion_check_status(moco_rig_set_position(&self->rig, position));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(motion_rig_set_position_obj, motion_rig_set_position);

// Parses target (a list/tuple of per-channel values, entries may be None or
// omitted) into a full n_channels-length array, using
// last target for every omitted or None entry.
static void motion_parse_target(motion_rig_obj_t *self, mp_obj_t target_obj, moco_float *target) {
    moco_rig_get_destination(&self->rig, target);

    size_t len;
    mp_obj_t *items;
    mp_obj_get_array(target_obj, &len, &items);
    if (len > (size_t)self->n_channels) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("target must have at most %d entries, got %d"), (int)self->n_channels, (int)len);
    }
    for (mp_int_t i = 0; i < self->n_channels; i++) {
        if ((size_t)i < len && items[i] != mp_const_none) {
            target[i] = mp_obj_get_float_to_f(items[i]);
        }
    }
}

// target, duration, speed, amax, replace -- see moco_rig_move()'s own doc
// (micromoco.h). There is currently no synchronous way to learn the actual
// duration/end speed a call achieved -- achieved state is only meaningful
// once a move is actually reached.
static mp_obj_t motion_rig_move(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_target, ARG_duration, ARG_speed, ARG_amax, ARG_replace };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_target,       MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_duration,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_speed,        MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_amax,         MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_replace,      MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    motion_rig_ensure_initialized(self);
    moco_float target[MOCO_MAX_CHANNELS];
    motion_parse_target(self, args[ARG_target].u_obj, target);
    moco_float duration = motion_get_float_or(args[ARG_duration].u_obj, (moco_float)0);
    moco_float speed = motion_get_float_or(args[ARG_speed].u_obj, MOCO_HUGE_VAL);
    moco_float amax = motion_get_float_or(args[ARG_amax].u_obj, MOCO_HUGE_VAL);
    moco_move_flags flags = args[ARG_replace].u_bool ? MOCO_MOVE_REPLACE : 0u;

    motion_check_status(moco_rig_move(&self->rig, target, duration, speed, amax, flags));

    motion_timer_kick();

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_move_obj, 1, motion_rig_move);

static mp_obj_t motion_rig_dwell(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_duration, ARG_replace };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_duration, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_replace,  MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    moco_float duration = mp_obj_get_float_to_f(args[ARG_duration].u_obj);
    bool replace = args[ARG_replace].u_bool;

    motion_check_status(moco_rig_dwell(&self->rig, duration, replace ? MOCO_MOVE_REPLACE : 0u));
    motion_timer_kick();

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_dwell_obj, 1, motion_rig_dwell);

// Validates obj is a writable, float-typed array.array/memoryview ('f' or
// 'd', whichever matches how moco_float was built) of at least n_channels
// entries -- rejects a plain list (which has no buffer protocol at all, so
// mp_get_buffer_raise() does that rejection for us) since only a typed array
// guarantees the fill below is a raw-value write with no per-element object
// allocation.
static void motion_get_float_array(motion_rig_obj_t *self, mp_obj_t obj, mp_buffer_info_t *bufinfo) {
    mp_get_buffer_raise(obj, bufinfo, MP_BUFFER_RW);
    if (bufinfo->typecode != 'f' && bufinfo->typecode != 'd') {
        mp_raise_TypeError(MP_ERROR_TEXT("array must have typecode 'f' or 'd'"));
    }
    size_t typesize = mp_binary_get_size('@', bufinfo->typecode, NULL);
    if (bufinfo->len < (size_t)self->n_channels * typesize) {
        mp_raise_ValueError(MP_ERROR_TEXT("array must have at least n_channels entries"));
    }
}

static mp_obj_t motion_rig_get_trajectory(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_position, ARG_velocity };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_position, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_velocity, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    motion_rig_ensure_initialized(self);
    bool want_position = args[ARG_position].u_obj != mp_const_none;
    bool want_velocity = args[ARG_velocity].u_obj != mp_const_none;

    moco_float pos[MOCO_MAX_CHANNELS], vel[MOCO_MAX_CHANNELS];
    moco_rig_get_trajectory(&self->rig, want_position ? pos : NULL, want_velocity ? vel : NULL);

    mp_buffer_info_t bufinfo;
    if (want_position) {
        motion_get_float_array(self, args[ARG_position].u_obj, &bufinfo);
        for (mp_int_t i = 0; i < self->n_channels; i++) {
            mp_binary_set_val_array(bufinfo.typecode, bufinfo.buf, i, mp_obj_new_float_from_f(pos[i]));
        }
    }
    if (want_velocity) {
        motion_get_float_array(self, args[ARG_velocity].u_obj, &bufinfo);
        for (mp_int_t i = 0; i < self->n_channels; i++) {
            mp_binary_set_val_array(bufinfo.typecode, bufinfo.buf, i, mp_obj_new_float_from_f(vel[i]));
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_get_trajectory_obj, 1, motion_rig_get_trajectory);

static mp_obj_t motion_rig_get_speed(mp_obj_t self_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_float_from_f(moco_rig_speed(&self->rig));
}
static MP_DEFINE_CONST_FUN_OBJ_1(motion_rig_get_speed_obj, motion_rig_get_speed);

static mp_obj_t motion_rig_queue_avail(mp_obj_t self_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(moco_rig_queue_avail(&self->rig));
}
static MP_DEFINE_CONST_FUN_OBJ_1(motion_rig_queue_avail_obj, motion_rig_queue_avail);

static mp_obj_t motion_rig_is_running(mp_obj_t self_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bool(moco_rig_is_running(&self->rig));
}
static MP_DEFINE_CONST_FUN_OBJ_1(motion_rig_is_running_obj, motion_rig_is_running);

static mp_obj_t motion_rig_channels_moving(mp_obj_t self_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int_from_uint(moco_rig_channels_moving(&self->rig));
}
static MP_DEFINE_CONST_FUN_OBJ_1(motion_rig_channels_moving_obj, motion_rig_channels_moving);

static mp_obj_t motion_rig_get_stats(mp_obj_t self_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    motion_stats_obj_t *result = mp_obj_malloc(motion_stats_obj_t, &motion_stats_type);
    motion_stats_fill(result, moco_rig_stats(&self->rig));
    return MP_OBJ_FROM_PTR(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(motion_rig_get_stats_obj, motion_rig_get_stats);

static mp_obj_t motion_rig_clear_stats(mp_obj_t self_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    moco_rig_clear_stats(&self->rig);
    // The cycle counters are rig-independent (one shared ISR), but clearing
    // them here is what makes "clear, run a move, read" a usable measurement.
    motion_isr_cycles = motion_isr_cycles_max = 0;
    motion_update_cycles = motion_update_cycles_max = 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(motion_rig_clear_stats_obj, motion_rig_clear_stats);

// Rig-wide, not per-channel -- unlike scale()/constraints() there is no
// channel argument.
static mp_obj_t motion_rig_corner_tol(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_value };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_value, MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    motion_rig_ensure_initialized(self);
    if (args[ARG_value].u_obj != mp_const_none) {
        moco_float value = mp_obj_get_float_to_f(args[ARG_value].u_obj);
        motion_check_status(moco_rig_set_corner_tol(&self->rig, value));
    }
    return mp_obj_new_float_from_f(moco_rig_get_corner_tol(&self->rig));
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_corner_tol_obj, 1, motion_rig_corner_tol);

static const mp_rom_map_elem_t motion_rig_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&motion_rig_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&motion_rig_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&motion_rig_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_feed_rate), MP_ROM_PTR(&motion_rig_feed_rate_obj) },
    { MP_ROM_QSTR(MP_QSTR_pause), MP_ROM_PTR(&motion_rig_pause_obj) },
    { MP_ROM_QSTR(MP_QSTR_resume), MP_ROM_PTR(&motion_rig_resume_obj) },
    { MP_ROM_QSTR(MP_QSTR_stepper), MP_ROM_PTR(&motion_rig_stepper_obj) },
    { MP_ROM_QSTR(MP_QSTR_scale), MP_ROM_PTR(&motion_rig_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_constraints), MP_ROM_PTR(&motion_rig_constraints_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_position), MP_ROM_PTR(&motion_rig_set_position_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_channel_position), MP_ROM_PTR(&motion_rig_set_channel_position_obj) },
    { MP_ROM_QSTR(MP_QSTR_corner_tol), MP_ROM_PTR(&motion_rig_corner_tol_obj) },
    { MP_ROM_QSTR(MP_QSTR_move), MP_ROM_PTR(&motion_rig_move_obj) },
    { MP_ROM_QSTR(MP_QSTR_dwell), MP_ROM_PTR(&motion_rig_dwell_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_trajectory), MP_ROM_PTR(&motion_rig_get_trajectory_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_speed), MP_ROM_PTR(&motion_rig_get_speed_obj) },
    { MP_ROM_QSTR(MP_QSTR_queue_avail), MP_ROM_PTR(&motion_rig_queue_avail_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_running), MP_ROM_PTR(&motion_rig_is_running_obj) },
    { MP_ROM_QSTR(MP_QSTR_channels_moving), MP_ROM_PTR(&motion_rig_channels_moving_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_stats), MP_ROM_PTR(&motion_rig_get_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_stats), MP_ROM_PTR(&motion_rig_clear_stats_obj) },
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
// clock_bench(): what a timer read actually costs.
//
// The update path's cost is dominated by how often it reads a clock, and the
// two candidates are not remotely alike: TIM24->CNT sits behind the
// AXI->AHB->APB1 bridge and stalls the core for the round trip, while
// DWT->CYCCNT is core-private. This measures both rather than leaving it to
// estimation -- returns (tim_cycles, dwt_cycles), each the cost of ONE read in
// core cycles, scaled by 256 so a sub-cycle difference is still visible.
//
// Both loops are written the same way, with a volatile accumulator so the
// reads cannot be hoisted or folded away, and the loop overhead is common to
// both so it largely cancels in the comparison.
#define MOTION_BENCH_READS (256)

static mp_obj_t motion_clock_bench(void) {
    volatile uint32_t sink = 0;
    uint32_t t0, tim_cycles, dwt_cycles;
    int i;

    mp_hal_ticks_cpu_enable();

    uint32_t irq_state = disable_irq();

    t0 = DWT->CYCCNT;
    for (i = 0; i < MOTION_BENCH_READS; i++) {
        sink += TIM24->CNT;
    }
    tim_cycles = DWT->CYCCNT - t0;

    t0 = DWT->CYCCNT;
    for (i = 0; i < MOTION_BENCH_READS; i++) {
        sink += DWT->CYCCNT;
    }
    dwt_cycles = DWT->CYCCNT - t0;

    enable_irq(irq_state);
    (void)sink;

    mp_obj_t items[] = {
        mp_obj_new_int_from_uint(tim_cycles),
        mp_obj_new_int_from_uint(dwt_cycles),
    };
    return mp_obj_new_tuple(MP_ARRAY_SIZE(items), items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(motion_clock_bench_obj, motion_clock_bench);

/******************************************************************************/
// The `motion` module.

static const mp_rom_map_elem_t motion_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_motion) },

    { MP_ROM_QSTR(MP_QSTR_clock_bench), MP_ROM_PTR(&motion_clock_bench_obj) },

    { MP_ROM_QSTR(MP_QSTR_Rig), MP_ROM_PTR(&motion_rig_type) },
    { MP_ROM_QSTR(MP_QSTR_Stats), MP_ROM_PTR(&motion_stats_type) },

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

