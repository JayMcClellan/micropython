/* micromoco_cfg.h - configuration for the BTT_OCTOPUS_PRO_H723 board.
 *
 * Channel events are direct GPIO register writes. Critical sections use
 * real PRIMASK save/restore.
 */

#ifndef MICROMOCO_CFG_H
#define MICROMOCO_CFG_H

#include <math.h>

#include "py/mphal.h" // pulls in mp_uint_t, CMSIS intrinsics, and irq.h

typedef float moco_float;

// The port's own libm is built for MICROPY_FLOAT_IMPL (double on this
// board), so it doesn't provide a real sqrtf() symbol -- and __builtin_sqrtf
// isn't a fix by itself: without -fno-math-errno (not set port-wide), GCC
// still lowers it to a real sqrtf() call so errno stays correct on a
// domain error, which is exactly the missing symbol above. Emitting
// VSQRT.F32 directly sidesteps both: no libm dependency, no errno
// semantics to reason about (matching moco_design.md's own build notes
// that the library never inspects errno), and, unlike the builtin, this
// doesn't depend on the optimizer choosing to fold anything -- it emits
// the same instruction at any optimization level, including -O0.
static inline float moco_hw_sqrtf(float x) {
    float result;
    __asm__ ("vsqrt.f32 %0, %1" : "=t" (result) : "t" (x));
    return result;
}
#define MOCO_SQRTF(x) moco_hw_sqrtf(x)

// The free-running counter motion.c also samples for moco_rig_update()'s `now`
// (motion_timer_service()). Read directly rather than passed in, for the two
// places a real-time obligation to the driver is being met: the pulse-width
// wait in a hard stop, and the setup/low-time floor before a rising edge. A
// floor that expired while the update was computing is then honored at once
// instead of costing another wake. Trajectory timing still uses `now`, so a
// single ISR can hold every rig it services to one coherent snapshot.
#define MOCO_NOW() (TIM24->CNT)

// Allocation for moco_rig_init()'s three blocks. m_malloc_maybe(), not
// m_malloc(): the latter raises MemoryError via nlr_jump on failure, which
// would unwind straight past moco_rig_init()'s own cleanup and strand any
// block it had already taken. Returning NULL lets it unwind properly and
// report MOCO_ERR_NO_MEM, which motion.c turns back into a MemoryError.
// The blocks live on the GC heap and are only ever referenced from the
// moco_rig embedded in motion_rig_obj_t, itself a GC-scanned object, so the
// collector traces them for as long as the Rig is alive.
static inline void *moco_alloc(size_t num_bytes) {
    return m_malloc_maybe(num_bytes);
}
static inline void moco_free(void *ptr) {
    m_free(ptr);
}

typedef struct {
    uint32_t *on_addr;
    uint32_t on_val;
    uint32_t *off_addr;
    uint32_t off_val;
} moco_pin;

typedef struct {
   moco_pin step;
   moco_pin dir;
} moco_channel_data;

/* Forward declaration at file scope, not left for each function below to
 * introduce implicitly via its own parameter list: a struct tag named only
 * inside a prototype's parameter list has prototype scope (C99 6.2.1p4),
 * not file scope, so each occurrence would otherwise name a different,
 * mutually incompatible incomplete type from micromoco.h's own later
 * `typedef struct moco_rig moco_rig;` -- an -Wincompatible-pointer-types
 * error at every moco_on_*() call site in micromoco.c. */
struct moco_rig;

// `position` is this channel's real-world position as of the pulse. Unused
// here -- the parameter costs nothing at -Os, being dead once inlined -- but
// it is what a config that wants to track position should read, rather than
// counting step pulses itself.
static inline void moco_on_pos_change(struct moco_rig *rig, moco_channel_data *d, moco_float position) {
    (void)rig; (void)position;
    *d->step.on_addr = d->step.on_val;
}
static inline void moco_on_pos_done(struct moco_rig *rig, moco_channel_data *d) {
    (void)rig;
    *d->step.off_addr = d->step.off_val;
}
static inline void moco_on_dir_change(struct moco_rig *rig, moco_channel_data *d, int dir) {
    (void)rig;
    if (dir > 0)
        *d->dir.on_addr = d->dir.on_val;
    else
        *d->dir.off_addr = d->dir.off_val;
}

/* ---- trace pins, bring-up instrumentation (kept until no longer needed) --
 *
 * The EXP1 header (LCD/TFT, pins.csv) is otherwise unused on this board and
 * gives 8 free GPIOE lines, all centralized here rather than split between
 * this file and motion.c, so the whole pin assignment is visible in one
 * place: PE8/PE7/PE9/PE10/PE12 outermost-to-innermost, in call order (the
 * timer ISR contains moco_rig_update(), which contains moco__advance_real()
 * -- itself containing a retarget when one happens to fire -- and,
 * separately, moco__servo_cycle()). PE13/PE14/PE15 (EXP1_6/7/8) are free for
 * whatever needs tracing next.
 *
 * Direct BSRR set/reset, same idiom as moco_make_pin()'s on/off pattern in
 * motion.c: one immediate each, no pointer chasing in a measured region.
 * All five pins must be configured as push-pull outputs before any of these
 * fire -- motion.c's motion_timer_enable() does this alongside the timer
 * setup itself. Macros, not inline functions, so they read as raw
 * instrumentation at every call site rather than being mistaken for one of
 * the library's real event callbacks below. */
#define MOTION_TRACE_ISR_ON()      (GPIOE->BSRR = GPIO_PIN_8)
#define MOTION_TRACE_ISR_OFF()     (GPIOE->BSRR = (uint32_t)GPIO_PIN_8 << 16)
#define MOTION_TRACE_UPDATE_ON()   (GPIOE->BSRR = GPIO_PIN_7)
#define MOTION_TRACE_UPDATE_OFF()  (GPIOE->BSRR = (uint32_t)GPIO_PIN_7 << 16)
#define MOTION_TRACE_ADVANCE_ON()  (GPIOE->BSRR = GPIO_PIN_9)
#define MOTION_TRACE_ADVANCE_OFF() (GPIOE->BSRR = (uint32_t)GPIO_PIN_9 << 16)
#define MOTION_TRACE_MOVE_ON()     (GPIOE->BSRR = GPIO_PIN_10)
#define MOTION_TRACE_MOVE_OFF()    (GPIOE->BSRR = (uint32_t)GPIO_PIN_10 << 16)
#define MOTION_TRACE_SERVO_ON()    (GPIOE->BSRR = GPIO_PIN_12)
#define MOTION_TRACE_SERVO_OFF()   (GPIOE->BSRR = (uint32_t)GPIO_PIN_12 << 16)

/* Move lifecycle (micromoco.h §4.1): PE10 brackets a whole move, high from
 * moco_on_move_begin() to moco_on_move_end(). */
static inline void moco_on_move_begin(struct moco_rig *rig, int32_t seq) {
    (void)rig; (void)seq;
    MOTION_TRACE_MOVE_ON();
}
static inline void moco_on_move_end(struct moco_rig *rig, int32_t seq) {
    (void)rig; (void)seq;
    MOTION_TRACE_MOVE_OFF();
}

/* Per-real-tick chase/retarget (moco__advance_real()) and the ~1kHz ceiling
 * recompute (moco__servo_cycle()) -- the two halves of moco_rig_update()'s
 * own work, broken out so their relative cost and timing are visible next
 * to each other and next to MOTION_TRACE_UPDATE's own span. */
static inline void moco_on_advance_begin(struct moco_rig *rig) {
    (void)rig;
    MOTION_TRACE_ADVANCE_ON();
}
static inline void moco_on_advance_end(struct moco_rig *rig) {
    (void)rig;
    MOTION_TRACE_ADVANCE_OFF();
}
static inline void moco_on_servo_begin(struct moco_rig *rig) {
    (void)rig;
    MOTION_TRACE_SERVO_ON();
}
static inline void moco_on_servo_end(struct moco_rig *rig) {
    (void)rig;
    MOTION_TRACE_SERVO_OFF();
}

#define MOCO_ENTER_CRITICAL() mp_uint_t _moco_irq_state = disable_irq()
#define MOCO_EXIT_CRITICAL()  enable_irq(_moco_irq_state)

#endif // MICROMOCO_CFG_H
