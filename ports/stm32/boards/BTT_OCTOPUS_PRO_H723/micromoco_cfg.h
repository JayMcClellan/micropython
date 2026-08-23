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

static inline void moco_on_pos_change(struct moco_rig *rig, moco_channel_data *d, int steps) {
    (void)rig; (void)steps;
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

/* Move/segment lifecycle (micromoco.h §4.1), for tracing on a logic
 * analyzer: PE9 brackets a whole move (high from moco_on_move_begin() to
 * moco_on_move_end()), PE10 brackets one segment/phase within it likewise.
 * For a single-phase move the two edges land together, which is expected.
 * Direct BSRR set/reset, same idiom as moco_make_pin()'s on/off pattern in
 * motion.c -- these two pins are dedicated to this tracing use and are not
 * otherwise driven by this port. Both must be configured as push-pull
 * outputs elsewhere (motion.c's motion_timer_enable(), alongside PE15/PE7's
 * own setup) before any of these fire. */
static inline void moco_on_move_begin(struct moco_rig *rig, int32_t seq) {
    (void)rig; (void)seq;
    GPIOE->BSRR = GPIO_PIN_9;
}
static inline void moco_on_move_end(struct moco_rig *rig, int32_t seq) {
    (void)rig; (void)seq;
    GPIOE->BSRR = (uint32_t)GPIO_PIN_9 << 16;
}
static inline void moco_on_seg_begin(struct moco_rig *rig, int32_t seq, uint8_t seg_idx) {
    (void)rig; (void)seq; (void)seg_idx;
    GPIOE->BSRR = GPIO_PIN_10;
}
static inline void moco_on_seg_end(struct moco_rig *rig, int32_t seq, uint8_t seg_idx) {
    (void)rig; (void)seq; (void)seg_idx;
    GPIOE->BSRR = (uint32_t)GPIO_PIN_10 << 16;
}

#define MOCO_ENTER_CRITICAL() mp_uint_t _moco_irq_state = disable_irq()
#define MOCO_EXIT_CRITICAL()  enable_irq(_moco_irq_state)

#endif // MICROMOCO_CFG_H
