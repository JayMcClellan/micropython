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

typedef float moco_mem;

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

static inline void moco_on_pos_change(moco_channel_data *d, int steps) {
    (void)steps;
    *d->step.on_addr = d->step.on_val;
}
static inline void moco_on_pos_done(moco_channel_data *d) {
    *d->step.off_addr = d->step.off_val;
}
static inline void moco_on_dir_change(moco_channel_data *d, int dir) {
    if (dir > 0)
        *d->dir.on_addr = d->dir.on_val;
    else
        *d->dir.off_addr = d->dir.off_val;
}

#define MOCO_ENTER_CRITICAL() mp_uint_t _moco_irq_state = disable_irq()
#define MOCO_EXIT_CRITICAL()  enable_irq(_moco_irq_state)

#endif // MICROMOCO_CFG_H
