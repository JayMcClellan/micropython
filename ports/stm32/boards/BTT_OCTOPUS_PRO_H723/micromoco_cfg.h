/* micromoco_cfg.h - stub configuration for the BTT_OCTOPUS_PRO_H723 board.
 *
 * All event functions are no-ops for now, just to get micromoco building and
 * linking into the firmware. Real GPIO output and critical-section handling
 * come later.
 */

#ifndef MICROMOCO_CFG_H
#define MICROMOCO_CFG_H

#include <math.h>

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

typedef double moco_mem;

typedef void *moco_channel_data;

static inline void moco_on_dir_change(moco_channel_data *data, int dir) {
    (void)data; (void)dir;
}
static inline void moco_on_pos_change(moco_channel_data *data, int steps) {
    (void)data; (void)steps;
}
static inline void moco_on_pos_done(moco_channel_data *data) {
    (void)data;
}

#define MOCO_ENTER_CRITICAL() ((void)0)
#define MOCO_EXIT_CRITICAL()  ((void)0)

#endif // MICROMOCO_CFG_H
