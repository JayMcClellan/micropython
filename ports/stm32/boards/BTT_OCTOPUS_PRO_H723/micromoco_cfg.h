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
#define MOCO_SQRTF(x) sqrtf(x)

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
