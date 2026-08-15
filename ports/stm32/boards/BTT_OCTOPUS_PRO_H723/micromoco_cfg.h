/* micromoco_cfg.h - stub configuration for the BTT_OCTOPUS_PRO_H723 board.
 *
 * All macros are no-ops for now, just to get micromoco building and linking
 * into the firmware. Real pin output and critical-section handling come
 * later.
 */

#ifndef MICROMOCO_CFG_H
#define MICROMOCO_CFG_H

#include <math.h>

typedef float moco_float;
#define MOCO_SQRTF(x) sqrtf(x)

typedef double moco_mem;

typedef void *moco_pin;
#define MOCO_PIN_ON(p)   ((void)(p))
#define MOCO_PIN_OFF(p)  ((void)(p))

#define MOCO_ENTER_CRITICAL() ((void)0)
#define MOCO_EXIT_CRITICAL()  ((void)0)

#endif // MICROMOCO_CFG_H
