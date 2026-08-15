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

#define MOTION_RIG_NUM (8)
#define MOTION_CLOCK_HZ (1000000)

typedef struct _motion_rig_obj_t {
    mp_obj_base_t base;
    mp_int_t n_channels, n_segs;
    size_t mem_size;
    void *mem;        // raw allocation; valid whenever mem_size != 0
    moco_rig *rig;    // == mem once successfully initialized, else NULL
} motion_rig_obj_t;

static motion_rig_obj_t motion_rig_obj[MOTION_RIG_NUM];
MP_REGISTER_ROOT_POINTER(struct _motion_rig_obj_t *motion_rig_obj[MOTION_RIG_NUM]);

static const mp_obj_type_t motion_rig_type;

// Minimal custom exception surface, per MicroPython's "do a lot with a
// little": QueueFull/Busy exist because a caller plausibly catches and
// reacts to them differently (backpressure vs. wait-and-retry); every other
// failure -- including "not initialized" -- is a plain Error with a message,
// not its own type.
MP_DEFINE_EXCEPTION(MotionError, RuntimeError)
MP_DEFINE_EXCEPTION(MotionQueueFull, MotionError)
MP_DEFINE_EXCEPTION(MotionBusy, MotionError)

void motion_init(void) {
    // reset motion.Rig objects
    for (int i = 0; i < MOTION_RIG_NUM; i++) {
        motion_rig_obj[i].base.type = &motion_rig_type;
        motion_rig_obj[i].n_channels = 0;
        motion_rig_obj[i].n_segs = 0;
        motion_rig_obj[i].mem_size = 0;
        motion_rig_obj[i].mem = NULL;
        motion_rig_obj[i].rig = NULL;
    }
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

static mp_obj_t motion_rig_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 1, 1, false);

    // Rig index is 0-based like channel index
    mp_int_t rig_index = mp_obj_get_int(args[0]);
    if (!(0 <= rig_index && rig_index < MOTION_RIG_NUM)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("Rig(%d) doesn't exist"), rig_index);
    }

    motion_rig_obj_t *rig_obj = &motion_rig_obj[rig_index];
    return MP_OBJ_FROM_PTR(rig_obj);
}

static void motion_rig_free(motion_rig_obj_t* self) {
    if (self->rig) {
        moco_rig_stop(self->rig);
    }
    if (self->mem) {
        m_free(self->mem);
    }
    self->mem = NULL;
    self->rig = NULL;
    self->mem_size = 0;
    self->n_channels = 0;
    self->n_segs = 0;
}

static mp_obj_t motion_rig_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    enum { ARG_n_channels, ARG_n_segs };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_n_channels, MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_n_segs,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 4} },
    };
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_arg_val_t parsed[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, parsed);

    if (self->rig) {
        mp_raise_msg(&mp_type_MotionError, MP_ERROR_TEXT("Rig is already initialized; call deinit() first"));
    }

    mp_int_t n_channels = parsed[ARG_n_channels].u_int;
    mp_int_t n_segs = parsed[ARG_n_segs].u_int;
    if (!(1 <= n_channels && n_channels <= MOCO_MAX_CHANNELS)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("n_channels must be 1 to %d, got %d"), MOCO_MAX_CHANNELS, n_channels);
    }
    if (!(4 <= n_segs)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("n_segs must be at least 4, got %d"), n_segs);
    }

    const size_t mem_size = MOCO_RIG_SIZE(n_channels, n_segs);
    self->mem = m_malloc(mem_size);
    if (!self->mem) {
        m_malloc_fail(mem_size);
    }
    self->mem_size = mem_size;

    moco_status status = moco_rig_init(self->mem, mem_size, n_channels, n_segs, MOTION_CLOCK_HZ);
    if (status != MOCO_OK) {
        motion_rig_free(self);
        mp_raise_msg_varg(&mp_type_MotionError, MP_ERROR_TEXT("Initialization failed with n_channels=%d, n_segs=%d"), n_channels, n_segs);
    }
    self->rig = (moco_rig *)self->mem;
    self->n_channels = n_channels;
    self->n_segs = n_segs;

    return MP_OBJ_FROM_PTR(self);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(motion_rig_init_obj, 1, motion_rig_init);

static mp_obj_t motion_rig_deinit(mp_obj_t self_in) {
    motion_rig_obj_t *self = MP_OBJ_TO_PTR(self_in);
    motion_rig_free(self);
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
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&motion_rig_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&motion_rig_deinit_obj) },
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

