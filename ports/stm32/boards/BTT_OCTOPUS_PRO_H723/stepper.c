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

 #if !BUILDING_MBOOT

#include "py/runtime.h"
#include "micromoco.h"

#define MY_MAX_AXES (8)
#define MY_QUEUE_DEPTH (4)
#define MY_CLOCK_HZ (1000000)

typedef struct _stepper_group_obj_t {
    mp_obj_base_t base;
    mp_int_t n_axes;
    moco_mem mem[MOCO_GROUP_SIZE(MY_MAX_AXES, MY_QUEUE_DEPTH) / sizeof(moco_mem)];
    moco_group* moco;
} stepper_group_obj_t;

static const mp_obj_type_t stepper_group_type;

static void stepper_group_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    stepper_group_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "StepperGroup(n_axes=%u)", self->n_axes);
}

static mp_obj_t stepper_group_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_n_axes };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_n_axes,  MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 4} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    stepper_group_obj_t *self = mp_obj_malloc(stepper_group_obj_t, &stepper_group_type);

    mp_int_t n_axes = args[ARG_n_axes].u_int;
    if (!(0 < n_axes && n_axes <= MY_MAX_AXES)) {
        mp_raise_ValueError(NULL);
    }

    moco_group* moco = NULL;
    moco_status mstatus = moco_group_init(&moco, self->mem, sizeof(self->mem), n_axes, MY_QUEUE_DEPTH, MY_CLOCK_HZ);
    if (mstatus != MOCO_OK) {
        mp_raise_ValueError(NULL);
    }
    self->n_axes = n_axes;
    self->moco = moco;

    return MP_OBJ_FROM_PTR(self);
}

static void stepper_group_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest) {
    if (dest[0] != MP_OBJ_NULL) {
        // not load attribute
        return;
    }
    stepper_group_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (attr == MP_QSTR_n_axes) {
        dest[0] = mp_obj_new_int(self->n_axes);
    } else {
        // Not one of our special attributes; fall back to locals_dict
        dest[1] = MP_OBJ_SENTINEL;
    }
}

static mp_obj_t stepper_group_go(mp_obj_t self_in) {
    //stepper_group_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(stepper_group_go_obj, stepper_group_go);

static const mp_rom_map_elem_t stepper_group_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_go), MP_ROM_PTR(&stepper_group_go_obj) },
 };
static MP_DEFINE_CONST_DICT(stepper_group_locals_dict, stepper_group_locals_dict_table);

static MP_DEFINE_CONST_OBJ_TYPE(
    stepper_group_type,
    MP_QSTR_StepperGroup,
    MP_TYPE_FLAG_NONE,
    make_new, stepper_group_make_new,
    print, stepper_group_print,
    attr, &stepper_group_attr,
    locals_dict, &stepper_group_locals_dict
    );

/******************************************************************************/
// The `stepper` module.

static const mp_rom_map_elem_t stepper_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_stepper) },

    { MP_ROM_QSTR(MP_QSTR_StepperGroup), MP_ROM_PTR(&stepper_group_type) },
};
static MP_DEFINE_CONST_DICT(stepper_module_globals, stepper_module_globals_table);

const mp_obj_module_t stepper_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&stepper_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_stepper, stepper_module);


#endif /*!BUILDING_MBOOT*/
 