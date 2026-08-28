USE_MBOOT ?= 0

# MCU settings
MCU_SERIES = h7
CMSIS_MCU = STM32H723xx
MICROPY_FLOAT_IMPL = double
AF_FILE = boards/stm32h723_af.csv

# This board's MCU is STM32H723ZET6 -- 512KB flash, single bank
ifeq ($(USE_MBOOT),1)
# When using Mboot everything goes after the bootloader
LD_FILES = boards/stm32h723_512k.ld boards/common_bl.ld
TEXT0_ADDR = 0x08020000
else
# When not using Mboot everything goes at the start of flash
LD_FILES = boards/stm32h723_512k.ld boards/common_basic.ld
TEXT0_ADDR = 0x08000000
endif

# MicroPython settings
MICROPY_HW_ENABLE_ISR_UART_FLASH_FUNCS_IN_RAM = 1

FROZEN_MANIFEST ?= $(BOARD_DIR)/manifest.py

# Flash tool configuration
OPENOCD_CONFIG = boards/BTT_OCTOPUS_PRO_H723/openocd_stm32h7.cfg

# micromoco (experimental)
CFLAGS += -I$(BOARD_DIR)/lib/micromoco
SRC_C += $(BOARD_DIR)/lib/micromoco/micromoco_rt.c $(BOARD_DIR)/lib/micromoco/micromoco_api.c

# The motion ISR is the one hot path on this board; -Os costs it real time for
# flash savings that do not matter on one file. Everything else stays -Os.
$(BUILD)/$(BOARD_DIR)/lib/micromoco/micromoco_rt.o: CFLAGS += -O2

# Disassembly of the update path with C source interleaved, for investigating
# where its cycles go: `make BOARD=BTT_OCTOPUS_PRO_H723 micromoco_lst`, then
# read build-BTT_OCTOPUS_PRO_H723/boards/.../micromoco_rt.lst. Reads the DWARF
# info -g already puts in the .o (see CFLAGS above) -- no separate compile.
# At -O2 the compiler reorders and schedules across source lines, so a line's
# instructions can appear split, out of order, or shared with a neighbour;
# read it as "what code exists for this line", not a literal trace.
$(BUILD)/$(BOARD_DIR)/lib/micromoco/micromoco_rt.lst: $(BUILD)/$(BOARD_DIR)/lib/micromoco/micromoco_rt.o
	$(CROSS_COMPILE)objdump -d -S -l $< > $@

.PHONY: micromoco_lst
micromoco_lst: $(BUILD)/$(BOARD_DIR)/lib/micromoco/micromoco_rt.lst
