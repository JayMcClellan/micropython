# MCU settings
MCU_SERIES = f4
CMSIS_MCU = STM32F446xx
AF_FILE = boards/stm32f446_af.csv

# Same 512K-flash/128K-RAM memory layout as NUCLEO_F446RE (same MCU); no
# board-specific linker script needed.
LD_FILES = boards/stm32f411.ld boards/common_ifs.ld
TEXT0_ADDR = 0x08000000
TEXT1_ADDR = 0x08020000

# MicroPython settings
# Plain FAT for the internal filesystem (the port default). A fresh board
# gets FAT-formatted internal storage regardless of MICROPY_VFS_LFS2 --
# factory_reset_create_filesystem() only ever creates a FAT filesystem --
# so enabling LFS2 here bought nothing but ~17KB of unused code, on a
# board with much less flash headroom than the H723 Pro board to spend.
MICROPY_HW_ENABLE_ISR_UART_FLASH_FUNCS_IN_RAM = 1

FROZEN_MANIFEST ?= $(BOARD_DIR)/manifest.py

# Use the DAP-direct OpenOCD interface -- ST-Link V3 probes don't support
# the older HLA transport that the shared boards/openocd_stm32f4.cfg uses.
OPENOCD_CONFIG = boards/BTT_OCTOPUS_F446/openocd_stm32f4_dap.cfg
