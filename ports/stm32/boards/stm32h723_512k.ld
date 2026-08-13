/*
    GNU linker script for STM32H723 with 512KB flash (e.g. STM32H723ZET6).

    This variant's flash is a single bank of 4x128K sectors (512K total).
    Sector size is fixed in silicon at 128K for this line of H7 (723/725/
    733/735/730 -- same as the 1MB STM32H723ZG in boards/stm32h723.ld,
    which just has more of them); it does not shrink with flash density.
    Boards using this script need target/stm32h7x.cfg (single bank), not
    target/stm32h7x_dual_bank.cfg.

    Because of the large 128K sector size and only 512K of total flash,
    boards using this script run with MICROPY_HW_ENABLE_INTERNAL_FLASH_STORAGE
    disabled so there's no FLASH_FS partition. Boards may use qspi and/or
    SD Card storage instead.
*/

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K    /* sectors 0-3, 128K each */
    FLASH_APP  (rx) : ORIGIN = 0x08020000, LENGTH = 384K    /* sectors 1-3 */
    DTCM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K    /* unused */
    RAM (xrw)       : ORIGIN = 0x24000000, LENGTH = 320K    /* AXI SRAM (128k) + ITCM/AXI SRAM (192k) */
    RAM_SRAM1 (xrw) : ORIGIN = 0x30000000, LENGTH = 16K     /* SRAM1 */
    RAM_SRAM2 (xrw) : ORIGIN = 0x30004000, LENGTH = 16K     /* SRAM2 */
    RAM_SRAM4 (xrw) : ORIGIN = 0x38000000, LENGTH = 16K     /* SRAM4 */
}

/* produce a link error if there is not this amount of RAM for these sections */
_minimum_stack_size = 2K;
_minimum_heap_size = 16K;

/* Define the stack.  The stack is full descending so begins just above last byte
   of RAM.  Note that EABI requires the stack to be 8-byte aligned for a call. */
_estack = ORIGIN(RAM) + LENGTH(RAM) - _estack_reserve;
_sstack = _estack - 16K; /* tunable */

/* RAM extents for the garbage collector */
_ram_start = ORIGIN(RAM);
_ram_end = ORIGIN(RAM) + LENGTH(RAM);
_heap_start = _ebss; /* heap starts just after statically allocated memory */
_heap_end = _sstack;

/* Define output sections */
SECTIONS
{
    .eth_buffers (NOLOAD) : {
        . = ABSOLUTE(0x30000000);
        *eth.o*(.bss.eth_dma)
    } >RAM_SRAM1
}
