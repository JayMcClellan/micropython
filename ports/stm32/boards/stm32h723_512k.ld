/*
    GNU linker script for STM32H723 with 512KB flash (e.g. STM32H723ZET6).

    This variant's flash is a single bank of 4x128K sectors (512K total).
    Sector size is fixed in silicon at 128K for this line of H7 (723/725/
    733/735/730 -- same as the 1MB STM32H723ZG in boards/stm32h723.ld,
    which just has more of them); it does not shrink with flash density.
    Boards using this script need target/stm32h7x.cfg (single bank), not
    target/stm32h7x_dual_bank.cfg.

    FLASH_FS (64K) is smaller than a sector and starts mid-sector, so it
    shares its physical 128K erase sector with the last 64K of FLASH/
    FLASH_APP. This is deliberate: giving FLASH_FS its own sector would
    cost 64-128K of code space we'd rather keep for the app. DTCM below
    is sized to the full 128K sector (not to FLASH_FS's 64K) because
    flashbdev.c's cache does a read-erase-modify-write of the whole
    physical sector on every flash-FS write -- it reads the code half
    out first and writes it back unchanged, so normal writes don't
    corrupt the app. A power loss during that erase window would,
    though, since the app's tail is erased along with the FS data.
    That residual risk is accepted rather than paying for
    sector isolation. DTCM must stay >= 128K (FLASH_SECTOR_SIZE_MAX in
    flashbdev.c) -- shrinking it would
    silently truncate the sector restore and corrupt the app on every
    flash-FS write, not just on power loss.
*/

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 448K    /* sectors 0-3, 128K each (sector 3 shared with FLASH_FS) */
    FLASH_APP  (rx) : ORIGIN = 0x08020000, LENGTH = 320K    /* sectors 1-3 (sector 3 shared with FLASH_FS) */
    FLASH_FS (r)    : ORIGIN = 0x08070000, LENGTH = 64K     /* back half of sector 3 */
    DTCM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K    /* FS storage cache; must be >= one flash erase sector (128K), see note above */
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

/* Location of filesystem RAM cache */
_micropy_hw_internal_flash_storage_ram_cache_start = ORIGIN(DTCM);
_micropy_hw_internal_flash_storage_ram_cache_end = ORIGIN(DTCM) + LENGTH(DTCM);

/* Location of filesystem flash storage */
_micropy_hw_internal_flash_storage_start = ORIGIN(FLASH_FS);
_micropy_hw_internal_flash_storage_end = ORIGIN(FLASH_FS) + LENGTH(FLASH_FS);

/* Define output sections */
SECTIONS
{
    .eth_buffers (NOLOAD) : {
        . = ABSOLUTE(0x30000000);
        *eth.o*(.bss.eth_dma)
    } >RAM_SRAM1
}
