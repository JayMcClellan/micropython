#define MICROPY_HW_BOARD_NAME               "BTT_OCTOPUS_PRO_H723"
#define MICROPY_HW_MCU_NAME                 "STM32H723ZGT6"

#define MICROPY_HW_ENABLE_RTC               (1)
#define MICROPY_HW_ENABLE_RNG               (0) // RNG needs proper configuration
#define MICROPY_HW_ENABLE_ADC               (1)
#define MICROPY_HW_ENABLE_DAC               (1)
#define MICROPY_HW_ENABLE_USB               (1)
#define MICROPY_HW_ENABLE_SDCARD            (1)
#define MICROPY_HW_HAS_FLASH                (1)

// This board has a 25MHz HSE crystal.
// The following gives a 550MHz CPU speed.
#define MICROPY_HW_CLK_USE_HSE              (1)
#define MICROPY_HW_CLK_PLLM                 (5)
#define MICROPY_HW_CLK_PLLN                 (110)
#define MICROPY_HW_CLK_PLLP                 (1)
#define MICROPY_HW_CLK_PLLQ                 (5)
#define MICROPY_HW_CLK_PLLR                 (2)
#define MICROPY_HW_CLK_PLLVCI               (RCC_PLL1VCIRANGE_2)
#define MICROPY_HW_CLK_PLLVCO               (RCC_PLL1VCOWIDE)
#define MICROPY_HW_CLK_PLLFRAC              (0)

// The USB clock is set using PLL3
#define MICROPY_HW_CLK_PLL3M                (5)
#define MICROPY_HW_CLK_PLL3N                (96)
#define MICROPY_HW_CLK_PLL3P                (10)
#define MICROPY_HW_CLK_PLL3Q                (10)
#define MICROPY_HW_CLK_PLL3R                (2)
#define MICROPY_HW_CLK_PLL3VCI              (RCC_PLL3VCIRANGE_2)
#define MICROPY_HW_CLK_PLL3VCO              (RCC_PLL3VCOWIDE)
#define MICROPY_HW_CLK_PLL3FRAC             (0)

// 4 wait states
#define MICROPY_HW_FLASH_LATENCY            FLASH_LATENCY_4

// No external 32kHz crystal on this board; RTC uses the internal LSI.

// UART config
// UART1: secondary display/UART header (e.g. a TFT touchscreen module)
#define MICROPY_HW_UART1_TX                 (pin_A9)
#define MICROPY_HW_UART1_RX                 (pin_A10)
// UART2: serial console header
#define MICROPY_HW_UART2_TX                 (pin_D5)
#define MICROPY_HW_UART2_RX                 (pin_D6)
// UART3: WiFi module (ESP-12S) header
#define MICROPY_HW_UART3_TX                 (pin_D8)
#define MICROPY_HW_UART3_RX                 (pin_D9)

// REPL runs over the USB-C virtual COM port, not a UART

// I2C bus (also feeds the onboard AT24C32 EEPROM)
#define MICROPY_HW_I2C1_SCL                 (pin_B8)
#define MICROPY_HW_I2C1_SDA                 (pin_B9)

// SPI buses
// SPI1: shared stepper-driver bus, also exposed on the EXP2 header
#define MICROPY_HW_SPI1_SCK                 (pin_A5)
#define MICROPY_HW_SPI1_MISO                (pin_A6)
#define MICROPY_HW_SPI1_MOSI                (pin_A7)
// SPI2: WiFi module (ESP-12S) bus
#define MICROPY_HW_SPI2_NSS                 (pin_B12)
#define MICROPY_HW_SPI2_SCK                 (pin_B13)
#define MICROPY_HW_SPI2_MISO                (pin_C2)
#define MICROPY_HW_SPI2_MOSI                (pin_C3)
// SPI3: J74 header
#define MICROPY_HW_SPI3_NSS                 (pin_A15)
#define MICROPY_HW_SPI3_SCK                 (pin_B3)
#define MICROPY_HW_SPI3_MISO                (pin_B4)
#define MICROPY_HW_SPI3_MOSI                (pin_B5)

// Status LED. Shared with SWDIO (PA13) -- driving it as a GPIO output
// will interfere with an attached SWD debug probe.
#define MICROPY_HW_LED1                     (pin_A13)
#define MICROPY_HW_LED_ON(pin)              (mp_hal_pin_high(pin))
#define MICROPY_HW_LED_OFF(pin)             (mp_hal_pin_low(pin))

// USB config
// The STM32H723 has only one USB controller (OTG_HS; unlike e.g. the
// STM32H743, it has no separate OTG_FS core). In HS_IN_FS mode, the
// port's H723-specific code path drives it out on PA11/PA12 (analog mode,
// not a GPIO alternate function) -- the USB-C connector, per the schematic.
// Two CDC (virtual COM port) interfaces: one is the REPL, the other is
// free for general use.
// PB14/PB15 (pins.csv USB_A_DM/USB_A_DP, the USB-A connector) are unused
// by this config -- reserved in pins.csv for future OTG/host-mode support.
#define MICROPY_HW_USB_HS                   (1)
#define MICROPY_HW_USB_HS_IN_FS             (1)
#define MICROPY_HW_USB_CDC_NUM              (2)

// FDCAN bus
#define MICROPY_HW_CAN1_NAME                "FDCAN1"
#define MICROPY_HW_CAN1_TX                  (pin_D1)
#define MICROPY_HW_CAN1_RX                  (pin_D0)

// SD card detect switch
#define MICROPY_HW_SDCARD_DETECT_PIN        (pin_C14)
#define MICROPY_HW_SDCARD_DETECT_PULL       (GPIO_PULLUP)
#define MICROPY_HW_SDCARD_DETECT_PRESENT    (GPIO_PIN_RESET)
