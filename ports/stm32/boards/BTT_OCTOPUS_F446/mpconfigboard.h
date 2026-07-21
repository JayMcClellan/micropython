#define MICROPY_HW_BOARD_NAME               "BTT_OCTOPUS_F446"
#define MICROPY_HW_MCU_NAME                 "STM32F446ZET6"

#define MICROPY_HW_ENABLE_RTC               (1)
// STM32F446 has no RNG peripheral at all (unlike the H723 Pro board, which
// has one but leaves it disabled) -- os.urandom() falls back to software.
#define MICROPY_HW_ENABLE_RNG               (0)
#define MICROPY_HW_ENABLE_ADC               (1)
#define MICROPY_HW_ENABLE_DAC               (1)
#define MICROPY_HW_ENABLE_USB               (1)
#define MICROPY_HW_ENABLE_SDCARD            (1)
#define MICROPY_HW_HAS_FLASH                (1)

// This board has a 12MHz HSE crystal. This clock config is hardware-
// verified on this exact board/MCU (see dmc_stm32/src/main.cpp).
// The following gives a 168MHz CPU speed with an exact 48MHz USB clock:
// VCO_in = 12/6 = 2MHz, VCO_out = 2*168 = 336MHz, SYSCLK = 336/2 = 168MHz,
// USB48 = 336/7 = 48MHz.
#define MICROPY_HW_CLK_USE_HSE              (1)
#define MICROPY_HW_CLK_PLLM                 (6)
#define MICROPY_HW_CLK_PLLN                 (168)
#define MICROPY_HW_CLK_PLLP                 (RCC_PLLP_DIV2)
#define MICROPY_HW_CLK_PLLQ                 (7)

// No external 32kHz crystal on this board; RTC uses the internal LSI.
// (PC14/PC15, the MCU's LSE oscillator pins, are used here as SD_DET and
// EXP2_7 -- confirming no crystal is fitted, same as the Pro board.)

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
// SPI3: header
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
// USB-C connector, on the dedicated OTG_FS core (fixed pins PA11/PA12).
// Two CDC (virtual COM port) interfaces: one is the REPL, the other is
// free for general use.
// Unlike the H723 Pro board, this MCU has a genuinely independent second
// USB controller (OTG_HS), wired to the USB-A connector (PB14/PB15,
// pins.csv USB_A_DM/USB_A_DP). It's left unconfigured here so this board
// behaves the same as the Pro board from Python -- it could be brought up
// as a second, independent USB device later if wanted.
#define MICROPY_HW_USB_FS                   (1)
#define MICROPY_HW_USB_CDC_NUM              (2)

// CAN bus
#define MICROPY_HW_CAN1_TX                  (pin_D1)
#define MICROPY_HW_CAN1_RX                  (pin_D0)

// SD card detect switch
#define MICROPY_HW_SDCARD_DETECT_PIN        (pin_C14)
#define MICROPY_HW_SDCARD_DETECT_PULL       (GPIO_PULLUP)
#define MICROPY_HW_SDCARD_DETECT_PRESENT    (GPIO_PIN_RESET)
