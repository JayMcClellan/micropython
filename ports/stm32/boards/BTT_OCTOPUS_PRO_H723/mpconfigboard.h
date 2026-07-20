#define MICROPY_HW_BOARD_NAME               "BTT_OCTOPUS_PRO_H723"
#define MICROPY_HW_MCU_NAME                 "STM32H723ZGT6"

#define MICROPY_HW_ENABLE_RTC               (1)
#define MICROPY_HW_ENABLE_RNG               (0) // RNG needs proper configuration
#define MICROPY_HW_ENABLE_ADC               (1)
#define MICROPY_HW_ENABLE_DAC               (1)
#define MICROPY_HW_ENABLE_USB               (1)
#define MICROPY_HW_ENABLE_SDCARD            (1)
#define MICROPY_HW_HAS_FLASH                (1)

#define MICROPY_BOARD_EARLY_INIT            BTT_OCTOPUS_PRO_H723_board_early_init

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
#define MICROPY_HW_UART2_TX                 (pin_D5)
#define MICROPY_HW_UART2_RX                 (pin_D6)
#define MICROPY_HW_UART2_RTS                (pin_D4)
#define MICROPY_HW_UART2_CTS                (pin_D3)
#define MICROPY_HW_UART3_TX                 (pin_D8)
#define MICROPY_HW_UART3_RX                 (pin_D9)
#define MICROPY_HW_UART5_TX                 (pin_B6)
#define MICROPY_HW_UART5_RX                 (pin_B12)
#define MICROPY_HW_UART6_TX                 (pin_C6)
#define MICROPY_HW_UART6_RX                 (pin_C7)
#define MICROPY_HW_UART7_TX                 (pin_F7)
#define MICROPY_HW_UART7_RX                 (pin_F6)
#define MICROPY_HW_UART8_TX                 (pin_E1)
#define MICROPY_HW_UART8_RX                 (pin_E0)

#define MICROPY_HW_UART_REPL                PYB_UART_3
#define MICROPY_HW_UART_REPL_BAUD           115200

// I2C buses
#define MICROPY_HW_I2C1_SCL                 (pin_B8)
#define MICROPY_HW_I2C1_SDA                 (pin_B9)
#define MICROPY_HW_I2C2_SCL                 (pin_F1)
#define MICROPY_HW_I2C2_SDA                 (pin_F0)
#define MICROPY_HW_I2C4_SCL                 (pin_F14)
#define MICROPY_HW_I2C4_SDA                 (pin_F15)

// SPI buses
#define MICROPY_HW_SPI3_NSS                 (pin_A4)
#define MICROPY_HW_SPI3_SCK                 (pin_B3)
#define MICROPY_HW_SPI3_MISO                (pin_B4)
#define MICROPY_HW_SPI3_MOSI                (pin_B5)

// LEDs
#define MICROPY_HW_LED1                     (pin_B0)    // green
#define MICROPY_HW_LED2                     (pin_E1)    // yellow
#define MICROPY_HW_LED3                     (pin_B14)   // red
#define MICROPY_HW_LED_ON(pin)              (mp_hal_pin_high(pin))
#define MICROPY_HW_LED_OFF(pin)             (mp_hal_pin_low(pin))

// USB config
#define MICROPY_HW_USB_HS                   (1)
#define MICROPY_HW_USB_HS_IN_FS             (1)
#define MICROPY_HW_USB_VBUS_DETECT_PIN      (pin_A9)
#define MICROPY_HW_USB_OTG_ID_PIN           (pin_A10)

// FDCAN bus
#define MICROPY_HW_CAN1_NAME                "FDCAN1"
#define MICROPY_HW_CAN1_TX                  (pin_D1)
#define MICROPY_HW_CAN1_RX                  (pin_D0)

// SD card detect switch
#define MICROPY_HW_SDCARD_DETECT_PIN        (pin_G2)
#define MICROPY_HW_SDCARD_DETECT_PULL       (GPIO_PULLUP)
#define MICROPY_HW_SDCARD_DETECT_PRESENT    (GPIO_PIN_RESET)

void BTT_OCTOPUS_PRO_H723_board_early_init(void);
