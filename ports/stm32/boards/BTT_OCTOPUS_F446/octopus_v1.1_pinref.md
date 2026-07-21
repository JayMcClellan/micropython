# BIGTREETECH Octopus V1.1 (non-Pro) — Pin Reference (STM32F446 build)

Reference for the STM32F446ZET6 BIGTREETECH Octopus board, compiled for
firmware-porting work. The schematic's own metadata names the design
`Octopus_V1`; board revisions labeled V1.0 and V1.1 share this schematic —
the revision bump is a silkscreen-only change, not an electrical one.

Sources:
1. OEM schematic (`BIGTREETECH_Octopus_Schematic.pdf`, rev B/C, 5 sheets,
   dated May 2021).
2. BTT's official pinout diagram (`BIGTREETECH_Octopus__PIN.pdf`, single
   page).
3. BTT's own Klipper config, `generic-bigtreetech-octopus-v1.1.cfg`, which
   confirms the large majority of pin assignments below directly.

Methodology: pin/net associations below were derived by systematically
tracing the schematic's MCU symbol block (net label immediately adjacent to
each pin, left-side labels reading as "into the pin" and right-side labels
reading as "out of the pin"), then confirmed directly against source 3
wherever a corresponding cfg entry exists.

**Note on board/firmware compatibility:** BTT's Klipper cfg for this board
carries an explicit warning not to reuse it (or, by extension, this pin
reference) on an Octopus Pro board, since mismatched heater_pin
assignments between the two boards could inadvertently enable a heater.
Treat this document and any Octopus Pro pin reference as board-specific and
not interchangeable, even where individual pins happen to match.

**Source-confidence key:** ✅ = confirmed by at least two of the three
sources above.

## 1. MCU & clock

- MCU: **STM32F446ZET6**, LQFP144. **STM32F429ZGT6** is a documented
  alternate BOM option (per BTT's Klipper cfg).
- Bootloader size: **32KiB** (per BTT's Klipper cfg).
- HSE crystal: **12 MHz** (`Y1`) on the STM32F446 build; **8 MHz** on the
  STM32F429 alternate. ✅
- BOOT0: jumper header near the MCU (schematic connector `J69` region),
  pulls BOOT0 pin (physical pin 138).
- NRST: `SW1` reset button, pulled up, also present on the SWD header
  (`J72`).

## 2. Debug / programming

- SWD: header (schematic `J72`) — SWDIO, SWCLK, GND, 3V3, RESET.
- **SWD pin conflicts**:
  - **SWDIO = PA13 = `WORK_LED`** (status LED). ✅ Driving it as a GPIO
    output while an SWD probe is attached will interfere with debugging.
  - **SWCLK = PA14 = Driver7's DIR pin** (`DRIVER7_DIR`). ✅ Driver 7 is
    unusable for motion while an SWD debug probe is attached.
  Neither conflict matters once SWD is disconnected.
- USART2 (serial console, host-MCU link): **TX = PD5, RX = PD6** ✅.
- Two independent USB device PHYs:
  - **USB-C connector** (schematic net names `USB-C_DN`/`USB-C_DP`,
    confirmed physically USB-C on this board): OTG1 (FS-only core) —
    **D− = PA11, D+ = PA12** ✅.
  - **USB-A connector**: OTG2 (HS core, internal-PHY FS mode) — **D− =
    PB14, D+ = PB15** ✅, routed through 22Ω series resistors (`R145`,
    `R147`).

## 3. Stepper drivers (8×, M0–M7 / "Driver0"–"Driver7")

18-pin TMC footprint per driver, shared SPI1 bus, per-driver CS doubling
as UART single-wire pin.

| Driver | STEP | DIR  | EN (active-low, `!`) | CS / UART | DIAG |
|--------|------|------|-----------------------|-----------|------|
| 0 | PF13 ✅ | PF12 ✅ | PF14 ✅ | PC4 ✅ | DIAG0 |
| 1 | PG0 ✅  | PG1 ✅  | PF15 ✅ | PD11 ✅ | DIAG1 |
| 2 | PF11 ✅ | PG3 ✅ | PG5 ✅ | PC6 ✅ | DIAG2 |
| 3 | PG4 ✅ | PC1 ✅ | PA0 ✅ | PC7 ✅ | DIAG3 |
| 4 (extruder) | PF9 ✅ | PF10 ✅ | PG2 ✅ | PF2 ✅ | DIAG4 |
| 5 | PC13 ✅ | PF0 ✅ | PF1 ✅ | PE4 ✅ | DIAG5 |
| 6 | PE2 ✅ | PE3 ✅ | PD4 ✅ | PE1 ✅ | DIAG6 |
| 7 | PE6 ✅ | PA14 ✅ | PE0 ✅ | PD3 ✅ | DIAG7 |

Notes:
- All STEP/DIR/EN pins are directly confirmed by the active stepper
  sections in BTT's Klipper cfg; all CS pins are confirmed by the cfg's
  commented-out `tmc2209`/`tmc2130` `uart_pin`/`cs_pin` entries; all DIAG
  pins are confirmed by the cfg's commented `diag_pin` lines. ✅
- Driver7's DIR pin (PA14) is shared with SWCLK — see §2.
- Shared SPI1 bus: **SCK = PA5, MISO = PA6, MOSI = PA7** ✅.
- SLEEP pin per driver is tied via pull resistors, not individually
  GPIO-controlled.

## 4. Endstops

The board provides 8 endstop inputs, one nominally paired with each
stepper driver. The silkscreen labels these headers `DIAG0`–`DIAG7`
(matching the underlying stall-detect signal name), but this document
renames them **Stop0–Stop7** for consistency with the equivalent header
naming used elsewhere for this board family — the numbering is unchanged,
only the label. Each Stop input is electrically identical to the
corresponding driver's DIAG line:

| Header | DIAG line | Pin |
|--------|-----------|-----|
| Stop0 | DIAG0 | PG6 ✅ |
| Stop1 | DIAG1 | PG9 ✅ |
| Stop2 | DIAG2 | PG10 ✅ |
| Stop3 | DIAG3 | PG11 ✅ |
| Stop4 | DIAG4 | PG12 ✅ |
| Stop5 | DIAG5 | PG13 ✅ |
| Stop6 | DIAG6 | PG14 ✅ |
| Stop7 | DIAG7 | PG15 ✅ |

Stop0–Stop3 (PG6/PG9/PG10/PG11) are directly confirmed by BTT's Klipper
cfg as both `endstop_pin` (active stepper sections) and `diag_pin`
(commented TMC sections) for their respective drivers. Stop4–Stop7
(PG12–PG15) are schematic-derived; the commented `[filament_switch_sensor]`
blocks in BTT's cfg use these same four pins, providing indirect
confirmation. Series resistors (100Ω–10KΩ) and pull-ups are present on
each line for sensor-type flexibility (NPN/PNP).

## 5. Thermistors / heaters

| Function | Net | Pin |
|----------|-----|-----|
| TB (bed) thermistor | TB | PF3 ✅ |
| TH0 thermistor | THERM0 | PF4 ✅ |
| TH1 thermistor | THERM1 | PF5 ✅ |
| TH2 thermistor | THERM2 | PF6 ✅ |
| TH3 thermistor | THERM3 | PF7 ✅ |
| HE0 heater (MOSFET) | HED0 | PA2 ✅ |
| HE1 heater | HED1 | PA3 ✅ |
| HE2 heater | HED2 | PB10 ✅ |
| HE3 heater | HED3 | PB11 ✅ |
| Bed heater (MOSFET) | HB | PA1 ✅ |

All thermistor pins, all heater pins, and the bed heater pin are directly
confirmed by BTT's Klipper cfg (`heater_pin`/`sensor_pin` entries in the
active `[extruder]`/`[heater_bed]` sections and the commented
`[extruder1]`–`[extruder3]` sections).

## 6. Fans (PWM-controlled MOSFET outputs)

Six fan outputs:

| Fan | Pin |
|-----|-----|
| FAN0 | PA8 ✅ |
| FAN1 | PE5 ✅ |
| FAN2 | PD12 ✅ |
| FAN3 | PD13 ✅ |
| FAN4 | PD14 ✅ |
| FAN5 | PD15 ✅ |

All confirmed directly by BTT's Klipper cfg (`[fan]` and the commented
`heater_fan`/`controller_fan` sections). The FAN6 and FAN7 connectors are always-on.

## 7. Display / peripheral headers

**EXP1** (10-pin, LCD/TFT-style header), aliases per BTT's Klipper cfg: ✅
`EXP1_1 = PE8, EXP1_2 = PE7, EXP1_3 = PE9, EXP1_4 = PE10, EXP1_5 = PE12,
EXP1_6 = PE13, EXP1_7 = PE14, EXP1_8 = PE15, EXP1_9 = GND, EXP1_10 = 5V`.
In a typical LCD panel wiring these carry `BEEPER`, `BTN_ENC` (PB7, on a
separate pin not part of this header), `LCD_ENA`, `LCD_RS`, `LCD_D4–D7`.

**EXP2** (10-pin), aliases per BTT's Klipper cfg: ✅
`EXP2_1 = PA6, EXP2_2 = PA5, EXP2_3 = PB1, EXP2_4 = PA4, EXP2_5 = PB2,
EXP2_6 = PA7, EXP2_7 = PC15, EXP2_8 = RST, EXP2_9 = GND, EXP2_10 = PC5`.
EXP2_1/2/6 (PA6/PA5/PA7) are the SPI1 bus shared with the driver bus.
EXP2_3/5 (PB1/PB2) are `BTN_EN2`/`BTN_EN1`. EXP2_4 (PA4) is `SD_CSEL`.
EXP2_7 (PC15) is `SD_DET` (the front-panel LCD's SD-card-detect, distinct
from the onboard microSD socket's `TF_DET` below).

**Secondary display/UART header** (`TFT`): **USART1: TX = PA9,
RX = PA10** ✅, plus reset.

**I2C**: **SCL = PB8, SDA = PB9** ✅.

**SPI3 header**: **SCK = PB3, MISO = PB4, MOSI = PB5, NSS = PA15** ✅.

**SD card slot** (micro-SD): dedicated SDIO peripheral — **D0 = PC8,
D1 = PC9, D2 = PC10, D3 = PC11, CLK = PC12, CMD = PD2, DET (`TF_DET`) =
PC14** ✅. A second, separate `SD_DET` net on **PC15** (EXP2_7 above)
exists for the EXP2 header's front-panel SD slot (distinct from the
onboard microSD socket's `TF_DET`), and PC15's role is directly confirmed
by BTT's Klipper cfg.

**WiFi module header** (`U10`, ESP-12S socket):

| ESP-12S pin | Net | STM32 pin |
|---|---|---|
| RST | ESP_RST | PG7 ✅ |
| EN | ESP_EN | PG8 ✅ (GPIO-controlled) |
| IO14 (SCK) | SPI2_SCK | PB13 ✅ |
| IO12 (MISO) | SPI2_MISO | PC2 ✅ |
| IO13 (MOSI) | SPI2_MOSI | PC3 ✅ |
| IO15 (NSS) | SPI2_NSS | PB12 ✅ |
| IO2 (GPIO0) | ESP_IO0 | PD7 ✅ |
| IO4 (GPIO4) | ESP_IO4 | PD10 ✅ |
| RXD0 | ESP_RX / USART3_TX | PD8 ✅ |
| TXD0 | ESP_TX / USART3_RX | PD9 ✅ |

Note the UART crossover: the module's RXD0 connects to the STM32's
USART3_TX (PD8), and the module's TXD0 connects to USART3_RX (PD9).

**BLTouch**: **control = PB6, data (sensor) = PB7** ✅ — confirmed directly
by BTT's Klipper cfg's commented `[bltouch]` section
(`sensor_pin: PB7`, `control_pin: PB6`). 

**Power-Det**: **PC0** ✅.

**PS_ON** (power-supply enable/soft-power control): **PE11** ✅.

**RGB LED**: **PB0** ✅ — confirmed directly by BTT's Klipper cfg's
commented `[neopixel my_neopixel]` section (`pin: PB0`). Driven through a
buffer (`U1`) to connector `J37`.

**Status LED**: `WORK_LED` net — **PA13** ✅, shared with SWDIO; see the
SWD conflict note in §2.

## 8. Communication buses

- **CAN**: **RX = PD0, TX = PD1** ✅ (schematic-derived), through a
  `U11 MCP2542` transceiver to a `CAN_H`/`CAN_L` connector. BTT's Klipper
  cfg confirms CAN is available on this board but doesn't itemize pins
  (Klipper's CAN support is configured at the `[mcu]` transport level, not
  via named pins).
- **I2C**: see §7 (SCL = PB8, SDA = PB9).
- **WiFi**: `U10 ESP-12S` — see §7 for the full pin table. Uses SPI2
  (PB12/PB13/PC2/PC3) and USART3 (PD8/PD9), independent of the main
  SPI1/driver bus.
- **USART1/2/3**: USART1 (PA9/PA10, secondary display header, §7), USART2
  (PD5/PD6, serial console header, §2), USART3 (PD8/PD9, WiFi header, §7).

## 9. Power rails (for reference, not GPIO)

Motor power input, main logic power input, and bed-heater power input on
screw terminals, with `V_FUSED`, `VCC_5V`, `VCC_3V3`, `VDDA` internal
rails and a `V_BUS`/USB VBUS sense net. Specific buck/LDO IC part numbers
were not itemized in this document — refer to the schematic's power sheet
(sheet 4) directly if that level of detail is needed.

