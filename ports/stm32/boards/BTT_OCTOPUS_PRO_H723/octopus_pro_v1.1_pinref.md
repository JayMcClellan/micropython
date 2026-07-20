# BIGTREETECH Octopus Pro V1.1 — Pin Reference (STM32H723 build)

Reference for the STM32H723ZET6 variant of the BIGTREETECH Octopus Pro V1.1
board, compiled for firmware-porting work.

Sources:
1. OEM schematic (`BIGTREETECH_Octopus_Pro_V1_1sch.pdf`, rev B/C, 5 sheets,
   dated Feb–Mar 2023).
2. BTT's own Klipper config, `generic-bigtreetech-octopus-pro-v1.1.cfg`,
   which documents the STM32H723 build variant explicitly.
3. BTT's official physical pinout diagram (`BIGTREETECH_Octopus_Pro_V1_1-Pin.jpg`,
   dated 2023-11-02), which labels silkscreen headers directly with STM32
   pin names — this is the highest-confidence source where it overlaps with
   the schematic, since it reflects the as-built board rather than a
   possibly-ambiguous net trace.

All net names are the schematic's original labels. STM32 pin names are the
MCU package pins, not header silkscreen numbers, unless noted.

**Source-confidence key:** ✅ = confirmed by the official pinout diagram
(source 3), optionally also cross-checked against sources 1/2.

## 1. MCU & clock

- MCU: **STM32H723ZET6**, LQFP144. (Schematic sheet 5 title block also lists
  STM32F446ZET6 @12 MHz and STM32F429ZGT6 @8 MHz as alternate BOM options —
  the schematic covers all three MCU variants of the V1.1 board; H723 is
  the variant documented here.)
- HSE crystal: **25.000 MHz**, confirmed directly from the crystal can
  marking visible in the official pinout diagram. ✅ This matches BTT's
  Klipper cfg note ("128KiB bootloader" + "25MHz crystal" for the H723
  build) and supersedes the schematic's Y1 footprint annotation of 12/8 MHz,
  which applies only to the F446/F429 variants.
- BOOT0: `J75` 2-pin BOOT jumper header, pulls BOOT0 pin (physical pin 138).
  Also broken out as a labeled button/pad near the USB connectors on the
  official pinout diagram.
- NRST: physical pin 25, pulled up, `SW1` reset button, also present on the
  SWD header.

## 2. Debug / programming

- SWD: `J72` 5-pin header — SWDIO, SWCLK, GND, 3V3, RESET.
- **SWD pin conflicts** — two other functions share pins with the SWD
  interface, which matters for anyone using an SWD debug probe:
  - **SWDIO = PA13 = `WORK_LED`** (status LED). The LED can be driven as a
    general-purpose GPIO output, but doing so while an SWD probe is
    attached will interfere with debugging (and toggling it as an output
    while a debugger expects SWDIO to behave as a debug pin will likely
    break the debug connection).
  - **SWCLK = PA14 = Driver7's DIR pin** (`DRIVER7_DIR`, see §3). Driver 7
    cannot be used for motion while an SWD probe is attached — driving DIR
    on that pin will disrupt the debug clock line.
  Neither conflict matters once SWD is disconnected/not in use; both pins
  are then free for normal GPIO use.
- Serial console header (10-pin, silkscreened `PD6-RX`/`PD5-TX` in the
  official diagram) — **USART2: TX = PD5, RX = PD6** ✅. This is the header
  used for a Klipper-style host-MCU serial link (e.g. to a Pi/CB1). Also
  breaks out two spare 5V/GND pairs and 4 unconnected (`NC`) pins.
- The board has **two independent USB device PHYs**, confirmed by the
  official pinout diagram, corresponding to the STM32H723's two OTG cores:
  - **USB-C connector**: OTG1 (FS-only core) — **D− = PA11, D+ = PA12** ✅.
  - **USB-A connector**: OTG2 (HS core, used in internal-PHY FS mode on this
    board) — **D− = PB14, D+ = PB15** ✅.
  The schematic's `USB2`/`KH-TYPE-C-16P` footprint with CC1/CC2 sense
  resistors describes the USB-C connector's physical connector part;
  `USB1` in the schematic is
  the USB-A connector. Net names `USB-C_DP`/`USB-C_DN` in the schematic map
  to PA12/PA11, and `USB-A_DP`/`USB-A_DN` map to PB15/PB14.

## 3. Stepper drivers (8×, M0–M7 / "Driver0"–"Driver7")

Each driver socket is a standard 18-pin BTT/Fysetc-style TMC footprint
(SPI+UART hybrid: STEP/DIR/EN plus CS which doubles as UART single-wire,
plus SCK/MOSI/MISO shared bus, plus per-driver DIAG and SLEEP). SPI bus is
shared (`SPI1_MOSI`/`SPI1_SCK`/`SPI1_MISO`), CS is per-driver and also used
as the UART pin for TMC2209-style single-wire UART mode.

| Driver | Motor conn | STEP | DIR  | EN (active-low, `!`) | CS / UART | DIAG |
|--------|-----------|------|------|-----------------------|-----------|------|
| 0 | J3 MOTOR0 | PF13 ✅ | PF12 ✅ | PF14 ✅ | PC4 ✅ | DIAG0 |
| 1 | J4 MOTOR1 | PG0 ✅  | PG1 ✅  | PF15 ✅ | PD11 ✅ | DIAG1 |
| 2 | J5 MOTOR2_1 | PF11 ✅ | PG3 ✅ | PG5 ✅ | PC6 ✅ | DIAG2 |
| 3 | J6 MOTOR3 | PG4 ✅ | PC1 ✅ | PA2 ✅ | PC7 ✅ | DIAG3 |
| 4 (extruder) | J7 MOTOR2_2 | PF9 ✅ | PF10 ✅ | PG2 ✅ | PF2 ✅ | DIAG4 |
| 5 | J11 MOTOR5 | PC13 ✅ | PF0 ✅ | PF1 ✅ | PE4 ✅ | DIAG5 |
| 6 | J13 MOTOR6 | PE2 ✅ | PE3 ✅ | PD4 ✅ | PE1 ✅ | DIAG6 |
| 7 | J12 MOTOR7 | PE6 ✅ | PA14 ✅ | PE0 ✅ | PD3 ✅ | DIAG7 |

Notes:
- All 8 driver rows are fully confirmed. Drivers 0/1/2/4/5/6 were confirmed
  against BTT's own Klipper cfg. Drivers 3 and 7 were resolved by tracing
  the schematic directly through the level-shifter ICs: PC7 → U21 pin A4 →
  U21 pin B4 → `DRIVER3_CS`; PG4 → U20 pin A3 → `DRIVER3_STEP`; PC1 →
  `DRIVER3_DIR`; PA2 → `DRIVER3_EN`; PE6 → `DRIVER7_STEP`; PD3 →
  `DRIVER7_CS`.
- **Driver7's DIR pin (PA14) is shared with SWCLK** — see the SWD
  pin-conflict note in §2. Driver 7 is unusable for motion while an SWD
  debug probe is attached.
- DIAG0–7 lines feed both the TMC stall/diag output and — via jumper headers
  (`J1`–`J2`, `J8`–`J9`, `J14`–`J15`, `J24`–`J25`, silkscreened "MS1"–"MS8")
  — into the endstop connectors for sensorless homing mode.
- SLEEP pin per driver (`+0DRV`…`+7DRV` nets) is tied via pull resistors, not
  individually GPIO-controlled — treat as hardware default-enabled.
- Shared SPI1 bus, confirmed by the official pinout diagram (labeled
  identically across all 8 driver sockets): **SCK = PA5, MISO = PA6,
  MOSI = PA7** ✅ — the standard STM32 SPI1 default AF pins. Buffered through
  74HCT125/TXS0104E level shifters (`U18`–`U27`) before reaching the driver
  sockets, so expect a small propagation delay when bit-banging.


## 4. Endstops

The board provides 8 generic 3-pin endstop inputs, `Stop0`–`Stop7`, one
nominally paired with each stepper driver — each with signal/GND/5V. These
inputs are electrically identical to the per-driver DIAG lines
(`M0DIAG`–`M7DIAG`): each stepper driver's DIAG output and its
correspondingly-numbered Stop input share the same MCU pin, confirmed
directly by the official diagram:

| Header | Also labeled | Pin |
|--------|-------------|-----|
| Stop0 | M0DIAG | PG6 ✅ |
| Stop1 | M1DIAG | PG9 ✅ |
| Stop2 | M2DIAG | PG10 ✅ |
| Stop3 | M3DIAG | PG11 ✅ |
| Stop4 | M4DIAG | PG12 ✅ |
| Stop5 | M5DIAG | PG13 ✅ |
| Stop6 | M6DIAG | PG14 ✅ |
| Stop7 | M7DIAG | PG15 ✅ |

This matches BTT's Klipper cfg, which uses PG6/PG9/PG10 as the default
X/Y/Z endstop pins — those are simply Stop0/Stop1/Stop2 wired to mechanical
switches instead of used for TMC sensorless-stall detection. A `2/4Wire`
vs `3Wire` jumper (`J84` in the schematic) selects NPN vs PNP sensor wiring,
default NPN. Axis assignment (which Stop header is X vs Y vs Z, etc.) is a
wiring/config choice, not a hardware constraint — any Stop header can serve
any axis.

## 5. Thermistors / heaters

All values below confirmed directly by the official pinout diagram. ✅

| Function | Net | Pin |
|----------|-----|-----|
| TB (bed) thermistor | TB | PF3 |
| T0 (hotend0) thermistor | TH0 / THERM0 | PF4 |
| T1 thermistor | TH1 / THERM1 | PF5 |
| T2 thermistor | TH2 / THERM2 | PF6 |
| T3 thermistor | TH3 / THERM3 | PF7 |
| HE0 heater (MOSFET) | HED0 | PA0 |
| HE1 heater | HED1 | PA3 |
| HE2 heater | HED2 | PB0 |
| HE3 heater | HED3 | PB11 |
| Bed heater (MOSFET) | BED_OUT | PA1 |

All thermistor inputs are simple resistor-divider analog pins (10K pull-up
to 3V3, standard NTC divider) — read via ADC.

PT100/PT1000 support: `J47` connector feeds a dedicated **MAX31865** RTD
amplifier (`U29`), sharing the driver SPI1 bus (SCK=PA5, MISO=PA6,
MOSI=PA7) with its own chip-select — **MAX31865_CS = PF8** ✅ (confirmed
by the official diagram). A 4-position DIP switch (`S1`) on the schematic
selects 2-wire/4-wire PT100/PT1000 sensor mode.

## 6. Fans (PWM-controlled MOSFET outputs)

All values below confirmed directly by the official pinout diagram. ✅

| Fan | Pin |
|-----|-----|
| FAN0 | PA8 (default `[fan]` part-cooling output in BTT's Klipper cfg) |
| FAN1 | PE5 |
| FAN2 | PD12 |
| FAN3 | PD13 |
| FAN4 | PD14 |
| FAN5 | PD15 |
| FAN6 | no control pin — fixed always-on output |
| FAN7 | no control pin — fixed always-on output |

FAN0–7 each have an independent voltage-select jumper (labeled `VF0`–`VF7`
in the diagram) choosing between 5V/12V/VIN rail for that fan's positive
supply.

## 7. Display / peripheral headers

**EXP1** (10-pin, LCD/TFT-style header): `EXP1_1..8 = PE8, PE7, PE9, PE10,
PE12, PE13, PE14, PE15`, `EXP1_9 = GND`, `EXP1_10 = 5V`. (Schematic-derived;
not itemized separately on the official pinout diagram, which instead shows
an EXP1/EXP2 block matching these same values.)

**EXP2** (10-pin): `EXP2_1..7 = PA6, PA5, PB1, PA4, PB2, PA7, PC15`,
`EXP2_8 = RST`, `EXP2_9 = GND`, `EXP2_10 = PC5`. ✅ (confirmed by the
official diagram, which labels this header directly).

These two headers carry `BEEPER`, `LCD_ENA/RS/D4-D7`, `BTN_ENC`, `BTN_EN1/2`,
`SD_DET`, `SD_CSEL`, plus `SPI1_MOSI/SCK/MISO` shared with the driver bus —
an LCD/SD-card front panel shares the SPI1 bus with the stepper drivers.

**Secondary display/UART header** (labeled with `5V`/`GND`/`PA9`/`PA10`/`RST`
in the official diagram): **USART1: TX = PA9, RX = PA10** ✅, plus reset —
likely intended for a TFT touchscreen module using the standard BTT TFT
UART protocol.

**I2C header** (labeled `I2C` in the official diagram): **SCL = PB8,
SDA = PB9** ✅. Also feeds an on-board **AT24C32** EEPROM (`U12`) on the
same bus — useful as a persistent config store independent of MCU flash.

**SPI3 header** (`J74`, schematic net names `SPI3_NSS/SCK/MOSI/MISO`):
**SCK = PB3, MISO = PB4, MOSI = PB5, CS = PA15** ✅ (confirmed by the
official diagram).

**SD card slot** (`J69`, micro-SD): dedicated SDIO peripheral, all pins
confirmed by the official diagram: ✅
**D0 = PC8, D1 = PC9, D2 = PC10, D3 = PC11, CLK = PC12, CMD = PD2,
DET (card-detect) = PC14**. Native SDIO, not SPI-mode.

**WiFi module header** (`U10`, ESP-12S socket): fully resolved by tracing
`U10`'s pins directly on the schematic, accounting for series resistors
that rename several nets between the MCU side and the module side. Socket
pin numbers in parentheses: ✅

| ESP-12S pin | Net | STM32 pin |
|---|---|---|
| (1) RST | ESP_RST | PG7 |
| (3) EN | ESP_EN | VCC (tied high, not GPIO-controlled) |
| (5) IO14 | SPI2_SCK | PB13 |
| (6) IO12 | SPI2_MISO | PC2 |
| (7) IO13 | SPI2_MOSI | PC3 |
| (10) IO15 | SPI2_NSS | PB12 |
| (12) IO2 | ESP_IO0 | PD7 |
| (13) IO4 | ESP_IO4 | PD10 |
| (15) RXD0 | ESP_RX / USART3_TX | PD8 |
| (16) TXD0 | ESP_TX / USART3_RX | PD9 |

Note the crossover: the module's RXD0 connects to the STM32's USART3_TX
(PD8), and the module's TXD0 connects to USART3_RX (PD9) — this is the
expected UART crossover, not an error. EN is hardwired to VCC (module
always enabled at boot; no GPIO control over module enable).

**Probe / BLTouch** (`J43`, 5-pin): **control = PB6, data = PB7** ✅
(confirmed by the official diagram, labeled `Probe`). Signal is buffered
through `U1` (74LVC1G125) per the schematic.

**Power-Det**: a separate low-power-detect input, **PC0** ✅ (confirmed by
the official diagram, labeled `Power-Det`) — distinct from the probe pins
above; do not conflate the two.

**PS_ON** (power-supply enable/soft-power control): **PE11** ✅ (confirmed
by the official diagram).

**Neopixel/RGB LED** (`J37`): **PB10** ✅ (confirmed by the official
diagram, labeled `RGB`).

**Status LED**: `WORK_LED` net — **PA13** ✅ (confirmed by direct schematic
trace). Shared with SWDIO; see the SWD pin-conflict note in §2.

## 8. Communication buses

- **CAN**: **RX = PD0, TX = PD1** ✅ (confirmed by the official diagram) from
  MCU → `U11 MCP2542` CAN transceiver → `CAN_H`/`CAN_L` on a screw-terminal
  connector, with an onboard 120Ω termination jumper (labeled `CAN-120R` in
  the diagram).
- **I2C**: see §7 (SCL = PB8, SDA = PB9), also feeds the on-board
  AT24C32 EEPROM.
- **WiFi**: `U10 ESP-12S` (ESP8266 module footprint) — see §7 for the full
  pin table. Uses SPI2 (PB12/PB13/PC2/PC3) and USART3 (PD8/PD9), independent
  of the STM32H723's main SPI1/driver bus.
- **USART1/2/3**: USART1 (PA9/PA10, secondary display header, §7), USART2
  (PD5/PD6, serial console header, §2), USART3 (PD8/PD9, WiFi header, §7).

## 9. Power rails (for reference, not GPIO)

Main power input is a screw-terminal block with four sections (labeled
`M-Power`, `Power`, `B-Power`, `Bed-OUT` in the official diagram): motor
power in (`HV`/`VM`/`VIN`), main logic power in, bed-heater power in
(`VB`), and the bed-heater MOSFET output itself (`VB`/`PA1`, matching the
bed heater pin in §5).

Internal rails: `V_FUSED` (input, after reverse-protection diode), `VM`
(motor rail, direct from input), `VCC_12V`, `VCC_5V`, `VCC_3V3`, `VDDA`
(analog 3V3), `5V_USB` (from the USB-C connector when only USB-powered,
jumper-selectable via `J68`), `V_BUS` (USB VBUS sense, also broken out as
its own labeled pad in the official diagram). Three independent buck/LDO
stages: `U5 SY8368` (main 5V buck), `U7 AOZ1283` (secondary buck),
`U28 HT7550` + `U8 AMS1117-3.3` (3V3 LDO stages).


