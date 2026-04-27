# PINMAP_v3.md

# ESP32-S3 + 4× VNH7040AYTR Quad Motor Controller

**Last updated:** 2026-04-09
**Source:** KiCad netlist export — `kicad-cli sch export netlist -o netlist.net esp32_s3_vnh5019_quad_motor.kicad_sch`
**Verified:** All GPIO/pin assignments extracted directly from netlist output and cross-checked against ESP32-S3-WROOM-1 module pinout.

> **Note on module pin numbering:** The ESP32-S3-WROOM-1 has 38 standard castellated edge pads (pins 1–38) plus additional bottom pads. Pins 38 and 39 are IO2 and IO1 respectively — these are bottom pads on the module, not part of the standard edge row. Pins 40–41 are GND bottom pads.

---

## System Overview

| Component | Part | Role |
|-----------|------|------|
| MCU | ESP32-S3-WROOM-1 (U1) | Main controller |
| Motor drivers | 4× VNH7040AYTR (U5–U8) | H-bridge, ~12A per channel |
| GPIO expander | MCP23017-E/SS (U4) | I2C (0x20) — motor direction, LEDs, MultiSense select |
| PWM | ESP32 LEDC | Motor speed control |
| Current feedback | M*_CS signals → ADC | Via RC filter |
| Power | AP63200WU-7 (U3) | USB 5V → 3.3V buck |
| USB | USB-C + USBLC6-2SC6 ESD (U2) | Power + programming |

---

## ESP32-S3 GPIO Assignments (Verified from Netlist)

### Reserved / System Pins

| Function | GPIO | Module Pin | Notes |
|----------|------|------------|-------|
| GND | — | 1, 38†, 40, 41 | |
| 3V3 | — | 2 | |
| EN | — | 3 | Reset circuit |
| USB D− | — | 13 | IO19, USB-C connector |
| USB D+ | — | 14 | IO20, USB-C connector |
| BOOT | 0 | 27 | SW1 boot button |

† Pin 38 edge pad = GND; IO2 is on the bottom pad also numbered 38 in the KiCad symbol.

---

### I2C Bus (to MCP23017 U4)

| Signal | GPIO | Module Pin |
|--------|------|------------|
| I2C_SCL | 15 | 8 |
| I2C_SDA | 16 | 9 |

---

### PWM Outputs (to VNH7040 drivers)

| Motor | Net | GPIO | Module Pin |
|-------|-----|------|------------|
| M1 | M1_PWM_ESP | 8 | 12 |
| M2 | M2_PWM_ESP | 48 | 25 |
| M3 | M3_PWM_ESP | **13** | **21** |
| M4 | M4_PWM_ESP | 9 | 17 |

---

### Current Sense ADC Inputs (MultiSense via RC filter)

| Motor | Net | GPIO | Module Pin |
|-------|-----|------|------------|
| M1 | M1_CS_ADC | 4 | 4 |
| M2 | M2_CS_ADC | 1 | 39 |
| M3 | M3_CS_ADC | 2 | 38 |
| M4 | M4_CS_ADC | 5 | 5 |

---

### Encoder Inputs (100Ω series resistors R44–R51)

| Motor | Signal | Net | GPIO | Module Pin |
|-------|--------|-----|------|------------|
| M1 | Encoder A | ENCA_M1 | 17 | 10 |
| M1 | Encoder B | ENCB_M1 | 7 | 7 |
| M2 | Encoder A | ENCA_M2 | **38** | **31** |
| M2 | Encoder B | ENCB_M2 | **39** | **32** |
| M3 | Encoder A | ENCA_M3 | **14** | **22** |
| M3 | Encoder B | ENCB_M3 | **47** | **24** |
| M4 | Encoder A | ENCA_M4 | **12** | **20** |
| M4 | Encoder B | ENCB_M4 | **11** | **19** |

---

### Status LED and Load Switch

| Function | Net | GPIO | Module Pin | Notes |
|----------|-----|------|------------|-------|
| Blue LED D5 | LED_ESP32 | 10 | 18 | 330Ω series (R_LED1) |
| Load switch Q1 | SW_CTRL | 42 | 35 | Gate via R19 100Ω; R18 10kΩ pull-down |

---

### Expansion

| Net | GPIO | Module Pin | Connector |
|-----|------|------------|-----------|
| GPIO40 | 40 | 33 | J13 Pin 1 |
| GPIO41 | 41 | 34 | J13 Pin 2 |

---

### Not Connected (no_connect in schematic)

| GPIO | Module Pin |
|------|------------|
| 3 | 15 |
| 6 | 6 |
| 18 | 11 |
| 21 | 23 |
| 35 | 28 |
| 36 | 29 |
| 37 | 30 |
| 45 | 26 |
| 46 | 16 |
| IO43 (TXD0) | 37 |
| IO44 (RXD0) | 36 |

---

## MCP23017 GPIO Expander (U4)

**I2C address:** 0x20 (A0=A1=A2=GND)
**RESET:** tied to ESP_EN

### Port A (GPA0–GPA7) — all outputs

| Port | MCP Pin | Net | Function |
|------|---------|-----|----------|
| GPA0 | 21 | M1_INB_ESP32 | Motor 1 direction B |
| GPA1 | 22 | Red_LED | Red status LED (D1) |
| GPA2 | 23 | Green_LED | Green status LED (D2) |
| GPA3 | 24 | M1_INA_ESP32 | Motor 1 direction A |
| GPA4 | 25 | M4_INB_ESP32 | Motor 4 direction B |
| GPA5 | 26 | SEL_0_CTRL | MultiSense SEL0 |
| GPA6 | 27 | SEL_1_CTRL | MultiSense SEL1 |
| GPA7 | 28 | M4_INA_ESP32 | Motor 4 direction A |

### Port B (GPB0–GPB7) — all outputs

| Port | MCP Pin | Net | Function |
|------|---------|-----|----------|
| GPB0 | 1 | M3_INB_ESP32 | Motor 3 direction B |
| GPB1 | 2 | — | Not connected |
| GPB2 | 3 | — | Not connected |
| GPB3 | 4 | M3_INA_ESP32 | Motor 3 direction A |
| GPB4 | 5 | M2_INB_ESP32 | Motor 2 direction B |
| GPB5 | 6 | — | Not connected |
| GPB6 | 7 | — | Not connected |
| GPB7 | 8 | M2_INA_ESP32 | Motor 2 direction A |

### Initialisation

```
IODIRA = 0x00  // All Port A as outputs
IODIRB = 0x00  // All Port B as outputs
```

---

## Motor Direction Truth Table

| INA | INB | Function |
|-----|-----|----------|
| L | L | Brake (low side) |
| H | L | Forward |
| L | H | Reverse |
| H | H | Brake (high side) |

---

## Motor Control Signal Summary

| Motor | Driver | PWM GPIO | INA (MCP port) | INB (MCP port) | CS ADC GPIO | ENC A GPIO | ENC B GPIO |
|-------|--------|----------|----------------|----------------|-------------|------------|------------|
| M1 | U5 | 8 | GPA3 | GPA0 | 4 | 17 | 7 |
| M2 | U6 | 48 | GPB7 | GPB4 | 1 | 38 | 39 |
| M3 | U7 | 13 | GPB3 | GPB0 | 2 | 14 | 47 |
| M4 | U8 | 9 | GPA7 | GPA4 | 5 | 12 | 11 |

---

## Firmware Pin Defines (reference)

```cpp
// I2C
#define I2C_SCL_PIN     15
#define I2C_SDA_PIN     16

// PWM (LEDC)
#define M1_PWM_PIN      8
#define M2_PWM_PIN      48
#define M3_PWM_PIN      13
#define M4_PWM_PIN      9

// Current sense ADC
#define M1_CS_PIN       4
#define M2_CS_PIN       1
#define M3_CS_PIN       2
#define M4_CS_PIN       5

// Encoders
#define M1_ENCA_PIN     17
#define M1_ENCB_PIN     7
#define M2_ENCA_PIN     38
#define M2_ENCB_PIN     39
#define M3_ENCA_PIN     14
#define M3_ENCB_PIN     47
#define M4_ENCA_PIN     12
#define M4_ENCB_PIN     11

// Misc
#define LED_STATUS_PIN  10   // Blue LED D5
#define SWITCH_CTRL_PIN 42   // Q1 MOSFET gate (load switch)
```

---

## VNH7040AYTR Per-Driver Connections

| Signal | VNH7040 Pin | Source |
|--------|-------------|--------|
| INA | 16 | MCP23017 via 1kΩ series resistor |
| INB | 21 | MCP23017 via 1kΩ series resistor |
| PWM | 17 | ESP32 via 1kΩ series resistor |
| MultiSense | 19 | → 1kΩ → RC filter (100nF) → ESP32 ADC |
| SEL0 | 15 | MCP23017 GPA5 via 1kΩ (shared all drivers) |
| SEL1 | 22 | MCP23017 GPA6 via 1kΩ (shared all drivers) |
| MS_EN | 20 | Tied to 3V3 |
| OUTA | 11–14 | Motor terminal + |
| OUTB | 23–26 | Motor terminal − |

---

## Load Switch Circuit (J14/J15/J16)

```
GPIO42 → R19 (100Ω) ─┬─ Q1 Gate (AO3400A SOT-23)
                      └─ R18 (10kΩ) → GND
Q1 Source → GND
Q1 Drain ──┬── D3 Anode (SS14 flyback)
           └── LOAD_OUT
D3 Cathode → EXT_V+
```

| Ref | Value | Package | LCSC | Function |
|-----|-------|---------|------|----------|
| Q1 | AO3400A | SOT-23 | C20917 | N-ch MOSFET, 30V 5.7A |
| R19 | 100Ω | 0805 | C17408 | Gate series resistor |
| R18 | 10kΩ | 0805 | C17414 | Gate pull-down |
| D3 | SS14 | SMA | C2480 | Flyback diode |
