# Project Description — ESP32-S3 Quad Motor Controller v4

**Last updated:** 2026-04-09
**Status:** Schematic and PCB layout complete, DRC clean. Not yet fabricated.

---

## Overview

4-layer 120×80mm PCB motor controller for driving four brushed DC motors with encoder feedback. Designed for robotics applications. Motor drive at up to ~12A per channel. All control via ESP32-S3 over Wi-Fi/USB; a web dashboard and REST API are provided by the firmware.

---

## Key ICs

| Ref | Part | Package | Function |
|-----|------|---------|----------|
| U1 | ESP32-S3-WROOM-1 | Module | MCU — Wi-Fi, PWM, ADC, I2C, USB |
| U2 | USBLC6-2SC6 | SOT-23-6 | USB ESD protection |
| U3 | AP63203WU-7 | TSOT-23-6 | Buck regulator, 5V → 3.3V |
| U4 | MCP23017-E/SS | SSOP-28 | I2C GPIO expander (I2C 0x20) |
| U5–U8 | VNH7040AYTR | PowerSSO-36 | H-bridge motor drivers, ~12A each |
| Q1 | AO3400A | SOT-23 | N-ch MOSFET, GPIO42 low-side load switch |
| D3 | SS14 | SMA | Flyback diode for load switch |

---

## Board Specifications

| Property | Value |
|----------|-------|
| Dimensions | 120 × 80 mm |
| Layers | 4 |
| Motor channels | 4 (M1–M4) |
| Motor current | ~12A per channel (VNH7040 thermal limited) |
| Motor voltage | 4–28V (VNH7040 rated) via J2 (XT60) |
| Logic voltage | 3.3V from USB 5V via AP63203WU-7 |
| USB | USB-C (J1), USB 2.0 |
| Encoder inputs | 4× quadrature (J7–J10, JST-XH 4-pin) |
| I2C expansion | J12 (4-pin: GND, 3V3, SCL, SDA) |
| GPIO expansion | J13 (6-pin: GPIO40, GPIO41, 3V3×2, GND×2) |
| Load switch output | J15/J16 — GPIO42-controlled, up to ~2A |
| Mounting holes | H1–H4 (corner) |

---

## Layer Stackup

| Layer | Name | Purpose |
|-------|------|---------|
| L1 | F.Cu | Components, signal traces, local VMOT and motor output copper pours |
| L2 | In1.Cu | Solid GND plane |
| L3 | In2.Cu | Solid VMOT plane |
| L4 | B.Cu | Solid GND plane + MultiSense analog signals |

---

## Board Layout

```
                        TOP EDGE
   ┌──────────────────────────────────────────────────────────────┐
   │   ESP32  SW1(Boot) SW2(Reset)  USB-C(J1)  J12(I2C)  J15/J16│
   │                                                              │
L  │  J3(M1 out)                              J4(M2 out)         │ R
E  │  J7(M1 enc)                              J8(M2 enc)         │ I
F  │                                                              │ G
T  │  U5(M1)  U6(M2)                                             │ H
E  │  U8(M4)  U7(M3)    U4(MCP23017)                             │ T
D  │  [bulk caps adjacent to drivers]                             │ E
G  │  J6(M4 out)                              J5(M3 out)         │ D
E  │  J10(M4 enc)                             J9(M3 enc)         │ G
   │                                                              │ E
   │                    J2 (VMOT power in, XT60)                  │
   └──────────────────────────────────────────────────────────────┘
                        BOTTOM EDGE
```

### Edge Assignments

| Edge | Components |
|------|-----------|
| Top | U1 (ESP32-S3), SW1 (Boot), SW2 (Reset), J1 (USB-C), J12 (I2C expansion), J15/J16 (load switch) |
| Left | J3 (M1 output), J7 (M1 encoder), J6 (M4 output), J10 (M4 encoder) |
| Right | J4 (M2 output), J8 (M2 encoder), J5 (M3 output), J9 (M3 encoder) |
| Bottom | J2 (VMOT power input, XT60) |
| Centre | U4 (MCP23017) |
| Left-centre | U5/U6/U7/U8 (VNH7040 drivers) with adjacent bulk capacitors |

---

## Connectors

| Ref | Type | Pitch | Function |
|-----|------|-------|----------|
| J1 | USB-C SMD 16P | — | Power + USB programming |
| J2 | XT60PW-M | 7.2mm | VMOT input (motor power) |
| J3–J6 | Phoenix MKDS-3, 2-pin | 5.08mm | Motor outputs M1–M4 (OUTA, OUTB) |
| J7–J10 | JST-XH 4-pin | 2.5mm | Encoder inputs (3V3, GND, ENC_A, ENC_B) |
| J11 | 2×2 pin header | 2.54mm | 5V breakout |
| J12 | 1×4 pin header | 2.54mm | I2C expansion (GND, 3V3, SCL, SDA) |
| J13 | 2×3 pin header | 2.54mm | GPIO expansion (GPIO40, GPIO41, 3V3×2, GND×2) |
| J15/J16 | 1×2 pin header | 2.54mm | Load switch output (OUT −/+) and input (IN +/−) |

---

## GPIO Map (ESP32-S3-WROOM-1 U1)

All assignments verified from KiCad netlist. See `docs/PINMAP_v3.md` for full detail.

| Function | GPIO | Module Pin |
|----------|------|------------|
| I2C SCL | 15 | 8 |
| I2C SDA | 16 | 9 |
| M1 PWM | 8 | 12 |
| M2 PWM | 48 | 25 |
| M3 PWM | **13** | **21** |
| M4 PWM | 9 | 17 |
| M1 CS ADC | 4 | 4 |
| M2 CS ADC | 1 | 39 |
| M3 CS ADC | 2 | 38 |
| M4 CS ADC | 5 | 5 |
| M1 ENC A | 17 | 10 |
| M1 ENC B | 7 | 7 |
| M2 ENC A | **38** | **31** |
| M2 ENC B | **39** | **32** |
| M3 ENC A | **14** | **22** |
| M3 ENC B | **47** | **24** |
| M4 ENC A | **12** | **20** |
| M4 ENC B | **11** | **19** |
| Blue LED D5 | 10 | 18 |
| Load switch Q1 | 42 | 35 |
| GPIO expansion | 40, 41 | 33, 34 |

> Bold entries were corrected 2026-04-09 after cross-checking against the netlist — prior versions of PINMAP_v3 had M3 PWM wrong (was GPIO47) and M2/M3/M4 encoder A/B swapped.

---

## MCP23017 GPIO Expander (U4)

I2C address 0x20 (A0=A1=A2=GND). All pins configured as outputs.

| Motor | INA | INB |
|-------|-----|-----|
| M1 | GPA3 | GPA0 |
| M2 | GPB7 | GPB4 |
| M3 | GPB3 | GPB0 |
| M4 | GPA7 | GPA4 |

| Signal | Port |
|--------|------|
| Red LED (D1) | GPA1 |
| Green LED (D2) | GPA2 |
| MultiSense SEL0 | GPA5 |
| MultiSense SEL1 | GPA6 |

Motor direction controlled via I2C — **not** direct ESP32 GPIO.

Motor direction truth table:

| INA | INB | Function |
|-----|-----|----------|
| L | L | Brake (low side) |
| H | L | Forward |
| L | H | Reverse |
| H | H | Brake (high side) |

---

## VNH7040AYTR Motor Driver

Per-driver control signals (all have 1kΩ series resistors):

| Signal | VNH7040 Pin | Source |
|--------|-------------|--------|
| INA | 16 | MCP23017 |
| INB | 21 | MCP23017 |
| PWM | 17 | ESP32 LEDC |
| SEL0 | 15 | MCP23017 GPA5 (shared all drivers) |
| SEL1 | 22 | MCP23017 GPA6 (shared all drivers) |
| MS_EN | 20 | Tied to 3V3 |
| MultiSense | 19 | → 1kΩ → 100nF RC filter → ESP32 ADC |

Bulk decoupling: 470µF SMD electrolytic + 10µF ceramic adjacent to each driver.
Thermal vias: array under exposed pad (PowerSSO-36), connected to GND.

---

## Current Sense (MultiSense)

The VNH7040 MultiSense output (pin 19) multiplexes motor current, temperature, and fault signals depending on SEL0/SEL1. All four drivers share the same SEL0/SEL1 lines. Each driver has its own RC filter (1kΩ + 100nF) and ADC input.

| SEL1 | SEL0 | MultiSense Output |
|------|------|-------------------|
| 0 | 0 | Motor current sense |
| 0 | 1 | Temperature sense |
| 1 | 0 | Fault indication |
| 1 | 1 | MS_EN control |

---

## Power Architecture

```
USB-C (J1) ──→ 5V_USB ──→ AP63203WU-7 (U3) ──→ 3V3 (logic, ESP32, MCP23017)
                   │
                   └──→ 5V (J11 breakout)

J2 (XT60) ──→ VMOT ──→ VNH7040 drivers (U5–U8)
                 └──→ VMOT pour (In2.Cu)
```

- 3V3 bulk: C1, C2 (22µF), C17, C20 (10µF)
- VMOT bulk: C3, C8, C10, C13, C16 (470µF SMD electrolytic, one per driver zone)
- Motor driver local decoupling: C4/C7/C9/C12/C15 (10µF ceramic)

---

## USB

- Connector: HRO TYPE-C-31-M-12 SMD 16-pin (J1, LCSC C2765186)
- ESD protection: USBLC6-2SC6 (U2, LCSC C2687116)
- D+/D− differential pair: 0.2mm trace, 0.15mm gap, length-matched
- ESP32 antenna: overhangs or clears top board edge — no copper beneath antenna area

---

## Load Switch Circuit

GPIO42 drives a low-side N-channel MOSFET for switching external 1–2A loads at external supply voltage.

```
GPIO42 → R19 (100Ω) ─┬─ Q1 Gate (AO3400A)
                      └─ R18 (10kΩ) → GND
Q1 Source → GND
Q1 Drain ──┬── D3 Cathode (SS14 flyback, required for inductive loads)
           └── J15/J16 LOAD_OUT
D3 Anode → EXT_V+ (J15/J16)
```

- R18 (10kΩ): gate pull-down keeps Q1 off during boot/reset
- R19 (100Ω): limits gate inrush, reduces switching EMI
- Rated: ~2A continuous, 3.7–8.4V external supply

---

## Routing Notes

| Net class | Rule |
|-----------|------|
| Motor output (OUTA/OUTB) | F.Cu copper pour, ≥3mm wide, solid pad connection |
| VMOT | Pour islands on F.Cu, stitched to In2.Cu via vias |
| 3V3 | Explicit traces only on F.Cu — no pour (avoids interference with VMOT/motor pours) |
| GND | In1.Cu and B.Cu solid planes; ~40–60 stitching vias throughout |
| MultiSense ADC | B.Cu only, over solid GND plane |
| USB D+/D− | 0.2mm trace, 0.15mm gap, length-matched, 90Ω differential |
| Encoder signals | 100Ω series resistors (R44–R51) at ESP32 side |
| INA/INB/PWM | 1kΩ series resistors at MCP23017/ESP32 side |

---

## DRC Status (2026-04-09)

- **0 errors**
- **0 unconnected pads**
- **0 footprint errors**
- 241 warnings — all cosmetic or intentional:
  - 95 silkscreen overlap
  - 72 dangling via (thermal/stitching via arrays on VMOT/GND nets — expected)
  - 44 silkscreen over copper
  - 26 silkscreen edge clearance
  - 4 text height (U5–U8 ref designator 0.787mm vs 0.8mm minimum — negligible)

---

## Key Files

| File | Description |
|------|-------------|
| `kicad/esp32_s3_vnh5019_quad_motor.kicad_sch` | Schematic |
| `kicad/esp32_s3_vnh5019_quad_motor.kicad_pcb` | PCB layout |
| `kicad/BOM_csvs/esp32_s3_vnh5019_quad_motor_BOM_JLCPCB.csv` | BOM (JLCPCB SMD assembly) |
| `kicad/BOM_csvs/esp32_s3_vnh5019_quad_motor_CPL_JLCPCB.csv` | CPL (pick-and-place) |
| `kicad/gerbers/` | Last exported Gerbers (regenerate before ordering) |
| `kicad/DRC.rpt` | Latest DRC report |
| `docs/PINMAP_v3.md` | Verified GPIO/signal assignments (authoritative) |
| `docs/SCHEMATIC_SPEC_COMPLETE.md` | Full schematic specification |
| `software_base/mc_esp32/` | PlatformIO firmware project |

---

## What Remains Before Fabrication

- [ ] Regenerate Gerbers from KiCad
- [ ] Confirm BOM LCSC part numbers still in stock
- [ ] Validate firmware pin defines against corrected PINMAP (especially M3 PWM = GPIO13, encoder A/B corrections)
- [ ] Validate firmware on hardware after board arrives
