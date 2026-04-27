# four_motor_controller

A 4-layer 120×80mm PCB motor controller for robotics applications. Drives 4 DC motors with encoder feedback using an ESP32-S3 microcontroller, 4× VNH7040AYTR high-current motor drivers (~12A per channel), and an MCP23017 I2C GPIO expander for direction control.

The design files are for KiCad 10.0.

## Repository Contents

```
hardware/         # KiCad schematic, PCB layout, and project files
  symbols/        # Custom KiCad symbol and footprint for VNH7040AYTR
outputs/          # BOM (JLCPCB format), Gerbers (zip), PINMAP, and PCB PDF
```

## Important Notes

**The BOM and PINMAP in `outputs/` may be out of date.** Always cross-reference against the schematic and PCB files before ordering parts or assembling hardware.

**The custom VNH7040AYTR symbol/footprint** (`hardware/symbols/`) must be added to your KiCad library if opening the project on a new machine.

## Contributing

This is primarily a personal project and is not actively maintained as a collaborative effort. You are welcome to fork it and adapt it for your own use. Pull requests with meaningful fixes or improvements may be considered, but there is no guarantee of review or merging.

## Disclaimer

This hardware design is provided as-is, with no warranty of any kind. Anyone who uses, manufactures, or builds upon these files does so entirely at their own risk. The author accepts no responsibility for any damage to property, equipment, or persons resulting from the use of this design. Always verify the design independently before fabrication or use in any application.

## Licence

The hardware design files in this repository are licensed under the
CERN Open Hardware Licence Version 2 - Permissive (CERN-OHL-P-2.0).

See the `LICENSE` file for the full licence text.
