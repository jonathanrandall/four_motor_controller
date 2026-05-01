# four_motor_controller

A 4-layer PCB motor controller for robotics applications. Drives 4 DC motors with encoder feedback using an ESP32-S3 microcontroller, 

* 4× VNH7040AYTR high-current motor drivers, and an MCP23017 I2C GPIO expander for direction control.

The design files are for KiCad 10.0.

The ROS2 control code used in the video is at https://github.com/jonathanrandall/new_pcb_robot

## Repository Contents

```
hardware/                   # KiCad schematic, PCB layout, and project files
  symbols/                  # Custom KiCad symbol and footprint for VNH7040AYTR
docs_and_outputs/           # BOM (JLCPCB format), Gerbers (zip), PINMAP, and datasheets
firmware/esp32_bridge/      # PlatformIO firmware project (Arduino framework, C++17)
  src/                      # main.cpp, Motor, PIDController, QuadratureEncoder,
                            #   RobotController, Mcp23017Bus, WebDashboard, PanTiltController
  include/                  # Header files
  platformio.ini            # Build configuration
```

## Important Notes

**The BOM and PINMAP in `docs_and_outputs/` may be out of date.** Always cross-reference against the schematic and PCB files before ordering parts or assembling hardware.

**The custom VNH7040AYTR symbol/footprint** (`hardware/symbols/`) must be added to your KiCad library if opening the project on a new machine.

**The firmware has not been validated on hardware.** It compiles and is provided as a starting point.

**WiFi credentials are not included.** Before building the firmware, set your SSID and password in `firmware/esp32_bridge/src/main.cpp`.

## Building the Firmware

Requires [PlatformIO](https://platformio.org/). All commands run from `firmware/esp32_bridge/`.

```bash
# Build
pio run

# Flash (USB-C, use BOOT mode if required)
pio run --target upload

# Monitor serial output (115200 baud)
pio device monitor
```

## Contributing

Contributions are welcome. If you have your own motor controller design or variant you would like to share, please add it in its own directory rather than modifying the existing files. Pull requests with bug fixes or improvements to the existing design are welcome.

## Disclaimer

This hardware design is provided as-is, with no warranty of any kind. Anyone who uses, manufactures, or builds upon these files does so entirely at their own risk. The author accepts no responsibility for any damage to property, equipment, or persons resulting from the use of this design. Always verify the design independently before fabrication or use in any application.

## Licence

The hardware design files in this repository are licensed under the
CERN Open Hardware Licence Version 2 - Permissive (CERN-OHL-P-2.0).

See the `LICENSE` file for the full licence text.
