# Red Bird Racing VCU 2025

Vehicle Control Unit (VCU) firmware for Red Bird Racing.
As of current, this codebase is ready for Gen 5 (2025) and Gen 6 (2026).

---

## Overview
This repository contains the embedded firmware for the VCU, responsible for pedal input processing, CAN communication, and vehicle state management. The codebase is modular, with a focus on reliability, maintainability, and real-time performance for electric race vehicles.

## Documentation
- [**Doxygen Documentation**](https://hkust-red-bird-racing-evrt.github.io/ee_sw_vcu_2025/)
- [**External System/Design Documentation**](https://drive.google.com/drive/folders/1Ny3oZfMf1QJ1frPJsuiHh42v5gLH2OJs?usp=drive_link)

## Key Components
- **Pedal:** Handles throttle and brake pedal input, producing output torque.
- **Telemetry:** Produces extra CAN frames for telemetry and debugging.
- **Scheduler:** Allow tasks to be run at set intervals. A mix of spinlock and yielding ensures accurate timing and maximum speeds.

## Getting Started
1. **Configure Car Constants:**
    - Edit `BoardConf.h` to choose the correct pin-mappings for a particular board.
    - Edit `Pedal.hpp` to edit Drivetrain and reverse parameters.
2. **Configure Pedal Input Constants:**
	- Edit `Curves.hpp` to set appropriate min/max values, as well as the torque curve.
3. **Build and Flash:**
	- Use PlatformIO or your preferred toolchain to build and upload the firmware.
	- Ensure the vehicle is safely jacked up and powered off during flashing.
4. **Sensor Calibration:**
	- Probe APPS sensor voltages (5V and 3.3V rails) to determine min/max values and scale the curve correctly.
	- Adjust configuration (step 2) as needed for reliable and desirable operation.

## Debugging
- Enable/disable debug messages by setting flags in `Debug.hpp`.
- Significantly slows down the VCU due to the extra work
- Should not affect motor/BMS/Telemetry frame timings majorly, though the Scheduler can handle missed frames correctly.
- Debug is centred around showing what Telemetry would show, so there's more focus on tracing the car's states before Telemetry starts communicating, or if the CAN don't work.
- **Serial Debugging:**
  - Connect the VCU to a serial monitor.
- **CAN Debugging:**
  - Connect the VCU's MCP2515 outputs to a USB PCAN.
  - Use the provided .dbc to interpret the frames.
  - Check the external doc for full references to the messages.

## Project Structure
```
include/         # Header files
lib/             # Modular libraries (Pedal, Signal_Processing, etc.)
src/             # Main application entry point (main.cpp)
scripts/         # Static analysis, formatting, and utility scripts
test/            # Unit and integration tests
Doxyfile         # Doxygen configuration
platformio.ini   # PlatformIO project config
```

## Current Development
- Debug awaiting cleanup

## Future Plans
- Continuous maintenance
- No significant development planned / expected.

---

For questions or contributions, please refer to the documentation links above or contact the Red Bird Racing Software subteam.