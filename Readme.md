# Red Bird Racing Vehicle Control Unit (VCU) 2025

## Files:
vcu_firmware.ino: Main file

Pedal.cpp, Pedal.h: Pedal object encapsulating functionality for reading pedal signal and constructing CAN frames

Queue.cpp, Queue.h, Signal_Processing.cpp, Signal_Processing.h: For filtering pedal input

pinMap.h: Pin mapping

Debug.h: Debugging

## Set-up and tuning:
1. Choose appropiate values for pedal input constants in pedal.h
2. Flash the vcu, make sure the car is jacked up and shut off during the process
3. Clear people around the car (esp. around the rear)
4. Test minimum and maximum values by probing the input voltage for APPS sensor (Input voltage for both 5v and 3.3v) Adjust the values (Including deadzones)

## Debugging:
Currently, debugging is done through Serial.

Connect VCU to a serial monitor.

Select which debug messages are needed by setting the respective debug flags under debug.h to true 

***If debug is set to true, expect large delays between pedal input and motor output due to Serial being slow af***

## Reverse Mode:
Although banned by rules, a reverse mode exist for testing purposes ***ONLY***.
The driver must hold down the reverse button to allow the car to reverse.
If the button is released, the car is placed in neutral. Throttle will not apply power.

## Future development:
- More CANBUS channels for BMS, datalogger and other components
- Better torque curve
- Reverse Mode full implementation

## Folder structure
```
include
| headers
lib
| libraries (folders)
| | library.json (e.g. in Pedal)  --> link with other files in include/lib
src
| main.cpp
```
