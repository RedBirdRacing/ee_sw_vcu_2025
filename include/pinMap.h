//#include "m328pMap.h"

#ifndef PINMAP_H
#define PINMAP_H

//#define USE_ARDUINO_PINS // Uncomment this line to use Arduino Uno pin numbers
#define USE_VCU_V2 // Uncomment this line to use VCU V2 pin numbers

// === Pinmap guard: define USE_ARDUINO_PINS to use Arduino Uno pin numbers ===
#ifndef USE_ARDUINO_PINS

#ifndef USE_VCU_V2

// VCU v3
// === CAN bus pins ===
#define CS_CAN_MOTOR PIN_PB2
#define CS_CAN_BMS PIN_PB1
#define CS_CAN_DL PIN_PD5 // Datalogger

// === APPS and Brake pins ===
#define APPS_5V PIN_PC0
#define APPS_3V3 PIN_PC1
#define BRAKE_5V_OUT PIN_PC2
#define BRAKE_IN PIN_PC3

// === Drive mode ===
#define DRIVE_MODE_LED PIN_PB0
#define DRIVE_MODE_BTN PIN_PC4

// === Buzzer for car status ===
#define BUZZER_OUT PIN_PD4
// end of VCU v3

#else // USE_VCU_V2

// VCU v2
// === CAN bus pins ===
#define CS_CAN_MOTOR PIN_PB2
#define CS_CAN_BMS PIN_PB1 // unused
#define CS_CAN_DL PIN_PD5 // Datalogger

// === APPS and Brake pins ===
#define APPS_5V PIN_PC0

//#define APPS_3V3 PIN_PC1
// temp override
#define APPS_3V3 PIN_PC0

#define BRAKE_5V_OUT PIN_PC2

// temp override
//#define BRAKE_IN PIN_PC3
#define BRAKE_IN PIN_PC1

// === Drive mode ===
#define DRIVE_MODE_LED PIN_PB0

// temp override
//#define DRIVE_MODE_BTN PIN_PC4  
#define DRIVE_MODE_BTN PIN_PC1

// === Buzzer for car status ===
#define BUZZER_OUT PIN_PD4
// end of VCU v2

#endif // USE_VCU_V2

#else // USE_ARDUINO_PINS

// === CAN bus pin for arduino testing ===
#define CS_CAN_MOTOR 9
#define CS_CAN_BMS 10
#define CS_CAN_DL 11 // Datalogger

// === APPS and Brake pins ===
#define APPS_5V A0
#define APPS_3V3 A1
#define BRAKE_5V_OUT A2
#define BRAKE_IN A3

// === Drive mode ===
#define DRIVE_MODE_LED 4
#define DRIVE_MODE_BTN 5
// === Buzzer for car status ===
#define BUZZER_OUT 6

#endif // USE_ARDUINO_PINS

#endif // PINMAP_H