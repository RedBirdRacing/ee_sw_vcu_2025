#ifndef PINMAP_H
#define PINMAP_H

#define USE_ARDUINO_PINS // Uncomment this line to use Arduino Uno pin numbers

// === Pinmap guard: define USE_ARDUINO_PINS to use Arduino Uno pin numbers ===
#ifndef USE_ARDUINO_PINS

// === CAN bus pins ===
#define CS_CAN_MOTOR PB2
#define CS_CAN_BMS PB1
#define CS_CAN_DL PD5 // Datalogger

// === APPS and Brake pins ===
#define APPS_5V PC0
#define APPS_3V3 PC1
#define BRAKE_5V_OUT PC2
#define BRAKE_IN PC3

// === Drive mode ===
#define DRIVE_MODE_LED PB0
#define DRIVE_MODE_BTN PC4

// === Buzzer for car status ===
#define BUZZER_OUT PD4

#else

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