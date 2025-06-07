#ifndef PINMAP_H
#define PINMAP_H

#define BTN1 5
// #define BTN2 6
// #define BTN3 7
// #define BTN4 8

// === CAN bus pins ===
// #define CS_CAN_MOTOR PB2
// #define CS_CAN_BMS PB1
// #define CS_CAN_DL PD5 // Datalogger

// === CAN bus pin for arduino testing ===
#define CS_CAN_MOTOR 10 
#define CS_CAN_BMS PB1
#define CS_CAN_DL PD5 // Datalogger

// === APPS and Brake pins ===
#define APPS_5V PC0
#define APPS_3V3 PC1
#define BRAKE_5V_OUT PC2
#define BRAKE_IN PC3

// === APPS and Brake pins for arduino testing ===
// #define APPS_5V A0   // For arduino testing
// #define APPS_3V3 A1  // For arduino testing
// #define BRAKE_5V_OUT A2  // For arduino testing
// #define BRAKE_IN A3 // For arduino testing

// === Drive mode indicator LED ===
#define LED PB0

// === Buzzer for car status ===
#define BUZZER_OUT PD4

#endif // PINMAP_H