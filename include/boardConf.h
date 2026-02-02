/**
 * @file BoardConf.h
 * @brief Board configuration for the VCU (Vehicle Control Unit)
 * This file defines the board configuration and pin mappings for different versions of the VCU and for Arduino Uno.
 * Define the appropriate macro to select the desired board configuration.
 *
 * @par Options
 * - @c USE_ARDUINO_PINS: Use Arduino Uno pin numbers
 * - @c USE_VCU_V2: Use VCU V2 pin numbers
 * - @c USE_VCU_V3: Use VCU V3 pin numbers
 * - @c USE_3CH_CAN: Use 3 channel CAN dev board pin numbers
 * - @c USE_VCU_V3_2: Use VCU V3.2 pin numbers
 * - Undefined: no pins defined, intentional compilation error
 *
 * @note Only one option should be uncommented at a time.
 */

#ifndef BOARDCONF_H
#define BOARDCONF_H

// select the board configuration to use
#define USE_VCU_V3_2

// VCU v3.2

#ifdef USE_VCU_V3_2
// === CAN bus pins ===
#define CS_CAN_MOTOR PIN_PD1 // CAN 1
#define CS_CAN_BMS PIN_PB2   // CAN 3
#define CS_CAN_DL PIN_PB1    // CAN 2

// === APPS and Brake pins ===
#define APPS_5V PIN_A6
#define APPS_3V3 PIN_A7
#define BRAKE_IN PIN_PC0
#define HALL_SENSOR PIN_PC1

// VCU brake light
#define BRAKE_LIGHT PIN_PD5 // P=Out1

// === Drive mode ===
#define FRG PIN_PD7 // P=Out3
#define DRIVE_MODE_BTN PIN_PC5 // IGN_5V


// === Buzzer for car status ===
#define BUZZER PIN_PD6 // P=Out2

// === BMS HV start failed LED ===
#define BMS_FAILED_LED PIN_PD7 // P=Out2

// === Button active state ===
#define BUTTON_ACTIVE HIGH

// === MCP2515 crystal frequency ===
#define MCP2515_CRYSTAL_FREQ MCP_20MHZ
#endif // USE_VCU_V3_2

// VCU v3

#ifdef USE_VCU_V3
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
#define FRG PIN_PB0
#define DRIVE_MODE_BTN PIN_PC4

// === Buzzer for car status ===
#define BUZZER PIN_PD4

// === BMS HV start failed LED ===
// #define BMS_FAILED_LED PIN_PD4

// === Button active state ===
#define BUTTON_ACTIVE HIGH

// === MCP2515 crystal frequency ===
#define MCP2515_CRYSTAL_FREQ MCP_20MHZ

// end of VCU v3

#endif // USE_VCU_V3

// 3 channel CAN dev board

#ifdef USE_3CH_CAN
// === CAN bus pins ===
#define CS_CAN_MOTOR PIN_PB2
#define CS_CAN_BMS PIN_PB1
#define CS_CAN_DL PIN_PB0

// === APPS and Brake pins ===
#define APPS_5V PIN_PC0
#define APPS_3V3 PIN_PC1
#define BRAKE_IN PIN_PC2
#define HALL_SENSOR PIN_PC3

// VCU brake light
#define BRAKE_LIGHT PIN_PD2

// === Drive mode ===
#define FRG PIN_PD3  // = drive mode LED, soft relay
#define DRIVE_MODE_BTN PIN_PC4

// === Buzzer for car status ===
#define BUZZER PIN_PD4

// === BMS HV start failed LED ===
#define BMS_FAILED_LED PIN_PD5

// === Button active state ===
#define BUTTON_ACTIVE LOW

// === MCP2515 crystal frequency ===
#define MCP2515_CRYSTAL_FREQ MCP_20MHZ
#endif // USE_3CH_CAN

// VCU v2
#ifdef USE_VCU_V2
// === CAN bus pins ===
#define CS_CAN_MOTOR PIN_PB2
#define CS_CAN_BMS PIN_PB1 // unused
#define CS_CAN_DL PIN_PD5  // Datalogger

// === APPS and Brake pins ===
#define APPS_5V PIN_PC0

#define APPS_3V3 PIN_PC1

#define BRAKE_IN PIN_PC2

// VCU brake light
#define BRAKE_LIGHT PIN_PC3

// === Drive mode ===
#define FRG PIN_PB0

#define DRIVE_MODE_BTN PIN_PC4

// === Buzzer for car status ===
#define BUZZER PIN_PD4

// === BMS HV start failed LED ===
// #define BMS_FAILED_LED PIN_PD4

// === Button active state ===
#define BUTTON_ACTIVE HIGH

// === MCP2515 crystal frequency ===
#define MCP2515_CRYSTAL_FREQ MCP_16MHZ
// end of VCU v2

#endif // USE_VCU_V2

#ifdef USE_ARDUINO_PINS

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
#define FRG 4
#define DRIVE_MODE_BTN 5
// === Buzzer for car status ===
#define BUZZER 6

// === BMS HV start failed LED ===
// #define BMS_FAILED_LED PIN_PD4

// === Button active state ===
#define BUTTON_ACTIVE HIGH

// === MCP2515 crystal frequency ===
#define MCP2515_CRYSTAL_FREQ MCP_8MHZ

#endif // USE_ARDUINO_PINS

#define CAN_RATE CAN_500KBPS

#endif // BOARDCONF_H