#ifndef DEBUG_H
#define DEBUG_H

// === Debug Flags ===
#define DEBUG true // Overall debug functionality

// ALWAYS LEAVE FALSE FOR GITHUB
#define DEBUG_SERIAL false && DEBUG // Sends Serial debug messages if enabled

#define DEBUG_CAN true && DEBUG // Sends CAN debug messages if enabled

#define DEBUG_PEDAL true && DEBUG
#define DEBUG_SIGNAL_PROC false && DEBUG
#define DEBUG_GENERAL true && DEBUG
#define DEBUG_PEDAL true && DEBUG
#define DEBUG_CAN true && DEBUG
#define DEBUG_STATUS true && DEBUG

// === Include other Debug Header Files ===
#if DEBUG_SERIAL
#include <Debug_serial.h>
#endif // DEBUG_SERIAL

#if DEBUG_CAN
#define DBC true // Debug CAN bus functionality, independent of DEBUG
#include <Debug_can.h>
#endif // DEBUG_CAN

#endif // DEBUG_H
