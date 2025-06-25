#ifndef DEBUG_H
#define DEBUG_H

#include "Enums.h"

// === Debug Flags ===
#define DEBUG true // Overall debug functionality

// ALWAYS LEAVE FALSE FOR GITHUB
#define DEBUG_SERIAL false && DEBUG // Sends Serial debug messages if enabled
#if DEBUG_SERIAL
#include <Debug_serial.h>
#endif

#define DEBUG_CAN true && DEBUG // Sends CAN debug messages if enabled
#if DEBUG_CAN
#include <Debug_can.h>
#endif

// Throttle pedal input values, fault detection, processed output for motor
#define DEBUG_THROTTLE true // Serial only
#define DEBUG_THROTTLE_IN true && DEBUG_THROTTLE && DEBUG_CAN
#define DEBUG_THROTTLE_OUT true && DEBUG_THROTTLE && DEBUG_CAN
#define DEBUG_THROTTLE_FAULT true && DEBUG_THROTTLE && DEBUG_CAN

// General debug messages, won't be sent through CAN
#define DEBUG_GENERAL true // Serial only

// Car status, brake pedal, other sensor inputs
#define DEBUG_STATUS true // Serial only
#define DEBUG_STATUS_CAR true && DEBUG_STATUS && DEBUG_CAN
#define DEBUG_STATUS_BRAKE true && DEBUG_STATUS && DEBUG_CAN

// === Unified Debug Macros ===
// These will automatically route to the configured output(s)

// ===== Simple Serial-Only Macros =====
#if DEBUG_THROTTLE && DEBUG_SERIAL
#define DBG_THROTTLE(x)    Debug_Serial::print(x)
#define DBGLN_THROTTLE(x)  Debug_Serial::println(x)
#else
#define DBG_THROTTLE(x)
#define DBGLN_THROTTLE(x)
#endif

#if DEBUG_GENERAL && DEBUG_SERIAL
#define DBG_GENERAL(x)    Debug_Serial::print(x)
#define DBGLN_GENERAL(x)  Debug_Serial::println(x)
#else
#define DBG_GENERAL(x)
#define DBGLN_GENERAL(x)
#endif

#if DEBUG_STATUS && DEBUG_SERIAL
#define DBG_STATUS(x)    Debug_Serial::print(x)
#define DBGLN_STATUS(x)  Debug_Serial::println(x)
#else
#define DBG_STATUS(x)
#define DBGLN_STATUS(x)
#endif

// ===== Specialized Message Macros =====
#if DEBUG_THROTTLE_IN && (DEBUG_SERIAL || DEBUG_CAN)
inline void DBG_THROTTLE_IN(uint16_t pedal_filtered_1, uint16_t pedal_filtered_2, uint16_t pedal_filtered_final) {
    #if DEBUG_SERIAL
        Debug_Serial::throttle_in(pedal_filtered_1, pedal_filtered_2, pedal_filtered_final);
    #endif
    #if DEBUG_CAN
        Debug_CAN::throttle_in(pedal_filtered_1, pedal_filtered_2, pedal_filtered_final);
    #endif
}
#else
#define DBG_THROTTLE_IN(pedal_filtered_1, pedal_filtered_2, pedal_filtered_final)
#endif

#if DEBUG_THROTTLE_OUT && (DEBUG_SERIAL || DEBUG_CAN)
inline void DBG_THROTTLE_OUT(uint16_t throttle_final, int16_t throttle_torque_val) {
    #if DEBUG_SERIAL
        Debug_Serial::throttle_out(throttle_final, throttle_torque_val);
    #endif
    #if DEBUG_CAN
        Debug_CAN::throttle_out(throttle_final, throttle_torque_val);
    #endif
}
#else
#define DBG_THROTTLE_OUT(throttle_volt, throttle_torque_val)
#endif

#if DEBUG_THROTTLE_FAULT && (DEBUG_SERIAL || DEBUG_CAN)
// Overload: with float value
// DIFF_FAULT_CONTINUING, THROTTLE_TOO_LOW, THROTTLE_TOO_HIGH
inline void DBG_THROTTLE_FAULT(Pedal_Fault_Status fault_status, float value) {
    #if DEBUG_SERIAL
        Debug_Serial::throttle_fault(fault_status, value);
    #endif
    #if DEBUG_CAN
        Debug_CAN::throttle_fault(fault_status, value);
    #endif
}
// Overload: no float value
// DIFF_FAULT_JUST_STARTED, DIFF_FAULT_EXCEED_100MS, DIFF_FAULT_RESOLVED
inline void DBG_THROTTLE_FAULT(Pedal_Fault_Status fault_status) {
    #if DEBUG_SERIAL
        Debug_Serial::throttle_fault(fault_status);
    #endif
    #if DEBUG_CAN
        Debug_CAN::throttle_fault(fault_status);
    #endif
}
#else
#define DBG_THROTTLE_FAULT(fault_status, value)
#define DBG_THROTTLE_FAULT(fault_status)
#endif

#if DEBUG_STATUS_CAR && (DEBUG_SERIAL || DEBUG_CAN)
inline void DBG_STATUS_CAR(CarStatus car_status) {
    #if DEBUG_SERIAL
        Debug_Serial::status_car(car_status);
    #endif
    #if DEBUG_CAN
        Debug_CAN::status_car(car_status);
    #endif
}
#else
#define DBG_STATUS_CAR(car_status)
#endif

#if DEBUG_STATUS_BRAKE && (DEBUG_SERIAL || DEBUG_CAN)
inline void DBG_STATUS_BRAKE(float brake_voltage) {
    #if DEBUG_SERIAL
        Debug_Serial::status_brake(brake_voltage);
    #endif
    #if DEBUG_CAN
        Debug_CAN::status_brake(brake_voltage);
    #endif
}
#else
#define DBG_STATUS_BRAKE(brake_voltage)
#endif

#endif // DEBUG_H
