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
#define DBG_THROTTLE(x)    DebugSerial::print(x)
#define DBGLN_THROTTLE(x)  DebugSerial::println(x)
#else
#define DBG_THROTTLE(x)
#define DBGLN_THROTTLE(x)
#endif

#if DEBUG_GENERAL && DEBUG_SERIAL
#define DBG_GENERAL(x)    DebugSerial::print(x)
#define DBGLN_GENERAL(x)  DebugSerial::println(x)
#else
#define DBG_GENERAL(x)
#define DBGLN_GENERAL(x)
#endif

#if DEBUG_STATUS && DEBUG_SERIAL
#define DBG_STATUS(x)    DebugSerial::print(x)
#define DBGLN_STATUS(x)  DebugSerial::println(x)
#else
#define DBG_STATUS(x)
#define DBGLN_STATUS(x)
#endif

// ===== Specialized Message Macros =====
#if DEBUG_THROTTLE_IN && (DEBUG_SERIAL || DEBUG_CAN)
inline void DBG_THROTTLE_IN(int pedal_filtered_1, int pedal_filtered_2, int pedal_filtered_final) {
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
inline void DBG_THROTTLE_OUT(float throttle_volt, int throttle_torque_val) {
    #if DEBUG_SERIAL
        Debug_Serial::throttle_out(throttle_volt, throttle_torque_val);
    #endif
    #if DEBUG_CAN
        Debug_CAN::throttle_out(throttle_volt, throttle_torque_val);
    #endif
}
#else
#define DBG_THROTTLE_OUT(throttle_volt, throttle_torque_val)
#endif

#if DEBUG_THROTTLE_FAULT && (DEBUG_SERIAL || DEBUG_CAN)
inline void DBG_THROTTLE_FAULT(Pedal_Fault_Status fault_status, float value) {
    #if DEBUG_SERIAL
        Debug_Serial::throttle_fault(fault_status, value);
    #endif
    #if DEBUG_CAN
        Debug_CAN::throttle_fault(fault_status, value);
    #endif
}
#else
#define DBG_THROTTLE_FAULT(percentage_difference)
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
inline void DBG_STATUS_BRAKE(int brake_voltage) {
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
