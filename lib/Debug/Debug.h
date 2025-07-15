#ifndef DEBUG_H
#define DEBUG_H

#include "Enums.h"

// === Debug Flags ===


#define DEBUG true // Disables all debug messages if false
/**
 * @brief Enables serial debug messages if true.
 * @note Always leave false for GitHub.
 */
#define DEBUG_SERIAL true && DEBUG
/**
 * @brief Enables CAN debug messages if true.
 */
#define DEBUG_CAN true && DEBUG

#if DEBUG_SERIAL
#include <Debug_serial.h>
#endif

#if DEBUG_CAN
#include <Debug_can.h>
#endif

// Throttle pedal input values, fault detection, processed output for motor

/**
 * @brief Enables throttle pedal input debug messages.
 * @note Serial only.
 */
#define DEBUG_THROTTLE true

/**
 * @brief Enables overall throttle debug messages.
 * @note Serial only.
 */
#define DEBUG_THROTTLE_OVERALL true

/**
 * @brief Enables throttle input debug messages.
 */
#define DEBUG_THROTTLE_IN true && DEBUG_THROTTLE_OVERALL && DEBUG_CAN

/**
 * @brief Enables throttle output debug messages.
 */
#define DEBUG_THROTTLE_OUT true && DEBUG_THROTTLE_OVERALL && DEBUG_CAN

/**
 * @brief Enables throttle fault debug messages.
 */
#define DEBUG_THROTTLE_FAULT true && DEBUG_THROTTLE_OVERALL && DEBUG_CAN

/**
 * @brief Enables general debug messages.
 * @note Serial only.
 */
#define DEBUG_GENERAL true

/**
 * @brief Enables car status, brake pedal, and other sensor input debug messages (serial only).
 */
#define DEBUG_STATUS true // Serial only

/**
 * @brief Enables car status debug messages (CAN).
 */
#define DEBUG_STATUS_CAR true && DEBUG_STATUS && DEBUG_CAN

/**
 * @brief Enables brake status debug messages (CAN).
 */
#define DEBUG_STATUS_BRAKE true && DEBUG_STATUS && DEBUG_CAN

// === Unified Debug Macros ===
// These will automatically route to the configured output(s)

// ===== Simple Serial-Only Macros =====
#if DEBUG_THROTTLE && DEBUG_SERIAL
/**
 * @brief Prints a throttle debug message to the serial console.
 * @param x The message to print.
 */
#define DBG_THROTTLE(x)    Debug_Serial::print(x)
/**
 * @brief Prints a line to the serial console for throttle debug.
 * @param x The message to print.
 */
#define DBGLN_THROTTLE(x)  Debug_Serial::println(x)
#else
#define DBG_THROTTLE(x)
#define DBGLN_THROTTLE(x)
#endif

#if DEBUG_GENERAL && DEBUG_SERIAL
/**
 * @brief Prints a general debug message to the serial console.
 * @param x The message to print.
 */
#define DBG_GENERAL(x)    Debug_Serial::print(x)
/**
 * @brief Prints a line to the serial console for general debug.
 * @param x The message to print.
 */
#define DBGLN_GENERAL(x)  Debug_Serial::println(x)
#else
#define DBG_GENERAL(x)
#define DBGLN_GENERAL(x)
#endif

#if DEBUG_STATUS && DEBUG_SERIAL
/**
 * @brief Prints a status message to the serial console.
 * @param x The message to print.
 */
#define DBG_STATUS(x)    Debug_Serial::print(x)
/**
 * @brief Prints a line to the serial console for status debug.
 * @param x The message to print.
 */
#define DBGLN_STATUS(x)  Debug_Serial::println(x)
#else
#define DBG_STATUS(x)
#define DBGLN_STATUS(x)
#endif

// ===== Specialized Message Macros =====

#if DEBUG_THROTTLE_IN && (DEBUG_SERIAL || DEBUG_CAN)
/**
 * @brief Sends throttle pedal input debug info via CAN or serial (if enabled).
 * @param pedal_filtered_1 Filtered value from pedal 1.
 * @param pedal_filtered_2 Filtered value from pedal 2.
 * @param pedal_filtered_final Final filtered value.
 */
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
/**
 * @brief Sends throttle output debug info via CAN or serial (if enabled).
 * @param throttle_volt Final throttle value.
 * @param throttle_torque_val Output torque value.
 */
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

/**
 * @brief Sends throttle fault debug info via CAN or serial (if enabled).
 * Overloads for fault status with or without value.
 * @param fault_status The fault status enum.
 * @param value Optional float value for fault.
 */
#if DEBUG_THROTTLE_FAULT && (DEBUG_SERIAL || DEBUG_CAN)
// Overload: with float value
// DIFF_CONTINUING, THROTTLE_LOW, THROTTLE_HIGH
inline void DBG_THROTTLE_FAULT(pedal_fault_status fault_status, float value) {
    #if DEBUG_SERIAL
        Debug_Serial::throttle_fault(fault_status, value);
    #endif
    #if DEBUG_CAN
        Debug_CAN::throttle_fault(fault_status, value);
    #endif
}
// Overload: no float value
// DIFF_STARTED, DIFF_EXCEED_100MS, DIFF_RESOLVED
inline void DBG_THROTTLE_FAULT(pedal_fault_status fault_status) {
    #if DEBUG_SERIAL
        Debug_Serial::throttle_fault(fault_status);
    #endif
    #if DEBUG_CAN
        Debug_CAN::throttle_fault(fault_status);
    #endif
}
#else
#define DBG_THROTTLE_FAULT(...)
#endif

/**
 * @brief Sends car status debug info via CAN or serial (if enabled).
 * @param car_status Car status enum value.
 */
#if DEBUG_STATUS_CAR && (DEBUG_SERIAL || DEBUG_CAN)
inline void DBG_STATUS_CAR(main_car_status car_status) {
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

/**
 * @brief Sends car status change debug info via CAN or serial (if enabled).
 * @param status_change Status change enum value.
 */
#if DEBUG_STATUS_CAR && (DEBUG_SERIAL || DEBUG_CAN)
inline void DBG_STATUS_CAR_CHANGE(state_changes status_change) {
    #if DEBUG_SERIAL
        Debug_Serial::status_car_change(status_change);
    #endif
    #if DEBUG_CAN
        Debug_CAN::status_car_change(status_change);
    #endif
}
#else
#define DBG_STATUS_CAR(car_status)
#endif

/**
 * @brief Sends brake status debug info via CAN or serial (if enabled).
 * @param brake_voltage Brake pedal voltage.
 */
#if DEBUG_STATUS_BRAKE && (DEBUG_SERIAL || DEBUG_CAN)
inline void DBG_STATUS_BRAKE(uint16_t brake_voltage) {
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
