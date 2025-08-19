#ifndef DEBUG_H
#define DEBUG_H

#include "Enums.h"

// === Debug Flags ===
#define DEBUG true                 // if false, all debug messages are ignored
#define DEBUG_SERIAL true && DEBUG // if false, all serial debug messages are ignored
#define DEBUG_CAN true && DEBUG    // if false, all CAN debug messages are ignored

#if DEBUG_SERIAL
#include <Debug_serial.h>
#endif

#if DEBUG_CAN
#include <Debug_can.h>
#endif

#define DEBUG_THROTTLE true && DEBUG
#define DEBUG_THROTTLE_IN true && DEBUG_THROTTLE
#define DEBUG_THROTTLE_OUT true && DEBUG_THROTTLE
#define DEBUG_THROTTLE_FAULT true && DEBUG_THROTTLE
#define DEBUG_BRAKE true && DEBUG
#define DEBUG_BRAKE_IN true && DEBUG_BRAKE
#define DEBUG_BRAKE_FAULT true && DEBUG_BRAKE
#define DEBUG_GENERAL true
#define DEBUG_STATUS true // Serial only
#define DEBUG_STATUS_CAR true && DEBUG_STATUS
#define DEBUG_STATUS_BRAKE true && DEBUG_STATUS

// ===== Simple Serial-Only Debug Functions =====

/** 
 * @brief Prints a throttle debug message to the serial console.
 * @param x The message to print.
 */
inline void DBG_THROTTLE(const char *x)
{
#if DEBUG_THROTTLE && DEBUG_SERIAL
    Debug_Serial::print(x);
#endif
}

/**
 * @brief Prints a line to the serial console for throttle debug.
 * @param x The message to print.
 */
inline void DBGLN_THROTTLE(const char *x)
{
#if DEBUG_THROTTLE && DEBUG_SERIAL
    Debug_Serial::println(x);
#endif
}

/**
 * @brief Prints a general debug message to the serial console.
 * @param x The message to print.
 */
inline void DBG_GENERAL(const char *x)
{
#if DEBUG_GENERAL && DEBUG_SERIAL
    Debug_Serial::print(x);
#endif
}

/**
 * @brief Prints a line to the serial console for general debug.
 * @param x The message to print.
 */
inline void DBGLN_GENERAL(const char *x)
{
#if DEBUG_GENERAL && DEBUG_SERIAL
    Debug_Serial::println(x);
#endif
}

/**
 * @brief Prints a status message to the serial console.
 * @param x The message to print.
 */
inline void DBG_STATUS(const char *x)
{
#if DEBUG_STATUS && DEBUG_SERIAL
    Debug_Serial::print(x);
#endif
}

/**
 * @brief Prints a line to the serial console for status debug.
 * @param x The message to print.
 */
inline void DBGLN_STATUS(const char *x)
{
#if DEBUG_STATUS && DEBUG_SERIAL
    Debug_Serial::println(x);
#endif
}

// ===== Specialized Message Debug Functions =====

/**
 * @brief Sends throttle pedal input debug info via CAN or serial (if enabled).
 * @param pedal_filtered_1 Filtered value from pedal 1.
 * @param pedal_filtered_2 Filtered value from pedal 2.
 * @param pedal_filtered_final Final filtered value.
 */
inline void DBG_THROTTLE_IN(uint16_t pedal_filtered_1, uint16_t pedal_filtered_2, uint16_t pedal_filtered_final)
{
#if DEBUG_THROTTLE_IN && (DEBUG_SERIAL || DEBUG_CAN)
#if DEBUG_SERIAL
    Debug_Serial::throttle_in(pedal_filtered_1, pedal_filtered_2, pedal_filtered_final);
#endif
#if DEBUG_CAN
    Debug_CAN::throttle_in(pedal_filtered_1, pedal_filtered_2, pedal_filtered_final);
#endif
#endif
}

/**
 * @brief Sends throttle output debug info via CAN or serial (if enabled).
 * @param throttle_final Final throttle value.
 * @param throttle_torque_val Output torque value.
 */
inline void DBG_THROTTLE_OUT(uint16_t throttle_final, int16_t throttle_torque_val)
{
#if DEBUG_THROTTLE_OUT && (DEBUG_SERIAL || DEBUG_CAN)
#if DEBUG_SERIAL
    Debug_Serial::throttle_out(throttle_final, throttle_torque_val);
#endif
#if DEBUG_CAN
    Debug_CAN::throttle_out(throttle_final, throttle_torque_val);
#endif
#endif
}

/**
 * @brief Sends throttle fault debug info via CAN or serial (if enabled).
 * Overloads for fault status with or without value.
 * @param fault_status The fault status enum.
 * @param value Optional float value for fault.
 */
inline void DBG_THROTTLE_FAULT(pedal_fault_status fault_status, uint16_t value)
{
#if DEBUG_THROTTLE_FAULT && (DEBUG_SERIAL || DEBUG_CAN)
#if DEBUG_SERIAL
    Debug_Serial::throttle_fault(fault_status, value);
#endif
#if DEBUG_CAN
    Debug_CAN::throttle_fault(fault_status, value);
#endif
#endif
}

inline void DBG_THROTTLE_FAULT(pedal_fault_status fault_status)
{
#if DEBUG_THROTTLE_FAULT && (DEBUG_SERIAL || DEBUG_CAN)
#if DEBUG_SERIAL
    Debug_Serial::throttle_fault(fault_status);
#endif
#if DEBUG_CAN
    Debug_CAN::throttle_fault(fault_status);
#endif
#endif
}

/**
 * @brief Sends car status debug info via CAN or serial (if enabled).
 * @param car_status Car status enum value.
 */
inline void DBG_STATUS_CAR(main_car_status car_status)
{
#if DEBUG_STATUS_CAR && (DEBUG_SERIAL || DEBUG_CAN)
#if DEBUG_SERIAL
    Debug_Serial::status_car(car_status);
#endif
#if DEBUG_CAN
    Debug_CAN::status_car(car_status);
#endif
#endif
}

/**
 * @brief Sends car status change debug info via CAN or serial (if enabled).
 * @param status_change Status change enum value.
 */
inline void DBG_STATUS_CAR_CHANGE(state_changes status_change)
{
#if DEBUG_STATUS_CAR && (DEBUG_SERIAL || DEBUG_CAN)
#if DEBUG_SERIAL
    Debug_Serial::status_car_change(status_change);
#endif
#if DEBUG_CAN
    Debug_CAN::status_car_change(status_change);
#endif
#endif
}

/**
 * @brief Sends brake fault debug info via CAN or serial (if enabled).
 * Overloads for fault status with or without value.
 * @param fault_status The fault status enum.
 * @param value Optional float value for fault.
 */
inline void DBG_BRAKE_FAULT(pedal_fault_status fault_status, uint16_t value)
{
#if DEBUG_BRAKE_FAULT && (DEBUG_SERIAL || DEBUG_CAN)
#if DEBUG_SERIAL
    Debug_Serial::brake_fault(fault_status, value);
#endif
#if DEBUG_CAN
    Debug_CAN::brake_fault(fault_status, value);
#endif
#endif
}

#endif // DEBUG_H