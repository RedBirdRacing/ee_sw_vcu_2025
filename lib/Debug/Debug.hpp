/**
 * @file Debug.hpp
 * @author Planeson, Red Bird Racing
 * @brief Debugging macros and functions for serial and CAN output
 * @version 1.1
 * @date 2026-01-15
 * @see Debug_serial, Debug_can
 * @dir Debug @brief The Debug library contains debugging macros and functions for serial and CAN output, allowing for easy toggling of debug messages and separation of concerns between different types of debug information.
 */

#ifndef DEBUG_HPP
#define DEBUG_HPP

#include "Enums.hpp"

// === Debug Flags ===
#define DEBUG 1                  // if 0, all debug messages are ignored
#define DEBUG_SERIAL (1 && DEBUG) // if 0, all serial debug messages are ignored
#define DEBUG_CAN (1 && DEBUG)    // if 0, all CAN debug messages are ignored

#if DEBUG_SERIAL
#include <Debug_serial.hpp>
#endif

#if DEBUG_CAN
#include <Debug_can.hpp>
#endif

#define DEBUG_THROTTLE (1 && DEBUG)
#define DEBUG_THROTTLE_IN (1 && DEBUG_THROTTLE)
#define DEBUG_THROTTLE_OUT (1 && DEBUG_THROTTLE)
#define DEBUG_THROTTLE_FAULT (1 && DEBUG_THROTTLE)
#define DEBUG_BRAKE (1 && DEBUG)
#define DEBUG_BRAKE_IN (1 && DEBUG_BRAKE)
#define DEBUG_BRAKE_FAULT (1 && DEBUG_BRAKE)
#define DEBUG_GENERAL 1
#define DEBUG_STATUS 1 // Serial only
#define DEBUG_STATUS_CAR (1 && DEBUG_STATUS)
#define DEBUG_STATUS_BRAKE (1 && DEBUG_STATUS)
#define DEBUG_HALL_SENSOR (1 && DEBUG)

// ===== Simple Serial-Only Debug Functions =====

/**
 * @brief Prints a throttle debug message to the serial console.
 * @param x The message to print.
 * @note Serial exclusive
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
 * @note Serial exclusive
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
 * @note Serial exclusive
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
 * @note Serial exclusive
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
 * @note Serial exclusive
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
 * @note Serial exclusive
 */
inline void DBGLN_STATUS(const char *x)
{
#if DEBUG_STATUS && DEBUG_SERIAL
    Debug_Serial::println(x);
#endif
}

/**
 * @brief Sends throttle fault debug info via CAN or serial (if enabled).
 * Overloads for fault status with or without value.
 * @param fault_status The fault status enum.
 * @param value Optional float value for fault.
 */
inline void DBG_THROTTLE_FAULT(PedalFault fault_status, uint16_t value)
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

inline void DBG_THROTTLE_FAULT(PedalFault fault_status)
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
 * @brief Sends BMS debug info via CAN or serial (if enabled).
 * @param BMS_status The BMS status enum.
 */
inline void DBG_BMS_STATUS(BmsStatus BMS_status)
{
#if DEBUG_BRAKE_FAULT && (DEBUG_SERIAL || DEBUG_CAN)
#if DEBUG_SERIAL
    Debug_Serial::status_bms(BMS_status);
#endif
#if DEBUG_CAN
    Debug_CAN::status_bms(BMS_status);
#endif
#endif
}

#endif // DEBUG_HPP