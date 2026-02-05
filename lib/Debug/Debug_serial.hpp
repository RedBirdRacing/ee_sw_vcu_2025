/**
 * @file Debug_serial.hpp
 * @author Planeson, Red Bird Racing
 * @brief Declaration of the Debug_Serial namespace for serial debugging functions
 * @version 1.1
 * @date 2026-01-14
 * @see Debug_serial.cpp
 */

#ifndef DEBUG_SERIAL_HPP
#define DEBUG_SERIAL_HPP

#include <Arduino.h>
#include "Enums.hpp"

/**
 * @brief Namespace for serial debugging functions
 */
namespace Debug_Serial {
    void initialize();

    void print(const char* msg);
    void println(const char* msg);
    
    void throttle_in(uint16_t pedal_filtered_1, uint16_t pedal_filtered_2, uint16_t pedal_filtered_final, uint16_t brake);
    void throttle_out(uint16_t throttle_final, int16_t throttle_torque_val);
    void throttle_fault(PedalFault fault_status, uint16_t value);
    void throttle_fault(PedalFault fault_status);
    void brake_fault(PedalFault fault_status, uint16_t value);
    void status_car(CarStatus car_status);
    void status_brake(uint16_t brake_voltage);
    void status_bms(BmsStatus BMS_status);
    void hall_sensor(uint16_t hall_sensor_value);
}

#endif // DEBUG_SERIAL_HPP