#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>
#include <mcp2515.h>
#include "Enums.h"

namespace Debug_Serial {
    // Initialize serial debug
    void initialize();
    
    // Basic print functions
    void print(const char* msg);
    void println(const char* msg);
    
    // Specialized throttle messages
    void throttle_in(uint16_t pedal_filtered_1, uint16_t pedal_filtered_2, uint16_t pedal_filtered_final);
    void throttle_out(float throttle_volt, int16_t throttle_torque_val);
    void throttle_fault(Pedal_Fault_Status fault_status, float value);
    void status_car(CarStatus car_status);
    void status_brake(float brake_voltage);
}

#endif // DEBUG_SERIAL_H