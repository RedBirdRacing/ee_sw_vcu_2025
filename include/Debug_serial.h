#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>
#include <mcp2515.h>
#include "Enums.h"

namespace Debug_Serial {
    // Initialize serial debug
    void initialize();
    
    // Basic print functions
    void print(const char* msg) { Serial.print(msg); }
    void println(const char* msg) { Serial.println(msg); }
    
    // Specialized throttle messages
    void throttle_in(int pedal_filtered_1, int pedal_filtered_2, int pedal_filtered_final);
    void throttle_out(float throttle_volt, int throttle_torque_val);
    void throttle_fault(Pedal_Fault_Status fault_status, float value);
    void status_car(CarStatus car_status);
    void status_brake(int brake_voltage);
    // #endif // DEBUG_SERIAL
}

#endif // DEBUG_SERIAL_H