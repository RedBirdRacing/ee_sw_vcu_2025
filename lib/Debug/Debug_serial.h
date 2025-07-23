#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>
#include "Enums.h"

namespace Debug_Serial {
    // Initialize serial debug
    void initialize();
    
    // Basic print functions
    void print(const char* msg);
    void println(const char* msg);
    
    // Specialized throttle messages
    void throttle_in(uint16_t pedal_filtered_1, uint16_t pedal_filtered_2, uint16_t pedal_filtered_final);
    void throttle_out(uint16_t throttle_final, int16_t throttle_torque_val);
    void throttle_fault(pedal_fault_status fault_status, uint16_t value);
    void throttle_fault(pedal_fault_status fault_status);
    void brake_fault(pedal_fault_status fault_status, uint16_t value);
    void status_car(main_car_status car_status);
    void status_car_change(state_changes status_change);
    void status_brake(uint16_t brake_voltage);
}

#endif // DEBUG_SERIAL_H