#ifndef DEBUG_CAN_H
#define DEBUG_CAN_H

#include <mcp2515.h>
#include "Enums.h"

static_assert(sizeof(float) == 4, "This code assumes float is 4 bytes");

namespace Debug_CAN {
    // enum CarStatus;
    extern MCP2515* can_interface;

    // Initialize CAN debug
    void initialize(MCP2515* can_interface);
    
    // Specialized messages
    void throttle_in(uint16_t pedal_filtered_1, uint16_t pedal_filtered_2, uint16_t pedal_filtered_final);
    void throttle_out(float throttle_volt, int16_t throttle_torque_val);
    void throttle_fault(Pedal_Fault_Status fault_status, float value);
    void status_car(CarStatus car_status);
    void status_brake(float brake_voltage);
}

#endif // DEBUG_CAN_H