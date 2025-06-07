#ifndef DEBUG_CAN_H
#define DEBUG_CAN_H

#include <mcp2515.h>
#include "Enums.h"

namespace Debug_CAN {
    // enum CarStatus;
    extern MCP2515* can_interface;

    // Initialize CAN debug
    void initialize(MCP2515* can_interface);
    
    // Specialized messages
    void throttle_in(int pedal_filtered_1, int pedal_filtered_2, int pedal_filtered_final);
    void throttle_out(float throttle_volt, int throttle_torque_val);
    void throttle_fault(Pedal_Fault_Status fault_status, float value);
    void status_car(CarStatus car_status);
    void status_brake(int brake_voltage);
}

#endif // DEBUG_CAN_H