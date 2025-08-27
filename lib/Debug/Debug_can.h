#ifndef DEBUG_CAN_H
#define DEBUG_CAN_H

// ignore -Wpedantic warnings for mcp2515.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mcp2515.h>
#pragma GCC diagnostic pop

#include "Enums.h"

// probably can remove since no float is used now, but can @Ariel help me check it?
static_assert(sizeof(float) == 4, "This code assumes float is 4 bytes");

namespace Debug_CAN
{
    // enum CarStatus;
    
    extern MCP2515 *can_interface;


    // Initialize CAN debug

    void initialize(MCP2515 *can_interface);


    // Specialized messages

    void throttle_in(uint16_t pedal_filtered_1, uint16_t pedal_filtered_2, uint16_t pedal_2_scaled);
    void throttle_out(uint16_t throttle_final, int16_t throttle_torque_val);
    void throttle_fault(pedal_fault_status fault_status, uint16_t value);
    void throttle_fault(pedal_fault_status fault_status);
    void brake_fault(pedal_fault_status fault_status, uint16_t value);
    void status_car(main_car_status car_status);
    void status_car_change(state_changes status_change);
    void status_brake(uint16_t brake_voltage);
    void status_bms(BMS_status BMS_status);
}

#endif // DEBUG_CAN_H