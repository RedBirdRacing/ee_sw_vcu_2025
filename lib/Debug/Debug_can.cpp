#include "Debug_can.h"

#include <mcp2515.h>

// Definition of the static member
MCP2515* Debug_CAN::can_interface = nullptr;  // <-- This remains the same

void Debug_CAN::initialize(MCP2515* can) {
    can_interface = can;
}

void Debug_CAN::throttle_in(uint16_t pedal_filtered_1, uint16_t pedal_filtered_2, uint16_t pedal_filtered_final) {
    if (!can_interface) 
        return;
    
    can_frame tx_msg;
    tx_msg.can_id = THROTTLE_IN_MSG;
    tx_msg.can_dlc = 6;

    // Little-endian format for uint16_t values
    tx_msg.data[0] = pedal_filtered_1 & 0xFF;
    tx_msg.data[1] = (pedal_filtered_1 >> 8) & 0xFF; // Upper byte
    tx_msg.data[2] = pedal_filtered_2 & 0xFF;
    tx_msg.data[3] = (pedal_filtered_2 >> 8) & 0xFF; // Upper byte
    tx_msg.data[4] = pedal_filtered_final & 0xFF;
    tx_msg.data[5] = (pedal_filtered_final >> 8) & 0xFF; // Upper byte

    can_interface->sendMessage(&tx_msg);
}

void Debug_CAN::throttle_out(float throttle_volt, int16_t throttle_torque_val) {
    if (!can_interface) 
        return;

    can_frame tx_msg;
    tx_msg.can_id = THROTTLE_OUT_MSG;
    tx_msg.can_dlc = 6;

    memcpy(&tx_msg.data[0], &throttle_volt, sizeof(float));

    // Little-endian format for int16_t value
    tx_msg.data[4] = throttle_torque_val & 0xFF;
    tx_msg.data[5] = (throttle_torque_val >> 8) & 0xFF; // Upper byte

    can_interface->sendMessage(&tx_msg);
}

void Debug_CAN::throttle_fault(Pedal_Fault_Status fault_status, float value) {
    if (!can_interface) 
        return;

    can_frame tx_msg;
    tx_msg.can_id = THROTTLE_FAULT_MSG;
    tx_msg.can_dlc = 5;

    tx_msg.data[0] = static_cast<uint8_t>(fault_status); // Convert enum to uint8_t

    memcpy(&tx_msg.data[1], &value, sizeof(float)); // Copy float value

    can_interface->sendMessage(&tx_msg);
}

void Debug_CAN::status_car(CarStatus car_status) {
    if (!can_interface) 
        return;

    can_frame tx_msg;
    tx_msg.can_id = STATUS_CAR_MSG;
    tx_msg.can_dlc = 1;

    tx_msg.data[0] = static_cast<uint8_t>(car_status); // Convert enum to uint8_t

    can_interface->sendMessage(&tx_msg);
}