#include "Debug_can.h"

#include <mcp2515.h>

MCP2515 *Debug_CAN::can_interface = nullptr;

/**
 * @brief Initializes the Debug_CAN interface with the given MCP2515 CAN controller.
 * 
 * This function sets the can_interface pointer to the provided MCP2515 instance.
 * It should be called before using any other Debug_CAN functions.
 * 
 * @param can Pointer to the MCP2515 CAN controller instance.
 * @return None
 */
void Debug_CAN::initialize(MCP2515 *can)
{
    can_interface = can;
}

/**
 * @brief Sends a debug throttle input message over CAN.
 * This function prepares a CAN frame with the throttle input values and sends it.
 * 
 * @param pedal_filtered_1 Filtered value from pedal sensor 1.
 * @param pedal_filtered_2 Filtered value from pedal sensor 2.
 * @param pedal_2_scaled Scaled value of pedal sensor 2.
 * @return None
 */
void Debug_CAN::throttle_in(uint16_t pedal_filtered_1, uint16_t pedal_filtered_2, uint16_t pedal_2_scaled)
{
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
    tx_msg.data[4] = pedal_2_scaled & 0xFF;
    tx_msg.data[5] = (pedal_2_scaled >> 8) & 0xFF; // Upper byte

    can_interface->sendMessage(&tx_msg);
}

/**
 * @brief Sends a debug throttle output message over CAN.
 * This function prepares a CAN frame with the throttle output values and sends it.
 * 
 * @param throttle_final Final value of the throttle pedal.
 * @param throttle_torque_val Calculated torque value based on the throttle input.
 * @return None
 */
void Debug_CAN::throttle_out(uint16_t throttle_final, int16_t throttle_torque_val)
{
    if (!can_interface)
        return;

    can_frame tx_msg;
    tx_msg.can_id = THROTTLE_OUT_MSG;
    tx_msg.can_dlc = 4;

    // Little-endian format for int16_t value
    tx_msg.data[0] = throttle_final & 0xFF;
    tx_msg.data[1] = (throttle_final >> 8) & 0xFF; // Upper byte
    tx_msg.data[2] = throttle_torque_val & 0xFF;
    tx_msg.data[3] = (throttle_torque_val >> 8) & 0xFF; // Upper byte

    can_interface->sendMessage(&tx_msg);
}

/**
 * @brief Sends a debug throttle fault message over CAN.
 * This function prepares a CAN frame with the throttle fault status and sends it.
 * 
 * @param fault_status The status of the throttle fault as defined in pedal_fault_status enum.
 * @param value Optional float value associated with the fault (e.g., pedal voltage).
 * @return None
 */
void Debug_CAN::throttle_fault(pedal_fault_status fault_status, uint16_t value)
{
    if (!can_interface)
        return;

    can_frame tx_msg;
    tx_msg.can_id = THROTTLE_FAULT_MSG;
    tx_msg.can_dlc = 5;

    tx_msg.data[0] = static_cast<uint8_t>(fault_status); // Convert enum to uint8_t

    tx_msg.data[1] = value & 0xFF;
    tx_msg.data[2] = (value >> 8) & 0xFF; // Upper byte

    can_interface->sendMessage(&tx_msg);
}

/**
 * @brief Sends a debug throttle fault message over CAN without a float value.
 * This function prepares a CAN frame with the throttle fault status and sends it.
 * 
 * @param fault_status The status of the throttle fault as defined in pedal_fault_status enum.
 * @return None
 */
void Debug_CAN::throttle_fault(pedal_fault_status fault_status)
{
    if (!can_interface)
        return;

    can_frame tx_msg;
    tx_msg.can_id = THROTTLE_FAULT_MSG;
    tx_msg.can_dlc = 1;

    tx_msg.data[0] = static_cast<uint8_t>(fault_status); // Convert enum to uint8_t

    can_interface->sendMessage(&tx_msg);
}

/**
 * @brief Sends a debug brake fault message over CAN.
 * This function prepares a CAN frame with the brake fault status and sends it.
 * 
 * @param fault_status The status of the brake fault as defined in pedal_fault_status enum.
 * @param value Optional float value associated with the fault (e.g., brake voltage).
 * @return None
 */
void Debug_CAN::brake_fault(pedal_fault_status fault_status, uint16_t value)
{
    if (!can_interface)
        return;

    can_frame tx_msg;
    tx_msg.can_id = THROTTLE_FAULT_MSG;
    tx_msg.can_dlc = 5;

    tx_msg.data[0] = static_cast<uint8_t>(fault_status); // Convert enum to uint8_t

    tx_msg.data[1] = value & 0xFF;
    tx_msg.data[2] = (value >> 8) & 0xFF; // Upper byte

    can_interface->sendMessage(&tx_msg);
}

/**
 * @brief Sends a debug car status message over CAN.
 * This function prepares a CAN frame with the current car status and sends it.
 * 
 * @param car_status The current status of the car as defined in main_car_status enum.
 * @return None
 */
void Debug_CAN::status_car(main_car_status car_status)
{
    if (!can_interface)
        return;

    can_frame tx_msg;
    tx_msg.can_id = STATUS_CAR_MSG;
    tx_msg.can_dlc = 1;

    tx_msg.data[0] = static_cast<uint8_t>(car_status); // Convert enum to uint8_t

    can_interface->sendMessage(&tx_msg);
}
 
/**
 * @brief Sends a debug car status change message over CAN.
 * This function prepares a CAN frame with the current car status change and sends it.
 * 
 * @param status_change The state change of the car as defined in state_changes enum.
 * @return None
 */
void Debug_CAN::status_car_change(state_changes status_change)
{
    if (!can_interface)
        return;

    can_frame tx_msg;
    tx_msg.can_id = STATUS_CAR_CHANGE_MSG;
    tx_msg.can_dlc = 1;

    tx_msg.data[0] = static_cast<uint8_t>(status_change); // Convert enum to uint8_t

    can_interface->sendMessage(&tx_msg);
}