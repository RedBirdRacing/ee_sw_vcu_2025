/**
 * @file Debug_can.cpp
 * @author Planeson, Red Bird Racing
 * @brief Implementation of the Debug_CAN namespace for CAN debugging functions
 * @version 1.1
 * @date 2026-01-14
 * @see Debug_can.h
 */

#include "Debug_can.hpp"

// ignore -Wpedantic warnings for mcp2515.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mcp2515.h>
#pragma GCC diagnostic pop

MCP2515 *Debug_CAN::can_interface = nullptr;

/**
 * @brief Initializes the Debug_CAN interface.
 * It should be called before using any other Debug_CAN functions.
 * 
 * @param can Pointer to the MCP2515 CAN controller instance.
 */
void Debug_CAN::initialize(MCP2515 *can)
{
    if (can == nullptr)
        return;
    can_interface = can;
}

/**
 * @brief Sends a debug throttle fault message over CAN with a float value.
 * 
 * @param fault_status The status of the throttle fault as defined in PedalFault enum.
 * @param value Optional uint16_t value associated with the fault (e.g., pedal voltage).
 */
void Debug_CAN::throttle_fault(PedalFault fault_status, uint16_t value)
{
    if (!can_interface)
        return;

    can_frame tx_msg;
    tx_msg.can_id = THROTTLE_FAULT_MSG;
    tx_msg.can_dlc = 3;

    tx_msg.data[0] = static_cast<uint8_t>(fault_status); // Convert enum to uint8_t

    tx_msg.data[1] = value & 0xFF;
    tx_msg.data[2] = (value >> 8) & 0xFF; // Upper byte

    can_interface->sendMessage(&tx_msg);
}

/**
 * @brief Sends a debug throttle fault message over CAN without a float value.
 * 
 * @param fault_status The status of the throttle fault as defined in PedalFault enum.
 */
void Debug_CAN::throttle_fault(PedalFault fault_status)
{
    if (!can_interface)
        return;

    can_frame tx_msg;
    tx_msg.can_id = THROTTLE_FAULT_MSG;
    tx_msg.can_dlc = 1;

    tx_msg.data[0] = static_cast<uint8_t>(fault_status); // Convert enum to uint8_t

    can_interface->sendMessage(&tx_msg);
}

void Debug_CAN::general(canid_t id,
                        uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
                        uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
    if (!can_interface) return;

    can_frame msg;
    msg.can_id = id;
    msg.can_dlc = 8;
    msg.data[0] = d0; msg.data[1] = d1; msg.data[2] = d2; msg.data[3] = d3;
    msg.data[4] = d4; msg.data[5] = d5; msg.data[6] = d6; msg.data[7] = d7;

    can_interface->sendMessage(&msg);
}