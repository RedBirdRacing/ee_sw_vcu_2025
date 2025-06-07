#include "Debug_can.h"

#include <mcp2515.h>

// Definition of the static member
MCP2515* Debug_CAN::can_interface = nullptr;  // <-- This remains the same

void Debug_CAN::initialize(MCP2515* can) {
    can_interface = can;
}