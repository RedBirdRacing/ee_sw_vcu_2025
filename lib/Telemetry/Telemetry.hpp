/**
 * @file Telemetry.hpp
 * @author Planeson, Red Bird Racing
 * @brief Declaration of the Telemetry class for sending telemetry data over CAN bus
 * @version 1.0
 * @date 2026-01-26
 * @see Telemetry.cpp
 * @dir lib/Telemetry @brief The Telemetry library contains the Telemetry class for managing telemetry data transmission over CAN bus, including grabbing and sending telemetry frames in fixed order based on scheduling logic.
 */

#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include "CarState.hpp"

// ignore -Wpedantic warnings for mcp2515.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mcp2515.h>
#pragma GCC diagnostic pop

/**
 * @brief Telemetry class for managing telemetry data transmission over CAN bus
 * Grabs and sends telemetry frames in fixed order based on scheduling logic
 */
class Telemetry
{
public:
    Telemetry(MCP2515 &mcp2515_, CarState &car_);
    void sendPedal();
    void sendMotor();
    void sendBms();

private:
    MCP2515 &mcp2515; /**< Reference to MCP2515 for sending CAN messages */
    CarState &car;    /**< Reference to CarState */
};
#endif // TELEMETRY_HPP