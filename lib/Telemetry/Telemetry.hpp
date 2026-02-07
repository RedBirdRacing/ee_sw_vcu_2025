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
    void sendTelemetry();

private:
    MCP2515 &mcp2515; /**< Reference to MCP2515 for sending CAN messages */
    CarState &car;    /**< Reference to CarState */

    uint8_t count; /**< Counter number of times message sent */

    // == Telemetry order ===
    // assume we have 2 frequency of frames, one slow one fast
    // we want to round-robin the them, so if msg1 and msg2 are 100Hz, and msg3 is 10Hz, then we do 12121...1212312212...

    static constexpr uint8_t NUM_FREQ_FRAMES = 2;   /**< Number of frequent telemetry frames to send */
    static constexpr uint8_t NUM_INFREQ_FRAMES = 1; /**< Number of infrequent telemetry frames to send, must be 1 because of how the fast modulo logic works */
    static constexpr uint8_t FRAMES_FREQ_RATIO = 10; /**< Ratio of frame frequencies */
    static constexpr uint8_t MAX_FRAME_COUNTER = NUM_FREQ_FRAMES * FRAMES_FREQ_RATIO + NUM_INFREQ_FRAMES - 2; /**< Count resets at this value, total frames per cycle - 2 because tree planting */

    void schedulerPedal();
    void schedulerMotor();
    void schedulerBms();
};
#endif // TELEMETRY_HPP