/**
 * @file Telemetry.cpp
 * @author Planeson, Red Bird Racing
 * @brief Implementation of the Telemetry class for sending telemetry data over CAN bus
 * @version 1.0
 * @date 2026-01-26
 * @see Telemetry.hpp
 */

#include "Telemetry.hpp"

/**
 * @brief Construct a new Telemetry object
 * @param mcp2515_ Reference to MCP2515 for sending CAN messages
 * @param car_ Reference to CarState
 */
Telemetry::Telemetry(MCP2515 &mcp2515_, CarState &car_)
    : mcp2515(mcp2515_), car(car_)
{
}

/**
 * @brief Internal helper to get and send the ADC telemetry frame
 */
void Telemetry::schedulerAdc()
{
    can_frame adc_frame = car.adc.toCanFrame();
    mcp2515.sendMessage(&adc_frame);
}

/**
 * @brief Internal helper to get and send the digital telemetry frame
 */
void Telemetry::schedulerDigital()
{
    can_frame digital_frame = car.digital.toCanFrame();
    mcp2515.sendMessage(&digital_frame);
}

/**
 * @brief Internal helper to get and send the state telemetry frame
 */
void Telemetry::schedulerStates()
{
    can_frame state_frame = car.state.toCanFrame();
    mcp2515.sendMessage(&state_frame);
}

/**
 * @brief Sends telemetry data based on the scheduling logic
 */
void Telemetry::sendTelemetry()
{
    if (count >= MAX_FRAME_COUNTER)
    {
        schedulerStates();
        count = 0;
        return;
    }
    
    if (count % FRAMES_FREQ_RATIO == 0)
    {
        schedulerAdc();
    }
    else
    {
        schedulerDigital();
    }
    ++count;
}