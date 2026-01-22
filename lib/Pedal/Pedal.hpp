#ifndef PEDAL_HPP
#define PEDAL_HPP

#include <stdint.h>
#include "Queue.hpp"
#include "CarState.hpp"
#include "Interp.hpp"
#include "Curves.hpp"

// ignore -Wpedantic warnings for mcp2515.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mcp2515.h>
#pragma GCC diagnostic pop

constexpr bool REGEN_ENABLED = true; // Set to true to enable regenerative braking, false to disable

// Constants

// Flips the direction of motor output
// set to true for gen 3
// false for gen 5
// later make class member for future dev
constexpr bool FLIP_MOTOR_DIR = false;

/**
 * @brief Ratio between 5V APPS and 3.3V APPS, use integer math to avoid float operations.
 * Expanded to apps_scaled = apps_3v3 * APPS_RATIO
 * @note max 64 for multiply to prevent overflow
 */
#define APPS_RATIO 50 / 33

#define ADC_BUFFER_SIZE 16

/**
 * @brief Pedal class for managing throttle and brake pedal inputs.
 * Handles filtering, fault detection, and CAN frame updates.
 */
class Pedal
{
public:
    Pedal(CarState &car, MCP2515 &motor_can_);
    void update(uint16_t pedal_1, uint16_t pedal_2, uint16_t brake);
    void sendFrame();
    uint16_t &pedal_final = car.adc.apps_5v; /**< Final pedal value is taken directly from apps_5v */

private:
    CarState &car;      /**< Reference to CarState */
    MCP2515 &motor_can; /**< Reference to MCP2515 for sending CAN messages */
    // If the two potentiometer inputs are too different (> 10%), the inputs are faulty
    // Definition for faulty is under FSEC 2024 Chapter 2, section 12.8, 12.9
    bool fault = true;
    uint32_t fault_start_millis; // rollover in 49.7 days
    /**
     * @brief CAN frame to stop the motor
     */
    const can_frame stop_frame = {
        MOTOR_COMMAND, /**< can_id */
        3,             /**< can_dlc */
        0x90,          /**< data, torque command */
        0x00,          /**< data, 0 torque * 2 */
        0x00};

    can_frame torque_msg; /**< CAN frame for torque command */

    // Cyclic queues for storing the pedal values
    // Used for filtering, right now average of the last values
    RingBuffer<uint16_t, ADC_BUFFER_SIZE> pedal_value_1;
    RingBuffer<uint16_t, ADC_BUFFER_SIZE> pedal_value_2;
    RingBuffer<uint16_t, ADC_BUFFER_SIZE> brake_value;

    LinearInterp<uint16_t, int16_t, int32_t, 5> throttleMap{throttleTable};
    LinearInterp<uint16_t, int16_t, int32_t, 5> brakeMap{brakeTable};

    bool checkPedalFault();
    int16_t throttleTorqueMapping(uint16_t pedal, uint16_t brake, bool flip_dir);
    int16_t brakeTorqueMapping(uint16_t brake, bool flip_dir);
};

#endif // PEDAL_HPP