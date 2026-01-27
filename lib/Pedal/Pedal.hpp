/**
 * @file Pedal.hpp
 * @author Planeson, Red Bird Racing
 * @brief Declaration of the Pedal class for handling throttle and brake pedal inputs
 * @version 1.4
 * @date 2026-01-26
 * @see Pedal.cpp
 */

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

// Constants

constexpr bool REGEN_ENABLED = true; /**< Boolean toggle for regenerative braking; false disables reverse torque. */

// Flips the direction of motor output
// set to true for gen 3
// false for gen 5
// later make class member for future dev
constexpr bool FLIP_MOTOR_DIR = false;

namespace PedalConstants
{
    constexpr uint8_t MIN_REGEN_KMH = 10;          /**< Minimum speed (km/h) for regenerative braking to be active. */
    constexpr uint8_t GEAR_RATIO_NUMERATOR = 60;   /**< Gear ratio numerator of the drivetrain. */
    constexpr uint8_t GEAR_RATIO_DENOMINATOR = 13; /**< Gear ratio denominator of the drivetrain. */

    // === Calculation for RPM threshold ===
    constexpr uint8_t WHEEL_DIAMETER_INCH = 13; /**< Wheel diameter in inches. */
    constexpr uint16_t MAX_MOTOR_RPM = 7000;    /**< Maximum motor RPM. */
    constexpr uint16_t MAX_TORQUE_VAL = 32767;  /**< Maximum torque value for motor controller. */

    constexpr uint16_t INCH_PER_KM = 39370;                   /**< Inches per kilometer. */
    constexpr uint8_t MINUTES_PER_HOUR = 60;                  /**< Minutes per hour. */
    constexpr double PI_ = 3.1415926535897932384626433832795; /**< Value of pi, unnamed to avoid clashing with Arduino.h's definition. */

    /** Final RPM = KMH -> Inches per Hour -> Inch per minute -> RPM at wheel -> RPM at motor */
    constexpr int16_t MIN_REGEN_RPM_VAL =
        (double)MIN_REGEN_KMH / MINUTES_PER_HOUR * INCH_PER_KM / WHEEL_DIAMETER_INCH / PI_ * GEAR_RATIO_NUMERATOR / GEAR_RATIO_DENOMINATOR * MAX_TORQUE_VAL / MAX_MOTOR_RPM; /**< Minimum RPM for regenerative braking to be active. */
} // namespace PedalConstants
constexpr uint8_t ADC_BUFFER_SIZE = 16; /**< Size of the ADC reading buffer for filtering. */

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
    void readMotor();
    uint16_t &pedal_final = car.pedal.apps_5v; /**< Final pedal value is taken directly from apps_5v */

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
        MOTOR_SEND, /**< can_id */
        3,          /**< can_dlc */
        0x90,       /**< data, torque command */
        0x00,       /**< data, 0 torque * 2 */
        0x00};

    can_frame torque_msg; /**< CAN frame for torque command */

    // Cyclic queues for storing the pedal values
    // Used for filtering, right now average of the last values

    RingBuffer<uint16_t, ADC_BUFFER_SIZE> pedal_value_1; /**< Ring buffer for APPS 5V values */
    RingBuffer<uint16_t, ADC_BUFFER_SIZE> pedal_value_2; /**< Ring buffer for APPS 3.3V values */
    RingBuffer<uint16_t, ADC_BUFFER_SIZE> brake_value;   /**< Ring buffer for brake pedal values */

    LinearInterp<uint16_t, int16_t, int32_t, 5> throttle_map{throttle_table}; /**< Interpolation map for throttle torque */
    LinearInterp<uint16_t, int16_t, int32_t, 5> brake_map{brake_table};       /**< Interpolation map for brake torque */

    static constexpr canid_t MOTOR_SEND = 0x201; /**< Motor send CAN ID */
    static constexpr canid_t MOTOR_READ = 0x181; /**< Motor read CAN ID */

    static constexpr uint8_t REGID_READ = 0x3D; /**< Register ID for reading motor data */

    static constexpr uint8_t SPEED_IST = 0x30; /**< Register ID for "actual speed value" */
    static constexpr uint8_t WARN_ERR = 0x8F;  /**< Register ID for warnings and errors */

    static constexpr uint8_t RPM_PERIOD = 20; /**< Period of reading motor data in ms, set to 20ms to get 10ms reads alongside errors */
    static constexpr uint8_t ERR_PERIOD = 20; /**< Period of reading motor errors in ms, set to 20ms to get 10ms reads alongside rpm */

    bool checkPedalFault();
    constexpr int16_t throttleTorqueMapping(const uint16_t pedal, const uint16_t brake, const int16_t motor_rpm, const bool flip_dir);
    constexpr int16_t brakeTorqueMapping(const uint16_t brake, const bool flip_dir);

    MCP2515::ERROR sendCyclicRead(uint8_t reg_id, uint8_t read_period);
};

#endif // PEDAL_HPP