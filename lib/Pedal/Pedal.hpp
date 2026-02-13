/**
 * @file Pedal.hpp
 * @author Planeson, Red Bird Racing
 * @brief Declaration of the Pedal class for handling throttle and brake pedal inputs
 * @version 1.6
 * @date 2026-02-12
 * @see Pedal.cpp
 * @dir Pedal @brief The Pedal library contains the Pedal class to manage throttle and brake pedal inputs, including filtering, fault detection, and CAN communication.
 */

#ifndef PEDAL_HPP
#define PEDAL_HPP

#include <stdint.h>
#include "CarState.hpp"
#include "Interp.hpp"
#include "Curves.hpp"
#include "SignalProcessing.hpp"

// ignore -Wpedantic warnings for mcp2515.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mcp2515.h>
#pragma GCC diagnostic pop

// Constants

constexpr bool REGEN_ENABLED = false; /**< Boolean toggle for regenerative braking; false disables reverse torque. */

constexpr bool FLIP_MOTOR_DIR = false; /**< Boolean toggle to flip motor direction; true inverts torque commands. */

constexpr bool BRAKE_RELIABLE = true; /**< brake assumed reliable; enable checking of min/max (narrow range); set to false compromises safety! */

constexpr uint16_t FAULT_CHECK_HEX = BRAKE_RELIABLE ? 0xFE : 0x3E; /**< Hex mask for fault checking based on brake reliability. */

constexpr uint32_t MAX_MOTOR_READ_MILLIS = 100; /**< Maximum time in milliseconds between motor data reads before disabling regen. */

/**
 * @brief Namespace for pedal-related constants, such as thresholds and calculation parameters.
 * This is to avoid polluting the Pedal class with intermediate results.
 */
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
    Pedal(MCP2515 &motor_can_, CarState &car, uint16_t &pedal_final_);
    void update(uint16_t pedal_1, uint16_t pedal_2, uint16_t brake);
    void sendFrame();
    void readMotor();
    uint16_t &pedal_final; /**< Final pedal value is taken directly from apps_5v, see initializer */

private:
    CarState &car;                   /**< Reference to CarState */
    MCP2515 &motor_can;              /**< Reference to MCP2515 for sending CAN messages */
    uint32_t fault_start_millis;     /**< Timestamp for when a fault started */
    uint32_t last_motor_read_millis; /**< Timestamp for the last motor data read */

    /**
     * @brief CAN frame to stop the motor
     */
    const can_frame stop_frame = {
        MOTOR_SEND, /**< can_id */
        3,          /**< can_dlc */
        0x90,       /**< data, torque command */
        0x00,       /**< data, 0 torque * 2 */
        0x00};

    /**
     * @brief CAN frame for torque command
     */
    can_frame torque_msg = {
        MOTOR_SEND, /**< can_id */
        3,          /**< can_dlc */
        0x90,       /**< data, torque command */
        0x00,       /**< data, init as 0 torque * 2 */
        0x00};

    // Filters for pedal and brake inputs, see Signal_Processing.hpp for options
    ExponentialFilter<uint16_t, uint16_t> pedal1_filter; /**< Filter for first pedal sensor input */
    ExponentialFilter<uint16_t, uint16_t> pedal2_filter; /**< Filter for second pedal sensor input */
    ExponentialFilter<uint16_t, uint16_t> brake_filter;  /**< Filter for brake sensor input */

    static constexpr LinearInterp<uint16_t, int16_t, int32_t, 5> THROTTLE_MAP{THROTTLE_TABLE};               /**< Interpolation map for throttle torque */
    static constexpr LinearInterp<uint16_t, int16_t, int32_t, 5> BRAKE_MAP{BRAKE_TABLE};                     /**< Interpolation map for brake torque */
    static constexpr LinearInterp<uint16_t, uint16_t, uint32_t, 2> APPS_3V3_SCALE_MAP{APPS_3V3_SCALE_TABLE}; /**< Interpolation map for APPS_3V3->APPS_5V */

    static constexpr canid_t MOTOR_SEND = 0x201; /**< Motor send CAN ID */
    static constexpr canid_t MOTOR_READ = 0x181; /**< Motor read CAN ID */

    static constexpr uint8_t REGID_READ = 0x3D; /**< Register ID for reading motor data */

    static constexpr uint8_t SPEED_IST = 0x30; /**< Register ID for "actual speed value" */
    static constexpr uint8_t WARN_ERR = 0x8F;  /**< Register ID for warnings and errors */

    static constexpr uint8_t RPM_PERIOD = 20; /**< Period of reading motor data in ms, set to 20ms to get 10ms reads alongside errors */
    static constexpr uint8_t ERR_PERIOD = 20; /**< Period of reading motor errors in ms, set to 20ms to get 10ms reads alongside rpm */

    bool checkPedalFault();
    constexpr int16_t pedalTorqueMapping(const uint16_t pedal, const uint16_t brake, const int16_t motor_rpm, const bool flip_dir);

    MCP2515::ERROR sendCyclicRead(uint8_t reg_id, uint8_t read_period);
};

#endif // PEDAL_HPP