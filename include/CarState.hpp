/**
 * @file CarState.hpp
 * @author Planeson, Red Bird Racing
 * @brief Definition of the CarState structure representing the state of the car
 * @version 1.4.1
 * @date 2026-02-09
 * @see can.h, Enums.h
 */

#ifndef CAR_STATE_HPP
#define CAR_STATE_HPP

#include "Enums.hpp"
#include <can.h>
#include <stdint.h>

constexpr canid_t TELEMETRY_PEDAL_MSG = 0x700; /**< Telemetry: Pedal readings message */
constexpr canid_t TELEMETRY_MOTOR_MSG = 0x701; /**< Telemetry: Digital signals message */
constexpr canid_t TELEMETRY_BMS_MSG = 0x710;   /**< Telemetry: Car state message */

/**
 * @brief Telemetry frame structure for the Pedals.
 */
struct TelemetryFramePedal
{
    uint16_t apps_5v;  /**< ADC reading for 5V APPS */
    uint16_t apps_3v3; /**< ADC reading for 3.3V APPS */
    uint16_t brake;       /**< ADC reading for brake pedal */
    uint16_t hall_sensor; /**< ADC reading for hall sensor */

    /** @brief Union of bits for car status besides Pedal */
    union StateByteStatus
    {
        uint8_t byte; /**< Byte representation of the status bits */
        
        /** @brief Bitfield representation of the status bits */
        struct Bits
        {
            CarStatus car_status : 2; /**< Current car status, produces compiler warning before GCC 9.3 due to bug */
            bool state_unknown : 1;   /**< Unknown car state */
            bool hv_ready : 1;        /**< High voltage ready */
            bool bms_no_msg : 1;      /**< BMS read no message */
            bool motor_no_read : 1;    /**< MCU read no message */
            bool screenshot : 1;      /**< Screenshot, throttle + brake > threshold */
            bool force_stop : 1;      /**< Fault forced car to stop */
        } bits;
    };
    /** @brief Union of bits for pedal faults */
    union StateByteFaults
    {
        uint8_t byte; /**< Byte representation of the fault bits */

        /** @brief Bitfield representation of the fault bits */
        struct Bits
        {
            bool fault_active : 1;   /**< Pedal faulty now, only one resetable */
            bool fault_exceeded : 1; /**< Current pedal fault exceeded allowed time */
            bool apps_5v_low : 1;    /**< APPS 5V considered shorted to ground*/
            bool apps_5v_high : 1;   /**< APPS 5V considered shorted to rail */
            bool apps_3v3_low : 1;   /**< APPS 3V3 considered shorted to ground */
            bool apps_3v3_high : 1;  /**< APPS 3V3 considered shorted to rail */
            bool brake_low : 1;      /**< Brake considered shorted to ground */
            bool brake_high : 1;     /**< Brake considered shorted to rail */
        } bits;
    };

    static_assert(sizeof(StateByteStatus) == 1, "TelemetryStateByte0 must be 1 byte"); // ensure compile is shoving the bits as expected
    static_assert(sizeof(StateByteFaults) == 1, "TelemetryStateByte1 must be 1 byte");

    StateByteStatus status; /**< Car Status */
    StateByteFaults faults; /**< Pedal Faults */

    /**
     * @brief Converts the TelemetryFramePedal to a CAN frame.
     * @return CAN frame representing the Pedal telemetry signals.
     */
    constexpr can_frame toCanFrame() const
    {
        return can_frame{
            TELEMETRY_PEDAL_MSG,               // can_id
            8,                                 // can_dlc
            static_cast<__u8>(apps_5v & 0xFF), // data
            static_cast<__u8>(((apps_5v >> 8) & 0x03) | ((apps_3v3 & 0x3F) << 2)),
            static_cast<__u8>(((apps_3v3 >> 6) & 0x0F) | ((brake & 0x0F) << 4)),
            static_cast<__u8>(((brake >> 4) & 0x3F) | ((hall_sensor & 0x03) << 6)),
            static_cast<__u8>((hall_sensor >> 2) & 0xFF),
            status.byte,
            faults.byte,
            0x00};
    }
};

/**
 * @brief Telemetry frame structure for motor signals.
 */
struct TelemetryFrameMotor
{
    uint16_t torque_val;  /**< Torque value sent to motor controller*/
    uint16_t motor_rpm;   /**< Motor RPM */
    uint16_t motor_error; /**< Motor status byte */
    uint16_t motor_warn;  /**< Motor error/warning byte */

    /**
     * @brief Converts the TelemetryFrameMotor to a CAN frame.
     * @return CAN frame representing the telemetry motor signals.
     */
    constexpr can_frame toCanFrame() const
    {
        return can_frame{
            TELEMETRY_MOTOR_MSG, // can_id
            8,                   // can_dlc
            static_cast<__u8>(torque_val & 0xFF),
            static_cast<__u8>((torque_val >> 8) & 0xFF),
            static_cast<__u8>(motor_rpm & 0xFF),
            static_cast<__u8>((motor_rpm >> 8) & 0xFF),
            static_cast<__u8>(motor_error & 0xFF),
            static_cast<__u8>((motor_error >> 8) & 0xFF),
            static_cast<__u8>(motor_warn & 0xFF),
            static_cast<__u8>((motor_warn >> 8) & 0xFF)};
    }
};

/**
 * @brief Telemetry frame structure for the BMS data.
 */
struct TelemetryFrameBms
{

    uint8_t bms_data[8]; /**< Raw BMS data bytes */

    /**
     * @brief Converts the TelemetryFrameBms to a CAN frame.
     * @return CAN frame representing the telemetry BMS data.
     */
    constexpr can_frame toCanFrame() const
    {
        return can_frame{
            TELEMETRY_BMS_MSG, // can_id
            8,                 // can_dlc
            bms_data[0],
            bms_data[1],
            bms_data[2],
            bms_data[3],
            bms_data[4],
            bms_data[5],
            bms_data[6],
            bms_data[7]};
    }
};

/**
 * @brief Represents the state of the car.
 * Holds telemetry data and status, used as central data sharing structure.
 *
 * @see TelemetryFramePedal, TelemetryFrameMotor, TelemetryFrameBms
 */
struct CarState
{
    TelemetryFramePedal pedal; /**< Struct holding pedal telemetry data, ready for sending over CAN */
    TelemetryFrameMotor motor; /**< Struct holding motor telemetry data, ready for sending over CAN */
    TelemetryFrameBms bms;     /**< Struct holding BMS telemetry data, ready for sending over CAN */
    uint32_t status_millis;    /**< Millisecond counter for the current car status (for state transitions) */
    uint32_t millis;           /**< Current time in milliseconds for the current loop iteration */
};
#endif // CAR_STATE_HPP