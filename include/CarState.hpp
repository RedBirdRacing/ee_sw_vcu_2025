/**
 * @file CarState.hpp
 * @author Planeson, Red Bird Racing
 * @brief Definition of the CarState structure representing the state of the car
 * @version 1.2.1
 * @date 2026-01-14
 * @see can.h
 * @see Enums.h
 */

#ifndef CAR_STATE_HPP
#define CAR_STATE_HPP

#include "Enums.h"
#include <can.h>
#include <stdint.h>

/**
 * @brief Telemetry frame structure for ADC readings.
 */
struct TelemetryFrameAdc
{
    uint16_t apps_5v;     /**< ADC reading for 5V APPS */
    uint16_t apps_3v3;    /**< ADC reading for 3.3V APPS */
    uint16_t brake;       /**< ADC reading for brake pedal */
    uint16_t hall_sensor; /**< ADC reading for hall sensor */
    /**
     * @brief Converts the TelemetryFrameAdc to a CAN frame.
     * @return CAN frame representing the telemetry ADC readings.
     */
    constexpr can_frame toCanFrame() const
    {
        return can_frame{
            TELEMETRY_ADC_MSG,                 // can_id
            8,                                 // can_dlc
            static_cast<__u8>(apps_5v & 0xFF), // data
            static_cast<__u8>((apps_5v >> 8) & 0xFF),
            static_cast<__u8>(apps_3v3 & 0xFF),
            static_cast<__u8>((apps_3v3 >> 8) & 0xFF),
            static_cast<__u8>(brake & 0xFF),
            static_cast<__u8>((brake >> 8) & 0xFF),
            static_cast<__u8>(hall_sensor & 0xFF),
            static_cast<__u8>((hall_sensor >> 8) & 0xFF)};
    }
};

/**
 * @brief Telemetry frame structure for digital signals.
 */
struct TelemetryFrameDigital
{
    uint16_t motor_rpm;       /**< Motor RPM */
    uint16_t motor_speed;     /**< Motor speed, can be found from RPM directly, only here because have space, thus calculate on VCU to lighten load on receiving end */
    uint16_t apps_3v3_scaled; /**< Scaled 3.3V APPS value, can be found from ADC readings directly, only here because have space, lighten load on receiving end / as debug */
    uint16_t torque_val;      /**< Torque value sent to motor controller*/

    /**
     * @brief Converts the TelemetryFrameDigital to a CAN frame.
     * @return CAN frame representing the telemetry digital signals.
     */
    constexpr can_frame toCanFrame() const
    {
        return can_frame{
            TELEMETRY_DIGITAL_MSG,               // can_id
            8,                                   // can_dlc
            static_cast<__u8>(motor_rpm & 0xFF), // data
            static_cast<__u8>((motor_rpm >> 8) & 0xFF),
            static_cast<__u8>(motor_speed & 0xFF),
            static_cast<__u8>((motor_speed >> 8) & 0xFF),
            static_cast<__u8>(apps_3v3_scaled & 0xFF),
            static_cast<__u8>((apps_3v3_scaled >> 8) & 0xFF),
            static_cast<__u8>(torque_val & 0xFF),
            static_cast<__u8>((torque_val >> 8) & 0xFF)};
    }
};

/**
 * @brief Telemetry frame structure for VCU states.
 */
struct TelemetryFrameState
{
    // first byte

    main_car_status car_status; /**< Current car status */
    bool state_unknown = false; /**< Unknown car state */
    bool hv_ready = false;      /**< High voltage ready */
    bool bms_no_msg = false;    /**< BMS read no message */
    bool bms_wrong_id = false;  /**< BMS read wrong ID */
    bool force_stop = false;    /**< Fault forced car to stop */
    bool screenshot = false;    /**< Screenshot, throttle + brake > threshold */

    // second byte
    bool fault_active = false;   /**< Pedal faulty now */
    bool fault_exceeded = false; /**< Current pedal fault exceeded allowed time */
    bool apps_5v_low = false;    /**< APPS 5V considered shorted to ground*/
    bool apps_5v_high = false;   /**< APPS 5V considered shorted to rail */
    bool apps_3v3_low = false;   /**< APPS 3V3 considered shorted to ground */
    bool apps_3v3_high = false;  /**< APPS 3V3 considered shorted to rail */
    bool brake_low = false;      /**< Brake considered shorted to ground */
    bool brake_high = false;     /**< Brake considered shorted to rail */

    uint8_t bms_data[6]; /**< Raw BMS data bytes */

    /**
     * @brief Converts the TelemetryFrameState to a CAN frame.
     * @return CAN frame representing the telemetry car state.
     */
    constexpr can_frame toCanFrame() const
    {
        return can_frame{
            TELEMETRY_STATE_MSG, // can_id
            8,                   // can_dlc
            static_cast<__u8>(
                (static_cast<uint8_t>(car_status) & 0x03) |
                (state_unknown ? 0x04 : 0x00) |
                (hv_ready ? 0x08 : 0x00) |
                (bms_no_msg ? 0x10 : 0x00) |
                (bms_wrong_id ? 0x20 : 0x00) |
                (force_stop ? 0x40 : 0x00) |
                (screenshot ? 0x80 : 0x00)),
            static_cast<__u8>(
                (fault_active ? 0x01 : 0x00) |
                (fault_exceeded ? 0x02 : 0x00) |
                (apps_5v_low ? 0x04 : 0x00) |
                (apps_5v_high ? 0x08 : 0x00) |
                (apps_3v3_low ? 0x10 : 0x00) |
                (apps_3v3_high ? 0x20 : 0x00) |
                (brake_low ? 0x40 : 0x00) |
                (brake_high ? 0x80 : 0x00)),
            bms_data[0],
            bms_data[1],
            bms_data[2],
            bms_data[3],
            bms_data[4],
            bms_data[5]};
    }
};

/**
 * @brief Represents the state of the car.
 * Holds telemetry data and status, used as central data sharing structure.
 *
 * @see TelemetryFrameAdc, TelemetryFrameDigital, TelemetryFrameState
 */
struct CarState
{
    TelemetryFrameAdc adc;         /**< Struct holding ADC telemetry data, ready for sending over CAN */
    TelemetryFrameDigital digital; /**< Struct holding digital telemetry data, ready for sending over CAN */
    TelemetryFrameState state;     /**< Struct holding state telemetry data, ready for sending over CAN */
    uint32_t status_millis;        /**< Millisecond counter for the current car status (for state transitions) */
    uint32_t millis;               /**< Current time in milliseconds for the current loop iteration */
};
#endif // CAR_STATE_HPP