/**
 * @file Pedal.cpp
 * @author Planeson, Red Bird Racing
 * @brief Implementation of the Pedal class for handling throttle pedal inputs
 * @version 1.4
 * @date 2026-01-26
 * @see Pedal.hpp
 */

#include "Pedal.hpp"
#include "Signal_Processing.hpp" // AVG_filter
#include "CarState.hpp"
#include <stdint.h>
#include "Queue.hpp"
#include "CarState.hpp"
#include "Interp.hpp"
#include "Curves.hpp"

// ignore -Wunused-parameter warnings for Debug.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "Debug.hpp" // DBGLN_GENERAL
#pragma GCC diagnostic pop

#include <Arduino.h> // round()

/**
 * @brief Constructor for the Pedal class.
 * Initializes the pedal state. fault is set to true initially,
 * so you must send update within 100ms of starting the car to clear it.
 */
Pedal::Pedal(CarState &car_, MCP2515 &motor_can_)
    : car(car_),
      motor_can(motor_can_),
      fault(true),
      fault_start_millis(0),
      pedal_value_1(), // RingBuffer default init 0
      pedal_value_2(),
      brake_value()
{
    while (sendCyclicRead(SPEED_IST, RPM_PERIOD) != MCP2515::ERROR_OK)
    {
    }
    while (sendCyclicRead(WARN_ERR, ERR_PERIOD) != MCP2515::ERROR_OK)
    {
    }
}

/**
 * @brief Updates pedal sensor readings, applies filtering, and checks for faults.
 *
 * Stores new pedal readings, applies an average filter, and updates car state.
 * If a fault is detected between pedal sensors, sets fault flags and logs status.
 *
 * @param pedal_1 Raw value from pedal sensor 1.
 * @param pedal_2 Raw value from pedal sensor 2.
 * @param brake Raw value from brake sensor.
 */
void Pedal::update(uint16_t pedal_1, uint16_t pedal_2, uint16_t brake)
{
    // Record readings in buffer
    pedal_value_1.push(pedal_1);
    pedal_value_2.push(pedal_2);
    brake_value.push(brake);

    // Range of pedal 1 is APPS_PEDAL_1_RANGE, pedal 2 is APPS_PEDAL_2_RANGE;

    // this is current taking the direct array the circular queue writes into. Bad idea to do anything other than a simple average
    // if not using a linear filter, pass the pedalValue_1.getLinearBuffer() to the filter function to ensure the ordering is correct.
    // can also consider injecting the filter into the queue if need
    // depends on the hardware filter, reduce software filtering as much as possible

    // currently a small average filter is good enough
    car.pedal.apps_5v = AVG_filter<uint16_t>(pedal_value_1.buffer, ADC_BUFFER_SIZE);
    car.pedal.apps_3v3 = AVG_filter<uint16_t>(pedal_value_2.buffer, ADC_BUFFER_SIZE);
    car.pedal.brake = AVG_filter<uint16_t>(brake_value.buffer, ADC_BUFFER_SIZE);

    if (car.pedal.apps_5v < APPS_5V_MIN)
        car.pedal.faults.bits.apps_5v_low = true;
    if (car.pedal.apps_5v > APPS_5V_MAX)
        car.pedal.faults.bits.apps_5v_high = true;
    if (car.pedal.apps_3v3 < APPS_3V3_MIN)
        car.pedal.faults.bits.apps_3v3_low = true;
    if (car.pedal.apps_3v3 > APPS_3V3_MAX)
        car.pedal.faults.bits.apps_3v3_high = true;
    if (car.pedal.brake < brake_min)
        car.pedal.faults.bits.brake_low = true;
    if (car.pedal.brake > brake_max)
        car.pedal.faults.bits.brake_high = true;

    if (checkPedalFault())
    {
        // fault now
        if (car.pedal.faults.bits.fault_active)
        {
            // was faulty already, check time
            if (car.millis - fault_start_millis > 100)
            {
                car.pedal.faults.bits.fault_exceeded = true; // Turning off the motor is achieved using another digital pin, not via canbus, but will still send 0 torque can frames

                DBG_THROTTLE_FAULT(PedalFault::DiffExceed100ms);
                return;
            }
            else
            {
                DBG_THROTTLE_FAULT(PedalFault::DiffContinuing); // will be optimized out if the debug macro is off
            }
        }
        else
        {
            // new fault
            fault_start_millis = car.millis;
            DBG_THROTTLE_FAULT(PedalFault::DiffStart);
        }
        car.pedal.faults.bits.fault_active = true;
    }
    else
    {
        // no fault
        if (car.pedal.faults.bits.fault_active)
        {
            DBG_THROTTLE_FAULT(PedalFault::DiffResolved);
        }
        car.pedal.faults.bits.fault_active = false;
    }

    if (car.pedal.faults.byte & 0xFE) // any fault bits other than active *** double check this is correct with real board ***
    {
        car.pedal.status.bits.force_stop = true; // set force stop if any fault other than active
    }

    return;
}

/**
 * @brief Sends the appropriate CAN frame to the motor based on pedal and car state.
 */
void Pedal::sendFrame()
{
    if (car.pedal.status.bits.force_stop)
    {
        DBGLN_THROTTLE("Stopping motor: pedal fault");
        motor_can.sendMessage(&stop_frame);
        return;
    }
    if (car.pedal.status.bits.car_status != CarStatus::Drive)
    {
        switch (car.pedal.status.bits.car_status)
        {
        case CarStatus::Init:
            DBGLN_THROTTLE("Stopping motor: in INIT.");
            break;
        case CarStatus::Startin:
            DBGLN_THROTTLE("Stopping motor: in STARTIN.");
            break;
        case CarStatus::Bussin:
            DBGLN_THROTTLE("Stopping motor: in BUSSIN.");
            break;
        default:
            DBGLN_THROTTLE("Stopping motor: in UNKNOWN STATE.");
            break;
        }
        motor_can.sendMessage(&stop_frame);
        return;
    }

    int16_t torque_val = pedalTorqueMapping(pedal_final, car.pedal.brake, car.motor.motor_rpm, FLIP_MOTOR_DIR);

    torque_msg.can_id = MOTOR_SEND;
    torque_msg.can_dlc = 3;
    torque_msg.data[0] = 0x90; // 0x90 for torque, 0x31 for speed
    torque_msg.data[1] = torque_val & 0xFF;
    torque_msg.data[2] = (torque_val >> 8) & 0xFF;
    motor_can.sendMessage(&torque_msg);
    return;
}

/**
 * @brief Maps the pedal ADC to a torque value.
 * If no braking requested, maps throttle normally.
 * If braking requested and regen enabled,
 *      applies regen if motor RPM larger than minimum regen RPM,
 *      preventing reverse torque at low speeds.
 *
 * @param pedal Pedal ADC in the range of 0-1023.
 * @param brake Brake ADC in the range of 0-1023.
 * @param motor_rpm Current motor RPM for regen logic.
 * @param flip_dir Boolean indicating whether to flip the motor direction.
 * @return Mapped torque value in the signed range of -TORQUE_MAX to TORQUE_MAX.
 */
constexpr int16_t Pedal::pedalTorqueMapping(const uint16_t pedal, const uint16_t brake, const int16_t motor_rpm, const bool flip_dir)
{
    if (REGEN_ENABLED && brake > BRAKE_MAP.start())
    {
        if (pedal > THROTTLE_MAP.start())
            car.pedal.status.bits.screenshot = true;
        if (flip_dir)
        {
            if (motor_rpm < PedalConstants::MIN_REGEN_RPM_VAL)
                return 0;
            else
                return -BRAKE_MAP.interp(brake);
        }
        else
        {
            if (motor_rpm > -PedalConstants::MIN_REGEN_RPM_VAL)
                return 0;
            else
                return BRAKE_MAP.interp(brake);
        }
    }

    if (flip_dir)
        return -THROTTLE_MAP.interp(pedal);
    else
        return THROTTLE_MAP.interp(pedal);
}

/**
 * @brief Checks for a fault between two pedal sensor readings.
 *
 * Scales pedal_2 to match the range of pedal_1, then calculates the absolute difference.
 * If the difference exceeds 10% of the full-scale value (i.e., >102.4 for a 10-bit ADC),
 * the function considers this a fault and returns true. Otherwise, returns false.
 *
 * @return true if the difference exceeds the threshold (fault detected), false otherwise.
 */
bool Pedal::checkPedalFault()
{
    car.pedal.apps_3v3_scaled = car.pedal.apps_3v3 * APPS_RATIO;

    const int16_t delta = (int16_t)car.pedal.apps_5v - (int16_t)car.pedal.apps_3v3_scaled;
    // if more than 10% difference between the two pedals, consider it a fault
    if (delta > 102.4 || delta < -102.4) // 10% of 1024, rounded down to 102
    {
        DBG_THROTTLE_FAULT(PedalFault::DiffContinuing, delta);
        return true;
    }
    return false;
}

/**
 * @brief Sends a cyclic read request to the motor controller for speed (rpm).
 * @param reg_id Register ID to read from the motor controller.
 * @param read_period Period of reading motor data in ms.
 * @return MCP2515::ERROR indicating success or failure of sending the message.
 */
MCP2515::ERROR Pedal::sendCyclicRead(uint8_t reg_id, uint8_t read_period)
{
    can_frame cyclic_request = {
        MOTOR_SEND, /**< can_id */
        3,          /**< can_dlc */
        REGID_READ, /**< data, register ID */
        reg_id,     /**< data, sub ID */
        read_period /**< data, read period in ms */
    };
    return motor_can.sendMessage(&cyclic_request);
}

/**
 * @brief Reads motor data from the CAN bus and updates the CarState.
 */
void Pedal::readMotor()
{
    can_frame rx_frame;
    if (motor_can.readMessage(&rx_frame) == MCP2515::ERROR_OK)
    {
        if (rx_frame.can_id == MOTOR_READ && rx_frame.can_dlc > 3)
        {
            if (rx_frame.data[0] == SPEED_IST)
            {
                car.motor.motor_rpm = static_cast<int16_t>(rx_frame.data[1] | (rx_frame.data[2] << 8));
                return;
            }
            else if (rx_frame.data[0] == WARN_ERR)
            {
                car.motor.motor_error = static_cast<uint16_t>(rx_frame.data[1] | (rx_frame.data[2] << 8));
                car.motor.motor_warn = static_cast<uint16_t>(rx_frame.data[3] | (rx_frame.data[4] << 8));
                return;
            }
        }
    }
    return;
}