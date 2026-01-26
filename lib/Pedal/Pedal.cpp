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
    car.adc.apps_5v = AVG_filter<uint16_t>(pedal_value_1.buffer, ADC_BUFFER_SIZE);
    car.adc.apps_3v3 = AVG_filter<uint16_t>(pedal_value_2.buffer, ADC_BUFFER_SIZE);
    car.adc.brake = AVG_filter<uint16_t>(brake_value.buffer, ADC_BUFFER_SIZE);

    if (car.adc.apps_5v < apps_5v_min)
        car.state.faults.bits.apps_5v_low = true;
    if (car.adc.apps_5v > apps_5v_max)
        car.state.faults.bits.apps_5v_high = true;
    if (car.adc.apps_3v3 < apps_3v3_min)
        car.state.faults.bits.apps_3v3_low = true;
    if (car.adc.apps_3v3 > apps_3v3_max)
        car.state.faults.bits.apps_3v3_high = true;
    if (car.adc.brake < brake_min)
        car.state.faults.bits.brake_low = true;
    if (car.adc.brake > brake_max)
        car.state.faults.bits.brake_high = true;

    if (checkPedalFault())
    {
        // fault now
        if (car.state.faults.bits.fault_active)
        {
            // was faulty already, check time
            if (car.millis - fault_start_millis > 100)
            {
                car.state.faults.bits.fault_exceeded = true; // Turning off the motor is achieved using another digital pin, not via canbus, but will still send 0 torque can frames

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
        car.state.faults.bits.fault_active = true;
    }
    else
    {
        // no fault
        if (car.state.faults.bits.fault_active)
        {
            DBG_THROTTLE_FAULT(PedalFault::DiffResolved);
        }
        car.state.faults.bits.fault_active = false;
    }

    if (car.state.faults.byte & 0xFE) // any fault bits other than active *** double check this is correct with real board ***
    {
        car.state.status.bits.force_stop = true; // set force stop if any fault other than active
    }

    return;
}

/**
 * @brief Sends the appropriate CAN frame based on pedal and car state.
 */
void Pedal::sendFrame()
{
    if (car.state.status.bits.force_stop)
    {
        DBGLN_THROTTLE("Stopping motor: pedal fault");
        motor_can.sendMessage(&stop_frame);
        return;
    }
    if (car.state.status.bits.car_status != CarStatus::Drive)
    {
        switch (car.state.status.bits.car_status)
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

    int16_t torque_val = throttleTorqueMapping(pedal_final, car.adc.brake, FLIP_MOTOR_DIR);

    torque_msg.can_id = MOTOR_COMMAND;
    torque_msg.can_dlc = 3;
    torque_msg.data[0] = 0x90; // 0x90 for torque, 0x31 for speed
    torque_msg.data[1] = torque_val & 0xFF;
    torque_msg.data[2] = (torque_val >> 8) & 0xFF;
    motor_can.sendMessage(&torque_msg);
    return;
}

/**
 * @brief Maps the pedal ADC to a torque value, handling deadzones and faults.
 *
 * @param pedal Pedal ADC in the range of 0-1023.
 * @param brake Brake ADC in the range of 0-1023.
 * @param flip_dir Boolean indicating whether to flip the motor direction.
 * @return Mapped torque value in the signed range of -TORQUE_MAX to TORQUE_MAX.
 */
constexpr int16_t Pedal::throttleTorqueMapping(const uint16_t pedal, const uint16_t brake, const bool flip_dir)
{
    if (brake > brake_map.start())
    {
        if (pedal > throttle_map.start())
        {
            car.state.status.bits.screenshot = true;
        }
        return brakeTorqueMapping(brake, flip_dir);
    }
    if (flip_dir)
    {
        return -throttle_map.interp(pedal);
    }
    return throttle_map.interp(pedal);
}

/**
 * @brief Maps the brake ADC to a torque value.
 *
 * This function takes a brake ADC in the range of 0-1023 and maps it to a torque value.
 * The mapping is linear, and the result is adjusted based on the motor direction.
 *
 * @param brake Brake ADC in the range of 0-1023.
 * @param flip_dir Boolean indicating whether to flip the motor direction.
 * @return Mapped torque value in the signed range of -REGEN_MAX to REGEN_MAX.
 */
constexpr int16_t Pedal::brakeTorqueMapping(const uint16_t brake, const bool flip_dir)
{
    if (flip_dir)
    {
        return -brake_map.interp(brake);
    }
    return brake_map.interp(brake);
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
    car.digital.apps_3v3_scaled = car.adc.apps_3v3 * APPS_RATIO;

    const int16_t delta = (int16_t)car.adc.apps_5v - (int16_t)car.digital.apps_3v3_scaled;
    // if more than 10% difference between the two pedals, consider it a fault
    if (delta > 102.4 || delta < -102.4) // 10% of 1024, rounded down to 102
    {
        DBG_THROTTLE_FAULT(PedalFault::DiffContinuing, delta);
        return true;
    }
    return false;
}
