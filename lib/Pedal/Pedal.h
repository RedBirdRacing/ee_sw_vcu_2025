#ifndef PEDAL_H
#define PEDAL_H

#include "Queue.h"
#include "mcp2515.h"

// Constants
const float APPS_PEDAL_1_MIN_VOLTAGE = 0.0;
const float APPS_PEDAL_1_MAX_VOLTAGE = 5.0;
const float APPS_PEDAL_2_MIN_VOLTAGE = 0.0;
const float APPS_PEDAL_2_MAX_VOLTAGE = 3.3;

const float FLOAT_APPS_PEDAL_1_RANGE = APPS_PEDAL_1_MAX_VOLTAGE - APPS_PEDAL_1_MIN_VOLTAGE;
const float FLOAT_APPS_PEDAL_2_RANGE = APPS_PEDAL_2_MAX_VOLTAGE - APPS_PEDAL_2_MIN_VOLTAGE;

const float FLOAT_APPS_PEDAL_1_LOWER_DEADZONE_WIDTH = 0.0;
const float FLOAT_APPS_PEDAL_1_UPPER_DEADZONE_WIDTH = 0.4;
// const float APPS_PEDAL_2_LOWER_DEADZONE_WIDTH = 0.0;
// const float APPS_PEDAL_2_UPPER_DEADZONE_WIDTH = 0.0;

const float FLOAT_MIN_THROTTLE_IN_VOLT = APPS_PEDAL_1_MIN_VOLTAGE + FLOAT_APPS_PEDAL_1_LOWER_DEADZONE_WIDTH / 2;
const float FLOAT_MAX_THROTTLE_IN_VOLT = APPS_PEDAL_1_MAX_VOLTAGE - FLOAT_APPS_PEDAL_1_UPPER_DEADZONE_WIDTH / 2;
const float FLOAT_THROTTLE_LOWER_DEADZONE_MIN_IN_VOLT = APPS_PEDAL_1_MIN_VOLTAGE - FLOAT_APPS_PEDAL_1_LOWER_DEADZONE_WIDTH / 2;
const float FLOAT_THROTTLE_UPPER_DEADZONE_MAX_IN_VOLT = APPS_PEDAL_1_MAX_VOLTAGE + FLOAT_APPS_PEDAL_1_UPPER_DEADZONE_WIDTH / 2;

// float to int16_t conversion
// The range of the pedal is 0-1023, 
const uint16_t MIN_THROTTLE_IN = FLOAT_MIN_THROTTLE_IN_VOLT * 1024 / FLOAT_APPS_PEDAL_1_RANGE;
const uint16_t MAX_THROTTLE_IN = FLOAT_MAX_THROTTLE_IN_VOLT * 1024 / FLOAT_APPS_PEDAL_1_RANGE;
const uint16_t THROTTLE_LOWER_DEADZONE_MIN_IN = FLOAT_THROTTLE_LOWER_DEADZONE_MIN_IN_VOLT * 1024 / FLOAT_APPS_PEDAL_1_RANGE;
const uint16_t THROTTLE_UPPER_DEADZONE_MAX_IN = FLOAT_THROTTLE_UPPER_DEADZONE_MAX_IN_VOLT * 1024 / FLOAT_APPS_PEDAL_1_RANGE;

const uint16_t MAX_THROTTLE_OUT_VAL = 32430; // Maximum torque value is 32760 for mcp2515
// currently set to a slightly lower value to not use speed control (100%)
// see E,EnS group discussion, 20250425HKT020800 discussion
const uint16_t MIN_THROTTLE_OUT_VAL = 300; // Minium torque value tested is 300 (TBC)

// Flips the direction of motor output
// set to true for gen 3
const bool flip_motor_dir = true;

#define ADC_BUFFER_SIZE 16

// Class for generic pedal object
// For Gen 5 car, only throttle pedal is wired through the VCU, so we use Pedal class for Throttle pedal only.
class Pedal
{
public:
    // Two input pins for reading both pedal potentiometer
    // Conversion rate in Hz
    Pedal(uint8_t input_pin_1, uint8_t input_pin_2, uint32_t millis, uint16_t conversion_rate = 1000);

    // Defualt constructor, expected another constructor should be called before start using
    Pedal();

    // Update function. To be called on every loop and pass the current time in millis
    void pedal_update(uint32_t millis);

    // Updates the can_frame with the most update pedal value. To be called on every loop and pass the can_frame by reference.
    void pedal_can_frame_update(can_frame *tx_throttle_msg);

    // Updates the can_frame to send a "0 Torque" value through canbus.
    void pedal_can_frame_stop_motor(can_frame *tx_throttle_msg);

    // Pedal value after filtering and processing
    // Under normal circumstance, should store a value between 0 and 1023 inclusive (translates to 0v - 5v)
    uint16_t final_pedal_value;

private:
    uint8_t input_pin_1, input_pin_2;

    // Will rollover every 49 days
    uint32_t previous_millis;

    uint8_t conversion_rate;

    // If the two potentiometer inputs are too different (> 10%), the inputs are faulty
    // Definition for faulty is under FSEC 2024 Chapter 2, section 12.8, 12.9
    bool fault = false;
    uint32_t fault_start_millis;

    // Forced stop the car due too long fault sensors, restart car to reset this to false
    bool fault_force_stop = true;

    // Period in millisecond
    uint8_t conversion_period;

    // Returns true if pedal is faulty
    bool check_pedal_fault(uint16_t pedal_1, uint16_t pedal_2);

    RingBuffer<float, ADC_BUFFER_SIZE> pedal_value_1;
    RingBuffer<float, ADC_BUFFER_SIZE> pedal_value_2;

    // throttle-torque mapping
    // input pedal value in 0-1023 range
    // output torque value in the SIGNED 300(?)-32760 range
    inline int16_t throttle_torque_mapping(uint16_t pedal, bool flip_motor_dir);
};

#endif // PEDAL_H