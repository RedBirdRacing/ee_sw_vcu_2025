#ifndef PEDAL_H
#define PEDAL_H

#include "Queue.h"
#include "mcp2515.h"

// Constants
const float APPS_PEDAL_1_MIN_VOLTAGE = 0.0;
const float APPS_PEDAL_1_MAX_VOLTAGE = 5.0;
const float APPS_PEDAL_2_MIN_VOLTAGE = 0.0;
const float APPS_PEDAL_2_MAX_VOLTAGE = 3.3;

const float APPS_PEDAL_1_RANGE = APPS_PEDAL_1_MAX_VOLTAGE - APPS_PEDAL_1_MIN_VOLTAGE;
const float APPS_PEDAL_2_RANGE = APPS_PEDAL_2_MAX_VOLTAGE - APPS_PEDAL_2_MIN_VOLTAGE;

const float APPS_PEDAL_1_LOWER_DEADZONE_WIDTH = 0.0;
const float APPS_PEDAL_1_UPPER_DEADZONE_WIDTH = 0.4;
// const float APPS_PEDAL_2_LOWER_DEADZONE_WIDTH = 0.0;
// const float APPS_PEDAL_2_UPPER_DEADZONE_WIDTH = 0.0;

const float MIN_THROTTLE_IN_VOLT = APPS_PEDAL_1_MIN_VOLTAGE + APPS_PEDAL_1_LOWER_DEADZONE_WIDTH;
const float MAX_THROTTLE_IN_VOLT = APPS_PEDAL_1_MAX_VOLTAGE - APPS_PEDAL_1_UPPER_DEADZONE_WIDTH;
const float THROTTLE_LOWER_DEADZONE_MIN_IN_VOLT = APPS_PEDAL_1_MIN_VOLTAGE - APPS_PEDAL_1_LOWER_DEADZONE_WIDTH;
const float THROTTLE_UPPER_DEADZONE_MAX_IN_VOLT = APPS_PEDAL_1_MAX_VOLTAGE + APPS_PEDAL_1_UPPER_DEADZONE_WIDTH;

const int MAX_THROTTLE_OUT_VAL = 32430; // Maximum torque value is 32760 for mcp2515
// current set to a slightly lower value to not use current control
// see E,EnS group discussion, 20250425HKT020800 discussion
const int MIN_THROTTLE_OUT_VAL = 300; // Minium torque value tested is 300 (TBC)

// To go forward, this should be true; false sets the motor to go in reverse
bool Flip_Motor_Dir = true; // Flips the direction of motor output
// set to true for gen 3

// Reverse mode "stationary" speed threshold
const float CAR_STATIONARY_SPEED_THRESHOLD = 0.2;
// Reverse mode entering brake threshold
const float REVERSE_ENTER_BRAKE_THRESHOLD = 0.5;
// Reverse mode entering throttle threshold
const float REVERSE_ENTER_THROTTLE_THRESHOLD = 0.1;
// Reverse mode maximum speed
const float REVERSE_SPEED_MAX = 0.2;
// Reverse mode buzzer cycle time
const unsigned short REVERSE_BEEP_CYCLE_TIME = 400; // in ms

#define ADC_BUFFER_SIZE 16

// Class for generic pedal object
// For Gen 5 car, only throttle pedal is wired through the VCU, so we use Pedal class for Throttle pedal only.
class Pedal
{
public:
    // Two input pins for reading both pedal potentiometer
    // Conversion rate in Hz
    Pedal(int input_pin_1, int input_pin_2, int reverse_pin, int buzzer_pin, unsigned long millis, unsigned short conversion_rate = 1000);

    // Defualt constructor, expected another constructor should be called before start using
    Pedal();

    // Update function. To be called on every loop and pass the current time in millis
    void pedal_update(unsigned long millis);

    // Updates the can_frame with the most update pedal value. To be called on every loop and pass the can_frame by reference.
    void pedal_can_frame_update(can_frame *tx_throttle_msg, unsigned long millis);

    // Updates the can_frame to send a "0 Torque" value through canbus.
    void pedal_can_frame_stop_motor(can_frame *tx_throttle_msg);

    // Pedal value after filtering and processing
    // Under normal circumstance, should store a value between 0 and 1023 inclusive (translates to 0v - 5v)
    int final_pedal_value;

private:
    int input_pin_1, input_pin_2, reverse_pin, buzzer_pin;

    // Will rollover every 49 days
    unsigned long previous_millis;

    unsigned short conversion_rate;

    // If the two potentiometer inputs are too different (> 10%), the inputs are faulty
    // Definition for faulty is under FSEC 2024 Chapter 2, section 12.8, 12.9
    bool fault = false;
    unsigned long fault_start_millis;

    // Forced stop the car due too long fault sensors, restart car to reset this to false
    bool fault_force_stop = true;

    // Period in millisecond
    unsigned short conversion_period;

    // Returns true if pedal is faulty
    bool check_pedal_fault(int pedal_1, int pedal_2);

    RingBuffer<float, ADC_BUFFER_SIZE> pedalValue_1;
    RingBuffer<float, ADC_BUFFER_SIZE> pedalValue_2;

    // reverse mode
    //
    // Do NOT use in actual competition!
    // Read documentation
    //

    // calculate reverse torque value
    int calculateReverseTorque(float throttleVolt, float vehicleSpeed, int torqueRequested);

    // reverse button pin to bool
    bool reverseButtonPressed = false;

    // Reverse mode status
    bool reverseMode = false;

    // function check and set reverse, return reverse mode status
    bool check_enter_reverse_mode(bool reverseButtonPressed, float brakePercentage, float throttlePercentage, float vehicleSpeed);

    // function check and set forward, return reverse mode status
    bool check_enter_forward_mode(bool reverseButtonPressed, float brakePercentage, float throttlePercentage, float vehicleSpeed);
};

#endif // PEDAL_H