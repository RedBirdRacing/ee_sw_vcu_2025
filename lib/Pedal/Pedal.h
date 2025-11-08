#ifndef PEDAL_H
#define PEDAL_H

#include <stdint.h>
#include "Queue.h"
#include <can.h> // for can_frame
#include "car_state.h"
#include <Arduino.h> // for string

#define REGEN_ENABLED false // Set to true to enable regenerative braking, false to disable

// Constants

/*
    Dear Users:
    First record the "normal" range of the pedals.
    You need to first flash the datalogger firmware to the VCU, then connect the VCU to the car.
    If normally, the reading is from 69 to 948,
    set PEDAL_X_MIN_VOLTAGE to 69 and PEDAL_X_MAX_VOLTAGE to 948.
*/

const uint16_t PEDAL_1_IN_MIN = 465; // Minimum ADC reading for APPS Pedal 1 (5V)
const uint16_t PEDAL_1_IN_MAX = 810; // Maximum ADC reading for APPS Pedal 1 (5V)
const uint16_t PEDAL_2_IN_MIN = 300; // Minimum ADC reading for APPS Pedal 2 (3.3V)
const uint16_t PEDAL_2_IN_MAX = 560; // Maximum ADC reading for APPS Pedal 2 (3.3V)
const uint16_t BRAKE_IN_MIN = 123; // Minimum ADC reading for Brake Pedal
const uint16_t BRAKE_IN_MAX = 191; // Maximum ADC reading for Brake Pedal

/*
    Second, set the deadzone. a deadzone is a range of reading that the pedal will not respond to.
    An upper deadzone of 41 for a MAX_VOLTAGE of 980 means that the the pedal is treated as "full" for 960-1000
    Reading higher than 1000 is considered as "faulty"
    While it's better to have an odd number for deadzone, it is not required. This is because the tree planting problem; there are 41 trees for 40 segments.
*/

const uint16_t PEDAL_1_LOWER_DEADZONE_WIDTH = 27;
const uint16_t PEDAL_1_UPPER_DEADZONE_WIDTH = 21;
// const uint16_t PEDAL_2_LOWER_DEADZONE_WIDTH = 3;
// const uint16_t PEDAL_2_UPPER_DEADZONE_WIDTH = 3;
const uint16_t BRAKE_LOWER_DEADZONE_WIDTH = 9;
const uint16_t BRAKE_UPPER_DEADZONE_WIDTH = 9;

/*
    Data entry ends here.
*/

const uint16_t PEDAL_1_RANGE = PEDAL_1_IN_MAX - PEDAL_1_IN_MIN;
const uint16_t PEDAL_2_RANGE = PEDAL_2_IN_MAX - PEDAL_2_IN_MIN;
const uint16_t BRAKE_RANGE = BRAKE_IN_MAX - BRAKE_IN_MIN;

/* Pedal Voltage Mapping
|······••••••••••••••••••••••••••••••••••••••••••······|
|LL--LU------------------------------------------UL--UU|
LL-LU: Lower Deadzone, treat as zero, LL at least 0V (uint)
UL-UU: Upper Deadzone, treat as full
Pedal only "outputs" when between LU and UL, i.e. in ••••••
above UU and below LL is considered "faulty"
e.g. UU of 1000 means 1000, 1001... are faulty
thus if throttle should be saturated, UU should be greater than or equal to 1024
*/
const uint16_t PEDAL_1_LL = (PEDAL_1_IN_MIN < (PEDAL_1_LOWER_DEADZONE_WIDTH - 1) / 2)
                           ? 0.0f
                           : (PEDAL_1_IN_MIN - (PEDAL_1_LOWER_DEADZONE_WIDTH - 1) / 2);
const uint16_t PEDAL_1_LU = (PEDAL_1_IN_MIN < (PEDAL_1_LOWER_DEADZONE_WIDTH - 1) / 2)
                           ? PEDAL_1_LOWER_DEADZONE_WIDTH
                           : (PEDAL_1_IN_MIN + (PEDAL_1_LOWER_DEADZONE_WIDTH - 1) / 2);
const uint16_t PEDAL_1_UL = PEDAL_1_IN_MAX - (PEDAL_1_UPPER_DEADZONE_WIDTH - 1) / 2;
const uint16_t PEDAL_1_UU = PEDAL_1_IN_MAX + (PEDAL_1_UPPER_DEADZONE_WIDTH - 1) / 2;
// since only take pedal 1, no need to define pedal 2 deadzones, directly coupled to pedal 1's deadzones
// pedal_final deadzones
const uint16_t PEDAL_LL = PEDAL_1_LL;
const uint16_t PEDAL_LU = PEDAL_1_LU;
const uint16_t PEDAL_UL = PEDAL_1_UL;
const uint16_t PEDAL_UU = PEDAL_1_UU;

const uint16_t BRAKE_LL = (BRAKE_IN_MIN < (BRAKE_LOWER_DEADZONE_WIDTH - 1) / 2)
                           ? 0.0f
                           : (BRAKE_IN_MIN - (BRAKE_LOWER_DEADZONE_WIDTH - 1) / 2);
const uint16_t BRAKE_LU = (BRAKE_IN_MIN < (BRAKE_LOWER_DEADZONE_WIDTH - 1) / 2)
                            ? BRAKE_LOWER_DEADZONE_WIDTH
                            : (BRAKE_IN_MIN + (BRAKE_LOWER_DEADZONE_WIDTH - 1) / 2);
const uint16_t BRAKE_UL = BRAKE_IN_MAX - (BRAKE_UPPER_DEADZONE_WIDTH - 1) / 2;
const uint16_t BRAKE_UU = BRAKE_IN_MAX + (BRAKE_UPPER_DEADZONE_WIDTH - 1) / 2;


const uint16_t MAX_THROTTLE_OUT_VAL = 32430; // Maximum torque value is 32760 for mcp2515
// currently set to a slightly lower value to not use speed control (100%)
// see E,EnS group discussion, 20250425HKT020800 discussion
const uint16_t MAX_REGEN = 20000;
const uint16_t MIN_REGEN = 0; // pedal off regen value
const uint16_t MIN_THROTTLE_OUT_VAL = 0; // 0 for off pedal regen

// Flips the direction of motor output
// set to true for gen 3
// false for gen 5
// later make class member for future dev
const bool FLIP_MOTOR_DIR = false;

#define ADC_BUFFER_SIZE 16

// Class for generic pedal object
// For Gen 5 car, only throttle pedal is wired through the VCU, so we use Pedal class for Throttle pedal only.
class Pedal
{
    // Allow test code to access private members
    friend void test_pedal_update_no_fault(void);
    friend void test_pedal_update_fault(void);
    friend void test_pedal_can_frame_stop_motor(void);
    friend void test_throttle_torque_mapping_normal(void);
    friend void test_check_pedal_fault(void);
    friend void setup(void);
    friend void loop(void);
    friend void test_brake_torque_mapping(void);
    
public:
    Pedal();
    void pedal_update(car_state *main_car_state, uint16_t pedal_1, uint16_t pedal_2, uint16_t brake);
    void pedal_can_frame_update(can_frame *tx_throttle_msg, car_state *car);
    void pedal_can_frame_stop_motor(can_frame *tx_throttle_msg);
    uint16_t pedal_filtered_1, pedal_filtered_2;

private:
    // If the two potentiometer inputs are too different (> 10%), the inputs are faulty
    // Definition for faulty is under FSEC 2024 Chapter 2, section 12.8, 12.9
    bool fault = true;
    uint32_t fault_start_millis; // rollover in 49.7 days

    // Two cyclic queues for storing the pedal values
    // Used for filter, right now average of the last 16 values
    RingBuffer<uint16_t, ADC_BUFFER_SIZE> pedal_value_1;
    RingBuffer<uint16_t, ADC_BUFFER_SIZE> pedal_value_2;
    RingBuffer<uint16_t, ADC_BUFFER_SIZE> brake_value;

    bool check_pedal_fault(int16_t pedal_1, int16_t pedal_2, int16_t brake);
    int16_t throttle_torque_mapping(uint16_t pedal, uint16_t brake, bool flip_dir);
    int16_t brake_torque_mapping(uint16_t brake, bool flip_dir);
};

#endif // PEDAL_H