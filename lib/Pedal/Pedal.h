#ifndef PEDAL_H
#define PEDAL_H

#include <stdint.h>
#include "Queue.h"
#include <can.h> // for can_frame
#include "car_state.h"
#include <Arduino.h> // for string

// Constants

/*
    Dear Users:
    First record the "normal" range of the pedals.
    You need to first flash the datalogger firmware to the VCU, then connect the VCU to the car.
    If normally, the reading is from 69 to 948,
    set PEDAL_X_MIN_VOLTAGE to 69 and PEDAL_X_MAX_VOLTAGE to 948.
*/

const uint16_t PEDAL_1_IN_MIN = 69; // Minimum ADC reading for APPS Pedal 1 (5V)
const uint16_t PEDAL_1_IN_MAX = 948; // Maximum ADC reading for APPS Pedal 1 (5V)
const uint16_t PEDAL_2_IN_MIN = 0; // Minimum ADC reading for APPS Pedal 2 (3.3V)
const uint16_t PEDAL_2_IN_MAX = 675; // Maximum ADC reading for APPS Pedal 2 (3.3V)

/*
    Second, set the deadzone. a deadzone is a range of reading that the pedal will not respond to.
    An upper deadzone of 41 for a MAX_VOLTAGE of 980 means that the the pedal is treated as "full" for 960-1000
    Reading higher than 1000 is considered as "faulty"
    While it's better to have an odd number for deadzone, it is not required. This is because the tree planting problem; there are 41 trees for 40 segments.
*/

const uint16_t PEDAL_1_LOWER_DEADZONE_WIDTH = 31;
const uint16_t PEDAL_1_UPPER_DEADZONE_WIDTH = 31;
// const uint16_t PEDAL_2_LOWER_DEADZONE_WIDTH = 31;
// const uint16_t PEDAL_2_UPPER_DEADZONE_WIDTH = 31;

const uint16_t PEDAL_1_RANGE = PEDAL_1_IN_MAX - PEDAL_1_IN_MIN;
const uint16_t PEDAL_2_RANGE = PEDAL_2_IN_MAX - PEDAL_2_IN_MIN;

/* Pedal Voltage Mapping
|······••••••••••••••••••••••••••••••••••••••••••······|
|LL--LU------------------------------------------UL--UU|
LL-LU: Lower Deadzone, treat as zero, LL at least 0V (uint)
UL-UU: Upper Deadzone, treat as full
Pedal only "outputs" when between LU and UL, i.e. in ••••••
above UU and below LL is considered "faulty"
e.g. UU of 1023 means 1023, 1024... are faulty
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

const uint16_t MAX_THROTTLE_OUT_VAL = 32430; // Maximum torque value is 32760 for mcp2515
// currently set to a slightly lower value to not use speed control (100%)
// see E,EnS group discussion, 20250425HKT020800 discussion
const uint16_t MIN_THROTTLE_OUT_VAL = 300; // Minium torque value tested is 300 (TBC)

// Flips the direction of motor output
// set to true for gen 3
const bool FLIP_MOTOR_DIR = true;

#define ADC_BUFFER_SIZE 16

/*
// mock CAN frame structure, decoupled from MCP2515 library for easier testing and debugging
// CAN payload length and DLC definitions according to ISO 11898-1
struct can_frame
{
    uint32_t can_id; // 32 bit CAN_ID + EFF/RTR/ERR flags
    uint8_t can_dlc; // frame payload length in byte (0 .. CAN_MAX_DLEN==8)
    uint8_t data[8] __attribute__((aligned(8)));
};
*/

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
    
public:
    // Default constructor
    Pedal();
    // Constructor with

    // Update function. To be called on every loop and pass the current time in millis
    void pedal_update(car_state *main_car_state, uint16_t pedal_1, uint16_t pedal_2);

    // Updates the can_frame with the most update pedal value. To be called on every loop and pass the can_frame by reference.
    void pedal_can_frame_update(can_frame *tx_throttle_msg, car_state *car);

    // Updates the can_frame to send a "0 Torque" value through canbus.
    void pedal_can_frame_stop_motor(can_frame *tx_throttle_msg, const char reason[] = "unknown reason.");

    // Pedal value after filtering and processing
    // Under normal circumstances, should store a value between 0 and 1023 inclusive (translates to 0v - 5v)
    uint16_t pedal_filtered_1, pedal_filtered_2;

private:
    // If the two potentiometer inputs are too different (> 10%), the inputs are faulty
    // Definition for faulty is under FSEC 2024 Chapter 2, section 12.8, 12.9
    bool fault = true;
    uint32_t fault_start_millis; // rollover in 49.7 days

    // Forced stop the car due too long fault sensors, restart car to reset this to false
    // Class initializer sets this to false, pedal fault > 100ms trips to true
    bool fault_force_stop = true;

    // Two cyclic queues for storing the pedal values
    // Used for filter, right now average of the last 16 values
    RingBuffer<float, ADC_BUFFER_SIZE> pedal_value_1;
    RingBuffer<float, ADC_BUFFER_SIZE> pedal_value_2;

    // Returns true if pedal is faulty
    // should be inlined in .cpp
    // not inlined now for easier testing
    bool check_pedal_fault(uint16_t pedal_1, uint16_t pedal_2);

    // throttle-torque mapping
    // input pedal value in 0-1023 range
    // output torque value in the SIGNED 300(?)-32760 range
    // should be inlined in .cpp
    // not inlined now for easier testing
    int16_t throttle_torque_mapping(uint16_t pedal, bool flip_motor_dir);
};

#endif // PEDAL_H