#ifndef ENUMS_H
#define ENUMS_H

/* === Car Status State Machine ===
Meaning of different car statuses
INIT (0):  Just started the car
STARTIN (1):  1st Transition state -- Driver holds the "Start" button and is on full brakes, lasts for STATUS_1_TIME_MILLIS milliseconds
BUSSIN (2):  2nd Transition state -- Buzzer bussin, driver can release "Start" button and brakes
DRIVE (3):  Ready to drive -- Motor starts responding according to the driver pedal input. "Drive mode" LED lights up, indicating driver can press the throttle

Separately, the following will be done outside the status checking part:
1.  Before the "Drive mode" LED lights up, if the throttle pedal is pressed (Throttle input is not equal to 0), the car_status will return to 0
2.  Before the "Drive mode" LED lights up, the canbus will keep sending "0 torque" messages to the motor

Also, during status 0, 1, and 2, the VCU will keep sending "0 torque" messages to the motor via CAN
*/

/**
 * @brief Main car status state machine.
 *
 * Represents the current state of the car in the startup and drive sequence.
 */
enum main_car_status
{
    INIT = 0,     // Just started the car
    STARTIN = 1,  // Driver holds "Start" button and full brakes (transition state)
    BUSSIN = 2,   // Buzzer active, driver can release "Start" and brakes (transition state)
    DRIVE = 3     // Ready to drive, "Drive mode" LED on, throttle enabled
};

/**
 * @brief Pedal fault status codes.
 *
 * Represents the current fault status related to pedal input.
 */
enum pedal_fault_status
{
    NONE = 0,             // No fault detected
    DIFF_START = 1,       // >10% difference fault just started
    DIFF_CONTINUING = 2,  // >10% difference fault is ongoing
    DIFF_EXCEED_100MS = 3,// >10% difference fault exceeded 100ms
    DIFF_RESOLVED = 4,    // Difference fault resolved
    THROTTLE_LOW = 5,     // Throttle pedal below lower threshold
    THROTTLE_HIGH = 6     // Throttle pedal above upper threshold
};

/**
 * @brief CAN message IDs for throttle and debug.
 *
 * Used for sending throttle and debug messages over CAN bus.
 */
enum throttle_can_id
{
    MOTOR_COMMAND = 0x201,        // Main motor command message
    THROTTLE_IN_MSG = 0x690,      // Debug: throttle pedal input message
    THROTTLE_OUT_MSG = 0x691,     // Debug: throttle output message
    THROTTLE_FAULT_MSG = 0x692    // Debug: throttle fault message
};

/**
 * @brief CAN message IDs for status and brake debug.
 *
 * Used for sending car status and brake messages over CAN bus.
 */
enum status_can_id
{
    STATUS_CAR_MSG = 0x693,         // Debug: car status message
    STATUS_CAR_CHANGE_MSG = 0x694,  // Debug: car status change message
    STATUS_BRAKE_MSG = 0x695        // Debug: brake status message
};

/**
 * @brief State change events for the car status state machine.
 *
 * Used to represent transitions between main car states.
 */
enum state_changes
{
    INIT_TO_STARTIN,      // Transition from INIT to STARTIN
    STARTIN_TO_BUSSIN,    // Transition from STARTIN to BUSSIN
    BUSSIN_TO_DRIVE,      // Transition from BUSSIN to DRIVE
    STARTIN_TO_INIT,      // Transition from STARTIN back to INIT
    THROTTLE_TO_INIT      // Transition to INIT due to throttle pedal input

#endif // Enums.h