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
enum main_car_status
{
    INIT = 0,
    STARTIN = 1,
    BUSSIN = 2,
    DRIVE = 3
};

enum pedal_fault_status
{
    NONE = 0,
    DIFF_START = 1, // DIFF mean "Difference", as in the >10% difference fault
    DIFF_CONTINUING = 2,
    DIFF_EXCEED_100MS = 3,
    DIFF_RESOLVED = 4,
    THROTTLE_LOW = 5,
    THROTTLE_HIGH = 6
};

// CAN message IDs -- debug CAN should start from 0x690
enum throttle_can_id {
    MOTOR_COMMAND       = 0x201,
    // Debug CAN messages
    THROTTLE_IN_MSG     = 0x690,
    THROTTLE_OUT_MSG    = 0x691,
    THROTTLE_FAULT_MSG  = 0x692
};
enum status_can_id {
    STATUS_CAR_MSG    = 0x693,
    STATUS_BRAKE_MSG  = 0x694,
};

#endif // Enums.h