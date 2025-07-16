#ifndef CAR_STATE_H
#define CAR_STATE_H

#include "Enums.h"
#include <stdint.h>

/**
 * @brief Represents the state of the car.
 *
 * This structure holds all relevant runtime information about the car's status,
 * timing, pedal input, fault flags, and output torque. It is updated each loop
 * iteration and used throughout the control logic.
 * @param car_status                Current main state of the car (e.g., INIT, DRIVE, etc.).
 * @param car_status_millis_counter Millisecond counter for state transitions.
 * @param millis                    Current time in milliseconds.
 * @param pedal_final               Final processed pedal value.
 * @param fault_force_stop          True if a fault has triggered a forced stop.
 * @param torque_out                Output torque value.
 */
struct car_state
{
    main_car_status car_status;         // Current main state of the car (e.g., INIT, DRIVE, etc.)
    uint32_t car_status_millis_counter; // Millisecond counter for the current car status (for state transitions)
    uint32_t millis;                    // Current time in milliseconds for the current loop iteration
    // uint16_t pedal_1_in;             // Filtered input value of the first pedal (APPS 5V)
    // uint16_t pedal_2_in;             // Filtered input value of the second pedal (APPS 3V3)
    uint16_t pedal_final;               // Final processed pedal value, normalized to 0-1023 range
    bool fault_force_stop;              // Whether an APPS fault has triggered a forced stop
    uint16_t torque_out;                // Output torque value sent to the motor
};
#endif // CAR_STATE_H