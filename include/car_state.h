#ifndef CAR_STATE_H
#define CAR_STATE_H

#include "Enums.h"
#include <stdint.h>

struct car_state
{
    main_car_status car_status;         // Current car status
    uint32_t car_status_millis_counter; // Millis counter for the current car status
    uint32_t millis;                    // Millis counter for the current loop iteration
    //uint16_t pedal_1_in;                // The FILTERED input value of the first pedal (APPS 5V)
    //uint16_t pedal_2_in;                // The FILTERED input value of the second pedal (APPS 3V3)
    uint16_t pedal_final;               // The final pedal value after filtering and processing, normalized to 0-1023 range
    bool fault_force_stop;              // fault force stop flag
    uint16_t torque_out;                // The torque output to the motor
};
#endif // CAR_STATE_H