#include "Pedal.h"
#include "Signal_Processing.cpp" // for AVG_filter, move to .h later
#include "Debug.h"
#include <Arduino.h> // for round() only.


Pedal::Pedal()
    : fault(true), fault_force_stop(false) {}



void Pedal::pedal_update(car_state *car, uint16_t pedal_1, uint16_t pedal_2)
{
    // Record readings in buffer
    pedal_value_1.push(pedal_1);
    pedal_value_2.push(pedal_2);

    // Range of pedal 1 is APPS_PEDAL_1_RANGE, pedal 2 is APPS_PEDAL_2_RANGE;

    // this is current taking the direct array the circular queue writes into. Bad idea to do anything other than a simple average
    // if not using a linear filter, pass the pedalValue_1.getLinearBuffer() to the filter function to ensure the ordering is correct.
    // can also consider injecting the filter into the queue if need
    // depends on the hardware filter, reduce software filtering as much as possible

    // currently a small average filter is good enough
    pedal_filtered_1 = round(AVG_filter<float>(pedal_value_1.buffer, ADC_BUFFER_SIZE));
    pedal_filtered_2 = round(AVG_filter<float>(pedal_value_2.buffer, ADC_BUFFER_SIZE));

    car->pedal_final = pedal_filtered_1; // Only take in pedal 1 value

    if (!check_pedal_fault(pedal_filtered_1, pedal_filtered_2))
    {
        if (fault)
            DBG_THROTTLE_FAULT(DIFF_RESOLVED);
        fault = false;
        return;
    }

    // Pedal fault detected
    if (fault)
    { // Previous scan is already faulty
        if (car->millis - fault_start_millis > 100) // Faulty for more than 100 ms
        {
            // Turning off the motor is achieved using another digital pin, not via canbus, but will still send 0 torque can signals
            car->fault_force_stop = true;
            // this force stop flag can only be reset by a power cycle

            DBG_THROTTLE_FAULT(DIFF_EXCEED_100MS);
            // DBGLN_THROTTLE("FAULT: Pedal mismatch persisted > 100ms!");

            // -- Debug: Pedal faulty too long

            return;
        }
    }
    else
    {
        fault_start_millis = car->millis;
        DBG_THROTTLE_FAULT(DIFF_START);
        // DBGLN_THROTTLE("FAULT: Pedal mismatch started");
    }

    fault = true;
    return;
}

void Pedal::pedal_can_frame_stop_motor(can_frame *tx_throttle_msg, const char reason[])
{
    tx_throttle_msg->can_id = MOTOR_COMMAND;
    tx_throttle_msg->can_dlc = 3;
    tx_throttle_msg->data[0] = 0x90; // 0x90 for torque, 0x31 for speed
    tx_throttle_msg->data[1] = 0x00;
    tx_throttle_msg->data[2] = 0x00;

    DBG_THROTTLE("Stopping motor due to: ");
    DBGLN_THROTTLE(reason); // default reason is "unknown reason."
}

void Pedal::pedal_can_frame_update(can_frame *tx_throttle_msg, car_state *car)
{
    if (fault_force_stop)
    {
        pedal_can_frame_stop_motor(tx_throttle_msg, "pedal fault.");
        return;
    }
    // uint8_t throttle_volt = pedal_final * APPS_PEDAL_1_RANGE / 1024; // Converts most update pedal value to a float between 0V and 5V

    int16_t throttle_torque_val = 0;
    /*
    Between 0V and THROTTLE_LOWER_DEADZONE_MAX_IN_VOLT: Error for open circuit
    Between THROTTLE_LOWER_DEADZONE_MAX_IN_VOLT and MIN_THROTTLE_IN_VOLT: 0% Torque
    Between MIN_THROTTLE_IN_VOLT and MAX_THROTTLE_IN_VOLT: Linear relationship
    Between MAX_THROTTLE_IN_VOLT and THORTTLE_UPPER_DEADZONE_MIN_IN_VOLT: 100% Torque
    Between THORTTLE_UPPER_DEADZONE_MIN_IN_VOLT and 5V: Error for short circuit
    */
    if (car->pedal_final < PEDAL_1_LL)
    {
        DBG_THROTTLE_FAULT(THROTTLE_LOW, car->pedal_final);
        throttle_torque_val = 0;
    }
    else if (car->pedal_final < PEDAL_1_LU)
    {
        // in lower deadzone, treat as 0% throttle
        throttle_torque_val = MIN_THROTTLE_OUT_VAL;
    }
    else if (car->pedal_final < PEDAL_1_UL)
    {
        // throttle_in -> torque_val
        // check mapping function for curve
        throttle_torque_val = throttle_torque_mapping(car->pedal_final, FLIP_MOTOR_DIR);
    }
    else if (car->pedal_final < PEDAL_1_UU)
    {
        // in upper deadzone, treat as 100% throttle
        throttle_torque_val = MAX_THROTTLE_OUT_VAL;
    }
    else
    {
        DBG_THROTTLE_FAULT(THROTTLE_HIGH, car->pedal_final);
        // throttle higher than upper deadzone, treat as throttle fault, zeroing torque for safety
        throttle_torque_val = 0;
    }

    DBG_THROTTLE_OUT(car->pedal_final, throttle_torque_val);

    tx_throttle_msg->can_id = MOTOR_COMMAND;
    tx_throttle_msg->can_dlc = 3;
    tx_throttle_msg->data[0] = 0x90; // 0x90 for torque, 0x31 for speed
    tx_throttle_msg->data[1] = throttle_torque_val & 0xFF;
    tx_throttle_msg->data[2] = (throttle_torque_val >> 8) & 0xFF;
}

int16_t Pedal::throttle_torque_mapping(uint16_t throttle, bool flip_motor_dir)
{
    if (throttle < PEDAL_1_LU || throttle > PEDAL_1_UL)
    {
        // throttle is out of range, return 0 torque
        return 0;
    }
    // Map the throttle voltage to a torque value
    // temp linear mapping, with proper casting to prevent overflow
    int32_t numerator = static_cast<int32_t>(throttle - PEDAL_1_LU) * static_cast<int32_t>(MAX_THROTTLE_OUT_VAL);
    int32_t denominator = static_cast<int32_t>(PEDAL_1_UL - PEDAL_1_LU);
    int16_t result = static_cast<int16_t>(numerator / denominator);

    if (flip_motor_dir)
    {
        return -result;
    }
    return result;
}

bool Pedal::check_pedal_fault(uint16_t pedal_1, uint16_t pedal_2)
{
    uint16_t pedal_2_scaled = round((float)pedal_2 * PEDAL_1_RANGE / PEDAL_2_RANGE);
    uint16_t delta;
    if (pedal_1 > pedal_2_scaled)
        delta = pedal_1 - pedal_2_scaled;
    else
        delta = pedal_2_scaled - pedal_1;
    DBG_THROTTLE_IN(pedal_1, pedal_2, pedal_2_scaled);

    // if more than 10% difference between the two pedals, consider it a fault
    if (delta > 102.4) // 10% of 1024
    {
        DBG_THROTTLE_FAULT(DIFF_CONTINUING, delta);
        return true;
    }
    return false;
}
