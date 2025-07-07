#include "Pedal.h"
#include "Signal_Processing.cpp" // for AVG_filter, move to .h later
#include "Debug.h"
#include <Arduino.h> // for round() only.


Pedal::Pedal()
    : fault(true), fault_force_stop(false) {}

void Pedal::pedal_update(uint32_t millis, uint16_t pedal_1, uint16_t pedal_2)
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
    uint16_t pedal_filtered_1 = round(AVG_filter<float>(pedal_value_1.buffer, ADC_BUFFER_SIZE));
    uint16_t pedal_filtered_2 = round(AVG_filter<float>(pedal_value_2.buffer, ADC_BUFFER_SIZE));

    pedal_final = pedal_filtered_1; // Only take in pedal 1 value

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
        if (millis - fault_start_millis > 100)
        { // Faulty for more than 100 ms
            // TODO: Add code for alerting the faulty pedal, and whatever else mandated in rules Ch.2 Section 12.8, 12.9
            // Rules actually states there's no need to alert the driver, stopping motor output is enough
            // Alerting the driver would be done if the digital dash is made, will have to see

            // Turning off the motor is achieved using another digital pin, not via canbus, but will still send 0 torque can signals
            fault_force_stop = true;
            // this force stop flag can only be reset by a power cycle

            DBG_THROTTLE_FAULT(DIFF_EXCEED_100MS);
            // DBGLN_THROTTLE("FAULT: Pedal mismatch persisted > 100ms!");

            // -- Debug: Pedal faulty too long

            return;
        }
    }
    else
    {
        fault_start_millis = millis;
        DBG_THROTTLE_FAULT(DIFF_START);
        // DBGLN_THROTTLE("FAULT: Pedal mismatch started");
    }

    fault = true;
    return;
}

void Pedal::pedal_can_frame_stop_motor(can_frame *tx_throttle_msg)
{
    tx_throttle_msg->can_id = MOTOR_COMMAND;
    tx_throttle_msg->can_dlc = 3;
    tx_throttle_msg->data[0] = 0x90; // 0x90 for torque, 0x31 for speed
    tx_throttle_msg->data[1] = 0x00;
    tx_throttle_msg->data[2] = 0x00;

    DBG_THROTTLE("Stopping motor due to: ");
}

void Pedal::pedal_can_frame_update(can_frame *tx_throttle_msg)
{
    if (fault_force_stop)
    {
        pedal_can_frame_stop_motor(tx_throttle_msg);

        DBGLN_THROTTLE("pedal fault.");
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
    if (pedal_final < PEDAL_1_LL)
    {
        DBG_THROTTLE_FAULT(THROTTLE_LOW, pedal_final);
        // DBG_THROTTLE_FAULT(THROTTLE_TOO_LOW "Throttle voltage too low");
        throttle_torque_val = 0;
    }
    else if (pedal_final < PEDAL_1_LU)
    {
        throttle_torque_val = MIN_THROTTLE_OUT_VAL;
    }
    else if (pedal_final < PEDAL_1_UL)
    {
        // Scale up the value for canbus
        throttle_torque_val = throttle_torque_mapping(pedal_final, FLIP_MOTOR_DIR);
    }
    else if (pedal_final < PEDAL_1_UU)
    {
        throttle_torque_val = MAX_THROTTLE_OUT_VAL;
    }
    else
    {
        // DBG_THROTTLE("Throttle voltage too high");
        DBG_THROTTLE_FAULT(THROTTLE_HIGH, pedal_final);
        // For safety, this should not be set to other values
        throttle_torque_val = 0;
    }

    DBG_THROTTLE_OUT(pedal_final, throttle_torque_val);
    // DBG_THROTTLE("CAN UPDATE: Throttle = ");

    tx_throttle_msg->can_id = MOTOR_COMMAND;
    tx_throttle_msg->can_dlc = 3;
    tx_throttle_msg->data[0] = 0x90; // 0x90 for torque, 0x31 for speed
    tx_throttle_msg->data[1] = throttle_torque_val & 0xFF;
    tx_throttle_msg->data[2] = (throttle_torque_val >> 8) & 0xFF;
}

inline int16_t Pedal::throttle_torque_mapping(uint16_t throttle, bool flip_motor_dir)
{
    // Map the throttle voltage to a torque value
    // temp linear mapping
    if (flip_motor_dir)
    {
        return (PEDAL_1_LU - throttle) * MAX_THROTTLE_OUT_VAL / (PEDAL_1_UL - PEDAL_1_LU);
    }
    return (throttle - PEDAL_1_LU) * MAX_THROTTLE_OUT_VAL / (PEDAL_1_UL - PEDAL_1_LU);
}

bool Pedal::check_pedal_fault(uint16_t pedal_1, uint16_t pedal_2)
{
    uint16_t pedal_2_scaled = round(pedal_2 * (PEDAL_1_RANGE / PEDAL_2_RANGE));
    uint16_t delta;
    if (pedal_1 > pedal_2_scaled)
        delta = pedal_1 - pedal_2_scaled;
    else
        delta = pedal_2_scaled - pedal_1;
    DBG_THROTTLE_IN(pedal_1, pedal_2, pedal_2_scaled);

    // Currently the only indication for faulty pedal is just 2 pedal values are more than 10% different

    if (delta > 102.4) // 10% of 1024
    {
        DBG_THROTTLE_FAULT(DIFF_CONTINUING, delta);
        // DBGLN_THROTTLE("WARNING: Pedal mismatch > 10%");
        return true;
    }
    return false;
}
