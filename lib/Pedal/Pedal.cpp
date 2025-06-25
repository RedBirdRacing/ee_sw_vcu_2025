#include "Pedal.h"
#include "Arduino.h"
#include "Signal_Processing.cpp"
#include "Debug.h"

// Sinc function of size 128
float SINC_128[128] = {0.017232, 0.002666, -0.013033, -0.026004, -0.032934, -0.031899, -0.022884, -0.007851, 0.009675, 0.025427,
                       0.035421, 0.036957, 0.029329, 0.014081, -0.005294, -0.024137, -0.037732, -0.042472, -0.036792, -0.021652,
                       -0.000402, 0.021937, 0.039841, 0.048626, 0.045647, 0.031053, 0.007888, -0.018512, -0.041722, -0.055750,
                       -0.056553, -0.043139, -0.017994, 0.013320, 0.043353, 0.064476, 0.070758, 0.059540, 0.032321, -0.005306,
                       -0.044714, -0.076126, -0.090908, -0.083781, -0.054402, -0.007911, 0.045791, 0.093940, 0.123670, 0.125067,
                       0.093855, 0.033095, -0.046569, -0.128280, -0.191785, -0.217229, -0.189201, -0.100224, 0.047040, 0.239389,
                       0.454649, 0.664997, 0.841471, 0.958851, 1, 0.958851, 0.841471, 0.664997, 0.454649, 0.239389, 0.047040,
                       -0.100224, -0.189201, -0.217229, -0.191785, -0.128280, -0.046569, 0.033095, 0.093855, 0.125067, 0.123670,
                       0.093940, 0.045791, -0.007911, -0.054402, -0.083781, -0.090908, -0.076126, -0.044714, -0.005306, 0.032321,
                       0.059540, 0.070758, 0.064476, 0.043353, 0.013320, -0.017994, -0.043139, -0.056553, -0.055750, -0.041722,
                       -0.018512, 0.007888, 0.031053, 0.045647, 0.048626, 0.039841, 0.021937, -0.000402, -0.021652, -0.036792,
                       -0.042472, -0.037732, -0.024137, -0.005294, 0.014081, 0.029329, 0.036957, 0.035421, 0.025427, 0.009675,
                       -0.007851, -0.022884, -0.031899, -0.032934, -0.026004, -0.013033};

Pedal::Pedal()
    : input_pin_1(-1), input_pin_2(-1), previous_millis(0), conversion_rate(0), fault(true), fault_force_stop(false) {}

Pedal::Pedal(uint8_t input_pin_1, uint8_t input_pin_2, uint32_t millis, uint16_t conversion_rate)
    : input_pin_1(input_pin_1), input_pin_2(input_pin_2), previous_millis(millis), conversion_rate(conversion_rate), fault(false), fault_force_stop(false)
{
    // Init pins
    pinMode(input_pin_1, INPUT);
    pinMode(input_pin_2, INPUT);
    conversion_period = 1000 / conversion_rate;

    // Init ADC buffers
    for (uint8_t i = 0; i < ADC_BUFFER_SIZE; ++i)
    {
        pedal_value_1.buffer[i] = 0;
        pedal_value_2.buffer[i] = 0;
    }

    // -- Debug: Pedal initialized
    DBGLN_THROTTLE("Throttle Pedal initialized");
    DBGLN_GENERAL("Throttle Pedal initialized");
}

void Pedal::pedal_update(uint32_t millis)
{
    // If is time to update
    if (millis - previous_millis > conversion_period)
    {
        // Updating the previous millis
        previous_millis = millis;
        // Record readings in buffer
        pedal_value_1.push(analogRead(input_pin_1));
        pedal_value_2.push(analogRead(input_pin_2));

        // By default range of pedal 1 is APPS_PEDAL_1_RANGE, pedal 2 is APPS_PEDAL_2_RANGE;

        // this is current taking the direct array the circular queue writes into. Bad idea to do anything other than a simple average
        // if not using a linear filter, pass the pedalValue_1.getLinearBuffer() to the filter function to ensure the ordering is correct.
        // can also consider injecting the filter into the queue if need
        // depends on the hardware filter, reduce software filtering as much as possible
        uint16_t pedal_filtered_1 = round(AVG_filter<float>(pedal_value_1.buffer, ADC_BUFFER_SIZE));
        uint16_t pedal_filtered_2 = round(AVG_filter<float>(pedal_value_2.buffer, ADC_BUFFER_SIZE));

        // int pedal_filtered_1 = round(FIR_filter<float>(pedalValue_1.buffer, SINC_128, ADC_BUFFER_SIZE, 6.176445));
        // int pedal_filtered_2 = round(FIR_filter<float>(pedalValue_2.buffer, SINC_128, ADC_BUFFER_SIZE, 6.176445));
        final_pedal_value = pedal_filtered_1; // Only take in pedal 1 value

        DBG_THROTTLE_IN(pedal_filtered_1, pedal_filtered_2, final_pedal_value);

        // Debug: Pedal input values, filtered values

        if (check_pedal_fault(pedal_filtered_1, pedal_filtered_2))
        {
            if (fault)
            { // Previous scan is already faulty
                if (millis - fault_start_millis > 100)
                { // Faulty for more than 100 ms
                    // TODO: Add code for alerting the faulty pedal, and whatever else mandated in rules Ch.2 Section 12.8, 12.9
                    // Rules actually states there's no need to alert the driver, stopping motor output is enough
                    // Alerting the driver would be done if the digital dash is made, will have to see

                    // Turning off the motor is achieved using another digital pin, not via canbus, but will still send 0 torque can signals
                    fault_force_stop = true;

                    DBG_THROTTLE_FAULT(DIFF_FAULT_EXCEED_100MS);
                    // DBGLN_THROTTLE("FAULT: Pedal mismatch persisted > 100ms!");

                    // -- Debug: Pedal faulty too long

                    return;
                }
            }
            else
            {
                fault_start_millis = millis;
                DBG_THROTTLE_FAULT(DIFF_FAULT_JUST_STARTED);
                // DBGLN_THROTTLE("FAULT: Pedal mismatch started");
            }

            fault = true;
            return;
        }
        else
        {
            if (fault)
            {
                DBG_THROTTLE_FAULT(DIFF_FAULT_RESOLVED);
                // DBGLN_THROTTLE("Pedal mismatch resolved");
            }
            fault = false;
            // by rules, car should be reset after a fault to continue driving
            // fault_force_stop = false; // Reset the force stop flag
        }
    }
}

void Pedal::pedal_can_frame_stop_motor(can_frame *tx_throttle_msg)
{
    tx_throttle_msg->can_id  = MOTOR_COMMAND;
    tx_throttle_msg->can_dlc = 3;
    tx_throttle_msg->data[0] = 0x90; // 0x90 for torque, 0x31 for speed
    tx_throttle_msg->data[1] = 0;
    tx_throttle_msg->data[2] = 0;

    DBGLN_THROTTLE("Stopping motor");
}

void Pedal::pedal_can_frame_update(can_frame *tx_throttle_msg)
{
    if (fault_force_stop)
    {
        pedal_can_frame_stop_motor(tx_throttle_msg);

        DBGLN_THROTTLE("Forced motor to stop due to pedal fault");
        return;
    }
    // uint8_t throttle_volt = final_pedal_value * APPS_PEDAL_1_RANGE / 1024; // Converts most update pedal value to a float between 0V and 5V

    int16_t throttle_torque_val = 0;
    /*
    Between 0V and THROTTLE_LOWER_DEADZONE_MAX_IN_VOLT: Error for open circuit
    Between THROTTLE_LOWER_DEADZONE_MAX_IN_VOLT and MIN_THROTTLE_IN_VOLT: 0% Torque
    Between MIN_THROTTLE_IN_VOLT and MAX_THROTTLE_IN_VOLT: Linear relationship
    Between MAX_THROTTLE_IN_VOLT and THORTTLE_UPPER_DEADZONE_MIN_IN_VOLT: 100% Torque
    Between THORTTLE_UPPER_DEADZONE_MIN_IN_VOLT and 5V: Error for short circuit
    */
    if (final_pedal_value < THROTTLE_LOWER_DEADZONE_MIN_IN)
    {
        DBG_THROTTLE_FAULT(THROTTLE_TOO_LOW, final_pedal_value);
        // DBG_THROTTLE_FAULT(THROTTLE_TOO_LOW "Throttle voltage too low");
        throttle_torque_val = 0;
    }
    else if (final_pedal_value < MIN_THROTTLE_IN)
    {
        throttle_torque_val = MIN_THROTTLE_OUT_VAL;
    }
    else if (final_pedal_value < MAX_THROTTLE_IN)
    {
        // Scale up the value for canbus
        throttle_torque_val = throttle_torque_mapping(final_pedal_value, flip_motor_dir);
    }
    else if (final_pedal_value < THROTTLE_UPPER_DEADZONE_MAX_IN)
    {
        throttle_torque_val = MAX_THROTTLE_OUT_VAL;
    }
    else
    {
        // DBG_THROTTLE("Throttle voltage too high");
        DBG_THROTTLE_FAULT(THROTTLE_TOO_HIGH, final_pedal_value);
        // For safety, this should not be set to other values
        throttle_torque_val = 0;
    }

    DBG_THROTTLE_OUT(final_pedal_value, throttle_torque_val);
    // DBG_THROTTLE("CAN UPDATE: Throttle = ");

    tx_throttle_msg->can_id  = MOTOR_COMMAND;
    tx_throttle_msg->can_dlc = 3;
    tx_throttle_msg->data[0] = 0x90; // 0x90 for torque, 0x31 for speed
    tx_throttle_msg->data[1] = throttle_torque_val & 0xFF;
    tx_throttle_msg->data[2] = (throttle_torque_val >> 8) & 0xFF;
}

inline int16_t Pedal::throttle_torque_mapping(uint16_t throttle_volt, bool flip_motor_dir)
{
    // Map the throttle voltage to a torque value
    // temp linear mapping
    if (flip_motor_dir)
    {
        return (MAX_THROTTLE_IN - throttle_volt) * MAX_THROTTLE_OUT_VAL / (MAX_THROTTLE_IN - MIN_THROTTLE_IN);
    }
    return (throttle_volt - MIN_THROTTLE_IN) * MAX_THROTTLE_OUT_VAL / (MAX_THROTTLE_IN - MIN_THROTTLE_IN);
}

bool Pedal::check_pedal_fault(uint16_t pedal_1, uint16_t pedal_2)
{
    uint16_t pedal_2_scaled = round(pedal_2 * (FLOAT_APPS_PEDAL_1_RANGE / FLOAT_APPS_PEDAL_2_RANGE));
    uint16_t delta = abs(pedal_1 - pedal_2_scaled);
    // Currently the only indication for faulty pedal is just 2 pedal values are more than 10% different

    if (delta > 102.4) // 10% of 1024
    {
        DBG_THROTTLE_FAULT(DIFF_FAULT_CONTINUING, delta);
        // DBGLN_THROTTLE("WARNING: Pedal mismatch > 10%");
        return true;
    }
    return false;
}
