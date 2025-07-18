#include "Pedal.h"
#include "Signal_Processing.cpp" // for AVG_filter, move to .h later
#include "Debug.h"
#include <Arduino.h> // for round() only.

/**
 * Pedal class implementation for handling throttle pedal inputs, filtering, and fault detection.
 * This class manages two pedal sensors, applies filtering, checks for faults, and updates the car state.
 * It uses a ring buffer for storing pedal values and applies an average filter to smooth the readings.
 */

/**
 * @brief Default constructor for the Pedal class.
 *
 * Initializes the pedal state and sets the fault flag to true.
 * The fault flag indicates that the pedal inputs are considered faulty until proven otherwise.
 * if no non-faulty pedal inputs are received in 100ms, the fault_force_stop flag in car_state is set to true.
 *
 * @param None
 * @return None
 */
Pedal::Pedal() : fault(true) {}

/**
 * @brief Updates pedal sensor readings, applies filtering, and checks for faults.
 *
 * Stores new pedal readings, applies an average filter, and updates car state.
 * If a fault is detected between pedal sensors, sets fault flags and logs status.
 *
 * @param car Pointer to car_state structure to update.
 * @param pedal_1 Raw value from pedal sensor 1.
 * @param pedal_2 Raw value from pedal sensor 2.
 * @return None
 */
void Pedal::pedal_update(car_state *car, uint16_t pedal_1, uint16_t pedal_2, uint16_t brake)
{
    // Record readings in buffer
    pedal_value_1.push(pedal_1);
    pedal_value_2.push(pedal_2);
    brake_value.push(brake);

    // Range of pedal 1 is APPS_PEDAL_1_RANGE, pedal 2 is APPS_PEDAL_2_RANGE;

    // this is current taking the direct array the circular queue writes into. Bad idea to do anything other than a simple average
    // if not using a linear filter, pass the pedalValue_1.getLinearBuffer() to the filter function to ensure the ordering is correct.
    // can also consider injecting the filter into the queue if need
    // depends on the hardware filter, reduce software filtering as much as possible

    // currently a small average filter is good enough
    pedal_filtered_1 = AVG_filter<uint16_t>(pedal_value_1.buffer, ADC_BUFFER_SIZE);
    pedal_filtered_2 = AVG_filter<uint16_t>(pedal_value_2.buffer, ADC_BUFFER_SIZE);
    car->brake_final = AVG_filter<uint16_t>(brake_value.buffer, ADC_BUFFER_SIZE);

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
    {                                               // Previous scan is already faulty
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

/**
 * @brief Updates the CAN frame with the current pedal values.
 *
 * This function prepares a CAN frame to send the current pedal values to the motor controller.
 * It checks for faults and applies appropriate torque values based on pedal readings.
 *
 * @param tx_throttle_msg Pointer to the CAN frame to update.
 * @param car Pointer to the car_state structure containing current car state.
 * @return None
 */
void Pedal::pedal_can_frame_stop_motor(can_frame *tx_throttle_msg)
{
    tx_throttle_msg->can_id = MOTOR_COMMAND;
    tx_throttle_msg->can_dlc = 3;
    tx_throttle_msg->data[0] = 0x90; // 0x90 for torque, 0x31 for speed
    tx_throttle_msg->data[1] = 0x00;
    tx_throttle_msg->data[2] = 0x00;
}

/**
 * @brief Updates the CAN frame with the most recent pedal value.
 *
 * This function prepares a CAN frame to send the current pedal value to the motor controller.
 * It checks for faults and applies appropriate torque values based on pedal readings.
 *
 * @param tx_throttle_msg Pointer to the CAN frame to update.
 * @param car Pointer to the car_state structure containing current car state.
 * @return None
 */
void Pedal::pedal_can_frame_update(can_frame *tx_throttle_msg, car_state *car)
{
    if (car->fault_force_stop)
    {
        pedal_can_frame_stop_motor(tx_throttle_msg);
        DBGLN_THROTTLE("Stopping motor: pedal fault");
        return;
    }
    // uint8_t throttle_volt = pedal_final * APPS_PEDAL_1_RANGE / 1024; // Converts most update pedal value to a float between 0V and 5V

    int16_t throttle_torque_val = 0;
    uint16_t pedal_final = car->pedal_final;
    /*
    Below PEDAL_LL: Error for open circuit
    Between PEDAL_LL and PEDAL_LU: 0% Torque
    Between PEDAL_LU and PEDAL_UL: mapping to torque, see throttle_torque_mapping()
    Between PEDAL_UL and PEDAL_UU: 100% Torque
    Above PEDAL_UU: Error for short circuit
    */
    if (pedal_final < PEDAL_LL)
    {
        DBG_THROTTLE_FAULT(THROTTLE_LOW, pedal_final);
        throttle_torque_val = 0;
    }
    else if (pedal_final < PEDAL_LU)
    {
        // in lower deadzone, treat as 0% throttle
        throttle_torque_val = brake_torque_mapping(car->brake_final, FLIP_MOTOR_DIR);
    }
    else if (pedal_final < PEDAL_UL)
    {
        // throttle_in -> torque_val
        // check mapping function for curve
        throttle_torque_val = throttle_torque_mapping(pedal_final, FLIP_MOTOR_DIR);
    }
    else if (pedal_final < PEDAL_UU)
    {
        // in upper deadzone, treat as 100% throttle
        throttle_torque_val = MAX_THROTTLE_OUT_VAL;
    }
    else
    {
        DBG_THROTTLE_FAULT(THROTTLE_HIGH, pedal_final);
        // throttle higher than upper deadzone, treat as throttle fault, zeroing torque for safety
        throttle_torque_val = 0;
    }

    DBG_THROTTLE_OUT(pedal_final, throttle_torque_val);

    tx_throttle_msg->can_id = MOTOR_COMMAND;
    tx_throttle_msg->can_dlc = 3;
    tx_throttle_msg->data[0] = 0x90; // 0x90 for torque, 0x31 for speed
    tx_throttle_msg->data[1] = throttle_torque_val & 0xFF;
    tx_throttle_msg->data[2] = (throttle_torque_val >> 8) & 0xFF;
}

/**
 * @brief Maps the pedal ADC to a torque value.
 *
 * This function takes a pedal ADC in the range of 0-1023 and maps it to a torque value.
 * The mapping is linear, and the result is adjusted based on the motor direction.
 *
 * @param throttle Pedal ADC in the range of 0-1023.
 * @param flip_motor_dir Boolean indicating whether to flip the motor direction.
 * @return Mapped torque value in the signed range of -32760 to 32760.
 */
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
        return -result;
    return result;
}
/**
 * @brief Maps the brake ADC to a torque value.
 *
 * This function takes a brake ADC in the range of 0-1023 and maps it to a torque value.
 * The mapping is linear, and the result is adjusted based on the motor direction.
 *
 * @param brake Brake ADC in the range of 0-1023.
 * @param flip_motor_dir Boolean indicating whether to flip the motor direction.
 * @return Mapped torque value in the signed range of -32760 to 32760.
 */
int16_t Pedal::brake_torque_mapping(uint16_t brake, bool flip_motor_dir)
{
    if (brake < BRAKE_LU || brake > BRAKE_UL)
    {
        // brake is out of range, return 0 torque
        return 0;
    }
    // Map the brake ADC to a torque value
    // temp linear mapping, with proper casting to prevent overflow
    int32_t numerator = static_cast<int32_t>(brake - PEDAL_1_LU) * static_cast<int32_t>(MAX_THROTTLE_OUT_VAL);
    int32_t denominator = static_cast<int32_t>(PEDAL_1_UL - PEDAL_1_LU);
    int16_t result = static_cast<int16_t>(numerator / denominator);

    // opposite direction of throttle
    if (flip_motor_dir)
        return result;
    return -result;
}

/**
 * @brief Checks for a fault between two pedal sensor readings.
 *
 * Scales pedal_2 to match the range of pedal_1, then calculates the absolute difference.
 * If the difference exceeds 10% of the full-scale value (i.e., >102.4 for a 10-bit ADC),
 * the function considers this a fault and returns true. Otherwise, returns false.
 * Also logs the readings and fault status for debugging.
 *
 * @param pedal_1 Raw value from pedal sensor 1 (uint16_t), intentionally casted to int16_t.
 * @param pedal_2 Raw value from pedal sensor 2 (uint16_t), intentionally casted to int16_t.
 * @return true if the difference exceeds the threshold (fault detected), false otherwise.
 */
bool Pedal::check_pedal_fault(int16_t pedal_1, int16_t pedal_2)
{

    int16_t pedal_2_scaled = round((float)pedal_2 * PEDAL_1_RANGE / PEDAL_2_RANGE);
    DBG_THROTTLE_IN(pedal_1, pedal_2, pedal_2_scaled);

    int16_t delta = pedal_1 - pedal_2_scaled;
    // if more than 10% difference between the two pedals, consider it a fault
    if (delta > 102.4 || delta < -102.4) // 10% of 1024, rounded down to 102
    {
        DBG_THROTTLE_FAULT(DIFF_CONTINUING, delta);
        return true;
    }
    return false;
}
