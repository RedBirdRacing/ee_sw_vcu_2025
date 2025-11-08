#include "Debug_serial.h"
#include "Enums.h"

/**
 * @brief Initializes the Debug_Serial interface.
 * This function sets up the serial communication for debugging purposes.
 * It should be called before using any other Debug_Serial functions.
 * 
 * @param None
 * @return None
 */
void Debug_Serial::initialize()
{
    Serial.begin(115200);
}

/**
 * @brief Prints a message to the serial console.
 * This function sends a string message to the serial console for debugging.
 * 
 * @param msg The message to print.
 * @return None
 */
void Debug_Serial::print(const char *msg) { Serial.print(msg); }

/**
 * @brief Prints a line to the serial console.
 * This function sends a string message followed by a newline to the serial console for debugging.
 * 
 * @param msg The message to print.
 * @return None
 */
void Debug_Serial::println(const char *msg) { Serial.println(msg); }

/**
 * @brief Prints a throttle input message to the serial console.
 * This function formats and sends the throttle input values to the serial console.
 * 
 * @param pedal_1 Value from pedal sensor 1.
 * @param pedal_2 Value from pedal sensor 2.
 * @param pedal_2_scaled Scaled value of pedal sensor 2.
 * @return None
 */
void Debug_Serial::throttle_in(uint16_t pedal_1, uint16_t pedal_2, uint16_t pedal_2_scaled, uint16_t brake)
{
    Serial.print("Pedal 1: ");
    Serial.print(pedal_1);
    Serial.print(" | Pedal 2: ");
    Serial.print(pedal_2);
    Serial.print(" | Pedal 2 scaled: ");
    Serial.print(pedal_2_scaled);
    Serial.print(" | Brake: ");
    Serial.println(brake);
}

/**
 * @brief Prints a throttle output message to the serial console.
 * This function formats and sends the throttle output values to the serial console.
 * 
 * @param throttle_final Final value of the throttle pedal.
 * @param throttle_torque_val Calculated torque value based on the throttle input.
 * @return None
 */
void Debug_Serial::throttle_out(uint16_t throttle_final, int16_t throttle_torque_val)
{
    Serial.print("Throttle Final: ");
    Serial.print(throttle_final);
    Serial.print(" | Torque Value: ");
    Serial.println(throttle_torque_val);
}

/**
 * @brief Prints a throttle fault message to the serial console.
 * This function formats and sends the throttle fault status and value to the serial console.
 * 
 * @param fault_status The status of the throttle fault as defined in pedal_fault_status enum.
 * @param value Optional value associated with the fault
 * @return None
 */
void Debug_Serial::throttle_fault(pedal_fault_status fault_status, uint16_t value)
{
    switch (fault_status)
    {
    case NONE:
        break;
    case DIFF_CONTINUING:
        Serial.print("Pedal mismatch continuing. Difference: ");
        Serial.println(value);
        break;
    case THROTTLE_LOW:
        Serial.print("Throttle input too low. Value: ");
        Serial.println(value);
        break;
    case THROTTLE_HIGH:
        Serial.print("Throttle too high. Value: ");
        Serial.println(value);
        break;
    default:
        Serial.println("Unknown fault status");
        break;
    }
}

/**
 * @brief Prints a throttle fault message to the serial console without a float value.
 * This function formats and sends the throttle fault status to the serial console.
 * 
 * @param fault_status The status of the throttle fault as defined in pedal_fault_status enum.
 * @return None
 */
void Debug_Serial::throttle_fault(pedal_fault_status fault_status)
{
    switch (fault_status)
    {
    case NONE:
        break;
    case DIFF_START:
        Serial.println("Pedal mismatch just started");
        break;
    case DIFF_EXCEED_100MS:
        Serial.println("FATAL FAULT: Pedal mismatch persisted > 100ms!");
        break;
    case DIFF_RESOLVED:
        Serial.println("Pedal mismatch resolved");
        break;
    default:
        Serial.println("Unknown fault status");
        break;
    }
}

/**
 * @brief Prints a brake fault message to the serial console.
 * This function formats and sends the brake fault status and value to the serial console.
 * 
 * @param fault_status The status of the brake fault as defined in pedal_fault_status enum.
 * @param value brake ADC reading
 * @return None
 */
void Debug_Serial::brake_fault(pedal_fault_status fault_status, uint16_t value)
{
    switch (fault_status)
    {
    case NONE:
        break;
    case BRAKE_LOW:
        Serial.print("Brake input too low. Value: ");
        Serial.println(value);
        break;
    case BRAKE_HIGH:
        Serial.print("Brake too high. Value: ");
        Serial.println(value);
        break;
    default:
        Serial.println("Unknown fault status");
        break;
    }
}

/**
 * @brief Prints the current car status to the serial console.
 * This function formats and sends the current car status to the serial console.
 * 
 * @param car_status The current status of the car as defined in main_car_status enum.
 * @return None
 */
void Debug_Serial::status_car(main_car_status car_status)
{
    static main_car_status last_status = INIT;
    if (car_status != last_status) {
        Serial.print("Car Status: ");
        switch (last_status)
        {
        case INIT:
            Serial.print("INIT -> ");
            break;
        case STARTIN:
            Serial.print("STARTIN -> ");
            break;
        case BUSSIN:
            Serial.print("BUSSIN -> ");
            break;
        case DRIVE:
            Serial.print("DRIVE -> ");
            break;
        default:
            Serial.print("UNKNOWN -> ");
            break;
        }
        switch (car_status)
        {
        case INIT:
            Serial.print("INIT -> ");
            break;
        case STARTIN:
            Serial.print("STARTIN -> ");
            break;
        case BUSSIN:
            Serial.print("BUSSIN -> ");
            break;
        case DRIVE:
            Serial.print("DRIVE -> ");
            break;
        default:
            Serial.print("UNKNOWN -> ");
            break;
        }
        last_status = car_status;
    }
}

/**
 * @brief Prints the current BMS status to the serial console.
 * This function formats and sends the current BMS status to the serial console.
 * 
 * @param BMS_status The current status of the BMS as defined in BMS_status enum.
 * @return None
 */
void Debug_Serial::status_bms(BMS_status BMS_status)
{
    switch (BMS_status)
    {
    case NO_MSG:
        Serial.println("BMS Status: No message received");
        break;
    case WRONG_ID:
        Serial.println("BMS Status: Wrong ID");
        break;
    case WAITING:
        Serial.println("BMS Status: Waiting to start");
        break;
    case STARTING:
        Serial.println("BMS Status: Starting");
        break;
    case STARTED:
        Serial.println("BMS Status: Started");
        break;
    default:
        Serial.println("BMS Status: UNKNOWN");
        break;
    }
}