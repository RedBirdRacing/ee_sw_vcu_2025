/**
 * @file Debug_serial.cpp
 * @author Planeson, Red Bird Racing
 * @brief Implementation of the Debug_Serial namespace for serial debugging functions
 * @version 1.1.1
 * @date 2026-02-09
 * @see Debug_serial.h
 */

#include "Debug_serial.hpp"
#include "Enums.hpp"

/**
 * @brief Initializes the Debug_Serial interface.
 * It should be called before using any other Debug_Serial functions.
 * 
 */
void Debug_Serial::initialize()
{
    Serial.begin(115200);
}

/**
 * @brief Prints a string to the serial console.
 * 
 * @param msg The message to print.
 * @note Serial exclusive
 */
void Debug_Serial::print(const char *msg) { Serial.print(msg); }

/**
 * @brief Prints a string AND a newline to the serial console.
 * 
 * @param msg The message to print.
 * @note Serial exclusive
 */
void Debug_Serial::println(const char *msg) { Serial.println(msg); }

/**
 * @brief Prints a throttle input message to the serial console.
 * This function formats and sends the throttle input values to the serial console.
 * 
 * @param pedal_1 Value from pedal sensor 1.
 * @param pedal_2 Value from pedal sensor 2.
 * @param pedal_2_scaled Scaled value of pedal sensor 2.
 * @param brake Brake pedal value.
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
 * @param fault_status The status of the throttle fault as defined in PedalFault enum.
 * @param value Optional uint16_t value associated with the fault
 */
void Debug_Serial::throttle_fault(PedalFault fault_status, uint16_t value)
{
    switch (fault_status)
    {
    case PedalFault::None:
        break;
    case PedalFault::DiffContinuing:
        Serial.print("Pedal mismatch continuing. Difference: ");
        Serial.println(value);
        break;
    case PedalFault::ThrottleLow:
        Serial.print("Throttle input too low. Value: ");
        Serial.println(value);
        break;
    case PedalFault::ThrottleHigh:
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
 * @param fault_status The status of the throttle fault as defined in PedalFault enum.
 */
void Debug_Serial::throttle_fault(PedalFault fault_status)
{
    switch (fault_status)
    {
    case PedalFault::None:
        break;
    case PedalFault::DiffStart:
        Serial.println("Pedal mismatch just started");
        break;
    case PedalFault::DiffExceed100ms:
        Serial.println("FATAL FAULT: Pedal mismatch persisted > 100ms!");
        break;
    case PedalFault::DiffResolved:
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
 * @param fault_status The status of the brake fault as defined in PedalFault enum.
 * @param value brake ADC reading
 */
void Debug_Serial::brake_fault(PedalFault fault_status, uint16_t value)
{
    switch (fault_status)
    {
    case PedalFault::None:
        break;
    case PedalFault::BrakeLow:
        Serial.print("Brake input too low. Value: ");
        Serial.println(value);
        break;
    case PedalFault::BrakeHigh:
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
 * @param car_status The current status of the car as defined in CarStatus enum.
 */
void Debug_Serial::status_car(CarStatus car_status)
{
    static CarStatus last_status = CarStatus::Init;
    if (car_status != last_status) {
        Serial.print("Car Status: ");
        switch (last_status)
        {
        case CarStatus::Init:
            Serial.print("INIT -> ");
            break;
        case CarStatus::Startin:
            Serial.print("STARTIN -> ");
            break;
        case CarStatus::Bussin:
            Serial.print("BUSSIN -> ");
            break;
        case CarStatus::Drive:
            Serial.print("DRIVE -> ");
            break;
        default:
            Serial.print("UNKNOWN -> ");
            break;
        }
        switch (car_status)
        {
        case CarStatus::Init:
            Serial.print("INIT");
            break;
        case CarStatus::Startin:
            Serial.print("STARTIN");
            break;
        case CarStatus::Bussin:
            Serial.print("BUSSIN");
            break;
        case CarStatus::Drive:
            Serial.print("DRIVE");
            break;
        default:
            Serial.print("UNKNOWN");
            break;
        }
        last_status = car_status;
    }
}

/**
 * @brief Prints the current BMS status to the serial console.
 * This function formats and sends the current BMS status to the serial console.
 * 
 * @param BMS_status The current status of the BMS as defined in BmsStatus enum.
 */
void Debug_Serial::status_bms(BmsStatus BMS_status)
{
    switch (BMS_status)
    {
    case BmsStatus::NoMsg:
        Serial.println("BMS Status: No message received");
        break;
/*
    case BmsStatus::WrongId:
        Serial.println("BMS Status: Wrong ID");
        break;
*/
    case BmsStatus::Waiting:
        Serial.println("BMS Status: Waiting to start");
        break;
    case BmsStatus::Starting:
        Serial.println("BMS Status: Starting");
        break;
    case BmsStatus::Started:
        Serial.println("BMS Status: Started");
        break;
    default:
        Serial.println("BMS Status: UNKNOWN");
        break;
    }
}


/**
 * @brief Prints the current hall sensor value to the serial console.
 * 
 * @param hall_sensor_value 
 */
void Debug_Serial::hall_sensor(uint16_t hall_sensor_value)
{
    void(Serial.print("Hall Sensor Value: "));
    void(Serial.println(hall_sensor_value));
}