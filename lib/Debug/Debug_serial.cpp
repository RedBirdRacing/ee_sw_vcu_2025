#include "Debug_serial.h"

void Debug_Serial::initialize()
{
    Serial.begin(115200);
}

void Debug_Serial::print(const char *msg) { Serial.print(msg); }

void Debug_Serial::println(const char *msg) { Serial.println(msg); }

void Debug_Serial::throttle_in(uint16_t pedal_filtered_1, uint16_t pedal_filtered_2, uint16_t pedal_filtered_final)
{
    Serial.print("Pedal 1 filtered: ");
    Serial.print(pedal_filtered_1);
    Serial.print(" | Pedal 2 filtered: ");
    Serial.print(pedal_filtered_2);
    Serial.print(" | Pedal 2 scaled: ");
    Serial.println(pedal_filtered_final);
}

void Debug_Serial::throttle_out(uint16_t throttle_final, int16_t throttle_torque_val)
{
    Serial.print("Throttle Final: ");
    Serial.print(throttle_final);
    Serial.print(" | Torque Value: ");
    Serial.println(throttle_torque_val);
}

void Debug_Serial::throttle_fault(pedal_fault_status fault_status, float value)
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

void Debug_Serial::status_car(main_car_status car_status)
{
    switch (car_status)
    {
    case INIT:
        Serial.println("Car Status: Initialised");
        break;
    case STARTIN:
        Serial.println("Car Status: In starting sequence");
        break;
    case BUSSIN:
        Serial.println("Car Status: Buzzer buzzing");
        break;
    case DRIVE:
        Serial.println("Car Status: Drive mode");
        break;
    default:
        Serial.println("Car Status: UNKNOWN");
        break;
    }
}

void Debug_Serial::status_car_change(state_changes status_change)
{
    switch (status_change)
    {
    case INIT_TO_STARTIN:
        Serial.println("Car Status Change: INIT -> STARTIN");
        break;
    case STARTIN_TO_BUSSIN:
        Serial.println("Car Status Change: STARTIN -> BUSSIN");
        break;
    case BUSSIN_TO_DRIVE:
        Serial.println("Car Status Change: BUSSIN -> DRIVE");
        break;
    case STARTIN_TO_INIT:
        Serial.println("Car Status Change: STARTIN -> INIT; Drive mode button or brake pedal is released, returning to INIT");
        break;
    case THROTTLE_TO_INIT:
        Serial.println("Car Status Change: THROTTLE -> INIT; Throttle pressed too early!");
        break;
    default:
        Serial.println("Car Status Change: UNKNOWN");
        break;
    }
}