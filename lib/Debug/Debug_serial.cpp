#include "Debug_serial.h"

void Debug_Serial::initialize() {
    Serial.begin(115200);
}

void Debug_Serial::print(const char* msg) { Serial.print(msg); }

void Debug_Serial::println(const char* msg) { Serial.println(msg); }

void Debug_Serial::throttle_in(uint16_t pedal_filtered_1, uint16_t pedal_filtered_2, uint16_t pedal_filtered_final) {
    Serial.print("Pedal 1: ");
    Serial.print(pedal_filtered_1);
    Serial.print(" | Pedal 2: ");
    Serial.print(pedal_filtered_2);
    Serial.print(" | Final: ");
    Serial.println(pedal_filtered_final);
}

void Debug_Serial::throttle_out(float throttle_volt, int16_t throttle_torque_val) {
    Serial.print("Throttle Volt: ");
    Serial.print(throttle_volt);
    Serial.print(" | Torque Value: ");
    Serial.println(throttle_torque_val);
}

void Debug_Serial::throttle_fault(Pedal_Fault_Status fault_status, float value) {
    switch (fault_status) {
        case NO_FAULT:
            break;
        case DIFF_FAULT_JUST_STARTED:
            Serial.println("Pedal mismatch just started");
            break;
        case DIFF_FAULT_CONTINUING: 
            Serial.print("Pedal mismatch continuing. Difference:");
            Serial.print(value*100);
            Serial.println("%");
            break;
        case DIFF_FAULT_EXCEED_100MS:
            Serial.println("FATAL FAULT: Pedal mismatch persisted > 100ms!");
            break;
        case DIFF_FAULT_RESOLVED:
            Serial.println("Pedal mismatch resolved");
            break;
        case THROTTLE_TOO_LOW:
            Serial.print("Throttle input too low. Value:");
            Serial.print(value);
            Serial.println("v");
            break;
        case THROTTLE_TOO_HIGH:
            Serial.println("Throttle too high. Value:");
            Serial.print(value);
            Serial.println("v");
            break;
        default:
            Serial.println("Unknown fault status");
            break;
    }
}

void Debug_Serial::status_car(CarStatus car_status) {
    switch (car_status) {
        case INIT:
            Serial.println("Car Status: Initialised");
            break;
        case IN_STARTING_SEQUENCE:
            Serial.println("Car Status: In starting sequence");
            break;
        case BUZZING:
            Serial.println("Car Status: Buzzer buzzing");
            break;
        case DRIVE_MODE:
            Serial.println("Car Status: Drive mode");
            break;
        default:
            Serial.println("Car Status: UNKNOWN");
            break;
    }
}