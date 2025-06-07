#include "Debug_serial.h"

void Debug_Serial::initialize() {
    Serial.begin(115200);
}

void Debug_Serial::throttle_in(int pedal_filtered_1, int pedal_filtered_2, int pedal_filtered_final) {
    Serial.print("Pedal 1: ");
    Serial.print(pedal_filtered_1);
    Serial.print(" | Pedal 2: ");
    Serial.print(pedal_filtered_2);
    Serial.print(" | Final: ");
    Serial.println(pedal_filtered_final);
}