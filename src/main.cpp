#include <Arduino.h>
#include "pinMap.h"
#include "Pedal.h"
#include <mcp2515.h>
#include "Debug.h"
#include "Enums.h"

// === Pin setup ===
// Pin setup for pedal pins are done by the constructor of Pedal object
const int INPUT_PINS_COUNT = 2;
uint8_t pin_in[INPUT_PINS_COUNT] = {DRIVE_MODE_BTN, BRAKE_IN};
const int OUTPUT_PINS_COUNT = 3;
uint8_t pin_out[OUTPUT_PINS_COUNT] = {DRIVE_MODE_LED, BRAKE_5V_OUT, BUZZER_OUT};

// === CAN (motor) + Pedal ===
MCP2515 mcp2515_motor(CS_CAN_MOTOR);
Pedal pedal;

// === CAN (BMS) ===
MCP2515 mcp2515_BMS(CS_CAN_BMS);

// === CAN (Datalogger) ===
MCP2515 mcp2515_DL(CS_CAN_DL);

struct can_frame tx_throttle_msg;
struct can_frame rx_msg;

// For limiting the throttle update cycle
// const int THROTTLE_UPDATE_PERIOD_MILLIS = 50; // Period of sending canbus signal
// unsigned long final_throttle_time_millis = 0;  // The last time sent a canbus message

CarStatus car_status = INIT;
unsigned long car_status_millis_counter = 0; // Millis counter for 1st and 2nd transitionin states
const int STATUS_1_TIME_MILLIS = 2000;       // The amount of time that the driver needs to hold the "Start" button and full brakes in order to activate driving mode
const int BUSSIN_TIME_MILLIS = 2000;         // The amount of time that the buzzer will buzz for

void setup()
{
    // Init pedals
    pedal = Pedal(APPS_5V, APPS_3V3, millis());

    // Init input pins
    for (int i = 0; i < INPUT_PINS_COUNT; i++)
        pinMode(pin_in[i], INPUT);
    // Init output pins
    for (int i = 0; i < OUTPUT_PINS_COUNT; i++)
        pinMode(pin_out[i], OUTPUT);

    // Init mcp2515 for motor CAN channel
    mcp2515_motor.reset();
    mcp2515_motor.setBitrate(CAN_500KBPS, MCP_8MHZ); // 8MHZ for testing on uno
    mcp2515_motor.setNormalMode();

    #if DEBUG_SERIAL
        while (!Serial) {}  // Wait for serial connection
        Debug_Serial::initialize();
        DBGLN_GENERAL("Debug systems initialized");
    #endif

    #if DEBUG_CAN
        Debug_CAN::initialize(&mcp2515_motor); // Currently using motor CAN for debug messages, should change to other
        DBGLN_GENERAL("Debug CAN initialized");
    #endif

    DBG_STATUS_CAR(car_status);
    DBGLN_STATUS("Entered State 0 (Idle)");
    DBGLN_GENERAL("Setup complete, entering main loop");
}

void loop()
{
    // Update pedal value
    pedal.pedal_update(millis());

    /*
    For the time being:
    DRIVE_MODE_BTN = "Start" button
    BRAKE_IN = Brake pedal
    BUZZER_OUT = Buzzer output
    DRIVE_MODE_LED = "Drive" mode indicator
    */

    if (car_status == INIT)
    {
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        DBGLN_STATUS("Holding 0 torque during state 0");

        if (digitalRead(DRIVE_MODE_BTN) == HIGH && digitalRead(BRAKE_IN) == HIGH) // Check if "Start" button and brake is fully pressed
        {
            car_status = IN_STARTING_SEQUENCE;
            car_status_millis_counter = millis();

            DBG_STATUS_CAR(car_status);
            DBGLN_STATUS("Entered State 1");
        }
    }
    else if (car_status == IN_STARTING_SEQUENCE)
    {
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        DBGLN_STATUS("Holding 0 torque during state 1");

        if (digitalRead(DRIVE_MODE_BTN) == LOW || digitalRead(BRAKE_IN) == LOW) // Check if "Start" button or brake is not fully pressed
        {
            car_status = INIT;
            car_status_millis_counter = millis();

            DBG_STATUS_CAR(car_status);
            DBGLN_STATUS("Drive mode button or brake pedal is released, returned to state 0 (Idle)");
        }
        else if (millis() - car_status_millis_counter >= STATUS_1_TIME_MILLIS) // Check if button held long enough
        {
            car_status = BUZZING;
            digitalWrite(BUZZER_OUT, HIGH); // Turn on buzzer
            car_status_millis_counter = millis();

            DBG_STATUS_CAR(car_status);
            DBGLN_STATUS("Transition to State 2: Buzzer ON");
        }
    }
    else if (car_status == BUZZING)
    {
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        DBGLN_STATUS("Holding 0 torque during state 2");

        if (millis() - car_status_millis_counter >= BUSSIN_TIME_MILLIS)
        {
            digitalWrite(DRIVE_MODE_LED, HIGH); // Turn on "Drive" mode indicator
            digitalWrite(BUZZER_OUT, LOW);  // Turn off buzzer
            car_status = DRIVE_MODE;

            DBG_STATUS_CAR(car_status);
            DBGLN_STATUS("Transition to State 3: Drive mode");
        }
    }
    else if (car_status == DRIVE_MODE)
    {
        // In "Drive mode", car_status won't change, the drvier either continue to drive, or shut off the car
        DBGLN_STATUS("In Drive Mode");
    }
    else
    {
        // Error, idk wtf to do here
        DBG_STATUS_CAR(car_status);
        DBGLN_STATUS("ERROR: Invalid car_status encountered!");
    }

    // Pedal update
    if (car_status == DRIVE_MODE)
    {
        // Send pedal value through canbus
        pedal.pedal_can_frame_update(&tx_throttle_msg);
        // The following if block is needed only if we limit the lower bound for canbus cycle period
        // if (millis() - final_throttle_time_millis >= THROTTLE_UPDATE_PERIOD_MILLIS)
        // {
        //     mcp2515.sendMessage(&tx_throttle_msg);
        //     final_throttle_time_millis = millis();
        // }
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        DBGLN_THROTTLE("Throttle CAN frame sent");
    }
    else
    {
        if (pedal.final_pedal_value > MIN_THROTTLE_OUT_VAL)
        {
            car_status = INIT;
            car_status_millis_counter = millis(); // Set to current time, in case any counter relies on this
            pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
            mcp2515_motor.sendMessage(&tx_throttle_msg);

            DBG_STATUS_CAR(car_status);
            DBGLN_STATUS("Throttle pressed too early — Resetting to State 0");
        }
    }

    // mcp2515.sendMessage(&tx_throttle_msg);
    // uint32_t lastLEDtick = 0;
    // Optional RX handling (disabled for now)
    // if (mcp2515.readMessage(&rx_msg) == MCP2515::ERROR_OK)
    // {
    //     // Commented out as currenlty no need to include receive functionality
    //     // if (rx_msg.can_id == 0x522)
    //     //     for (int i = 0; i < 8; i++)
    //     //         digitalWrite(pin_out[i], (rx_msg.data[0] >> i) & 0x01);
    // }
}
