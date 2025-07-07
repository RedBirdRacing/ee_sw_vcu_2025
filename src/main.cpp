#include <Arduino.h>
#include "pinMap.h"
#include "Pedal.h"
#include <mcp2515.h>
#include "Debug.h"
#include "Enums.h"

// === Pin setup ===
// Pin setup for pedal pins are done by the constructor of Pedal object
const uint8_t INPUT_COUNT = 2;
const uint8_t pins_in[INPUT_COUNT] = {DRIVE_MODE_BTN, BRAKE_IN};
const uint8_t OUTPUT_COUNT = 3;
const uint8_t pins_out[OUTPUT_COUNT] = {DRIVE_MODE_LED, BRAKE_5V_OUT, BUZZER_OUT};

// === CAN (motor) + Pedal ===
MCP2515 mcp2515_motor(CS_CAN_MOTOR);
Pedal pedal;

// === CAN (BMS) ===
MCP2515 mcp2515_BMS(CS_CAN_BMS);

// === CAN (Datalogger) ===
MCP2515 mcp2515_DL(CS_CAN_DL);

struct can_frame tx_throttle_msg;
struct can_frame rx_msg;

const uint16_t STARTING_MILLIS = 2000; // The amount of time that the driver needs to hold the "Start" button and full brakes in order to activate driving mode
const uint16_t BUSSIN_MILLIS = 2000;   // The amount of time that the buzzer will buzz for

struct car_state
{
    main_car_status car_status;         // Current car status
    uint32_t car_status_millis_counter; // Millis counter for the current car status
    uint16_t pedal_1_in;                // The input value of the first pedal (APPS 5V)
    uint16_t pedal_2_in;                // The input value of the second pedal (APPS 3V3)
    uint16_t pedal_final;               // The final pedal value after filtering and processing, normalized to 0-1023 range
    bool fault_force_stop;              // fault force stop flag
    uint16_t torque_out;                // The torque output to the motor
} main_car_state = {
    INIT,  // car_status
    0,     // car_status_millis_counter
    0,     // pedal_1_in
    0,     // pedal_2_in
    0,     // pedal_final
    false, // fault_force_stop
    0      // torque_out
};

void setup()
{
    // Init pedals
    pedal = Pedal();

    // Init input pins
    for (int i = 0; i < INPUT_COUNT; i++)
        pinMode(pins_in[i], INPUT);
    // Init output pins
    for (int i = 0; i < OUTPUT_COUNT; i++)
        pinMode(pins_out[i], OUTPUT);

    // Init mcp2515 for motor CAN channel
    mcp2515_motor.reset();
    mcp2515_motor.setBitrate(CAN_500KBPS, MCP_8MHZ); // 8MHZ for testing on uno
    mcp2515_motor.setNormalMode();

#if DEBUG_SERIAL
    while (!Serial)
    {
    } // Wait for serial connection
    Debug_Serial::initialize();
    DBGLN_GENERAL("Debug systems initialized");
#endif

#if DEBUG_CAN
    Debug_CAN::initialize(&mcp2515_motor); // Currently using motor CAN for debug messages, should change to other
    DBGLN_GENERAL("Debug CAN initialized");
#endif

    DBG_STATUS_CAR(main_car_state.car_status);
    DBGLN_STATUS("Entered State 0 (Idle)");
    DBGLN_GENERAL("Setup complete, entering main loop");
}

void loop()
{
    // Update pedal value
    pedal.pedal_update(millis(), analogRead(APPS_5V), analogRead(APPS_3V3));

    /*
    For the time being:
    DRIVE_MODE_BTN = "Start" button
    BRAKE_IN = Brake pedal
    BUZZER_OUT = Buzzer output
    DRIVE_MODE_LED = "Drive" mode indicator
    */

    if (main_car_state.car_status == INIT)
    {
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        DBGLN_STATUS("Holding 0 torque during state 0");

        if (digitalRead(DRIVE_MODE_BTN) == HIGH && digitalRead(BRAKE_IN) == HIGH) // Check if "Start" button and brake is fully pressed
        {
            main_car_state.car_status = STARTIN;
            main_car_state.car_status_millis_counter = millis();

            DBG_STATUS_CAR(main_car_state.car_status);
            DBGLN_STATUS("Entered State 1");
        }
    }
    else if (main_car_state.car_status == STARTIN)
    {
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        DBGLN_STATUS("Holding 0 torque during state 1");

        if (digitalRead(DRIVE_MODE_BTN) == LOW || digitalRead(BRAKE_IN) == LOW) // Check if "Start" button or brake is not fully pressed
        {
            main_car_state.car_status = INIT;
            main_car_state.car_status_millis_counter = millis();

            DBG_STATUS_CAR(main_car_state.car_status);
            DBGLN_STATUS("Drive mode button or brake pedal is released, returned to state 0 (Idle)");
        }
        else if (millis() - main_car_state.car_status_millis_counter >= STARTING_MILLIS) // Check if button held long enough
        {
            main_car_state.car_status = BUSSIN;
            digitalWrite(BUZZER_OUT, HIGH); // Turn on buzzer
            main_car_state.car_status_millis_counter = millis();

            DBG_STATUS_CAR(main_car_state.car_status);
            DBGLN_STATUS("Transition to State 2: Buzzer ON");
        }
    }
    else if (main_car_state.car_status == BUSSIN)
    {
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        DBGLN_STATUS("Holding 0 torque during state 2");

        if (millis() - main_car_state.car_status_millis_counter >= BUSSIN_MILLIS)
        {
            digitalWrite(DRIVE_MODE_LED, HIGH); // Turn on "Drive" mode indicator
            digitalWrite(BUZZER_OUT, LOW);      // Turn off buzzer
            main_car_state.car_status = DRIVE;

            DBG_STATUS_CAR(main_car_state.car_status);
            DBGLN_STATUS("Transition to State 3: Drive mode");
        }
    }
    else if (main_car_state.car_status == DRIVE)
    {
        // In "Drive mode", car_status won't change, the driver either continue to drive, or shut off the car
        DBGLN_STATUS("In Drive Mode");
    }
    else
    {
        // Error, idk wtf to do here
        DBG_STATUS_CAR(main_car_state.car_status);
        DBGLN_STATUS("ERROR: Invalid car_status encountered!");
    }

    // Pedal update
    if (main_car_state.car_status == DRIVE)
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
        if (pedal.pedal_final > MIN_THROTTLE_OUT_VAL)
        {
            main_car_state.car_status = INIT;
            main_car_state.car_status_millis_counter = millis(); // Set to current time, in case any counter relies on this
            pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
            mcp2515_motor.sendMessage(&tx_throttle_msg);

            DBG_STATUS_CAR(main_car_state.car_status);
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
