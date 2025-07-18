#include <Arduino.h>
#include "pinMap.h"
#include "Pedal.h"

// ignore -Wpedantic warnings for mcp2515.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mcp2515.h>
#pragma GCC diagnostic pop

// ignore -Wunused-parameter warnings for Debug.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "Debug.h"
#pragma GCC diagnostic pop

#include "Enums.h"
#include "car_state.h"

// === Pin setup ===
// Pin setup for pedal pins are done by the constructor of Pedal object
const uint8_t INPUT_COUNT = 4;
const uint8_t pins_in[INPUT_COUNT] = {DRIVE_MODE_BTN, BRAKE_IN, APPS_5V, APPS_3V3};
const uint8_t OUTPUT_COUNT = 3;
const uint8_t pins_out[OUTPUT_COUNT] = {DRIVE_MODE_LED, BRAKE_5V_OUT, BUZZER_OUT};

// === Pedal ===
Pedal pedal;

// === CAN (motor) ===
MCP2515 mcp2515_motor(CS_CAN_MOTOR);

// === CAN (BMS) ===
MCP2515 mcp2515_BMS(CS_CAN_BMS);

// === CAN (Datalogger) ===
MCP2515 mcp2515_DL(CS_CAN_DL);

struct can_frame tx_throttle_msg;
struct can_frame rx_msg;

const uint16_t STARTING_MILLIS = 2000; // The amount of time that the driver needs to hold the "Start" button and full brakes in order to activate driving mode
const uint16_t BUSSIN_MILLIS = 2000;   // The amount of time that the buzzer will buzz for

const uint16_t BRAKE_THRESHOLD = 256; // The threshold for the brake pedal to be considered pressed

/**
 * @brief Global car state structure.
 *
 * Holds the current state of the car, including status, timers, pedal input, fault flags, and output torque.
 *
 * @param car_status                Current main state of the car (e.g., INIT, DRIVE, etc.).
 * @param car_status_millis_counter Millisecond counter for state transitions.
 * @param millis                    Current time in milliseconds.
 * @param pedal_final               Final processed pedal value.
 * @param fault_force_stop          True if a fault has triggered a forced stop.
 * @param torque_out                Output torque value.
 */
struct car_state main_car_state = {
    INIT,  // car_status
    0,     // car_status_millis_counter
    0,     // millis
    0,     // pedal_final
    0,     // brake
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
    mcp2515_motor.setBitrate(CAN_500KBPS, MCP_20MHZ);
    mcp2515_motor.setNormalMode();

#if DEBUG_SERIAL
    while (!Serial)
    {
    } // Wait for serial connection
    Debug_Serial::initialize();
    DBGLN_GENERAL("Debug serial initialized");
#endif

#if DEBUG_CAN
    Debug_CAN::initialize(&mcp2515_motor); // Currently using motor CAN for debug messages, should change to other
    DBGLN_GENERAL("Debug CAN initialized");
#endif

    DBG_STATUS_CAR(main_car_state.car_status);
    DBGLN_STATUS("Entered INIT");
    DBGLN_GENERAL("Setup complete, entering main loop");

    /*
    For sim only
    pinMode(BRAKE_IN, INPUT_PULLUP); // Set brake input pin to pull-up mode
    pinMode(DRIVE_MODE_BTN, INPUT_PULLUP); // Set drive mode button pin
    */
}

void loop()
{
    main_car_state.millis = millis(); // Update the current millis time
    // Read pedals
    pedal.pedal_update(&main_car_state, analogRead(APPS_5V), analogRead(APPS_3V3), analogRead(BRAKE_IN));

    /*
    For the time being:
    DRIVE_MODE_BTN = "Start" button
    BRAKE_IN = Brake pedal
    BUZZER_OUT = Buzzer output
    DRIVE_MODE_LED = "Drive" mode indicator
    */

    if (main_car_state.fault_force_stop)
    {
        main_car_state.car_status = INIT;
        digitalWrite(BUZZER_OUT, LOW);     // Turn off buzzer
        digitalWrite(DRIVE_MODE_LED, LOW); // Turn off drive mode LED
        return;                            // If fault force stop is active, do not proceed with the rest of the loop
        // pedal is still being updated, data can still be gathered and sent through CAN/serial
    }
    switch (main_car_state.car_status)
    {
    // do not return here if not in DRIVE mode, else can't detect pedal being on while starting
    case INIT:
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        DBGLN_THROTTLE("Stopping motor: INIT.");
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        if (digitalRead(DRIVE_MODE_BTN) == HIGH &&
            static_cast<uint16_t>(analogRead(BRAKE_IN)) >= BRAKE_THRESHOLD)
        {
            main_car_state.car_status = STARTIN;
            main_car_state.car_status_millis_counter = main_car_state.millis;

            DBG_STATUS_CAR(main_car_state.car_status);
            DBG_STATUS_CAR_CHANGE(INIT_TO_STARTIN);
        }
        break;

    case STARTIN:
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        DBGLN_THROTTLE("Stopping motor: STARTIN.");
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        if (digitalRead(DRIVE_MODE_BTN) == LOW ||
            static_cast<uint16_t>(analogRead(BRAKE_IN)) < BRAKE_THRESHOLD)
        {
            main_car_state.car_status = INIT;
            main_car_state.car_status_millis_counter = main_car_state.millis; // safety

            DBG_STATUS_CAR(main_car_state.car_status);
            DBG_STATUS_CAR_CHANGE(STARTIN_TO_INIT);
        }
        else if (main_car_state.millis - main_car_state.car_status_millis_counter >= STARTING_MILLIS)
        {
            main_car_state.car_status = BUSSIN;
            main_car_state.car_status_millis_counter = main_car_state.millis;
            digitalWrite(BUZZER_OUT, HIGH);

            DBG_STATUS_CAR(main_car_state.car_status);
            DBG_STATUS_CAR_CHANGE(STARTIN_TO_BUSSIN);
        }
        break;

    case BUSSIN:
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        DBGLN_THROTTLE("Stopping motor: BUSSIN.");
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        if (main_car_state.millis - main_car_state.car_status_millis_counter >= BUSSIN_MILLIS)
        {
            digitalWrite(BUZZER_OUT, LOW);
            digitalWrite(DRIVE_MODE_LED, HIGH);
            main_car_state.car_status = DRIVE;

            DBG_STATUS_CAR(main_car_state.car_status);
            DBG_STATUS_CAR_CHANGE(BUSSIN_TO_DRIVE);
        }
        break;

    case DRIVE:
        // Pedal update
        // Send pedal value through canbus
        pedal.pedal_can_frame_update(&tx_throttle_msg, &main_car_state);
        mcp2515_motor.sendMessage(&tx_throttle_msg);
        return;

    default:
        DBG_STATUS_CAR(main_car_state.car_status);
        break;
    }

    // DRIVE mode has already returned, if reached here, then means car isn't in DRIVE
    if (main_car_state.pedal_final > PEDAL_1_LU)
    {
        main_car_state.car_status = INIT;
        main_car_state.car_status_millis_counter = main_car_state.millis; // Set to current time, in case any counter relies on this
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        DBGLN_THROTTLE("Stopping motor: throttle input while starting.");
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        DBG_STATUS_CAR(main_car_state.car_status);
        DBG_STATUS_CAR_CHANGE(THROTTLE_TO_INIT);
    }
}