#include <Arduino.h>
#include "boardConf.h"
#include "Pedal.h"
#include "BMS.h"

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
constexpr uint8_t INPUT_COUNT = 5;
constexpr uint8_t pins_in[INPUT_COUNT] = {DRIVE_MODE_BTN, BRAKE_IN, APPS_5V, APPS_3V3, HALL_SENSOR};
constexpr uint8_t OUTPUT_COUNT = 4;
constexpr uint8_t pins_out[OUTPUT_COUNT] = {FRG, BRAKE_LIGHT, BUZZER, BMS_FAILED_LED};

// === Pedal ===
Pedal pedal;

// === even if unused, initialize ALL mcp2515 to make sure the CS pin is set up and they don't interfere with the SPI bus ===
// === CAN (motor) ===
MCP2515 mcp2515_motor(CS_CAN_MOTOR);

// === CAN (BMS) ===
MCP2515 mcp2515_BMS(CS_CAN_BMS);
BMS bms(&mcp2515_BMS);

// === CAN (Datalogger) ===
MCP2515 mcp2515_DL(CS_CAN_DL);
#define mcp2515_motor mcp2515_DL

struct can_frame tx_throttle_msg;

constexpr uint16_t STARTING_MILLIS = 2000; // The amount of time that the driver needs to hold the "Start" button and full brakes in order to activate driving mode
constexpr uint16_t BUSSIN_MILLIS = 2000;   // The amount of time that the buzzer will buzz for
constexpr uint16_t BMS_MILLIS = 10000;     // The maximum amount of time to wait for the BMS to start HV, if passed, assume started but not reading response

constexpr uint16_t BRAKE_THRESHOLD = 130; // The threshold for the brake pedal to be considered pressed

bool brake_pressed = false; // boolean for brake light on VCU (for ignition)

/**
 * @brief Global car state stru=cture.
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


void scheduler_pedal(){
    pedal.pedal_can_frame_update(&tx_throttle_msg, &main_car_state);
    mcp2515_motor.sendMessage(&tx_throttle_msg);
}
void scheduler_bms(){
    bms.check_hv();
}

/*    Scheduler(uint32_t period_us_,
                uint32_t spin_threshold_us_,
                const void (*tasks_[])(),
                const uint8_t task_ticks_[])
*/
/*
Scheduler<2> scheduler(10000, 100,
                       {&scheduler_pedal, &scheduler_bms},
                       {
                           0,  // Pedal task runs every tick (10ms)
                           4 // BMS task runs every 5 ticks (50ms)
                       });*/

void setup()
{
    // Init pedals
    pedal = Pedal();

#if DEBUG_SERIAL
    Debug_Serial::initialize();
    DBGLN_GENERAL("Debug serial initialized");
#endif

    // Init input pins
    for (int i = 0; i < INPUT_COUNT; i++)
    {
        pinMode(pins_in[i], INPUT);
    }
    // Init output pins
    for (int i = 0; i < OUTPUT_COUNT; i++)
    {
        pinMode(pins_out[i], OUTPUT);
        digitalWrite(pins_out[i], LOW); // initialize output pins to LOW
    }

    // Init mcp2515 for motor CAN channel
    mcp2515_motor.reset();
    mcp2515_motor.setBitrate(CAN_500KBPS, MCP2515_CRYSTAL_FREQ);
    mcp2515_motor.setNormalMode();

    // Init mcp2515 for BMS channel
    mcp2515_BMS.reset();
    mcp2515_BMS.setBitrate(CAN_500KBPS, MCP2515_CRYSTAL_FREQ);
    mcp2515_BMS.setNormalMode();

    mcp2515_DL.reset();
    mcp2515_DL.setBitrate(CAN_500KBPS, MCP2515_CRYSTAL_FREQ);
    mcp2515_DL.setNormalMode();

#if DEBUG_CAN
    Debug_CAN::initialize(&mcp2515_DL); // Currently using motor CAN for debug messages, should change to other
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
    delay(9); // Loop every 10ms
    DBG_HALL_SENSOR(analogRead(HALL_SENSOR));
    main_car_state.millis = millis(); // Update the current millis time
    // Read pedals
    pedal.pedal_update(&main_car_state, analogRead(APPS_5V), analogRead(APPS_3V3), analogRead(BRAKE_IN));

    brake_pressed = static_cast<uint16_t>(analogRead(BRAKE_IN)) >= BRAKE_THRESHOLD;
    digitalWrite(BRAKE_LIGHT, brake_pressed ? HIGH : LOW);
    /*
    For the time being:
    DRIVE_MODE_BTN = "Start" button
    BRAKE_IN = Brake pedal
    BUZZER = Buzzer output
    FRG = "Drive" mode indicator
    */
    if (main_car_state.fault_force_stop)
    {
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        DBGLN_THROTTLE("Stopping motor: Fault exceeded 100ms.");
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        main_car_state.car_status = INIT; // safety, later change to fault status
        digitalWrite(BUZZER, LOW);        // Turn off buzzer
        digitalWrite(FRG, LOW);           // Turn off drive mode LED
        return;                           // If fault force stop is active, do not proceed with the rest of the loop
        // pedal is still being updated, data can still be gathered and sent through CAN/serial
    }

    switch (main_car_state.car_status)
    {
    case DRIVE:
        // Pedal update
        // Send pedal value through canbus
        pedal.pedal_can_frame_update(&tx_throttle_msg, &main_car_state);
        mcp2515_motor.sendMessage(&tx_throttle_msg);
        return; // no need logic to check if pedal on, car started

    // do not return here if not in DRIVE mode, else can't detect pedal being on while starting
    case INIT:
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        DBGLN_THROTTLE("Stopping motor: INIT.");
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        if (digitalRead(DRIVE_MODE_BTN) == BUTTON_ACTIVE && brake_pressed)
        {
            main_car_state.car_status = STARTIN;
            main_car_state.car_status_millis_counter = main_car_state.millis;

            DBG_STATUS_CAR(main_car_state.car_status);
        }
        break;

    case STARTIN:
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        DBGLN_THROTTLE("Stopping motor: STARTIN.");
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        bms.check_hv(); // check for HV ready, if not start it

        if (digitalRead(DRIVE_MODE_BTN) != BUTTON_ACTIVE || !brake_pressed)
        {
            main_car_state.car_status = INIT;
            main_car_state.car_status_millis_counter = main_car_state.millis; // safety

            DBG_STATUS_CAR(main_car_state.car_status);
        }
        else if (main_car_state.millis - main_car_state.car_status_millis_counter >= STARTING_MILLIS)
        {
            if (bms.hv_ready()) // if HV not started, return to INIT
            {
                main_car_state.car_status = INIT;
                main_car_state.car_status_millis_counter = main_car_state.millis; // safety

                DBG_STATUS_CAR(main_car_state.car_status);
                break;
            }
            if (main_car_state.millis - main_car_state.car_status_millis_counter >= BMS_MILLIS)
            {
                main_car_state.car_status = BUSSIN;
                main_car_state.car_status_millis_counter = main_car_state.millis; // safety

                DBG_STATUS_CAR(main_car_state.car_status);
                break;
            }
        }
        break;

    case BUSSIN:
        digitalWrite(BUZZER, HIGH); // Turn on buzzer
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        DBGLN_THROTTLE("Stopping motor: BUSSIN.");
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        if (main_car_state.millis - main_car_state.car_status_millis_counter >= BUSSIN_MILLIS)
        {
            digitalWrite(BUZZER, LOW);
            digitalWrite(FRG, HIGH);
            main_car_state.car_status = DRIVE;

            DBG_STATUS_CAR(main_car_state.car_status);
        }
        break;

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
    }
}
