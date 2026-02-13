/**
 * @file main.cpp
 * @author Planeson, Red Bird Racing
 * @brief Main VCU program entry point
 * @version 2.1
 * @date 2026-02-09
 * @dir include @brief Contains all header-only files.
 * @dir lib @brief Contains all the libraries. Each library is in its own folder of the same name.
 * @dir src @brief Contains the main.cpp file, the main file of the program.
 */

#include <Arduino.h>
#include "BoardConf.h"
#include "Pedal.hpp"
#include "BMS.hpp"
#include "Enums.hpp"
#include "CarState.hpp"
#include "Scheduler.hpp"
#include "Curves.hpp"
#include "Telemetry.hpp"
#include "Debug.hpp"

// ignore -Wpedantic warnings for mcp2515.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mcp2515.h>
#pragma GCC diagnostic pop

// ignore -Wunused-parameter warnings for Debug.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "Debug.hpp"
#pragma GCC diagnostic pop

// === Pin setup ===
// Pin setup for pedal pins are done by the constructor of Pedal object
constexpr uint8_t INPUT_COUNT = 5;
constexpr uint8_t OUTPUT_COUNT = 4;
constexpr uint8_t pins_in[INPUT_COUNT] = {DRIVE_MODE_BTN, BRAKE_IN, APPS_5V, APPS_3V3, HALL_SENSOR};
constexpr uint8_t pins_out[OUTPUT_COUNT] = {FRG, BRAKE_LIGHT, BUZZER, BMS_FAILED_LED};

// === even if unused, initialize ALL mcp2515 to make sure the CS pin is set up and they don't interfere with the SPI bus ===
MCP2515 mcp2515_motor(CS_CAN_MOTOR); // motor CAN
MCP2515 mcp2515_BMS(CS_CAN_BMS);     // BMS CAN
MCP2515 mcp2515_DL(CS_CAN_DL);       // datalogger CAN

#define mcp2515_motor mcp2515_DL
#define mcp2515_BMS mcp2515_DL

constexpr uint8_t NUM_MCP = 3;
MCP2515 MCPS[NUM_MCP] = {mcp2515_motor, mcp2515_BMS, mcp2515_DL};

constexpr uint16_t BUSSIN_MILLIS = 2000;       // The amount of time that the buzzer will buzz for
constexpr uint16_t BMS_OVERRIDE_MILLIS = 1000; // The maximum amount of time to wait for the BMS to start HV, if passed, assume started but not reading response

constexpr uint16_t BRAKE_THRESHOLD = BRAKE_TABLE[0].in; // The threshold for the brake pedal to be considered pressed

bool brake_pressed = false; // boolean for brake light on VCU (for ignition)

/**
 * @brief Global car state structure.
 * @see CarState
 */
struct CarState car = {
    {}, // TelemetryFrameAdc
    {}, // TelemetryFrameDigital
    {}, // TelemetryFrameState
    0,  // millis
    0   // status_millis
};

// Global objects
Pedal pedal(mcp2515_motor, car, car.pedal.apps_5v);
BMS bms(mcp2515_BMS, car);
Telemetry telem(mcp2515_DL, car);

void scheduler_pedal()
{
    pedal.sendFrame();
    pedal.readMotor();
}
void scheduler_bms()
{
    bms.checkHv();
}
void schedulerTelemetryPedal()
{
    telem.sendPedal();
}
void schedulerTelemetryMotor()
{
    telem.sendMotor();
}
void schedulerTelemetryBms()
{
    telem.sendBms();
}

Scheduler<2, NUM_MCP> scheduler(
    10000, // period_us
    500    // spin_threshold_us
);

/**
 * @brief Setup function for initializing the VCU system.
 * Initializes MCP2515s, IO pins, as well as own modules such as Pedal and Debug.
 */
void setup()
{
#if DEBUG_SERIAL
    Debug_Serial::initialize();
    DBGLN_GENERAL("Debug serial initialized");
#endif

    for (uint8_t i = 0; i < NUM_MCP; ++i)
    {
        MCPS[i].reset();
        MCPS[i].setBitrate(CAN_RATE, MCP2515_CRYSTAL_FREQ);
        MCPS[i].setNormalMode();
    }

    // init GPIO pins (MCP2515 CS pins initialized in constructor))
    for (uint8_t i = 0; i < INPUT_COUNT; ++i)
    {
        pinMode(pins_in[i], INPUT);
    }
    for (uint8_t i = 0; i < OUTPUT_COUNT; ++i)
    {
        pinMode(pins_out[i], OUTPUT);
        digitalWrite(pins_out[i], LOW);
    }

#if DEBUG_CAN
    Debug_CAN::initialize(&mcp2515_DL); // Currently using motor CAN for debug messages, should change to other
    DBGLN_GENERAL("Debug CAN initialized");
#endif

    scheduler.addTask(McpIndex::Motor, scheduler_pedal, 1);
    scheduler.addTask(McpIndex::Datalogger, schedulerTelemetryPedal, 1);
    scheduler.addTask(McpIndex::Datalogger, schedulerTelemetryMotor, 1);
    scheduler.addTask(McpIndex::Datalogger, schedulerTelemetryBms, 10);
    DBGLN_GENERAL("Setup complete, entering main loop");
}

/**
 * @brief Main loop function for the VCU system.
 * Handles car state transitions, pipes dataflow between modules.
 */
void loop()
{
    // DBG_HALL_SENSOR(analogRead(HALL_SENSOR));
    car.millis = millis();
    pedal.update(analogRead(APPS_5V), analogRead(APPS_3V3), analogRead(BRAKE_IN));

    brake_pressed = (car.pedal.brake >= BRAKE_THRESHOLD);
    digitalWrite(BRAKE_LIGHT, brake_pressed ? HIGH : LOW);
    scheduler.update(*micros);

    car.pedal.hall_sensor = analogRead(HALL_SENSOR);

    if (car.pedal.status.bits.force_stop)
    {
        car.pedal.status.bits.car_status = CarStatus::Init; // safety, later change to fault status
        digitalWrite(BUZZER, LOW);                          // Turn off buzzer
        digitalWrite(FRG, LOW);                             // Turn off drive mode LED
        return;                                             // If fault force stop is active, do not proceed with the rest of the loop
        // pedal is still being updated, data can still be gathered and sent through CAN/serial
    }

    switch (car.pedal.status.bits.car_status)
    {
    case CarStatus::Drive:
        // send pedal update, done via Scheduler (always on)
        return; // no need logic to check if pedal on, car started

    // do not return here if not in DRIVE mode, else can't detect pedal being on while starting
    case CarStatus::Init:
        DBGLN_THROTTLE("Stopping motor: INIT.");

        if (digitalRead(DRIVE_MODE_BTN) == BUTTON_ACTIVE && brake_pressed)
        {
            car.pedal.status.bits.car_status = CarStatus::Startin;
            car.status_millis = car.millis;

            scheduler.addTask(McpIndex::Bms, scheduler_bms, 5); // check for HV ready in STARTIN
        }
        break;

    case CarStatus::Startin:
        DBGLN_THROTTLE("Stopping motor: STARTIN.");

        if (digitalRead(DRIVE_MODE_BTN) != BUTTON_ACTIVE || !brake_pressed)
        {
            car.pedal.status.bits.car_status = CarStatus::Init;
            car.status_millis = car.millis;
            scheduler.removeTask(McpIndex::Bms, scheduler_bms); // stop checking BMS HV ready since return to INIT
            break;
        }
        if (car.pedal.status.bits.hv_ready)
        {
            car.pedal.status.bits.car_status = CarStatus::Bussin;
            car.status_millis = car.millis;
            digitalWrite(BUZZER, HIGH);
            scheduler.removeTask(McpIndex::Bms, scheduler_bms); // stop checking BMS HV ready since is already ready
            break;
        }
        if (car.millis - car.status_millis >= BMS_OVERRIDE_MILLIS)
        {
            car.pedal.status.bits.car_status = CarStatus::Bussin;
            car.status_millis = car.millis;
            digitalWrite(BUZZER, HIGH);
            scheduler.removeTask(McpIndex::Bms, scheduler_bms); // stop checking BMS HV ready since override to BUSSIN
            break;
        }
        break;

    case CarStatus::Bussin:
        if (car.millis - car.status_millis >= BUSSIN_MILLIS)
        {
            digitalWrite(BUZZER, LOW);
            digitalWrite(FRG, HIGH);
            car.pedal.status.bits.car_status = CarStatus::Drive;
        }
        break;

    default:
        // unreachable, reset to INIT
        car.pedal.status.bits.state_unknown = true;
        car.pedal.status.bits.car_status = CarStatus::Init;
        car.status_millis = car.millis;
        break;
    }

    // DRIVE mode has already returned, if reached here, then means car isn't in DRIVE
    if (pedal.pedal_final > THROTTLE_TABLE[0].in) // if pedal pressed while not in DRIVE, reset to INIT
    {
        car.pedal.status.bits.car_status = CarStatus::Init;
        car.status_millis = car.millis; // Set to current time, in case any counter relies on this
    }
}
