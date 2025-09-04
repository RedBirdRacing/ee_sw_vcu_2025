#include <Arduino.h>
#include "pinMap.h"
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
const uint8_t INPUT_COUNT = 5;
const uint8_t pins_in[INPUT_COUNT] = {DRIVE_MODE_BTN, APPS_5V, APPS_3V3, HV_CURRENT, BRAKE_IN};
const uint8_t OUTPUT_COUNT = 3;
const uint8_t pins_out[OUTPUT_COUNT] = {DRIVE_MODE_LED, BRAKE_LIGHT, BUZZER_OUT};

// === Pedal ===
Pedal pedal;

// === CAN (motor) ===
MCP2515 mcp2515_motor(CS_CAN_MOTOR);

// === CAN (BMS) ===
MCP2515 mcp2515_BMS(CS_CAN_BMS);

// === CAN (Datalogger) ===
MCP2515 mcp2515_DL(CS_CAN_DL); // currently used for debug CAN only

struct can_frame tx_throttle_msg;
struct can_frame tx_bms_msg;
struct can_frame rx_bms_msg;

const uint16_t STARTING_MILLIS = 2000; // The amount of time that the driver needs to hold the "Start" button and full brakes in order to activate driving mode
const uint16_t BUSSIN_MILLIS = 2000;   // The amount of time that the buzzer will buzz for

const uint16_t BRAKE_THRESHOLD = 256; // The threshold for the brake pedal to be considered pressed, not full / min because we don't need the brakes to be fully pressed to start, just sufficiently down

bool brake_pressed = false; // boolean for brake light on VCU (for ignition)

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


/**
 * === Fast analogRead() ===
 * Uses atmega328 registers directly to read analog values faster than the standard analogRead() function.
 */

// Define channels
#define NUM_ADC_CHANNELS 4
const uint8_t adc_channels[NUM_ADC_CHANNELS] = {APPS_5V, APPS_3V3, HV_CURRENT, BRAKE_IN}; // {ADC0, ADC1, ADC2} = {PC0, PC1, PC2}
volatile uint16_t adc_results[NUM_ADC_CHANNELS];
volatile uint8_t current_channel = 0;

// Free-running ADC interrupt for multi-channel sampling
ISR(ADC_vect) {
    // Read result
    uint8_t low = ADCL;
    uint8_t high = ADCH;
    adc_results[current_channel] = (high << 8) | low;

    // Move to next channel
    current_channel = (current_channel + 1) % NUM_ADC_CHANNELS;
    ADMUX = (ADMUX & 0xF0) | (current_channel & 0x0F); // Set next channel in ADMUX
} 

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
        pinMode(pins_in[i], INPUT);
    // Init output pins
    for (int i = 0; i < OUTPUT_COUNT; i++)
        pinMode(pins_out[i], OUTPUT);

    // Init mcp2515 for motor CAN channel
    mcp2515_motor.reset();
    mcp2515_motor.setBitrate(CAN_500KBPS, MCP_20MHZ);
    mcp2515_motor.setNormalMode();

    // Init mcp2515 for BMS channel
    mcp2515_BMS.reset();
    mcp2515_BMS.setBitrate(CAN_500KBPS, MCP_20MHZ);
    mcp2515_BMS.setNormalMode();

    // Init mcp2515 for DL channel
    mcp2515_DL.reset();
    mcp2515_DL.setBitrate(CAN_500KBPS, MCP_20MHZ);
    mcp2515_DL.setNormalMode();

#if DEBUG_CAN
    Debug_CAN::initialize(&mcp2515_DL); // Currently using datalogger CAN for debug messages, should change to other
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

    // === ADC Free-Running Mode Setup ===
    // Page 217 of https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
    //          == https://www.arnabkumardas.com/arduino-tutorial/adc-register-description/
    // Sample code https://github.com/VinnieM-3/Arduino/blob/master/ADC_FreeRunning.ino
    
    // AVcc as reference
    //ADMUX &= ~(1 << REFS1);
    //ADMUX |= (1 << REFS0);

    // ARef as reference
    ADMUX &= ~((1 << REFS1) | (1 << REFS0));

    // Right adjust result
    ADMUX &= ~(1 << ADLAR);

    // Select first channel
    ADMUX = (ADMUX & 0xF0) | 0x00; // Clear lower 4 bits and set to ADC0 (APPS_5V)

    // Enable ADC
    ADCSRA |= (1 << ADEN);
    // Enable auto trigger
    ADCSRA |= (1 << ADATE);
    // Enable ADC interrupt
    ADCSRA |= (1 << ADIE);
    // Prescaler 128 for 16MHz/128 = 125kHz ADC clock
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    // Disable Power Reduction ADC bit
    PRR &= ~(1 << PRADC);
    
    // Free running mode
    ADCSRB &= 0xF8;
    // Enable global interrupts
    sei();
    // Start first conversion
    ADCSRA |= (1 << ADSC);
}

void loop()
{
    main_car_state.millis = millis(); // Update the current millis time
    // Read pedals
    pedal.pedal_update(&main_car_state, adc_results[1], adc_results[2], adc_results[0]);

    brake_pressed = static_cast<uint16_t>(analogRead(BRAKE_IN)) >= BRAKE_THRESHOLD;
    digitalWrite(BRAKE_LIGHT, brake_pressed ? HIGH : LOW);
    /*
    For the time being:
    DRIVE_MODE_BTN = "Start" button
    BRAKE_IN = Brake pedal
    BUZZER_OUT = Buzzer output
    DRIVE_MODE_LED = "Drive" mode indicator
    */
    if (main_car_state.fault_force_stop)
    {
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        DBGLN_THROTTLE("Stopping motor: Fault exceeded 100ms.");
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        main_car_state.car_status = INIT;  // safety, later change to fault status
        digitalWrite(BUZZER_OUT, LOW);     // Turn off buzzer
        digitalWrite(DRIVE_MODE_LED, LOW); // Turn off drive mode LED
        return;                            // If fault force stop is active, do not proceed with the rest of the loop
        // pedal is still being updated, data can still be gathered and sent through CAN/serial
    }

    switch (main_car_state.car_status)
    {
        // I'm unsure if the compiler is using a jump table or if else, putting DRIVE at the top for efficiency.
        case DRIVE:
            // Pedal update
            // Send pedal value through canbus
            pedal.pedal_can_frame_update(&tx_throttle_msg, &main_car_state);
            mcp2515_motor.sendMessage(&tx_throttle_msg);
            return;

        // do not return here if not in DRIVE mode, else can't detect pedal being on while starting
        case INIT:
        pedal.pedal_can_frame_stop_motor(&tx_throttle_msg);
        DBGLN_THROTTLE("Stopping motor: INIT.");
        mcp2515_motor.sendMessage(&tx_throttle_msg);

        if (digitalRead(DRIVE_MODE_BTN) == HIGH && brake_pressed)
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

        if (digitalRead(DRIVE_MODE_BTN) == LOW || !brake_pressed)
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
            BMS::bms_start_hv(&tx_bms_msg, &rx_bms_msg, &mcp2515_BMS);
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
        DBG_STATUS_CAR_CHANGE(THROTTLE_TO_INIT);
    }
}
