/**
 * @file Pedal.cpp
 * @author Planeson, Red Bird Racing
 * @brief Implementation of the Pedal class for handling throttle pedal inputs
 * @version 1.6
 * @date 2026-02-12
 * @see Pedal.hpp
 */

#include "Pedal.hpp"
#include "SignalProcessing.hpp"
#include "CarState.hpp"
#include <stdint.h>
#include "Queue.hpp"
#include "CarState.hpp"
#include "Interp.hpp"
#include "Curves.hpp"

// ignore -Wunused-parameter warnings for Debug.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "Debug.hpp" // DBGLN_GENERAL
#pragma GCC diagnostic pop

/**
 * @brief Constructor for the Pedal class.
 * Initializes the pedal state. fault is set to true initially,
 * so you must send update within 100ms of starting the car to clear it.
 * Sends request to motor controller for cyclic RPM and error reads.
 * @param motor_can_ Reference to the MCP2515 instance for motor CAN communication.
 * @param car_ Reference to the CarState structure.
 * @param pedal_final_ Reference to the pedal used as the final pedal value. Although not recommended, you can set another uint16 outside Pedal to be something like 0.3 APPS_1 + 0.7 APPS_2, then reference that here. If in future, this become a sustained need, should consider adding a function pointer to find the final pedal value to let Pedal class call it itself.
 */
Pedal::Pedal(MCP2515 &motor_can_, CarState &car_, uint16_t &pedal_final_)
    : pedal_final(pedal_final_),
      car(car_),
      motor_can(motor_can_),
      fault_start_millis(0),
      last_motor_read_millis(0)
{
    // ask MCU to send motor rpm and error/warn signals
    while (sendCyclicRead(SPEED_IST, RPM_PERIOD) != MCP2515::ERROR_OK)
        ;
    while (sendCyclicRead(WARN_ERR, ERR_PERIOD) != MCP2515::ERROR_OK)
        ;
    // set MCU CAN filter
    while (motor_can.setFilter(MCP2515::RXF0, false, MOTOR_READ) != MCP2515::ERROR_OK)
        ;
}

/**
 * @brief Updates pedal sensor readings, applies filtering, and checks for faults.
 *
 * Stores new pedal readings, applies an average filter, and updates car state.
 * If a fault is detected between pedal sensors, sets fault flags and logs status.
 *
 * @param pedal_1 Raw value from pedal sensor 1.
 * @param pedal_2 Raw value from pedal sensor 2.
 * @param brake Raw value from brake sensor.
 */
void Pedal::update(uint16_t pedal_1, uint16_t pedal_2, uint16_t brake)
{
    // Add new samples to the filters
    pedal1_filter.addSample(pedal_1);
    pedal2_filter.addSample(pedal_2);
    brake_filter.addSample(brake);

    if (pedal_1 < APPS_5V_MIN)
        car.pedal.faults.bits.apps_5v_low = true;
    if (pedal_1 > APPS_5V_MAX)
        car.pedal.faults.bits.apps_5v_high = true;
    if (pedal_2 < APPS_3V3_MIN)
        car.pedal.faults.bits.apps_3v3_low = true;
    if (pedal_2 > APPS_3V3_MAX)
        car.pedal.faults.bits.apps_3v3_high = true;
    if (brake < brake_min)
        car.pedal.faults.bits.brake_low = true;
    if (brake > brake_max)
        car.pedal.faults.bits.brake_high = true;

    if (checkPedalFault())
    {
        // fault now
        if (car.pedal.faults.bits.fault_active)
        {
            // was faulty already, check time
            if (car.millis - fault_start_millis > 100)
            {
                car.pedal.faults.bits.fault_exceeded = true;
                car.pedal.status.bits.force_stop = true; // critical fault, force stop; since early return, need set here
                DBG_THROTTLE_FAULT(PedalFault::DiffExceed100ms);
                return;
            }
            else
            {
                DBG_THROTTLE_FAULT(PedalFault::DiffContinuing); // will be optimized out if the debug macro is off
            }
        }
        else
        {
            // new fault
            fault_start_millis = car.millis;
            DBG_THROTTLE_FAULT(PedalFault::DiffStart);
        }
        car.pedal.faults.bits.fault_active = true;
    }
    else
    {
        // no fault
        if (car.pedal.faults.bits.fault_active)
        {
            DBG_THROTTLE_FAULT(PedalFault::DiffResolved);
        }
        car.pedal.faults.bits.fault_active = false;
    }

    if (car.pedal.faults.byte & FAULT_CHECK_HEX)
    {
        car.pedal.status.bits.force_stop = true; // critical fault, force stop
    }

    return;
}

/**
 * @brief Sends the appropriate CAN frame to the motor based on pedal and car state.
 */
void Pedal::sendFrame()
{
    // Update Telemetry struct
    car.pedal.apps_5v = pedal1_filter.getFiltered();
    car.pedal.apps_3v3 = pedal2_filter.getFiltered();
    car.pedal.brake = brake_filter.getFiltered();

    if (car.pedal.status.bits.force_stop)
    {
        DBGLN_THROTTLE("Stopping motor: pedal fault");
        motor_can.sendMessage(&stop_frame);
        return;
    }
    if (car.pedal.status.bits.car_status != CarStatus::Drive)
    {
        switch (car.pedal.status.bits.car_status)
        {
        case CarStatus::Init:
            DBGLN_THROTTLE("Stopping motor: in INIT.");
            break;
        case CarStatus::Startin:
            DBGLN_THROTTLE("Stopping motor: in STARTIN.");
            break;
        case CarStatus::Bussin:
            DBGLN_THROTTLE("Stopping motor: in BUSSIN.");
            break;
        default:
            DBGLN_THROTTLE("Stopping motor: in UNKNOWN STATE.");
            break;
        }
        motor_can.sendMessage(&stop_frame);
        return;
    }

    car.motor.torque_val = pedalTorqueMapping(pedal_final, car.pedal.brake, car.motor.motor_rpm, FLIP_MOTOR_DIR);

    torque_msg.data[1] = car.motor.torque_val & 0xFF;
    torque_msg.data[2] = (car.motor.torque_val >> 8) & 0xFF;
    motor_can.sendMessage(&torque_msg);
    return;
}

/**
 * @brief Maps the pedal ADC to a torque value.
 * If no braking requested, maps throttle normally.
 * If braking requested and regen enabled,
 *      applies regen if motor RPM larger than minimum regen RPM,
 *      preventing reverse torque at low speeds.
 *      Regen is also disabled if motor rpm isn't read recently to prevent reverse power.
 *
 * @param pedal Pedal ADC in the range of 0-1023.
 * @param brake Brake ADC in the range of 0-1023.
 * @param motor_rpm Current motor RPM for regen logic, scaled to 0-32767.
 * @param flip_dir Boolean indicating whether to flip the motor direction.
 * @return Mapped torque value in the signed range of -TORQUE_MAX to TORQUE_MAX.
 */
constexpr int16_t Pedal::pedalTorqueMapping(const uint16_t pedal, const uint16_t brake, const int16_t motor_rpm, const bool flip_dir)
{
    if (REGEN_ENABLED && brake > BRAKE_MAP.start() && !car.pedal.status.bits.motor_no_read)
    {
        if (pedal > THROTTLE_MAP.start())
        {
            car.pedal.status.bits.screenshot = true;
            // to ensure BSPD can be tested, skip regen if both throttle and brake pressed
        }
        else if (flip_dir)
        {
            if (motor_rpm < PedalConstants::MIN_REGEN_RPM_VAL)
                return 0;
            else
                return -BRAKE_MAP.interp(brake);
        }
        else
        {
            if (motor_rpm > -PedalConstants::MIN_REGEN_RPM_VAL)
                return 0;
            else
                return BRAKE_MAP.interp(brake);
        }
    }

    if (flip_dir)
        return -THROTTLE_MAP.interp(pedal);
    else
        return THROTTLE_MAP.interp(pedal);
}

/**
 * @brief Checks for a fault between two pedal sensor readings.
 *
 * Scales pedal_2 to match the range of pedal_1, then calculates the absolute difference.
 * If the difference exceeds 10% of the full-scale value (i.e., >102.4 for a 10-bit ADC),
 * the function considers this a fault and returns true. Otherwise, returns false.
 *
 * @return true if the difference exceeds the threshold (fault detected), false otherwise.
 */
bool Pedal::checkPedalFault()
{
    const int16_t delta = (int16_t)car.pedal.apps_5v - (int16_t)APPS_3V3_SCALE_MAP.interp(car.pedal.apps_3v3);
    constexpr int16_t MAX_DELTA = THROTTLE_MAP.range() / 10; /**< MAX_DELTA is floor of 10% of APPS_5V valid range, later comparison will give rounding room */
    // if more than 10% difference between the two pedals, consider it a fault
    if (delta > MAX_DELTA || delta < -MAX_DELTA)
    {
        DBG_THROTTLE_FAULT(PedalFault::DiffContinuing, delta);
        return true;
    }
    return false;
}

/**
 * @brief Sends a cyclic read request to the motor controller for speed (rpm).
 * @param reg_id Register ID to read from the motor controller.
 * @param read_period Period of reading motor data in ms.
 * @return MCP2515::ERROR indicating success or failure of sending the message.
 */
MCP2515::ERROR Pedal::sendCyclicRead(uint8_t reg_id, uint8_t read_period)
{
    can_frame cyclic_request = {
        MOTOR_SEND, /**< can_id */
        3,          /**< can_dlc */
        REGID_READ, /**< data, register ID */
        reg_id,     /**< data, sub ID */
        read_period /**< data, read period in ms */
    };
    return motor_can.sendMessage(&cyclic_request);
}

/**
 * @brief Reads motor data from the CAN bus and updates the CarState.
 */
void Pedal::readMotor()
{
    can_frame rx_frame;
    if (motor_can.readMessage(&rx_frame) == MCP2515::ERROR_OK)
    {
        if (rx_frame.can_id == MOTOR_READ && rx_frame.can_dlc > 3)
        {
            if (rx_frame.data[0] == SPEED_IST)
            {
                last_motor_read_millis = car.millis;
                car.pedal.status.bits.motor_no_read = false;
                car.motor.motor_rpm = static_cast<int16_t>(rx_frame.data[1] | (rx_frame.data[2] << 8));
                return;
            }
            else if (rx_frame.data[0] == WARN_ERR)
            {
                car.motor.motor_error = static_cast<uint16_t>(rx_frame.data[1] | (rx_frame.data[2] << 8));
                car.motor.motor_warn = static_cast<uint16_t>(rx_frame.data[3] | (rx_frame.data[4] << 8));
                return;
            }
        }
    }
    if (car.millis - last_motor_read_millis > MAX_MOTOR_READ_MILLIS)
    {
        car.pedal.status.bits.motor_no_read = true;
        DBG_THROTTLE("No motor read for over 100 ms, disabling regen");
    }
    return;
}