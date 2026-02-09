/**
 * @file Enums.hpp
 * @author Planeson, Red Bird Racing
 * @brief Enumeration definitions for the VCU
 * @version 1.3.1
 * @date 2026-02-09
 */

#ifndef ENUMS_HPP
#define ENUMS_HPP
#include <stdint.h>
#include <can.h> // canid_t

/**
 * @brief Main car status state machine.
 *
 * Represents the current state of the car in the startup and drive sequence.
 */
enum class CarStatus : uint8_t
{
    Init = 0,    /**< Just started the car */
    Startin = 1, /**< Driver holds "Start" button and full brakes (transition state) */
    Bussin = 2,  /**< Buzzer active, driver can release "Start" and brakes (transition state) */
    Drive = 3    /**< Ready to drive, Drive mode LED on, throttle enabled */
};

/**
 * @brief Pedal fault status codes.
 *
 * Represents the current fault status related to pedal input.
 */
enum class PedalFault : uint8_t
{
    None = 0x00,            /**< No fault detected */
    DiffStart = 0x10,       /**< >10% difference fault just started */
    DiffContinuing = 0x11,  /**< >10% difference fault is ongoing */
    DiffExceed100ms = 0x12, /**< >10% difference fault exceeded 100ms */
    DiffResolved = 0x19,    /**< Difference fault resolved */
    ThrottleLow = 0x20,     /**< Throttle pedal below lower threshold */
    ThrottleHigh = 0x29,    /**< Throttle pedal above upper threshold */
    BrakeLow = 0x30,        /**< Brake pedal below lower threshold */
    BrakeHigh = 0x39,       /**< Brake pedal above upper threshold */
};

/**
 * @brief BMS status
 *
 * Represents the current state of the Battery Management System (BMS).
 */
enum class BmsStatus : uint8_t
{
    NoMsg = 0,    /**< No message received from BMS */
    //WrongId = 1,  /**< Received message with wrong CAN ID */
    Waiting = 1,  /**< BMS is in standby, waiting to start HV */
    Starting = 2, /**< BMS is starting high voltage */
    Started = 3,  /**< BMS has started high voltage */
    Unused = 4    /**< Unused status code */
};

/**
 * @brief MCP2515 instance indices.
 *
 * Used to identify different MCP2515 CAN controller instances.
 */
enum class McpIndex : uint8_t
{
    Motor = 0,     /**< Motor CAN MCP2515 instance */
    Bms = 1,       /**< BMS CAN MCP2515 instance */
    Datalogger = 2 /**< Datalogger CAN MCP2515 instance */
};

// === CAN IDs ===

/**
 * @brief CAN message IDs for status and brake debug.
 *
 * Used for sending car status and brake messages over CAN bus.
 */
enum class StatusCanId : canid_t
{
    CarMsg = 0x693,          /**< Debug: car status message */
    StaCarChangeMsg = 0x694, /**< Debug: car status change message */
    BrakeMsg = 0x695,        /**< Debug: brake status message */
    BmsMsg = 0x696,          /**< Debug: BMS status message */
    HallSensorMsg = 0x697    /**< Debug: Hall sensor message */
};

#endif // ENUMS_HPP