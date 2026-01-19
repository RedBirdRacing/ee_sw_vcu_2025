/**
 * @file BMS.cpp
 * @author Planeson, Red Bird Racing
 * @brief Implementation of the BMS class for managing the Accumulator (Kclear BMS) via CAN bus
 * @version 1.1
 * @date 2026-01-13
 * @see BMS.h
 */

#include "BMS.hpp"
#include <Arduino.h> // wait()

// ignore -Wunused-parameter warnings for Debug.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "Debug.hpp" // DBGLN_GENERAL
#pragma GCC diagnostic pop

// ignore -Wpedantic warnings for mcp2515.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mcp2515.h> // CAN frames, sending CAN frame, reading CAN frame
#pragma GCC diagnostic pop

/**
 * @brief Construct a new BMS object
 */
BMS::BMS(MCP2515 &bms_can_)
    : bms_can(bms_can_)
{
}

/**
 * @brief Attempts to start HV.
 * First check BMS is in standby(3) state, then send the HV start command.
 * Keep sending the command until the BMS state changes to precharge(4).
 * Sets hv_started to true when BMS state changes to run(5).
 *
 */
void BMS::checkHv()
{
    if (hv_started)
        return; // already started
    if (bms_can.readMessage(&rx_bms_msg) == MCP2515::ERROR_NOMSG)
    {
        DBG_BMS_STATUS(NO_MSG);
        hv_started = false;
        return;
    }

    // Check if the BMS is in standby state (0x3 in upper 4 bits)
    if (rx_bms_msg.can_id != BMS_INFO_EXT)
    {
        DBG_BMS_STATUS(WRONG_ID);
        hv_started = false;
        return;
    } // Not a BMS info frame, retry

    switch (rx_bms_msg.data[6] & 0xF0)
    {
    case 0x30: // Standby state
        DBG_BMS_STATUS(WAITING);
        bms_can.sendMessage(&start_hv_msg);
        DBGLN_GENERAL("BMS in standby state, sent start HV cmd");
        // sent start HV cmd, wait for BMS to change state
        hv_started = false;
        return;
    case 0x40: // Precharge state
        DBG_BMS_STATUS(STARTING);
        bms_can.sendMessage(&start_hv_msg);
        DBGLN_GENERAL("BMS in precharge state, HV starting");
        hv_started = false;
        return; // BMS is in precharge state, wait
    case 0x50:  // Run state
        DBG_BMS_STATUS(STARTED);
        DBGLN_GENERAL("BMS in run state, HV started");
        hv_started = true; // BMS is in run state
        return;
    default:
        DBG_BMS_STATUS(UNUSED);
        DBGLN_GENERAL("BMS in unknown state, retrying...");
        hv_started = false;
        return; // Unknown state, retry
    }
    hv_started = false; // guard, probably will be optimized out
}
