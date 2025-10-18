#include "BMS.h"
#include "mcp2515.h" // CAN frames, sending CAN frame, reading CAN frame
#include "debug.h"   // DBGLN_GENERAL
#include <Arduino.h> // wait()

// Constructor
BMS::BMS(MCP2515 *mcp2515_BMS)
    : last_msg_ms(0),
      read_start_ms(0),
      tx_bms_start_msg{
          BMS_SEND_CMD, // can_id
          2,            // can_dlc
          {0x01, 0x01}  // data: MainRlyCmd = 1 (output HV), ShutDownCmd = 1 (do not shutdown)
      },
      tx_bms_stop_msg{
          BMS_SEND_CMD, // can_id
          2,            // can_dlc
          {0x00, 0x00}  // data: MainRlyCmd = 0 (break open HV), ShutDownCmd = 0 (shutdown)
      },
      rx_bms_msg{
          0,   // can_id
          0,   // can_dlc
          {0}, // data
      },
      mcp2515_BMS(mcp2515_BMS)
{
}

/**
 * @brief Updates the CAN frame with the current pedal values.
 *
 * This function prepares a CAN frame to stop the BMS high voltage output.
 *
 * @param tx_bms_msg Pointer to the CAN frame to update.
 * @return None
 */
void BMS::bms_can_frame_start_hv(can_frame *tx_bms_msg)
{
    tx_bms_msg->can_id = BMS_SEND_CMD; // extended already
    tx_bms_msg->can_dlc = 2;
    // MainRlyCmd
    // 1 is output HV
    tx_bms_msg->data[0] = 0x01;

    // ShutDownCmd
    // 1 is do not shutdown
    tx_bms_msg->data[1] = 0x01;
}

/**
 * @brief Updates the CAN frame with the current pedal values.
 *
 * This function prepares a CAN frame to stop the BMS high voltage output.
 *
 * @param tx_bms_msg Pointer to the CAN frame to update.
 * @return None
 */
void BMS::bms_can_frame_stop_hv(can_frame *tx_bms_msg)
{
    tx_bms_msg->can_id = BMS_SEND_CMD; // extended already
    tx_bms_msg->can_dlc = 2;
    // MainRlyCmd
    // 0 is break open HV
    tx_bms_msg->data[0] = 0x00;

    // ShutDownCmd
    // 0 is shutdown
    tx_bms_msg->data[1] = 0x00;
}

/**
 * @brief Keep retrying to send BMS start HV command until it succeeds.
 * First check BMS is in standby(3) state, then send the HV start command.
 * Keep sending the command until the BMS state changes to precharge(4).
 * Returns when BMS state changes to run(5).
 *
 * @param rx_bms_msg Pointer to the CAN frame to read the response.
 * @param mcp2515_BMS Pointer to the MCP2515 CAN controller instance.
 * @return None
 */
void BMS::check_hv()
{
    hv_started = true;
    return; // TEMP disable BMS HV start logic for testing without BMS
    if (hv_started)
        return; // already started
    if (mcp2515_BMS->readMessage(&rx_bms_msg) == MCP2515::ERROR_NOMSG)
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
        hv_send_start = true;
        // last_msg_ms = *current_ms;
        DBGLN_GENERAL("BMS in standby state, sent start HV cmd");
        // sent start HV cmd, wait for BMS to change state
        hv_started = false;
        return;
    case 0x40: // Precharge state
        DBG_BMS_STATUS(STARTING);
        hv_send_start = false;
        DBGLN_GENERAL("BMS in precharge state, HV starting");
        hv_started = false;
        return; // BMS is in precharge state, wait
    case 0x50:  // Run state
        DBG_BMS_STATUS(STARTED);
        hv_send_start = false;
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

