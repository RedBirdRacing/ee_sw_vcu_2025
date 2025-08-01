#include "BMS.h"
#include "mcp2515.h" // CAN frames, sending CAN frame,
#include "debug.h"   // DBGLN_GENERAL
#include <Arduino.h> // wait()

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
 * @param tx_bms_msg Pointer to the CAN frame to update.
 * @param rx_bms_msg Pointer to the CAN frame to read the response.
 * @param mcp2515_BMS Pointer to the MCP2515 CAN controller instance.
 * @return None
 */
void BMS::bms_start_hv(can_frame *tx_bms_msg, can_frame *rx_bms_msg, MCP2515 *mcp2515_BMS)
{
    while (true)
    {
        delay(100); // wait for 100ms before sending the command again
        mcp2515_BMS->readMessage(rx_bms_msg);
        // Check if the BMS is in standby state (0x3 in upper 4 bits)
        if (rx_bms_msg->can_id == BMS_INFO_EXT &&
            (rx_bms_msg->data[6] & 0xF0) == 0x30)
        {
            BMS::bms_can_frame_start_hv(tx_bms_msg);
            mcp2515_BMS->sendMessage(tx_bms_msg);
            DBGLN_GENERAL("BMS in standby state, sent start HV cmd");
            // sent start HV cmd, wait for BMS to change state
        }
        // check if BMS is in precharge state (0x4 in upper 4 bits)
        else if (rx_bms_msg->can_id == BMS_INFO_EXT &&
                 (rx_bms_msg->data[6] & 0xF0) == 0x40)
        {
            // BMS is in precharge state, stop sending the command
            DBGLN_GENERAL("BMS in precharge state, HV starting");
        }
        else if (rx_bms_msg->can_id == BMS_INFO_EXT &&
                 (rx_bms_msg->data[6] & 0xF0) == 0x50)
        {
            // BMS is in run state, return
            DBGLN_GENERAL("BMS in run state, HV started");
            return;
        }
    }
}