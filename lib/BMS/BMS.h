#ifndef BMS_H
#define BMS_H
#include "mcp2515.h"

const uint32_t BMS_COMMAND = 0x1801F340; // BMS command ID
const uint32_t BMS_SEND_CMD = BMS_COMMAND | CAN_EFF_FLAG; // Extended Frame Format flag

const uint32_t BMS_INFO = 0x186040F3; // BMS info ID
const uint32_t BMS_INFO_EXT = BMS_INFO | CAN_EFF_FLAG; // Extended Frame Format flag

namespace BMS
{
    void bms_can_frame_start_hv(can_frame *tx_bms_msg);
    void bms_can_frame_stop_hv(can_frame *tx_bms_msg);
    void bms_start_hv(can_frame *tx_bms_msg, can_frame *rx_bms_msg, MCP2515 *mcp2515_BMS);
}
#endif // BMS_H