#ifndef BMS_H
#define BMS_H

// ignore -Wpedantic warnings for mcp2515.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mcp2515.h>
#pragma GCC diagnostic pop

constexpr uint32_t BMS_COMMAND = 0x1801F340;                  // BMS command ID
constexpr uint32_t BMS_SEND_CMD = BMS_COMMAND | CAN_EFF_FLAG; // Extended Frame Format flag

constexpr uint32_t BMS_INFO = 0x186040F3;                  // BMS info ID
constexpr uint32_t BMS_INFO_EXT = BMS_INFO | CAN_EFF_FLAG; // Extended Frame Format flag

constexpr uint16_t BMS_READ_MS = 10; // BMS reading max time

class BMS
{
public:
    BMS(MCP2515 *mcp2515_BMS);
    bool hv_ready() { return hv_started; };
    void check_hv(MCP2515 *mcp2515_BMS);

private:
    uint32_t last_msg_ms, read_start_ms;
    can_frame tx_bms_start_msg, tx_bms_stop_msg, rx_bms_msg;
    MCP2515 *mcp2515_BMS;
    bool hv_started = false;
    bool hv_send_start = false;

    void bms_can_frame_start_hv(can_frame *tx_bms_msg);
    void bms_can_frame_stop_hv(can_frame *tx_bms_msg);
};
#endif // BMS_H