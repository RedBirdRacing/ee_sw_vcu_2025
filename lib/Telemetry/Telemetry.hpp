

// ignore -Wpedantic warnings for mcp2515.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mcp2515.h>
#pragma GCC diagnostic pop

class Telemetry{
public:
    Telemetry();
    void scheduler_adc(MCP2515 *mcp2515_);
    void scheduler_digital(MCP2515 *mcp2515_);
    void scheduler_states(MCP2515 *mcp2515_);

}