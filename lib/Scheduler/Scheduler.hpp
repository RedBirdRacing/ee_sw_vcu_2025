/**
 * @file Scheduler.hpp
 * @author Planeson, Red Bird Racing
 * @brief Declaration of the Scheduler class template, for scheduling tasks on multiple MCP2515 instances
 * @version 1.0
 * @date 2025-12-23
 * @see Scheduler.tpp
 */

#ifndef SCHEDULER_HPP
#define SCHEDULER_HPP

#include "mcp2515.h" // mcp2515 objects

// template because we can't declare the size of the arrays without macros here
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>

class Scheduler
{
public:
    // delete default constructor, i.e. all arguments must be provided
    Scheduler() = delete;

    using TaskFn = void (*)(MCP2515 *);

    // constructor

    Scheduler(uint32_t period_us_,
              uint32_t spin_threshold_us_,
              MCP2515 *mcps_[NUM_MCP2515]);

    // no need destructor, since no dynamic memory allocation, and won't destruct in the middle of the program anyway

    void update(unsigned long (*current_time_us)());
    void add_task(uint8_t mcp_index, TaskFn task, uint8_t tick_interval);
    void remove_task(uint8_t mcp_index, TaskFn task);

    // array of function pointers to tasks
    TaskFn TASKS[NUM_MCP2515][NUM_TASKS] = {nullptr};

    // stores the number of ticks between two runs of the task
    // for instance, if a task runs every 10 ticks, store 10
    // if a task runs every tick, store 1
    // 0 means disabled
    uint8_t TASK_TICKS[NUM_MCP2515][NUM_TASKS] = {0};

private:
    // how the period of the scheduler, i.e. time between two fires of the scheduler, in microseconds
    // e.g. 10000 for 10ms cycle time
    // note that if missed more than 1 period, next period is set to current time + this value, so would be slightly late
    const uint32_t PERIOD_US;

    // threshold to switch from letting non-scheduler task in loop() run, to busy-waiting (to ensure on time firing)
    // e.g. 100 for a 0.1ms
    const uint32_t SPIN_US;

    // last time the scheduler was fired
    // overriden when if we miss more than one period, to avoid bursts
    uint32_t last_fire_us;

    // counter of the functions per MCP2515, how many times the scheduler fires for that function to run
    // for instance, if BMS tasks every 10 ticks, then initially store 10, and decrement every tick, when it reaches 1, run the task and reset to 10
    uint8_t task_counters[NUM_MCP2515][NUM_TASKS];

    // array of mcp2515 pointers
    MCP2515 *MCPS[NUM_MCP2515];

    // array to count number of tasks per MCP2515
    uint8_t task_cnt[NUM_MCP2515] = {0};

    // helper function to run tasks
    inline void run_tasks();
};

#include "Scheduler.tpp"

#endif // SCHEDULER_HPP