/**
 * @file Scheduler.hpp
 * @author Planeson, Red Bird Racing
 * @brief Declaration of the Scheduler class template, for scheduling tasks on multiple MCP2515 instances
 * @version 1.1
 * @date 2026-01-13
 * @see Scheduler.tpp
 */

#ifndef SCHEDULER_HPP
#define SCHEDULER_HPP

#include "Enums.h"

// ignore -Wpedantic warnings for mcp2515.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mcp2515.h> // mcp2515 objects
#pragma GCC diagnostic pop

// template because we can't declare the size of the arrays without macros here
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>

/**
 * @brief Scheduler class template for scheduling tasks on multiple MCP2515 instances
 * Takes in function pointers to member functions of MCP2515 class,
 * and calls them at specified intervals, using a unified central ticker.
 * @tparam NUM_TASKS Number of tasks per MCP2515, choose highest of all, but keep as low as possible
 * @tparam NUM_MCP2515 Number of MCP2515 instances
 */
class Scheduler
{
public:
    using TaskFn = void (*)(MCP2515 *);

    Scheduler() = delete; // all arguments must be provided
    Scheduler(uint32_t period_us_, uint32_t spin_threshold_us_, MCP2515 *const *const mcps_);
    // no need destructor, since no dynamic memory allocation, and won't destruct in the middle of the program anyway

    void update(unsigned long (*const current_time_us)());
    bool addTask(const mcp_index mcp_index, const TaskFn task, const uint8_t tick_interval);
    bool removeTask(const mcp_index mcp_index, const TaskFn task);

    uint8_t cycle_count = 0; /**< counts number of scheduler cycles since start, useful for other timers */

    /**
     * @brief Returns the period of the scheduler in microseconds
     * @return The period in microseconds
     */
    constexpr uint32_t getPeriodUs() const { return PERIOD_US; }

    /**
     * @brief Returns the number of cycles needed for a given interval in microseconds
     * @param[in] interval_us The interval in microseconds
     * @return The number of cycles needed
     */
    constexpr uint32_t cyclesNeeded(const uint32_t interval_us) const { return interval_us / PERIOD_US; }

private:
    TaskFn tasks[NUM_MCP2515][NUM_TASKS];          /**< Array of tasks, sorted by each MCP2515. */
    uint8_t task_ticks[NUM_MCP2515][NUM_TASKS];    /**< Period (in ticks) of each function, 1 is fire every tick, 0 is disabled */
    const uint32_t PERIOD_US;                      /**< Period (tick length) */
    const uint32_t SPIN_US;                        /**< Threshold to switch from letting non-scheduler task in loop() run, to spin-locking (to ensure on time firing) */
    uint32_t last_fire_us;                         /**< Last time scheduler fired, overridden if missed more than one period */
    uint8_t task_counters[NUM_MCP2515][NUM_TASKS]; /**< Counter to hold firing for n ticks, "how many ticks left before firing" */
    MCP2515 *const *const MCPS;                    /**< Array of MCP2515 pointers */
    uint8_t task_cnt[NUM_MCP2515];                 /**< Array of number of tasks per MCP2515 */

    inline void runTasks();
};

#include "Scheduler.tpp"

#endif // SCHEDULER_HPP