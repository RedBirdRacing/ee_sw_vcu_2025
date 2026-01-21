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
 *
 * @details The Scheduler class holds function pointers to tasks to be run on multiple MCP2515 instances.
 * Each task is associated with a specific MCP2515 instance and a tick interval.
 * The Scheduler runs on a fixed period. When Scheduler.update() is called, it checks if a period has already passed.
 * If not, it checks if it is almost passed. If the time to the next cycle is less than SPIN_US, it spin-waits until the period is reached.
 * Otherwise, it returns immediately, allowing other non-scheduler tasks to run.
 *
 * In the rare case where the system is busy and misses more than one period, the scheduler will skip to the next period, preventing bursts.
 *
 * The Scheduler class doesn't hold the MCP2515 instances itself, but rather pointers to them. Therefore, you must create the array externally and pass it to the Scheduler constructor.
 *
 * @tparam NUM_TASKS Number of tasks per MCP2515, choose highest of all, but keep as low as possible
 * @tparam NUM_MCP2515 Number of MCP2515 instances
 */
class Scheduler
{
public:
    using TaskFn = void (*)(MCP2515 *);

    Scheduler() = delete; /**< all arguments must be provided */
    Scheduler(uint32_t period_us_, uint32_t spin_threshold_us_, MCP2515 (&mcps_)[NUM_MCP2515]);
    // no need destructor, since no dynamic memory allocation, and won't destruct in the middle of the program anyway

    void update(unsigned long (*const current_time_us)());
    bool addTask(const mcp_index mcp_index, const TaskFn task, const uint8_t tick_interval);
    bool removeTask(const mcp_index mcp_index, const TaskFn task);

    uint8_t cycle_count = 0; /**< counts number of scheduler cycles since start, useful for other timers. */

    /**
     * @brief Returns the period of the scheduler in microseconds.
     * @return The period in microseconds.
     */
    constexpr uint32_t getPeriodUs() const { return PERIOD_US; }

    /**
     * @brief Returns the number of cycles needed for a given interval in microseconds.
     * @param[in] interval_us The interval in microseconds.
     * @return The number of cycles needed.
     */
    constexpr uint32_t cyclesNeeded(const uint32_t interval_us) const { return interval_us / PERIOD_US; }

private:
    TaskFn tasks[NUM_MCP2515][NUM_TASKS];          /**< Array of tasks, sorted by each MCP2515. */
    uint8_t task_ticks[NUM_MCP2515][NUM_TASKS];    /**< Period (in ticks) of each function, 1 is fire every tick, 0 is disabled. */
    uint8_t task_counters[NUM_MCP2515][NUM_TASKS]; /**< Counter to hold firing for n ticks, "how many ticks left before firing?". */
    uint8_t task_cnt[NUM_MCP2515];                 /**< Array of number of tasks per MCP2515. */
    const uint32_t PERIOD_US;                      /**< Period (tick length). */
    const uint32_t SPIN_US;                        /**< Threshold to switch from letting non-scheduler task in loop() run, to spin-locking (to ensure on time firing). */
    uint32_t last_fire_us;                         /**< Last time scheduler fired, overridden if missed more than one period. */
    MCP2515 (&MCPS)[NUM_MCP2515]; /**< Reference to array of non-const MCP2515 objects. */

    inline void runTasks();
};

#include "Scheduler.tpp"

#endif // SCHEDULER_HPP