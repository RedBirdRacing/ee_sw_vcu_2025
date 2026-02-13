/**
 * @file Scheduler.tpp
 * @author Planeson, Red Bird Racing
 * @brief Implementation of the Scheduler class template
 * @version 1.1
 * @date 2026-01-13
 * @see Scheduler.hpp
 */

#include "Enums.hpp"

// ignore -Wpedantic warnings for mcp2515.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <mcp2515.h> // mcp2515 objects
#pragma GCC diagnostic pop

#include "Scheduler.hpp" // Scheduler class template declaration
using TaskFn = void (*)(MCP2515 *);

/**
 * @brief Construct a new Scheduler object
 *
 * @tparam NUM_TASKS Number of tasks per MCP2515
 * @tparam NUM_MCP2515 Number of MCP2515 instances
 * @param[in] period_us_ Period of the scheduler in microseconds
 * @param[in] spin_threshold_us_ Spin-wait threshold in microseconds
 */
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>
Scheduler<NUM_TASKS, NUM_MCP2515>::Scheduler(uint32_t period_us_,
                                             uint32_t spin_threshold_us_)
    : tasks{nullptr},
      task_ticks{0},
      task_counters{0}, // run on first tick
      task_cnt{0},
      PERIOD_US(period_us_),
      SPIN_US(spin_threshold_us_),
      last_fire_us(0)
{
}

/**
 * @brief Update the scheduler, checking if tasks need to be run based on the current time
 *
 * @tparam NUM_TASKS Number of tasks per MCP2515
 * @tparam NUM_MCP2515 Number of MCP2515 instances
 * @param[in] current_time_us Function pointer to a function returning the current time in microseconds
 */
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>
void Scheduler<NUM_TASKS, NUM_MCP2515>::update(unsigned long (*const current_time_us)())
{
    if (current_time_us == nullptr)
        return;

    uint32_t delta = current_time_us() - last_fire_us;
    if (delta >= PERIOD_US)
    {
        runTasks();
        if (delta >= 2 * PERIOD_US)
            // we missed more than one period, override last_fire_us to avoid bursts
            last_fire_us = current_time_us();
        else
            last_fire_us += PERIOD_US;

        return;
    }
    // not time yet, check if we should spin-wait or return
    if (delta >= PERIOD_US - SPIN_US)
    {
        // spin-wait
        while ((uint32_t)(current_time_us() - last_fire_us) < PERIOD_US)
            ;
        // now it's time, run the tasks
        runTasks();
        last_fire_us += PERIOD_US;
    }
    return;
}

/**
 * @brief Synchonize the scheduler to the current time, resetting all task counters, used when starting multiple Schedulers across different boards together
 * 
 * @tparam NUM_TASKS Number of tasks per MCP2515
 * @tparam NUM_MCP2515 Number of MCP2515 instances
 * @param[in] current_time_us Function pointer to a function returning the current time in microseconds
 */
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>
void Scheduler<NUM_TASKS, NUM_MCP2515>::synchronize(unsigned long (*const current_time_us)())
{
    if (current_time_us == nullptr)
        return;

    last_fire_us = current_time_us();
    for (uint8_t mcp_index = 0; mcp_index < NUM_MCP2515; ++mcp_index)
    {
        for (uint8_t task_index = 0; task_index < NUM_TASKS; ++task_index)
        {
            task_counters[mcp_index][task_index] = task_ticks[mcp_index][task_index];
        }
    }
}

/**
 * @brief Add a task to the scheduler for a specific MCP2515 index
 *
 * @tparam NUM_TASKS Number of tasks per MCP2515
 * @tparam NUM_MCP2515 Number of MCP2515 instances
 * @param[in] mcp_index Index of the MCP2515 instance
 * @param[in] task Function pointer to the task to be added
 * @param[in] tick_interval Number of ticks between task executions, so 1 for every tick, 10 for every 10 ticks
 * @return true if the task was added successfully, false otherwise
 */
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>
bool Scheduler<NUM_TASKS, NUM_MCP2515>::addTask(const McpIndex mcp_index, const TaskFn task, const uint8_t tick_interval)
{
    uint8_t mcp_idx = static_cast<uint8_t>(mcp_index);
    if (mcp_idx >= NUM_MCP2515 || task == nullptr)
        return false;

    if (task_cnt[mcp_idx] >= NUM_TASKS)
        return false; // no space

    tasks[mcp_idx][task_cnt[mcp_idx]] = task;
    task_ticks[mcp_idx][task_cnt[mcp_idx]] = tick_interval;
    task_counters[mcp_idx][task_cnt[mcp_idx]] = 1; // run on first tick
    ++task_cnt[mcp_idx];
    return true;
}

/**
 * @brief Remove a task from the scheduler for a specific MCP2515 instance
 *
 * @tparam NUM_TASKS Number of tasks per MCP2515
 * @tparam NUM_MCP2515 Number of MCP2515 instances
 * @param[in] mcp_index Index of the MCP2515 instance
 * @param[in] task Function pointer to the task to be removed
 * @return true if the task was removed successfully, false otherwise
 */
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>
bool Scheduler<NUM_TASKS, NUM_MCP2515>::removeTask(const McpIndex mcp_index, const TaskFn task)
{
    uint8_t mcp_idx = static_cast<uint8_t>(mcp_index);
    if (mcp_idx >= NUM_MCP2515 || task == nullptr)
        return false;

    for (uint8_t i = 0; i < task_cnt[mcp_idx]; ++i)
    {
        if (tasks[mcp_idx][i] == task)
        {
            // shift left remaining tasks
            for (uint8_t j = i; j < task_cnt[mcp_idx] - 1; ++j)
            {
                tasks[mcp_idx][j] = tasks[mcp_idx][j + 1];
                task_ticks[mcp_idx][j] = task_ticks[mcp_idx][j + 1];
                task_counters[mcp_idx][j] = task_counters[mcp_idx][j + 1];
            }

            // clean last slot
            tasks[mcp_idx][task_cnt[mcp_idx] - 1] = nullptr;
            task_ticks[mcp_idx][task_cnt[mcp_idx] - 1] = 0;
            task_counters[mcp_idx][task_cnt[mcp_idx] - 1] = 0;

            --task_cnt[mcp_idx];
            return true;
        }
    }
    return false;
}

/**
 * @brief Helper function to run scheduled tasks
 *
 * @tparam NUM_TASKS Number of tasks per MCP2515
 * @tparam NUM_MCP2515 Number of MCP2515 instances
 */
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>
inline void Scheduler<NUM_TASKS, NUM_MCP2515>::runTasks()
{

    for (uint8_t task_index = 0; task_index < NUM_TASKS; ++task_index)
    {
        for (uint8_t mcp_index = 0; mcp_index < NUM_MCP2515; ++mcp_index)
        {
            if (task_counters[mcp_index][task_index] == 0)
                continue; // task slot empty

            if (task_counters[mcp_index][task_index] == 1)
            {
                if (tasks[mcp_index][task_index] == nullptr)
                    continue; // no task to run

                // call functions
                (tasks[mcp_index][task_index])();

                // reset counter
                task_counters[mcp_index][task_index] = task_ticks[mcp_index][task_index];
                continue;
            }
            // not time yet, decrement counter
            --task_counters[mcp_index][task_index];
        }
    }
}