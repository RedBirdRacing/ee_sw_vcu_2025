/**
 * @file Scheduler.tpp
 * @author Planeson, Red Bird Racing
 * @brief Implementation of the Scheduler class template
 * @version 1.0
 * @date 2025-12-23
 * @see Scheduler.hpp
 *
 */

#include "mcp2515.h"     // mcp2515 objects
#include "Scheduler.hpp" // Scheduler class template declaration
using TaskFn = void (*)(MCP2515 *);

/**
 * @brief Construct a new Scheduler<NUM_TASKS, NUM_MCP2515>::Scheduler object
 *
 * @tparam NUM_TASKS
 * @tparam NUM_MCP2515
 * @param period_us_
 * @param spin_threshold_us_
 * @param mcps_
 */
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>
Scheduler<NUM_TASKS, NUM_MCP2515>::Scheduler(uint32_t period_us_,
                                             uint32_t spin_threshold_us_,
                                             MCP2515 *mcps_[NUM_MCP2515])
    : TASKS{nullptr},
      TASK_TICKS{0},
      PERIOD_US(period_us_),
      SPIN_US(spin_threshold_us_),
      last_fire_us(0),
      task_counters{0} // run on first tick
{
    for (uint8_t i = 0; i < NUM_MCP2515; ++i)
    {
        MCPS[i] = mcps_[i];
    }
}

/**
 * @brief Update the scheduler, checking if tasks need to be run based on the current time
 *
 * @tparam NUM_TASKS
 * @tparam NUM_MCP2515
 * @param current_time_us Function pointer to a function returning the current time in microseconds
 * @return None
 */
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>
void Scheduler<NUM_TASKS, NUM_MCP2515>::update(unsigned long (*current_time_us)())
{
    if (current_time_us == nullptr)
        return;

    uint32_t delta = current_time_us() - last_fire_us;
    if (delta >= PERIOD_US)
    {
        run_tasks();
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
        run_tasks();
        last_fire_us += PERIOD_US;
    }
    return;
}

/**
 * @brief Add a task to the scheduler for a specific MCP2515 index
 *
 * @tparam NUM_TASKS
 * @tparam NUM_MCP2515
 * @param mcp_index Index of the MCP2515 instance
 * @param task Function pointer to the task to be added
 * @param tick_interval Number of ticks between task executions, so 0 for every tick, 9 for every 10 ticks
 * @return None
 */
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>
void Scheduler<NUM_TASKS, NUM_MCP2515>::add_task(uint8_t mcp_index, TaskFn task, uint8_t tick_interval)
{
    if (mcp_index >= NUM_MCP2515 || task == nullptr)
        return;

    if (task_cnt[mcp_index] >= NUM_TASKS)
        return; // no space

    TASKS[mcp_index][task_cnt[mcp_index]] = task;
    TASK_TICKS[mcp_index][task_cnt[mcp_index]] = tick_interval;
    task_counters[mcp_index][task_cnt[mcp_index]] = 1; // run on first tick
    ++task_cnt[mcp_index];
}

/**
 * @brief Remove a task from the scheduler for a specific MCP2515 instance
 *
 * @tparam NUM_TASKS
 * @tparam NUM_MCP2515
 * @param mcp_index Index of the MCP2515 instance
 * @param task Function pointer to the task to be removed
 * @return None
 */
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>
void Scheduler<NUM_TASKS, NUM_MCP2515>::remove_task(uint8_t mcp_index, TaskFn task)
{
    if (mcp_index >= NUM_MCP2515 || task == nullptr)
        return;

    for (uint8_t i = 0; i < task_cnt[mcp_index]; ++i)
    {
        if (TASKS[mcp_index][i] == task)
        {
            // shift left remaining tasks
            for (uint8_t j = i; j < task_cnt[mcp_index] - 1; ++j)
            {
                TASKS[mcp_index][j] = TASKS[mcp_index][j + 1];
                TASK_TICKS[mcp_index][j] = TASK_TICKS[mcp_index][j + 1];
                task_counters[mcp_index][j] = task_counters[mcp_index][j + 1];
            }

            // clean last slot
            TASKS[mcp_index][task_cnt[mcp_index] - 1] = nullptr;
            TASK_TICKS[mcp_index][task_cnt[mcp_index] - 1] = 0;
            task_counters[mcp_index][task_cnt[mcp_index] - 1] = 0;

            --task_cnt[mcp_index];
            return;
        }
    }
}

/**
 * @brief Run scheduled tasks as needed
 *
 * @tparam NUM_TASKS
 * @tparam NUM_MCP2515
 */
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>
inline void Scheduler<NUM_TASKS, NUM_MCP2515>::run_tasks()
{
    for (uint8_t task_index = 0; task_index < NUM_TASKS; ++task_index)
    {
        for (uint8_t mcp_index = 0; mcp_index < NUM_MCP2515; ++mcp_index)
        {
            if (MCPS[mcp_index] == nullptr)
                continue; // invalid MCP2515 pointer

            if (task_counters[mcp_index][task_index] == 0)
                continue; // task slot empty

            if (task_counters[mcp_index][task_index] == 1)
            {
                if (TASKS[mcp_index][task_index] == nullptr)
                    continue; // no task to run

                // call member function on the MCPS instance
                (*TASKS[mcp_index][task_index])(MCPS[mcp_index]);

                // reset counter
                task_counters[mcp_index][task_index] = TASK_TICKS[mcp_index][task_index];
                continue;
            }
            // not time yet, decrement counter
            --task_counters[mcp_index][task_index];
        }
    }
}