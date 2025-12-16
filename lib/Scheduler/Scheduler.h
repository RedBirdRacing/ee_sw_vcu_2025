#include "mcp2515.h" // mcp2515 objects

/**
 * @brief Scheduler class for managing tasks on multiple MCP2515 instances
 *
 * Take a note on examples on how to define tasks and use the scheduler.
 */

// template because we can't declare the size of the arrays without macros here
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>

// assume we only schedule CAN bus sends/reads, so all tasks are can_frame MCP2515 member functions
class Scheduler
{
public:
    // store pointer-to-member that takes a can_frame* (adjust return type if your API differs)
    // delete default constructor, i.e. all arguments must be provided
    Scheduler() = delete;

    using TaskFn = void (*)(MCP2515 *);
    // constructor
    Scheduler(uint32_t period_us_,
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
    };

    // no need destructor, since no dynamic memory allocation, and won't destruct in the middle of the program anyway

    /**
     * @brief Update the scheduler, checking if tasks need to be run based on the current time
     *
     * @param current_time_us Function pointer to a function returning the current time in microseconds
     * @return None
     */
    void update(unsigned long (*current_time_us)())
    {
        // check if it's time to fire
        uint32_t delta = current_time_us() - last_fire_us;
        if (delta >= PERIOD_US)
        {
            run_tasks();
            if (delta >= 2 * PERIOD_US)
            {
                // we missed more than one period, override last_fire_us to avoid bursts
                last_fire_us = current_time_us();
            }
            else
            {
                last_fire_us += PERIOD_US;
            }
        }
        else
        {
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
            else
            {
                return; // return to let other non-scheduler tasks run
            }
        }
    }

    /**
     * @brief Add a task to the scheduler for a specific MCP2515 instance
     *
     * @param mcp_index Index of the MCP2515 instance
     * @param task Function pointer to the task to be added
     * @param tick_interval Number of ticks between task executions, so 0 for every tick, 9 for every 10 ticks
     * @return None
     */
    void add_task(uint8_t mcp_index, TaskFn task, uint8_t tick_interval)
    {
        if (mcp_index >= NUM_MCP2515)
            return;
        TASKS[mcp_index][task_cnt[mcp_index]] = task;
        TASK_TICKS[mcp_index][task_cnt[mcp_index]] = tick_interval;
        task_counters[mcp_index][task_cnt[mcp_index]] = 1; // run on first tick
        ++task_cnt[mcp_index];
    }

    /**
     * @brief Remove a task from the scheduler for a specific MCP2515 instance
     *
     * @param mcp_index Index of the MCP2515 instance
     * @param task Function pointer to the task to be removed
     * @return None
     */
    void remove_task(uint8_t mcp_index, TaskFn task)
    {
        if (mcp_index >= NUM_MCP2515)
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
                TASKS[mcp_index][task_cnt[mcp_index] - 1] = nullptr;
                TASK_TICKS[mcp_index][task_cnt[mcp_index] - 1] = 0;
                task_counters[mcp_index][task_cnt[mcp_index] - 1] = 0;
                --task_cnt[mcp_index];
                return;
            }
        }
    }

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
    inline void run_tasks()
    {
        for (uint8_t task_index = 0; task_index < NUM_TASKS; ++task_index)
        {
            for (uint8_t mcp_index = 0; mcp_index < NUM_MCP2515; ++mcp_index)
            {
                if (task_counters[mcp_index][task_index] == 0)
                    continue; // task slot empty
                if (task_counters[mcp_index][task_index] == 1)
                {
                    // call member function on the MCPS instance with the stored frame pointer
                    (*TASKS[mcp_index][task_index])(MCPS[mcp_index]);

                    // reset counter
                    task_counters[mcp_index][task_index] = TASK_TICKS[mcp_index][task_index];
                }
                else
                {
                    // decrement counter
                    --task_counters[mcp_index][task_index];
                }
            }
        }
    }
};
