#include "mcp2515.h"   // can_frame, TXBn, mcp2515 objects
#include "miniArray.h" // miniArray

// template because we can't declare the size of the arrays without macros here
template <uint8_t NUM_TASKS, uint8_t NUM_MCP2515>
// assume we only schedule CAN bus sends, so all tasks are void func(*tx_msg);
class Scheduler
{
public:
    using TaskFn = void (*)();

    // delete default constructor, i.e. all arguments must be provided
    Scheduler() = delete;

    // constructor
    Scheduler(uint32_t period_us_,
              uint32_t spin_threshold_us_,
              miniArray<TaskFn, NUM_TASKS> tasks_,
              miniArray<uint8_t, NUM_TASKS> task_ticks_)
        : TASKS(tasks_),
          TASK_TICKS(task_ticks_),
          PERIOD_US(period_us_),
          SPIN_US(spin_threshold_us_),
          last_fire_us(0),
          task_counters{0} // run on first tick
    {
    };

    // no need destructor, since no dynamic memory allocation, and won't destruct in the middle of the program anyway

    // main updater
    void update(uint32_t (*current_time_us)())
    {
        // check if it's time to fire
        uint32_t delta = current_time_us() - last_fire_us;
        if (delta > PERIOD_US)
        {
            // it's time to fire
            run_tasks(&delta);
        }
        else
        {
            // not time yet, check if we should spin-wait or return
            if (delta > PERIOD_US - SPIN_US)
            {
                // spin-wait
                while ((uint32_t)(current_time_us() - last_fire_us) < PERIOD_US)
                    ;
                // now it's time, run the tasks
                run_tasks(&delta);
            }
            else
            {
                return; // return to let other non-scheduler tasks run
            }
        }
    }

    // array of function pointers to tasks
    const miniArray<TaskFn, NUM_TASKS> TASKS;

    // stores the number of ticks between two runs of the task, - 1.
    // for instance, if a task runs every 10 ticks, store 9
    // if a task runs every tick, store 0
    const miniArray<uint8_t, NUM_TASKS> TASK_TICKS;

private:
    // how the period of the scheduler, i.e. time between two fires of the scheduler, in microseconds
    // e.g. 10000 for 10ms cycle time
    // you should tune this value to make sure it is a multiple of your task periods
    // but you should also slightly reduce it to account for the slight delays in the microcontroller
    // e.g. if you want 10ms cycle time, set to something like 9990
    const uint32_t PERIOD_US;

    // threshold to switch from letting non-scheduler task in loop() run, to busy-waiting (to ensure on time firing)
    // e.g. 100 for a 0.1ms
    const uint32_t SPIN_US;

    // last time the scheduler was fired
    // overriden when if we miss more than one period, to avoid bursts
    uint32_t last_fire_us;

    // counter of the functions, how many times the scheduler fires for that function to run
    // for instance, if BMS tasks every 10 ticks, then initially store 9, and decrement every tick, when it reaches 0, run the task and reset to 9
    uint8_t task_counters[NUM_TASKS];

    // miniArray of mcp2515 objects
    const miniArray<MCP2515, NUM_MCP2515> MCPS;

    // helper function to run tasks
    inline void run_tasks(uint32_t *delta, uint32_t (*current_time_us)())
    {
        for (uint8_t i = 0; i < NUM_TASKS; ++i)
        {
            if (task_counters[i] == 0)
            {
                // run the task, no safety to check for nullptr, don't write shit code thx
                TASKS[i]();
                // reset counter
                task_counters[i] = TASK_TICKS[i];
            }
            else
            {
                // decrement counter
                --task_counters[i];
            }
        }

        // finally, set next target fire time
        if (*delta >= PERIOD_US << 1) // if we missed more than one period, just set to now + period
        {
            last_fire_us = current_time_us();
            // this will mean the next cycle is slightly more than 1 period away
        }
        else // else just advance by one period to keep phase
        {
            last_fire_us += PERIOD_US;
        }
    }
};
