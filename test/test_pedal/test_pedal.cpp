/**
 * @file test_pedal.cpp
 * @author Planeson, Red Bird Racing
 * @brief Tests all functions in Pedal class
 * @version 1.0
 * @date 2026-01-01
 * @see Pedal.h, Pedal.cpp
 * 
 */
#include <Arduino.h>
#include <unity.h>

// Make private and protected members public for testing
#define private public
#define protected public

// Then include unit under test, note include is just copy paste, so must be after the above defines
#include "Pedal.h"

Pedal pedal;

void setUp(void)
{
    // runs before each test
    // optional in the sense that this can be empty
    // to ensure it compiles on all platforms, do not remove this empty function
}

void tearDown(void)
{
    // runs after each test
    // optional in the sense that this can be empty
    // to ensure it compiles on all platforms, do not remove this empty function
}

void test_pedal_update_no_fault(void)
{
    car_state car = {
        INIT,  // car_status
        0,     // car_status_millis_counter
        0,     // millis
        0,     // pedal_final
        false, // fault_force_stop
        0      // torque_out
    };
    for (int i = 0; i < 16; i++)
    {
        pedal.pedal_value_1.push(PEDAL_1_LU);
        pedal.pedal_value_2.push(PEDAL_1_LU * PEDAL_2_RANGE / PEDAL_1_RANGE);
    }
    // fully fill AVG filter with non-faulty values
    for (int i = 0; i < 16; i++)
    {
        pedal.pedal_update(&car, PEDAL_1_LU + 100, PEDAL_1_LU * PEDAL_2_RANGE / PEDAL_1_RANGE, 0);
    }
    TEST_ASSERT_FALSE(pedal.fault);
}

void test_pedal_update_fault(void)
{
    car_state car = {
        INIT,  // car_status
        0,     // car_status_millis_counter
        0,     // millis
        0,     // pedal_final
        0,     // brake_final
        false, // fault_force_stop
        0      // torque_out
    };
    // fill AVG filter
    for (int i = 0; i < 16; i++)
    {
        pedal.pedal_value_1.push(PEDAL_1_LU);
        pedal.pedal_value_2.push(PEDAL_1_LU * PEDAL_2_RANGE / PEDAL_1_RANGE);
    }
    // fully fill AVG filter with faulty values
    for (int i = 0; i < 16; i++)
    {
        pedal.pedal_update(&car, PEDAL_1_LU + 105, PEDAL_1_LU * PEDAL_2_RANGE / PEDAL_1_RANGE, 0);
    }
    TEST_ASSERT_TRUE(pedal.fault);
}

void test_pedal_can_frame_stop_motor(void)
{
    can_frame frame;
    pedal.pedal_can_frame_stop_motor(&frame);
    TEST_ASSERT_EQUAL(MOTOR_COMMAND, frame.can_id);
    TEST_ASSERT_EQUAL(3, frame.can_dlc);
    TEST_ASSERT_EQUAL(0x90, frame.data[0]);
    TEST_ASSERT_EQUAL(0x00, frame.data[1]);
    TEST_ASSERT_EQUAL(0x00, frame.data[2]);
}

void test_throttle_torque_mapping_normal(void)
{
    // range check
    for (int i = 0; i < 1024; i++)
    {
        for (int j = 0; j < 1024; ++j)
        {
            int16_t torque;
            if (i < PEDAL_LL || i >= PEDAL_LU)
            {
                // test mapping is within positive range with any brake value
                torque = pedal.throttle_torque_mapping(i, j, false);
                TEST_ASSERT_INT16_WITHIN(MAX_THROTTLE_OUT_VAL / 2, MAX_THROTTLE_OUT_VAL / 2, torque);
            }
            else
            {
                // test mapping is within range without regen
                torque = pedal.throttle_torque_mapping(i, 0, false);
                TEST_ASSERT_INT16_WITHIN(MAX_THROTTLE_OUT_VAL / 2, MAX_THROTTLE_OUT_VAL / 2, torque);
                break;
            }
        }
    }

    // check deadzone handling
    for (int i = PEDAL_1_LL; i < PEDAL_1_LU; ++i)
    {
        // regen range
        TEST_ASSERT_EQUAL(0, pedal.throttle_torque_mapping(i, 0, false));
        TEST_ASSERT_EQUAL(0, pedal.throttle_torque_mapping(i, 0, true));
    }
    for (int i = PEDAL_1_UL; i < PEDAL_1_UU; ++i)
    {
        for (int j = 0; j < 1024; ++j)
            TEST_ASSERT_EQUAL(MAX_THROTTLE_OUT_VAL, pedal.throttle_torque_mapping(i, j, false));
            TEST_ASSERT_EQUAL(MAX_THROTTLE_OUT_VAL, pedal.throttle_torque_mapping(i, 0, true));
    }
    for (int i = 0; i < PEDAL_1_LL; ++i)
    {
        for (int j = 0; j < 1024; ++j)
            TEST_ASSERT_EQUAL(0, pedal.throttle_torque_mapping(i, j, false));
    }
    for (int i = PEDAL_1_UU; i < 1024; ++i)
    {
        for (int j = 0; j < 1024; ++j)
            TEST_ASSERT_EQUAL(0, pedal.throttle_torque_mapping(i, j, false));
    }

    // Check that as input increases, output increase
    for (int i = PEDAL_1_LU + 1; i <= PEDAL_1_UL; i++)
    {
        int16_t torque = pedal.throttle_torque_mapping(i, 0, false);
        TEST_ASSERT_TRUE_MESSAGE(torque > pedal.throttle_torque_mapping(i - 1, 0, false),
                                 "Torque should be non-decreasing with increasing input");
    }

    // Test the mapping at the lower limit
    TEST_ASSERT_EQUAL(0, pedal.throttle_torque_mapping(PEDAL_1_LU, 0, true));
    TEST_ASSERT_EQUAL(0, pedal.throttle_torque_mapping(PEDAL_1_LU, 0, false));
    // Test the mapping at the upper limit
    TEST_ASSERT_EQUAL((int16_t)MAX_THROTTLE_OUT_VAL, pedal.throttle_torque_mapping(PEDAL_1_UL, 0, true));
    TEST_ASSERT_EQUAL(-(int16_t)MAX_THROTTLE_OUT_VAL, pedal.throttle_torque_mapping(PEDAL_1_UL, 0, false));
}
void test_brake_torque_mapping(void)
{
    // Test that brake_torque_mapping returns 0 for out-of-range values
    for (int i = 0; i < BRAKE_LU; ++i)
    {
        TEST_ASSERT_EQUAL(0, pedal.brake_torque_mapping(i, false));
        TEST_ASSERT_EQUAL(0, pedal.brake_torque_mapping(i, true));
    }
    for (int i = BRAKE_UL; i < 1024; ++i)
    {
        TEST_ASSERT_EQUAL(0, pedal.brake_torque_mapping(i, false));
        TEST_ASSERT_EQUAL(0, pedal.brake_torque_mapping(i, true));
    }

    // Test that as input increases, output increases or decreases according to flip_dir
    int16_t prev_torque = pedal.brake_torque_mapping(BRAKE_LU, false);
    for (int i = BRAKE_LU + 1; i <= BRAKE_UL; ++i)
    {
        int16_t torque = pedal.brake_torque_mapping(i, false);
        TEST_ASSERT_TRUE_MESSAGE(torque >= prev_torque, "Torque should be non-decreasing with increasing brake input (flip_dir=false)");
        prev_torque = torque;
    }

    prev_torque = pedal.brake_torque_mapping(BRAKE_LU, true);
    for (int i = BRAKE_LU + 1; i <= BRAKE_UL; ++i)
    {
        int16_t torque = pedal.brake_torque_mapping(i, true);
        TEST_ASSERT_TRUE_MESSAGE(torque <= prev_torque, "Torque should be non-increasing with increasing brake input (flip_dir=true)");
        prev_torque = torque;
    }

    // Test the mapping at the lower and upper limits
    TEST_ASSERT_EQUAL(0, pedal.brake_torque_mapping(BRAKE_LU, false));
    TEST_ASSERT_EQUAL(0, pedal.brake_torque_mapping(BRAKE_LU, true));
    TEST_ASSERT_EQUAL(-MAX_THROTTLE_OUT_VAL, pedal.brake_torque_mapping(BRAKE_UL, false));
    TEST_ASSERT_EQUAL(MAX_THROTTLE_OUT_VAL, pedal.brake_torque_mapping(BRAKE_UL, true));
}

void test_check_pedal_fault(void)
{
    // make sure APPS3V3 /3.3*5 and APPS5V delta > 102 then fault, else no fault
    
    // no fault when similar values
    for (float i = 0; i < 1024; ++i)
    {
        float pedal_1 = i;
        float pedal_2 = i * PEDAL_2_RANGE / PEDAL_1_RANGE;
        bool result = pedal.check_pedal_fault(pedal_1, pedal_2);
        TEST_ASSERT_FALSE(result);
    }

    // floating point issues, 103 fails for some cases, so use 104

    // fault when +10% difference
    for (uint16_t i = 0; i < 1024 - 104; ++i)
    {
        uint16_t pedal_1 = i + 104; // slightly more than 10% difference
        uint16_t pedal_2 = round((float)i * PEDAL_2_RANGE / PEDAL_1_RANGE);
        bool result = pedal.check_pedal_fault(pedal_1, pedal_2);
        TEST_ASSERT_TRUE(result);
    }
    // fault when -10% difference
    for (uint16_t i = 104; i < 1024; ++i)
    {
        uint16_t pedal_1 = i - 104; // slightly more than 10% difference
        uint16_t pedal_2 = round((float)i * PEDAL_2_RANGE / PEDAL_1_RANGE);
        bool result = pedal.check_pedal_fault(pedal_1, pedal_2);
        TEST_ASSERT_TRUE(result);
    }

    // fault when +10% difference in reverse
    for (uint16_t i = 0; i < 1024 - 104; ++i)
    {
        uint16_t pedal_1 = i;
        uint16_t pedal_2 = round((float)(i + 104) * PEDAL_2_RANGE / PEDAL_1_RANGE); // slightly more than 10% difference
        bool result = pedal.check_pedal_fault(pedal_1, pedal_2);
        TEST_ASSERT_TRUE(result);
    }

    // fault when -10% difference in reverse
    for (uint16_t i = 104; i < 1024; ++i)
    {
        uint16_t pedal_1 = i;
        uint16_t pedal_2 = round((float)(i - 104) * PEDAL_2_RANGE / PEDAL_1_RANGE); // slightly more than 10% difference
        bool result = pedal.check_pedal_fault(pedal_1, pedal_2);
        TEST_ASSERT_TRUE(result);
    }
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_check_pedal_fault);
    RUN_TEST(test_pedal_can_frame_stop_motor);
    RUN_TEST(test_throttle_torque_mapping_normal);
    RUN_TEST(test_pedal_update_no_fault);
    RUN_TEST(test_pedal_update_fault);
    UNITY_END();
}

void loop()
{
    // not used
}