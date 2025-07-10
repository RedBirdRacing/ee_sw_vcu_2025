#include "Pedal.h"
#include <Arduino.h>
#include <unity.h>

Pedal pedal;

void setUp(void)
{
    // Optional: runs before each test
}

void tearDown(void)
{
    // Optional: runs after each test
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
        pedal.pedal_update(&car, PEDAL_1_LU + 100, PEDAL_1_LU * PEDAL_2_RANGE / PEDAL_1_RANGE);
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
        pedal.pedal_update(&car, PEDAL_1_LU + 105, PEDAL_1_LU * PEDAL_2_RANGE / PEDAL_1_RANGE);
    }
    TEST_ASSERT_TRUE(pedal.fault);
}

void test_pedal_can_frame_stop_motor(void)
{
    can_frame frame;
    pedal.pedal_can_frame_stop_motor(&frame, "test reason");
    TEST_ASSERT_EQUAL(MOTOR_COMMAND, frame.can_id);
    TEST_ASSERT_EQUAL(3, frame.can_dlc);
    TEST_ASSERT_EQUAL(0x90, frame.data[0]);
    TEST_ASSERT_EQUAL(0x00, frame.data[1]);
    TEST_ASSERT_EQUAL(0x00, frame.data[2]);
}

void test_throttle_torque_mapping_normal(void)
{
    for (int i = 0; i < 1024; i++)
    {
        int16_t torque = pedal.throttle_torque_mapping(i, false);
        TEST_ASSERT_INT16_WITHIN(MAX_THROTTLE_OUT_VAL / 2, MAX_THROTTLE_OUT_VAL / 2, torque);
    }

    for (int i = 0; i < 2; ++i)
    {
        // Test the mapping at the lower limit
        TEST_ASSERT_EQUAL(0, pedal.throttle_torque_mapping(PEDAL_1_LU, i));
        // Test the mapping at the upper limit
        TEST_ASSERT_EQUAL((i % 2 == 0 ? 1 : -1) * (int16_t)MAX_THROTTLE_OUT_VAL, pedal.throttle_torque_mapping(PEDAL_1_UL, i));
    }
}

void test_check_pedal_fault(void)
{
    // 84 was arbitary, just make sure APPS3V3 /3.3*5 and APPS5V delta > 102 then fault, else no fault
    // no fault when similar values
    bool result = pedal.check_pedal_fault(84, 84);
    TEST_ASSERT_FALSE(result);

    // edge case: APPS5V higher by 128 -> fault
    // fine on 127
    result = pedal.check_pedal_fault(84 + 127, 84);
    TEST_ASSERT_FALSE(result);
    // but 128 is too much
    result = pedal.check_pedal_fault(84 + 128, 84);
    TEST_ASSERT_TRUE(result);

    // edge case: APPS3V higher by 60 -> fault
    // fine on 59
    result = pedal.check_pedal_fault(84, 84 + 59);
    TEST_ASSERT_FALSE(result);
    // but 60 is too much
    result = pedal.check_pedal_fault(84, 84 + 60);
    TEST_ASSERT_TRUE(result);
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