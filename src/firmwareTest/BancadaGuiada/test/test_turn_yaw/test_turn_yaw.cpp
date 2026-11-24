#include <unity.h>
#include "continuous_yaw.hpp"
void setUp() {}
void tearDown() {}
void right_crossing() {
    ContinuousTurnYaw yaw;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, yaw.update(-90.0f, 1));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 179.0f, yaw.update(-179.0f, 1));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 181.0f, yaw.update(179.0f, 1));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 185.0f, yaw.update(175.0f, 1));
}
void left_crossing() {
    ContinuousTurnYaw yaw;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, yaw.update(90.0f, -1));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 179.0f, yaw.update(179.0f, -1));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 181.0f, yaw.update(-179.0f, -1));
}
void wrong_direction_stays_negative() {
    ContinuousTurnYaw yaw;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -10.0f, yaw.update(10.0f, 1));
}
void reset_and_reverse_correction() {
    ContinuousTurnYaw yaw;
    yaw.update(-90.0f, 1);
    yaw.reset(175.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, yaw.update(-175.0f, -1));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 8.0f, yaw.update(-177.0f, -1));
}
int main() {
    UNITY_BEGIN();
    RUN_TEST(right_crossing);
    RUN_TEST(left_crossing);
    RUN_TEST(wrong_direction_stays_negative);
    RUN_TEST(reset_and_reverse_correction);
    return UNITY_END();
}
