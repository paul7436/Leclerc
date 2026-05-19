#include <unity.h>

#include "arming_switch.h"

void setUp() {}
void tearDown() {}

void test_starts_disarmed() {
  ArmingSwitch sw(30);
  TEST_ASSERT_FALSE(sw.armed());
  TEST_ASSERT_FALSE(sw.update(false, 0));
  TEST_ASSERT_FALSE(sw.armed());
}

void test_arms_after_stable_debounce_time() {
  ArmingSwitch sw(30);
  TEST_ASSERT_FALSE(sw.update(true, 100));
  TEST_ASSERT_FALSE(sw.update(true, 129));
  TEST_ASSERT_FALSE(sw.armed());
  TEST_ASSERT_TRUE(sw.update(true, 130));
  TEST_ASSERT_TRUE(sw.armed());
  TEST_ASSERT_FALSE(sw.update(true, 200));  // no repeated change report
}

void test_bounce_restarts_arming_delay() {
  ArmingSwitch sw(30);
  sw.update(true, 0);
  sw.update(false, 20);
  sw.update(true, 25);
  TEST_ASSERT_FALSE(sw.update(true, 50));
  TEST_ASSERT_FALSE(sw.armed());
  TEST_ASSERT_TRUE(sw.update(true, 55));
  TEST_ASSERT_TRUE(sw.armed());
}

void test_disarms_on_first_disarmed_reading() {
  ArmingSwitch sw(30);
  sw.update(true, 0);
  sw.update(true, 30);
  TEST_ASSERT_TRUE(sw.armed());
  TEST_ASSERT_TRUE(sw.update(false, 31));
  TEST_ASSERT_FALSE(sw.armed());
  TEST_ASSERT_FALSE(sw.update(false, 32));
}

void test_zero_debounce_arms_immediately() {
  ArmingSwitch sw(0);
  TEST_ASSERT_TRUE(sw.update(true, 5));
  TEST_ASSERT_TRUE(sw.armed());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_starts_disarmed);
  RUN_TEST(test_arms_after_stable_debounce_time);
  RUN_TEST(test_bounce_restarts_arming_delay);
  RUN_TEST(test_disarms_on_first_disarmed_reading);
  RUN_TEST(test_zero_debounce_arms_immediately);
  return UNITY_END();
}
