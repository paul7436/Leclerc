#include <math.h>
#include <unity.h>

#include "aim_axis.h"

void setUp() {}
void tearDown() {}

namespace {

// 20..160 degrees, home 90, 100 degrees per second.
AimAxis makeAxis() { return AimAxis(20.0f, 160.0f, 90.0f, 100.0f); }

}  // namespace

void test_clamp_angle() {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 20.0f, clampAngle(-5.0f, 20.0f, 160.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 160.0f, clampAngle(500.0f, 20.0f, 160.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 42.0f, clampAngle(42.0f, 20.0f, 160.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 20.0f, clampAngle(NAN, 20.0f, 160.0f));
}

void test_axis_starts_at_home() {
  AimAxis axis = makeAxis();
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 90.0f, axis.current());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 90.0f, axis.target());
}

void test_home_outside_limits_is_clamped() {
  AimAxis axis(20.0f, 160.0f, 175.0f, 100.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 160.0f, axis.current());
}

void test_target_is_clamped_to_limits() {
  AimAxis axis = makeAxis();
  axis.setTarget(200.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 160.0f, axis.target());
  axis.setTarget(-30.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 20.0f, axis.target());
}

void test_update_respects_max_speed() {
  AimAxis axis = makeAxis();
  axis.setTarget(120.0f);

  TEST_ASSERT_TRUE(axis.update(100));  // 100 deg/s for 100 ms = 10 degrees
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, axis.current());
  TEST_ASSERT_TRUE(axis.update(100));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 110.0f, axis.current());
}

void test_update_moves_down_as_well() {
  AimAxis axis = makeAxis();
  axis.setTarget(60.0f);
  axis.update(50);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 85.0f, axis.current());
}

void test_update_lands_exactly_on_target() {
  AimAxis axis = makeAxis();
  axis.setTarget(93.0f);
  TEST_ASSERT_TRUE(axis.update(100));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 93.0f, axis.current());
  TEST_ASSERT_FALSE(axis.update(100));  // already there
}

void test_update_without_elapsed_time_does_not_move() {
  AimAxis axis = makeAxis();
  axis.setTarget(120.0f);
  TEST_ASSERT_FALSE(axis.update(0));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 90.0f, axis.current());
}

void test_hold_current_stops_motion() {
  AimAxis axis = makeAxis();
  axis.setTarget(150.0f);
  axis.update(100);
  axis.holdCurrent();
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, axis.target());
  TEST_ASSERT_FALSE(axis.update(100));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, axis.current());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_clamp_angle);
  RUN_TEST(test_axis_starts_at_home);
  RUN_TEST(test_home_outside_limits_is_clamped);
  RUN_TEST(test_target_is_clamped_to_limits);
  RUN_TEST(test_update_respects_max_speed);
  RUN_TEST(test_update_moves_down_as_well);
  RUN_TEST(test_update_lands_exactly_on_target);
  RUN_TEST(test_update_without_elapsed_time_does_not_move);
  RUN_TEST(test_hold_current_stops_motion);
  return UNITY_END();
}
