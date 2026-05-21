#include <unity.h>

#include "link_watchdog.h"

void setUp() {}
void tearDown() {}

namespace {

typedef LinkWatchdog::Change Change;

}  // namespace

void test_link_starts_down_and_never_reports_lost_at_boot() {
  LinkWatchdog watchdog(500);
  TEST_ASSERT_FALSE(watchdog.linkOk());
  TEST_ASSERT_EQUAL(Change::None, watchdog.update(10000));
}

void test_aim_or_status_brings_link_up() {
  LinkWatchdog watchdog(500);
  TEST_ASSERT_EQUAL(Change::Restored, watchdog.onCommand(true, 100));
  TEST_ASSERT_TRUE(watchdog.linkOk());
  TEST_ASSERT_EQUAL(Change::None, watchdog.onCommand(true, 200));
}

void test_fire_alone_does_not_bring_link_up() {
  LinkWatchdog watchdog(500);
  TEST_ASSERT_EQUAL(Change::None, watchdog.onCommand(false, 100));
  TEST_ASSERT_FALSE(watchdog.linkOk());
}

void test_link_times_out_once() {
  LinkWatchdog watchdog(500);
  watchdog.onCommand(true, 1000);
  TEST_ASSERT_EQUAL(Change::None, watchdog.update(1499));
  TEST_ASSERT_TRUE(watchdog.linkOk());
  TEST_ASSERT_EQUAL(Change::Lost, watchdog.update(1500));
  TEST_ASSERT_FALSE(watchdog.linkOk());
  TEST_ASSERT_EQUAL(Change::None, watchdog.update(3000));
}

void test_every_command_feeds_the_timer() {
  LinkWatchdog watchdog(500);
  watchdog.onCommand(true, 0);
  watchdog.onCommand(false, 400);  // F keeps a healthy link alive
  TEST_ASSERT_EQUAL(Change::None, watchdog.update(800));
  TEST_ASSERT_EQUAL(Change::Lost, watchdog.update(900));
}

void test_fire_after_timeout_does_not_restore() {
  LinkWatchdog watchdog(500);
  watchdog.onCommand(true, 0);
  watchdog.update(600);
  TEST_ASSERT_EQUAL(Change::None, watchdog.onCommand(false, 650));
  TEST_ASSERT_FALSE(watchdog.linkOk());
  TEST_ASSERT_EQUAL(Change::Restored, watchdog.onCommand(true, 700));
}

void test_timeout_survives_millis_wrap_around() {
  LinkWatchdog watchdog(500);
  watchdog.onCommand(true, 0xFFFFFFFFu - 100u);
  TEST_ASSERT_EQUAL(Change::None, watchdog.update(398));
  TEST_ASSERT_EQUAL(Change::Lost, watchdog.update(399));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_link_starts_down_and_never_reports_lost_at_boot);
  RUN_TEST(test_aim_or_status_brings_link_up);
  RUN_TEST(test_fire_alone_does_not_bring_link_up);
  RUN_TEST(test_link_times_out_once);
  RUN_TEST(test_every_command_feeds_the_timer);
  RUN_TEST(test_fire_after_timeout_does_not_restore);
  RUN_TEST(test_timeout_survives_millis_wrap_around);
  return UNITY_END();
}
