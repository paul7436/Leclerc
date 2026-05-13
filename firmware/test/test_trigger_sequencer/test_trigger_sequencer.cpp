#include <unity.h>

#include "trigger_sequencer.h"

void setUp() {}
void tearDown() {}

namespace {

const TriggerTiming kTiming = {
    20.0f,  // restDeg
    75.0f,  // pullDeg
    100,    // travelMs
    50,     // holdMs
    80,     // releaseMs
    1000,   // cooldownMs
};

constexpr uint32_t kSweepMs = 100 + 50 + 80;

// Runs a complete sweep starting at `startMs`; returns the time it ended.
uint32_t runFullSweep(TriggerSequencer& trigger, uint32_t startMs) {
  TEST_ASSERT_EQUAL(FireVerdict::Accepted, trigger.requestShot(true, true, startMs));
  trigger.update(startMs + 100);
  trigger.update(startMs + 150);
  trigger.update(startMs + kSweepMs);
  TEST_ASSERT_EQUAL(TriggerState::Idle, trigger.state());
  return startMs + kSweepMs;
}

}  // namespace

void test_starts_idle_at_rest() {
  TriggerSequencer trigger(kTiming);
  TEST_ASSERT_EQUAL(TriggerState::Idle, trigger.state());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 20.0f, trigger.servoAngle());
  TEST_ASSERT_EQUAL_UINT32(0, trigger.cooldownRemainingMs(0));
}

void test_refuses_when_disarmed_or_link_lost() {
  TriggerSequencer trigger(kTiming);
  TEST_ASSERT_EQUAL(FireVerdict::Disarmed, trigger.requestShot(false, true, 10));
  TEST_ASSERT_EQUAL(FireVerdict::Disarmed, trigger.requestShot(false, false, 10));
  TEST_ASSERT_EQUAL(FireVerdict::LinkLost, trigger.requestShot(true, false, 10));
  TEST_ASSERT_EQUAL(TriggerState::Idle, trigger.state());
}

void test_sweep_goes_through_every_state() {
  TriggerSequencer trigger(kTiming);
  TEST_ASSERT_EQUAL(FireVerdict::Accepted, trigger.requestShot(true, true, 1000));
  TEST_ASSERT_EQUAL(TriggerState::Pull, trigger.state());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 75.0f, trigger.servoAngle());

  trigger.update(1099);
  TEST_ASSERT_EQUAL(TriggerState::Pull, trigger.state());
  trigger.update(1100);
  TEST_ASSERT_EQUAL(TriggerState::Hold, trigger.state());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 75.0f, trigger.servoAngle());

  trigger.update(1150);
  TEST_ASSERT_EQUAL(TriggerState::Release, trigger.state());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 20.0f, trigger.servoAngle());

  trigger.update(1229);
  TEST_ASSERT_EQUAL(TriggerState::Release, trigger.state());
  trigger.update(1230);
  TEST_ASSERT_EQUAL(TriggerState::Idle, trigger.state());
}

void test_refuses_while_busy() {
  TriggerSequencer trigger(kTiming);
  trigger.requestShot(true, true, 0);
  TEST_ASSERT_EQUAL(FireVerdict::Busy, trigger.requestShot(true, true, 10));
  TEST_ASSERT_EQUAL_UINT32(1000, trigger.cooldownRemainingMs(10));
}

void test_cooldown_counts_from_end_of_sweep() {
  TriggerSequencer trigger(kTiming);
  const uint32_t endMs = runFullSweep(trigger, 0);

  TEST_ASSERT_EQUAL_UINT32(1000, trigger.cooldownRemainingMs(endMs));
  TEST_ASSERT_EQUAL_UINT32(1, trigger.cooldownRemainingMs(endMs + 999));
  TEST_ASSERT_EQUAL(FireVerdict::Cooldown, trigger.requestShot(true, true, endMs + 999));
  TEST_ASSERT_EQUAL_UINT32(0, trigger.cooldownRemainingMs(endMs + 1000));
  TEST_ASSERT_EQUAL(FireVerdict::Accepted, trigger.requestShot(true, true, endMs + 1000));
}

void test_abort_releases_and_still_enforces_cooldown() {
  TriggerSequencer trigger(kTiming);
  trigger.requestShot(true, true, 0);
  trigger.update(100);  // HOLD
  trigger.abort(120);
  TEST_ASSERT_EQUAL(TriggerState::Release, trigger.state());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 20.0f, trigger.servoAngle());

  trigger.update(200);
  TEST_ASSERT_EQUAL(TriggerState::Idle, trigger.state());
  TEST_ASSERT_EQUAL(FireVerdict::Cooldown, trigger.requestShot(true, true, 300));
}

void test_abort_is_a_no_op_when_idle_or_releasing() {
  TriggerSequencer trigger(kTiming);
  trigger.abort(0);
  TEST_ASSERT_EQUAL(TriggerState::Idle, trigger.state());

  trigger.requestShot(true, true, 0);
  trigger.update(100);
  trigger.update(150);  // RELEASE started at 150
  trigger.abort(160);
  trigger.update(229);
  TEST_ASSERT_EQUAL(TriggerState::Release, trigger.state());
  trigger.update(230);
  TEST_ASSERT_EQUAL(TriggerState::Idle, trigger.state());
}

void test_timing_survives_millis_wrap_around() {
  TriggerSequencer trigger(kTiming);
  const uint32_t startMs = 0xFFFFFFFFu - 120u;
  const uint32_t endMs = runFullSweep(trigger, startMs);  // wraps past zero

  TEST_ASSERT_EQUAL_UINT32(500, trigger.cooldownRemainingMs(endMs + 500));
  TEST_ASSERT_EQUAL(FireVerdict::Accepted, trigger.requestShot(true, true, endMs + 1000));
}

void test_names_match_protocol() {
  TEST_ASSERT_EQUAL_STRING("PULL", triggerStateName(TriggerState::Pull));
  TEST_ASSERT_EQUAL_STRING("RELEASE", triggerStateName(TriggerState::Release));
  TEST_ASSERT_EQUAL_STRING("DISARMED", fireVerdictName(FireVerdict::Disarmed));
  TEST_ASSERT_EQUAL_STRING("LINK", fireVerdictName(FireVerdict::LinkLost));
  TEST_ASSERT_EQUAL_STRING("BUSY", fireVerdictName(FireVerdict::Busy));
  TEST_ASSERT_EQUAL_STRING("COOLDOWN", fireVerdictName(FireVerdict::Cooldown));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_starts_idle_at_rest);
  RUN_TEST(test_refuses_when_disarmed_or_link_lost);
  RUN_TEST(test_sweep_goes_through_every_state);
  RUN_TEST(test_refuses_while_busy);
  RUN_TEST(test_cooldown_counts_from_end_of_sweep);
  RUN_TEST(test_abort_releases_and_still_enforces_cooldown);
  RUN_TEST(test_abort_is_a_no_op_when_idle_or_releasing);
  RUN_TEST(test_timing_survives_millis_wrap_around);
  RUN_TEST(test_names_match_protocol);
  return UNITY_END();
}
