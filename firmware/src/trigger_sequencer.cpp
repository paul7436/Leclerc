#include "trigger_sequencer.h"

TriggerSequencer::TriggerSequencer(const TriggerTiming& timing)
    : timing_(timing),
      state_(TriggerState::Idle),
      stateStartMs_(0),
      hasCompletedSweep_(false),
      lastSweepEndMs_(0) {}

FireVerdict TriggerSequencer::requestShot(bool armed, bool linkOk, uint32_t nowMs) {
  if (!armed) {
    return FireVerdict::Disarmed;
  }
  if (!linkOk) {
    return FireVerdict::LinkLost;
  }
  if (state_ != TriggerState::Idle) {
    return FireVerdict::Busy;
  }
  if (cooldownRemainingMs(nowMs) > 0) {
    return FireVerdict::Cooldown;
  }
  enter(TriggerState::Pull, nowMs);
  return FireVerdict::Accepted;
}

void TriggerSequencer::abort(uint32_t nowMs) {
  if (state_ == TriggerState::Pull || state_ == TriggerState::Hold) {
    enter(TriggerState::Release, nowMs);
  }
}

void TriggerSequencer::update(uint32_t nowMs) {
  if (state_ == TriggerState::Idle || nowMs - stateStartMs_ < stateDurationMs()) {
    return;
  }

  switch (state_) {
    case TriggerState::Pull:
      enter(TriggerState::Hold, nowMs);
      break;
    case TriggerState::Hold:
      enter(TriggerState::Release, nowMs);
      break;
    case TriggerState::Release:
      hasCompletedSweep_ = true;
      lastSweepEndMs_ = nowMs;
      enter(TriggerState::Idle, nowMs);
      break;
    case TriggerState::Idle:
      break;
  }
}

float TriggerSequencer::servoAngle() const {
  const bool pulled = (state_ == TriggerState::Pull || state_ == TriggerState::Hold);
  return pulled ? timing_.pullDeg : timing_.restDeg;
}

uint32_t TriggerSequencer::cooldownRemainingMs(uint32_t nowMs) const {
  if (state_ != TriggerState::Idle) {
    return timing_.cooldownMs;  // the cooldown starts when the sweep ends
  }
  if (!hasCompletedSweep_) {
    return 0;
  }
  const uint32_t sinceEnd = nowMs - lastSweepEndMs_;
  return sinceEnd >= timing_.cooldownMs ? 0 : timing_.cooldownMs - sinceEnd;
}

void TriggerSequencer::enter(TriggerState state, uint32_t nowMs) {
  state_ = state;
  stateStartMs_ = nowMs;
}

uint32_t TriggerSequencer::stateDurationMs() const {
  switch (state_) {
    case TriggerState::Pull:
      return timing_.travelMs;
    case TriggerState::Hold:
      return timing_.holdMs;
    case TriggerState::Release:
      return timing_.releaseMs;
    case TriggerState::Idle:
      break;
  }
  return 0;
}

const char* triggerStateName(TriggerState state) {
  switch (state) {
    case TriggerState::Idle:
      return "IDLE";
    case TriggerState::Pull:
      return "PULL";
    case TriggerState::Hold:
      return "HOLD";
    case TriggerState::Release:
      return "RELEASE";
  }
  return "IDLE";
}

const char* fireVerdictName(FireVerdict verdict) {
  switch (verdict) {
    case FireVerdict::Accepted:
      return "OK";
    case FireVerdict::Disarmed:
      return "DISARMED";
    case FireVerdict::LinkLost:
      return "LINK";
    case FireVerdict::Busy:
      return "BUSY";
    case FireVerdict::Cooldown:
      return "COOLDOWN";
  }
  return "DISARMED";
}
