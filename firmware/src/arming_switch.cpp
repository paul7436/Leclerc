#include "arming_switch.h"

ArmingSwitch::ArmingSwitch(uint32_t debounceMs)
    : debounceMs_(debounceMs), armed_(false), candidateArmed_(false), candidateSinceMs_(0) {}

bool ArmingSwitch::update(bool rawArmed, uint32_t nowMs) {
  if (!rawArmed) {
    candidateArmed_ = false;
    if (armed_) {
      armed_ = false;  // disarm without waiting
      return true;
    }
    return false;
  }

  if (!candidateArmed_) {
    candidateArmed_ = true;  // start timing a new ARMED streak
    candidateSinceMs_ = nowMs;
  }
  if (!armed_ && nowMs - candidateSinceMs_ >= debounceMs_) {
    armed_ = true;
    return true;
  }
  return false;
}
