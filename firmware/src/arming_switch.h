#pragma once

// Debounced view of the physical arming switch. Hardware independent: main.cpp
// reads the sense pin and feeds the raw level in.
//
// Asymmetric on purpose: arming requires the switch to read ARMED without
// interruption for the debounce time, while a single DISARMED reading disarms
// immediately. Starts DISARMED.

#include <stdint.h>

class ArmingSwitch {
 public:
  explicit ArmingSwitch(uint32_t debounceMs);

  // Feeds one raw reading. Returns true when armed() changed.
  bool update(bool rawArmed, uint32_t nowMs);

  bool armed() const { return armed_; }

 private:
  uint32_t debounceMs_;
  bool armed_;
  bool candidateArmed_;
  uint32_t candidateSinceMs_;
};
