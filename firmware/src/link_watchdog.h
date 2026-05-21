#pragma once

// Serial link watchdog. The link starts DOWN and comes up with the first A or
// S command. It goes DOWN when no valid command arrives for the timeout, and
// only an A or S command brings it back: a fire command alone never does.

#include <stdint.h>

class LinkWatchdog {
 public:
  enum class Change : uint8_t { None, Lost, Restored };

  explicit LinkWatchdog(uint32_t timeoutMs);

  // Records a valid command. `restoresLink` is true for A and S, false for F.
  Change onCommand(bool restoresLink, uint32_t nowMs);

  // Checks the timeout. Returns Lost once, when the link times out.
  Change update(uint32_t nowMs);

  bool linkOk() const { return linkOk_; }

 private:
  uint32_t timeoutMs_;
  uint32_t lastCommandMs_;
  bool linkOk_;
};
