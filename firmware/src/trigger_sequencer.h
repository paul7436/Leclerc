#pragma once

// Non-blocking trigger sweep: IDLE -> PULL -> HOLD -> RELEASE -> IDLE, with a
// minimum cooldown after each sweep. Hardware independent; main.cpp writes
// servoAngle() to the trigger servo. All times use the millis() clock and are
// safe across its 32-bit wrap-around.

#include <stdint.h>

enum class TriggerState : uint8_t { Idle, Pull, Hold, Release };

// Answer to a fire request, in the order the conditions are checked.
enum class FireVerdict : uint8_t { Accepted, Disarmed, LinkLost, Busy, Cooldown };

struct TriggerTiming {
  float restDeg;
  float pullDeg;
  uint32_t travelMs;   // time given to reach the pull angle
  uint32_t holdMs;     // time the trigger stays pulled
  uint32_t releaseMs;  // time given to return to rest
  uint32_t cooldownMs; // minimum time from the end of a sweep to the next one
};

class TriggerSequencer {
 public:
  explicit TriggerSequencer(const TriggerTiming& timing);

  // Starts a sweep only if armed, the link is up, no sweep is running and
  // the cooldown has elapsed.
  FireVerdict requestShot(bool armed, bool linkOk, uint32_t nowMs);

  // Cuts a sweep short: PULL or HOLD jump straight to RELEASE.
  void abort(uint32_t nowMs);

  // Advances the state machine. Call it on every loop iteration.
  void update(uint32_t nowMs);

  TriggerState state() const { return state_; }

  // Angle the trigger servo must be driven to in the current state.
  float servoAngle() const;

  // Time left before a new sweep may start (0 when ready).
  uint32_t cooldownRemainingMs(uint32_t nowMs) const;

 private:
  void enter(TriggerState state, uint32_t nowMs);
  uint32_t stateDurationMs() const;

  TriggerTiming timing_;
  TriggerState state_;
  uint32_t stateStartMs_;
  bool hasCompletedSweep_;
  uint32_t lastSweepEndMs_;
};

// Protocol names: "IDLE", "PULL", "HOLD", "RELEASE".
const char* triggerStateName(TriggerState state);

// Protocol denial reasons: "DISARMED", "LINK", "BUSY", "COOLDOWN" ("OK" when accepted).
const char* fireVerdictName(FireVerdict verdict);
