#pragma once

// One pan or tilt axis: hard angle limits plus a maximum slew speed.
// Hardware independent; main.cpp writes current() to the servo.

#include <stdint.h>

// Limits `value` to [low, high]. A NaN input returns `low`.
float clampAngle(float value, float low, float high);

class AimAxis {
 public:
  AimAxis(float minDeg, float maxDeg, float homeDeg, float maxSpeedDegPerS);

  // Sets the angle to move toward, clamped to the axis limits.
  void setTarget(float deg);

  // Stops where the axis currently is (used when the link is lost).
  void holdCurrent();

  // Advances the current angle toward the target by at most the distance
  // allowed by the maximum speed over `elapsedMs`. Returns true if it moved.
  bool update(uint32_t elapsedMs);

  float target() const { return targetDeg_; }
  float current() const { return currentDeg_; }

 private:
  float minDeg_;
  float maxDeg_;
  float maxSpeedDegPerS_;
  float targetDeg_;
  float currentDeg_;
};
