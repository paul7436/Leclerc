#include "aim_axis.h"

float clampAngle(float value, float low, float high) {
  if (!(value >= low)) {
    return low;  // also catches NaN
  }
  if (value > high) {
    return high;
  }
  return value;
}

AimAxis::AimAxis(float minDeg, float maxDeg, float homeDeg, float maxSpeedDegPerS)
    : minDeg_(minDeg),
      maxDeg_(maxDeg),
      maxSpeedDegPerS_(maxSpeedDegPerS),
      targetDeg_(clampAngle(homeDeg, minDeg, maxDeg)),
      currentDeg_(targetDeg_) {}

void AimAxis::setTarget(float deg) { targetDeg_ = clampAngle(deg, minDeg_, maxDeg_); }

void AimAxis::holdCurrent() { targetDeg_ = currentDeg_; }

bool AimAxis::update(uint32_t elapsedMs) {
  const float previousDeg = currentDeg_;
  const float maxStep = maxSpeedDegPerS_ * static_cast<float>(elapsedMs) / 1000.0f;
  const float remaining = targetDeg_ - currentDeg_;

  if (remaining > maxStep) {
    currentDeg_ += maxStep;
  } else if (remaining < -maxStep) {
    currentDeg_ -= maxStep;
  } else {
    currentDeg_ = targetDeg_;
  }
  return currentDeg_ != previousDeg;
}
