#pragma once

// Every pin, angle limit and timing used by the firmware lives in this file.
// Angles are servo angles in degrees (0 to 180). Times are milliseconds.

#include <stdint.h>

namespace config {

// Reported in the READY banner at boot.
constexpr const char* kFirmwareVersion = "1.0.0";

// ---------------------------------------------------------------------------
// Serial link
// ---------------------------------------------------------------------------
constexpr uint32_t kSerialBaud = 115200;

// Without a valid command for this long, the firmware holds position and
// refuses to fire until an A or S command re-establishes the link.
constexpr uint32_t kLinkTimeoutMs = 500;

// Longest accepted command line, newline excluded. Longer lines are dropped.
constexpr uint8_t kMaxLineLength = 48;

// ---------------------------------------------------------------------------
// Pins (ESP32-S3)
// ---------------------------------------------------------------------------
constexpr int kPanServoPin = 4;
constexpr int kTiltServoPin = 5;
constexpr int kTriggerServoPin = 6;

// Arming switch pole 2 pulls this pin to GND when the switch is on. The
// internal pull-up makes an open or broken wire read as DISARMED.
constexpr int kArmSensePin = 7;

// Optional indicator LED, lit while armed. Set to -1 when not fitted.
constexpr int kArmedLedPin = 15;

// ---------------------------------------------------------------------------
// Servo PWM
// ---------------------------------------------------------------------------
constexpr int kServoPwmHz = 50;
constexpr int kServoMinPulseUs = 500;   // pulse width at 0 degrees
constexpr int kServoMaxPulseUs = 2400;  // pulse width at 180 degrees

// ---------------------------------------------------------------------------
// Pan and tilt limits
// Restrict these to the area where targets are placed, for example so the
// barrel can never point above the target zone or toward a door.
// ---------------------------------------------------------------------------
constexpr float kPanMinDeg = 20.0f;
constexpr float kPanMaxDeg = 160.0f;
constexpr float kPanHomeDeg = 90.0f;

constexpr float kTiltMinDeg = 60.0f;
constexpr float kTiltMaxDeg = 115.0f;
constexpr float kTiltHomeDeg = 90.0f;

// Pan and tilt move toward their target at this maximum speed, updated every
// kAimUpdatePeriodMs, which protects the mechanics from violent jumps.
constexpr float kAimMaxSpeedDegPerS = 240.0f;
constexpr uint32_t kAimUpdatePeriodMs = 10;

// ---------------------------------------------------------------------------
// Trigger sweep: rest -> pull -> hold -> release -> cooldown
// ---------------------------------------------------------------------------
constexpr float kTriggerRestDeg = 20.0f;
constexpr float kTriggerPullDeg = 75.0f;
constexpr uint32_t kTriggerTravelMs = 180;   // time allowed to reach the pull angle
constexpr uint32_t kTriggerHoldMs = 120;     // time the trigger stays pulled
constexpr uint32_t kTriggerReleaseMs = 200;  // time allowed to return to rest

// Minimum time between the end of one sweep and the start of the next.
constexpr uint32_t kShotCooldownMs = 1500;

// ---------------------------------------------------------------------------
// Arming switch
// ---------------------------------------------------------------------------
// The switch must read ARMED continuously for this long before the firmware
// arms. Any DISARMED reading disarms immediately.
constexpr uint32_t kArmDebounceMs = 30;

// ---------------------------------------------------------------------------
// Compile-time sanity checks
// ---------------------------------------------------------------------------
static_assert(0.0f <= kPanMinDeg && kPanMinDeg < kPanMaxDeg && kPanMaxDeg <= 180.0f,
              "pan limits must satisfy 0 <= min < max <= 180");
static_assert(kPanMinDeg <= kPanHomeDeg && kPanHomeDeg <= kPanMaxDeg,
              "pan home must lie within the pan limits");
static_assert(0.0f <= kTiltMinDeg && kTiltMinDeg < kTiltMaxDeg && kTiltMaxDeg <= 180.0f,
              "tilt limits must satisfy 0 <= min < max <= 180");
static_assert(kTiltMinDeg <= kTiltHomeDeg && kTiltHomeDeg <= kTiltMaxDeg,
              "tilt home must lie within the tilt limits");
static_assert(0.0f <= kTriggerRestDeg && kTriggerRestDeg <= 180.0f &&
                  0.0f <= kTriggerPullDeg && kTriggerPullDeg <= 180.0f,
              "trigger angles must lie within 0 to 180 degrees");
static_assert(kServoMinPulseUs < kServoMaxPulseUs, "servo pulse range is inverted");
static_assert(kShotCooldownMs >= 500, "keep at least 500 ms between shots");
static_assert(kLinkTimeoutMs >= 100 && kLinkTimeoutMs <= 2000,
              "link timeout must stay between 100 and 2000 ms");
static_assert(kMaxLineLength >= 16, "line buffer too short for an A command");

}  // namespace config
