// Turret firmware entry point. This is the only file that touches the Arduino
// and ESP32Servo APIs; every decision is delegated to the hardware-independent
// modules, which are unit tested on the development machine.
//
// The loop never blocks: each service function looks at millis() and returns.

#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

#include "aim_axis.h"
#include "arming_switch.h"
#include "command_parser.h"
#include "config.h"
#include "link_watchdog.h"
#include "trigger_sequencer.h"

namespace {

const TriggerTiming kTriggerTiming = {
    config::kTriggerRestDeg,   config::kTriggerPullDeg,   config::kTriggerTravelMs,
    config::kTriggerHoldMs,    config::kTriggerReleaseMs, config::kShotCooldownMs,
};

Servo panServo;
Servo tiltServo;
Servo triggerServo;

AimAxis panAxis(config::kPanMinDeg, config::kPanMaxDeg, config::kPanHomeDeg,
                config::kAimMaxSpeedDegPerS);
AimAxis tiltAxis(config::kTiltMinDeg, config::kTiltMaxDeg, config::kTiltHomeDeg,
                 config::kAimMaxSpeedDegPerS);
TriggerSequencer trigger(kTriggerTiming);
ArmingSwitch armingSwitch(config::kArmDebounceMs);
LinkWatchdog linkWatchdog(config::kLinkTimeoutMs);
LineAssembler lineAssembler;

uint32_t lastAimUpdateMs = 0;
TriggerState lastTriggerState = TriggerState::Idle;

// ---------------------------------------------------------------------------
// Servo output
// ---------------------------------------------------------------------------

int angleToPulseUs(float deg) {
  const float span = static_cast<float>(config::kServoMaxPulseUs - config::kServoMinPulseUs);
  const float fraction = clampAngle(deg, 0.0f, 180.0f) / 180.0f;
  return config::kServoMinPulseUs + static_cast<int>(lroundf(fraction * span));
}

void writeServoAngle(Servo& servo, float deg) { servo.writeMicroseconds(angleToPulseUs(deg)); }

void attachServo(Servo& servo, int pin, float initialDeg) {
  servo.setPeriodHertz(config::kServoPwmHz);
  servo.attach(pin, config::kServoMinPulseUs, config::kServoMaxPulseUs);
  writeServoAngle(servo, initialDeg);
}

void setArmedLed(bool armed) {
  if (config::kArmedLedPin >= 0) {
    digitalWrite(static_cast<uint8_t>(config::kArmedLedPin), armed ? HIGH : LOW);
  }
}

// ---------------------------------------------------------------------------
// Replies to the host (protocol in docs/protocol.md)
// ---------------------------------------------------------------------------

void sendStatus(uint32_t nowMs) {
  Serial.printf("STATUS armed=%d link=%d pan=%.2f tilt=%.2f trigger=%s cooldown_ms=%lu\n",
                armingSwitch.armed() ? 1 : 0, linkWatchdog.linkOk() ? 1 : 0,
                static_cast<double>(panAxis.current()), static_cast<double>(tiltAxis.current()),
                triggerStateName(trigger.state()),
                static_cast<unsigned long>(trigger.cooldownRemainingMs(nowMs)));
}

void sendFireReply(FireVerdict verdict) {
  if (verdict == FireVerdict::Accepted) {
    Serial.print("FIRE OK\n");
  } else {
    Serial.printf("FIRE DENIED %s\n", fireVerdictName(verdict));
  }
}

// ---------------------------------------------------------------------------
// Command handling
// ---------------------------------------------------------------------------

void handleCommand(const Command& command, uint32_t nowMs) {
  const bool restoresLink = (command.type != CommandType::Fire);
  if (linkWatchdog.onCommand(restoresLink, nowMs) == LinkWatchdog::Change::Restored) {
    Serial.print("EVT LINK 1\n");
  }

  switch (command.type) {
    case CommandType::Aim:
      panAxis.setTarget(command.panDeg);
      tiltAxis.setTarget(command.tiltDeg);
      break;
    case CommandType::Fire:
      sendFireReply(trigger.requestShot(armingSwitch.armed(), linkWatchdog.linkOk(), nowMs));
      break;
    case CommandType::Status:
      sendStatus(nowMs);
      break;
  }
}

void handleLine(const char* line, uint32_t nowMs) {
  Command command;
  const ParseResult result = parseCommand(line, command);
  if (result == ParseResult::Empty) {
    return;
  }
  if (result != ParseResult::Ok) {
    Serial.printf("ERR %s\n", parseResultCode(result));
    return;
  }
  handleCommand(command, nowMs);
}

void serviceSerial(uint32_t nowMs) {
  while (Serial.available() > 0) {
    const char byte = static_cast<char>(Serial.read());
    switch (lineAssembler.push(byte)) {
      case LineAssembler::Event::Line:
        handleLine(lineAssembler.line(), nowMs);
        break;
      case LineAssembler::Event::Overflow:
        Serial.print("ERR OVERFLOW\n");
        break;
      case LineAssembler::Event::None:
        break;
    }
  }
}

// ---------------------------------------------------------------------------
// Periodic services
// ---------------------------------------------------------------------------

void serviceArmingSwitch(uint32_t nowMs) {
  const bool rawArmed = (digitalRead(config::kArmSensePin) == LOW);
  if (!armingSwitch.update(rawArmed, nowMs)) {
    return;
  }
  const bool armed = armingSwitch.armed();
  if (!armed) {
    trigger.abort(nowMs);  // never leave the trigger pulled without power
  }
  setArmedLed(armed);
  Serial.printf("EVT ARM %d\n", armed ? 1 : 0);
}

void serviceLinkWatchdog(uint32_t nowMs) {
  if (linkWatchdog.update(nowMs) != LinkWatchdog::Change::Lost) {
    return;
  }
  panAxis.holdCurrent();
  tiltAxis.holdCurrent();
  trigger.abort(nowMs);
  Serial.print("EVT LINK 0\n");
}

void serviceTrigger(uint32_t nowMs) {
  trigger.update(nowMs);
  if (trigger.state() != lastTriggerState) {
    lastTriggerState = trigger.state();
    writeServoAngle(triggerServo, trigger.servoAngle());
  }
}

void serviceAim(uint32_t nowMs) {
  const uint32_t elapsedMs = nowMs - lastAimUpdateMs;
  if (elapsedMs < config::kAimUpdatePeriodMs) {
    return;
  }
  lastAimUpdateMs = nowMs;
  if (panAxis.update(elapsedMs)) {
    writeServoAngle(panServo, panAxis.current());
  }
  if (tiltAxis.update(elapsedMs)) {
    writeServoAngle(tiltServo, tiltAxis.current());
  }
}

}  // namespace

void setup() {
  pinMode(config::kArmSensePin, INPUT_PULLUP);
  if (config::kArmedLedPin >= 0) {
    pinMode(static_cast<uint8_t>(config::kArmedLedPin), OUTPUT);
  }
  setArmedLed(false);

  // The trigger is attached first and starts at rest. Pan and tilt start at
  // their home angles.
  attachServo(triggerServo, config::kTriggerServoPin, trigger.servoAngle());
  attachServo(panServo, config::kPanServoPin, panAxis.current());
  attachServo(tiltServo, config::kTiltServoPin, tiltAxis.current());

  Serial.begin(config::kSerialBaud);
  lastAimUpdateMs = millis();
  Serial.printf("READY %s\n", config::kFirmwareVersion);
}

void loop() {
  const uint32_t nowMs = millis();
  // Safety inputs are refreshed before any command is handled.
  serviceArmingSwitch(nowMs);
  serviceLinkWatchdog(nowMs);
  serviceSerial(nowMs);
  serviceTrigger(nowMs);
  serviceAim(nowMs);
}
