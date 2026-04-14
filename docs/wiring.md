# Wiring notes

This page describes how the ESP32-S3, the three servos, the arming switch and
the power supply are connected. The pin numbers match the defaults in
`firmware/include/config.h`; change both together.

## Overview

```text
                       +---------------------------+
   Host USB  =========>| ESP32-S3 dev board        |
   (power + serial)    |                           |
                       | GPIO4  ---------------------------> Pan servo signal
                       | GPIO5  ---------------------------> Tilt servo signal
                       | GPIO6  ---------------------------> Trigger servo signal
                       | GPIO7  <---- arming switch pole 2 (to GND)
                       | GPIO15 ---[330R]---|>|--- GND   (optional armed LED)
                       | GND    -----------+
                       +---------------------------+
                                           |
                                           | common ground
                                           |
   Servo PSU 5-6 V ---[fuse]---[E-stop]---+-------------> Pan servo V+
                                          +-------------> Tilt servo V+
                                          +--[SW pole 1]-> Trigger servo V+
   Servo PSU GND -------------------------+-------------> all servo GND
                                          |
                                        [1000 uF]  bulk capacitor across the rail
```

## Power

- Power the servos from a dedicated 5 to 6 V supply able to deliver at least
  5 A (an MG996R alone can draw over 2 A at stall). A UBEC or a bench supply
  works; a phone charger does not.
- Never power servos from the ESP32 `3V3` or `5V` pins. The ESP32 is powered
  by the host USB cable.
- Tie the servo supply ground to the ESP32 ground. Without a common ground the
  PWM signals have no reference and the servos jitter or ignore commands.
- Put a 1000 uF (10 V or more) electrolytic capacitor across the servo rail,
  close to the servos, to absorb current spikes.
- Recommended: an inline fuse and a latching emergency stop button in series
  with the whole servo supply, within reach of the operator.

## Servo signals

- ESP32-S3 GPIOs output 3.3 V. Most hobby servos, including the MG996R and
  MG90S, accept a 3.3 V signal. If one does not, add a 74AHCT125 buffer
  powered from 5 V.
- A 220 to 470 ohm series resistor on each signal line is optional but limits
  current if a servo is miswired.
- The firmware drives 50 Hz pulses between `kServoMinPulseUs` and
  `kServoMaxPulseUs` (500 to 2400 us by default). Narrow this range if a servo
  buzzes against its mechanical end stop.

GPIOs 4, 5, 6, 7 and 15 are general purpose on the ESP32-S3 and avoid the
strapping pins (0, 3, 45, 46), the native USB pins (19, 20) and the pins used
by octal PSRAM (35 to 37).

## Arming switch

The arming switch is a **double-pole, single-throw** (DPST) switch, ideally a
key switch, rated for the trigger servo current.

| Pole | Wiring | Purpose |
| --- | --- | --- |
| Pole 1 | In series with the trigger servo positive supply only | Physically removes power from the trigger servo when off |
| Pole 2 | Between GPIO7 and GND | Tells the firmware the switch is on (pin pulled LOW) |

GPIO7 uses the internal pull-up, so:

- switch off: pole 2 open, GPIO7 reads HIGH, the firmware reports DISARMED;
- switch on: pole 2 closed, GPIO7 reads LOW, the firmware reports ARMED;
- broken sense wire: GPIO7 floats HIGH through the pull-up and reads
  DISARMED, so this failure is safe.

The two poles are independent interlocks. If pole 2 fails closed while the
switch is off, the firmware may think it is armed but the trigger servo still
has no power. If pole 1 fails closed while the switch is off, the firmware
still reads DISARMED and refuses every fire command.

The pan and tilt servos stay powered when the switch is off, so you can aim
and calibrate safely.

## Armed indicator LED (optional)

GPIO15, through a 330 ohm resistor, to an LED to ground. The firmware lights it
when it reads the switch as armed. Set `kArmedLedPin` to `-1` in `config.h` if
you do not fit one.

## Camera and mechanics

- Mount the webcam rigidly on the tilt bracket, directly below the barrel and
  in the same vertical plane, so it moves with the barrel. The host tracking
  and the deg/pixel calibration both assume the image moves when the servos
  move.
- Keep the camera optical axis parallel to the barrel. The crosshair
  calibration absorbs a small fixed offset; the holdover table absorbs dart
  drop with distance.
- The tilt axis carries the blaster, so use a high-torque servo there and keep
  the center of mass close to the tilt axis. A counterweight or a spring
  reduces the holding current.
- The pan axis benefits from a lazy-susan bearing so the servo only provides
  torque and does not carry the vertical load.
- The trigger servo horn should pull the trigger along its natural path. Find
  the rest and pull angles by hand with the arming switch off, then enter them
  in `config.h`.
