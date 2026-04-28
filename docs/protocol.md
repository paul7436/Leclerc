# Serial protocol

The host and the firmware exchange newline-terminated ASCII lines over USB
serial at 115200 baud, 8N1. Lines may end with `\n` or `\r\n`. The firmware is
the authority on every safety rule; the host is never trusted.

## Host to firmware

| Command | Arguments | Reply | Effect |
| --- | --- | --- | --- |
| `A<pan> <tilt>` | two decimal numbers, degrees | none on success | Set the pan and tilt targets. Values are clamped to the firmware limits. A space after `A` is allowed. |
| `F` | none | `FIRE OK` or `FIRE DENIED <reason>` | Request one trigger pull. |
| `S` | none | `STATUS ...` | Request a status line. |

Command letters are upper case. Examples: `A90 75`, `A 101.5 80.25`, `F`, `S`.

Numbers accept an optional sign and a decimal point (`-12.5`, `90`, `90.`).
Exponents and hexadecimal are not part of the protocol.

## Firmware to host

| Line | When |
| --- | --- |
| `READY <version>` | Once after boot. |
| `STATUS armed=<0/1> link=<0/1> pan=<deg> tilt=<deg> trigger=<state> cooldown_ms=<ms>` | Reply to `S`. `pan` and `tilt` are the angles currently driven, after clamping and slew limiting. `trigger` is `IDLE`, `PULL`, `HOLD` or `RELEASE`. `cooldown_ms` is the time left before the next shot can start. |
| `FIRE OK` | `F` accepted, the trigger sweep has started. |
| `FIRE DENIED <reason>` | `F` refused. See the reasons below. |
| `EVT ARM <0/1>` | The debounced arming switch state changed. |
| `EVT LINK <0/1>` | The link watchdog timed out (`0`) or recovered (`1`). |
| `ERR <code>` | The last line was rejected. See the codes below. |

Parsers on the host must ignore lines they do not recognise, so the firmware
can add new messages later.

### Fire denial reasons

Checked in this order:

| Reason | Meaning |
| --- | --- |
| `DISARMED` | The arming switch is off (sense pin not pulled low). |
| `LINK` | The link watchdog has timed out and was not re-established by `A` or `S`. |
| `BUSY` | A trigger sweep is already in progress. |
| `COOLDOWN` | The minimum time since the end of the previous sweep has not elapsed. |

### Error codes

| Code | Meaning |
| --- | --- |
| `OVERFLOW` | The line exceeded `kMaxLineLength` characters and was discarded. |
| `UNKNOWN` | The command letter is not `A`, `F` or `S`. |
| `SYNTAX` | Missing, extra or malformed arguments. |
| `VALUE` | A number has more than six integer digits. |

Empty lines are ignored without a reply.

## Link watchdog

Every valid command records the time it was received. If no valid command is
received for `kLinkTimeoutMs` (500 ms by default), the firmware:

1. sends `EVT LINK 0`;
2. freezes the pan and tilt targets at the angles currently driven;
3. aborts any trigger sweep in progress by returning the trigger to rest;
4. refuses `F` with `FIRE DENIED LINK`.

The link is re-established by the next `A` or `S` command, which sends
`EVT LINK 1` before its own effect. An `F` command alone never re-establishes
the link, so a stale fire request cannot be the first thing that happens after
a communication gap. The host keeps the link alive by polling `S` every
`serial.status_period_s` (200 ms by default).

## Trigger sweep

```text
 IDLE --F accepted--> PULL --kTriggerTravelMs--> HOLD --kTriggerHoldMs--> RELEASE --kTriggerReleaseMs--> IDLE
                       |                          |
                       +------ abort (switch off or link lost) ------------> RELEASE
```

- `PULL`: the trigger servo is commanded to `kTriggerPullDeg` and given
  `kTriggerTravelMs` to get there.
- `HOLD`: the trigger stays pulled for `kTriggerHoldMs`.
- `RELEASE`: the servo is commanded back to `kTriggerRestDeg` and given
  `kTriggerReleaseMs` to return.
- The next shot may start `kShotCooldownMs` after the end of `RELEASE`.

If the arming switch is turned off or the link times out during `PULL` or
`HOLD`, the sweep jumps to `RELEASE`, so the trigger servo is commanded to rest
and does not snap back to the pull angle when power returns.

## Timing budget

At 115200 baud a 20 character `A` command takes under 2 ms to transmit, so a
30 Hz control loop plus a 5 Hz status poll uses well under 10 percent of the
link.
