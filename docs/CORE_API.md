# Board-independent core

The headers under `src/robomaster` do not depend on Arduino, a CAN driver,
an operating system, dynamic allocation, or exceptions. They require C++11.

Include the complete public API with:

```cpp
#include <robomaster/RoboMaster.hpp>
```

Every public status enum can be passed to `toString(status)`. This returns a
static diagnostic name such as `"CommandTimeout"` without allocation or
Arduino dependencies.

## Hardware boundary

`RoboMasterCore.h` always exposes only the board-independent API, including on
ESP32. CAN initialization, receive/send queues, clock reads, hardware timers,
interrupts, and application tasks are owned by the application. The library
accepts `CanFrame` values and timestamps; generating a frame performs no I/O.
No hardware timer is required. Regular application calls drive the core's
existing period checks and watchdogs.

Transport fault notifications remain available: call `notifyFeedbackLoss()`
for lost feedback and `emergencyStopAll()` for transport faults requiring a
latched stop. Frame/time input alone cannot detect every receive-queue loss.
Use one timestamp clock domain and call `updateControl()` even without new
feedback. Never mark command frames sent after a failed or partial batch.

## Object model

- Create one `robomaster::Motor` for each physical motor.
- Create one `robomaster::MotorBus` for each physical CAN bus.
- Register up to eight motors in a bus with `MotorBus::add()`.
- A registered `Motor` must outlive its `MotorBus`. Motors and buses cannot be
  copied or moved, so their physical identity and stored addresses stay stable.

## Units and ratios

- Position targets and readings use output-shaft degrees.
- Speed targets and readings use output-shaft rpm.
- Current targets, commands, and estimates use amperes.
- `setMaxSpeed()` limits commanded output-shaft speed. Measured speed may
  temporarily exceed it because of inertia or an external force.
- `setTrackingMaxSpeed()` separately declares the maximum possible
  output-shaft speed used to judge whether encoder unwrapping is trustworthy.
  Position control requires this tracking limit to be set.
- `setExternalGearRatio(ratio)` uses motor-shaft rotations divided by
  output-shaft rotations. It is multiplied by the motor's internal ratio.
- Direction changes the signs seen by the application, not the raw protocol.

Set the external ratio and direction before accepting feedback or setting a
target. Set the maximum speed and maximum current before nonzero control, and
set the tracking maximum speed before position control.

## Typical non-blocking loop

1. Convert each received native CAN frame to `robomaster::CanFrame` and call
   `MotorBus::updateFeedback(frame, nowUs)`.
2. Call `MotorBus::updateControl(nowUs)` as often as practical. Each control
   loop runs only when its configured period has elapsed.
3. When `MotorBus::commandFramesDue(nowUs)` is true, call
   `makeCommandFrames()`, convert and send every returned frame, then call
   `markCommandFramesSent(nowUs)`.

All timestamps are unsigned 32-bit microseconds. Elapsed-time calculations
remain valid across normal timer wraparound.

## Targets and stopping

`setTarget(type, value, nowUs)` selects the control type for that individual
command. Changing type resets the related PID state; updating a target of the
same type retains it.

- `coast()` commands zero current.
- A zero-current `coast()` target does not expire through the command
  watchdog because it is already the watchdog's safe output.
- `brake()` commands zero speed using only the configured speed `Kp`. It does
  not accumulate integral output, so its current returns to zero at rest.
- `setTarget(ControlType::Speed, 0.0, nowUs)` uses the complete configured
  speed PID when integral action at zero speed is intentionally required.
- `hold()` captures and controls the current position.
- `emergencyStop()` immediately commands zero current and latches the stop.
  `clearEmergencyStop()` clears the latch but does not restore the old target.

Invalid targets are rejected and clear the active target. They are never
silently clamped. Current commands are constrained by both the controller's
physical limit and the user-configured maximum.

## Feedback and position trust

`MotorState` measures the actual feedback interval and exposes the observed
frequency, nominal detected rate, timing status, and one-byte position flags.
Position accumulation becomes untrusted when its configured speed/rate pair
can cross 180 degrees per frame, measured rpm exceeds the configured maximum,
the platform reports lost feedback with `notifyFeedbackLoss()`, or an encoder
delta is ambiguous. These faults latch until the position is explicitly
referenced again with `setPosition()`.

The timestamp passed to `updateFeedback()` is the time at which software
processed the frame. It is used for freshness and approximate rate detection,
but scheduling jitter alone does not set `FrameGap`. A platform adapter must
call `MotorBus::notifyFeedbackLoss()` when its CAN driver reports a missed or
overrun frame.

Relative position control is allowed without an absolute startup reference as
long as accumulation remains continuous. `isPositionAbsolute()` additionally
requires an explicit reference.

## Fail-safe behavior

Malformed feedback for a registered motor, stale feedback, a stale command,
an invalid target, a PID calculation error, or an untrusted position used for
position control clears that motor's target and commands zero current.
Malformed or unknown traffic that cannot be assigned to a registered motor
does not stop unrelated motors.

The command timeout is disabled by default. Feedback loss protection is always
active for nonzero output; its default threshold is three feedback periods.
When a command timeout, feedback loss, or position-integrity fault clears a
target, recovery is explicit: correct the cause, re-reference position with
`setPosition()` if its flags remain latched, and then issue a new target. The
library never resumes the previous motion command automatically.
