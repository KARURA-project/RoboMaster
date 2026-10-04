# XIAO ESP32S3 samples

Two applications, each supplied for Arduino IDE and PlatformIO:

| Application | Arduino sketch | PlatformIO project |
| --- | --- | --- |
| Basic | `arduino/XiaoEsp32s3Basic` | `platformio/basic` |
| Keyboard | `arduino/XiaoEsp32s3Keyboard` | `platformio/keyboard` |

Use an M3508 + C620 with controller ID 1, 1 Mbit/s CAN and 1 kHz feedback.
Connect XIAO D0/GPIO1 to transceiver TXD and D1/GPIO2 to RXD; use a compatible
3.3 V CAN transceiver (the prior prototype used TCAN332G), common ground and
correct CAN termination. The motor controller needs its own power supply.
The current limit is 2 A. Gains are starting values for an unloaded M3508;
adjust limits and gains for your mechanism before applying power.

Install the RoboMaster core library for Arduino, then open the desired `.ino`.
The sketch-local `Esp32Can.cpp/.hpp` are application sources; keep them beside
the sketch. For PlatformIO, open the desired project and run build/upload.
Its local library dependency points to this repository; keep the directory layout.
Serial monitor baud is 115200. No dedicated hardware timer is used.

Basic automatically starts after feedback arrives and repeats +30 rpm, zero
speed, -30 rpm, zero speed, for two seconds each. Zero speed is active speed
control, not zero current. CAN processing continues throughout every phase.

Keyboard starts at zero current. Send individual characters (newlines ignored):

| Selection | Control | Positive | Zero | Negative |
| --- | --- | --- | --- | --- |
| `1` | Position | `q`: +90 deg | `a`: 0 deg | `z`: -90 deg |
| `2` | Speed | `w`: +30 rpm | `s`: 0 rpm | `x`: -30 rpm |
| `3` | Current | `e`: +0.5 A | `d`: 0 A | `c`: -0.5 A |

Selecting a mode first coasts. Keys for other modes are ignored. Space coasts;
`!` latches an emergency stop. Reset the application to recover from a fault.
Position zero is referenced to the first feedback after startup: `a` moves back
to that reference, so it is not a general stop key. Targets persist without
further input; a disconnected serial terminal does not stop motion.

Both applications wait for feedback without blocking (2 s startup timeout),
continue control/receive processing without new input, and latch a stop on CAN,
feedback or control errors. Receive overflow and driver errors preserve the
previous prototype's feedback-loss and emergency-stop notifications. On a fault,
zero-current sends continue when the transport is available; a failed bus cannot
guarantee delivery. Position integrity must be re-established after a reset.

These replace the earlier per-mode and two-motor samples. Hardware execution of
these new applications has not yet been verified; build results are separate from
physical CAN/motor verification.

Build validation: both PlatformIO projects and both actual Arduino `.ino`
sketches compiled for XIAO ESP32S3 using PlatformIO Espressif32 6.13.0 /
Arduino-ESP32 2.0.17. The sketches were built through temporary PlatformIO
projects; Arduino IDE itself and hardware operation were not tested.
