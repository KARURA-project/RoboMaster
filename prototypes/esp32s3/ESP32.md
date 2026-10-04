> Retained prototype reference, not a supported library platform API.
> Peripheral code lives in `board/` and is compiled by the prototype application.
> The PlatformIO project compiles `src/Board.cpp`; keep this directory structure.
> Arduino sketches are retained as references. To build one, copy the five files
> from `board/` into the sketch directory so Arduino compiles them as application
> sources. They are no longer supplied by the installed RoboMaster library.

# ESP32 board adapter

The ESP32 adapter targets the on-chip TWAI controller through Arduino-ESP32
2.x or 3.x. The verified hardware configuration is:

- Seeed Studio XIAO ESP32S3
- Arduino-ESP32 3.3.10
- PlatformIO Espressif 32 6.13.0 / Arduino-ESP32 2.0.17
- TCAN332G transceiver
- D0 / GPIO1 for TXD
- D1 / GPIO2 for RXD
- Classic CAN at 1 Mbit/s
- 1 ms hardware-timer notification
- TX queue length 8 and RX queue length 64

Include the core and the matching platform API with:

```cpp
#include <RoboMasterCore.h>
#include "Esp32.hpp"
```

`Esp32Timer` only increments a saturating notification counter in its ISR.
Call `takePending()` from `loop()`. When the returned value is greater than
one, record the scheduling delay but run the current control/send operation
only once; old CAN commands must not be replayed in a burst.

Arduino-ESP32 2.x does not support an ISR argument for its hardware-timer API,
so only one `Esp32Timer` instance can be active there. Arduino-ESP32 3.x does
not have this restriction.

`Esp32Can::receive()` drains the TWAI receive queue, converts feedback into
the board-independent `CanFrame`, and routes it through `MotorBus`. It also
checks the driver's missed, overrun, bus-error, error-passive, and bus-off
state. A missed, overrun, or bus-error event calls
`MotorBus::notifyFeedbackLoss()`, invalidating accumulated positions and
zeroing active motor commands. Error-passive and bus-off latch the emergency
stop for every registered motor.

The processing timestamp returned by `micros()` is suitable for freshness and
approximate nominal-rate detection. It is not treated as proof that a frame
was lost because frames may wait in the FreeRTOS receive queue and then be
dequeued in a burst.

Do not print large diagnostics while polling CAN. At approximately 1 kHz, a
64-entry RX queue holds only about 64 ms of feedback. The zero-current example
therefore runs silently for ten seconds, stops the peripherals, and only then
prints its report.

See the verified Arduino integration examples:

- `prototypes/esp32s3/examples/XiaoEsp32s3ZeroCurrent`
- `prototypes/esp32s3/examples/XiaoEsp32s3Speed`
- `prototypes/esp32s3/examples/XiaoEsp32s3Position`
- `prototypes/esp32s3/examples/XiaoEsp32s3TwoMotors`

## PlatformIO

Open `prototypes/esp32s3/examples/platformio/XiaoEsp32s3` as a PlatformIO project. Its
`lib_deps` setting points at this repository,
so a local library edit is used by the next build without copying files. When using an installed or
published release, replace the local `lib_deps` entry and add `RoboMaster` to `lib_deps`.
