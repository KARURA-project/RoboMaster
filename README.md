# RoboMaster

Board-independent C++11 control and protocol library for DJI RoboMaster
M2006/M3508 motors with C610/C620 controllers.

The library owns no CAN peripheral, clock, hardware timer, interrupt, or task.
Applications supply received CAN frames and unsigned 32-bit microsecond timestamps,
and transmit the generated command frames using their existing CAN transport.
No board-specific implementation is required inside this library.

```cpp
#include <RoboMasterCore.h>
using namespace robomaster;

Motor motor{1, MotorModel::M3508, ControllerModel::C620};
MotorBus bus;
// During application initialization: bus.add(motor), then configure the motor.
```

## Application integration

Convert your driver's frame to `CanFrame` (`id`, `dlc`, `data`, `extended`,
`remote`) and call `bus.updateFeedback(frame, receivedAtUs)`. Regularly call
`bus.updateControl(nowUs)`, even when no feedback arrives, so watchdogs run.
To transmit, call `makeCommandFrames(frames, count)` and pass each frame to
an application-owned CAN sender. Optional `commandFramesDue(nowUs)` and
`markCommandFramesSent(nowUs)` helpers retain the existing send-period behavior;
mark sent only after every generated frame has been accepted by the transport.
Transport acceptance does not guarantee physical delivery.

Use one clock domain for all timestamps. Prefer actual receive timestamps when
available; otherwise drain the receive queue promptly and use processing time.
Process feedback before control and avoid blocking work between these operations.
A driver-frame conversion is the only board-specific glue normally needed.

Keep receive routing and shared-bus scheduling in the application. The library
never drains a CAN queue or sends frames on its own. Core PID periods, current
limits, feedback/command watchdogs, position integrity, and latched emergency
stops are unchanged.

When the transport reports lost feedback, call `notifyFeedbackLoss()`; when it
reports bus-off or error-passive, call `emergencyStopAll()` to preserve the
previous prototype's safety policy. These board-independent fault notifications
are optional for basic integration, but omitting them loses protection against
faults that cannot be inferred from frames and timestamps. Correct the cause
and explicitly recover; stale motion targets are never resumed automatically.
See [the core API](docs/CORE_API.md) for configuration and recovery behavior.

## Installation

Arduino: install the library and include `<RoboMasterCore.h>`. The package has
no board-specific dependencies. PlatformIO: add this repository to `lib_deps`.

Native CMake / ROS 2:

```sh
cmake -S . -B build -DROBOMASTER_BUILD_TESTS=OFF
cmake --build build
cmake --install build --prefix /path/to/prefix
```

```cmake
find_package(RoboMasterCore CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE RoboMaster::Core)
```

## Examples

Two ESP32S3 samples are provided for both Arduino and PlatformIO under
`examples/`: `XiaoEsp32s3Basic` repeats forward/stop/reverse/stop, and
`XiaoEsp32s3Keyboard` accepts position, speed, and current commands over serial.
Arduino sketches are directly under `examples/`; complete PlatformIO projects
are under `examples/platformio/`. Board identity is in each sample name.
See [the examples guide](examples/README.md) for wiring, controls, and validation.

CAN implementation files belong to each sample application, outside the library
source tree. No board-specific API is included by `RoboMasterCore.h`.

## Validation

Run `sh test/run_native_tests.sh` and `sh test/run_sanitized_tests.sh`.

## License

MIT License. See `LICENSE`.
