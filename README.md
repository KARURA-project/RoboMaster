# RoboMaster

Board-independent C++11 control and protocol library for DJI RoboMaster
M2006/M3508 motors with C610/C620 controllers. CAN hardware access is provided
by the separate [CANBridge](https://github.com/KARURA-project/CANBridge) library.

RoboMaster owns no CAN peripheral, driver, clock, hardware timer, interrupt, or
task. It consumes and produces `canbridge::Frame` values only. CANBridge owns
controller selection, initialization, receive/send operations and transport
health. This keeps every board-specific implementation outside RoboMaster.

```cpp
#include <RoboMasterCore.h>
using namespace robomaster;

Motor motor{1, MotorModel::M3508, ControllerModel::C620};
MotorBus bus;
// During application initialization: bus.add(motor), then configure the motor.
```

## Application integration

Pass each `canbridge::Frame` returned by `canbridge::Bus::receive()` directly to
`robomaster::MotorBus::updateFeedback(frame, receivedAtUs)`. Regularly call
`updateControl(nowUs)`, even when no feedback arrives, so watchdogs run. Pass
the frames from `makeCommandFrames()` directly to `canbridge::Bus::send()`.
No frame conversion or duplicate CAN type is required.

Optional `commandFramesDue(nowUs)` and `markCommandFramesSent(nowUs)` helpers
retain the send-period behavior. Mark a batch sent only after CANBridge accepts
every generated frame. Preserve a frame and retry when CANBridge returns
`Result::Busy`; transport acceptance does not guarantee physical delivery.

Use one clock domain for all timestamps. Prefer actual receive timestamps when
available; otherwise drain the receive queue promptly and use processing time.
Process feedback before control and avoid blocking work between these operations.
Keep receive routing and shared-bus scheduling in the application. RoboMaster
never initializes CAN, drains a receive queue or sends a frame. Core PID
periods, current limits, feedback/command watchdogs, position integrity, and
latched emergency stops are unchanged.

After `canbridge::Bus::pollHealth()`, call `notifyFeedbackLoss()` when
`Health::receiveLoss` is set, and call `emergencyStopAll()` for bus-off or
error-passive according to the application's safety policy. Omitting these
notifications loses protection against faults that cannot be inferred from
frames and timestamps. Correct the cause and explicitly recover; stale motion
targets are never resumed automatically.
See [the core API](docs/CORE_API.md) for configuration and recovery behavior.

## Installation

Arduino: install CANBridge and RoboMaster, then include `<RoboMasterCore.h>` and
one CANBridge controller header. PlatformIO: add both repositories to `lib_deps`.

```ini
lib_deps =
    https://github.com/KARURA-project/CANBridge.git
    https://github.com/KARURA-project/RoboMaster.git
```

Version 0.2 replaces the former `robomaster::CanFrame` with
`canbridge::Frame`. Its payload-length member is `length` rather than `dlc`.
Code that used the example-local `Esp32Can` must select a CANBridge controller
header and use `canbridge::Bus` instead.

Native CMake / ROS 2:

```sh
cmake -S . -B build -DROBOMASTER_BUILD_TESTS=OFF \
  -DCANBRIDGE_INCLUDE_DIR=/path/to/CANBridge/src
cmake --build build
cmake --install build --prefix /path/to/prefix
```

```cmake
find_package(RoboMasterCore CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE RoboMaster::Core)
```

## Examples

The representative examples use XIAO ESP32S3 internal CAN through
`CANBridge/EspCan.h`: `XiaoEsp32s3Basic` repeats
forward/stop/reverse/stop, and `XiaoEsp32s3Keyboard` accepts position, speed,
and current commands over serial. Arduino sketches are directly under
`examples/`; complete PlatformIO projects are under `examples/platformio/`.

RoboMaster logic is not tied to that board. To use another CANBridge-supported
combination, such as MCP2515 or MCP2518FD on ESP32/RP2040 boards, replace only
the CANBridge controller header and `canbridge::Config` initialization. Frame
routing and all RoboMaster control code remain unchanged. See
[the examples guide](examples/README.md) and the CANBridge repository for the
available board/controller combinations.

## Validation

Place CANBridge beside RoboMaster, or set `CANBRIDGE_INCLUDE_DIR` to its `src`
directory, then run `sh test/run_native_tests.sh` and
`sh test/run_sanitized_tests.sh`.

## License

MIT License. See `LICENSE`.
