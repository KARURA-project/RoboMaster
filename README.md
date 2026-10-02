# RoboMaster

C++11 library for DJI RoboMaster M2006/M3508 motors with C610/C620 controllers. The control and protocol core is board-independent; the current Arduino package supports ESP32 through its TWAI CAN and hardware-timer adapter.

## Arduino installation

Place this repository in the Arduino libraries directory and restart Arduino
IDE after changing library files. Include the complete API with:

```cpp
#include <RoboMasterCore.h>

using namespace robomaster;
```

Create one `Motor` per physical motor and one `MotorBus` per CAN bus. A bus
accepts controller IDs 1 through 8:

```cpp
Motor motor1{1, MotorModel::M3508, ControllerModel::C620};
Motor motor2{2, MotorModel::M3508, ControllerModel::C620};
MotorBus bus;

bus.add(motor1);
bus.add(motor2);
```

All operations return typed status values. They can be printed without a
lookup table:

```cpp
const MotorStatus status = motor1.setMaxCurrent(2.0);
Serial.println(toString(status));
```

## Examples

- `XiaoEsp32s3ZeroCurrent`: safe wiring and communication check.
- `XiaoEsp32s3Speed`: speed PI control.
- `XiaoEsp32s3Position`: referenced multi-turn position control.
- `XiaoEsp32s3TwoMotors`: two motors on one CAN bus.
- `platformio/XiaoEsp32s3`: complete PlatformIO project.

ESP32 examples use XIAO ESP32S3 D0/GPIO1 for CAN TX and D1/GPIO2 for CAN RX at 1 Mbit/s. An external CAN transceiver and correct termination are required. See `docs/CORE_API.md` and `docs/ESP32.md`.

The PID gains and limits in the examples are tested starting values for an
unloaded M3508 with a C620. They are not universal tuning values. In
particular, the tracking maximum speed is an application assumption used to
judge multi-turn position integrity, not a guaranteed physical maximum.

## Native CMake / ROS 2

The board-independent core can be installed as a normal CMake package:

```sh
cmake -S . -B build -DROBOMASTER_BUILD_TESTS=OFF
cmake --build build
cmake --install build --prefix /path/to/prefix
```

An application or ROS 2 package can then use:

```cmake
find_package(RoboMasterCore CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE RoboMaster::Core)
```

The ESP32 TWAI and timer adapters are intentionally excluded from the native
target. A Linux SocketCAN adapter can therefore be added without introducing
Arduino dependencies into the control core.

## License

MIT License. See `LICENSE`.
