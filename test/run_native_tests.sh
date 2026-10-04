#!/bin/sh
set -eu

root_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
compiler=${CXX:-c++}
compiler_name=$(basename "$compiler")
build_dir="${TMPDIR:-/tmp}/robomaster-native-tests-$compiler_name"

mkdir -p "$build_dir"

"$compiler" \
    -std=c++11 \
    -Wall \
    -Wextra \
    -Wpedantic \
    -Werror \
    -I"$root_dir/src" \
    "$root_dir/src/robomaster/Protocol.cpp" \
    "$root_dir/test/test_protocol.cpp" \
    -o "$build_dir/test_protocol"

"$build_dir/test_protocol"

"$compiler" \
    -std=c++11 \
    -Wall \
    -Wextra \
    -Wpedantic \
    -Werror \
    -I"$root_dir/src" \
    "$root_dir/test/test_control.cpp" \
    -o "$build_dir/test_control"

"$build_dir/test_control"

"$compiler" \
    -std=c++11 \
    -Wall \
    -Wextra \
    -Wpedantic \
    -Werror \
    -I"$root_dir/src" \
    "$root_dir/src/robomaster/Diagnostics.cpp" \
    "$root_dir/test/test_diagnostics.cpp" \
    -o "$build_dir/test_diagnostics"

"$build_dir/test_diagnostics"

"$compiler" \
    -std=c++11 \
    -Wall \
    -Wextra \
    -Wpedantic \
    -Werror \
    -I"$root_dir/src" \
    "$root_dir/src/robomaster/Pid.cpp" \
    "$root_dir/test/test_pid.cpp" \
    -o "$build_dir/test_pid"

"$build_dir/test_pid"

"$compiler" \
    -std=c++11 \
    -Wall \
    -Wextra \
    -Wpedantic \
    -Werror \
    -I"$root_dir/src" \
    "$root_dir/src/robomaster/Protocol.cpp" \
    "$root_dir/src/robomaster/MotorState.cpp" \
    "$root_dir/test/test_motor_state.cpp" \
    -o "$build_dir/test_motor_state"

"$build_dir/test_motor_state"

"$compiler" \
    -std=c++11 \
    -Wall \
    -Wextra \
    -Wpedantic \
    -Werror \
    -I"$root_dir/src" \
    "$root_dir/src/robomaster/Protocol.cpp" \
    "$root_dir/src/robomaster/MotorState.cpp" \
    "$root_dir/src/robomaster/Pid.cpp" \
    "$root_dir/src/robomaster/Motor.cpp" \
    "$root_dir/test/test_motor.cpp" \
    -o "$build_dir/test_motor"

"$build_dir/test_motor"

"$compiler" \
    -std=c++11 \
    -Wall \
    -Wextra \
    -Wpedantic \
    -Werror \
    -I"$root_dir/src" \
    "$root_dir/src/robomaster/Protocol.cpp" \
    "$root_dir/src/robomaster/MotorState.cpp" \
    "$root_dir/src/robomaster/Pid.cpp" \
    "$root_dir/src/robomaster/Motor.cpp" \
    "$root_dir/src/robomaster/MotorBus.cpp" \
    "$root_dir/test/test_motor_bus.cpp" \
    -o "$build_dir/test_motor_bus"

"$build_dir/test_motor_bus"

"$compiler" \
    -std=c++11 \
    -Wall \
    -Wextra \
    -Wpedantic \
    -Werror \
    -I"$root_dir/src" \
    "$root_dir/src/robomaster/Protocol.cpp" \
    "$root_dir/src/robomaster/MotorState.cpp" \
    "$root_dir/src/robomaster/Pid.cpp" \
    "$root_dir/src/robomaster/Motor.cpp" \
    "$root_dir/test/test_simulation.cpp" \
    -o "$build_dir/test_simulation"

"$build_dir/test_simulation"

"$compiler" \
    -std=c++11 \
    -Wall \
    -Wextra \
    -Wpedantic \
    -Werror \
    -I"$root_dir/src" \
    "$root_dir/src/robomaster/Protocol.cpp" \
    "$root_dir/src/robomaster/MotorState.cpp" \
    "$root_dir/src/robomaster/Pid.cpp" \
    "$root_dir/test/test_properties.cpp" \
    -o "$build_dir/test_properties"

"$build_dir/test_properties"

# The public package must not require ESP32 headers even on an ESP32 build.
"$compiler" -std=c++11 -Wall -Wextra -Wpedantic -Werror \
    -DARDUINO_ARCH_ESP32=1 -I"$root_dir/src" \
    "$root_dir"/src/robomaster/*.cpp \
    "$root_dir/test/test_portable_boundary.cpp" \
    -o "$build_dir/test_portable_boundary"
"$build_dir/test_portable_boundary"
