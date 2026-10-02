#!/bin/sh
set -eu

root_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
build_dir="${TMPDIR:-/tmp}/robomaster-sanitized-tests"
binary="$build_dir/test_all"

mkdir -p "$build_dir"

"${CXX:-clang++}" \
    -std=c++11 \
    -Wall \
    -Wextra \
    -Wpedantic \
    -Werror \
    -fsanitize=address,undefined \
    -fno-omit-frame-pointer \
    -I"$root_dir/src" \
    "$root_dir/src/robomaster/Protocol.cpp" \
    "$root_dir/src/robomaster/MotorState.cpp" \
    "$root_dir/src/robomaster/Pid.cpp" \
    "$root_dir/src/robomaster/Motor.cpp" \
    "$root_dir/src/robomaster/MotorBus.cpp" \
    "$root_dir/test/test_sanitized.cpp" \
    -o "$binary"

"$binary"
