#pragma once

#include <cstdint>

namespace robomaster {

enum class ControlType : std::uint8_t {
    Current,
    Speed,
    Position,
};

struct ControlTarget {
    ControlTarget() = delete;

    constexpr ControlTarget(ControlType type, double value)
        : type(type), value(value)
    {
    }

    ControlType type;

    // Current: amperes. Speed: output-shaft rpm.
    // Position: output-shaft degrees.
    double value;
};

}  // namespace robomaster
