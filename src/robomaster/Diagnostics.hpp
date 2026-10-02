#pragma once

#include "MotorBus.hpp"

namespace robomaster {

const char *toString(ProtocolStatus status);
const char *toString(PidStatus status);
const char *toString(MotorStateStatus status);
const char *toString(FeedbackTimingStatus status);
const char *toString(MotorStatus status);
const char *toString(MotorBusStatus status);

}  // namespace robomaster
