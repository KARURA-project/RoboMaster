#include <cassert>
#include <cstring>

#include "robomaster/Diagnostics.hpp"

using namespace robomaster;

namespace {

void expect(const char *actual, const char *expected)
{
    assert(std::strcmp(actual, expected) == 0);
}

}  // namespace

int main()
{
    expect(toString(ProtocolStatus::InvalidFrameLength), "InvalidFrameLength");
    expect(toString(PidStatus::CalculationOverflow), "CalculationOverflow");
    expect(toString(MotorStateStatus::InvalidEncoder), "InvalidEncoder");
    expect(toString(FeedbackTimingStatus::Mismatch), "Mismatch");
    expect(toString(MotorStatus::CommandTimeout), "CommandTimeout");
    expect(toString(MotorBusStatus::DuplicateId), "DuplicateId");

    expect(toString(static_cast<ProtocolStatus>(0xFF)), "Unknown");
    expect(toString(static_cast<MotorStatus>(0xFF)), "Unknown");
    return 0;
}
