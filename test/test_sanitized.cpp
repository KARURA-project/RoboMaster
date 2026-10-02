#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

#include "robomaster/RoboMaster.hpp"

using namespace robomaster;

namespace {

CanFrame feedbackFrame(std::uint8_t id, std::uint16_t encoder)
{
    CanFrame frame;
    frame.id = 0x200U + id;
    frame.dlc = 8U;
    frame.data[0] = static_cast<std::uint8_t>(encoder >> 8U);
    frame.data[1] = static_cast<std::uint8_t>(encoder & 0xFFU);
    return frame;
}

}  // namespace

int main()
{
    Motor motor{1, MotorModel::M2006, ControllerModel::C610};
    MotorBus bus;
    assert(motor.setExpectedFeedbackRate(FeedbackRate::Hz1000) ==
        MotorStatus::Ok);
    assert(motor.setMaxSpeed(200.0) == MotorStatus::Ok);
    assert(motor.setTrackingMaxSpeed(500.0) == MotorStatus::Ok);
    assert(motor.setMaxCurrent(5.0) == MotorStatus::Ok);
    assert(motor.setPositionGains(PidGains{2.0, 0.0, 0.0}) ==
        MotorStatus::Ok);
    assert(motor.setSpeedGains(PidGains{0.05, 0.01, 0.0}) ==
        MotorStatus::Ok);
    assert(bus.add(motor) == MotorBusStatus::Ok);

    std::uint16_t encoder = 0U;
    for (std::uint32_t step = 0U; step < 100000U; ++step) {
        const std::uint32_t nowUs = step * 1000U;
        encoder = static_cast<std::uint16_t>((encoder + 5U) % kEncoderCounts);
        assert(bus.updateFeedback(feedbackFrame(1, encoder), nowUs) ==
            MotorBusStatus::Ok);
        if (step % 1000U == 0U) {
            const double target = (step / 1000U) % 2U == 0U ? 90.0 : -90.0;
            assert(motor.setTarget(ControlType::Position, target, nowUs) ==
                MotorStatus::Ok);
        }
        assert(bus.updateControl(nowUs) == MotorBusStatus::Ok);
        std::array<CanFrame, MotorBus::kMaxCommandFrames> frames{};
        std::size_t count = 0U;
        assert(bus.makeCommandFrames(frames, count) == MotorBusStatus::Ok);
        assert(count == 1U);
    }
    return 0;
}
