#include <cassert>
#include <cmath>
#include <cstdint>

#include "robomaster/MotorState.hpp"
#include "robomaster/Pid.hpp"
#include "robomaster/Protocol.hpp"

using namespace robomaster;

namespace {

std::uint32_t nextRandom(std::uint32_t &state)
{
    state = state * 1664525U + 1013904223U;
    return state;
}

CanFrame feedbackFrame(
    std::uint8_t id,
    std::uint16_t encoder,
    std::int16_t rpm)
{
    CanFrame frame;
    frame.id = 0x200U + id;
    frame.dlc = 8;
    frame.data[0] = static_cast<std::uint8_t>(encoder >> 8U);
    frame.data[1] = static_cast<std::uint8_t>(encoder & 0xFFU);
    const std::uint16_t rawRpm = static_cast<std::uint16_t>(rpm);
    frame.data[2] = static_cast<std::uint8_t>(rawRpm >> 8U);
    frame.data[3] = static_cast<std::uint8_t>(rawRpm & 0xFFU);
    return frame;
}

void testCurrentConversionAcrossFullCommandRange()
{
    const ControllerModel models[] = {
        ControllerModel::C610,
        ControllerModel::C620,
    };
    for (const ControllerModel model : models) {
        ControllerSpec spec;
        assert(getControllerSpec(model, spec) == ProtocolStatus::Ok);
        for (std::int32_t step = -1000; step <= 1000; ++step) {
            const double requested = spec.currentMaxA *
                static_cast<double>(step) / 1000.0;
            std::int16_t raw = 0;
            assert(encodeCurrent(requested, model, raw) == ProtocolStatus::Ok);
            assert(raw >= -spec.currentFullScale);
            assert(raw <= spec.currentFullScale);

            double estimated = 0.0;
            assert(estimateCurrent(raw, model, estimated) ==
                ProtocolStatus::Ok);
            const double resolution = spec.currentMaxA /
                static_cast<double>(spec.currentFullScale);
            assert(std::fabs(estimated - requested) <= resolution * 0.51);
        }
    }
}

void testPidAlwaysReturnsFiniteBoundedOutput()
{
    Pid pid{PidConfig{
        PidGains{0.7, 0.4, 0.02},
        PidLimits{-12.0, 7.0}}};
    std::uint32_t random = 0x12345678U;
    for (std::size_t i = 0; i < 100000U; ++i) {
        const double target = static_cast<double>(
            static_cast<std::int32_t>(nextRandom(random) % 20001U) - 10000) /
            10.0;
        const double measured = static_cast<double>(
            static_cast<std::int32_t>(nextRandom(random) % 20001U) - 10000) /
            10.0;
        const double dt = static_cast<double>(
            1U + nextRandom(random) % 10000U) / 1000000.0;
        const PidResult result = pid.update(target, measured, dt);
        assert(result.status == PidStatus::Ok);
        assert(std::isfinite(result.output));
        assert(result.output >= -12.0);
        assert(result.output <= 7.0);
        assert(std::isfinite(pid.integralOutput()));
        assert(pid.integralOutput() >= -12.0);
        assert(pid.integralOutput() <= 7.0);
    }
}

void testLongRandomEncoderWalk()
{
    MotorState state{1, MotorModel::M3508, ControllerModel::C620};
    assert(state.setExpectedFeedbackRate(FeedbackRate::Hz1000) ==
        MotorStateStatus::Ok);
    assert(state.setMaxRpm(5000.0) == MotorStateStatus::Ok);
    assert(state.setRotorPositionDegrees(0.0) == MotorStateStatus::Ok);

    std::int64_t expectedCounts = 0;
    std::int32_t encoder = 0;
    std::uint32_t nowUs = 0;
    assert(state.update(feedbackFrame(1, 0, 0), nowUs) ==
        MotorStateStatus::Ok);

    std::uint32_t random = 0x87654321U;
    for (std::size_t i = 0; i < 100000U; ++i) {
        const std::int32_t delta = static_cast<std::int32_t>(
            nextRandom(random) % 201U) - 100;
        expectedCounts += delta;
        encoder = (encoder + delta) % static_cast<std::int32_t>(kEncoderCounts);
        if (encoder < 0) {
            encoder += static_cast<std::int32_t>(kEncoderCounts);
        }
        nowUs += 1000U;
        assert(state.update(feedbackFrame(
            1,
            static_cast<std::uint16_t>(encoder),
            0), nowUs) == MotorStateStatus::Ok);
    }

    const double actualCounts = state.rotorPositionDegrees() *
        static_cast<double>(kEncoderCounts) / 360.0;
    assert(std::fabs(actualCounts - static_cast<double>(expectedCounts)) <
        1e-6);
    assert(!hasPositionFlag(
        state.positionFlags(), PositionFlag::AmbiguousDelta));
}

}  // namespace

int main()
{
    testCurrentConversionAcrossFullCommandRange();
    testPidAlwaysReturnsFiniteBoundedOutput();
    testLongRandomEncoderWalk();
    return 0;
}
