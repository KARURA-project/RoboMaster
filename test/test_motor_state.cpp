#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "robomaster/MotorState.hpp"

using namespace robomaster;

namespace {

canbridge::Frame feedbackFrame(
    std::uint8_t id,
    std::uint16_t encoder,
    std::int16_t rpm)
{
    canbridge::Frame frame;
    frame.id = 0x200U + id;
    frame.length = 8;
    frame.data[0] = static_cast<std::uint8_t>(encoder >> 8U);
    frame.data[1] = static_cast<std::uint8_t>(encoder & 0xFFU);
    const std::uint16_t rawRpm = static_cast<std::uint16_t>(rpm);
    frame.data[2] = static_cast<std::uint8_t>(rawRpm >> 8U);
    frame.data[3] = static_cast<std::uint8_t>(rawRpm & 0xFFU);
    return frame;
}

bool near(double actual, double expected, double tolerance = 1e-9)
{
    return std::fabs(actual - expected) <= tolerance;
}

void testRejectsInvalidConfigurationAndWrongMotor()
{
    MotorState invalidId{0, MotorModel::M3508, ControllerModel::C620};
    static_assert(sizeof(PositionFlags) == 1U, "PositionFlags must be one byte");
    assert(invalidId.configurationStatus() == MotorStateStatus::InvalidId);

    MotorState state{1, MotorModel::M3508, ControllerModel::C620};
    assert(state.configurationStatus() == MotorStateStatus::Ok);
    assert(state.update(feedbackFrame(2, 0, 0), 1000) ==
        MotorStateStatus::InvalidId);
    assert(!state.hasFeedback());
}

void testInitialFlagsAndExplicitReference()
{
    MotorState state{1, MotorModel::M3508, ControllerModel::C620};
    assert(hasPositionFlag(state.positionFlags(), PositionFlag::NoFeedback));
    assert(hasPositionFlag(state.positionFlags(), PositionFlag::NoReference));
    assert(hasPositionFlag(state.positionFlags(), PositionFlag::MaxRpmUnset));
    assert(hasPositionFlag(state.positionFlags(), PositionFlag::RateUnknown));

    assert(state.setRotorPositionDegrees(90.0) == MotorStateStatus::Ok);
    assert(!hasPositionFlag(
        state.positionFlags(), PositionFlag::NoReference));
    assert(near(state.rotorPositionDegrees(), 90.0));
}

void testMeasuresAndDetectsFeedbackRate()
{
    MotorState state{1, MotorModel::M3508, ControllerModel::C620};
    assert(state.setMaxRpm(10000.0) == MotorStateStatus::Ok);

    std::uint32_t nowUs = 1000;
    assert(state.update(feedbackFrame(1, 0, 0), nowUs) ==
        MotorStateStatus::Ok);
    for (std::uint16_t i = 1; i <= 8; ++i) {
        nowUs += 1000;
        assert(state.update(feedbackFrame(1, i, 0), nowUs) ==
            MotorStateStatus::Ok);
    }

    assert(state.feedbackTimingStatus() == FeedbackTimingStatus::Ready);
    assert(near(state.observedFeedbackHz(), 1000.0));
    assert(state.observedFeedbackPeriodUs() == 1000);
    FeedbackRate rate;
    assert(state.detectedFeedbackRate(rate));
    assert(rate == FeedbackRate::Hz1000);
    assert(!hasPositionFlag(state.positionFlags(), PositionFlag::RateUnknown));
    assert(!hasPositionFlag(state.positionFlags(), PositionFlag::UnsafeRate));
}

void testDetectsEverySupportedFeedbackRate()
{
    const FeedbackRate rates[] = {
        FeedbackRate::Hz125,
        FeedbackRate::Hz250,
        FeedbackRate::Hz500,
        FeedbackRate::Hz1000,
    };

    for (const FeedbackRate expected : rates) {
        MotorState state{1, MotorModel::M3508, ControllerModel::C620};
        std::uint32_t nowUs = 0;
        state.update(feedbackFrame(1, 0, 0), nowUs);
        for (std::uint16_t i = 1; i <= 8; ++i) {
            nowUs += feedbackPeriodUs(expected);
            state.update(feedbackFrame(1, i, 0), nowUs);
        }

        FeedbackRate detected;
        assert(state.detectedFeedbackRate(detected));
        assert(detected == expected);
    }
}

void testUnstableFeedbackPeriodIsNotClassified()
{
    MotorState state{1, MotorModel::M3508, ControllerModel::C620};
    std::uint32_t nowUs = 0;
    state.update(feedbackFrame(1, 0, 0), nowUs);
    for (std::uint16_t i = 1; i <= 8; ++i) {
        nowUs += 3000;
        state.update(feedbackFrame(1, i, 0), nowUs);
    }

    FeedbackRate detected;
    assert(!state.detectedFeedbackRate(detected));
    assert(state.feedbackTimingStatus() == FeedbackTimingStatus::Unstable);
}

void testTimestampWraparound()
{
    MotorState state{1, MotorModel::M3508, ControllerModel::C620};
    std::uint32_t nowUs = UINT32_MAX - 499U;
    state.update(feedbackFrame(1, 0, 0), nowUs);
    for (std::uint16_t i = 1; i <= 8; ++i) {
        nowUs += 1000U;
        state.update(feedbackFrame(1, i, 0), nowUs);
    }
    assert(near(state.observedFeedbackHz(), 1000.0));
}

void testMedianDetectionSurvivesDequeueJitter()
{
    MotorState state{1, MotorModel::M3508, ControllerModel::C620};
    std::uint32_t nowUs = 0;
    state.update(feedbackFrame(1, 0, 0), nowUs);
    for (std::uint16_t i = 1; i <= 8; ++i) {
        nowUs += i == 4 ? 2000U : 1000U;
        state.update(feedbackFrame(1, i, 0), nowUs);
    }

    FeedbackRate rate;
    assert(state.detectedFeedbackRate(rate));
    assert(rate == FeedbackRate::Hz1000);
    assert(state.observedFeedbackHz() < 1000.0);
    assert(!hasPositionFlag(state.positionFlags(), PositionFlag::FrameGap));
}

void testExplicitFeedbackLossInvalidatesPosition()
{
    MotorState state{1, MotorModel::M3508, ControllerModel::C620};
    state.setExpectedFeedbackRate(FeedbackRate::Hz1000);
    state.setMaxRpm(1000.0);
    state.update(feedbackFrame(1, 0, 0), 0);
    state.update(feedbackFrame(1, 1, 0), 2000);
    state.notifyFeedbackLoss();
    assert(hasPositionFlag(state.positionFlags(), PositionFlag::FrameGap));
    assert(!state.isPositionContinuous());
    assert(state.feedbackTimingStatus() == FeedbackTimingStatus::Measuring);
}

void testExpectedRateMismatch()
{
    MotorState state{1, MotorModel::M2006, ControllerModel::C610};
    assert(state.setExpectedFeedbackRate(FeedbackRate::Hz500) ==
        MotorStateStatus::Ok);

    std::uint32_t nowUs = 0;
    state.update(feedbackFrame(1, 0, 0), nowUs);
    for (std::uint16_t i = 1; i <= 8; ++i) {
        nowUs += 1000;
        state.update(feedbackFrame(1, i, 0), nowUs);
    }
    assert(state.feedbackTimingStatus() == FeedbackTimingStatus::Mismatch);
}

void testAccumulatesAcrossEncoderWrap()
{
    MotorState state{1, MotorModel::M3508, ControllerModel::C620};
    state.setExpectedFeedbackRate(FeedbackRate::Hz1000);
    state.setMaxRpm(1000.0);
    state.setRotorPositionDegrees(0.0);

    state.update(feedbackFrame(1, 8190, 100), 1000);
    state.update(feedbackFrame(1, 2, 100), 2000);
    const double expectedDegrees = 4.0 * 360.0 / 8192.0;
    assert(near(state.rotorPositionDegrees(), expectedDegrees));
    assert(state.isPositionContinuous());
    assert(state.isPositionAbsolute());
}

void testUnsafeRateAndRuntimeRpmViolation()
{
    MotorState state{1, MotorModel::M3508, ControllerModel::C620};
    state.setExpectedFeedbackRate(FeedbackRate::Hz125);
    state.setMaxRpm(4000.0);
    assert(hasPositionFlag(state.positionFlags(), PositionFlag::UnsafeRate));

    state.setMaxRpm(3000.0);
    assert(!hasPositionFlag(state.positionFlags(), PositionFlag::UnsafeRate));
    state.update(feedbackFrame(1, 0, 3500), 1000);
    assert(hasPositionFlag(
        state.positionFlags(), PositionFlag::RpmExceeded));
}

void testDequeueDelayDoesNotInvalidatePosition()
{
    MotorState state{1, MotorModel::M3508, ControllerModel::C620};
    state.setExpectedFeedbackRate(FeedbackRate::Hz1000);
    state.setMaxRpm(10000.0);
    state.setRotorPositionDegrees(0.0);
    state.update(feedbackFrame(1, 0, 0), 1000);
    state.update(feedbackFrame(1, 100, 0), 5000);
    assert(!hasPositionFlag(
        state.positionFlags(), PositionFlag::AmbiguousDelta));
    assert(state.isPositionContinuous());

    state.notifyFeedbackLoss();
    assert(hasPositionFlag(state.positionFlags(), PositionFlag::FrameGap));

    state.setRotorPositionDegrees(45.0);
    assert(!hasPositionFlag(
        state.positionFlags(), PositionFlag::AmbiguousDelta));
    assert(near(state.rotorPositionDegrees(), 45.0));
    assert(state.feedbackTimingStatus() == FeedbackTimingStatus::Measuring);
    assert(!hasPositionFlag(state.positionFlags(), PositionFlag::FrameGap));
}

void testHalfTurnDeltaIsAlwaysAmbiguous()
{
    MotorState state{1, MotorModel::M3508, ControllerModel::C620};
    state.setExpectedFeedbackRate(FeedbackRate::Hz1000);
    state.setMaxRpm(1000.0);
    state.update(feedbackFrame(1, 0, 0), 1000);
    state.update(feedbackFrame(1, kEncoderCounts / 2U, 0), 2000);
    assert(hasPositionFlag(
        state.positionFlags(), PositionFlag::AmbiguousDelta));
}

void testRejectsInvalidSettings()
{
    MotorState state{1, MotorModel::M3508, ControllerModel::C620};
    assert(state.setExpectedFeedbackRate(static_cast<FeedbackRate>(0)) ==
        MotorStateStatus::InvalidFeedbackRate);
    assert(state.setMaxRpm(-1.0) == MotorStateStatus::InvalidMaxRpm);
    assert(state.setRotorPositionDegrees(
        std::numeric_limits<double>::quiet_NaN()) ==
        MotorStateStatus::InvalidPosition);
}

}  // namespace

int main()
{
    testRejectsInvalidConfigurationAndWrongMotor();
    testInitialFlagsAndExplicitReference();
    testMeasuresAndDetectsFeedbackRate();
    testDetectsEverySupportedFeedbackRate();
    testUnstableFeedbackPeriodIsNotClassified();
    testTimestampWraparound();
    testMedianDetectionSurvivesDequeueJitter();
    testExplicitFeedbackLossInvalidatesPosition();
    testExpectedRateMismatch();
    testAccumulatesAcrossEncoderWrap();
    testUnsafeRateAndRuntimeRpmViolation();
    testDequeueDelayDoesNotInvalidatePosition();
    testHalfTurnDeltaIsAlwaysAmbiguous();
    testRejectsInvalidSettings();
    return 0;
}
