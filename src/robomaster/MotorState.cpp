#include "MotorState.hpp"

#include <algorithm>
#include <cmath>

namespace robomaster {
namespace {

constexpr double kDegreesPerRevolution = 360.0;
constexpr std::uint32_t kRateTolerancePercent = 20U;

bool isValidId(std::uint8_t id)
{
    return id >= 1U && id <= 8U;
}

bool isValidRate(FeedbackRate rate)
{
    return feedbackRateHz(rate) != 0U;
}

MotorStateStatus motorStateStatus(ProtocolStatus status)
{
    switch (status) {
        case ProtocolStatus::Ok:
            return MotorStateStatus::Ok;
        case ProtocolStatus::InvalidId:
            return MotorStateStatus::InvalidId;
        case ProtocolStatus::InvalidCanId:
            return MotorStateStatus::InvalidCanId;
        case ProtocolStatus::InvalidFrameType:
            return MotorStateStatus::InvalidFrameType;
        case ProtocolStatus::InvalidFrameLength:
            return MotorStateStatus::InvalidFrameLength;
        case ProtocolStatus::InvalidEncoder:
            return MotorStateStatus::InvalidEncoder;
        case ProtocolStatus::InvalidModel:
            return MotorStateStatus::InvalidModel;
        case ProtocolStatus::InvalidCurrent:
            break;
    }
    return MotorStateStatus::ProtocolError;
}

std::uint32_t absoluteDifference(std::uint32_t a, std::uint32_t b)
{
    return a > b ? a - b : b - a;
}

bool rateFromPeriod(std::uint32_t periodUs, FeedbackRate &rate)
{
    const FeedbackRate rates[] = {
        FeedbackRate::Hz125,
        FeedbackRate::Hz250,
        FeedbackRate::Hz500,
        FeedbackRate::Hz1000,
    };

    std::uint32_t smallestDifference = UINT32_MAX;
    FeedbackRate closest = FeedbackRate::Hz1000;
    for (const FeedbackRate candidate : rates) {
        const std::uint32_t candidatePeriod = feedbackPeriodUs(candidate);
        const std::uint32_t difference =
            absoluteDifference(periodUs, candidatePeriod);
        if (difference < smallestDifference) {
            smallestDifference = difference;
            closest = candidate;
        }
    }

    const std::uint32_t closestPeriod = feedbackPeriodUs(closest);
    if (static_cast<std::uint64_t>(smallestDifference) * 100U >
        static_cast<std::uint64_t>(closestPeriod) * kRateTolerancePercent) {
        return false;
    }

    rate = closest;
    return true;
}

}  // namespace

MotorState::MotorState(
    std::uint8_t id,
    MotorModel motorModel,
    ControllerModel controllerModel)
    : id_(id),
      motorModel_(motorModel),
      controllerModel_(controllerModel),
      configurationStatus_(MotorStateStatus::Ok)
{
    MotorSpec motorSpec;
    ControllerSpec controllerSpec;
    if (!isValidId(id_)) {
        configurationStatus_ = MotorStateStatus::InvalidId;
    } else if (getMotorSpec(motorModel_, motorSpec) != ProtocolStatus::Ok ||
        getControllerSpec(controllerModel_, controllerSpec) !=
            ProtocolStatus::Ok) {
        configurationStatus_ = MotorStateStatus::InvalidModel;
    }
}

MotorStateStatus MotorState::configurationStatus() const
{
    return configurationStatus_;
}

MotorStateStatus MotorState::update(
    const canbridge::Frame &frame,
    std::uint32_t receivedAtUs)
{
    if (configurationStatus_ != MotorStateStatus::Ok) {
        return configurationStatus_;
    }

    Feedback decoded;
    const ProtocolStatus status =
        decodeFeedback(frame, controllerModel_, decoded);
    if (status != ProtocolStatus::Ok) {
        return motorStateStatus(status);
    }
    if (decoded.id != id_) {
        return MotorStateStatus::InvalidId;
    }

    std::uint32_t intervalUs = 0;
    if (hasFeedback_) {
        intervalUs = receivedAtUs - lastReceivedAtUs_;
        if (intervalUs > 0U) {
            recordInterval(intervalUs);
        }
        updatePosition(decoded);
    } else {
        previousEncoder_ = decoded.encoder;
        hasFeedback_ = true;
        clearPositionFlag(PositionFlag::NoFeedback);
        timingStatus_ = FeedbackTimingStatus::Measuring;
    }

    feedback_ = decoded;
    lastReceivedAtUs_ = receivedAtUs;

    if (hasMaxRpm_) {
        const std::int32_t rpm = static_cast<std::int32_t>(decoded.rpm);
        const double absoluteRpm = static_cast<double>(rpm < 0 ? -rpm : rpm);
        if (absoluteRpm > maxRpm_) {
            setPositionFlag(PositionFlag::RpmExceeded);
        }
    }

    updateTimingEstimate();
    updateRateFlags();
    return MotorStateStatus::Ok;
}

MotorStateStatus MotorState::setExpectedFeedbackRate(FeedbackRate rate)
{
    if (!isValidRate(rate)) {
        return MotorStateStatus::InvalidFeedbackRate;
    }

    expectedRate_ = rate;
    hasExpectedRate_ = true;
    updateTimingEstimate();
    updateRateFlags();
    return MotorStateStatus::Ok;
}

void MotorState::clearExpectedFeedbackRate()
{
    hasExpectedRate_ = false;
    updateTimingEstimate();
    updateRateFlags();
}

void MotorState::notifyFeedbackLoss()
{
    setPositionFlag(PositionFlag::FrameGap);
    resetTimingWindow();
    updateRateFlags();
}

MotorStateStatus MotorState::setMaxRpm(double maxRpm)
{
    if (!std::isfinite(maxRpm) || maxRpm < 0.0) {
        return MotorStateStatus::InvalidMaxRpm;
    }

    maxRpm_ = maxRpm;
    hasMaxRpm_ = true;
    clearPositionFlag(PositionFlag::MaxRpmUnset);
    updateRateFlags();
    return MotorStateStatus::Ok;
}

MotorStateStatus MotorState::setRotorPositionDegrees(double positionDegrees)
{
    if (!std::isfinite(positionDegrees)) {
        return MotorStateStatus::InvalidPosition;
    }

    rotorPositionCounts_ = positionDegrees *
        static_cast<double>(kEncoderCounts) / kDegreesPerRevolution;
    hasPositionReference_ = true;
    clearPositionFlag(PositionFlag::NoReference);
    clearPositionFlag(PositionFlag::RpmExceeded);
    clearPositionFlag(PositionFlag::FrameGap);
    clearPositionFlag(PositionFlag::AmbiguousDelta);
    resetTimingWindow();
    updateRateFlags();
    return MotorStateStatus::Ok;
}

std::uint8_t MotorState::id() const
{
    return id_;
}

MotorModel MotorState::motorModel() const
{
    return motorModel_;
}

ControllerModel MotorState::controllerModel() const
{
    return controllerModel_;
}

bool MotorState::hasFeedback() const
{
    return hasFeedback_;
}

const Feedback &MotorState::feedback() const
{
    return feedback_;
}

std::uint32_t MotorState::lastReceivedAtUs() const
{
    return lastReceivedAtUs_;
}

FeedbackTimingStatus MotorState::feedbackTimingStatus() const
{
    return timingStatus_;
}

double MotorState::observedFeedbackHz() const
{
    if (intervalCount_ == 0U || intervalSumUs_ == 0U) {
        return 0.0;
    }
    return static_cast<double>(intervalCount_) * 1000000.0 /
        static_cast<double>(intervalSumUs_);
}

std::uint32_t MotorState::observedFeedbackPeriodUs() const
{
    if (intervalCount_ == 0U) {
        return 0U;
    }
    return static_cast<std::uint32_t>(
        intervalSumUs_ / static_cast<std::uint64_t>(intervalCount_));
}

bool MotorState::detectedFeedbackRate(FeedbackRate &rate) const
{
    if (!hasDetectedRate_) {
        return false;
    }
    rate = detectedRate_;
    return true;
}

bool MotorState::expectedFeedbackRate(FeedbackRate &rate) const
{
    if (!hasExpectedRate_) {
        return false;
    }
    rate = expectedRate_;
    return true;
}

double MotorState::rotorPositionDegrees() const
{
    return rotorPositionCounts_ * kDegreesPerRevolution /
        static_cast<double>(kEncoderCounts);
}

PositionFlags MotorState::positionFlags() const
{
    return positionFlags_;
}

bool MotorState::isPositionContinuous() const
{
    const PositionFlags invalid =
        positionFlagMask(PositionFlag::NoFeedback) |
        positionFlagMask(PositionFlag::MaxRpmUnset) |
        positionFlagMask(PositionFlag::RateUnknown) |
        positionFlagMask(PositionFlag::UnsafeRate) |
        positionFlagMask(PositionFlag::RpmExceeded) |
        positionFlagMask(PositionFlag::FrameGap) |
        positionFlagMask(PositionFlag::AmbiguousDelta);
    return (positionFlags_ & invalid) == 0U;
}

bool MotorState::isPositionAbsolute() const
{
    return isPositionContinuous() && hasPositionReference_;
}

void MotorState::recordInterval(std::uint32_t intervalUs)
{
    if (intervalCount_ < kTimingWindowSize) {
        intervalsUs_[intervalIndex_] = intervalUs;
        intervalSumUs_ += intervalUs;
        ++intervalCount_;
    } else {
        intervalSumUs_ -= intervalsUs_[intervalIndex_];
        intervalsUs_[intervalIndex_] = intervalUs;
        intervalSumUs_ += intervalUs;
    }
    intervalIndex_ = (intervalIndex_ + 1U) % kTimingWindowSize;
}

void MotorState::resetTimingWindow()
{
    intervalsUs_.fill(0U);
    intervalCount_ = 0U;
    intervalIndex_ = 0U;
    intervalSumUs_ = 0U;
    hasDetectedRate_ = false;
    timingStatus_ = hasFeedback_
        ? FeedbackTimingStatus::Measuring
        : FeedbackTimingStatus::NoData;
}

void MotorState::updateTimingEstimate()
{
    if (!hasFeedback_) {
        timingStatus_ = FeedbackTimingStatus::NoData;
        return;
    }
    if (intervalCount_ < kTimingSamplesRequired) {
        timingStatus_ = FeedbackTimingStatus::Measuring;
        return;
    }

    std::array<std::uint32_t, kTimingWindowSize> sorted{};
    for (std::size_t i = 0; i < intervalCount_; ++i) {
        sorted[i] = intervalsUs_[i];
    }
    std::sort(sorted.begin(), sorted.begin() + intervalCount_);
    const std::uint32_t medianUs = sorted[intervalCount_ / 2U];

    FeedbackRate detected;
    if (!rateFromPeriod(medianUs, detected)) {
        hasDetectedRate_ = false;
        timingStatus_ = FeedbackTimingStatus::Unstable;
        return;
    }

    detectedRate_ = detected;
    hasDetectedRate_ = true;
    timingStatus_ = hasExpectedRate_ && expectedRate_ != detectedRate_
        ? FeedbackTimingStatus::Mismatch
        : FeedbackTimingStatus::Ready;

}

void MotorState::updateRateFlags()
{
    FeedbackRate activeRate;
    if (hasExpectedRate_) {
        activeRate = expectedRate_;
    } else if (hasDetectedRate_) {
        activeRate = detectedRate_;
    } else {
        setPositionFlag(PositionFlag::RateUnknown);
        clearPositionFlag(PositionFlag::UnsafeRate);
        return;
    }

    clearPositionFlag(PositionFlag::RateUnknown);
    if (!hasMaxRpm_) {
        clearPositionFlag(PositionFlag::UnsafeRate);
        return;
    }

    const double maximumUnambiguousRpm =
        static_cast<double>(feedbackRateHz(activeRate)) * 30.0;
    if (maxRpm_ >= maximumUnambiguousRpm) {
        setPositionFlag(PositionFlag::UnsafeRate);
    } else {
        clearPositionFlag(PositionFlag::UnsafeRate);
    }
}

void MotorState::updatePosition(const Feedback &feedback)
{
    std::int32_t delta = static_cast<std::int32_t>(feedback.encoder) -
        static_cast<std::int32_t>(previousEncoder_);
    previousEncoder_ = feedback.encoder;

    if (delta == static_cast<std::int32_t>(kEncoderCounts / 2U) ||
        delta == -static_cast<std::int32_t>(kEncoderCounts / 2U)) {
        setPositionFlag(PositionFlag::AmbiguousDelta);
        return;
    }
    if (delta > static_cast<std::int32_t>(kEncoderCounts / 2U)) {
        delta -= static_cast<std::int32_t>(kEncoderCounts);
    } else if (delta < -static_cast<std::int32_t>(kEncoderCounts / 2U)) {
        delta += static_cast<std::int32_t>(kEncoderCounts);
    }

    rotorPositionCounts_ += static_cast<double>(delta);
}

void MotorState::setPositionFlag(PositionFlag flag)
{
    positionFlags_ |= positionFlagMask(flag);
}

void MotorState::clearPositionFlag(PositionFlag flag)
{
    positionFlags_ &= static_cast<PositionFlags>(~positionFlagMask(flag));
}

}  // namespace robomaster
