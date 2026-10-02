#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "Protocol.hpp"

namespace robomaster {

enum class FeedbackTimingStatus : std::uint8_t {
    NoData,
    Measuring,
    Ready,
    Mismatch,
    Unstable,
};

enum class MotorStateStatus : std::uint8_t {
    Ok,
    InvalidId,
    InvalidModel,
    InvalidCanId,
    InvalidFrameType,
    InvalidFrameLength,
    InvalidEncoder,
    InvalidFeedbackRate,
    InvalidMaxRpm,
    InvalidPosition,
    ProtocolError,
};

enum class PositionFlag : std::uint8_t {
    NoFeedback = 1U << 0U,
    NoReference = 1U << 1U,
    MaxRpmUnset = 1U << 2U,
    RateUnknown = 1U << 3U,
    UnsafeRate = 1U << 4U,
    RpmExceeded = 1U << 5U,
    FrameGap = 1U << 6U,
    AmbiguousDelta = 1U << 7U,
};

using PositionFlags = std::uint8_t;

constexpr PositionFlags positionFlagMask(PositionFlag flag)
{
    return static_cast<PositionFlags>(flag);
}

constexpr bool hasPositionFlag(PositionFlags flags, PositionFlag flag)
{
    return (flags & positionFlagMask(flag)) != 0U;
}

class MotorState {
public:
    static constexpr std::size_t kTimingWindowSize = 16;
    static constexpr std::size_t kTimingSamplesRequired = 8;

    MotorState(
        std::uint8_t id,
        MotorModel motorModel,
        ControllerModel controllerModel);

    MotorStateStatus configurationStatus() const;
    MotorStateStatus update(const CanFrame &frame, std::uint32_t receivedAtUs);

    MotorStateStatus setExpectedFeedbackRate(FeedbackRate rate);
    void clearExpectedFeedbackRate();
    void notifyFeedbackLoss();
    MotorStateStatus setMaxRpm(double maxRpm);
    MotorStateStatus setRotorPositionDegrees(double positionDegrees);

    std::uint8_t id() const;
    MotorModel motorModel() const;
    ControllerModel controllerModel() const;
    bool hasFeedback() const;
    const Feedback &feedback() const;
    std::uint32_t lastReceivedAtUs() const;

    FeedbackTimingStatus feedbackTimingStatus() const;
    double observedFeedbackHz() const;
    std::uint32_t observedFeedbackPeriodUs() const;
    bool detectedFeedbackRate(FeedbackRate &rate) const;
    bool expectedFeedbackRate(FeedbackRate &rate) const;

    double rotorPositionDegrees() const;
    PositionFlags positionFlags() const;
    bool isPositionContinuous() const;
    bool isPositionAbsolute() const;

private:
    void recordInterval(std::uint32_t intervalUs);
    void resetTimingWindow();
    void updateTimingEstimate();
    void updateRateFlags();
    void updatePosition(const Feedback &feedback);
    void setPositionFlag(PositionFlag flag);
    void clearPositionFlag(PositionFlag flag);

    std::uint8_t id_;
    MotorModel motorModel_;
    ControllerModel controllerModel_;
    MotorStateStatus configurationStatus_;

    Feedback feedback_{};
    bool hasFeedback_ = false;
    std::uint16_t previousEncoder_ = 0;
    std::uint32_t lastReceivedAtUs_ = 0;
    double rotorPositionCounts_ = 0.0;

    bool hasExpectedRate_ = false;
    FeedbackRate expectedRate_ = FeedbackRate::Hz1000;
    bool hasDetectedRate_ = false;
    FeedbackRate detectedRate_ = FeedbackRate::Hz1000;
    FeedbackTimingStatus timingStatus_ = FeedbackTimingStatus::NoData;

    std::array<std::uint32_t, kTimingWindowSize> intervalsUs_{};
    std::size_t intervalCount_ = 0;
    std::size_t intervalIndex_ = 0;
    std::uint64_t intervalSumUs_ = 0;

    bool hasMaxRpm_ = false;
    double maxRpm_ = 0.0;
    bool hasPositionReference_ = false;
    PositionFlags positionFlags_ =
        positionFlagMask(PositionFlag::NoFeedback) |
        positionFlagMask(PositionFlag::NoReference) |
        positionFlagMask(PositionFlag::MaxRpmUnset) |
        positionFlagMask(PositionFlag::RateUnknown);
};

}  // namespace robomaster
