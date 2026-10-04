#pragma once

#include <CANBridge.h>

#include <array>
#include <cstddef>
#include <cstdint>

namespace robomaster {

constexpr std::size_t kCanPayloadSize = 8;
constexpr std::uint16_t kCommandId1To4 = 0x200;
constexpr std::uint16_t kCommandId5To8 = 0x1FF;
constexpr std::uint16_t kFeedbackIdFirst = 0x201;
constexpr std::uint16_t kFeedbackIdLast = 0x208;
constexpr std::uint16_t kEncoderCounts = 8192;

enum class MotorModel : std::uint8_t {
    M2006,
    M3508,
};

enum class ControllerModel : std::uint8_t {
    C610,
    C620,
};

enum class FeedbackRate : std::uint16_t {
    Hz125 = 125,
    Hz250 = 250,
    Hz500 = 500,
    Hz1000 = 1000,
};

constexpr std::uint16_t feedbackRateHz(FeedbackRate rate)
{
    return rate == FeedbackRate::Hz125 ? 125U :
        rate == FeedbackRate::Hz250 ? 250U :
        rate == FeedbackRate::Hz500 ? 500U :
        rate == FeedbackRate::Hz1000 ? 1000U : 0U;
}

constexpr std::uint32_t feedbackPeriodUs(FeedbackRate rate)
{
    return rate == FeedbackRate::Hz125 ? 8000U :
        rate == FeedbackRate::Hz250 ? 4000U :
        rate == FeedbackRate::Hz500 ? 2000U :
        rate == FeedbackRate::Hz1000 ? 1000U : 0U;
}

struct MotorSpec {
    double gearRatio;
};

struct ControllerSpec {
    double currentMaxA;
    std::int16_t currentFullScale;
    FeedbackRate defaultFeedbackRate;
};

struct Feedback {
    // Speed controller ID, 1-8.
    std::uint8_t id = 0;

    // Rotor mechanical angle, 0-8191 counts per revolution.
    std::uint16_t encoder = 0;

    // Rotor rotational speed in rpm.
    std::int16_t rpm = 0;

    // Actual torque current in the controller protocol representation.
    std::int16_t currentRaw = 0;

    // Motor temperature in degrees Celsius.
    std::uint8_t temperatureC = 0;

    // C620 reports motor temperature. C610 reserves this field.
    bool hasTemperature = false;
};

enum class ProtocolStatus : std::uint8_t {
    Ok,
    InvalidId,
    InvalidCanId,
    InvalidFrameType,
    InvalidFrameLength,
    InvalidEncoder,
    InvalidCurrent,
    InvalidModel,
};

ProtocolStatus getMotorSpec(MotorModel model, MotorSpec &spec);
ProtocolStatus getControllerSpec(
    ControllerModel model,
    ControllerSpec &spec);

ProtocolStatus decodeFeedback(
    const canbridge::Frame &frame,
    ControllerModel model,
    Feedback &feedback);

ProtocolStatus encodeCurrent(
    double currentA,
    ControllerModel model,
    std::int16_t &currentRaw);

// This is an estimate. It assumes the feedback value uses the same
// full-scale mapping as the current command.
ProtocolStatus estimateCurrent(
    std::int16_t currentRaw,
    ControllerModel model,
    double &currentA);

ProtocolStatus setCommandCurrent(
    canbridge::Frame &frame,
    std::uint8_t id,
    std::int16_t currentRaw);

ProtocolStatus makeCommandFrame(
    std::uint8_t firstId,
    const std::array<std::int16_t, 4> &currents,
    canbridge::Frame &frame);

}  // namespace robomaster
