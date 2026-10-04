#include "Protocol.hpp"

#include <cmath>

namespace robomaster {
namespace {

const MotorSpec kM2006Spec = {36.0};
const MotorSpec kM3508Spec = {3591.0 / 187.0};
const ControllerSpec kC610Spec = {10.0, 10000, FeedbackRate::Hz1000};
const ControllerSpec kC620Spec = {20.0, 16384, FeedbackRate::Hz1000};

std::uint16_t readUint16BigEndian(std::uint8_t upper, std::uint8_t lower)
{
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(upper) << 8U) |
        static_cast<std::uint16_t>(lower));
}

std::int16_t readInt16BigEndian(std::uint8_t upper, std::uint8_t lower)
{
    return static_cast<std::int16_t>(readUint16BigEndian(upper, lower));
}

void writeInt16BigEndian(
    std::int16_t value,
    std::array<std::uint8_t, kCanPayloadSize> &data,
    std::size_t offset)
{
    const std::uint16_t raw = static_cast<std::uint16_t>(value);
    data[offset] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
    data[offset + 1U] = static_cast<std::uint8_t>(raw & 0xFFU);
}

bool isValidId(std::uint8_t id)
{
    return id >= 1U && id <= 8U;
}

}  // namespace

ProtocolStatus getMotorSpec(MotorModel model, MotorSpec &spec)
{
    switch (model) {
        case MotorModel::M2006:
            spec = kM2006Spec;
            return ProtocolStatus::Ok;
        case MotorModel::M3508:
            spec = kM3508Spec;
            return ProtocolStatus::Ok;
    }

    spec = MotorSpec{};
    return ProtocolStatus::InvalidModel;
}

ProtocolStatus getControllerSpec(
    ControllerModel model,
    ControllerSpec &spec)
{
    switch (model) {
        case ControllerModel::C610:
            spec = kC610Spec;
            return ProtocolStatus::Ok;
        case ControllerModel::C620:
            spec = kC620Spec;
            return ProtocolStatus::Ok;
    }

    spec = ControllerSpec{};
    return ProtocolStatus::InvalidModel;
}

ProtocolStatus decodeFeedback(
    const canbridge::Frame &frame,
    ControllerModel model,
    Feedback &feedback)
{
    feedback = Feedback{};
    ControllerSpec spec;
    if (getControllerSpec(model, spec) != ProtocolStatus::Ok) {
        return ProtocolStatus::InvalidModel;
    }
    if (frame.extended || frame.remote) {
        return ProtocolStatus::InvalidFrameType;
    }
    if (frame.id < kFeedbackIdFirst || frame.id > kFeedbackIdLast) {
        return ProtocolStatus::InvalidCanId;
    }
    if (frame.length != kCanPayloadSize) {
        return ProtocolStatus::InvalidFrameLength;
    }

    const std::uint16_t encoder = readUint16BigEndian(frame.data[0], frame.data[1]);
    if (encoder >= kEncoderCounts) {
        return ProtocolStatus::InvalidEncoder;
    }

    const std::int16_t currentRaw = readInt16BigEndian(frame.data[4], frame.data[5]);
    feedback.id = static_cast<std::uint8_t>(frame.id - 0x200U);
    feedback.encoder = encoder;
    feedback.rpm = readInt16BigEndian(frame.data[2], frame.data[3]);
    feedback.currentRaw = currentRaw;
    feedback.temperatureC = model == ControllerModel::C620
        ? frame.data[6]
        : 0U;
    feedback.hasTemperature = model == ControllerModel::C620;
    return ProtocolStatus::Ok;
}

ProtocolStatus encodeCurrent(
    double currentA,
    ControllerModel model,
    std::int16_t &currentRaw)
{
    if (!std::isfinite(currentA)) {
        currentRaw = 0;
        return ProtocolStatus::InvalidCurrent;
    }

    ControllerSpec spec;
    if (getControllerSpec(model, spec) != ProtocolStatus::Ok) {
        currentRaw = 0;
        return ProtocolStatus::InvalidModel;
    }
    if (currentA > spec.currentMaxA) {
        currentA = spec.currentMaxA;
    }
    if (currentA < -spec.currentMaxA) {
        currentA = -spec.currentMaxA;
    }

    const double scaled = currentA *
        static_cast<double>(spec.currentFullScale) /
        spec.currentMaxA;
    currentRaw = static_cast<std::int16_t>(std::round(scaled));
    return ProtocolStatus::Ok;
}

ProtocolStatus estimateCurrent(
    std::int16_t currentRaw,
    ControllerModel model,
    double &currentA)
{
    ControllerSpec spec;
    if (getControllerSpec(model, spec) != ProtocolStatus::Ok) {
        currentA = 0.0;
        return ProtocolStatus::InvalidModel;
    }
    currentA = static_cast<double>(currentRaw) * spec.currentMaxA /
        static_cast<double>(spec.currentFullScale);
    return ProtocolStatus::Ok;
}

ProtocolStatus setCommandCurrent(
    canbridge::Frame &frame,
    std::uint8_t id,
    std::int16_t currentRaw)
{
    if (!isValidId(id)) {
        return ProtocolStatus::InvalidId;
    }

    const std::uint32_t expectedId = id <= 4U
        ? kCommandId1To4
        : kCommandId5To8;
    if (frame.extended || frame.remote) {
        return ProtocolStatus::InvalidFrameType;
    }
    if (frame.id != expectedId) {
        return ProtocolStatus::InvalidCanId;
    }
    if (frame.length != kCanPayloadSize) {
        return ProtocolStatus::InvalidFrameLength;
    }

    const std::size_t slot = static_cast<std::size_t>((id - 1U) % 4U);
    writeInt16BigEndian(currentRaw, frame.data, slot * 2U);
    return ProtocolStatus::Ok;
}

ProtocolStatus makeCommandFrame(
    std::uint8_t firstId,
    const std::array<std::int16_t, 4> &currents,
    canbridge::Frame &frame)
{
    if (firstId != 1U && firstId != 5U) {
        return ProtocolStatus::InvalidId;
    }

    frame = canbridge::Frame{};
    frame.id = firstId == 1U ? kCommandId1To4 : kCommandId5To8;
    frame.length = static_cast<std::uint8_t>(kCanPayloadSize);

    for (std::size_t slot = 0; slot < currents.size(); ++slot) {
        writeInt16BigEndian(currents[slot], frame.data, slot * 2U);
    }
    return ProtocolStatus::Ok;
}

}  // namespace robomaster
