#include "MotorBus.hpp"

namespace robomaster {

MotorBusStatus MotorBus::add(Motor &motorToAdd)
{
    if (motorToAdd.configurationStatus() != MotorStatus::Ok ||
        motorToAdd.id() < 1U || motorToAdd.id() > 8U) {
        return MotorBusStatus::InvalidMotor;
    }
    for (Motor *registered : motors_) {
        if (registered == &motorToAdd) {
            return MotorBusStatus::DuplicateMotor;
        }
        if (registered != nullptr && registered->id() == motorToAdd.id()) {
            return MotorBusStatus::DuplicateId;
        }
    }
    if (size_ >= kMaxMotors) {
        return MotorBusStatus::BusFull;
    }

    motors_[motorToAdd.id() - 1U] = &motorToAdd;
    ++size_;
    return MotorBusStatus::Ok;
}

Motor *MotorBus::motor(std::uint8_t id)
{
    return id >= 1U && id <= 8U ? motors_[id - 1U] : nullptr;
}

const Motor *MotorBus::motor(std::uint8_t id) const
{
    return id >= 1U && id <= 8U ? motors_[id - 1U] : nullptr;
}

std::size_t MotorBus::size() const
{
    return size_;
}

MotorBusStatus MotorBus::updateFeedback(
    const canbridge::Frame &frame,
    std::uint32_t receivedAtUs)
{
    if (frame.id < kFeedbackIdFirst || frame.id > kFeedbackIdLast) {
        return MotorBusStatus::InvalidCanId;
    }
    const std::uint8_t id = static_cast<std::uint8_t>(
        frame.id - static_cast<std::uint32_t>(kCommandId1To4));
    Motor *target = motor(id);
    if (target == nullptr) {
        return MotorBusStatus::UnknownMotor;
    }
    return target->updateFeedback(frame, receivedAtUs) == MotorStateStatus::Ok
        ? MotorBusStatus::Ok
        : MotorBusStatus::MotorError;
}

void MotorBus::notifyFeedbackLoss()
{
    for (Motor *registered : motors_) {
        if (registered != nullptr) {
            registered->notifyFeedbackLoss();
        }
    }
}

MotorBusStatus MotorBus::updateControl(std::uint32_t nowUs)
{
    MotorBusStatus result = MotorBusStatus::Ok;
    for (Motor *registered : motors_) {
        if (registered == nullptr) {
            continue;
        }
        const MotorStatus status = registered->updateControl(nowUs);
        if (status != MotorStatus::Ok && status != MotorStatus::NoTarget) {
            result = MotorBusStatus::MotorError;
        }
    }
    return result;
}

MotorBusStatus MotorBus::makeCommandFrames(
    std::array<canbridge::Frame, kMaxCommandFrames> &frames,
    std::size_t &count) const
{
    frames = std::array<canbridge::Frame, kMaxCommandFrames>{};
    count = 0U;

    for (std::uint8_t firstId : {std::uint8_t{1U}, std::uint8_t{5U}}) {
        std::array<std::int16_t, 4> currents{};
        bool hasMotor = false;
        for (std::uint8_t offset = 0U; offset < 4U; ++offset) {
            const std::uint8_t id = firstId + offset;
            const Motor *registered = motor(id);
            if (registered == nullptr) {
                continue;
            }
            hasMotor = true;
            if (encodeCurrent(
                    registered->commandCurrent(),
                    registered->controllerModel(),
                    currents[offset]) != ProtocolStatus::Ok) {
                frames = std::array<canbridge::Frame, kMaxCommandFrames>{};
                count = 0U;
                return MotorBusStatus::ProtocolError;
            }
        }

        if (hasMotor) {
            if (makeCommandFrame(firstId, currents, frames[count]) !=
                ProtocolStatus::Ok) {
                frames = std::array<canbridge::Frame, kMaxCommandFrames>{};
                count = 0U;
                return MotorBusStatus::ProtocolError;
            }
            ++count;
        }
    }
    return MotorBusStatus::Ok;
}

MotorBusStatus MotorBus::setSendPeriodUs(std::uint32_t periodUs)
{
    if (periodUs == 0U) {
        return MotorBusStatus::InvalidSendPeriod;
    }
    sendPeriodUs_ = periodUs;
    hasSent_ = false;
    return MotorBusStatus::Ok;
}

std::uint32_t MotorBus::sendPeriodUs() const
{
    return sendPeriodUs_;
}

bool MotorBus::commandFramesDue(std::uint32_t nowUs) const
{
    return !hasSent_ || nowUs - lastSentAtUs_ >= sendPeriodUs_;
}

void MotorBus::markCommandFramesSent(std::uint32_t nowUs)
{
    lastSentAtUs_ = nowUs;
    hasSent_ = true;
}

void MotorBus::emergencyStopAll()
{
    for (Motor *registered : motors_) {
        if (registered != nullptr) {
            registered->emergencyStop();
        }
    }
}

void MotorBus::clearEmergencyStopAll()
{
    for (Motor *registered : motors_) {
        if (registered != nullptr) {
            registered->clearEmergencyStop();
        }
    }
}

MotorBusStatus MotorBus::coastAll(std::uint32_t nowUs)
{
    MotorBusStatus result = MotorBusStatus::Ok;
    for (Motor *registered : motors_) {
        if (registered != nullptr &&
            registered->coast(nowUs) != MotorStatus::Ok) {
            result = MotorBusStatus::MotorError;
        }
    }
    return result;
}

}  // namespace robomaster
