#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "Motor.hpp"

namespace robomaster {

enum class MotorBusStatus : std::uint8_t {
    Ok,
    InvalidMotor,
    DuplicateId,
    DuplicateMotor,
    BusFull,
    InvalidCanId,
    UnknownMotor,
    InvalidSendPeriod,
    MotorError,
    ProtocolError,
};

class MotorBus {
public:
    static constexpr std::size_t kMaxMotors = 8;
    static constexpr std::size_t kMaxCommandFrames = 2;

    MotorBus() = default;
    MotorBus(const MotorBus &) = delete;
    MotorBus &operator=(const MotorBus &) = delete;
    MotorBus(MotorBus &&) = delete;
    MotorBus &operator=(MotorBus &&) = delete;

    MotorBusStatus add(Motor &motor);
    Motor *motor(std::uint8_t id);
    const Motor *motor(std::uint8_t id) const;
    std::size_t size() const;

    MotorBusStatus updateFeedback(
        const canbridge::Frame &frame,
        std::uint32_t receivedAtUs);
    void notifyFeedbackLoss();
    MotorBusStatus updateControl(std::uint32_t nowUs);

    MotorBusStatus makeCommandFrames(
        std::array<canbridge::Frame, kMaxCommandFrames> &frames,
        std::size_t &count) const;

    MotorBusStatus setSendPeriodUs(std::uint32_t periodUs);
    std::uint32_t sendPeriodUs() const;
    bool commandFramesDue(std::uint32_t nowUs) const;
    void markCommandFramesSent(std::uint32_t nowUs);

    void emergencyStopAll();
    void clearEmergencyStopAll();
    MotorBusStatus coastAll(std::uint32_t nowUs);

private:
    std::array<Motor *, kMaxMotors> motors_{};
    std::size_t size_ = 0U;
    std::uint32_t sendPeriodUs_ = 1000U;
    std::uint32_t lastSentAtUs_ = 0U;
    bool hasSent_ = false;
};

}  // namespace robomaster
