#pragma once

#include <cstdint>

#include "Control.hpp"
#include "MotorState.hpp"
#include "Pid.hpp"

namespace robomaster {

enum class RotationDirection : std::int8_t {
    Forward = 1,
    Reverse = -1,
};

enum class MotorStatus : std::uint8_t {
    Ok,
    NoTarget,
    InvalidConfiguration,
    InvalidGearRatio,
    InvalidDirection,
    InvalidMaxSpeed,
    InvalidTrackingMaxSpeed,
    InvalidMaxCurrent,
    InvalidControlPeriod,
    InvalidTimeout,
    InvalidFeedbackRate,
    InvalidPidGains,
    InvalidControlType,
    InvalidTarget,
    InvalidFeedback,
    TargetOutOfRange,
    FeedbackUnavailable,
    FeedbackRateUnavailable,
    FeedbackTimeout,
    PositionUnreliable,
    CommandTimeout,
    EmergencyStopped,
    PidError,
};

class Motor {
public:
    Motor(
        std::uint8_t id,
        MotorModel motorModel,
        ControllerModel controllerModel);
    Motor(const Motor &) = delete;
    Motor &operator=(const Motor &) = delete;
    Motor(Motor &&) = delete;
    Motor &operator=(Motor &&) = delete;

    MotorStatus configurationStatus() const;
    MotorStatus status() const;
    MotorStateStatus updateFeedback(
        const CanFrame &frame,
        std::uint32_t receivedAtUs);
    void notifyFeedbackLoss();
    MotorStatus updateControl(std::uint32_t nowUs);

    MotorStatus setExternalGearRatio(double ratio);
    MotorStatus setDirection(RotationDirection direction);
    MotorStatus setMaxSpeed(double rpm);
    MotorStatus setTrackingMaxSpeed(double rpm);
    MotorStatus setMaxCurrent(double currentA);
    MotorStatus setPositionControlPeriodUs(std::uint32_t periodUs);
    MotorStatus setSpeedControlPeriodUs(std::uint32_t periodUs);
    MotorStatus setCommandTimeoutUs(std::uint32_t timeoutUs);
    MotorStatus setFeedbackTimeoutPeriods(std::uint8_t periods);
    MotorStatus setExpectedFeedbackRate(FeedbackRate rate);
    void clearExpectedFeedbackRate();
    MotorStatus setPositionGains(const PidGains &gains);
    MotorStatus setSpeedGains(const PidGains &gains);

    MotorStatus setTarget(
        ControlType type,
        double value,
        std::uint32_t nowUs);
    MotorStatus setTarget(const ControlTarget &target, std::uint32_t nowUs);
    MotorStatus coast(std::uint32_t nowUs);
    MotorStatus brake(std::uint32_t nowUs);
    MotorStatus hold(std::uint32_t nowUs);
    void emergencyStop();
    void clearEmergencyStop();
    MotorStatus setPosition(double positionDegrees);

    std::uint8_t id() const;
    MotorModel motorModel() const;
    ControllerModel controllerModel() const;
    double externalGearRatio() const;
    RotationDirection direction() const;
    double maxSpeed() const;
    double trackingMaxSpeed() const;
    double maxCurrent() const;
    double position() const;
    double speed() const;
    double estimatedCurrent() const;
    double commandCurrent() const;
    bool hasTarget() const;
    const ControlTarget &target() const;
    bool emergencyStopped() const;
    const MotorState &state() const;

private:
    MotorStatus failSafe(MotorStatus status);
    MotorStatus validateTarget(const ControlTarget &target) const;
    MotorStatus validateFeedbackForTarget(
        const ControlTarget &target,
        std::uint32_t nowUs) const;
    bool feedbackPeriod(std::uint32_t &periodUs) const;
    bool controlDue(
        std::uint32_t nowUs,
        std::uint32_t periodUs,
        bool &hasRun,
        std::uint32_t &lastRunUs,
        double &dtSeconds);
    void resetControlState();
    void updateStateTrackingMaxRpm();
    double directionSign() const;
    double totalGearRatio() const;

    MotorState state_;
    MotorStatus configurationStatus_ = MotorStatus::Ok;
    MotorStatus status_ = MotorStatus::NoTarget;
    MotorSpec motorSpec_{};
    ControllerSpec controllerSpec_{};

    double externalGearRatio_ = 1.0;
    RotationDirection direction_ = RotationDirection::Forward;
    bool hasMaxSpeed_ = false;
    double maxSpeedRpm_ = 0.0;
    bool hasTrackingMaxSpeed_ = false;
    double trackingMaxSpeedRpm_ = 0.0;
    bool hasMaxCurrent_ = false;
    double maxCurrentA_ = 0.0;

    Pid positionPid_{};
    Pid speedPid_{};
    Pid brakePid_{};
    std::uint32_t positionPeriodUs_ = 4000U;
    std::uint32_t speedPeriodUs_ = 1000U;
    bool hasPositionRun_ = false;
    bool hasSpeedRun_ = false;
    std::uint32_t lastPositionRunUs_ = 0U;
    std::uint32_t lastSpeedRunUs_ = 0U;
    double speedTargetRpm_ = 0.0;
    bool braking_ = false;

    ControlTarget target_{ControlType::Current, 0.0};
    bool hasTarget_ = false;
    std::uint32_t targetUpdatedAtUs_ = 0U;
    std::uint32_t commandTimeoutUs_ = 0U;
    std::uint8_t feedbackTimeoutPeriods_ = 3U;
    bool emergencyStopped_ = false;
    double commandCurrentA_ = 0.0;
};

}  // namespace robomaster
