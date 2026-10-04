#include "Motor.hpp"

#include <cmath>

namespace robomaster {
namespace {

bool isValidControlType(ControlType type)
{
    return type == ControlType::Current ||
        type == ControlType::Speed ||
        type == ControlType::Position;
}

bool isValidDirection(RotationDirection direction)
{
    return direction == RotationDirection::Forward ||
        direction == RotationDirection::Reverse;
}

MotorStatus pidStatus(PidStatus status)
{
    return status == PidStatus::Ok
        ? MotorStatus::Ok
        : MotorStatus::InvalidPidGains;
}

}  // namespace

Motor::Motor(
    std::uint8_t id,
    MotorModel motorModel,
    ControllerModel controllerModel)
    : state_(id, motorModel, controllerModel)
{
    if (state_.configurationStatus() != MotorStateStatus::Ok ||
        getMotorSpec(motorModel, motorSpec_) != ProtocolStatus::Ok ||
        getControllerSpec(controllerModel, controllerSpec_) !=
            ProtocolStatus::Ok) {
        configurationStatus_ = MotorStatus::InvalidConfiguration;
        status_ = configurationStatus_;
    }
}

MotorStatus Motor::configurationStatus() const
{
    return configurationStatus_;
}

MotorStatus Motor::status() const
{
    return status_;
}

MotorStateStatus Motor::updateFeedback(
    const canbridge::Frame &frame,
    std::uint32_t receivedAtUs)
{
    const MotorStateStatus result = state_.update(frame, receivedAtUs);
    if (result != MotorStateStatus::Ok) {
        failSafe(MotorStatus::InvalidFeedback);
    }
    return result;
}

void Motor::notifyFeedbackLoss()
{
    state_.notifyFeedbackLoss();
    failSafe(MotorStatus::PositionUnreliable);
}

MotorStatus Motor::updateControl(std::uint32_t nowUs)
{
    if (configurationStatus_ != MotorStatus::Ok) {
        return failSafe(configurationStatus_);
    }
    if (emergencyStopped_) {
        return failSafe(MotorStatus::EmergencyStopped);
    }
    if (!hasTarget_) {
        commandCurrentA_ = 0.0;
        status_ = MotorStatus::NoTarget;
        return status_;
    }
    const bool isSafeCoast =
        target_.type == ControlType::Current && target_.value == 0.0;
    if (!isSafeCoast && commandTimeoutUs_ > 0U &&
        nowUs - targetUpdatedAtUs_ > commandTimeoutUs_) {
        return failSafe(MotorStatus::CommandTimeout);
    }

    const MotorStatus feedbackStatus =
        validateFeedbackForTarget(target_, nowUs);
    if (feedbackStatus != MotorStatus::Ok) {
        return failSafe(feedbackStatus);
    }

    if (target_.type == ControlType::Current) {
        commandCurrentA_ = target_.value * directionSign();
        status_ = MotorStatus::Ok;
        return status_;
    }

    if (target_.type == ControlType::Position) {
        double dtSeconds = 0.0;
        if (controlDue(
                nowUs,
                positionPeriodUs_,
                hasPositionRun_,
                lastPositionRunUs_,
                dtSeconds)) {
            const PidResult result =
                positionPid_.update(target_.value, position(), dtSeconds);
            if (result.status != PidStatus::Ok) {
                return failSafe(MotorStatus::PidError);
            }
            speedTargetRpm_ = result.output;
        }
    } else {
        speedTargetRpm_ = target_.value;
    }

    double dtSeconds = 0.0;
    if (controlDue(
            nowUs,
            speedPeriodUs_,
            hasSpeedRun_,
            lastSpeedRunUs_,
            dtSeconds)) {
        const PidResult result = braking_
            ? brakePid_.update(0.0, speed(), dtSeconds)
            : speedPid_.update(speedTargetRpm_, speed(), dtSeconds);
        if (result.status != PidStatus::Ok) {
            return failSafe(MotorStatus::PidError);
        }
        commandCurrentA_ = result.output * directionSign();
    }

    status_ = MotorStatus::Ok;
    return status_;
}

MotorStatus Motor::setExternalGearRatio(double ratio)
{
    if (!std::isfinite(ratio) || ratio <= 0.0) {
        return MotorStatus::InvalidGearRatio;
    }
    if (state_.hasFeedback() || hasTarget_) {
        return MotorStatus::InvalidConfiguration;
    }

    externalGearRatio_ = ratio;
    updateStateTrackingMaxRpm();
    return MotorStatus::Ok;
}

MotorStatus Motor::setDirection(RotationDirection direction)
{
    if (!isValidDirection(direction)) {
        return MotorStatus::InvalidDirection;
    }
    if (state_.hasFeedback() || hasTarget_) {
        return MotorStatus::InvalidConfiguration;
    }

    direction_ = direction;
    return MotorStatus::Ok;
}

MotorStatus Motor::setMaxSpeed(double rpm)
{
    if (!std::isfinite(rpm) || rpm < 0.0) {
        return MotorStatus::InvalidMaxSpeed;
    }

    maxSpeedRpm_ = rpm;
    hasMaxSpeed_ = true;
    if (positionPid_.setLimits(PidLimits{-rpm, rpm}) != PidStatus::Ok) {
        return MotorStatus::InvalidMaxSpeed;
    }
    if (hasTarget_ && target_.type == ControlType::Speed &&
        std::fabs(target_.value) > maxSpeedRpm_) {
        return failSafe(MotorStatus::TargetOutOfRange);
    }
    return MotorStatus::Ok;
}

MotorStatus Motor::setTrackingMaxSpeed(double rpm)
{
    if (!std::isfinite(rpm) || rpm <= 0.0) {
        return MotorStatus::InvalidTrackingMaxSpeed;
    }

    trackingMaxSpeedRpm_ = rpm;
    hasTrackingMaxSpeed_ = true;
    updateStateTrackingMaxRpm();
    return configurationStatus_ == MotorStatus::Ok
        ? MotorStatus::Ok
        : MotorStatus::InvalidTrackingMaxSpeed;
}

MotorStatus Motor::setMaxCurrent(double currentA)
{
    if (!std::isfinite(currentA) || currentA < 0.0 ||
        currentA > controllerSpec_.currentMaxA) {
        return MotorStatus::InvalidMaxCurrent;
    }

    maxCurrentA_ = currentA;
    hasMaxCurrent_ = true;
    if (speedPid_.setLimits(PidLimits{-currentA, currentA}) != PidStatus::Ok) {
        return MotorStatus::InvalidMaxCurrent;
    }
    if (brakePid_.setLimits(PidLimits{-currentA, currentA}) != PidStatus::Ok) {
        return MotorStatus::InvalidMaxCurrent;
    }
    if (std::fabs(commandCurrentA_) > maxCurrentA_) {
        commandCurrentA_ = commandCurrentA_ < 0.0
            ? -maxCurrentA_
            : maxCurrentA_;
    }
    if (hasTarget_ && target_.type == ControlType::Current &&
        std::fabs(target_.value) > maxCurrentA_) {
        return failSafe(MotorStatus::TargetOutOfRange);
    }
    return MotorStatus::Ok;
}

MotorStatus Motor::setPositionControlPeriodUs(std::uint32_t periodUs)
{
    if (periodUs == 0U) {
        return MotorStatus::InvalidControlPeriod;
    }
    positionPeriodUs_ = periodUs;
    hasPositionRun_ = false;
    return MotorStatus::Ok;
}

MotorStatus Motor::setSpeedControlPeriodUs(std::uint32_t periodUs)
{
    if (periodUs == 0U) {
        return MotorStatus::InvalidControlPeriod;
    }
    speedPeriodUs_ = periodUs;
    hasSpeedRun_ = false;
    return MotorStatus::Ok;
}

MotorStatus Motor::setCommandTimeoutUs(std::uint32_t timeoutUs)
{
    commandTimeoutUs_ = timeoutUs;
    return MotorStatus::Ok;
}

MotorStatus Motor::setFeedbackTimeoutPeriods(std::uint8_t periods)
{
    if (periods == 0U) {
        return MotorStatus::InvalidTimeout;
    }
    feedbackTimeoutPeriods_ = periods;
    return MotorStatus::Ok;
}

MotorStatus Motor::setExpectedFeedbackRate(FeedbackRate rate)
{
    return state_.setExpectedFeedbackRate(rate) == MotorStateStatus::Ok
        ? MotorStatus::Ok
        : MotorStatus::InvalidFeedbackRate;
}

void Motor::clearExpectedFeedbackRate()
{
    state_.clearExpectedFeedbackRate();
}

MotorStatus Motor::setPositionGains(const PidGains &gains)
{
    return pidStatus(positionPid_.setGains(gains));
}

MotorStatus Motor::setSpeedGains(const PidGains &gains)
{
    const PidStatus speedStatus = speedPid_.setGains(gains);
    if (speedStatus != PidStatus::Ok) {
        return pidStatus(speedStatus);
    }
    return pidStatus(brakePid_.setGains(PidGains{gains.kp, 0.0, 0.0}));
}

MotorStatus Motor::setTarget(
    ControlType type,
    double value,
    std::uint32_t nowUs)
{
    return setTarget(ControlTarget{type, value}, nowUs);
}

MotorStatus Motor::setTarget(
    const ControlTarget &target,
    std::uint32_t nowUs)
{
    if (emergencyStopped_) {
        return failSafe(MotorStatus::EmergencyStopped);
    }

    const MotorStatus targetStatus = validateTarget(target);
    if (targetStatus != MotorStatus::Ok) {
        return failSafe(targetStatus);
    }
    const MotorStatus feedbackStatus = validateFeedbackForTarget(target, nowUs);
    if (feedbackStatus != MotorStatus::Ok) {
        return failSafe(feedbackStatus);
    }

    if (!hasTarget_ || target_.type != target.type) {
        resetControlState();
    }
    braking_ = false;
    target_ = target;
    hasTarget_ = true;
    targetUpdatedAtUs_ = nowUs;
    status_ = MotorStatus::Ok;
    return status_;
}

MotorStatus Motor::coast(std::uint32_t nowUs)
{
    return setTarget(ControlType::Current, 0.0, nowUs);
}

MotorStatus Motor::brake(std::uint32_t nowUs)
{
    resetControlState();
    const MotorStatus status = setTarget(ControlType::Speed, 0.0, nowUs);
    if (status == MotorStatus::Ok) {
        braking_ = true;
    }
    return status;
}

MotorStatus Motor::hold(std::uint32_t nowUs)
{
    if (!state_.isPositionContinuous()) {
        return failSafe(MotorStatus::PositionUnreliable);
    }
    const double currentPosition = position();
    resetControlState();
    return setTarget(ControlType::Position, currentPosition, nowUs);
}

void Motor::emergencyStop()
{
    emergencyStopped_ = true;
    failSafe(MotorStatus::EmergencyStopped);
}

void Motor::clearEmergencyStop()
{
    emergencyStopped_ = false;
    hasTarget_ = false;
    commandCurrentA_ = 0.0;
    resetControlState();
    status_ = MotorStatus::NoTarget;
}

MotorStatus Motor::setPosition(double positionDegrees)
{
    if (!std::isfinite(positionDegrees)) {
        return MotorStatus::InvalidTarget;
    }
    const double rotorDegrees =
        positionDegrees * totalGearRatio() * directionSign();
    if (state_.setRotorPositionDegrees(rotorDegrees) !=
        MotorStateStatus::Ok) {
        return MotorStatus::InvalidTarget;
    }
    resetControlState();
    return MotorStatus::Ok;
}

std::uint8_t Motor::id() const
{
    return state_.id();
}

MotorModel Motor::motorModel() const
{
    return state_.motorModel();
}

ControllerModel Motor::controllerModel() const
{
    return state_.controllerModel();
}

double Motor::externalGearRatio() const
{
    return externalGearRatio_;
}

RotationDirection Motor::direction() const
{
    return direction_;
}

double Motor::maxSpeed() const
{
    return maxSpeedRpm_;
}

double Motor::trackingMaxSpeed() const
{
    return trackingMaxSpeedRpm_;
}

double Motor::maxCurrent() const
{
    return maxCurrentA_;
}

double Motor::position() const
{
    return directionSign() * state_.rotorPositionDegrees() /
        totalGearRatio();
}

double Motor::speed() const
{
    return directionSign() * static_cast<double>(state_.feedback().rpm) /
        totalGearRatio();
}

double Motor::estimatedCurrent() const
{
    double currentA = 0.0;
    if (estimateCurrent(
            state_.feedback().currentRaw,
            controllerModel(),
            currentA) != ProtocolStatus::Ok) {
        return 0.0;
    }
    return directionSign() * currentA;
}

double Motor::commandCurrent() const
{
    return commandCurrentA_;
}

bool Motor::hasTarget() const
{
    return hasTarget_;
}

const ControlTarget &Motor::target() const
{
    return target_;
}

bool Motor::emergencyStopped() const
{
    return emergencyStopped_;
}

const MotorState &Motor::state() const
{
    return state_;
}

MotorStatus Motor::failSafe(MotorStatus status)
{
    commandCurrentA_ = 0.0;
    hasTarget_ = false;
    target_ = ControlTarget{ControlType::Current, 0.0};
    resetControlState();
    status_ = status;
    return status_;
}

MotorStatus Motor::validateTarget(const ControlTarget &target) const
{
    if (!isValidControlType(target.type)) {
        return MotorStatus::InvalidControlType;
    }
    if (!std::isfinite(target.value)) {
        return MotorStatus::InvalidTarget;
    }
    if (target.type == ControlType::Current) {
        if (!hasMaxCurrent_ && target.value != 0.0) {
            return MotorStatus::InvalidConfiguration;
        }
        if (hasMaxCurrent_ && std::fabs(target.value) > maxCurrentA_) {
            return MotorStatus::TargetOutOfRange;
        }
    }
    if (target.type == ControlType::Speed) {
        if (!hasMaxSpeed_ || !hasMaxCurrent_) {
            return MotorStatus::InvalidConfiguration;
        }
        if (std::fabs(target.value) > maxSpeedRpm_) {
            return MotorStatus::TargetOutOfRange;
        }
    }
    if (target.type == ControlType::Position &&
        (!hasMaxSpeed_ || !hasMaxCurrent_)) {
        return MotorStatus::InvalidConfiguration;
    }
    return MotorStatus::Ok;
}

MotorStatus Motor::validateFeedbackForTarget(
    const ControlTarget &target,
    std::uint32_t nowUs) const
{
    if (target.type == ControlType::Current && target.value == 0.0) {
        return MotorStatus::Ok;
    }
    if (!state_.hasFeedback()) {
        return MotorStatus::FeedbackUnavailable;
    }

    std::uint32_t periodUs = 0U;
    if (!feedbackPeriod(periodUs)) {
        return MotorStatus::FeedbackRateUnavailable;
    }
    const std::uint64_t timeoutUs =
        static_cast<std::uint64_t>(periodUs) * feedbackTimeoutPeriods_;
    if (static_cast<std::uint64_t>(nowUs - state_.lastReceivedAtUs()) >
        timeoutUs) {
        return MotorStatus::FeedbackTimeout;
    }
    if (target.type == ControlType::Position &&
        !state_.isPositionContinuous()) {
        return MotorStatus::PositionUnreliable;
    }
    return MotorStatus::Ok;
}

bool Motor::feedbackPeriod(std::uint32_t &periodUs) const
{
    FeedbackRate rate;
    if (state_.expectedFeedbackRate(rate) ||
        state_.detectedFeedbackRate(rate)) {
        periodUs = feedbackPeriodUs(rate);
        return periodUs > 0U;
    }
    return false;
}

bool Motor::controlDue(
    std::uint32_t nowUs,
    std::uint32_t periodUs,
    bool &hasRun,
    std::uint32_t &lastRunUs,
    double &dtSeconds)
{
    if (!hasRun) {
        hasRun = true;
        lastRunUs = nowUs;
        dtSeconds = static_cast<double>(periodUs) / 1000000.0;
        return true;
    }

    const std::uint32_t elapsedUs = nowUs - lastRunUs;
    if (elapsedUs < periodUs) {
        return false;
    }
    lastRunUs = nowUs;
    dtSeconds = static_cast<double>(elapsedUs) / 1000000.0;
    return true;
}

void Motor::resetControlState()
{
    positionPid_.reset();
    speedPid_.reset();
    brakePid_.reset();
    hasPositionRun_ = false;
    hasSpeedRun_ = false;
    speedTargetRpm_ = 0.0;
    braking_ = false;
}

void Motor::updateStateTrackingMaxRpm()
{
    if (!hasTrackingMaxSpeed_) {
        return;
    }
    const double rotorMaxRpm = trackingMaxSpeedRpm_ * totalGearRatio();
    if (state_.setMaxRpm(rotorMaxRpm) != MotorStateStatus::Ok) {
        configurationStatus_ = MotorStatus::InvalidTrackingMaxSpeed;
    }
}

double Motor::directionSign() const
{
    return static_cast<double>(static_cast<std::int8_t>(direction_));
}

double Motor::totalGearRatio() const
{
    return motorSpec_.gearRatio * externalGearRatio_;
}

}  // namespace robomaster
