#include "Pid.hpp"

#include <cmath>

namespace robomaster {
namespace {

double clamp(double value, double minimum, double maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

bool isFinite(const PidGains &gains)
{
    return std::isfinite(gains.kp) &&
        std::isfinite(gains.ki) &&
        std::isfinite(gains.kd);
}

bool isValid(const PidLimits &limits)
{
    return std::isfinite(limits.minimum) &&
        std::isfinite(limits.maximum) &&
        limits.minimum <= 0.0 && limits.maximum >= 0.0;
}

bool isValid(DerivativeSource source)
{
    return source == DerivativeSource::Measurement ||
        source == DerivativeSource::Error;
}

}  // namespace

Pid::Pid(const PidConfig &config)
{
    configure(config);
}

PidStatus Pid::configure(const PidConfig &config)
{
    if (!isFinite(config.gains)) {
        config_ = PidConfig{};
        reset();
        configurationStatus_ = PidStatus::InvalidGains;
        return PidStatus::InvalidGains;
    }
    if (!isValid(config.limits)) {
        config_ = PidConfig{};
        reset();
        configurationStatus_ = PidStatus::InvalidLimits;
        return PidStatus::InvalidLimits;
    }
    if (!isValid(config.derivativeSource)) {
        config_ = PidConfig{};
        reset();
        configurationStatus_ = PidStatus::InvalidDerivativeSource;
        return PidStatus::InvalidDerivativeSource;
    }

    config_ = config;
    reset();
    configurationStatus_ = PidStatus::Ok;
    return PidStatus::Ok;
}

PidStatus Pid::setGains(const PidGains &gains)
{
    if (!isFinite(gains)) {
        return PidStatus::InvalidGains;
    }

    config_.gains = gains;
    return PidStatus::Ok;
}

PidStatus Pid::setLimits(const PidLimits &limits)
{
    if (!isValid(limits)) {
        return PidStatus::InvalidLimits;
    }

    config_.limits = limits;
    integralOutput_ = clamp(
        integralOutput_,
        limits.minimum,
        limits.maximum);
    lastOutput_ = clamp(lastOutput_, limits.minimum, limits.maximum);
    return PidStatus::Ok;
}

PidResult Pid::update(double target, double measured, double dtSeconds)
{
    if (configurationStatus_ != PidStatus::Ok) {
        lastOutput_ = 0.0;
        return PidResult{0.0, configurationStatus_, false};
    }
    if (!std::isfinite(target) || !std::isfinite(measured)) {
        lastOutput_ = 0.0;
        hasPreviousError_ = false;
        return PidResult{0.0, PidStatus::InvalidInput, false};
    }
    if (!std::isfinite(dtSeconds) || dtSeconds <= 0.0) {
        lastOutput_ = 0.0;
        hasPreviousError_ = false;
        return PidResult{0.0, PidStatus::InvalidTimeStep, false};
    }

    const double error = target - measured;
    if (!std::isfinite(error)) {
        lastOutput_ = 0.0;
        hasPreviousError_ = false;
        return PidResult{0.0, PidStatus::CalculationOverflow, false};
    }

    const double proportional = config_.gains.kp * error;
    double derivative = 0.0;
    if (hasPreviousError_) {
        const double change =
            config_.derivativeSource == DerivativeSource::Measurement
            ? -(measured - previousMeasured_)
            : error - previousError_;
        derivative = config_.gains.kd * change / dtSeconds;
    }
    const double baseOutput = proportional + derivative;

    const double integralDelta = config_.gains.ki * error * dtSeconds;
    if (!std::isfinite(proportional) || !std::isfinite(derivative) ||
        !std::isfinite(baseOutput) || !std::isfinite(integralDelta)) {
        lastOutput_ = 0.0;
        hasPreviousError_ = false;
        return PidResult{0.0, PidStatus::CalculationOverflow, false};
    }
    const double candidateIntegralUnclamped =
        integralOutput_ + integralDelta;
    if (!std::isfinite(candidateIntegralUnclamped)) {
        lastOutput_ = 0.0;
        hasPreviousError_ = false;
        return PidResult{0.0, PidStatus::CalculationOverflow, false};
    }
    const double candidateIntegral = clamp(
        candidateIntegralUnclamped,
        config_.limits.minimum,
        config_.limits.maximum);
    const double candidateOutput = baseOutput + candidateIntegral;
    if (!std::isfinite(candidateIntegral) ||
        !std::isfinite(candidateOutput)) {
        lastOutput_ = 0.0;
        hasPreviousError_ = false;
        return PidResult{0.0, PidStatus::CalculationOverflow, false};
    }

    const bool saturated =
        candidateIntegral != candidateIntegralUnclamped ||
        candidateOutput > config_.limits.maximum ||
        candidateOutput < config_.limits.minimum;

    if (candidateOutput > config_.limits.maximum && integralDelta > 0.0) {
        const double boundary = config_.limits.maximum - baseOutput;
        if (integralOutput_ < boundary) {
            integralOutput_ = clamp(
                boundary,
                config_.limits.minimum,
                config_.limits.maximum);
        }
    } else if (
        candidateOutput < config_.limits.minimum && integralDelta < 0.0) {
        const double boundary = config_.limits.minimum - baseOutput;
        if (integralOutput_ > boundary) {
            integralOutput_ = clamp(
                boundary,
                config_.limits.minimum,
                config_.limits.maximum);
        }
    } else {
        integralOutput_ = candidateIntegral;
    }

    const double unclampedOutput = baseOutput + integralOutput_;
    if (!std::isfinite(unclampedOutput)) {
        lastOutput_ = 0.0;
        hasPreviousError_ = false;
        return PidResult{0.0, PidStatus::CalculationOverflow, false};
    }
    lastOutput_ = clamp(
        unclampedOutput,
        config_.limits.minimum,
        config_.limits.maximum);
    previousError_ = error;
    previousMeasured_ = measured;
    hasPreviousError_ = true;

    return PidResult{
        lastOutput_,
        PidStatus::Ok,
        saturated || unclampedOutput != lastOutput_};
}

void Pid::reset()
{
    integralOutput_ = 0.0;
    previousError_ = 0.0;
    previousMeasured_ = 0.0;
    lastOutput_ = 0.0;
    hasPreviousError_ = false;
}

const PidConfig &Pid::config() const
{
    return config_;
}

PidStatus Pid::configurationStatus() const
{
    return configurationStatus_;
}

double Pid::integralOutput() const
{
    return integralOutput_;
}

double Pid::lastOutput() const
{
    return lastOutput_;
}

}  // namespace robomaster
