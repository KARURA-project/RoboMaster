#pragma once

#include <cstdint>

namespace robomaster {

struct PidGains {
    constexpr PidGains(double kp = 0.0, double ki = 0.0, double kd = 0.0)
        : kp(kp), ki(ki), kd(kd)
    {
    }

    double kp;
    double ki;
    double kd;
};

struct PidLimits {
    constexpr PidLimits(
        double minimum = 0.0,
        double maximum = 0.0)
        : minimum(minimum), maximum(maximum)
    {
    }

    double minimum;
    double maximum;
};

enum class DerivativeSource : std::uint8_t {
    Measurement,
    Error,
};

struct PidConfig {
    constexpr PidConfig(
        PidGains gains = PidGains{},
        PidLimits limits = PidLimits{},
        DerivativeSource derivativeSource = DerivativeSource::Measurement)
        : gains(gains),
          limits(limits),
          derivativeSource(derivativeSource)
    {
    }

    PidGains gains;
    PidLimits limits;
    DerivativeSource derivativeSource;
};

enum class PidStatus : std::uint8_t {
    Ok,
    InvalidGains,
    InvalidLimits,
    InvalidInput,
    InvalidTimeStep,
    InvalidDerivativeSource,
    CalculationOverflow,
};

struct PidResult {
    constexpr PidResult(
        double output = 0.0,
        PidStatus status = PidStatus::Ok,
        bool saturated = false)
        : output(output), status(status), saturated(saturated)
    {
    }

    double output;
    PidStatus status;
    bool saturated;
};

class Pid {
public:
    Pid() = default;
    explicit Pid(const PidConfig &config);

    PidStatus configure(const PidConfig &config);
    PidStatus setGains(const PidGains &gains);
    PidStatus setLimits(const PidLimits &limits);

    PidResult update(double target, double measured, double dtSeconds);
    void reset();

    const PidConfig &config() const;
    PidStatus configurationStatus() const;
    double integralOutput() const;
    double lastOutput() const;

private:
    PidConfig config_{};
    double integralOutput_ = 0.0;
    double previousError_ = 0.0;
    double previousMeasured_ = 0.0;
    double lastOutput_ = 0.0;
    bool hasPreviousError_ = false;
    PidStatus configurationStatus_ = PidStatus::Ok;
};

}  // namespace robomaster
