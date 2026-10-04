#include <cassert>
#include <cmath>
#include <cstdint>

#include "robomaster/Motor.hpp"

using namespace robomaster;

namespace {

class SimulatedM2006 {
public:
    canbridge::Frame feedbackFrame(std::uint8_t id) const
    {
        const double rotorDegrees = positionDegrees_ * kGearRatio;
        double wrappedDegrees = std::fmod(rotorDegrees, 360.0);
        if (wrappedDegrees < 0.0) {
            wrappedDegrees += 360.0;
        }
        std::uint16_t encoder = static_cast<std::uint16_t>(std::lround(
            wrappedDegrees * static_cast<double>(kEncoderCounts) / 360.0));
        encoder %= kEncoderCounts;

        const std::int16_t rotorRpm = static_cast<std::int16_t>(std::lround(
            speedRpm_ * kGearRatio));
        std::int16_t currentRaw = 0;
        assert(encodeCurrent(
            appliedCurrentA_,
            ControllerModel::C610,
            currentRaw) == ProtocolStatus::Ok);

        canbridge::Frame frame;
        frame.id = 0x200U + id;
        frame.length = 8;
        frame.data[0] = static_cast<std::uint8_t>(encoder >> 8U);
        frame.data[1] = static_cast<std::uint8_t>(encoder & 0xFFU);
        const std::uint16_t rawRpm = static_cast<std::uint16_t>(rotorRpm);
        frame.data[2] = static_cast<std::uint8_t>(rawRpm >> 8U);
        frame.data[3] = static_cast<std::uint8_t>(rawRpm & 0xFFU);
        const std::uint16_t rawCurrent =
            static_cast<std::uint16_t>(currentRaw);
        frame.data[4] = static_cast<std::uint8_t>(rawCurrent >> 8U);
        frame.data[5] = static_cast<std::uint8_t>(rawCurrent & 0xFFU);
        return frame;
    }

    void step(double currentA, double dtSeconds)
    {
        appliedCurrentA_ = currentA;
        const double accelerationRpmPerSecond =
            kCurrentAcceleration * currentA - kDamping * speedRpm_;
        speedRpm_ += accelerationRpmPerSecond * dtSeconds;
        positionDegrees_ += speedRpm_ * 6.0 * dtSeconds;
    }

    double speedRpm() const
    {
        return speedRpm_;
    }

    double positionDegrees() const
    {
        return positionDegrees_;
    }

private:
    static constexpr double kGearRatio = 36.0;
    static constexpr double kCurrentAcceleration = 200.0;
    static constexpr double kDamping = 4.0;

    double positionDegrees_ = 0.0;
    double speedRpm_ = 0.0;
    double appliedCurrentA_ = 0.0;
};

void configureMotor(Motor &motor)
{
    assert(motor.setExpectedFeedbackRate(FeedbackRate::Hz1000) ==
        MotorStatus::Ok);
    assert(motor.setMaxSpeed(200.0) == MotorStatus::Ok);
    assert(motor.setTrackingMaxSpeed(500.0) == MotorStatus::Ok);
    assert(motor.setMaxCurrent(5.0) == MotorStatus::Ok);
    assert(motor.setPositionControlPeriodUs(4000) == MotorStatus::Ok);
    assert(motor.setSpeedControlPeriodUs(1000) == MotorStatus::Ok);
    assert(motor.setPositionGains(PidGains{2.0, 0.0, 0.0}) ==
        MotorStatus::Ok);
    assert(motor.setSpeedGains(PidGains{0.04, 0.2, 0.0}) ==
        MotorStatus::Ok);
}

void runStep(Motor &motor, SimulatedM2006 &plant, std::uint32_t nowUs)
{
    assert(motor.updateFeedback(plant.feedbackFrame(motor.id()), nowUs) ==
        MotorStateStatus::Ok);
    const MotorStatus status = motor.updateControl(nowUs);
    assert(status == MotorStatus::Ok || status == MotorStatus::NoTarget);
    plant.step(motor.commandCurrent(), 0.001);
}

void testClosedLoopSpeedControl()
{
    Motor motor{1, MotorModel::M2006, ControllerModel::C610};
    SimulatedM2006 plant;
    configureMotor(motor);
    runStep(motor, plant, 0);
    assert(motor.setTarget(ControlType::Speed, 100.0, 0) == MotorStatus::Ok);

    for (std::uint32_t step = 1; step <= 5000; ++step) {
        runStep(motor, plant, step * 1000U);
    }

    assert(std::fabs(plant.speedRpm() - 100.0) < 0.5);
    assert(std::fabs(motor.commandCurrent()) < 2.1);
}

void testClosedLoopPositionControl()
{
    Motor motor{1, MotorModel::M2006, ControllerModel::C610};
    SimulatedM2006 plant;
    configureMotor(motor);
    runStep(motor, plant, 0);
    assert(motor.setPosition(0.0) == MotorStatus::Ok);
    assert(motor.setTarget(ControlType::Position, 90.0, 0) ==
        MotorStatus::Ok);

    for (std::uint32_t step = 1; step <= 10000; ++step) {
        runStep(motor, plant, step * 1000U);
    }

    assert(std::fabs(plant.positionDegrees() - 90.0) < 0.5);
    assert(std::fabs(plant.speedRpm()) < 0.5);
}

void testBrakeStopsFasterThanCoast()
{
    Motor coastMotor{1, MotorModel::M2006, ControllerModel::C610};
    Motor brakeMotor{2, MotorModel::M2006, ControllerModel::C610};
    SimulatedM2006 coastPlant;
    SimulatedM2006 brakePlant;
    configureMotor(coastMotor);
    configureMotor(brakeMotor);
    runStep(coastMotor, coastPlant, 0);
    runStep(brakeMotor, brakePlant, 0);
    coastMotor.setTarget(ControlType::Speed, 100.0, 0);
    brakeMotor.setTarget(ControlType::Speed, 100.0, 0);

    for (std::uint32_t step = 1; step <= 3000; ++step) {
        runStep(coastMotor, coastPlant, step * 1000U);
        runStep(brakeMotor, brakePlant, step * 1000U);
    }

    assert(coastMotor.coast(3000000U) == MotorStatus::Ok);
    assert(brakeMotor.brake(3000000U) == MotorStatus::Ok);
    for (std::uint32_t step = 3001; step <= 3200; ++step) {
        runStep(coastMotor, coastPlant, step * 1000U);
        runStep(brakeMotor, brakePlant, step * 1000U);
    }

    assert(std::fabs(brakePlant.speedRpm()) <
        std::fabs(coastPlant.speedRpm()));

    for (std::uint32_t step = 3201; step <= 5000; ++step) {
        runStep(brakeMotor, brakePlant, step * 1000U);
    }
    assert(std::fabs(brakePlant.speedRpm()) < 0.1);
    assert(std::fabs(brakeMotor.commandCurrent()) < 0.01);
}

void testFeedbackLossForcesZeroCurrent()
{
    Motor motor{1, MotorModel::M2006, ControllerModel::C610};
    SimulatedM2006 plant;
    configureMotor(motor);
    runStep(motor, plant, 0);
    motor.setTarget(ControlType::Speed, 100.0, 0);
    runStep(motor, plant, 1000);
    assert(motor.commandCurrent() > 0.0);

    assert(motor.updateControl(4001) == MotorStatus::FeedbackTimeout);
    assert(motor.commandCurrent() == 0.0);
    assert(!motor.hasTarget());
}

}  // namespace

int main()
{
    testClosedLoopSpeedControl();
    testClosedLoopPositionControl();
    testBrakeStopsFasterThanCoast();
    testFeedbackLossForcesZeroCurrent();
    return 0;
}
