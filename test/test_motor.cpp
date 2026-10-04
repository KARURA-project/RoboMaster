#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "robomaster/Motor.hpp"

using namespace robomaster;

namespace {

canbridge::Frame feedbackFrame(
    std::uint8_t id,
    std::uint16_t encoder,
    std::int16_t rpm,
    std::int16_t currentRaw = 0)
{
    canbridge::Frame frame;
    frame.id = 0x200U + id;
    frame.length = 8;
    frame.data[0] = static_cast<std::uint8_t>(encoder >> 8U);
    frame.data[1] = static_cast<std::uint8_t>(encoder & 0xFFU);
    const std::uint16_t rawRpm = static_cast<std::uint16_t>(rpm);
    frame.data[2] = static_cast<std::uint8_t>(rawRpm >> 8U);
    frame.data[3] = static_cast<std::uint8_t>(rawRpm & 0xFFU);
    const std::uint16_t rawCurrent = static_cast<std::uint16_t>(currentRaw);
    frame.data[4] = static_cast<std::uint8_t>(rawCurrent >> 8U);
    frame.data[5] = static_cast<std::uint8_t>(rawCurrent & 0xFFU);
    return frame;
}

bool near(double actual, double expected, double tolerance = 1e-9)
{
    return std::fabs(actual - expected) <= tolerance;
}

void configureMotor(Motor &motor)
{
    assert(motor.setExpectedFeedbackRate(FeedbackRate::Hz1000) ==
        MotorStatus::Ok);
    assert(motor.setMaxSpeed(200.0) == MotorStatus::Ok);
    assert(motor.setTrackingMaxSpeed(500.0) == MotorStatus::Ok);
    assert(motor.setMaxCurrent(5.0) == MotorStatus::Ok);
    assert(motor.setPositionGains(PidGains{10.0, 0.0, 0.0}) ==
        MotorStatus::Ok);
    assert(motor.setSpeedGains(PidGains{0.05, 0.0, 0.0}) ==
        MotorStatus::Ok);
}

void testConfigurationAndOutputUnits()
{
    Motor motor{1, MotorModel::M2006, ControllerModel::C610};
    assert(motor.configurationStatus() == MotorStatus::Ok);
    assert(motor.setExternalGearRatio(2.0) == MotorStatus::Ok);
    assert(motor.setDirection(RotationDirection::Reverse) == MotorStatus::Ok);
    configureMotor(motor);
    assert(motor.setPosition(90.0) == MotorStatus::Ok);

    assert(motor.updateFeedback(feedbackFrame(1, 0, -7200, -5000), 1000) ==
        MotorStateStatus::Ok);
    assert(near(motor.position(), 90.0));
    assert(near(motor.speed(), 100.0));
    assert(near(motor.estimatedCurrent(), 5.0));

    assert(motor.setExternalGearRatio(3.0) ==
        MotorStatus::InvalidConfiguration);
}

void testCommandAndTrackingSpeedLimitsAreIndependent()
{
    Motor motor{1, MotorModel::M3508, ControllerModel::C620};
    assert(motor.setMaxSpeed(60.0) == MotorStatus::Ok);
    assert(hasPositionFlag(
        motor.state().positionFlags(), PositionFlag::MaxRpmUnset));
    assert(motor.setTrackingMaxSpeed(500.0) == MotorStatus::Ok);
    assert(!hasPositionFlag(
        motor.state().positionFlags(), PositionFlag::MaxRpmUnset));
    assert(near(motor.maxSpeed(), 60.0));
    assert(near(motor.trackingMaxSpeed(), 500.0));
}

void testCurrentTargetAndDirection()
{
    Motor motor{1, MotorModel::M3508, ControllerModel::C620};
    assert(motor.coast(0) == MotorStatus::Ok);
    assert(motor.updateControl(0) == MotorStatus::Ok);
    assert(motor.commandCurrent() == 0.0);

    assert(motor.setDirection(RotationDirection::Reverse) ==
        MotorStatus::InvalidConfiguration);

    Motor reverse{1, MotorModel::M3508, ControllerModel::C620};
    assert(reverse.setDirection(RotationDirection::Reverse) == MotorStatus::Ok);
    configureMotor(reverse);
    reverse.updateFeedback(feedbackFrame(1, 0, 0), 1000);
    assert(reverse.setTarget(ControlType::Current, 2.0, 1000) ==
        MotorStatus::Ok);
    assert(reverse.updateControl(1000) == MotorStatus::Ok);
    assert(reverse.commandCurrent() == -2.0);
}

void testRejectsUnsafeTargets()
{
    Motor motor{1, MotorModel::M3508, ControllerModel::C620};
    configureMotor(motor);
    motor.updateFeedback(feedbackFrame(1, 0, 0), 1000);

    assert(motor.setTarget(ControlType::Speed, 201.0, 1000) ==
        MotorStatus::TargetOutOfRange);
    assert(!motor.hasTarget());
    assert(motor.commandCurrent() == 0.0);

    assert(motor.setTarget(
        ControlType::Current,
        std::numeric_limits<double>::quiet_NaN(),
        1000) == MotorStatus::InvalidTarget);
    assert(motor.commandCurrent() == 0.0);

    assert(motor.setTarget(static_cast<ControlType>(0xFF), 0.0, 1000) ==
        MotorStatus::InvalidControlType);
}

void testSpeedControl()
{
    Motor motor{1, MotorModel::M2006, ControllerModel::C610};
    configureMotor(motor);
    motor.updateFeedback(feedbackFrame(1, 0, 0), 1000);

    assert(motor.setTarget(ControlType::Speed, 100.0, 1000) ==
        MotorStatus::Ok);
    assert(motor.updateControl(1000) == MotorStatus::Ok);
    assert(near(motor.commandCurrent(), 5.0));

    motor.updateFeedback(feedbackFrame(1, 0, 3600), 2000);
    assert(motor.updateControl(2000) == MotorStatus::Ok);
    assert(near(motor.speed(), 100.0));
    assert(near(motor.commandCurrent(), 0.0));
}

void testRelativePositionControl()
{
    Motor motor{1, MotorModel::M2006, ControllerModel::C610};
    configureMotor(motor);
    motor.updateFeedback(feedbackFrame(1, 0, 0), 1000);
    assert(motor.state().isPositionContinuous());
    assert(!motor.state().isPositionAbsolute());

    assert(motor.setTarget(ControlType::Position, 10.0, 1000) ==
        MotorStatus::Ok);
    assert(motor.updateControl(1000) == MotorStatus::Ok);
    assert(near(motor.commandCurrent(), 5.0));
}

void testStopsAndEmergencyStop()
{
    Motor motor{1, MotorModel::M2006, ControllerModel::C610};
    configureMotor(motor);
    motor.updateFeedback(feedbackFrame(1, 0, 0), 1000);

    motor.setTarget(ControlType::Current, 2.0, 1000);
    motor.updateControl(1000);
    assert(near(motor.commandCurrent(), 2.0));

    assert(motor.coast(1100) == MotorStatus::Ok);
    motor.updateControl(1100);
    assert(motor.commandCurrent() == 0.0);

    assert(motor.brake(1200) == MotorStatus::Ok);
    assert(motor.target().type == ControlType::Speed);
    assert(motor.target().value == 0.0);

    assert(motor.hold(1300) == MotorStatus::Ok);
    assert(motor.target().type == ControlType::Position);

    motor.emergencyStop();
    assert(motor.emergencyStopped());
    assert(motor.commandCurrent() == 0.0);
    assert(motor.setTarget(ControlType::Current, 1.0, 1400) ==
        MotorStatus::EmergencyStopped);

    motor.clearEmergencyStop();
    assert(!motor.emergencyStopped());
    assert(!motor.hasTarget());
    assert(motor.status() == MotorStatus::NoTarget);
}

void testBrakeUsesOnlyProportionalGain()
{
    Motor motor{1, MotorModel::M2006, ControllerModel::C610};
    configureMotor(motor);
    assert(motor.setSpeedGains(PidGains{0.05, 1.0, 0.0}) ==
        MotorStatus::Ok);

    motor.updateFeedback(feedbackFrame(1, 0, 3600), 1000);
    assert(motor.brake(1000) == MotorStatus::Ok);
    assert(motor.updateControl(1000) == MotorStatus::Ok);
    assert(near(motor.commandCurrent(), -5.0));

    motor.updateFeedback(feedbackFrame(1, 0, 0), 2000);
    assert(motor.updateControl(2000) == MotorStatus::Ok);
    assert(near(motor.commandCurrent(), 0.0));

    motor.updateFeedback(feedbackFrame(1, 0, 0), 3000);
    assert(motor.updateControl(3000) == MotorStatus::Ok);
    assert(near(motor.commandCurrent(), 0.0));

    assert(motor.setTarget(ControlType::Speed, 0.0, 3000) ==
        MotorStatus::Ok);
    motor.updateFeedback(feedbackFrame(1, 0, 360), 4000);
    assert(motor.updateControl(4000) == MotorStatus::Ok);
    assert(motor.commandCurrent() < -0.5);
}

void testCommandAndFeedbackTimeouts()
{
    Motor coastMotor{1, MotorModel::M2006, ControllerModel::C610};
    configureMotor(coastMotor);
    assert(coastMotor.setCommandTimeoutUs(2000) == MotorStatus::Ok);
    assert(coastMotor.coast(1000) == MotorStatus::Ok);
    assert(coastMotor.updateControl(10000) == MotorStatus::Ok);
    assert(coastMotor.hasTarget());
    assert(coastMotor.commandCurrent() == 0.0);

    Motor commandTimeout{1, MotorModel::M2006, ControllerModel::C610};
    configureMotor(commandTimeout);
    commandTimeout.setCommandTimeoutUs(2000);
    commandTimeout.updateFeedback(feedbackFrame(1, 0, 0), 1000);
    commandTimeout.setTarget(ControlType::Current, 1.0, 1000);
    commandTimeout.updateControl(1000);
    assert(commandTimeout.updateControl(3001) == MotorStatus::CommandTimeout);
    assert(commandTimeout.commandCurrent() == 0.0);
    assert(!commandTimeout.hasTarget());

    Motor feedbackTimeout{1, MotorModel::M2006, ControllerModel::C610};
    configureMotor(feedbackTimeout);
    feedbackTimeout.updateFeedback(feedbackFrame(1, 0, 0), 1000);
    feedbackTimeout.setTarget(ControlType::Current, 1.0, 1000);
    assert(feedbackTimeout.updateControl(4001) ==
        MotorStatus::FeedbackTimeout);
    assert(feedbackTimeout.commandCurrent() == 0.0);
}

void testPositionFaultStopsControl()
{
    Motor motor{1, MotorModel::M3508, ControllerModel::C620};
    configureMotor(motor);
    motor.setPosition(0.0);
    motor.updateFeedback(feedbackFrame(1, 0, 0), 1000);
    motor.setTarget(ControlType::Position, 10.0, 1000);
    motor.updateControl(1000);
    assert(motor.commandCurrent() != 0.0);

    motor.notifyFeedbackLoss();
    assert(motor.status() == MotorStatus::PositionUnreliable);
    assert(!motor.hasTarget());
    assert(motor.updateControl(10000) == MotorStatus::NoTarget);
    assert(motor.commandCurrent() == 0.0);
}

void testMalformedFeedbackStopsOutput()
{
    Motor motor{1, MotorModel::M2006, ControllerModel::C610};
    configureMotor(motor);
    motor.updateFeedback(feedbackFrame(1, 0, 0), 1000);
    motor.setTarget(ControlType::Current, 2.0, 1000);
    motor.updateControl(1000);
    assert(motor.commandCurrent() == 2.0);

    canbridge::Frame malformed = feedbackFrame(1, 0, 0);
    malformed.length = 7U;
    assert(motor.updateFeedback(malformed, 2000) ==
        MotorStateStatus::InvalidFrameLength);
    assert(motor.status() == MotorStatus::InvalidFeedback);
    assert(motor.commandCurrent() == 0.0);
    assert(!motor.hasTarget());
}

void testLoweringLimitsCannotLeaveAnUnsafeCommand()
{
    Motor currentMotor{1, MotorModel::M2006, ControllerModel::C610};
    configureMotor(currentMotor);
    currentMotor.updateFeedback(feedbackFrame(1, 0, 0), 1000);
    currentMotor.setTarget(ControlType::Current, 4.0, 1000);
    currentMotor.updateControl(1000);
    assert(currentMotor.commandCurrent() == 4.0);
    assert(currentMotor.setMaxCurrent(3.0) ==
        MotorStatus::TargetOutOfRange);
    assert(currentMotor.commandCurrent() == 0.0);
    assert(!currentMotor.hasTarget());

    Motor speedMotor{1, MotorModel::M2006, ControllerModel::C610};
    configureMotor(speedMotor);
    speedMotor.updateFeedback(feedbackFrame(1, 0, 0), 1000);
    speedMotor.setTarget(ControlType::Speed, 150.0, 1000);
    assert(speedMotor.setMaxSpeed(100.0) ==
        MotorStatus::TargetOutOfRange);
    assert(speedMotor.commandCurrent() == 0.0);
    assert(!speedMotor.hasTarget());
}

}  // namespace

int main()
{
    static_assert(!std::is_copy_constructible<Motor>::value,
        "Motor must preserve its physical identity");
    static_assert(!std::is_move_constructible<Motor>::value,
        "Registered Motor addresses must remain stable");
    testConfigurationAndOutputUnits();
    testCommandAndTrackingSpeedLimitsAreIndependent();
    testCurrentTargetAndDirection();
    testRejectsUnsafeTargets();
    testSpeedControl();
    testRelativePositionControl();
    testStopsAndEmergencyStop();
    testBrakeUsesOnlyProportionalGain();
    testCommandAndFeedbackTimeouts();
    testPositionFaultStopsControl();
    testMalformedFeedbackStopsOutput();
    testLoweringLimitsCannotLeaveAnUnsafeCommand();
    return 0;
}
