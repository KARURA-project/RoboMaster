#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "robomaster/MotorBus.hpp"

using namespace robomaster;

namespace {

canbridge::Frame feedbackFrame(std::uint8_t id)
{
    canbridge::Frame frame;
    frame.id = 0x200U + id;
    frame.length = 8;
    return frame;
}

void configureMotor(Motor &motor)
{
    assert(motor.setExpectedFeedbackRate(FeedbackRate::Hz1000) ==
        MotorStatus::Ok);
    assert(motor.setMaxSpeed(1000.0) == MotorStatus::Ok);
    assert(motor.setTrackingMaxSpeed(1000.0) == MotorStatus::Ok);
    assert(motor.setMaxCurrent(
        motor.controllerModel() == ControllerModel::C610 ? 10.0 : 20.0) ==
        MotorStatus::Ok);
}

std::int16_t readCurrent(const canbridge::Frame &frame, std::size_t slot)
{
    const std::uint16_t raw = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(frame.data[slot * 2U]) << 8U) |
        frame.data[slot * 2U + 1U]);
    return static_cast<std::int16_t>(raw);
}

void testRegistrationAndRouting()
{
    Motor motor1{1, MotorModel::M3508, ControllerModel::C620};
    Motor duplicateId{1, MotorModel::M2006, ControllerModel::C610};
    Motor motor2{2, MotorModel::M2006, ControllerModel::C610};
    MotorBus bus;

    assert(bus.add(motor1) == MotorBusStatus::Ok);
    assert(bus.add(motor1) == MotorBusStatus::DuplicateMotor);
    assert(bus.add(duplicateId) == MotorBusStatus::DuplicateId);
    assert(bus.add(motor2) == MotorBusStatus::Ok);
    assert(bus.size() == 2U);
    assert(bus.motor(1) == &motor1);
    assert(bus.motor(2) == &motor2);
    assert(bus.motor(0) == nullptr);

    assert(bus.updateFeedback(feedbackFrame(2), 1000) ==
        MotorBusStatus::Ok);
    assert(motor2.state().hasFeedback());
    assert(!motor1.state().hasFeedback());
    assert(bus.updateFeedback(feedbackFrame(3), 1000) ==
        MotorBusStatus::UnknownMotor);
}

void testCommandFrameAggregation()
{
    Motor motor1{1, MotorModel::M2006, ControllerModel::C610};
    Motor motor3{3, MotorModel::M3508, ControllerModel::C620};
    Motor motor5{5, MotorModel::M2006, ControllerModel::C610};
    configureMotor(motor1);
    configureMotor(motor3);
    configureMotor(motor5);
    motor1.updateFeedback(feedbackFrame(1), 1000);
    motor3.updateFeedback(feedbackFrame(3), 1000);
    motor5.updateFeedback(feedbackFrame(5), 1000);
    motor1.setTarget(ControlType::Current, 5.0, 1000);
    motor3.setTarget(ControlType::Current, 10.0, 1000);
    motor5.setTarget(ControlType::Current, -10.0, 1000);

    MotorBus bus;
    assert(bus.add(motor1) == MotorBusStatus::Ok);
    assert(bus.add(motor3) == MotorBusStatus::Ok);
    assert(bus.add(motor5) == MotorBusStatus::Ok);
    assert(bus.updateControl(1000) == MotorBusStatus::Ok);

    std::array<canbridge::Frame, MotorBus::kMaxCommandFrames> frames{};
    std::size_t count = 0U;
    assert(bus.makeCommandFrames(frames, count) == MotorBusStatus::Ok);
    assert(count == 2U);
    assert(frames[0].id == kCommandId1To4);
    assert(readCurrent(frames[0], 0) == 5000);
    assert(readCurrent(frames[0], 1) == 0);
    assert(readCurrent(frames[0], 2) == 8192);
    assert(readCurrent(frames[0], 3) == 0);
    assert(frames[1].id == kCommandId5To8);
    assert(readCurrent(frames[1], 0) == -10000);
    assert(readCurrent(frames[1], 1) == 0);
}

void testUnusedGroupIsNotGenerated()
{
    Motor motor1{1, MotorModel::M2006, ControllerModel::C610};
    MotorBus bus;
    assert(bus.add(motor1) == MotorBusStatus::Ok);

    std::array<canbridge::Frame, MotorBus::kMaxCommandFrames> frames{};
    std::size_t count = 0U;
    assert(bus.makeCommandFrames(frames, count) == MotorBusStatus::Ok);
    assert(count == 1U);
    assert(frames[0].id == kCommandId1To4);
}

void testBothCommandGroupsPreserveMotorSlots()
{
    Motor motor4{4, MotorModel::M2006, ControllerModel::C610};
    Motor motor8{8, MotorModel::M3508, ControllerModel::C620};
    configureMotor(motor4);
    configureMotor(motor8);
    motor4.updateFeedback(feedbackFrame(4), 1000);
    motor8.updateFeedback(feedbackFrame(8), 1000);
    motor4.setTarget(ControlType::Current, 1.0, 1000);
    motor8.setTarget(ControlType::Current, -2.0, 1000);

    MotorBus bus;
    assert(bus.add(motor4) == MotorBusStatus::Ok);
    assert(bus.add(motor8) == MotorBusStatus::Ok);
    assert(bus.updateControl(1000) == MotorBusStatus::Ok);

    std::array<canbridge::Frame, MotorBus::kMaxCommandFrames> frames{};
    std::size_t count = 0U;
    assert(bus.makeCommandFrames(frames, count) == MotorBusStatus::Ok);
    assert(count == 2U);
    assert(frames[0].id == kCommandId1To4);
    assert(readCurrent(frames[0], 0) == 0);
    assert(readCurrent(frames[0], 3) == 1000);
    assert(frames[1].id == kCommandId5To8);
    assert(readCurrent(frames[1], 0) == 0);
    assert(readCurrent(frames[1], 3) == -1638);
}

void testMalformedRoutedFeedbackStopsOnlyItsMotor()
{
    Motor motor1{1, MotorModel::M2006, ControllerModel::C610};
    Motor motor2{2, MotorModel::M2006, ControllerModel::C610};
    configureMotor(motor1);
    configureMotor(motor2);
    motor1.updateFeedback(feedbackFrame(1), 1000);
    motor2.updateFeedback(feedbackFrame(2), 1000);
    motor1.setTarget(ControlType::Current, 1.0, 1000);
    motor2.setTarget(ControlType::Current, 2.0, 1000);
    motor1.updateControl(1000);
    motor2.updateControl(1000);

    MotorBus bus;
    bus.add(motor1);
    bus.add(motor2);
    canbridge::Frame malformed = feedbackFrame(1);
    malformed.length = 7U;
    assert(bus.updateFeedback(malformed, 2000) ==
        MotorBusStatus::MotorError);
    assert(motor1.commandCurrent() == 0.0);
    assert(motor2.commandCurrent() == 2.0);
}

void testBusWideFeedbackLossStopsEveryMotor()
{
    Motor motor1{1, MotorModel::M2006, ControllerModel::C610};
    Motor motor2{2, MotorModel::M2006, ControllerModel::C610};
    configureMotor(motor1);
    configureMotor(motor2);
    motor1.updateFeedback(feedbackFrame(1), 1000);
    motor2.updateFeedback(feedbackFrame(2), 1000);
    motor1.setTarget(ControlType::Current, 1.0, 1000);
    motor2.setTarget(ControlType::Current, 2.0, 1000);
    motor1.updateControl(1000);
    motor2.updateControl(1000);

    MotorBus bus;
    bus.add(motor1);
    bus.add(motor2);
    bus.notifyFeedbackLoss();

    assert(motor1.commandCurrent() == 0.0);
    assert(motor2.commandCurrent() == 0.0);
    assert(hasPositionFlag(
        motor1.state().positionFlags(), PositionFlag::FrameGap));
    assert(hasPositionFlag(
        motor2.state().positionFlags(), PositionFlag::FrameGap));
}

void testSendSchedulingIncludingWraparound()
{
    MotorBus bus;
    assert(bus.setSendPeriodUs(1000) == MotorBusStatus::Ok);
    assert(bus.commandFramesDue(UINT32_MAX - 499U));
    bus.markCommandFramesSent(UINT32_MAX - 499U);
    assert(!bus.commandFramesDue(UINT32_MAX));
    assert(bus.commandFramesDue(500U));
    assert(bus.setSendPeriodUs(0) == MotorBusStatus::InvalidSendPeriod);
}

void testGroupSafetyOperations()
{
    Motor motor1{1, MotorModel::M2006, ControllerModel::C610};
    Motor motor2{2, MotorModel::M2006, ControllerModel::C610};
    MotorBus bus;
    bus.add(motor1);
    bus.add(motor2);

    assert(bus.coastAll(0) == MotorBusStatus::Ok);
    bus.emergencyStopAll();
    assert(motor1.emergencyStopped());
    assert(motor2.emergencyStopped());
    bus.clearEmergencyStopAll();
    assert(!motor1.emergencyStopped());
    assert(!motor2.emergencyStopped());
}

void testAllEightMotorsAndDuplicatePriority()
{
    Motor motor1{1, MotorModel::M2006, ControllerModel::C610};
    Motor motor2{2, MotorModel::M2006, ControllerModel::C610};
    Motor motor3{3, MotorModel::M2006, ControllerModel::C610};
    Motor motor4{4, MotorModel::M2006, ControllerModel::C610};
    Motor motor5{5, MotorModel::M3508, ControllerModel::C620};
    Motor motor6{6, MotorModel::M3508, ControllerModel::C620};
    Motor motor7{7, MotorModel::M3508, ControllerModel::C620};
    Motor motor8{8, MotorModel::M3508, ControllerModel::C620};
    Motor duplicate8{8, MotorModel::M2006, ControllerModel::C610};
    MotorBus bus;

    assert(bus.add(motor1) == MotorBusStatus::Ok);
    assert(bus.add(motor2) == MotorBusStatus::Ok);
    assert(bus.add(motor3) == MotorBusStatus::Ok);
    assert(bus.add(motor4) == MotorBusStatus::Ok);
    assert(bus.add(motor5) == MotorBusStatus::Ok);
    assert(bus.add(motor6) == MotorBusStatus::Ok);
    assert(bus.add(motor7) == MotorBusStatus::Ok);
    assert(bus.add(motor8) == MotorBusStatus::Ok);
    assert(bus.size() == MotorBus::kMaxMotors);
    assert(bus.add(motor8) == MotorBusStatus::DuplicateMotor);
    assert(bus.add(duplicate8) == MotorBusStatus::DuplicateId);
}

}  // namespace

int main()
{
    static_assert(!std::is_copy_constructible<MotorBus>::value,
        "MotorBus must not duplicate registered motor pointers");
    testRegistrationAndRouting();
    testCommandFrameAggregation();
    testUnusedGroupIsNotGenerated();
    testBothCommandGroupsPreserveMotorSlots();
    testMalformedRoutedFeedbackStopsOnlyItsMotor();
    testBusWideFeedbackLossStopsEveryMotor();
    testSendSchedulingIncludingWraparound();
    testGroupSafetyOperations();
    testAllEightMotorsAndDuplicatePriority();
    return 0;
}
