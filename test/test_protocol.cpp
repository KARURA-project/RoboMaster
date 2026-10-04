#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "robomaster/Protocol.hpp"

using namespace robomaster;

namespace {

void testDecodeFeedback()
{
    canbridge::Frame frame;
    frame.id = 0x203;
    frame.length = 8;
    frame.data[0] = 0x10;
    frame.data[1] = 0x00;
    frame.data[2] = 0xFF;
    frame.data[3] = 0x9C;
    frame.data[4] = 0x27;
    frame.data[5] = 0x10;
    frame.data[6] = 42;

    Feedback feedback;
    assert(decodeFeedback(frame, ControllerModel::C620, feedback) ==
        ProtocolStatus::Ok);
    assert(feedback.id == 3);
    assert(feedback.encoder == 4096);
    assert(feedback.rpm == -100);
    assert(feedback.currentRaw == 10000);
    assert(feedback.temperatureC == 42);
    assert(feedback.hasTemperature);

    double currentA = 0.0;
    assert(estimateCurrent(
        feedback.currentRaw,
        ControllerModel::C610,
        currentA) == ProtocolStatus::Ok);
    assert(std::fabs(currentA - 10.0) < 1e-12);
}

void testRejectsMalformedFeedback()
{
    canbridge::Frame frame;
    frame.id = 0x201;
    frame.length = 7;
    Feedback feedback;
    feedback.id = 8;
    feedback.rpm = 100;
    assert(decodeFeedback(frame, ControllerModel::C610, feedback) ==
        ProtocolStatus::InvalidFrameLength);
    assert(feedback.id == 0);
    assert(feedback.rpm == 0);

    frame.length = 8;
    frame.extended = true;
    assert(decodeFeedback(frame, ControllerModel::C610, feedback) ==
        ProtocolStatus::InvalidFrameType);

    frame.extended = false;
    frame.data[0] = 0x20;
    frame.data[1] = 0x00;
    assert(decodeFeedback(frame, ControllerModel::C610, feedback) ==
        ProtocolStatus::InvalidEncoder);
}

void testControllerSpecificFeedback()
{
    canbridge::Frame frame;
    frame.id = 0x201;
    frame.length = 8;
    frame.data[6] = 42;

    Feedback feedback;
    assert(decodeFeedback(frame, ControllerModel::C610, feedback) ==
        ProtocolStatus::Ok);
    assert(!feedback.hasTemperature);
    assert(feedback.temperatureC == 0);

    assert(decodeFeedback(frame, ControllerModel::C620, feedback) ==
        ProtocolStatus::Ok);
    assert(feedback.hasTemperature);
    assert(feedback.temperatureC == 42);
}

void testCurrentEncoding()
{
    std::int16_t raw = 0;
    assert(encodeCurrent(10.0, ControllerModel::C610, raw) ==
        ProtocolStatus::Ok);
    assert(raw == 10000);

    assert(encodeCurrent(-11.0, ControllerModel::C610, raw) ==
        ProtocolStatus::Ok);
    assert(raw == -10000);

    assert(encodeCurrent(20.0, ControllerModel::C620, raw) ==
        ProtocolStatus::Ok);
    assert(raw == 16384);

    assert(encodeCurrent(
        std::numeric_limits<double>::quiet_NaN(),
        ControllerModel::C620,
        raw) ==
        ProtocolStatus::InvalidCurrent);
    assert(raw == 0);
}

void testGroupCurrentEncoding()
{
    const std::array<std::int16_t, 4> currents{{0x1234, -1, 0, 0x0102}};
    canbridge::Frame frame;
    assert(makeCommandFrame(1, currents, frame) == ProtocolStatus::Ok);
    assert(frame.id == 0x200);
    assert(frame.length == 8);
    assert(frame.data[0] == 0x12 && frame.data[1] == 0x34);
    assert(frame.data[2] == 0xFF && frame.data[3] == 0xFF);
    assert(frame.data[6] == 0x01 && frame.data[7] == 0x02);

    assert(setCommandCurrent(frame, 4, -10000) == ProtocolStatus::Ok);
    assert(frame.data[6] == 0xD8 && frame.data[7] == 0xF0);
    assert(setCommandCurrent(frame, 5, 0) == ProtocolStatus::InvalidCanId);

    frame.extended = true;
    assert(setCommandCurrent(frame, 4, 0) ==
        ProtocolStatus::InvalidFrameType);
    frame.extended = false;
    frame.length = 7;
    assert(setCommandCurrent(frame, 4, 0) ==
        ProtocolStatus::InvalidFrameLength);

    assert(makeCommandFrame(2, currents, frame) == ProtocolStatus::InvalidId);
}

void testProductSpecs()
{
    MotorSpec motor;
    ControllerSpec controller;
    assert(getMotorSpec(MotorModel::M3508, motor) == ProtocolStatus::Ok);
    assert(std::fabs(motor.gearRatio - 3591.0 / 187.0) <
        1e-12);
    assert(getControllerSpec(ControllerModel::C620, controller) ==
        ProtocolStatus::Ok);
    assert(controller.currentMaxA == 20.0);
    assert(controller.currentFullScale == 16384);
    assert(getControllerSpec(ControllerModel::C610, controller) ==
        ProtocolStatus::Ok);
    assert(controller.defaultFeedbackRate ==
        FeedbackRate::Hz1000);
    assert(getControllerSpec(ControllerModel::C620, controller) ==
        ProtocolStatus::Ok);
    assert(controller.defaultFeedbackRate ==
        FeedbackRate::Hz1000);

    assert(feedbackRateHz(FeedbackRate::Hz125) == 125);
    assert(feedbackRateHz(FeedbackRate::Hz250) == 250);
    assert(feedbackRateHz(FeedbackRate::Hz500) == 500);
    assert(feedbackRateHz(FeedbackRate::Hz1000) == 1000);
    assert(feedbackPeriodUs(FeedbackRate::Hz125) == 8000);
    assert(feedbackPeriodUs(FeedbackRate::Hz250) == 4000);
    assert(feedbackPeriodUs(FeedbackRate::Hz500) == 2000);
    assert(feedbackPeriodUs(FeedbackRate::Hz1000) == 1000);
    const FeedbackRate invalidRate = static_cast<FeedbackRate>(0);
    assert(feedbackRateHz(invalidRate) == 0);
    assert(feedbackPeriodUs(invalidRate) == 0);

    double currentA = 0.0;
    assert(estimateCurrent(8192, ControllerModel::C620, currentA) ==
        ProtocolStatus::Ok);
    assert(std::fabs(currentA - 10.0) < 1e-12);
}

void testRejectsInvalidModels()
{
    const MotorModel invalidMotor = static_cast<MotorModel>(0xFF);
    const ControllerModel invalidController =
        static_cast<ControllerModel>(0xFF);
    MotorSpec motor;
    ControllerSpec controller;
    assert(getMotorSpec(invalidMotor, motor) == ProtocolStatus::InvalidModel);
    assert(getControllerSpec(invalidController, controller) ==
        ProtocolStatus::InvalidModel);

    std::int16_t raw = 123;
    assert(encodeCurrent(1.0, invalidController, raw) ==
        ProtocolStatus::InvalidModel);
    assert(raw == 0);

    double currentA = 1.0;
    assert(estimateCurrent(100, invalidController, currentA) ==
        ProtocolStatus::InvalidModel);
    assert(currentA == 0.0);

    canbridge::Frame frame;
    frame.id = 0x201;
    frame.length = 8;
    Feedback feedback;
    assert(decodeFeedback(frame, invalidController, feedback) ==
        ProtocolStatus::InvalidModel);
}

}  // namespace

int main()
{
    testDecodeFeedback();
    testRejectsMalformedFeedback();
    testControllerSpecificFeedback();
    testCurrentEncoding();
    testGroupCurrentEncoding();
    testProductSpecs();
    testRejectsInvalidModels();
    return 0;
}
