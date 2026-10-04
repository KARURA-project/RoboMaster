#include <Arduino.h>
#include <RoboMasterCore.h>
#include "../../../../board/Esp32.hpp"
using namespace robomaster;

Motor motor{1, MotorModel::M3508, ControllerModel::C620};
MotorBus bus;
Esp32Can can{Esp32CanConfig{1, 2, 8, 64}};
Esp32Timer timer;

void fail(const char *message) {
    motor.emergencyStop();
    if (can.started()) can.send(bus);
    Serial.println(message);
    while (true) delay(1000);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    if (bus.add(motor) != MotorBusStatus::Ok) fail("bus.add failed");
    if (motor.setExpectedFeedbackRate(FeedbackRate::Hz1000) != MotorStatus::Ok) fail("feedback rate failed");
    if (motor.coast(micros()) != MotorStatus::Ok) fail("coast failed");
    if (can.begin() != Esp32CanStatus::Ok) fail("CAN start failed");
    if (timer.begin(1000U) != Esp32TimerStatus::Ok) fail("timer start failed");
    Serial.println("Zero-current communication check started.");
}

void loop() {
    const Esp32CanStatus rx = can.receive(bus);
    if (rx == Esp32CanStatus::BusOff ||
        rx == Esp32CanStatus::ErrorPassive ||
        rx == Esp32CanStatus::FeedbackLost ||
        rx == Esp32CanStatus::MotorError) {
        fail("CAN receive fault");
    }
    if (timer.takePending() > 0U) {
        const uint32_t nowUs = micros();
        if (bus.updateControl(nowUs) != MotorBusStatus::Ok) fail("control fault");
        if (can.send(bus) != Esp32CanStatus::Ok) fail("CAN send failed");
        bus.markCommandFramesSent(nowUs);
    }
}
