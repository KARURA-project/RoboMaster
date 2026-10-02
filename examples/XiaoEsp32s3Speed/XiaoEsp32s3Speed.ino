#include <RoboMasterCore.h>
using namespace robomaster;

Motor motor{1, MotorModel::M3508, ControllerModel::C620};
MotorBus bus;
Esp32Can can{Esp32CanConfig{1, 2, 8, 64}};
Esp32Timer timer;
constexpr uint32_t kFeedbackWaitMs = 2000U;

void fail(const char *message) {
    motor.emergencyStop();
    if (can.started()) can.send(bus);
    Serial.println(message);
    while (true) delay(1000);
}

void waitForFeedback() {
    const uint32_t startedAtMs = millis();
    while (!motor.state().hasFeedback()) {
        const Esp32CanStatus status = can.receive(bus);
        if (status != Esp32CanStatus::Ok) fail("CAN receive failed");
        if (millis() - startedAtMs >= kFeedbackWaitMs) fail("feedback timeout");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    if (bus.add(motor) != MotorBusStatus::Ok) fail("bus.add failed");
    if (motor.setMaxCurrent(2.0) != MotorStatus::Ok) fail("max current failed");
    if (motor.setMaxSpeed(60.0) != MotorStatus::Ok) fail("max speed failed");
    if (motor.setExpectedFeedbackRate(FeedbackRate::Hz1000) != MotorStatus::Ok) fail("feedback rate failed");
    if (motor.setSpeedGains(PidGains{0.10, 0.10, 0.0}) != MotorStatus::Ok) fail("speed gains failed");
    if (motor.coast(micros()) != MotorStatus::Ok) fail("coast failed");
    if (can.begin() != Esp32CanStatus::Ok) fail("CAN start failed");
    if (timer.begin(1000U) != Esp32TimerStatus::Ok) fail("timer start failed");
    waitForFeedback();
    if (motor.setTarget(ControlType::Speed, 30.0, micros()) != MotorStatus::Ok) fail("speed target failed");
}

void loop() {
    const Esp32CanStatus rx = can.receive(bus);
    if (rx == Esp32CanStatus::BusOff || rx == Esp32CanStatus::ErrorPassive) fail("CAN fault");
    if (timer.takePending() > 0U) {
        const uint32_t nowUs = micros();
        if (bus.updateControl(nowUs) != MotorBusStatus::Ok) fail("control fault");
        if (can.send(bus) != Esp32CanStatus::Ok) fail("CAN send failed");
        bus.markCommandFramesSent(nowUs);
    }
}
