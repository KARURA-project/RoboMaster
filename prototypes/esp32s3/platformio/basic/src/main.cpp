#include <Arduino.h>
#include <RoboMasterCore.h>
#include "Esp32Can.hpp"
using namespace robomaster;

// XIAO ESP32S3: D0/GPIO1 TX, D1/GPIO2 RX; M3508 + C620, ID 1.
Motor motor{1, MotorModel::M3508, ControllerModel::C620};
MotorBus bus;
Esp32Can can{Esp32CanConfig{1, 2, 8, 64}};
bool ready = false;
bool faulted = false;
uint32_t waitingSinceUs = 0;

void stopOnFault() {
    faulted = true;
    bus.emergencyStopAll(); // Remains latched; reset the application to recover.
}

void updateTarget(uint32_t nowUs);

void setup() {
    Serial.begin(115200);
    if (bus.add(motor) != MotorBusStatus::Ok ||
        motor.setMaxCurrent(2.0) != MotorStatus::Ok ||
        motor.setMaxSpeed(60.0) != MotorStatus::Ok ||
        motor.setTrackingMaxSpeed(500.0) != MotorStatus::Ok ||
        motor.setExpectedFeedbackRate(FeedbackRate::Hz1000) != MotorStatus::Ok ||
        motor.setSpeedGains(PidGains{0.10, 0.10, 0.0}) != MotorStatus::Ok ||
        motor.setPositionGains(PidGains{0.50, 0.0, 0.0}) != MotorStatus::Ok ||
        motor.coast(micros()) != MotorStatus::Ok ||
        can.begin() != Esp32CanStatus::Ok) {
        stopOnFault();
    }
    waitingSinceUs = micros();
}

void loop() {
    if (can.started() && can.receive(bus) != Esp32CanStatus::Ok) stopOnFault();
    const uint32_t nowUs = micros();
    if (!faulted && !ready) {
        if (motor.state().hasFeedback()) {
            if (motor.setPosition(0.0) != MotorStatus::Ok) stopOnFault();
            else ready = true;
        } else if (nowUs - waitingSinceUs >= 2000000U) {
            stopOnFault();
        }
    }
    if (ready && !faulted) updateTarget(nowUs);
    if (!faulted && bus.updateControl(nowUs) != MotorBusStatus::Ok) stopOnFault();
    // Keep sending zero-current frames after a fault; never replay an old target.
    if (can.started() && bus.commandFramesDue(nowUs)) {
        if (can.send(bus) == Esp32CanStatus::Ok) bus.markCommandFramesSent(nowUs);
        else stopOnFault();
    }
}

// Non-blocking sequence: +30 rpm, 0 rpm, -30 rpm, 0 rpm; 2 s each.
void updateTarget(uint32_t nowUs) {
    static bool started = false;
    static uint8_t phase = 0;
    static uint32_t changedAtUs = 0;
    static const double targets[] = {30.0, 0.0, -30.0, 0.0};
    if (!started || nowUs - changedAtUs >= 2000000U) {
        if (started) phase = (phase + 1U) % 4U;
        started = true;
        changedAtUs = nowUs;
        if (motor.setTarget(ControlType::Speed, targets[phase], nowUs) != MotorStatus::Ok)
            stopOnFault();
    }
}
