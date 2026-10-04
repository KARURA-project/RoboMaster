#include <Arduino.h>
#include <RoboMasterCore.h>
#include "Esp32Can.hpp"
using namespace robomaster;
using board::Esp32Can;
using board::Esp32CanConfig;
using board::Esp32CanStatus;

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
    if (can.started()) {
        // Receive routing belongs to the application, never to the CAN driver.
        // Bound work per loop so control and keyboard processing cannot starve.
        for (unsigned i = 0; i < 64U; ++i) {
            board::CanFrame received;
            const Esp32CanStatus status = can.receive(received);
            if (status == Esp32CanStatus::NoFrame) break;
            if (status != Esp32CanStatus::Ok) { stopOnFault(); break; }
            CanFrame frame;
            frame.id = received.id;
            frame.dlc = received.dlc;
            frame.data = received.data;
            frame.extended = received.extended;
            frame.remote = received.remote;
            const MotorBusStatus feedback = bus.updateFeedback(frame, micros());
            if (feedback != MotorBusStatus::Ok &&
                feedback != MotorBusStatus::InvalidCanId &&
                feedback != MotorBusStatus::UnknownMotor) stopOnFault();
        }
        const board::Esp32CanHealth before = can.health();
        const Esp32CanStatus healthStatus = can.pollHealth();
        const board::Esp32CanHealth &after = can.health();
        if (after.missedFrames != before.missedFrames ||
            after.overrunFrames != before.overrunFrames ||
            after.busErrors != before.busErrors ||
            after.queueFullAlerts != before.queueFullAlerts) {
            bus.notifyFeedbackLoss();
        }
        if (healthStatus != Esp32CanStatus::Ok) stopOnFault();
    }
    const uint32_t nowUs = micros();
    if (!faulted && !ready) {
        if (motor.state().hasFeedback()) {
            if (motor.setPosition(0.0) != MotorStatus::Ok) stopOnFault();
            else ready = true;
        } else if (nowUs - waitingSinceUs >= 2000000U) {
            stopOnFault();
        }
    }
    if (!ready && Serial.available()) {
        if (Serial.read() == '!') stopOnFault();
    }
    if (ready && !faulted) updateTarget(nowUs);
    if (!faulted && bus.updateControl(nowUs) != MotorBusStatus::Ok) stopOnFault();
    // Keep sending zero-current frames after a fault; never replay an old target.
    if (can.started() && bus.commandFramesDue(nowUs)) {
        std::array<CanFrame, MotorBus::kMaxCommandFrames> frames{};
        std::size_t count = 0U;
        bool sent = bus.makeCommandFrames(frames, count) == MotorBusStatus::Ok;
        for (std::size_t i = 0; sent && i < count; ++i) {
            board::CanFrame outgoing;
            outgoing.id = frames[i].id;
            outgoing.dlc = frames[i].dlc;
            outgoing.data = frames[i].data;
            outgoing.extended = frames[i].extended;
            outgoing.remote = frames[i].remote;
            sent = can.send(outgoing) == Esp32CanStatus::Ok;
        }
        if (sent) bus.markCommandFramesSent(nowUs);
        else stopOnFault();
    }
}

// 1/2/3 select position/speed/current and coast until the next target key.
// q/a/z = +90/0/-90 deg; w/s/x = +30/0/-30 rpm; e/d/c = +0.5/0/-0.5 A.
// Space: coast (zero current). !: latched emergency stop in any startup state.
ControlType selected = ControlType::Speed;
void updateTarget(uint32_t nowUs) {
    if (!Serial.available()) return; // Handle one key per loop; keep servicing CAN.
    const char key = static_cast<char>(Serial.read());
    if (key == '!') { stopOnFault(); return; }
    if (key == '1' || key == '2' || key == '3' || key == ' ') {
        if (key == '1') selected = ControlType::Position;
        if (key == '2') selected = ControlType::Speed;
        if (key == '3') selected = ControlType::Current;
        if (motor.coast(nowUs) != MotorStatus::Ok) stopOnFault();
        return;
    }
    double value = 0.0;
    if (selected == ControlType::Position) {
        if (key == 'q') value = 90.0;
        else if (key == 'z') value = -90.0;
        else if (key != 'a') return;
    } else if (selected == ControlType::Speed) {
        if (key == 'w') value = 30.0;
        else if (key == 'x') value = -30.0;
        else if (key != 's') return;
    } else {
        if (key == 'e') value = 0.5;
        else if (key == 'c') value = -0.5;
        else if (key != 'd') return;
    }
    if (motor.setTarget(selected, value, nowUs) != MotorStatus::Ok) stopOnFault();
}
