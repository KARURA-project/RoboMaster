#include <Arduino.h>
#include <CANBridge/EspCan.h>
#include <RoboMasterCore.h>

#include <array>

robomaster::Motor motor{
    1,
    robomaster::MotorModel::M3508,
    robomaster::ControllerModel::C620};
robomaster::MotorBus motorBus;
canbridge::Config canConfig;
canbridge::Bus canBus;

bool ready = false;
bool faulted = false;
uint32_t waitingSinceUs = 0;

std::array<canbridge::Frame, robomaster::MotorBus::kMaxCommandFrames>
    pendingFrames{};
std::size_t pendingCount = 0;
std::size_t pendingIndex = 0;

void stopOnFault()
{
    faulted = true;
    motorBus.emergencyStopAll();
    pendingCount = 0;
    pendingIndex = 0;
}

void receiveFeedback()
{
    for (unsigned int i = 0; i < 64U; ++i) {
        canbridge::Frame frame;
        const canbridge::Result result = canBus.receive(frame);
        if (result == canbridge::Result::Empty) {
            break;
        }
        if (result != canbridge::Result::Ok) {
            stopOnFault();
            return;
        }

        const robomaster::MotorBusStatus status =
            motorBus.updateFeedback(frame, micros());
        if (status != robomaster::MotorBusStatus::Ok &&
            status != robomaster::MotorBusStatus::InvalidCanId &&
            status != robomaster::MotorBusStatus::UnknownMotor) {
            stopOnFault();
            return;
        }
    }

    canbridge::Health health;
    if (canBus.pollHealth(health) != canbridge::Result::Ok) {
        stopOnFault();
        return;
    }
    if (health.receiveLoss) {
        motorBus.notifyFeedbackLoss();
        stopOnFault();
    }
    if (health.busOff || health.errorPassive) {
        stopOnFault();
    }
}

void sendCommands(uint32_t nowUs)
{
    if (pendingIndex == pendingCount) {
        if (!motorBus.commandFramesDue(nowUs)) {
            return;
        }
        if (motorBus.makeCommandFrames(pendingFrames, pendingCount) !=
            robomaster::MotorBusStatus::Ok) {
            stopOnFault();
            return;
        }
        pendingIndex = 0;
    }

    while (pendingIndex < pendingCount) {
        const canbridge::Result result = canBus.send(pendingFrames[pendingIndex]);
        if (result == canbridge::Result::Busy) {
            return;
        }
        if (result != canbridge::Result::Ok) {
            stopOnFault();
            return;
        }
        ++pendingIndex;
    }

    pendingCount = 0;
    pendingIndex = 0;
    motorBus.markCommandFramesSent(nowUs);
}

void updateTarget(uint32_t nowUs)
{
    static const double targets[] = {30.0, 0.0, -30.0, 0.0};
    static bool started = false;
    static uint8_t phase = 0;
    static uint32_t changedAtUs = 0;

    if (started && nowUs - changedAtUs < 2000000U) {
        return;
    }
    if (started) {
        phase = (phase + 1U) % 4U;
    }
    started = true;
    changedAtUs = nowUs;
    if (motor.setTarget(
            robomaster::ControlType::Speed,
            targets[phase],
            nowUs) != robomaster::MotorStatus::Ok) {
        stopOnFault();
    }
}

void setup()
{
    Serial.begin(115200);

    canConfig.bitrate = 1000000;
    canConfig.txPin = D0;
    canConfig.rxPin = D1;

    if (motorBus.add(motor) != robomaster::MotorBusStatus::Ok ||
        motor.setMaxCurrent(2.0) != robomaster::MotorStatus::Ok ||
        motor.setMaxSpeed(60.0) != robomaster::MotorStatus::Ok ||
        motor.setTrackingMaxSpeed(500.0) != robomaster::MotorStatus::Ok ||
        motor.setExpectedFeedbackRate(robomaster::FeedbackRate::Hz1000) !=
            robomaster::MotorStatus::Ok ||
        motor.setSpeedGains(robomaster::PidGains{0.10, 0.10, 0.0}) !=
            robomaster::MotorStatus::Ok ||
        motor.setPositionGains(robomaster::PidGains{0.50, 0.0, 0.0}) !=
            robomaster::MotorStatus::Ok ||
        motor.coast(micros()) != robomaster::MotorStatus::Ok ||
        canBus.begin(canConfig) != canbridge::Result::Ok) {
        stopOnFault();
    }

    waitingSinceUs = micros();
}

void loop()
{
    receiveFeedback();

    const uint32_t nowUs = micros();
    if (!faulted && !ready) {
        if (motor.state().hasFeedback()) {
            if (motor.setPosition(0.0) == robomaster::MotorStatus::Ok) {
                ready = true;
            } else {
                stopOnFault();
            }
        } else if (nowUs - waitingSinceUs >= 2000000U) {
            stopOnFault();
        }
    }

    if (ready && !faulted) {
        updateTarget(nowUs);
    }
    if (!faulted &&
        motorBus.updateControl(nowUs) != robomaster::MotorBusStatus::Ok) {
        stopOnFault();
    }
    sendCommands(nowUs);
}
