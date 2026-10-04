// This must compile even when a board macro is present and no board SDK exists.
#include <RoboMasterCore.h>
#include <array>
#include <cassert>

int main()
{
    using namespace robomaster;
    Motor motor{1, MotorModel::M3508, ControllerModel::C620};
    MotorBus bus;
    assert(bus.add(motor) == MotorBusStatus::Ok);
    assert(motor.coast(0U) == MotorStatus::Ok);
    assert(bus.updateControl(1000U) == MotorBusStatus::Ok);
    std::array<canbridge::Frame, MotorBus::kMaxCommandFrames> frames{};
    std::size_t count = 0U;
    assert(bus.makeCommandFrames(frames, count) == MotorBusStatus::Ok);
    assert(count == 1U);
    bus.notifyFeedbackLoss();
    assert(!motor.hasTarget());
    assert(motor.commandCurrent() == 0.0);
    bus.emergencyStopAll();
    assert(motor.emergencyStopped());
    bus.clearEmergencyStopAll();
    assert(!motor.hasTarget());
    return 0;
}
