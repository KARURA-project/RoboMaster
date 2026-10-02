#include <RoboMasterCore.h>

int main()
{
    robomaster::Motor motor{
        1,
        robomaster::MotorModel::M3508,
        robomaster::ControllerModel::C620};
    return motor.configurationStatus() == robomaster::MotorStatus::Ok
        ? 0 : 1;
}
