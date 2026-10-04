#include <RoboMasterCore.h>
#include "Esp32.hpp"

using namespace robomaster;

namespace {

constexpr uint32_t kTestDurationUs = 10000000U;

Motor motor{1, MotorModel::M3508, ControllerModel::C620};
MotorBus bus;
Esp32Can can{Esp32CanConfig{1, 2, 8, 64}};
Esp32Timer timer;

uint32_t startedAtUs = 0U;
uint32_t maximumPending = 0U;
uint32_t skippedTicks = 0U;
bool finished = false;

void stopWithError(const char *message)
{
    Serial.print("ERROR: ");
    Serial.println(message);
    while (true) {
        delay(1000);
    }
}

void finishTest()
{
    finished = true;
    timer.end();
    can.receive(bus);
    can.end();

    const uint32_t elapsedUs = micros() - startedAtUs;
    const Esp32CanHealth &health = can.health();

    Serial.println();
    Serial.println("=== RoboMaster library zero-current test ===");
    Serial.print("Elapsed: ");
    Serial.print(static_cast<double>(elapsedUs) / 1000000.0, 6);
    Serial.println(" s");
    Serial.print("Timer ticks: ");
    Serial.println(timer.totalTicks());
    Serial.print("Maximum pending ticks: ");
    Serial.println(maximumPending);
    Serial.print("Skipped ticks: ");
    Serial.println(skippedTicks);
    Serial.print("RX feedback frames: ");
    Serial.println(health.receivedFrames);
    Serial.print("TX command frames: ");
    Serial.println(health.transmittedFrames);
    Serial.print("TX failures: ");
    Serial.println(health.transmitFailures);
    Serial.print("RX missed: ");
    Serial.println(health.missedFrames);
    Serial.print("RX overrun: ");
    Serial.println(health.overrunFrames);
    Serial.print("Bus errors: ");
    Serial.println(health.busErrors);
    Serial.print("Position flags: 0x");
    Serial.println(motor.state().positionFlags(), HEX);
    Serial.println("Reset the board to run again.");
}

}  // namespace

void setup()
{
    Serial.begin(115200);
    const uint32_t waitStartedAtMs = millis();
    while (!Serial && millis() - waitStartedAtMs < 3000U) {
        delay(10);
    }

    Serial.println();
    Serial.println("XIAO ESP32S3 RoboMaster library test");
    Serial.println("Motor ID 1, zero current, 10 seconds");
    Serial.println("Starting quiet measurement...");
    Serial.flush();
    delay(100);

    if (bus.add(motor) != MotorBusStatus::Ok) {
        stopWithError("bus.add() failed");
    }
    if (motor.coast(micros()) != MotorStatus::Ok) {
        stopWithError("motor.coast() failed");
    }
    if (can.begin() != Esp32CanStatus::Ok) {
        stopWithError("can.begin() failed");
    }
    if (timer.begin(1000U) != Esp32TimerStatus::Ok) {
        can.end();
        stopWithError("timer.begin() failed");
    }

    startedAtUs = micros();
}

void loop()
{
    if (finished) {
        delay(1000);
        return;
    }

    const Esp32CanStatus receiveStatus = can.receive(bus);
    if (receiveStatus == Esp32CanStatus::BusOff ||
        receiveStatus == Esp32CanStatus::ErrorPassive) {
        motor.emergencyStop();
    }

    const uint32_t pending = timer.takePending();
    if (pending > maximumPending) {
        maximumPending = pending;
    }
    if (pending > 1U) {
        skippedTicks += pending - 1U;
    }

    if (pending > 0U) {
        const uint32_t nowUs = micros();
        bus.updateControl(nowUs);
        can.send(bus);
        bus.markCommandFramesSent(nowUs);
    }

    if (micros() - startedAtUs >= kTestDurationUs) {
        finishTest();
    }
}
