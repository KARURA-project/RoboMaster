#pragma once

#include <cstdint>

#include <array>

namespace board {

// Generic Classic CAN frame; independent of motor protocols and libraries.
struct CanFrame {
    std::uint32_t id = 0U;
    std::uint8_t dlc = 0U;
    std::array<std::uint8_t, 8> data{};
    bool extended = false;
    bool remote = false;
};

struct Esp32CanConfig {
    constexpr Esp32CanConfig(
        int txPin = 1,
        int rxPin = 2,
        std::uint16_t txQueueLength = 8U,
        std::uint16_t rxQueueLength = 64U)
        : txPin(txPin), rxPin(rxPin),
          txQueueLength(txQueueLength), rxQueueLength(rxQueueLength)
    {
    }

    int txPin;
    int rxPin;
    std::uint16_t txQueueLength;
    std::uint16_t rxQueueLength;
};

struct Esp32CanHealth {
    std::uint32_t receivedFrames = 0U;
    std::uint32_t transmittedFrames = 0U;
    std::uint32_t transmitFailures = 0U;
    std::uint32_t missedFrames = 0U;
    std::uint32_t overrunFrames = 0U;
    std::uint32_t busErrors = 0U;
    std::uint32_t queueFullAlerts = 0U;
    std::uint32_t errorPassiveAlerts = 0U;
    std::uint32_t busOffAlerts = 0U;
};

enum class Esp32CanStatus : std::uint8_t {
    Ok,
    AlreadyStarted,
    NotStarted,
    InvalidConfiguration,
    InstallFailed,
    StartFailed,
    StopFailed,
    ReceiveFailed,
    TransmitFailed,
    NoFrame,
    InvalidFrame,
    FeedbackLost,
    ErrorPassive,
    BusOff,
};

class Esp32Can {
public:
    explicit Esp32Can(const Esp32CanConfig &config = Esp32CanConfig{});
    Esp32Can(const Esp32Can &) = delete;
    Esp32Can &operator=(const Esp32Can &) = delete;

    Esp32CanStatus begin();
    Esp32CanStatus end();
    Esp32CanStatus receive(CanFrame &frame);
    Esp32CanStatus pollHealth();
    Esp32CanStatus send(const CanFrame &frame);

    bool started() const;
    const Esp32CanConfig &config() const;
    const Esp32CanHealth &health() const;
    void clearHealth();

private:

    Esp32CanConfig config_;
    Esp32CanHealth health_{};
    bool started_ = false;
    std::uint32_t previousMissedFrames_ = 0U;
    std::uint32_t previousOverrunFrames_ = 0U;
    std::uint32_t previousBusErrors_ = 0U;
};

}  // namespace board
