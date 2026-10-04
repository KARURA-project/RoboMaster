#include "Esp32Can.hpp"

#include <Arduino.h>
#include <driver/twai.h>

#include <array>
#include <cstddef>

namespace robomaster {
namespace {

CanFrame toCanFrame(const twai_message_t &message)
{
    CanFrame frame;
    frame.id = message.identifier;
    frame.dlc = message.data_length_code;
    frame.extended = message.extd != 0U;
    frame.remote = message.rtr != 0U;
    const std::size_t count = frame.dlc < frame.data.size()
        ? frame.dlc : frame.data.size();
    for (std::size_t i = 0U; i < count; ++i) {
        frame.data[i] = message.data[i];
    }
    return frame;
}

twai_message_t toTwaiMessage(const CanFrame &frame)
{
    twai_message_t message{};
    message.identifier = frame.id;
    message.data_length_code = frame.dlc;
    message.extd = frame.extended ? 1U : 0U;
    message.rtr = frame.remote ? 1U : 0U;
    const std::size_t count = frame.dlc < kCanPayloadSize
        ? frame.dlc : kCanPayloadSize;
    for (std::size_t i = 0U; i < count; ++i) {
        message.data[i] = frame.data[i];
    }
    return message;
}

}  // namespace

Esp32Can::Esp32Can(const Esp32CanConfig &config) : config_(config)
{
}

Esp32CanStatus Esp32Can::begin()
{
    if (started_) {
        return Esp32CanStatus::AlreadyStarted;
    }
    if (config_.txPin < 0 || config_.rxPin < 0 ||
        config_.txQueueLength == 0U || config_.rxQueueLength == 0U) {
        return Esp32CanStatus::InvalidConfiguration;
    }

    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        static_cast<gpio_num_t>(config_.txPin),
        static_cast<gpio_num_t>(config_.rxPin), TWAI_MODE_NORMAL);
    general.tx_queue_len = config_.txQueueLength;
    general.rx_queue_len = config_.rxQueueLength;
    general.alerts_enabled = TWAI_ALERT_RX_QUEUE_FULL |
        TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF;

    const twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
    const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&general, &timing, &filter) != ESP_OK) {
        return Esp32CanStatus::InstallFailed;
    }
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return Esp32CanStatus::StartFailed;
    }

    started_ = true;
    clearHealth();
    twai_status_info_t status{};
    if (twai_get_status_info(&status) == ESP_OK) {
        previousMissedFrames_ = status.rx_missed_count;
        previousOverrunFrames_ = status.rx_overrun_count;
        previousBusErrors_ = status.bus_error_count;
    }
    return Esp32CanStatus::Ok;
}

Esp32CanStatus Esp32Can::end()
{
    if (!started_) {
        return Esp32CanStatus::NotStarted;
    }
    const esp_err_t stopResult = twai_stop();
    const esp_err_t uninstallResult = twai_driver_uninstall();
    started_ = false;
    return stopResult == ESP_OK && uninstallResult == ESP_OK
        ? Esp32CanStatus::Ok : Esp32CanStatus::StopFailed;
}

Esp32CanStatus Esp32Can::receive(MotorBus &bus)
{
    if (!started_) {
        return Esp32CanStatus::NotStarted;
    }

    Esp32CanStatus result = Esp32CanStatus::Ok;
    twai_message_t message{};
    esp_err_t receiveResult = ESP_OK;
    while ((receiveResult = twai_receive(&message, 0U)) == ESP_OK) {
        const CanFrame frame = toCanFrame(message);
        if (frame.id < kFeedbackIdFirst || frame.id > kFeedbackIdLast) {
            ++health_.ignoredFrames;
            continue;
        }
        ++health_.receivedFrames;
        const MotorBusStatus busStatus = bus.updateFeedback(frame, micros());
        if (busStatus != MotorBusStatus::Ok &&
            busStatus != MotorBusStatus::UnknownMotor) {
            result = Esp32CanStatus::MotorError;
        }
    }
    if (receiveResult != ESP_ERR_TIMEOUT) {
        return Esp32CanStatus::ReceiveFailed;
    }

    const Esp32CanStatus healthStatus = updateDriverHealth(bus);
    return healthStatus == Esp32CanStatus::Ok ? result : healthStatus;
}

Esp32CanStatus Esp32Can::send(const CanFrame &frame)
{
    if (!started_) {
        return Esp32CanStatus::NotStarted;
    }
    const twai_message_t message = toTwaiMessage(frame);
    if (twai_transmit(&message, 0U) != ESP_OK) {
        ++health_.transmitFailures;
        return Esp32CanStatus::TransmitFailed;
    }
    ++health_.transmittedFrames;
    return Esp32CanStatus::Ok;
}

Esp32CanStatus Esp32Can::send(const MotorBus &bus)
{
    std::array<CanFrame, MotorBus::kMaxCommandFrames> frames{};
    std::size_t count = 0U;
    if (bus.makeCommandFrames(frames, count) != MotorBusStatus::Ok) {
        return Esp32CanStatus::MotorError;
    }
    for (std::size_t i = 0U; i < count; ++i) {
        const Esp32CanStatus status = send(frames[i]);
        if (status != Esp32CanStatus::Ok) {
            return status;
        }
    }
    return Esp32CanStatus::Ok;
}

bool Esp32Can::started() const
{
    return started_;
}

const Esp32CanConfig &Esp32Can::config() const
{
    return config_;
}

const Esp32CanHealth &Esp32Can::health() const
{
    return health_;
}

void Esp32Can::clearHealth()
{
    health_ = Esp32CanHealth{};
}

Esp32CanStatus Esp32Can::updateDriverHealth(MotorBus &bus)
{
    twai_status_info_t status{};
    if (twai_get_status_info(&status) != ESP_OK) {
        return Esp32CanStatus::ReceiveFailed;
    }

    const std::uint32_t missed = status.rx_missed_count - previousMissedFrames_;
    const std::uint32_t overrun =
        status.rx_overrun_count - previousOverrunFrames_;
    const std::uint32_t busErrors =
        status.bus_error_count - previousBusErrors_;
    previousMissedFrames_ = status.rx_missed_count;
    previousOverrunFrames_ = status.rx_overrun_count;
    previousBusErrors_ = status.bus_error_count;
    health_.missedFrames += missed;
    health_.overrunFrames += overrun;
    health_.busErrors += busErrors;

    uint32_t alerts = 0U;
    if (twai_read_alerts(&alerts, 0U) != ESP_OK) {
        alerts = 0U;
    }
    if ((alerts & TWAI_ALERT_RX_QUEUE_FULL) != 0U) {
        ++health_.queueFullAlerts;
    }
    if ((alerts & TWAI_ALERT_ERR_PASS) != 0U) {
        ++health_.errorPassiveAlerts;
    }
    if ((alerts & TWAI_ALERT_BUS_OFF) != 0U) {
        ++health_.busOffAlerts;
    }

    if (status.state == TWAI_STATE_BUS_OFF ||
        (alerts & TWAI_ALERT_BUS_OFF) != 0U) {
        bus.emergencyStopAll();
        return Esp32CanStatus::BusOff;
    }
    if (missed > 0U || overrun > 0U || busErrors > 0U) {
        bus.notifyFeedbackLoss();
        return Esp32CanStatus::FeedbackLost;
    }
    if ((alerts & TWAI_ALERT_ERR_PASS) != 0U) {
        bus.emergencyStopAll();
        return Esp32CanStatus::ErrorPassive;
    }
    return Esp32CanStatus::Ok;
}

}  // namespace robomaster
