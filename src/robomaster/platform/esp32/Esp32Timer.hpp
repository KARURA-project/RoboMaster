#pragma once

#include <Arduino.h>
#include <esp_arduino_version.h>

#include <cstdint>

namespace robomaster {

enum class Esp32TimerStatus : std::uint8_t {
    Ok,
    AlreadyStarted,
    NotStarted,
    InvalidPeriod,
    AllocationFailed,
};

class Esp32Timer {
public:
    Esp32Timer() = default;
    Esp32Timer(const Esp32Timer &) = delete;
    Esp32Timer &operator=(const Esp32Timer &) = delete;

    Esp32TimerStatus begin(std::uint32_t periodUs = 1000U);
    Esp32TimerStatus end();

    std::uint32_t takePending();
    std::uint32_t pending() const;
    std::uint32_t totalTicks() const;
    std::uint32_t periodUs() const;
    bool started() const;

private:
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    static void ARDUINO_ISR_ATTR handleInterrupt(void *argument);
#else
    static void ARDUINO_ISR_ATTR handleInterrupt();
    static Esp32Timer *activeTimer_;
#endif
    void ARDUINO_ISR_ATTR onInterrupt();

    hw_timer_t *timer_ = nullptr;
    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    volatile std::uint32_t pending_ = 0U;
    volatile std::uint32_t totalTicks_ = 0U;
    std::uint32_t periodUs_ = 0U;
};

}  // namespace robomaster
