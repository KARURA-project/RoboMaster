#include "Esp32Timer.hpp"

namespace robomaster {

#if ESP_ARDUINO_VERSION_MAJOR < 3
Esp32Timer *Esp32Timer::activeTimer_ = nullptr;
#endif

Esp32TimerStatus Esp32Timer::begin(std::uint32_t periodUs)
{
    if (timer_ != nullptr) {
        return Esp32TimerStatus::AlreadyStarted;
    }
    if (periodUs == 0U) {
        return Esp32TimerStatus::InvalidPeriod;
    }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timer_ = timerBegin(1000000U);
#else
    if (activeTimer_ != nullptr) {
        return Esp32TimerStatus::AllocationFailed;
    }
    timer_ = timerBegin(0U, 80U, true);
#endif
    if (timer_ == nullptr) {
        return Esp32TimerStatus::AllocationFailed;
    }

    portENTER_CRITICAL(&mux_);
    pending_ = 0U;
    totalTicks_ = 0U;
    portEXIT_CRITICAL(&mux_);
    periodUs_ = periodUs;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timerAttachInterruptArg(timer_, &Esp32Timer::handleInterrupt, this);
    timerAlarm(timer_, periodUs_, true, 0U);
#else
    activeTimer_ = this;
    timerAttachInterrupt(timer_, &Esp32Timer::handleInterrupt, false);
    timerAlarmWrite(timer_, periodUs_, true);
    timerAlarmEnable(timer_);
#endif
    return Esp32TimerStatus::Ok;
}

Esp32TimerStatus Esp32Timer::end()
{
    if (timer_ == nullptr) {
        return Esp32TimerStatus::NotStarted;
    }

    timerStop(timer_);
    timerDetachInterrupt(timer_);
    timerEnd(timer_);
#if ESP_ARDUINO_VERSION_MAJOR < 3
    activeTimer_ = nullptr;
#endif
    timer_ = nullptr;
    periodUs_ = 0U;
    return Esp32TimerStatus::Ok;
}

std::uint32_t Esp32Timer::takePending()
{
    portENTER_CRITICAL(&mux_);
    const std::uint32_t result = pending_;
    pending_ = 0U;
    portEXIT_CRITICAL(&mux_);
    return result;
}

std::uint32_t Esp32Timer::pending() const
{
    portENTER_CRITICAL(&mux_);
    const std::uint32_t result = pending_;
    portEXIT_CRITICAL(&mux_);
    return result;
}

std::uint32_t Esp32Timer::totalTicks() const
{
    portENTER_CRITICAL(&mux_);
    const std::uint32_t result = totalTicks_;
    portEXIT_CRITICAL(&mux_);
    return result;
}

std::uint32_t Esp32Timer::periodUs() const
{
    return periodUs_;
}

bool Esp32Timer::started() const
{
    return timer_ != nullptr;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void ARDUINO_ISR_ATTR Esp32Timer::handleInterrupt(void *argument)
{
    static_cast<Esp32Timer *>(argument)->onInterrupt();
}
#else
void ARDUINO_ISR_ATTR Esp32Timer::handleInterrupt()
{
    if (activeTimer_ != nullptr) {
        activeTimer_->onInterrupt();
    }
}
#endif

void ARDUINO_ISR_ATTR Esp32Timer::onInterrupt()
{
    portENTER_CRITICAL_ISR(&mux_);
    if (pending_ != UINT32_MAX) {
        ++pending_;
    }
    ++totalTicks_;
    portEXIT_CRITICAL_ISR(&mux_);
}

}  // namespace robomaster
