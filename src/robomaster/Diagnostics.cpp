#include "Diagnostics.hpp"

namespace robomaster {

const char *toString(ProtocolStatus status)
{
    switch (status) {
        case ProtocolStatus::Ok: return "Ok";
        case ProtocolStatus::InvalidId: return "InvalidId";
        case ProtocolStatus::InvalidCanId: return "InvalidCanId";
        case ProtocolStatus::InvalidFrameType: return "InvalidFrameType";
        case ProtocolStatus::InvalidFrameLength: return "InvalidFrameLength";
        case ProtocolStatus::InvalidEncoder: return "InvalidEncoder";
        case ProtocolStatus::InvalidCurrent: return "InvalidCurrent";
        case ProtocolStatus::InvalidModel: return "InvalidModel";
    }
    return "Unknown";
}

const char *toString(PidStatus status)
{
    switch (status) {
        case PidStatus::Ok: return "Ok";
        case PidStatus::InvalidGains: return "InvalidGains";
        case PidStatus::InvalidLimits: return "InvalidLimits";
        case PidStatus::InvalidInput: return "InvalidInput";
        case PidStatus::InvalidTimeStep: return "InvalidTimeStep";
        case PidStatus::InvalidDerivativeSource: return "InvalidDerivativeSource";
        case PidStatus::CalculationOverflow: return "CalculationOverflow";
    }
    return "Unknown";
}

const char *toString(MotorStateStatus status)
{
    switch (status) {
        case MotorStateStatus::Ok: return "Ok";
        case MotorStateStatus::InvalidId: return "InvalidId";
        case MotorStateStatus::InvalidModel: return "InvalidModel";
        case MotorStateStatus::InvalidCanId: return "InvalidCanId";
        case MotorStateStatus::InvalidFrameType: return "InvalidFrameType";
        case MotorStateStatus::InvalidFrameLength: return "InvalidFrameLength";
        case MotorStateStatus::InvalidEncoder: return "InvalidEncoder";
        case MotorStateStatus::InvalidFeedbackRate: return "InvalidFeedbackRate";
        case MotorStateStatus::InvalidMaxRpm: return "InvalidMaxRpm";
        case MotorStateStatus::InvalidPosition: return "InvalidPosition";
        case MotorStateStatus::ProtocolError: return "ProtocolError";
    }
    return "Unknown";
}

const char *toString(FeedbackTimingStatus status)
{
    switch (status) {
        case FeedbackTimingStatus::NoData: return "NoData";
        case FeedbackTimingStatus::Measuring: return "Measuring";
        case FeedbackTimingStatus::Ready: return "Ready";
        case FeedbackTimingStatus::Mismatch: return "Mismatch";
        case FeedbackTimingStatus::Unstable: return "Unstable";
    }
    return "Unknown";
}

const char *toString(MotorStatus status)
{
    switch (status) {
        case MotorStatus::Ok: return "Ok";
        case MotorStatus::NoTarget: return "NoTarget";
        case MotorStatus::InvalidConfiguration: return "InvalidConfiguration";
        case MotorStatus::InvalidGearRatio: return "InvalidGearRatio";
        case MotorStatus::InvalidDirection: return "InvalidDirection";
        case MotorStatus::InvalidMaxSpeed: return "InvalidMaxSpeed";
        case MotorStatus::InvalidTrackingMaxSpeed: return "InvalidTrackingMaxSpeed";
        case MotorStatus::InvalidMaxCurrent: return "InvalidMaxCurrent";
        case MotorStatus::InvalidControlPeriod: return "InvalidControlPeriod";
        case MotorStatus::InvalidTimeout: return "InvalidTimeout";
        case MotorStatus::InvalidFeedbackRate: return "InvalidFeedbackRate";
        case MotorStatus::InvalidPidGains: return "InvalidPidGains";
        case MotorStatus::InvalidControlType: return "InvalidControlType";
        case MotorStatus::InvalidTarget: return "InvalidTarget";
        case MotorStatus::InvalidFeedback: return "InvalidFeedback";
        case MotorStatus::TargetOutOfRange: return "TargetOutOfRange";
        case MotorStatus::FeedbackUnavailable: return "FeedbackUnavailable";
        case MotorStatus::FeedbackRateUnavailable: return "FeedbackRateUnavailable";
        case MotorStatus::FeedbackTimeout: return "FeedbackTimeout";
        case MotorStatus::PositionUnreliable: return "PositionUnreliable";
        case MotorStatus::CommandTimeout: return "CommandTimeout";
        case MotorStatus::EmergencyStopped: return "EmergencyStopped";
        case MotorStatus::PidError: return "PidError";
    }
    return "Unknown";
}

const char *toString(MotorBusStatus status)
{
    switch (status) {
        case MotorBusStatus::Ok: return "Ok";
        case MotorBusStatus::InvalidMotor: return "InvalidMotor";
        case MotorBusStatus::DuplicateId: return "DuplicateId";
        case MotorBusStatus::DuplicateMotor: return "DuplicateMotor";
        case MotorBusStatus::BusFull: return "BusFull";
        case MotorBusStatus::InvalidCanId: return "InvalidCanId";
        case MotorBusStatus::UnknownMotor: return "UnknownMotor";
        case MotorBusStatus::InvalidSendPeriod: return "InvalidSendPeriod";
        case MotorBusStatus::MotorError: return "MotorError";
        case MotorBusStatus::ProtocolError: return "ProtocolError";
    }
    return "Unknown";
}

}  // namespace robomaster
