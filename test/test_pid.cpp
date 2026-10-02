#include <cassert>
#include <cmath>
#include <limits>

#include "robomaster/Pid.hpp"

using namespace robomaster;

namespace {

bool near(double actual, double expected)
{
    return std::fabs(actual - expected) < 1e-12;
}

void testSafeDefaults()
{
    Pid pid;
    const PidResult result = pid.update(100.0, 0.0, 0.001);
    assert(result.status == PidStatus::Ok);
    assert(!result.saturated);
    assert(result.output == 0.0);
}

void testProportionalAndOutputLimits()
{
    Pid pid{PidConfig{
        PidGains{2.0, 0.0, 0.0},
        PidLimits{-10.0, 10.0}}};

    PidResult result = pid.update(3.0, 1.0, 0.01);
    assert(result.status == PidStatus::Ok);
    assert(!result.saturated);
    assert(near(result.output, 4.0));

    result = pid.update(10.0, 0.0, 0.01);
    assert(result.saturated);
    assert(near(result.output, 10.0));
}

void testConditionalIntegration()
{
    Pid pid{PidConfig{
        PidGains{0.0, 1.0, 0.0},
        PidLimits{-1.0, 1.0}}};

    PidResult result = pid.update(2.0, 0.0, 1.0);
    assert(result.saturated);
    assert(near(result.output, 1.0));
    assert(near(pid.integralOutput(), 1.0));

    result = pid.update(2.0, 0.0, 1.0);
    assert(result.saturated);
    assert(near(result.output, 1.0));
    assert(near(pid.integralOutput(), 1.0));

    result = pid.update(0.0, 0.5, 1.0);
    assert(!result.saturated);
    assert(near(result.output, 0.5));
    assert(near(pid.integralOutput(), 0.5));
}

void testIntegralUsesOutputLimits()
{
    Pid pid{PidConfig{
        PidGains{0.0, 1.0, 0.0},
        PidLimits{-0.25, 0.25}}};

    const PidResult result = pid.update(1.0, 0.0, 1.0);
    assert(result.saturated);
    assert(near(result.output, 0.25));
    assert(near(pid.integralOutput(), 0.25));
}

void testIntegralRespectsRemainingOutputRange()
{
    Pid pid{PidConfig{
        PidGains{0.75, 1.0, 0.0},
        PidLimits{-1.0, 1.0}}};

    PidResult result = pid.update(1.0, 0.0, 1.0);
    assert(result.saturated);
    assert(near(result.output, 1.0));
    assert(near(pid.integralOutput(), 0.25));

    result = pid.update(1.0, 0.0, 1.0);
    assert(result.saturated);
    assert(near(result.output, 1.0));
    assert(near(pid.integralOutput(), 0.25));

    result = pid.update(0.0, 1.0, 1.0);
    assert(result.saturated);
    assert(near(result.output, -1.0));
    assert(near(pid.integralOutput(), -0.25));
}

void testDerivativeOnMeasurementAvoidsSetpointKick()
{
    Pid pid{PidConfig{
        PidGains{0.0, 0.0, 2.0},
        PidLimits{-100.0, 100.0}}};

    PidResult result = pid.update(10.0, 0.0, 1.0);
    assert(near(result.output, 0.0));

    result = pid.update(12.0, 0.0, 1.0);
    assert(near(result.output, 0.0));

    result = pid.update(12.0, 2.0, 1.0);
    assert(near(result.output, -4.0));
}

void testDerivativeOnErrorCanBeSelected()
{
    Pid pid{PidConfig{
        PidGains{0.0, 0.0, 2.0},
        PidLimits{-100.0, 100.0},
        DerivativeSource::Error}};

    pid.update(10.0, 0.0, 1.0);
    const PidResult result = pid.update(12.0, 0.0, 1.0);
    assert(near(result.output, 4.0));
}

void testInvalidValuesProduceSafeOutput()
{
    Pid pid{PidConfig{
        PidGains{1.0, 1.0, 1.0},
        PidLimits{-10.0, 10.0}}};

    PidResult result = pid.update(
        std::numeric_limits<double>::quiet_NaN(),
        0.0,
        0.01);
    assert(result.status == PidStatus::InvalidInput);
    assert(result.output == 0.0);

    result = pid.update(1.0, 0.0, 0.0);
    assert(result.status == PidStatus::InvalidTimeStep);
    assert(result.output == 0.0);
}

void testRejectsInvalidConfiguration()
{
    Pid pid;
    assert(pid.setGains(PidGains{
        std::numeric_limits<double>::infinity(), 0.0, 0.0}) ==
        PidStatus::InvalidGains);
    assert(pid.setLimits(PidLimits{1.0, -1.0}) ==
        PidStatus::InvalidLimits);

    Pid invalid{PidConfig{
        PidGains{1.0, 0.0, 0.0},
        PidLimits{1.0, 2.0}}};
    assert(invalid.configurationStatus() == PidStatus::InvalidLimits);
    const PidResult result = invalid.update(1.0, 0.0, 1.0);
    assert(result.status == PidStatus::InvalidLimits);
    assert(result.output == 0.0);

    const DerivativeSource invalidSource =
        static_cast<DerivativeSource>(0xFF);
    assert(pid.configure(PidConfig{
        PidGains{}, PidLimits{}, invalidSource}) ==
        PidStatus::InvalidDerivativeSource);
    assert(pid.configurationStatus() ==
        PidStatus::InvalidDerivativeSource);
}

void testCalculationOverflowProducesSafeOutput()
{
    Pid pid{PidConfig{
        PidGains{std::numeric_limits<double>::max(), 0.0, 0.0},
        PidLimits{-10.0, 10.0}}};
    const PidResult result = pid.update(
        std::numeric_limits<double>::max(),
        -std::numeric_limits<double>::max(),
        1.0);
    assert(result.status == PidStatus::CalculationOverflow);
    assert(result.output == 0.0);
}

void testReset()
{
    Pid pid{PidConfig{
        PidGains{0.0, 1.0, 0.0},
        PidLimits{-10.0, 10.0}}};
    pid.update(1.0, 0.0, 1.0);
    assert(near(pid.integralOutput(), 1.0));

    pid.reset();
    assert(pid.integralOutput() == 0.0);
    assert(pid.lastOutput() == 0.0);
}

void testReconfigureResetsState()
{
    Pid pid{PidConfig{
        PidGains{0.0, 1.0, 0.0},
        PidLimits{-10.0, 10.0}}};
    pid.update(1.0, 0.0, 1.0);
    assert(near(pid.integralOutput(), 1.0));

    assert(pid.configure(PidConfig{
        PidGains{1.0, 0.0, 0.0},
        PidLimits{-5.0, 5.0}}) == PidStatus::Ok);
    assert(pid.integralOutput() == 0.0);
    assert(pid.lastOutput() == 0.0);
}

}  // namespace

int main()
{
    testSafeDefaults();
    testProportionalAndOutputLimits();
    testConditionalIntegration();
    testIntegralUsesOutputLimits();
    testIntegralRespectsRemainingOutputRange();
    testDerivativeOnMeasurementAvoidsSetpointKick();
    testDerivativeOnErrorCanBeSelected();
    testInvalidValuesProduceSafeOutput();
    testRejectsInvalidConfiguration();
    testCalculationOverflowProducesSafeOutput();
    testReset();
    testReconfigureResetsState();
    return 0;
}
