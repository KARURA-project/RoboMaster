#include <cassert>
#include <type_traits>

#include "robomaster/Control.hpp"

using namespace robomaster;

namespace {

void testTargetKeepsTypeAndValueTogether()
{
    const ControlTarget speedTarget{ControlType::Speed, 1000.0};
    assert(speedTarget.type == ControlType::Speed);
    assert(speedTarget.value == 1000.0);

    const ControlTarget stopTarget{ControlType::Current, 0.0};
    assert(stopTarget.type == ControlType::Current);
    assert(stopTarget.value == 0.0);
}

}  // namespace

int main()
{
    static_assert(
        !std::is_default_constructible<ControlTarget>::value,
        "ControlTarget must require an explicit type and value");
    testTargetKeepsTypeAndValueTogether();
    return 0;
}
