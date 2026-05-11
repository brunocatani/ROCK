#include "physics-interaction/input/DebugControllerPolicy.h"

#include <cmath>
#include <cstdio>

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }

        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }

        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectFloat(const char* label, float actual, float expected, float epsilon = 0.0001f)
    {
        if (std::fabs(actual - expected) <= epsilon) {
            return true;
        }

        std::printf("%s expected %.5f got %.5f\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace rock::debug_controller_policy;

    bool ok = true;

    const auto first = evaluateButtonEdges(false, 0, true, xinput_button::A);
    ok &= expectTrue("first sample is connected", first.connected);
    ok &= expectFalse("first sample does not press A", commandPressed(first, Command::ToggleHandColliders));

    const auto second = evaluateButtonEdges(true, 0, true, xinput_button::A | xinput_button::B | xinput_button::X | xinput_button::Y);
    ok &= expectTrue("A maps to hand collider command", commandPressed(second, Command::ToggleHandColliders));
    ok &= expectTrue("B maps to weapon collider command", commandPressed(second, Command::ToggleWeaponColliders));
    ok &= expectTrue("X maps to pivot tuning command", commandPressed(second, Command::TogglePivotTuning));
    ok &= expectTrue("Y maps to hand selection command", commandPressed(second, Command::SelectPivotHand));

    const auto heldA = evaluateButtonEdges(true, xinput_button::A, true, xinput_button::A | xinput_button::B);
    ok &= expectFalse("held A does not retrigger hand collider command", commandPressed(heldA, Command::ToggleHandColliders));
    ok &= expectTrue("new B still triggers while A is held", commandPressed(heldA, Command::ToggleWeaponColliders));

    const auto disconnected = evaluateButtonEdges(true, xinput_button::A, false, 0);
    ok &= expectFalse("disconnected sample is unavailable", disconnected.connected);
    ok &= expectFalse("disconnected sample emits no command", commandPressed(disconnected, Command::ToggleHandColliders));

    ok &= expectFloat("axis below deadzone is zero", applyAxisDeadzone(0.10f, 0.22f), 0.0f);
    ok &= expectFloat("full positive axis survives deadzone", applyAxisDeadzone(1.0f, 0.22f), 1.0f);
    ok &= expectFloat("full negative axis survives deadzone", applyAxisDeadzone(-1.0f, 0.22f), -1.0f);
    ok &= expectFloat("half post-deadzone axis normalizes", applyAxisDeadzone(0.61f, 0.22f), 0.5f, 0.0002f);

    const PivotVector base{ .x = 6.0f, .y = 0.2f, .z = -2.0f };
    const auto moved = applyPivotTuning(base, AnalogSample{ .leftX = -1.0f, .leftY = 1.0f, .rightY = 0.61f }, 0.05f);
    ok &= expectFloat("left stick vertical adjusts pivot X", moved.x, 6.4f);
    ok &= expectFloat("left stick horizontal adjusts pivot Y", moved.y, -0.2f);
    ok &= expectFloat("right stick vertical adjusts pivot Z", moved.z, -1.8f, 0.0002f);

    const auto clamped = applyPivotTuning(PivotVector{ .x = 49.9f, .y = -49.9f, .z = 49.9f },
        AnalogSample{ .leftX = -1.0f, .leftY = 1.0f, .rightY = 1.0f },
        0.1f);
    ok &= expectFloat("pivot X clamps high", clamped.x, 50.0f);
    ok &= expectFloat("pivot Y clamps low", clamped.y, -50.0f);
    ok &= expectFloat("pivot Z clamps high", clamped.z, 50.0f);

    ok &= expectFalse("unchanged pivot is not dirty", pivotChanged(base, base));
    ok &= expectTrue("changed pivot is dirty", pivotChanged(base, moved));

    return ok ? 0 : 1;
}
