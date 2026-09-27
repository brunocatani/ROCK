#include "physics-interaction/input/FarPullGesturePolicy.h"
#include "physics-interaction/input/GrabInputIntentPolicy.h"
#include "physics-interaction/hand/HandInteractionStateMachine.h"
#include "rock_support/VRMotion.h"

#include <cstdio>
#include <limits>

namespace
{
    struct Point { float x, y, z; };
    bool check(bool value, const char* label)
    {
        if (!value) std::printf("FAILED: %s\n", label);
        return value;
    }
    vr::TrackedDevicePose_t pose()
    {
        vr::TrackedDevicePose_t result{};
        result.bDeviceIsConnected = result.bPoseIsValid = true;
        result.eTrackingResult = vr::TrackingResult_Running_OK;
        for (int i = 0; i < 3; ++i) result.mDeviceToAbsoluteTracking.m[i][i] = 1.0f;
        return result;
    }
}

int main()
{
    using namespace rock::far_pull_gesture;
    bool ok = true;
    const Point hand{0, 0, 0}, target{0, 100, 0};
    auto pulls = [&](Point velocity, Point origin = Point{}) {
        return confirmsPull(true, false, true, speedAwayFromTarget(velocity, origin, target));
    };
    ok &= check(!pulls({0, 0, 0}), "holding grab alone cannot pull");
    ok &= check(!pulls({0, -0.15f, 0}), "ordinary hand jitter does not pull");
    ok &= check(!pulls({0, -1.2f, 0}), "threshold is strict");
    ok &= check(pulls({0, -1.21f, 0}), "inward flick confirms");
    ok &= check(!pulls({0, 3, 0}), "outward flick does not confirm");
    ok &= check(!pulls({3, 0, 0}) && !pulls({0, 0, 3}), "sideways and vertical movement do not confirm forward target");
    ok &= check(!pulls({4, -0.5f, 0}), "total hand speed is not the inward component");
    ok &= check(!confirmsPull(false, false, true, 3) && !confirmsPull(true, true, true, 3), "release wins over flick");
    ok &= check(!confirmsPull(true, false, false, 3), "invalid tracking cannot confirm");
    ok &= check(!confirmsPull(true, false, true, std::numeric_limits<float>::infinity()), "nonfinite speed fails closed");
    ok &= check(speedAwayFromTarget(Point{1, 2, 3}, target, target) == 0, "zero-length aim fails closed");
    ok &= check(sanitizeMode(1) == 1 && sanitizeMode(2) == 2 && sanitizeMode(-1) == 1 && sanitizeMode(3) == 1,
        "mode defaults and invalid fallback preserve immediate pull");

    auto hmd = pose(), controller = pose();
    controller.vVelocity.v[2] = 2.0f; // Physical movement backward in OpenVR.
    std::array<float, 3> velocity{};
    ok &= check(rock::vr_input::velocityInHmdAxes(controller, hmd, velocity) &&
        velocity[0] == 0 && velocity[1] == -2 && velocity[2] == 0, "OpenVR backward maps to headset backward");
    // A game-world teleport/joystick offset never enters this physical sample.
    controller.mDeviceToAbsoluteTracking.m[0][3] = 400.0f;
    ok &= check(rock::vr_input::velocityInHmdAxes(controller, hmd, velocity) && velocity[1] == -2,
        "tracking translation does not contaminate measured velocity");
    controller.vVelocity = {};
    ok &= check(rock::vr_input::velocityInHmdAxes(controller, hmd, velocity) &&
        !pulls({velocity[0], velocity[1], velocity[2]}), "stationary controller remains idle regardless of game movement");
    hmd.mDeviceToAbsoluteTracking.m[0][0] = 0;
    hmd.mDeviceToAbsoluteTracking.m[0][2] = 1;
    hmd.mDeviceToAbsoluteTracking.m[2][0] = -1;
    hmd.mDeviceToAbsoluteTracking.m[2][2] = 0;
    controller.vVelocity.v[0] = 2;
    ok &= check(rock::vr_input::velocityInHmdAxes(controller, hmd, velocity) && velocity[1] == -2,
        "headset yaw preserves backward gesture direction");
    for (int failure = 0; failure < 5; ++failure) {
        auto bad = pose();
        if (failure == 0) bad.bPoseIsValid = false;
        if (failure == 1) bad.bDeviceIsConnected = false;
        if (failure == 2) bad.eTrackingResult = vr::TrackingResult_Running_OutOfRange;
        if (failure == 3) bad.vVelocity.v[0] = std::numeric_limits<float>::quiet_NaN();
        if (failure == 4) bad.mDeviceToAbsoluteTracking.m[0][0] = std::numeric_limits<float>::infinity();
        ok &= check(!rock::vr_input::velocityInHmdAxes(bad, hmd, velocity), "bad controller sample rejected");
        ok &= check(!rock::vr_input::velocityInHmdAxes(controller, bad, velocity), "bad headset sample rejected");
    }
    hmd = pose();
    hmd.mDeviceToAbsoluteTracking.m[0][0] = 0;
    ok &= check(!rock::vr_input::velocityInHmdAxes(controller, hmd, velocity), "degenerate headset basis rejected");

    using rock::HandState;
    using rock::HandInteractionEvent;
    auto transition = rock::evaluateHandTransition({ .current = HandState::SelectedFar, .event = HandInteractionEvent::LockFarSelection });
    ok &= check(transition.accepted && transition.next == HandState::SelectionLocked, "far press locks without pulling");
    ok &= check(!rock::canUpdateSelectionFromState(transition.next) && rock::hasExclusiveObjectSelection(transition.next),
        "locked selection cannot drift to a new target or be shared");
    auto pull = rock::evaluateHandTransition({ .current = transition.next, .event = HandInteractionEvent::BeginPull });
    ok &= check(pull.accepted && pull.next == HandState::Pulled, "confirmed gesture enters existing pull path");
    auto release = rock::evaluateHandTransition({ .current = transition.next, .event = HandInteractionEvent::ClearSelection });
    ok &= check(release.accepted && release.next == HandState::Idle, "cancel releases the selection");
    auto equipment = rock::evaluateHandTransition({ .current = transition.next, .event = HandInteractionEvent::UnlockFarSelection });
    ok &= check(equipment.accepted && equipment.next == HandState::SelectedFar, "equipment confirmation hands off to existing drop path");
    ok &= check(!rock::evaluateHandTransition({ .current = HandState::HeldBody, .event = HandInteractionEvent::UnlockFarSelection }).accepted,
        "equipment handoff cannot unlock a physical hold");

    // A launched gesture returns to ordinary selection without preserving its
    // press. Exercise the real input policy through launch, arrival and catch,
    // including disabled leeway and a press buffered before the target was ready.
    namespace intent = rock::grab_input_intent_policy;
    for (const bool leewayEnabled : {false, true}) {
        for (const bool targetInitiallyReady : {false, true}) {
            intent::RuntimeState input{};
            const intent::Config config{ .enabled = leewayEnabled };
            (void)intent::update(input, { .held = true, .pressed = true },
                targetInitiallyReady, false, 0.01f, config);
            intent::reset(input); // Launch consumes any original/buffered press.
            for (int frame = 0; frame < 90; ++frame) {
                const auto held = intent::update(input, { .held = true },
                    frame >= 20, false, 1.0f / 90.0f, config);
                ok &= check(!held.pressed && !held.pendingPress,
                    "holding launch press cannot catch on arrival or later");
            }
            const auto released = intent::update(input, { .released = true }, true, false, 0.01f, config);
            ok &= check(released.released && !released.pressed, "release alone does not catch");
            const auto catchPress = intent::update(input, { .held = true, .pressed = true }, true, false, 0.01f, config);
            ok &= check(catchPress.pressed && !catchPress.syntheticPressed, "fresh second press can catch immediately");
        }
    }
    const auto launchFinished = rock::evaluateHandTransition({ .current = HandState::Pulled, .event = HandInteractionEvent::ClearSelection });
    ok &= check(launchFinished.accepted && rock::canUpdateSelectionFromState(launchFinished.next) &&
        !rock::hasExclusiveObjectSelection(launchFinished.next), "finished launch frees selection and other-hand ownership");
    return ok ? 0 : 1;
}
