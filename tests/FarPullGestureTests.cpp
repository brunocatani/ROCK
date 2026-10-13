#include "physics-interaction/input/FarPullGesturePolicy.h"
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
    const Point target{0, 100, 0};
    ok &= check(!confirmsPull(true, false, true, std::numeric_limits<float>::infinity()), "nonfinite speed fails closed");
    ok &= check(speedAwayFromTarget(Point{1, 2, 3}, target, target) == 0, "zero-length aim fails closed");

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
        velocity[0] == 0 && velocity[1] == 0 && velocity[2] == 0, "stationary controller remains idle regardless of game movement");
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
    return ok ? 0 : 1;
}
