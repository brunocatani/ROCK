#pragma once

#include <array>
#include <cmath>
#include <openvr.h>

namespace rock::vr_input
{
    [[nodiscard]] inline bool usableMotionPose(const vr::TrackedDevicePose_t& pose) noexcept
    {
        if (!pose.bDeviceIsConnected || !pose.bPoseIsValid ||
            pose.eTrackingResult != vr::TrackingResult_Running_OK) return false;
        for (const auto& row : pose.mDeviceToAbsoluteTracking.m)
            for (float value : row)
                if (!std::isfinite(value)) return false;
        for (float value : pose.vVelocity.v)
            if (!std::isfinite(value)) return false;
        return true;
    }

    // OpenVR velocity is physical tracking-space m/s: it contains no joystick
    // translation or game snap turn. Express it in headset-local right/forward/up
    // axes so the current FO4VR HmdNode basis can carry it into game world space.
    [[nodiscard]] inline bool velocityInHmdAxes(const vr::TrackedDevicePose_t& controller,
        const vr::TrackedDevicePose_t& hmd, std::array<float, 3>& result) noexcept
    {
        result = {};
        if (!usableMotionPose(controller) || !usableMotionPose(hmd)) return false;
        std::array<float, 3> local{};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            float lengthSquared = 0.0f;
            for (std::size_t row = 0; row < 3; ++row) {
                const float basis = hmd.mDeviceToAbsoluteTracking.m[row][axis];
                lengthSquared += basis * basis;
                local[axis] += basis * controller.vVelocity.v[row];
            }
            if (std::abs(lengthSquared - 1.0f) > 0.01f || !std::isfinite(local[axis])) return false;
        }
        result = { local[0], -local[2], local[1] };
        return true;
    }
}
