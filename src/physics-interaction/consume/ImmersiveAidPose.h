#pragma once

#include "physics-interaction/grab/SavedGrabOffsetFormat.h"

namespace rock::immersive_aid
{
    // Paired Stimpak capture from 2026-09-09: generated grab-proxy local
    // object transform and authored fingers. Med-X uses this same pair.
    // The capture's historical file key is not the runtime ALCH identity.
    inline constexpr saved_grab_offset::HandOffset kStimpakLeft{
        .present = true,
        .translateGame = { 0.896606326f, -2.84351349f, 2.60140061f },
        .rotate = { -0.957532287f, -0.165003806f, 0.236449167f, 0.231522828f, 0.0487563014f, 0.971606731f, -0.171846882f, 0.985087574f, -0.00848293304f },
        .hasFingerPose = true,
        .fingerValues = { 1.11802304f, 0.531901002f, 0.537351012f, 0.491921008f, 0.570991993f },
        .hasFingerJointValues = true,
        .fingerJointValues = { 1.11802304f, 1.11802304f, 1.11802304f, 0.648925781f, 0.531901002f, 0.461686134f, 0.653013229f, 0.537351012f, 0.467953652f, 0.618940771f, 0.491921008f, 0.415709138f, 0.678243995f, 0.570991993f, 0.506640792f },
    };

    inline constexpr saved_grab_offset::HandOffset kStimpakRight{
        .present = true,
        .translateGame = { -0.466310024f, -0.671931148f, -2.44398689f },
        .rotate = { -0.843581557f, -0.525941908f, -0.108419999f, 0.110439926f, 0.0276644230f, -0.993497431f, 0.525521576f, -0.850070298f, 0.0347478986f },
        .hasFingerPose = true,
        .fingerValues = { 1.02808297f, 0.493472010f, 0.537351012f, 0.502476990f, 0.602303028f },
        .hasFingerJointValues = true,
        .fingerJointValues = { 1.02808297f, 1.02808297f, 1.02808297f, 0.620104015f, 0.493472010f, 0.417492807f, 0.653013229f, 0.537351012f, 0.467953652f, 0.626857758f, 0.502476990f, 0.427848518f, 0.701727271f, 0.602303028f, 0.542648494f },
    };

    [[nodiscard]] constexpr const saved_grab_offset::HandOffset& stimpakPose(bool left)
    {
        return left ? kStimpakLeft : kStimpakRight;
    }
}
