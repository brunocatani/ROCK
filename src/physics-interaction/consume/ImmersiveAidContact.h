#pragma once

#include "physics-interaction/grab/GrabPoseCandidateSelector.h"

namespace rock::immersive_aid
{
    // All inputs share mesh-local space. Bounds only reject distant zones;
    // actual mesh triangles, including small needle tips, decide contact.
    [[nodiscard]] inline bool capsuleTouchesMesh(std::span<const GrabLocalTriangle> triangles,
        const RE::NiPoint3& minimum, const RE::NiPoint3& maximum,
        const RE::NiPoint3& a, const RE::NiPoint3& b, float radius)
    {
        if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
            !std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(b.z) ||
            !std::isfinite(radius) || radius <= 0.0f ||
            (std::max)(a.x, b.x) + radius < minimum.x || (std::min)(a.x, b.x) - radius > maximum.x ||
            (std::max)(a.y, b.y) + radius < minimum.y || (std::min)(a.y, b.y) - radius > maximum.y ||
            (std::max)(a.z, b.z) + radius < minimum.z || (std::min)(a.z, b.z) - radius > maximum.z) {
            return false;
        }
        for (const auto& triangle : triangles) {
            const float distanceSquared = grab_pose_candidate_selector::detail::segmentTriangleDistanceSquared(a, b, triangle);
            if (std::isfinite(distanceSquared) && distanceSquared <= radius * radius) {
                return true;
            }
        }
        return false;
    }
}
