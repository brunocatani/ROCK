#pragma once

#include "GrabFingerPoseMath.h"
#include "MeshGrab.h"
#include "PalmTransform.h"

#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace frik::rock::grab_finger_pose_runtime
{
    /*
     * HIGGS solves grasped fingers from the held object's triangle geometry and
     * lets its finger animator own the rendered skeleton. ROCK follows that
     * ownership boundary: this helper only converts the already selected mesh
     * contact into stable five-finger curl values, while FRIK remains the only
     * system that applies hand-pose overrides. The constants below are authored
     * in ROCK's active hand-space convention so grab, collider, and pose code use
     * the same right-hand basis with the configured left-hand mirroring rules.
     */
    struct SolvedGrabFingerPose
    {
        std::array<float, 5> values{ 0.2f, 0.2f, 0.2f, 0.2f, 0.2f };
        std::array<float, 15> jointValues{};
        std::array<RE::NiPoint3, 5> probeStart{};
        std::array<RE::NiPoint3, 5> probeEnd{};
        int hitCount = 0;
        int candidateTriangleCount = 0;
        bool solved = false;
        bool hasJointValues = false;
    };

    inline const std::array<RE::NiPoint3, 5>& normalFingerBasesAuthored()
    {
        static const std::array<RE::NiPoint3, 5> kBases{
            RE::NiPoint3(1.582972f, 1.853201f, -1.262648f),
            RE::NiPoint3(7.501364f, 2.277657f, 0.430291f),
            RE::NiPoint3(7.595781f, 0.457392f, 0.620980f),
            RE::NiPoint3(7.464033f, -1.438817f, 0.350152f),
            RE::NiPoint3(6.637259f, -3.018480f, -0.357420f),
        };
        return kBases;
    }

    inline RE::NiPoint3 alternateThumbBaseAuthored()
    {
        return RE::NiPoint3(2.9f, 0.55f, -0.45f);
    }

    inline float distanceSquared(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        const float dz = a.z - b.z;
        return dx * dx + dy * dy + dz * dz;
    }

    inline RE::NiPoint3 normalizedOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
        if (lengthSquared <= 0.000001f) {
            return normalizeDirection(fallback);
        }

        const float inv = 1.0f / std::sqrt(lengthSquared);
        return RE::NiPoint3(value.x * inv, value.y * inv, value.z * inv);
    }

    inline void appendCandidateTriangles(const std::vector<TriangleData>& triangles, const RE::NiPoint3& grabSurfacePoint,
        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>>& outTriangles)
    {
        constexpr float kCandidateRadius = 32.0f;
        constexpr float kCandidateRadiusSquared = kCandidateRadius * kCandidateRadius;

        outTriangles.clear();
        outTriangles.reserve((std::min)(triangles.size(), static_cast<std::size_t>(256)));
        for (const auto& triangle : triangles) {
            const RE::NiPoint3 centroid(
                (triangle.v0.x + triangle.v1.x + triangle.v2.x) / 3.0f,
                (triangle.v0.y + triangle.v1.y + triangle.v2.y) / 3.0f,
                (triangle.v0.z + triangle.v1.z + triangle.v2.z) / 3.0f);

            const float minDistanceSquared = (std::min)({
                distanceSquared(centroid, grabSurfacePoint),
                distanceSquared(triangle.v0, grabSurfacePoint),
                distanceSquared(triangle.v1, grabSurfacePoint),
                distanceSquared(triangle.v2, grabSurfacePoint),
            });
            if (minDistanceSquared <= kCandidateRadiusSquared) {
                outTriangles.push_back({ triangle.v0, triangle.v1, triangle.v2 });
            }
        }
    }

    inline SolvedGrabFingerPose solveGrabFingerPoseFromTriangles(const std::vector<TriangleData>& triangles, const RE::NiTransform& handTransform, bool isLeft,
        const RE::NiPoint3& grabAnchorWorld, const RE::NiPoint3& grabSurfacePoint, float minValue)
    {
        SolvedGrabFingerPose result{};
        const float clampedMin = std::clamp(minValue, 0.0f, 1.0f);
        result.values = { clampedMin, clampedMin, clampedMin, clampedMin, clampedMin };

        if (triangles.empty()) {
            return result;
        }

        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>> candidateTriangles;
        appendCandidateTriangles(triangles, grabSurfacePoint, candidateTriangles);
        result.candidateTriangleCount = static_cast<int>(candidateTriangles.size());
        if (candidateTriangles.empty()) {
            return result;
        }

        constexpr float kFingerReachPadding = 6.0f;
        constexpr float kMinFingerProbeDistance = 6.0f;
        constexpr float kMaxFingerProbeDistance = 26.0f;
        constexpr float kFingerProbeRadius = 1.25f;
        const RE::NiPoint3 palmToContact = grabSurfacePoint - grabAnchorWorld;
        const RE::NiPoint3 fallbackDirection = transformHandspaceDirection(handTransform, RE::NiPoint3(1.0f, 0.0f, 0.0f), isLeft);
        const auto& bases = normalFingerBasesAuthored();
        std::array<grab_finger_pose_math::FingerCurlValue, 5> solvedFingers{};

        for (std::size_t finger = 0; finger < bases.size(); ++finger) {
            const RE::NiPoint3 baseWorld = transformHandspacePosition(handTransform, bases[finger], isLeft) + palmToContact;
            const RE::NiPoint3 toContact = grabSurfacePoint - baseWorld;
            const float distanceToContact = std::sqrt(distanceSquared(grabSurfacePoint, baseWorld));
            const float probeDistance = std::clamp(distanceToContact + kFingerReachPadding, kMinFingerProbeDistance, kMaxFingerProbeDistance);
            const RE::NiPoint3 probeDirection = normalizedOrFallback(toContact, fallbackDirection);
            result.probeStart[finger] = baseWorld;
            result.probeEnd[finger] = baseWorld + probeDirection * probeDistance;

            const auto solved = grab_finger_pose_math::solveFingerCurlValue(candidateTriangles, baseWorld, probeDirection, probeDistance, clampedMin, kFingerProbeRadius);
            solvedFingers[finger] = solved;
            result.values[finger] = solved.value;
            if (solved.hit) {
                result.hitCount++;
            }
        }

        {
            const RE::NiPoint3 baseWorld = transformHandspacePosition(handTransform, alternateThumbBaseAuthored(), isLeft) + palmToContact;
            const RE::NiPoint3 toContact = grabSurfacePoint - baseWorld;
            const float distanceToContact = std::sqrt(distanceSquared(grabSurfacePoint, baseWorld));
            const float probeDistance = std::clamp(distanceToContact + kFingerReachPadding, kMinFingerProbeDistance, kMaxFingerProbeDistance);
            const RE::NiPoint3 probeDirection = normalizedOrFallback(toContact, fallbackDirection);
            const auto alternateThumb =
                grab_finger_pose_math::solveFingerCurlValue(candidateTriangles, baseWorld, probeDirection, probeDistance, clampedMin, kFingerProbeRadius);
            if (alternateThumb.hit && (!solvedFingers[0].hit || alternateThumb.value > result.values[0])) {
                if (!solvedFingers[0].hit) {
                    result.hitCount++;
                }
                result.values[0] = alternateThumb.value;
                result.probeStart[0] = baseWorld;
                result.probeEnd[0] = baseWorld + probeDirection * probeDistance;
            }
        }

        result.solved = result.hitCount > 0;
        result.jointValues = grab_finger_pose_math::expandFingerCurlsToJointValues(result.values);
        result.hasJointValues = result.solved;
        return result;
    }
}
