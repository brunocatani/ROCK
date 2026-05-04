#pragma once

#include "GrabFingerPoseMath.h"
#include "RootFlattenedFingerSkeletonRuntime.h"
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
     * lets its finger animator own the rendered skeleton. ROCK keeps that output
     * boundary through FRIK while sourcing probe bases, open vectors, and palm
     * curl normals directly from the live root flattened finger bones.
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
        bool usedAlternateThumbCurve = false;
        bool usedLiveRootFlattenedFingerBones = false;
        bool hasThumbAlternateCurveFrame = false;
        RE::NiPoint3 thumbAlternateCurveBaseWorld{};
        RE::NiPoint3 thumbAlternateCurveOpenDirectionWorld{};
        RE::NiPoint3 thumbAlternateCurveNormalWorld{};
        float thumbAlternateCurveMaxCurlAngleRadians = 0.0f;
        bool hasThumbCurveDiagnostics = false;
        grab_finger_pose_math::FingerCurlValue thumbPrimaryCurve{};
        grab_finger_pose_math::FingerCurlValue thumbAlternateCurve{};
    };

    inline const std::array<float, 5>& fingerMaxAnglesRadians()
    {
        static const std::array<float, 5> kAngles{ 1.225f, 1.45f, 1.50f, 1.48f, 1.42f };
        return kAngles;
    }

    inline RE::NiPoint3 crossPoint(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        return RE::NiPoint3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    }

    inline float dotPoint(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
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

    inline RE::NiPoint3 liveThumbAlternateCurlNormalWorld(const RE::NiPoint3& openVectorWorld, const RE::NiPoint3& primaryNormalWorld,
        const RE::NiPoint3& baseWorld, const RE::NiPoint3& grabAnchorWorld, bool isLeft)
    {
        RE::NiPoint3 alternateNormal = normalizedOrFallback(crossPoint(openVectorWorld, primaryNormalWorld), RE::NiPoint3(0.0f, isLeft ? -1.0f : 1.0f, 0.0f));
        const RE::NiPoint3 towardPalm = normalizedOrFallback(grabAnchorWorld - baseWorld, RE::NiPoint3(0.0f, isLeft ? 1.0f : -1.0f, 0.0f));
        if (dotPoint(alternateNormal, crossPoint(openVectorWorld, towardPalm)) < 0.0f) {
            alternateNormal.x *= -1.0f;
            alternateNormal.y *= -1.0f;
            alternateNormal.z *= -1.0f;
        }
        return alternateNormal;
    }

    inline float distanceSquared(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        const float dz = a.z - b.z;
        return dx * dx + dy * dy + dz * dz;
    }

    inline void appendCandidateTriangles(const std::vector<TriangleData>& triangles, const RE::NiPoint3& grabSurfacePoint, float maxTriangleDistanceSquared,
        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>>& outTriangles)
    {
        outTriangles.clear();
        if (!std::isfinite(maxTriangleDistanceSquared) || maxTriangleDistanceSquared <= 0.0f) {
            return;
        }

        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>> allTriangles;
        allTriangles.reserve((std::min)(triangles.size(), static_cast<std::size_t>(512)));
        for (const auto& triangle : triangles) {
            allTriangles.push_back({ triangle.v0, triangle.v1, triangle.v2 });
        }
        outTriangles = grab_finger_pose_math::filterTrianglesNearPoint(allTriangles, grabSurfacePoint, maxTriangleDistanceSquared);
    }

    inline SolvedGrabFingerPose solveGrabFingerPoseFromTriangles(const std::vector<TriangleData>& triangles, const RE::NiTransform& handTransform, bool isLeft,
        const RE::NiPoint3& grabAnchorWorld, const RE::NiPoint3& grabSurfacePoint, float minValue, float maxTriangleDistanceSquared = 100.0f, bool useCurveSolver = true,
        const root_flattened_finger_skeleton_runtime::Snapshot* liveFingerSnapshot = nullptr)
    {
        SolvedGrabFingerPose result{};
        const float clampedMin = std::clamp(minValue, 0.0f, 1.0f);
        result.values = { clampedMin, clampedMin, clampedMin, clampedMin, clampedMin };

        if (triangles.empty()) {
            return result;
        }

        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>> candidateTriangles;
        appendCandidateTriangles(triangles, grabSurfacePoint, maxTriangleDistanceSquared, candidateTriangles);
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
        const auto& maxAngles = fingerMaxAnglesRadians();
        const auto liveLandmarks = liveFingerSnapshot ? root_flattened_finger_skeleton_runtime::buildLandmarkSet(*liveFingerSnapshot) : root_flattened_finger_skeleton_runtime::LandmarkSet{};
        if (!liveLandmarks.valid) {
            return result;
        }
        const RE::NiPoint3 curlNormalWorld = liveLandmarks.palmNormalWorld;
        result.usedLiveRootFlattenedFingerBones = true;

        for (std::size_t finger = 0; finger < result.values.size(); ++finger) {
            const auto& live = liveLandmarks.fingers[finger];
            const RE::NiPoint3 baseWorld = live.base + palmToContact;
            const RE::NiPoint3 openDirectionWorld = live.openDirection;
            const float fingerOpenLengthWorld = live.length;
            const RE::NiPoint3 thumbAlternateCurlNormalWorld = liveThumbAlternateCurlNormalWorld(openDirectionWorld, curlNormalWorld, baseWorld, grabAnchorWorld, isLeft);
            const RE::NiPoint3 toContact = grabSurfacePoint - baseWorld;
            const float distanceToContact = std::sqrt(distanceSquared(grabSurfacePoint, baseWorld));
            const float probeDistance = std::clamp(distanceToContact + kFingerReachPadding, kMinFingerProbeDistance, kMaxFingerProbeDistance);
            const RE::NiPoint3 probeDirection = normalizedOrFallback(toContact, fallbackDirection);
            result.probeStart[finger] = baseWorld;
            result.probeEnd[finger] = baseWorld + probeDirection * probeDistance;
            if (finger == 0) {
                result.hasThumbAlternateCurveFrame = true;
                result.thumbAlternateCurveBaseWorld = baseWorld;
                result.thumbAlternateCurveOpenDirectionWorld = normalizedOrFallback(openDirectionWorld, fallbackDirection);
                result.thumbAlternateCurveNormalWorld = normalizedOrFallback(thumbAlternateCurlNormalWorld, curlNormalWorld);
                result.thumbAlternateCurveMaxCurlAngleRadians = maxAngles[finger];
            }

            auto solved = grab_finger_pose_math::FingerCurlValue{};
            bool curveSolverRan = false;
            bool curveSolverSelectedAlternateThumb = false;
            if (useCurveSolver) {
                const bool isThumb = finger == 0;
                const auto curveSolved = grab_finger_pose_math::solveThumbAwareFingerCurveCurlValue(candidateTriangles,
                    baseWorld,
                    curlNormalWorld,
                    thumbAlternateCurlNormalWorld,
                    openDirectionWorld,
                    maxAngles[finger],
                    fingerOpenLengthWorld,
                    clampedMin,
                    isThumb);
                curveSolverRan = true;
                curveSolverSelectedAlternateThumb = isThumb && curveSolved.usedAlternateThumbCurve;
                solved = curveSolved.value;
                result.usedAlternateThumbCurve = result.usedAlternateThumbCurve || curveSolved.usedAlternateThumbCurve;
                if (isThumb) {
                    result.hasThumbCurveDiagnostics = true;
                    result.thumbPrimaryCurve = curveSolved.primary;
                    result.thumbAlternateCurve = curveSolved.alternateThumb;
                }
            }
            if (!solved.hit &&
                (!curveSolverRan || grab_finger_pose_math::shouldRunFallbackRayAfterCurveSolve(finger, solved.hit, curveSolverSelectedAlternateThumb))) {
                solved = grab_finger_pose_math::solveFingerCurlValue(candidateTriangles, baseWorld, probeDirection, probeDistance, clampedMin, kFingerProbeRadius);
            }
            result.values[finger] = solved.value;
            if (solved.hit) {
                result.hitCount++;
            }
        }

        result.solved = result.hitCount > 0;
        result.jointValues = grab_finger_pose_math::expandFingerCurlsToJointValues(result.values);
        result.hasJointValues = result.solved;
        return result;
    }
}
