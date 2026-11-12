#pragma once

#include <algorithm>
#include <array>
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>
#include <vector>
#include "RE/NetImmerse/NiTransform.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/VectorMath.h"
#include "physics-interaction/grab/GeneratedGrabFingerCalibration.h"
#include "physics-interaction/grab/GrabFingerPoseMath.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

namespace rock::grab_finger_pose_runtime
{
    /*
     * ROCK solves grasped fingers from the held object's triangle geometry and
     * lets FRIK own the rendered skeleton. Probe bases, open vectors, and palm
     * curl normals come directly from the live root flattened finger bones.
     */
    constexpr std::size_t kMaxFingerPoseCandidateTriangles = 2048;

    inline constexpr std::size_t kFingerSweepDebugProbeCount = 3;
    inline constexpr std::size_t kFingerSweepDebugPointsPerProbe = 9;

    enum class FingerPoseMeshRelation : std::uint8_t
    {
        CurrentMeshRequiresVirtualSeat,
        AlreadyAtCommandedSeat
    };

    [[nodiscard]] inline RE::NiPoint3 resolveFingerSweepBaseWorld(const RE::NiPoint3& liveProximalBaseWorld,
        const RE::NiPoint3& seatPointWorld, const RE::NiPoint3& grabAnchorWorld, bool useWholeMeshForMissingTargets,
        std::size_t explicitTargetCount, FingerPoseMeshRelation meshRelation)
    {
        const bool needsVirtualSeatShift = meshRelation == FingerPoseMeshRelation::CurrentMeshRequiresVirtualSeat &&
            useWholeMeshForMissingTargets && explicitTargetCount == 0;
        return needsVirtualSeatShift ? liveProximalBaseWorld + (seatPointWorld - grabAnchorWorld)
                                     : liveProximalBaseWorld;
    }

    enum class FingerSweepDebugState : std::uint8_t
    {
        Hit,
        ClosedLimit,
        MissFallback,
        OutOfReach,
        OverOpen
    };

    [[nodiscard]] inline const char* fingerSweepDebugStateName(FingerSweepDebugState state)
    {
        switch (state) {
        case FingerSweepDebugState::Hit:
            return "HIT";
        case FingerSweepDebugState::ClosedLimit:
            return "HIT@CLOSE-LIMIT";
        case FingerSweepDebugState::OutOfReach:
            return "OUT-OF-REACH";
        case FingerSweepDebugState::OverOpen:
            return "OVER-OPEN";
        case FingerSweepDebugState::MissFallback:
        default:
            return "MISS->FALLBACK";
        }
    }

    [[nodiscard]] inline const char* fingerSweepDebugFingerName(std::size_t finger)
    {
        constexpr std::array<const char*, 5> kNames{ "THUMB", "INDEX", "MIDDLE", "RING", "PINKY" };
        return finger < kNames.size() ? kNames[finger] : "UNKNOWN";
    }

    [[nodiscard]] inline const char* fingerSweepDebugProbeName(grab_finger_pose_math::CalibratedFingerProbe probe)
    {
        switch (probe) {
        case grab_finger_pose_math::CalibratedFingerProbe::Outer:
            return "OUTER";
        case grab_finger_pose_math::CalibratedFingerProbe::Inner:
            return "INNER";
        case grab_finger_pose_math::CalibratedFingerProbe::Tip:
        default:
            return "TIP";
        }
    }

    struct FingerSweepDebugFingerCapture
    {
        std::array<std::array<RE::NiPoint3, kFingerSweepDebugPointsPerProbe>, kFingerSweepDebugProbeCount>
            probePointsObjectLocal{};
        std::array<std::uint8_t, kFingerSweepDebugProbeCount> probePointCount{};
        std::array<grab_finger_pose_math::CalibratedFingerProbe, kFingerSweepDebugProbeCount> probeKind{
            grab_finger_pose_math::CalibratedFingerProbe::Tip,
            grab_finger_pose_math::CalibratedFingerProbe::Outer,
            grab_finger_pose_math::CalibratedFingerProbe::Inner,
        };
        std::array<RE::NiPoint3, kFingerSweepDebugProbeCount> authoredOpenPointObjectLocal{};
        std::array<std::uint8_t, kFingerSweepDebugProbeCount> authoredOpenPointValid{};
        std::array<float, kFingerSweepDebugProbeCount> probeStartOpenValue{};
        std::array<float, kFingerSweepDebugProbeCount> probeEndOpenValue{};
        RE::NiPoint3 pivotObjectLocal{};
        RE::NiPoint3 contactCenterObjectLocal{};
        RE::NiPoint3 hitPointObjectLocal{};
        RE::NiPoint3 hitNormalObjectLocal{};
        float contactRadiusObjectLocal = 0.0f;
        float publishedValue = 0.0f;
        float rawCurveValue = 0.0f;
        FingerSweepDebugState state = FingerSweepDebugState::MissFallback;
        grab_finger_calibration_data::BakedGrabThumbLane thumbLane =
            grab_finger_calibration_data::BakedGrabThumbLane::Wrap;
        std::uint8_t selectedProbeIndex = 0xFF;
        bool hasPivot = false;
        bool hasContact = false;
        bool hasHitPoint = false;
        bool hasHitNormal = false;
        bool valid = false;
    };

    struct FingerSweepDebugCapture
    {
        std::array<FingerSweepDebugFingerCapture, 5> fingers{};
        std::uint32_t spatialNodeVisits = 0;
        std::uint32_t spatialTriangleTests = 0;
        std::uint32_t candidateTriangleCount = 0;
        bool isLeft = false;
        bool inPowerArmor = false;
        bool valid = false;
    };

    struct FingerSweepDebugSnapshot
    {
        FingerSweepDebugCapture capture{};
        RE::NiTransform objectWorld{};
        bool valid = false;
    };

    inline constexpr std::uint8_t kCompleteFingerContactMask = 0x1Fu;

    struct SolvedGrabFingerPose
    {
        std::array<float, 5> values{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
        std::array<float, 15> jointValues{};
        std::array<RE::NiPoint3, 5> probeStart{};
        std::array<RE::NiPoint3, 5> probeEnd{};
        std::array<RE::NiPoint3, 5> surfaceAimTarget{};
        std::array<RE::NiPoint3, 5> surfaceAimNormal{};
        std::array<std::uint8_t, 5> surfaceAimTargetValid{};
        std::array<std::uint8_t, 5> surfaceAimNormalValid{};
        std::array<RE::NiPoint3, 5> surfaceAimTargetObjectLocal{};
        std::array<RE::NiPoint3, 5> surfaceAimNormalObjectLocal{};
        std::array<std::uint8_t, 5> surfaceAimTargetObjectLocalValid{};
        std::array<std::uint8_t, 5> surfaceAimNormalObjectLocalValid{};
        std::array<grab_finger_pose_math::FingerCurlValue::HitKind, 5> hitKind{};
        /*
         * Signed in-plane rotation (about the unsigned palm curl normal, baked
         * normal-sign already applied) from the arc zero to the contact row
         * each finger selected. Valid only for swept front contacts on
         * the palm-plane arc (thumb: Wrap lane only - alternate lanes solve in
         * a different plane). Diagnostics-only: the target-space solve can
         * report the exact calibrated contact row without re-reading rendered
         * finger geometry.
         */
        std::array<float, 5> contactArcRotationRadians{};
        std::array<std::uint8_t, 5> contactArcRotationValid{};
        std::uint8_t contactValidMask = 0;
        int hitCount = 0;
        int candidateTriangleCount = 0;
        int poseTargetCount = 0;
        std::uint32_t spatialNodeVisitCount = 0;
        std::uint32_t spatialTriangleTestCount = 0;
        bool solved = false;
        bool hasJointValues = false;
        bool hasObjectLocalSurfaceAim = false;
        bool usedSpatialIndex = false;
        bool usedAlternateThumbCurve = false;
        bool usedAlternateThumbSurfaceHit = false;
        bool thumbSurfaceFollowAllowed = true;
        grab_finger_calibration_data::BakedGrabThumbLane selectedThumbLane =
            grab_finger_calibration_data::BakedGrabThumbLane::Wrap;
        float selectedThumbLaneNormalBlend = 0.0f;
        float selectedThumbLaneLocalCorrectionStrength = 0.0f;
        bool usedLiveRootFlattenedFingerBones = false;
        bool hasThumbAlternateCurveFrame = false;
        RE::NiPoint3 thumbAlternateCurveBaseWorld{};
        RE::NiPoint3 thumbAlternateCurveOpenDirectionWorld{};
        RE::NiPoint3 thumbAlternateCurveNormalWorld{};
        float thumbAlternateCurveMaxCurlAngleRadians = 0.0f;
        bool hasThumbCurveDiagnostics = false;
        grab_finger_pose_math::FingerCurlValue thumbPrimaryCurve{};
        grab_finger_pose_math::FingerCurlValue thumbAlternateCurve{};
        grab_finger_pose_math::FingerCurlValue thumbSidePadCurve{};
    };

    [[nodiscard]] inline bool hasCompleteFingerContactEvidence(const SolvedGrabFingerPose& pose)
    {
        return pose.solved && pose.contactValidMask == kCompleteFingerContactMask;
    }

    enum class OppositionPocketKind : std::uint8_t
    {
        None = 0,
        ThumbIndex = 1,
        ThumbPinky = 2,
    };

    struct OppositionPocketEvidence
    {
        RE::NiPoint3 surfacePointWorld{};
        float fingerGapGameUnits = 0.0f;
        float gripToSurfaceDistanceGameUnits = (std::numeric_limits<float>::infinity)();
        std::uint8_t endpointMask = 0;
        std::uint8_t directEndpointMask = 0;
        std::size_t opposedFingerIndex = 0;
        OppositionPocketKind kind = OppositionPocketKind::None;
        bool valid = false;
    };

    [[nodiscard]] inline const char* oppositionPocketKindName(OppositionPocketKind kind)
    {
        switch (kind) {
        case OppositionPocketKind::ThumbIndex:
            return "thumb-index";
        case OppositionPocketKind::ThumbPinky:
            return "thumb-pinky";
        case OppositionPocketKind::None:
        default:
            return "none";
        }
    }

    /*
     * Weapon parts are not eligible for the regular whole-object pinch-size
     * test: a small slide or control often belongs to a much larger weapon.
     * This local witness instead requires the selected weapon surface to cross
     * the live pad-to-pad segment near the captured grip point, plus at least
     * one direct endpoint contact from the calibrated sweep. It can therefore
     * authorize one coherent opposition pose without treating an arbitrary
     * four-of-five contact mask as complete evidence.
     */
    [[nodiscard]] inline OppositionPocketEvidence findLocalOppositionPocketEvidence(
        const std::vector<TriangleData>& worldTriangles,
        const root_flattened_finger_skeleton_runtime::Snapshot& snapshot, const RE::NiPoint3& gripPointWorld,
        std::uint8_t directContactMask, float minFingerGapGameUnits, float maxFingerGapGameUnits,
        float maxGripToSurfaceDistanceGameUnits, float segmentProbeRadiusGameUnits = 0.5f)
    {
        OppositionPocketEvidence best{};
        const auto finitePoint = [](const RE::NiPoint3& point) {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        };
        if (!snapshot.valid || worldTriangles.empty() || !finitePoint(gripPointWorld)) {
            return best;
        }

        const float minGap = (std::max)(0.0f, std::isfinite(minFingerGapGameUnits) ? minFingerGapGameUnits : 1.0f);
        const float maxGap = (std::max)(minGap, std::isfinite(maxFingerGapGameUnits) ? maxFingerGapGameUnits : 12.0f);
        const float maxGripDistance = (std::max)(0.0f,
            std::isfinite(maxGripToSurfaceDistanceGameUnits) ? maxGripToSurfaceDistanceGameUnits : 8.0f);
        const float probeRadius =
            std::clamp(std::isfinite(segmentProbeRadiusGameUnits) ? segmentProbeRadiusGameUnits : 0.5f, 0.0f, 2.0f);

        const auto evaluatePair = [&](const std::size_t opposedFinger, const OppositionPocketKind kind) {
            OppositionPocketEvidence candidate{};
            if (opposedFinger >= snapshot.fingers.size() || !snapshot.fingers[0].valid ||
                !snapshot.fingers[opposedFinger].valid) {
                return candidate;
            }
            const std::uint8_t endpointMask = static_cast<std::uint8_t>((1u << 0u) | (1u << opposedFinger));
            const std::uint8_t directEndpointMask = static_cast<std::uint8_t>(directContactMask & endpointMask);
            if (directEndpointMask == 0) {
                return candidate;
            }

            const RE::NiPoint3 thumbPad = snapshot.fingers[0].points[2];
            const RE::NiPoint3 opposedPad = snapshot.fingers[opposedFinger].points[2];
            if (!finitePoint(thumbPad) || !finitePoint(opposedPad)) {
                return candidate;
            }
            const RE::NiPoint3 segment = opposedPad - thumbPad;
            const float gapSquared = grab_finger_pose_math::lengthSquared(segment);
            if (!std::isfinite(gapSquared) || gapSquared <= 0.000001f) {
                return candidate;
            }
            const float gap = std::sqrt(gapSquared);
            if (gap < minGap || gap > maxGap) {
                return candidate;
            }
            const RE::NiPoint3 direction = segment * (1.0f / gap);

            float bestGripDistance = (std::numeric_limits<float>::infinity)();
            RE::NiPoint3 bestSurfacePoint{};
            for (const auto& triangle : worldTriangles) {
                const grab_finger_pose_math::Triangle<RE::NiPoint3> mathTriangle{
                    triangle.v0,
                    triangle.v1,
                    triangle.v2,
                };
                float travel = 0.0f;
                RE::NiPoint3 surfacePoint{};
                if (grab_finger_pose_math::rayTriangleIntersection(thumbPad, direction, mathTriangle, gap, travel)) {
                    surfacePoint = thumbPad + direction * travel;
                } else if (probeRadius > 0.0f &&
                    grab_finger_pose_math::probeCapsuleTriangleIntersection(thumbPad, direction, mathTriangle, gap,
                        probeRadius, travel, &surfacePoint)) {
                    // The bounded capsule admits near-tangent mesh contact,
                    // while the grip-distance gate below keeps it local.
                } else {
                    continue;
                }

                const float gripDistanceSquared =
                    grab_finger_pose_math::lengthSquared(grab_finger_pose_math::sub(gripPointWorld, surfacePoint));
                if (!std::isfinite(gripDistanceSquared)) {
                    continue;
                }
                const float gripDistance = std::sqrt(gripDistanceSquared);
                if (gripDistance <= maxGripDistance && gripDistance < bestGripDistance) {
                    bestGripDistance = gripDistance;
                    bestSurfacePoint = surfacePoint;
                }
            }

            if (!std::isfinite(bestGripDistance)) {
                return candidate;
            }
            candidate.surfacePointWorld = bestSurfacePoint;
            candidate.fingerGapGameUnits = gap;
            candidate.gripToSurfaceDistanceGameUnits = bestGripDistance;
            candidate.endpointMask = endpointMask;
            candidate.directEndpointMask = directEndpointMask;
            candidate.opposedFingerIndex = opposedFinger;
            candidate.kind = kind;
            candidate.valid = true;
            return candidate;
        };

        const auto thumbIndex = evaluatePair(1, OppositionPocketKind::ThumbIndex);
        const auto thumbPinky = evaluatePair(4, OppositionPocketKind::ThumbPinky);
        if (thumbIndex.valid) {
            best = thumbIndex;
        }
        if (thumbPinky.valid &&
            (!best.valid || thumbPinky.gripToSurfaceDistanceGameUnits < best.gripToSurfaceDistanceGameUnits)) {
            best = thumbPinky;
        }
        return best;
    }

    /*
     * The arc zero reference reconstructed from the COMMANDED hand model:
     * chain bone origins of hFRIK's authored fully-open pose in hand-bone
     * space - the identical zero definition the offline bake uses
     * (normalize(distal_open_origin - base_origin)). Rendered finger bones
     * anchor the regular target-space solve: ROCK's own surface-aim local
     * transforms bend the rendered chain, so measuring the arc zero from
     * rendered geometry makes the result depend on the pose currently being
     * replaced. The authored-open direction is immutable and exactly matches
     * the calibration bake's reference.
     */
    [[nodiscard]] inline std::array<RE::NiPoint3, 5> computeCommandedOpenDirectionsHandLocal(
        const RE::NiTransform* openLocalTransforms /* 15 finger bones, finger-major */)
    {
        std::array<RE::NiPoint3, 5> result{};
        if (!openLocalTransforms) {
            return result;
        }
        for (std::size_t finger = 0; finger < result.size(); ++finger) {
            const RE::NiTransform& bone1 = openLocalTransforms[finger * 3];
            const RE::NiTransform bone2 = transform_math::composeTransforms(bone1, openLocalTransforms[finger * 3 + 1]);
            const RE::NiTransform bone3 = transform_math::composeTransforms(bone2, openLocalTransforms[finger * 3 + 2]);
            const RE::NiPoint3 span = bone3.translate - bone1.translate;
            const float lengthSquared = span.x * span.x + span.y * span.y + span.z * span.z;
            if (!std::isfinite(lengthSquared) || lengthSquared <= 0.000001f) {
                // Zero vector = "invalid" downstream; callers gate on magnitude.
                continue;
            }
            const float inverseLength = 1.0f / std::sqrt(lengthSquared);
            result[finger] = RE::NiPoint3{ span.x * inverseLength, span.y * inverseLength, span.z * inverseLength };
        }
        return result;
    }

    struct GrabFingerPoseTargetSet
    {
        std::array<RE::NiPoint3, 5> targets{};
        std::array<RE::NiPoint3, 5> targetNormals{};
        std::array<std::uint8_t, 5> targetValid{};
        std::array<std::uint8_t, 5> targetNormalValid{};
        RE::NiPoint3 seatPointWorld{};
        RE::NiPoint3 seatNormalWorld{};
        bool seatPointValid = false;
        bool seatNormalValid = false;
        bool useSeatPointForMissingTargets = true;
        bool useWholeMeshForMissingTargets = false;
        std::uint32_t targetCount = 0;
    };

    struct FingerPadSurfaceEvidence
    {
        RE::NiPoint3 startWorld{};
        RE::NiPoint3 endWorld{};
        RE::NiPoint3 hitPointWorld{};
        RE::NiPoint3 hitNormalWorld{};
        float distanceGameUnits = 0.0f;
        float quality = 0.0f;
        bool hit = false;
        bool fromClosestSurface = false;
        bool padMayBeInsideSurface = false;
    };

    struct FingerPadProbeOptions
    {
        float probeRadiusGameUnits = 1.0f;
        float probeDistanceGameUnits = 6.0f;
        float closestSurfaceMaxDistanceGameUnits = 4.0f;
        float targetOverrideMinQuality = 0.5f;
    };

    inline FingerPadProbeOptions sanitizeFingerPadProbeOptions(FingerPadProbeOptions options)
    {
        if (!std::isfinite(options.probeRadiusGameUnits) || options.probeRadiusGameUnits <= 0.0f) {
            options.probeRadiusGameUnits = 1.0f;
        }
        if (!std::isfinite(options.probeDistanceGameUnits) || options.probeDistanceGameUnits <= 0.0f) {
            options.probeDistanceGameUnits = 6.0f;
        }
        if (!std::isfinite(options.closestSurfaceMaxDistanceGameUnits) ||
            options.closestSurfaceMaxDistanceGameUnits <= 0.0f) {
            options.closestSurfaceMaxDistanceGameUnits = 4.0f;
        }

        options.probeRadiusGameUnits = std::clamp(options.probeRadiusGameUnits, 0.05f, 8.0f);
        options.probeDistanceGameUnits = std::clamp(options.probeDistanceGameUnits, 0.5f, 32.0f);
        options.closestSurfaceMaxDistanceGameUnits =
            std::clamp(options.closestSurfaceMaxDistanceGameUnits, 0.5f, 24.0f);
        options.targetOverrideMinQuality = std::isfinite(options.targetOverrideMinQuality)
            ? std::clamp(options.targetOverrideMinQuality, 0.0f, 1.0f)
            : 0.5f;
        return options;
    }

    inline void captureSurfaceAimObjectLocal(SolvedGrabFingerPose& pose, const RE::NiTransform& objectWorldTransform)
    {
        pose.surfaceAimTargetObjectLocal = {};
        pose.surfaceAimNormalObjectLocal = {};
        pose.surfaceAimTargetObjectLocalValid = {};
        pose.surfaceAimNormalObjectLocalValid = {};
        pose.hasObjectLocalSurfaceAim = false;

        if (!std::isfinite(objectWorldTransform.scale) || std::abs(objectWorldTransform.scale) <= 0.000001f) {
            return;
        }

        for (std::size_t finger = 0; finger < pose.surfaceAimTarget.size(); ++finger) {
            if (pose.surfaceAimTargetValid[finger]) {
                pose.surfaceAimTargetObjectLocal[finger] =
                    transform_math::worldPointToLocal(objectWorldTransform, pose.surfaceAimTarget[finger]);
                pose.surfaceAimTargetObjectLocalValid[finger] = 1;
                pose.hasObjectLocalSurfaceAim = true;
            }
            if (pose.surfaceAimNormalValid[finger]) {
                pose.surfaceAimNormalObjectLocal[finger] =
                    transform_math::worldVectorToLocal(objectWorldTransform, pose.surfaceAimNormal[finger]);
                pose.surfaceAimNormalObjectLocalValid[finger] = 1;
                pose.hasObjectLocalSurfaceAim = true;
            }
        }
    }

    inline void useThumbIndexCurveOnlyPose(SolvedGrabFingerPose& pose)
    {
        pose.thumbSurfaceFollowAllowed = false;
        pose.usedAlternateThumbSurfaceHit = false;
        for (std::size_t finger = 0; finger < 2 && finger < pose.surfaceAimTargetValid.size(); ++finger) {
            pose.surfaceAimTargetValid[finger] = 0;
            pose.surfaceAimNormalValid[finger] = 0;
            pose.surfaceAimTargetObjectLocalValid[finger] = 0;
            pose.surfaceAimNormalObjectLocalValid[finger] = 0;
            pose.surfaceAimTarget[finger] = {};
            pose.surfaceAimNormal[finger] = {};
            pose.surfaceAimTargetObjectLocal[finger] = {};
            pose.surfaceAimNormalObjectLocal[finger] = {};
        }
        pose.hasObjectLocalSurfaceAim = false;
        for (std::size_t finger = 2; finger < pose.surfaceAimTargetObjectLocalValid.size(); ++finger) {
            if (pose.surfaceAimTargetObjectLocalValid[finger] || pose.surfaceAimNormalObjectLocalValid[finger]) {
                pose.hasObjectLocalSurfaceAim = true;
                break;
            }
        }
    }

    [[nodiscard]] inline SolvedGrabFingerPose resolveSurfaceAimObjectLocal(const SolvedGrabFingerPose& pose,
        const RE::NiTransform& objectWorldTransform)
    {
        if (!pose.hasObjectLocalSurfaceAim || !std::isfinite(objectWorldTransform.scale) ||
            std::abs(objectWorldTransform.scale) <= 0.000001f) {
            return pose;
        }

        auto resolved = pose;
        for (std::size_t finger = 0; finger < resolved.surfaceAimTarget.size(); ++finger) {
            if (pose.surfaceAimTargetObjectLocalValid[finger]) {
                resolved.surfaceAimTarget[finger] =
                    transform_math::localPointToWorld(objectWorldTransform, pose.surfaceAimTargetObjectLocal[finger]);
                resolved.surfaceAimTargetValid[finger] = 1;
            }
            if (pose.surfaceAimNormalObjectLocalValid[finger]) {
                const RE::NiPoint3 normalWorld =
                    transform_math::localVectorToWorld(objectWorldTransform, pose.surfaceAimNormalObjectLocal[finger]);
                if ((normalWorld.x * normalWorld.x + normalWorld.y * normalWorld.y + normalWorld.z * normalWorld.z) >
                    0.000001f) {
                    resolved.surfaceAimNormal[finger] = normalizeDirection(normalWorld);
                    resolved.surfaceAimNormalValid[finger] = 1;
                }
            }
        }
        return resolved;
    }

    [[nodiscard]] inline float missedFingerCurlFallbackValue(bool hasExplicitFingerTarget,
        grab_finger_pose_math::FingerCurlValue::HitKind hitKind, float minValue)
    {
        if (!hasExplicitFingerTarget) {
            return 0.3f;
        }
        if (hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::BackSurface ||
            hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::Rejected) {
            return 1.0f;
        }
        return std::clamp(minValue, 0.0f, 1.0f);
    }

    inline RE::NiPoint3 crossPoint(const RE::NiPoint3& a, const RE::NiPoint3& b) { return vector_math::cross(a, b); }

    inline float dotPoint(const RE::NiPoint3& a, const RE::NiPoint3& b) { return vector_math::dot(a, b); }

    inline RE::NiPoint3 normalizedOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float lengthSquared = vector_math::lengthSquared(value);
        if (lengthSquared <= 0.000001f) {
            return normalizeDirection(fallback);
        }

        const float inv = 1.0f / std::sqrt(lengthSquared);
        return RE::NiPoint3(value.x * inv, value.y * inv, value.z * inv);
    }

    inline RE::NiPoint3 liveThumbAlternateCurlNormalWorld(const RE::NiPoint3& openVectorWorld,
        const RE::NiPoint3& primaryNormalWorld, const RE::NiPoint3& baseWorld, const RE::NiPoint3& grabAnchorWorld,
        bool isLeft)
    {
        RE::NiPoint3 alternateNormal = normalizedOrFallback(crossPoint(openVectorWorld, primaryNormalWorld),
            RE::NiPoint3(0.0f, isLeft ? -1.0f : 1.0f, 0.0f));
        const RE::NiPoint3 towardPalm =
            normalizedOrFallback(grabAnchorWorld - baseWorld, RE::NiPoint3(0.0f, isLeft ? 1.0f : -1.0f, 0.0f));
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

    inline bool isFinitePoint(const RE::NiPoint3& point) { return vector_math::hasFiniteComponents(point); }

    [[nodiscard]] inline FingerSweepDebugState classifyFingerSweepDebugState(
        const grab_finger_pose_math::FingerCurlValue& solved, float minValue)
    {
        if (solved.hit) {
            if (solved.rawCurveValue > grab_finger_pose_math::kMaxFingerOpenValue + 0.0001f) {
                return FingerSweepDebugState::OverOpen;
            }
            const float clampedMin = std::clamp(std::isfinite(minValue) ? minValue : 0.0f, 0.0f, 1.0f);
            if (solved.value <= clampedMin + 0.0001f) {
                return FingerSweepDebugState::ClosedLimit;
            }
            return FingerSweepDebugState::Hit;
        }
        return solved.outOfReach ? FingerSweepDebugState::OutOfReach : FingerSweepDebugState::MissFallback;
    }

    [[nodiscard]] inline grab_finger_pose_math::CalibratedFingerCurve<RE::NiPoint3> makeSelectedFingerSweepDebugCurve(
        std::size_t finger, bool isLeft, bool inPowerArmor, const RE::NiPoint3& centerWorld,
        const RE::NiPoint3& selectedNormalWorld, const RE::NiPoint3& zeroAngleVectorWorld, float fingerLength,
        grab_finger_calibration_data::BakedGrabThumbLane selectedThumbLane)
    {
        const auto& handProfile = grab_finger_calibration_data::bakedGrabFingerHandProfile(isLeft, inPowerArmor);
        if (finger >= handProfile.fingers.size()) {
            return {};
        }

        const RE::NiPoint3 selectedNormal = normalizedOrFallback(selectedNormalWorld, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        if (finger != 0 || selectedThumbLane == grab_finger_calibration_data::BakedGrabThumbLane::Wrap) {
            return grab_finger_pose_math::makeBakedCalibratedFingerCurveFromBaked<RE::NiPoint3>(
                handProfile.fingers[finger], centerWorld, selectedNormal, zeroAngleVectorWorld, fingerLength,
                /*applyBakedNormalSign=*/false);
        }

        const auto& thumbProfile = grab_finger_calibration_data::bakedGrabThumbProfile(isLeft, inPowerArmor);
        for (const auto& lane : thumbProfile.lanes) {
            if (lane.lane != selectedThumbLane) {
                continue;
            }
            const auto& bakedCurve =
                lane.curveSource == grab_finger_calibration_data::BakedGrabThumbCurveSource::SidePad
                ? thumbProfile.sidePadCurve
                : handProfile.fingers[0];
            return grab_finger_pose_math::makeBakedCalibratedFingerCurveFromBaked<RE::NiPoint3>(bakedCurve, centerWorld,
                selectedNormal, zeroAngleVectorWorld, fingerLength,
                /*applyBakedNormalSign=*/false, lane.surfaceThicknessScale);
        }
        return {};
    }

    inline bool captureFingerSweepDebugCurve(FingerSweepDebugFingerCapture& out,
        const grab_finger_pose_math::CalibratedFingerCurve<RE::NiPoint3>& curve,
        const grab_finger_pose_math::FingerCurlValue& solved, float minValue,
        grab_finger_calibration_data::BakedGrabThumbLane selectedThumbLane, const RE::NiTransform& objectWorldTransform)
    {
        out = {};
        if (!std::isfinite(objectWorldTransform.scale) || std::abs(objectWorldTransform.scale) <= 0.000001f ||
            !isFinitePoint(curve.center) || !isFinitePoint(curve.normal) || !isFinitePoint(curve.zeroAngleVector)) {
            return false;
        }

        const RE::NiPoint3 planeNormal = normalizedOrFallback(curve.normal, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        const RE::NiPoint3 zeroAngle = normalizedOrFallback(curve.zeroAngleVector, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        out.pivotObjectLocal = transform_math::worldPointToLocal(objectWorldTransform, curve.center);
        out.hasPivot = isFinitePoint(out.pivotObjectLocal);
        auto reconstructPointWorld =
            [&](const grab_finger_pose_math::CalibratedFingerCurveSample<RE::NiPoint3>& sample) {
                const RE::NiPoint3 arm =
                    grab_finger_pose_math::rotateAroundUnitAxis(zeroAngle, planeNormal, sample.angleRadians);
                return grab_finger_pose_math::add(curve.center, grab_finger_pose_math::scale(arm, sample.reachLength));
            };
        bool anyPath = false;
        for (std::size_t probeIndex = 0; probeIndex < curve.probeCount && probeIndex < curve.probes.size() &&
            probeIndex < out.probePointsObjectLocal.size();
            ++probeIndex) {
            const auto& probe = curve.probes[probeIndex];
            const std::size_t sampleCount = (std::min)(probe.sampleCount, probe.samples.size());
            if (sampleCount == 0) {
                continue;
            }

            std::size_t startRow = 0;
            if (probeIndex < solved.sweptProbeStartRowValid.size() && solved.sweptProbeStartRowValid[probeIndex] != 0 &&
                solved.sweptProbeStartRow[probeIndex] < sampleCount) {
                startRow = solved.sweptProbeStartRow[probeIndex];
            }
            if (startRow >= sampleCount) {
                continue;
            }

            const std::size_t availableRows = sampleCount - startRow;
            const std::size_t pointCount = (std::min)(availableRows, kFingerSweepDebugPointsPerProbe);
            if (pointCount == 0) {
                continue;
            }
            out.probeKind[probeIndex] = probe.probe;
            out.probeStartOpenValue[probeIndex] = probe.samples[startRow].openValue;
            out.probeEndOpenValue[probeIndex] = probe.samples[sampleCount - 1].openValue;
            for (std::size_t row = startRow; row < sampleCount; ++row) {
                if (probe.samples[row].openValue > grab_finger_pose_math::kMaxFingerOpenValue + 0.0001f) {
                    continue;
                }
                const RE::NiPoint3 authoredOpenWorld = reconstructPointWorld(probe.samples[row]);
                if (isFinitePoint(authoredOpenWorld)) {
                    out.authoredOpenPointObjectLocal[probeIndex] =
                        transform_math::worldPointToLocal(objectWorldTransform, authoredOpenWorld);
                    out.authoredOpenPointValid[probeIndex] =
                        isFinitePoint(out.authoredOpenPointObjectLocal[probeIndex]) ? 1 : 0;
                }
                break;
            }
            for (std::size_t pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
                const std::size_t rowOffset =
                    pointCount > 1 ? ((availableRows - 1) * pointIndex) / (pointCount - 1) : 0;
                const auto& sample = probe.samples[startRow + rowOffset];
                const RE::NiPoint3 pointWorld = reconstructPointWorld(sample);
                if (!isFinitePoint(pointWorld)) {
                    break;
                }
                const RE::NiPoint3 pointObjectLocal =
                    transform_math::worldPointToLocal(objectWorldTransform, pointWorld);
                if (!isFinitePoint(pointObjectLocal)) {
                    break;
                }
                out.probePointsObjectLocal[probeIndex][out.probePointCount[probeIndex]++] = pointObjectLocal;
            }
            anyPath = anyPath || out.probePointCount[probeIndex] > 1;
        }

        out.publishedValue = solved.value;
        out.rawCurveValue = solved.rawCurveValue;
        out.state = classifyFingerSweepDebugState(solved, minValue);
        out.thumbLane = selectedThumbLane;
        out.selectedProbeIndex = solved.selectedProbeIndex;

        if (solved.hasContactCenter) {
            const RE::NiPoint3 centerWorld{ solved.contactCenterX, solved.contactCenterY, solved.contactCenterZ };
            if (isFinitePoint(centerWorld)) {
                out.contactCenterObjectLocal = transform_math::worldPointToLocal(objectWorldTransform, centerWorld);
                out.contactRadiusObjectLocal = solved.contactRadius / std::abs(objectWorldTransform.scale);
                out.hasContact = isFinitePoint(out.contactCenterObjectLocal) &&
                    std::isfinite(out.contactRadiusObjectLocal) && out.contactRadiusObjectLocal > 0.0f;
            }
        }
        if (solved.hasHitPoint) {
            const RE::NiPoint3 hitPointWorld{ solved.hitPointX, solved.hitPointY, solved.hitPointZ };
            if (isFinitePoint(hitPointWorld)) {
                out.hitPointObjectLocal = transform_math::worldPointToLocal(objectWorldTransform, hitPointWorld);
                out.hasHitPoint = isFinitePoint(out.hitPointObjectLocal);
            }
        }
        if (solved.hasHitNormal) {
            const RE::NiPoint3 hitNormalWorld{ solved.hitNormalX, solved.hitNormalY, solved.hitNormalZ };
            if (isFinitePoint(hitNormalWorld)) {
                const RE::NiPoint3 hitNormalObjectLocal =
                    transform_math::worldVectorToLocal(objectWorldTransform, hitNormalWorld);
                if (isFinitePoint(hitNormalObjectLocal) &&
                    distanceSquared(hitNormalObjectLocal, RE::NiPoint3{}) > 0.000001f) {
                    out.hitNormalObjectLocal =
                        normalizedOrFallback(hitNormalObjectLocal, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
                    out.hasHitNormal = true;
                }
            }
        }

        out.valid = anyPath;
        return out.valid;
    }

    struct FingerPoseSpatialQueryStats
    {
        std::uint32_t nodeVisits = 0;
        std::uint32_t triangleTests = 0;
    };

    /*
     * Immutable, object-local BVH for the bounded finger-pose triangle set.
     * A regular grab builds it once at commit, then every swept-arc sphere
     * probe queries the exact same triangles without linearly revisiting all
     * 2048 candidates. Nodes and triangles own no engine pointers, and query
     * uses a fixed stack so the runtime sweep performs no heap allocation.
     */
    class FingerPoseTriangleSpatialIndex
    {
    public:
        void clear()
        {
            _triangles.clear();
            _triangleIndices.clear();
            _nodes.clear();
        }

        template <class TriangleContainer>
        bool buildFromLocalTriangles(const TriangleContainer& localTriangles)
        {
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::GrabFingerIndexBuild);
            clear();
            const std::size_t triangleLimit = (std::min)(localTriangles.size(), kMaxFingerPoseCandidateTriangles);
            _triangles.reserve(triangleLimit);
            for (const auto& source : localTriangles) {
                if (!isFinitePoint(source.v0) || !isFinitePoint(source.v1) || !isFinitePoint(source.v2)) {
                    continue;
                }
                _triangles.push_back(grab_finger_pose_math::Triangle<RE::NiPoint3>{ source.v0, source.v1, source.v2 });
                if (_triangles.size() >= kMaxFingerPoseCandidateTriangles) {
                    break;
                }
            }
            if (_triangles.empty()) {
                return false;
            }

            // Keep the original float centroid arithmetic, split axes and tie
            // order. Compute each triangle's bounds/centroid once for the build.
            std::vector<TriangleBuildData> prepared;
            prepared.reserve(_triangles.size());
            for (const auto& triangle : _triangles) {
                TriangleBuildData data{ triangle.v0, triangle.v0, triangleCentroid(triangle) };
                expandBounds(data.minimum, data.maximum, triangle.v1);
                expandBounds(data.minimum, data.maximum, triangle.v2);
                prepared.push_back(data);
            }

            _triangleIndices.resize(_triangles.size());
            for (std::size_t i = 0; i < _triangleIndices.size(); ++i) {
                _triangleIndices[i] = static_cast<std::uint32_t>(i);
            }
            _nodes.reserve(_triangles.size() * 2);
            (void)buildNode(prepared, 0, static_cast<std::uint32_t>(_triangleIndices.size()));
            return !_nodes.empty();
        }

        [[nodiscard]] bool empty() const { return _nodes.empty(); }

        [[nodiscard]] std::size_t triangleCount() const { return _triangles.size(); }

        using CandidateMask = std::bitset<kMaxFingerPoseCandidateTriangles>;

        // The caller's world triangles must be this index's same finite, bounded
        // local triangle sequence transformed by objectWorldTransform. Transform
        // node bounds forward, rather than assuming an exactly orthonormal inverse.
        // Candidate bits preserve source order, including the pad solver's last tie.
        CandidateMask collectWorldBoundsCandidates(const RE::NiTransform& objectWorldTransform,
            const RE::NiPoint3& minimum, const RE::NiPoint3& maximum,
            FingerPoseSpatialQueryStats* stats = nullptr) const
        {
            CandidateMask candidates;
            if (_nodes.empty())
                return candidates;
            double coefficients[3][3]{};
            for (int axis = 0; axis < 3; ++axis) {
                for (int local = 0; local < 3; ++local) {
                    coefficients[axis][local] = static_cast<double>(objectWorldTransform.rotate.entry[local][axis]) *
                        objectWorldTransform.scale;
                }
            }
            std::array<std::uint32_t, 64> stack{};
            std::size_t count = 1;
            while (count != 0) {
                const auto& node = _nodes[stack[--count]];
                if (stats)
                    ++stats->nodeVisits;
                bool outside = false;
                for (int axis = 0; axis < 3; ++axis) {
                    double low = axisValue(objectWorldTransform.translate, axis);
                    double high = low;
                    double magnitude = std::abs(low);
                    for (int local = 0; local < 3; ++local) {
                        const double a = coefficients[axis][local] * axisValue(node.boundsMin, local);
                        const double b = coefficients[axis][local] * axisValue(node.boundsMax, local);
                        low += (std::min)(a, b);
                        high += (std::max)(a, b);
                        magnitude += (std::max)(std::abs(a), std::abs(b));
                    }
                    // Widen only the rejection test for float world-coordinate
                    // and exact-predicate rounding. Extra candidates cannot alter
                    // results; non-finite bounds deliberately do not reject.
                    const double padding = 0.001 + magnitude * 0.00001;
                    if (std::isfinite(low) && std::isfinite(high) &&
                        (high + padding < axisValue(minimum, axis) || low - padding > axisValue(maximum, axis))) {
                        outside = true;
                        break;
                    }
                }
                if (outside)
                    continue;
                if (node.isLeaf()) {
                    for (std::uint32_t i = 0; i < node.count; ++i)
                        candidates.set(_triangleIndices[node.begin + i]);
                } else {
                    // Median splits of at most 2048 triangles need < 12 deferred siblings.
                    stack[count++] = node.right;
                    stack[count++] = node.left;
                }
            }
            return candidates;
        }

        [[nodiscard]] bool querySphereWorld(const RE::NiTransform& objectWorldTransform,
            const RE::NiPoint3& centerWorld, float radiusWorld, RE::NiPoint3* outPointWorld,
            RE::NiPoint3* outNormalWorld, FingerPoseSpatialQueryStats* stats = nullptr,
            bool orientNormalTowardCenter = true) const
        {
            if (_nodes.empty() || !isFinitePoint(centerWorld) || !std::isfinite(radiusWorld) || radiusWorld <= 0.0f ||
                !std::isfinite(objectWorldTransform.scale) || std::abs(objectWorldTransform.scale) <= 0.000001f) {
                return false;
            }

            const RE::NiPoint3 centerLocal = transform_math::worldPointToLocal(objectWorldTransform, centerWorld);
            const float radiusLocal = radiusWorld / std::abs(objectWorldTransform.scale);
            float bestDistanceSquared = radiusLocal * radiusLocal;
            std::uint32_t bestTriangleIndex = kInvalidIndex;
            RE::NiPoint3 bestPointLocal{};

            std::array<std::uint32_t, 64> stack{};
            std::size_t stackSize = 0;
            stack[stackSize++] = 0;
            while (stackSize > 0) {
                const std::uint32_t nodeIndex = stack[--stackSize];
                const Node& node = _nodes[nodeIndex];
                if (stats) {
                    ++stats->nodeVisits;
                }
                if (pointAabbDistanceSquared(centerLocal, node.boundsMin, node.boundsMax) > bestDistanceSquared) {
                    continue;
                }

                if (node.isLeaf()) {
                    for (std::uint32_t offset = 0; offset < node.count; ++offset) {
                        const std::uint32_t triangleIndex = _triangleIndices[node.begin + offset];
                        const auto& triangle = _triangles[triangleIndex];
                        if (stats) {
                            ++stats->triangleTests;
                        }
                        const RE::NiPoint3 closest =
                            grab_finger_pose_math::closestPointOnTriangle(centerLocal, triangle);
                        const float distanceSquared =
                            grab_finger_pose_math::lengthSquared(grab_finger_pose_math::sub(closest, centerLocal));
                        if (distanceSquared <= bestDistanceSquared &&
                            (bestTriangleIndex == kInvalidIndex || distanceSquared < bestDistanceSquared ||
                                triangleIndex < bestTriangleIndex)) {
                            bestDistanceSquared = distanceSquared;
                            bestTriangleIndex = triangleIndex;
                            bestPointLocal = closest;
                        }
                    }
                    continue;
                }

                const float leftDistance =
                    pointAabbDistanceSquared(centerLocal, _nodes[node.left].boundsMin, _nodes[node.left].boundsMax);
                const float rightDistance =
                    pointAabbDistanceSquared(centerLocal, _nodes[node.right].boundsMin, _nodes[node.right].boundsMax);
                const std::uint32_t nearChild = leftDistance <= rightDistance ? node.left : node.right;
                const std::uint32_t farChild = leftDistance <= rightDistance ? node.right : node.left;
                if (pointAabbDistanceSquared(centerLocal, _nodes[farChild].boundsMin, _nodes[farChild].boundsMax) <=
                    bestDistanceSquared) {
                    if (stackSize >= stack.size()) {
                        return false;
                    }
                    stack[stackSize++] = farChild;
                }
                if (pointAabbDistanceSquared(centerLocal, _nodes[nearChild].boundsMin, _nodes[nearChild].boundsMax) <=
                    bestDistanceSquared) {
                    if (stackSize >= stack.size()) {
                        return false;
                    }
                    stack[stackSize++] = nearChild;
                }
            }

            if (bestTriangleIndex == kInvalidIndex) {
                return false;
            }
            if (outPointWorld) {
                *outPointWorld = transform_math::localPointToWorld(objectWorldTransform, bestPointLocal);
            }
            if (outNormalWorld) {
                RE::NiPoint3 normalWorld = transform_math::localVectorToWorld(objectWorldTransform,
                    grab_finger_pose_math::triangleNormal(_triangles[bestTriangleIndex]));
                normalWorld = normalizedOrFallback(normalWorld, RE::NiPoint3{});
                const RE::NiPoint3 pointWorld = transform_math::localPointToWorld(objectWorldTransform, bestPointLocal);
                // Unoriented, the normal keeps the mesh winding (outward), so a caller can tell inside from outside.
                if (orientNormalTowardCenter && dotPoint(normalWorld, centerWorld - pointWorld) < 0.0f) {
                    normalWorld = normalWorld * -1.0f;
                }
                *outNormalWorld = normalWorld;
            }
            return true;
        }

        [[nodiscard]] bool queryPointInsideWorld(const RE::NiTransform& objectWorldTransform,
            const RE::NiPoint3& pointWorld, FingerPoseSpatialQueryStats* stats = nullptr) const
        {
            if (_triangles.empty() || !isFinitePoint(pointWorld) || !std::isfinite(objectWorldTransform.scale) ||
                std::abs(objectWorldTransform.scale) <= 0.000001f) {
                return false;
            }
            if (stats) {
                stats->triangleTests += static_cast<std::uint32_t>(_triangles.size());
            }
            return grab_finger_pose_math::pointInsideClosedTriangleMesh(_triangles,
                transform_math::worldPointToLocal(objectWorldTransform, pointWorld));
        }

    private:
        static constexpr std::uint32_t kInvalidIndex = 0xFFFF'FFFFu;
        static constexpr std::uint32_t kLeafTriangleCount = 8;

        struct TriangleBuildData
        {
            RE::NiPoint3 minimum{}, maximum{}, centroid{};
        };

        struct Node
        {
            RE::NiPoint3 boundsMin{};
            RE::NiPoint3 boundsMax{};
            std::uint32_t left = kInvalidIndex;
            std::uint32_t right = kInvalidIndex;
            std::uint32_t begin = 0;
            std::uint32_t count = 0;

            [[nodiscard]] bool isLeaf() const { return count > 0; }
        };

        static void expandBounds(RE::NiPoint3& boundsMin, RE::NiPoint3& boundsMax, const RE::NiPoint3& point)
        {
            boundsMin.x = (std::min)(boundsMin.x, point.x);
            boundsMin.y = (std::min)(boundsMin.y, point.y);
            boundsMin.z = (std::min)(boundsMin.z, point.z);
            boundsMax.x = (std::max)(boundsMax.x, point.x);
            boundsMax.y = (std::max)(boundsMax.y, point.y);
            boundsMax.z = (std::max)(boundsMax.z, point.z);
        }

        [[nodiscard]] static RE::NiPoint3 triangleCentroid(
            const grab_finger_pose_math::Triangle<RE::NiPoint3>& triangle)
        {
            return (triangle.v0 + triangle.v1 + triangle.v2) * (1.0f / 3.0f);
        }

        [[nodiscard]] static float axisValue(const RE::NiPoint3& point, int axis)
        {
            return axis == 0 ? point.x : (axis == 1 ? point.y : point.z);
        }

        [[nodiscard]] static float pointAabbDistanceSquared(const RE::NiPoint3& point, const RE::NiPoint3& boundsMin,
            const RE::NiPoint3& boundsMax)
        {
            const float dx =
                point.x < boundsMin.x ? boundsMin.x - point.x : (point.x > boundsMax.x ? point.x - boundsMax.x : 0.0f);
            const float dy =
                point.y < boundsMin.y ? boundsMin.y - point.y : (point.y > boundsMax.y ? point.y - boundsMax.y : 0.0f);
            const float dz =
                point.z < boundsMin.z ? boundsMin.z - point.z : (point.z > boundsMax.z ? point.z - boundsMax.z : 0.0f);
            return dx * dx + dy * dy + dz * dz;
        }

        std::uint32_t buildNode(const std::vector<TriangleBuildData>& prepared, std::uint32_t begin, std::uint32_t end)
        {
            const float infinity = std::numeric_limits<float>::infinity();
            Node node{};
            node.boundsMin = RE::NiPoint3{ infinity, infinity, infinity };
            node.boundsMax = RE::NiPoint3{ -infinity, -infinity, -infinity };
            RE::NiPoint3 centroidMin{ infinity, infinity, infinity };
            RE::NiPoint3 centroidMax{ -infinity, -infinity, -infinity };
            for (std::uint32_t i = begin; i < end; ++i) {
                const auto& triangle = prepared[_triangleIndices[i]];
                expandBounds(node.boundsMin, node.boundsMax, triangle.minimum);
                expandBounds(node.boundsMin, node.boundsMax, triangle.maximum);
                expandBounds(centroidMin, centroidMax, triangle.centroid);
            }

            const std::uint32_t nodeIndex = static_cast<std::uint32_t>(_nodes.size());
            _nodes.push_back(node);
            const std::uint32_t count = end - begin;
            if (count <= kLeafTriangleCount) {
                _nodes[nodeIndex].begin = begin;
                _nodes[nodeIndex].count = count;
                return nodeIndex;
            }

            const RE::NiPoint3 centroidExtent = centroidMax - centroidMin;
            const int splitAxis = centroidExtent.x >= centroidExtent.y && centroidExtent.x >= centroidExtent.z
                ? 0
                : (centroidExtent.y >= centroidExtent.z ? 1 : 2);
            const std::uint32_t middle = begin + count / 2;
            std::nth_element(_triangleIndices.begin() + begin, _triangleIndices.begin() + middle,
                _triangleIndices.begin() + end, [&](std::uint32_t lhs, std::uint32_t rhs) {
                    const float lhsValue = axisValue(prepared[lhs].centroid, splitAxis);
                    const float rhsValue = axisValue(prepared[rhs].centroid, splitAxis);
                    return lhsValue == rhsValue ? lhs < rhs : lhsValue < rhsValue;
                });
            const std::uint32_t left = buildNode(prepared, begin, middle);
            const std::uint32_t right = buildNode(prepared, middle, end);
            _nodes[nodeIndex].left = left;
            _nodes[nodeIndex].right = right;
            return nodeIndex;
        }

        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>> _triangles;
        std::vector<std::uint32_t> _triangleIndices;
        std::vector<Node> _nodes;
    };

    inline float triangleDistanceSquaredToPoint(const TriangleData& triangle, const RE::NiPoint3& point)
    {
        const RE::NiPoint3 centroid = (triangle.v0 + triangle.v1 + triangle.v2) * (1.0f / 3.0f);
        return (std::min)({
            distanceSquared(centroid, point),
            distanceSquared(triangle.v0, point),
            distanceSquared(triangle.v1, point),
            distanceSquared(triangle.v2, point),
        });
    }

    inline void appendTriangleCandidate(const TriangleData& triangle,
        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>>& outTriangles)
    {
        if (outTriangles.size() >= kMaxFingerPoseCandidateTriangles) {
            return;
        }
        outTriangles.push_back({ triangle.v0, triangle.v1, triangle.v2 });
    }

    inline bool isFiniteTransformForFingerPadProbe(const RE::NiTransform& transform)
    {
        if (!isFinitePoint(transform.translate) || !std::isfinite(transform.scale) ||
            std::abs(transform.scale) <= 0.000001f) {
            return false;
        }
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(transform.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    inline bool shouldRunFingerPadProbeRefinement(bool meshFingerPoseEnabled, bool grabFingerPosePublished,
        bool hasObjectTransform, bool hasLiveFingerSnapshot, bool hasTriangles)
    {
        return meshFingerPoseEnabled && grabFingerPosePublished && hasObjectTransform && hasLiveFingerSnapshot &&
            hasTriangles;
    }

    inline bool computeFingerPadCenter(const root_flattened_finger_skeleton_runtime::FingerChain& chain,
        RE::NiPoint3& outPadCenterWorld)
    {
        if (!chain.valid || !isFinitePoint(chain.points[2])) {
            outPadCenterWorld = {};
            return false;
        }

        outPadCenterWorld = chain.points[2];
        return isFinitePoint(outPadCenterWorld);
    }

    inline bool computeOpposedFingerPadProbeDirection(
        const root_flattened_finger_skeleton_runtime::Snapshot& liveFingerSnapshot,
        const root_flattened_finger_skeleton_runtime::FingerLandmark& liveFinger, std::size_t finger,
        const RE::NiPoint3& padCenterWorld, RE::NiPoint3& outProbeDirection)
    {
        outProbeDirection = {};
        if (!liveFingerSnapshot.valid || !liveFingerSnapshot.fingers[0].valid || !liveFingerSnapshot.fingers[1].valid ||
            !isFinitePoint(padCenterWorld)) {
            return false;
        }

        const auto& aimChain = finger == 0 ? liveFingerSnapshot.fingers[1] : liveFingerSnapshot.fingers[0];
        if (!aimChain.valid || !isFinitePoint(aimChain.points[2])) {
            return false;
        }

        const RE::NiPoint3 toOpposedFinger = aimChain.points[2] - padCenterWorld;
        if (distanceSquared(toOpposedFinger, RE::NiPoint3{}) <= 0.000001f) {
            return false;
        }

        const RE::NiPoint3 fallback = liveFinger.valid ? liveFinger.openDirection : RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        outProbeDirection = normalizedOrFallback(toOpposedFinger, fallback);
        return isFinitePoint(outProbeDirection) && distanceSquared(outProbeDirection, RE::NiPoint3{}) > 0.000001f;
    }

    inline RE::NiPoint3 fingerPadProbeDirection(const SolvedGrabFingerPose& pose,
        const GrabFingerPoseTargetSet& poseTargets,
        const root_flattened_finger_skeleton_runtime::FingerLandmark& liveFinger, std::size_t finger,
        const RE::NiPoint3& padCenterWorld)
    {
        RE::NiPoint3 targetWorld{};
        bool hasTarget = false;
        if (finger < pose.surfaceAimTargetValid.size() && pose.surfaceAimTargetValid[finger]) {
            targetWorld = pose.surfaceAimTarget[finger];
            hasTarget = true;
        } else if (finger < poseTargets.targetValid.size() && poseTargets.targetValid[finger]) {
            targetWorld = poseTargets.targets[finger];
            hasTarget = true;
        } else if (poseTargets.seatPointValid) {
            targetWorld = poseTargets.seatPointWorld;
            hasTarget = true;
        }

        const RE::NiPoint3 fallback = liveFinger.valid ? liveFinger.openDirection : RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        if (!hasTarget || !isFinitePoint(targetWorld)) {
            return normalizedOrFallback(fallback, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        }

        return normalizedOrFallback(targetWorld - padCenterWorld, fallback);
    }

    inline RE::NiPoint3 flippedNormal(const RE::NiPoint3& normal)
    {
        return RE::NiPoint3{ -normal.x, -normal.y, -normal.z };
    }

    inline RE::NiPoint3 orientSurfaceNormalTowardPoint(const RE::NiPoint3& rawNormal, const RE::NiPoint3& surfacePoint,
        const RE::NiPoint3& pointWorld)
    {
        RE::NiPoint3 normal = normalizedOrFallback(rawNormal, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        const RE::NiPoint3 toPoint = pointWorld - surfacePoint;
        if (distanceSquared(toPoint, RE::NiPoint3{}) > 0.000001f && dotPoint(normal, toPoint) < 0.0f) {
            normal = flippedNormal(normal);
        }
        return normal;
    }

    inline RE::NiPoint3 orientSurfaceNormalAgainstProbe(const RE::NiPoint3& rawNormal,
        const RE::NiPoint3& probeDirectionWorld)
    {
        RE::NiPoint3 normal = normalizedOrFallback(rawNormal, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        if (dotPoint(normal, probeDirectionWorld) > 0.0f) {
            normal = flippedNormal(normal);
        }
        return normal;
    }

    inline bool padSurfaceNormalCompatible(const RE::NiPoint3& candidateNormalWorld,
        const RE::NiPoint3& preferredNormalWorld, bool hasPreferredNormal)
    {
        if (!hasPreferredNormal || distanceSquared(preferredNormalWorld, RE::NiPoint3{}) <= 0.000001f) {
            return true;
        }

        const RE::NiPoint3 candidate = normalizedOrFallback(candidateNormalWorld, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        const RE::NiPoint3 preferred = normalizedOrFallback(preferredNormalWorld, candidate);
        return dotPoint(candidate, preferred) >= -0.55f;
    }

    inline bool hasFingerPadProbeLine(const FingerPadSurfaceEvidence& evidence)
    {
        return isFinitePoint(evidence.startWorld) && isFinitePoint(evidence.endWorld) &&
            distanceSquared(evidence.startWorld, evidence.endWorld) > 0.000001f;
    }

    inline FingerPadSurfaceEvidence solveFingerPadSurfaceEvidenceFromTriangles(
        const std::vector<TriangleData>& triangles, const RE::NiPoint3& padCenterWorld,
        const RE::NiPoint3& probeDirectionWorld, const RE::NiPoint3& preferredSurfaceNormalWorld,
        bool hasPreferredSurfaceNormal, FingerPadProbeOptions options = {},
        const FingerPoseTriangleSpatialIndex* spatialIndex = nullptr,
        const RE::NiTransform* spatialObjectWorldTransform = nullptr, FingerPoseSpatialQueryStats* stats = nullptr)
    {
        options = sanitizeFingerPadProbeOptions(options);
        FingerPadSurfaceEvidence result{};
        const RE::NiPoint3 probeDirection = normalizedOrFallback(probeDirectionWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        result.startWorld = padCenterWorld;
        result.endWorld = padCenterWorld + probeDirection * options.probeDistanceGameUnits;
        if (triangles.empty() || !isFinitePoint(padCenterWorld) || !isFinitePoint(probeDirection)) {
            return result;
        }

        const bool indexed = spatialIndex && spatialObjectWorldTransform && !spatialIndex->empty() &&
            spatialIndex->triangleCount() == triangles.size();
        FingerPoseTriangleSpatialIndex::CandidateMask candidates;
        if (indexed) {
            // Union of the closest-point sphere and the finite capsule probe.
            const float radius = (std::max)(options.closestSurfaceMaxDistanceGameUnits, options.probeRadiusGameUnits);
            const auto& end = result.endWorld;
            candidates = spatialIndex->collectWorldBoundsCandidates(*spatialObjectWorldTransform,
                { (std::min)(padCenterWorld.x, end.x) - radius, (std::min)(padCenterWorld.y, end.y) - radius,
                    (std::min)(padCenterWorld.z, end.z) - radius },
                { (std::max)(padCenterWorld.x, end.x) + radius, (std::max)(padCenterWorld.y, end.y) + radius,
                    (std::max)(padCenterWorld.z, end.z) + radius },
                stats);
        }
        if (performance_profiler::enabled()) {
            performance_profiler::observeValue(performance_profiler::ValueMetric::FingerPadCandidateTriangles,
                triangles.size());
            performance_profiler::observeValue(performance_profiler::ValueMetric::FingerPadTriangleTests,
                indexed ? candidates.count() : triangles.size());
        }

        FingerPadSurfaceEvidence bestClosest{};
        FingerPadSurfaceEvidence bestProbe{};
        bool bestClosestValid = false;
        bool bestProbeValid = false;
        float bestClosestDistanceSquared =
            options.closestSurfaceMaxDistanceGameUnits * options.closestSurfaceMaxDistanceGameUnits;
        float bestProbeDistance = options.probeDistanceGameUnits;

        auto considerClosest = [&](const RE::NiPoint3& point, const RE::NiPoint3& normal, float distanceSq) {
            if (!std::isfinite(distanceSq) || distanceSq > bestClosestDistanceSquared || !isFinitePoint(point) ||
                !isFinitePoint(normal)) {
                return;
            }
            if (!padSurfaceNormalCompatible(normal, preferredSurfaceNormalWorld, hasPreferredSurfaceNormal)) {
                return;
            }

            const float distance = std::sqrt(std::max(0.0f, distanceSq));
            const RE::NiPoint3 toSurface = point - padCenterWorld;
            if (distance > options.probeRadiusGameUnits &&
                dotPoint(toSurface, probeDirection) < -options.probeRadiusGameUnits) {
                return;
            }

            bestClosestDistanceSquared = distanceSq;
            bestClosest.startWorld = result.startWorld;
            bestClosest.endWorld = result.endWorld;
            bestClosest.hitPointWorld = point;
            bestClosest.hitNormalWorld = normal;
            bestClosest.distanceGameUnits = distance;
            bestClosest.quality =
                std::clamp(1.0f - (distance / options.closestSurfaceMaxDistanceGameUnits), 0.0f, 1.0f);
            bestClosest.hit = true;
            bestClosest.fromClosestSurface = true;
            bestClosest.padMayBeInsideSurface = distance <= options.probeRadiusGameUnits;
            bestClosestValid = true;
        };

        auto considerProbe = [&](const RE::NiPoint3& point, const RE::NiPoint3& normal, float travelDistance) {
            if (!std::isfinite(travelDistance) || travelDistance > bestProbeDistance || !isFinitePoint(point) ||
                !isFinitePoint(normal)) {
                return;
            }
            if (!padSurfaceNormalCompatible(normal, preferredSurfaceNormalWorld, hasPreferredSurfaceNormal)) {
                return;
            }

            bestProbeDistance = travelDistance;
            const float pointDistance = std::sqrt(std::max(0.0f, distanceSquared(point, padCenterWorld)));
            bestProbe.startWorld = result.startWorld;
            bestProbe.endWorld = result.endWorld;
            bestProbe.hitPointWorld = point;
            bestProbe.hitNormalWorld = normal;
            bestProbe.distanceGameUnits = pointDistance;
            bestProbe.quality = std::clamp(1.0f - (travelDistance / options.probeDistanceGameUnits), 0.0f, 1.0f);
            bestProbe.hit = true;
            bestProbe.fromClosestSurface = false;
            bestProbe.padMayBeInsideSurface =
                pointDistance <= options.probeRadiusGameUnits || travelDistance <= options.probeRadiusGameUnits;
            bestProbeValid = true;
        };

        for (std::size_t triangleIndex = 0; triangleIndex < triangles.size(); ++triangleIndex) {
            if (indexed && !candidates.test(triangleIndex))
                continue;
            if (stats)
                ++stats->triangleTests;
            const auto& triangleData = triangles[triangleIndex];
            if (!isFinitePoint(triangleData.v0) || !isFinitePoint(triangleData.v1) || !isFinitePoint(triangleData.v2)) {
                continue;
            }

            const grab_finger_pose_math::Triangle<RE::NiPoint3> triangle{ triangleData.v0, triangleData.v1,
                triangleData.v2 };
            const RE::NiPoint3 rawNormal = grab_finger_pose_math::triangleNormal(triangle);
            if (distanceSquared(rawNormal, RE::NiPoint3{}) <= 0.000001f) {
                continue;
            }

            const RE::NiPoint3 closestPoint = grab_finger_pose_math::closestPointOnTriangle(padCenterWorld, triangle);
            const RE::NiPoint3 closestNormal = orientSurfaceNormalTowardPoint(rawNormal, closestPoint, padCenterWorld);
            considerClosest(closestPoint, closestNormal, distanceSquared(closestPoint, padCenterWorld));

            float travelDistance = 0.0f;
            RE::NiPoint3 probeHitPoint{};
            bool hasProbeHitPoint = false;
            if (grab_finger_pose_math::rayTriangleIntersection(padCenterWorld, probeDirection, triangle,
                    options.probeDistanceGameUnits, travelDistance)) {
                probeHitPoint = padCenterWorld + probeDirection * travelDistance;
                hasProbeHitPoint = true;
            } else if (grab_finger_pose_math::probeCapsuleTriangleIntersection(padCenterWorld, probeDirection, triangle,
                           options.probeDistanceGameUnits, options.probeRadiusGameUnits, travelDistance,
                           &probeHitPoint)) {
                hasProbeHitPoint = true;
            }
            if (hasProbeHitPoint) {
                considerProbe(probeHitPoint, orientSurfaceNormalAgainstProbe(rawNormal, probeDirection),
                    travelDistance);
            }
        }

        if (bestClosestValid &&
            (!bestProbeValid || bestClosest.padMayBeInsideSurface || bestClosest.quality >= bestProbe.quality)) {
            return bestClosest;
        }
        if (bestProbeValid) {
            return bestProbe;
        }
        return result;
    }

    /*
     * The proximity-scaled pad "open bias" (and its thumb over-open variant)
     * was removed deliberately: it mutated PUBLISHED values from live
     * pad-to-surface distance AFTER the held-re-solve deadband, forming a
     * publish->pad-moves->bias-changes feedback loop with no hysteresis -
     * the in-game finger twitch. Over-open is now a first-class swept-arc
     * result (baked rows past 1.0, capped per call), deterministic against
     * the mesh instead of reactive to the skeleton. Do not reintroduce a
     * live-skeleton-driven value mutation on the publish path.
     */
    inline bool shouldAcceptFingerPadTarget(const SolvedGrabFingerPose& pose,
        const GrabFingerPoseTargetSet& poseTargets, const FingerPadSurfaceEvidence& evidence, std::size_t finger,
        FingerPadProbeOptions options)
    {
        options = sanitizeFingerPadProbeOptions(options);
        if (!evidence.hit || evidence.quality < options.targetOverrideMinQuality ||
            !isFinitePoint(evidence.hitPointWorld)) {
            return false;
        }
        if (!pose.thumbSurfaceFollowAllowed && finger < 2) {
            return false;
        }

        const bool existingTarget =
            finger < pose.surfaceAimTargetValid.size() && pose.surfaceAimTargetValid[finger] != 0;
        const bool explicitFingerTarget =
            finger < poseTargets.targetValid.size() && poseTargets.targetValid[finger] != 0;
        if (!existingTarget) {
            return true;
        }

        const float existingDistance =
            std::sqrt(std::max(0.0f, distanceSquared(pose.surfaceAimTarget[finger], evidence.startWorld)));
        if (evidence.padMayBeInsideSurface) {
            return true;
        }
        if (explicitFingerTarget) {
            return false;
        }

        constexpr float kReplacementSlopGameUnits = 0.25f;
        return evidence.distanceGameUnits + kReplacementSlopGameUnits < existingDistance;
    }

    inline bool refineGrabFingerPoseWithPadProbes(SolvedGrabFingerPose& pose,
        const std::vector<TriangleData>& worldTriangles, const GrabFingerPoseTargetSet& poseTargets,
        const root_flattened_finger_skeleton_runtime::Snapshot& liveFingerSnapshot,
        const RE::NiTransform& objectWorldTransform, bool meshFingerPoseEnabled, bool grabFingerPosePublished,
        std::array<FingerPadSurfaceEvidence, 5>& outEvidence, bool allowSurfaceTargetRefinement = true,
        FingerPadProbeOptions options = {}, const FingerPoseTriangleSpatialIndex* spatialIndex = nullptr)
    {
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::GrabFingerPadProbes);
        options = sanitizeFingerPadProbeOptions(options);
        outEvidence = {};
        const bool liveSnapshotValid = liveFingerSnapshot.valid;
        const bool shouldRun = shouldRunFingerPadProbeRefinement(meshFingerPoseEnabled, grabFingerPosePublished,
            isFiniteTransformForFingerPadProbe(objectWorldTransform), liveSnapshotValid, !worldTriangles.empty());
        if (!shouldRun || !pose.solved) {
            return false;
        }

        const auto liveLandmarks = root_flattened_finger_skeleton_runtime::buildLandmarkSet(liveFingerSnapshot);
        if (!liveLandmarks.valid) {
            return false;
        }

        bool anyProbeLine = false;
        for (std::size_t finger = 0; finger < outEvidence.size(); ++finger) {
            RE::NiPoint3 padCenterWorld{};
            if (!computeFingerPadCenter(liveFingerSnapshot.fingers[finger], padCenterWorld)) {
                continue;
            }

            RE::NiPoint3 probeDirection{};
            if (!computeOpposedFingerPadProbeDirection(liveFingerSnapshot, liveLandmarks.fingers[finger], finger,
                    padCenterWorld, probeDirection)) {
                probeDirection =
                    fingerPadProbeDirection(pose, poseTargets, liveLandmarks.fingers[finger], finger, padCenterWorld);
            }
            const bool hasPreferredNormal =
                finger < pose.surfaceAimNormalValid.size() && pose.surfaceAimNormalValid[finger] != 0;
            const RE::NiPoint3 preferredNormal = hasPreferredNormal ? pose.surfaceAimNormal[finger] : RE::NiPoint3{};
            FingerPadSurfaceEvidence evidence =
                solveFingerPadSurfaceEvidenceFromTriangles(worldTriangles, padCenterWorld, probeDirection,
                    preferredNormal, hasPreferredNormal, options, spatialIndex, &objectWorldTransform);
            outEvidence[finger] = evidence;
            anyProbeLine = anyProbeLine || hasFingerPadProbeLine(evidence);
            if (!evidence.hit) {
                continue;
            }

            if (allowSurfaceTargetRefinement &&
                shouldAcceptFingerPadTarget(pose, poseTargets, evidence, finger, options)) {
                pose.surfaceAimTarget[finger] = evidence.hitPointWorld;
                pose.surfaceAimTargetValid[finger] = 1;
                if (distanceSquared(evidence.hitNormalWorld, RE::NiPoint3{}) > 0.000001f) {
                    pose.surfaceAimNormal[finger] =
                        normalizedOrFallback(evidence.hitNormalWorld, liveLandmarks.palmNormalWorld);
                    pose.surfaceAimNormalValid[finger] = 1;
                }
            }
        }

        return anyProbeLine;
    }

    // Red surface targets are contact hints only. Side-to-side splay must stay anatomically bounded.
    inline constexpr float kMaxSurfaceContactSplayRadians = 0.08726646259971647f;
    inline constexpr float kDefaultSurfaceContactSplayMaxRadians = kMaxSurfaceContactSplayRadians;

    inline bool hasSurfaceContactSplayCandidates(const SolvedGrabFingerPose& pose)
    {
        for (const auto targetValid : pose.surfaceAimTargetValid) {
            if (targetValid != 0) {
                return true;
            }
        }
        return false;
    }

    inline RE::NiPoint3 rejectFromDirection(const RE::NiPoint3& value, const RE::NiPoint3& direction)
    {
        return RE::NiPoint3{
            value.x - direction.x * dotPoint(value, direction),
            value.y - direction.y * dotPoint(value, direction),
            value.z - direction.z * dotPoint(value, direction),
        };
    }

    inline float signedPalmPlaneSplayRadians(const RE::NiPoint3& openDirectionWorld,
        const RE::NiPoint3& targetDirectionWorld, const RE::NiPoint3& palmNormalWorld)
    {
        const RE::NiPoint3 palmNormal = normalizedOrFallback(palmNormalWorld, RE::NiPoint3{ 0.0f, 0.0f, -1.0f });
        const RE::NiPoint3 openPlanar = rejectFromDirection(openDirectionWorld, palmNormal);
        const RE::NiPoint3 targetPlanar = rejectFromDirection(targetDirectionWorld, palmNormal);

        if (dotPoint(openPlanar, openPlanar) <= 0.000001f || dotPoint(targetPlanar, targetPlanar) <= 0.000001f) {
            return 0.0f;
        }

        const RE::NiPoint3 open = normalizedOrFallback(openPlanar, openDirectionWorld);
        const RE::NiPoint3 target = normalizedOrFallback(targetPlanar, open);
        const float sinAngle = std::clamp(dotPoint(palmNormal, crossPoint(open, target)), -1.0f, 1.0f);
        const float cosAngle = std::clamp(dotPoint(open, target), -1.0f, 1.0f);
        const float angle = std::atan2(sinAngle, cosAngle);
        return std::isfinite(angle) ? angle : 0.0f;
    }

    inline float clampSurfaceContactSplayRadians(float splayRadians,
        float maxSplayRadians = kDefaultSurfaceContactSplayMaxRadians)
    {
        const float requestedMax =
            std::isfinite(maxSplayRadians) ? std::max(0.0f, maxSplayRadians) : kDefaultSurfaceContactSplayMaxRadians;
        const float resolvedMax = std::min(requestedMax, kMaxSurfaceContactSplayRadians);
        if (!std::isfinite(splayRadians) || resolvedMax <= 0.0f) {
            return 0.0f;
        }
        return std::clamp(splayRadians, -resolvedMax, resolvedMax);
    }

    inline bool buildSurfaceContactSplayValues(const SolvedGrabFingerPose& pose,
        const root_flattened_finger_skeleton_runtime::Snapshot& liveFingerSnapshot,
        const std::array<RE::NiPoint3, 5>& commandedOpenDirectionsWorld, std::array<float, 5>& outSplayRadians,
        float maxSplayRadians = kDefaultSurfaceContactSplayMaxRadians)
    {
        outSplayRadians = {};
        if (!hasSurfaceContactSplayCandidates(pose)) {
            return false;
        }

        const auto liveLandmarks = root_flattened_finger_skeleton_runtime::buildLandmarkSet(liveFingerSnapshot);
        if (!liveLandmarks.valid) {
            return false;
        }

        bool anySplay = false;
        for (std::size_t finger = 0; finger < pose.surfaceAimTargetValid.size(); ++finger) {
            if (pose.surfaceAimTargetValid[finger] == 0 || !liveLandmarks.fingers[finger].valid ||
                !isFinitePoint(commandedOpenDirectionsWorld[finger]) ||
                distanceSquared(commandedOpenDirectionsWorld[finger], RE::NiPoint3{}) <= 0.000001f) {
                continue;
            }

            const RE::NiPoint3 toSurface = pose.surfaceAimTarget[finger] - liveLandmarks.fingers[finger].base;
            if (distanceSquared(toSurface, RE::NiPoint3{}) <= 0.000001f) {
                continue;
            }

            const float splayRadians =
                clampSurfaceContactSplayRadians(signedPalmPlaneSplayRadians(commandedOpenDirectionsWorld[finger],
                                                    toSurface, liveLandmarks.palmNormalWorld),
                    maxSplayRadians);
            if (std::abs(splayRadians) <= 0.0001f) {
                continue;
            }

            outSplayRadians[finger] = splayRadians;
            anySplay = true;
        }

        return anySplay;
    }

    inline bool resolveSurfaceContactSplayValues(bool isLeft, const SolvedGrabFingerPose& pose,
        std::array<float, 5>& outSplayRadians, float maxSplayRadians = kDefaultSurfaceContactSplayMaxRadians);

    inline GrabFingerPoseTargetSet makeSharedGripPoseTarget(const RE::NiPoint3& grabGripPoint,
        const RE::NiPoint3& grabGripNormal = RE::NiPoint3{})
    {
        GrabFingerPoseTargetSet targets{};
        targets.seatPointWorld = grabGripPoint;
        targets.seatPointValid = true;
        if (distanceSquared(grabGripNormal, RE::NiPoint3{}) > 0.000001f) {
            targets.seatNormalWorld = normalizeDirection(grabGripNormal);
            targets.seatNormalValid = true;
        }
        targets.useSeatPointForMissingTargets = true;
        return targets;
    }

    inline void appendCandidateTriangles(const std::vector<TriangleData>& triangles, const RE::NiPoint3& grabGripPoint,
        float maxTriangleDistanceSquared, std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>>& outTriangles)
    {
        outTriangles.clear();
        if (!std::isfinite(maxTriangleDistanceSquared) || maxTriangleDistanceSquared <= 0.0f) {
            return;
        }

        outTriangles.reserve((std::min)(triangles.size(), kMaxFingerPoseCandidateTriangles));
        for (const auto& triangle : triangles) {
            if (triangleDistanceSquaredToPoint(triangle, grabGripPoint) <= maxTriangleDistanceSquared) {
                appendTriangleCandidate(triangle, outTriangles);
                if (outTriangles.size() >= kMaxFingerPoseCandidateTriangles) {
                    break;
                }
            }
        }
    }

    inline void appendCandidateTriangles(const std::vector<TriangleData>& triangles,
        const GrabFingerPoseTargetSet& poseTargets, float maxTriangleDistanceSquared,
        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>>& outTriangles)
    {
        outTriangles.clear();
        if (!std::isfinite(maxTriangleDistanceSquared) || maxTriangleDistanceSquared <= 0.0f) {
            return;
        }

        if (poseTargets.useWholeMeshForMissingTargets && poseTargets.targetCount == 0) {
            outTriangles.reserve((std::min)(triangles.size(), kMaxFingerPoseCandidateTriangles));
            if (!poseTargets.seatPointValid || triangles.size() <= kMaxFingerPoseCandidateTriangles) {
                for (const auto& triangle : triangles) {
                    appendTriangleCandidate(triangle, outTriangles);
                    if (outTriangles.size() >= kMaxFingerPoseCandidateTriangles) {
                        break;
                    }
                }
                return;
            }

            struct RankedTriangle
            {
                float distanceSquared = 0.0f;
                std::size_t index = 0;
            };

            std::vector<RankedTriangle> rankedTriangles;
            rankedTriangles.reserve(triangles.size());
            for (std::size_t i = 0; i < triangles.size(); ++i) {
                rankedTriangles.push_back(RankedTriangle{
                    triangleDistanceSquaredToPoint(triangles[i], poseTargets.seatPointWorld),
                    i,
                });
            }

            const auto selectedEnd = rankedTriangles.begin() + kMaxFingerPoseCandidateTriangles;
            std::nth_element(rankedTriangles.begin(), selectedEnd, rankedTriangles.end(),
                [](const RankedTriangle& lhs, const RankedTriangle& rhs) {
                    if (lhs.distanceSquared == rhs.distanceSquared) {
                        return lhs.index < rhs.index;
                    }
                    return lhs.distanceSquared < rhs.distanceSquared;
                });
            std::sort(rankedTriangles.begin(), selectedEnd, [](const RankedTriangle& lhs, const RankedTriangle& rhs) {
                if (lhs.distanceSquared == rhs.distanceSquared) {
                    return lhs.index < rhs.index;
                }
                return lhs.distanceSquared < rhs.distanceSquared;
            });

            for (auto it = rankedTriangles.begin(); it != selectedEnd; ++it) {
                appendTriangleCandidate(triangles[it->index], outTriangles);
            }
            return;
        }

        std::array<RE::NiPoint3, 6> points{};
        std::size_t pointCount = 0;
        if (poseTargets.seatPointValid) {
            points[pointCount++] = poseTargets.seatPointWorld;
        }
        for (std::size_t finger = 0; finger < poseTargets.targets.size() && pointCount < points.size(); ++finger) {
            if (poseTargets.targetValid[finger]) {
                points[pointCount++] = poseTargets.targets[finger];
            }
        }
        if (pointCount == 0) {
            return;
        }

        outTriangles.reserve((std::min)(triangles.size(), kMaxFingerPoseCandidateTriangles));
        for (const auto& triangle : triangles) {
            bool nearAnyTarget = false;
            for (std::size_t pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
                const auto& point = points[pointIndex];
                const float bestVertexOrCentroidDistance = triangleDistanceSquaredToPoint(triangle, point);
                if (bestVertexOrCentroidDistance <= maxTriangleDistanceSquared) {
                    nearAnyTarget = true;
                    break;
                }
            }
            if (nearAnyTarget) {
                appendTriangleCandidate(triangle, outTriangles);
                if (outTriangles.size() >= kMaxFingerPoseCandidateTriangles) {
                    break;
                }
            }
        }
    }

    inline SolvedGrabFingerPose solveGrabFingerPoseFromTriangles(const std::vector<TriangleData>& triangles,
        const RE::NiTransform& handTransform, bool isLeft, const RE::NiPoint3& grabAnchorWorld,
        const GrabFingerPoseTargetSet& poseTargets, float minValue, float maxTriangleDistanceSquared = 100.0f,
        bool useCurveSolver = true,
        const root_flattened_finger_skeleton_runtime::Snapshot* liveFingerSnapshot = nullptr,
        bool rejectBacksideHits = true, float surfacePlaneToleranceGameUnits = 1.5f, bool allowSurfaceAimTargets = true,
        float sweepContactRadiusGameUnits = 1.0f, float unreachableFingerOpenValue = -1.0f,
        float thumbSweepMaxOpenValue = grab_finger_pose_math::kMaxOverOpenValue,
        float fingerSweepMaxOpenValue = grab_finger_pose_math::kMaxOverOpenValue,
        const std::array<RE::NiPoint3, 5>* commandedOpenDirectionsWorld = nullptr,
        const FingerPoseTriangleSpatialIndex* spatialIndex = nullptr,
        const RE::NiTransform* spatialObjectWorldTransform = nullptr,
        FingerSweepDebugCapture* outSweepDebugCapture = nullptr,
        FingerPoseMeshRelation meshRelation = FingerPoseMeshRelation::CurrentMeshRequiresVirtualSeat)
    {
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::GrabFingerSolve);
        if (outSweepDebugCapture) {
            *outSweepDebugCapture = {};
        }
        SolvedGrabFingerPose result{};
        const float clampedMin = std::clamp(minValue, 0.0f, 1.0f);
        result.values = { clampedMin, clampedMin, clampedMin, clampedMin, clampedMin };
        result.poseTargetCount = static_cast<int>(poseTargets.targetCount);
        const bool useSpatialIndex = useCurveSolver && spatialIndex && !spatialIndex->empty() &&
            spatialObjectWorldTransform && std::isfinite(spatialObjectWorldTransform->scale) &&
            std::abs(spatialObjectWorldTransform->scale) > 0.000001f;
        result.usedSpatialIndex = useSpatialIndex;

        if ((!useSpatialIndex && triangles.empty()) || !poseTargets.seatPointValid) {
            return result;
        }

        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>> candidateTriangles;
        if (useSpatialIndex) {
            result.candidateTriangleCount = static_cast<int>(spatialIndex->triangleCount());
        } else {
            appendCandidateTriangles(triangles, poseTargets, maxTriangleDistanceSquared, candidateTriangles);
            result.candidateTriangleCount = static_cast<int>(candidateTriangles.size());
        }
        if (result.candidateTriangleCount == 0) {
            return result;
        }

        constexpr float kFingerReachPadding = 6.0f;
        constexpr float kMinFingerProbeDistance = 6.0f;
        constexpr float kMaxFingerProbeDistance = 26.0f;
        constexpr float kFingerProbeRadius = 1.25f;
        const RE::NiPoint3 fallbackDirection =
            transformHandspaceDirection(handTransform, RE::NiPoint3(1.0f, 0.0f, 0.0f), isLeft);
        const auto liveLandmarks = liveFingerSnapshot
            ? root_flattened_finger_skeleton_runtime::buildLandmarkSet(*liveFingerSnapshot)
            : root_flattened_finger_skeleton_runtime::LandmarkSet{};
        if (!liveLandmarks.valid) {
            return result;
        }
        const bool inPowerArmor = liveFingerSnapshot && liveFingerSnapshot->inPowerArmor;
        const RE::NiPoint3 curlNormalWorld = liveLandmarks.palmNormalWorld;
        result.usedLiveRootFlattenedFingerBones = true;
        FingerPoseSpatialQueryStats spatialQueryStats{};

        for (std::size_t finger = 0; finger < result.values.size(); ++finger) {
            const auto& live = liveLandmarks.fingers[finger];
            const float maxCurlAngle =
                grab_finger_pose_math::bakedCalibratedFingerMaxAngleRadians(finger, isLeft, inPowerArmor);
            /*
             * The indexed regular-grab path already queries triangles at the
             * frozen commanded seat, so its curve pivot must remain the live
             * proximal bone. Current-geometry consumers (the support-hand
             * solver) retain the explicit virtual-seat translation.
             */
            const RE::NiPoint3 baseWorld = resolveFingerSweepBaseWorld(live.base, poseTargets.seatPointWorld,
                grabAnchorWorld, poseTargets.useWholeMeshForMissingTargets, poseTargets.targetCount, meshRelation);
            const bool hasFingerTarget = poseTargets.targetValid[finger] != 0;
            const bool useTarget = hasFingerTarget || poseTargets.useSeatPointForMissingTargets;
            const RE::NiPoint3 fingerTargetWorld =
                hasFingerTarget ? poseTargets.targets[finger] : poseTargets.seatPointWorld;
            const bool hasFingerTargetNormal = hasFingerTarget && poseTargets.targetNormalValid[finger] != 0;
            const bool useTargetNormal = hasFingerTargetNormal || poseTargets.seatNormalValid;
            const RE::NiPoint3 fingerTargetNormalWorld =
                hasFingerTargetNormal ? poseTargets.targetNormals[finger] : poseTargets.seatNormalWorld;
            const float fingerOpenLengthWorld = live.length;
            /*
             * The live chord (pad - base) is rotated by the chain's CURRENT
             * curl, but the arc reconstruction treats it as the fully-open
             * zero reference. Solving from a curled chain therefore stopped
             * every finger short by exactly the current curl angle (the
             * "finger-width air gap").
             *
             * Anchor priority:
             * 1. A caller-provided COMMANDED open direction (hFRIK's authored
             *    open-pose chain in hand-bone space, rotated by the hand
             *    bone). Zero feedback by construction: no rendered finger
             *    data can influence it, so a static hand-object relation has
             *    one deterministic endpoint.
             * 2. Otherwise, estimate the current curl from the chord length
             *    via the baked Tip reach table (deferred pinch callers, where
             *    the inversion tracks the ACTUAL pose). THE
             *    THUMB IS EXCLUDED from the estimate: its chord
             *    shortens from opposition and twist - motion outside the arc
             *    plane - so the inversion reads a large spurious curl
             *    (session evidence: thumb inside the mesh on
             *    every grab). Without a commanded anchor the thumb uses its
             *    raw chord.
             */
            RE::NiPoint3 openDirectionWorld = live.openDirection;
            const bool hasCommandedOpenDirection = commandedOpenDirectionsWorld &&
                isFinitePoint((*commandedOpenDirectionsWorld)[finger]) &&
                distanceSquared((*commandedOpenDirectionsWorld)[finger], RE::NiPoint3{}) > 0.25f;
            if (hasCommandedOpenDirection) {
                openDirectionWorld = (*commandedOpenDirectionsWorld)[finger];
            } else if (finger != 0) {
                const auto& liveChain = liveFingerSnapshot->fingers[finger];
                const float liveChordLength = std::sqrt(distanceSquared(liveChain.points[2], liveChain.points[0]));
                const auto chordCurl = grab_finger_pose_math::estimateCalibratedChainCurlFromChord(finger, isLeft,
                    inPowerArmor, fingerOpenLengthWorld, liveChordLength);
                if (chordCurl.valid && std::fabs(chordCurl.chordAngleRadians) > 0.0035f) {
                    openDirectionWorld =
                        normalizedOrFallback(grab_finger_pose_math::rotateAroundUnitAxis(openDirectionWorld,
                                                 curlNormalWorld, -chordCurl.chordAngleRadians * chordCurl.normalSign),
                            openDirectionWorld);
                }
            }
            const RE::NiPoint3 thumbAlternateCurlNormalWorld = liveThumbAlternateCurlNormalWorld(openDirectionWorld,
                curlNormalWorld, baseWorld, grabAnchorWorld, isLeft);
            const RE::NiPoint3 toContact = useTarget ? fingerTargetWorld - baseWorld : openDirectionWorld;
            const float distanceToContact =
                useTarget ? std::sqrt(distanceSquared(fingerTargetWorld, baseWorld)) : fingerOpenLengthWorld;
            const float probeDistance =
                std::clamp(distanceToContact + kFingerReachPadding, kMinFingerProbeDistance, kMaxFingerProbeDistance);
            const RE::NiPoint3 probeDirection = normalizedOrFallback(toContact, fallbackDirection);
            result.probeStart[finger] = baseWorld;
            result.probeEnd[finger] = baseWorld + probeDirection * probeDistance;
            if (finger == 0) {
                result.hasThumbAlternateCurveFrame = true;
                result.thumbAlternateCurveBaseWorld = baseWorld;
                result.thumbAlternateCurveOpenDirectionWorld =
                    normalizedOrFallback(openDirectionWorld, fallbackDirection);
                result.thumbAlternateCurveNormalWorld =
                    normalizedOrFallback(thumbAlternateCurlNormalWorld, curlNormalWorld);
                result.thumbAlternateCurveMaxCurlAngleRadians = maxCurlAngle;
            }

            auto solved = grab_finger_pose_math::FingerCurlValue{};
            solved.value = clampedMin;
            RE::NiPoint3 fallbackHitPoint{};
            bool fallbackHitPointValid = false;
            bool curveSolverRan = false;
            bool sweepDebugCurveAvailable = false;
            RE::NiPoint3 sweepDebugCurveNormalWorld{};
            grab_finger_calibration_data::BakedGrabThumbLane sweepDebugThumbLane =
                grab_finger_calibration_data::BakedGrabThumbLane::Wrap;
            if (useCurveSolver) {
                const bool isThumb = finger == 0;
                /*
                 * Swept-arc solver: the finger closes along its baked hFRIK
                 * arc and stops at first volumetric contact. Backside/surface
                 * gates are unnecessary here — far-side geometry lies later on
                 * the arc than the near surface, so a first-contact stop can
                 * never select it.
                 */
                grab_finger_pose_math::ThumbAwareFingerCurveCurlValue<RE::NiPoint3> curveSolved{};
                if (useSpatialIndex) {
                    auto solveSpatialCurve =
                        [&](const grab_finger_pose_math::CalibratedFingerCurve<RE::NiPoint3>& curve,
                            float curveMinValue, float curveContactRadius, float curveMaxOpenValue) {
                            auto sphereContact = [&](const RE::NiPoint3& centerWorld, float radiusWorld,
                                                     RE::NiPoint3* outPointWorld, RE::NiPoint3* outNormalWorld) {
                                return spatialIndex->querySphereWorld(*spatialObjectWorldTransform, centerWorld,
                                    radiusWorld, outPointWorld, outNormalWorld, &spatialQueryStats);
                            };
                            auto pointInside = [&](const RE::NiPoint3& pointWorld) {
                                return spatialIndex->queryPointInsideWorld(*spatialObjectWorldTransform, pointWorld,
                                    &spatialQueryStats);
                            };
                            return grab_finger_pose_math::sweepCalibratedFingerCurveCurlValueWithContactQuery(curve,
                                curveMinValue, curveContactRadius, curveMaxOpenValue, sphereContact, pointInside);
                        };
                    curveSolved = grab_finger_pose_math::sweepThumbAwareCalibratedFingerCurveCurlValueWithCurveSolver<
                        RE::NiPoint3>(finger, isLeft, inPowerArmor, baseWorld, curlNormalWorld,
                        thumbAlternateCurlNormalWorld, openDirectionWorld, fingerOpenLengthWorld, clampedMin, isThumb,
                        sweepContactRadiusGameUnits, isThumb ? thumbSweepMaxOpenValue : fingerSweepMaxOpenValue,
                        solveSpatialCurve);
                } else {
                    curveSolved = grab_finger_pose_math::sweepThumbAwareCalibratedFingerCurveCurlValue(
                        candidateTriangles, finger, isLeft, inPowerArmor, baseWorld, curlNormalWorld,
                        thumbAlternateCurlNormalWorld, openDirectionWorld, fingerOpenLengthWorld, clampedMin, isThumb,
                        sweepContactRadiusGameUnits, isThumb ? thumbSweepMaxOpenValue : fingerSweepMaxOpenValue);
                }
                curveSolverRan = true;
                solved = curveSolved.value;
                sweepDebugCurveAvailable = useSpatialIndex && outSweepDebugCapture;
                sweepDebugCurveNormalWorld = curveSolved.selectedThumbLaneNormal;
                sweepDebugThumbLane = curveSolved.selectedThumbLane;
                result.usedAlternateThumbCurve = result.usedAlternateThumbCurve || curveSolved.usedAlternateThumbCurve;
                if (isThumb) {
                    result.hasThumbCurveDiagnostics = true;
                    result.thumbPrimaryCurve = curveSolved.primary;
                    result.thumbAlternateCurve = curveSolved.alternateThumb;
                    result.thumbSidePadCurve = curveSolved.sidePadThumb;
                    result.selectedThumbLane = curveSolved.selectedThumbLane;
                    result.selectedThumbLaneNormalBlend = curveSolved.selectedThumbLaneNormalBlend;
                    result.selectedThumbLaneLocalCorrectionStrength =
                        curveSolved.selectedThumbLaneLocalCorrectionStrength;
                    if (curveSolved.usedAlternateThumbCurve) {
                        result.thumbAlternateCurveNormalWorld =
                            normalizedOrFallback(curveSolved.selectedThumbLaneNormal, thumbAlternateCurlNormalWorld);
                        if (curveSolved.selectedThumbLaneMaxCurlAngleRadians > 0.0001f) {
                            result.thumbAlternateCurveMaxCurlAngleRadians =
                                curveSolved.selectedThumbLaneMaxCurlAngleRadians;
                        }
                    }
                    const auto& selectedThumbCurve = curveSolved.selectedThumbCurve;
                    const bool alternateThumbSurfaceHit = curveSolved.usedAlternateThumbCurve &&
                        selectedThumbCurve.hit && selectedThumbCurve.hasHitPoint &&
                        !selectedThumbCurve.openedByBehindContact &&
                        selectedThumbCurve.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::FrontValid;
                    result.usedAlternateThumbSurfaceHit = alternateThumbSurfaceHit;
                    if (alternateThumbSurfaceHit) {
                        result.surfaceAimTarget[0] = RE::NiPoint3{ selectedThumbCurve.hitPointX,
                            selectedThumbCurve.hitPointY, selectedThumbCurve.hitPointZ };
                        result.surfaceAimTargetValid[0] = 1;
                        if (selectedThumbCurve.hasHitNormal) {
                            result.surfaceAimNormal[0] =
                                normalizedOrFallback(RE::NiPoint3{ selectedThumbCurve.hitNormalX,
                                                         selectedThumbCurve.hitNormalY, selectedThumbCurve.hitNormalZ },
                                    curlNormalWorld);
                            result.surfaceAimNormalValid[0] = 1;
                        }
                    }
                }
            }
            if (useTarget && !solved.hit && !curveSolverRan) {
                solved = grab_finger_pose_math::solveFingerCurlValue(candidateTriangles, baseWorld, probeDirection,
                    probeDistance, clampedMin, kFingerProbeRadius, curlNormalWorld, openDirectionWorld, maxCurlAngle,
                    fingerTargetWorld, fingerTargetNormalWorld, rejectBacksideHits && useTargetNormal,
                    surfacePlaneToleranceGameUnits, &fallbackHitPoint);
                fallbackHitPointValid =
                    solved.hit && solved.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::FrontValid;
            }
            /*
             * Generic grab poses often use whole-mesh probing instead of
             * explicit per-finger targets. A miss in that mode deliberately
             * closes to 0.3 so missing targets are visible without reusing the
             * palm seat point as fake finger evidence.
             */
            if (!solved.hit &&
                (!useTarget || solved.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::BackSurface ||
                    solved.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::Rejected)) {
                solved.value = missedFingerCurlFallbackValue(useTarget, solved.hitKind, clampedMin);
                solved.rawCurveValue = solved.value;
            }
            /*
             * Whole arc out of reach: the object cannot be touched from the
             * current relative pose. During pull-to-grab flight callers pass
             * the anticipation value so the hand awaits arrival open instead
             * of closing on nothing; the legacy sentinel (-1) keeps the plain
             * miss semantics for at-hand solves.
             */
            if (curveSolverRan && !solved.hit && solved.outOfReach && unreachableFingerOpenValue >= 0.0f) {
                solved.value = std::clamp(unreachableFingerOpenValue, clampedMin, 1.0f);
                solved.rawCurveValue = solved.value;
            }
            if (sweepDebugCurveAvailable) {
                const auto selectedDebugCurve =
                    makeSelectedFingerSweepDebugCurve(finger, isLeft, inPowerArmor, baseWorld,
                        sweepDebugCurveNormalWorld, openDirectionWorld, fingerOpenLengthWorld, sweepDebugThumbLane);
                (void)captureFingerSweepDebugCurve((*outSweepDebugCapture).fingers[finger], selectedDebugCurve, solved,
                    clampedMin, sweepDebugThumbLane, *spatialObjectWorldTransform);
            }
            result.values[finger] = solved.value;
            result.hitKind[finger] = solved.hitKind;
            /*
             * Record the selected contact-row rotation for diagnostics. Only
             * palm-plane sweep contacts qualify: the thumb's alternate lanes
             * rotate in a blended plane the palm-normal de-rotation cannot
             * invert, and a value floored by minValue no longer matches its
             * contact row.
             */
            if (curveSolverRan && solved.hit &&
                solved.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::FrontValid &&
                solved.rawCurveValue >= clampedMin - 0.0001f && (finger != 0 || !result.usedAlternateThumbCurve)) {
                const auto& bakedAnchorProfile =
                    grab_finger_calibration_data::bakedGrabFingerHandProfile(isLeft, inPowerArmor);
                const float bakedAnchorNormalSign = bakedAnchorProfile.fingers[finger].normalSign < 0.0f ? -1.0f : 1.0f;
                result.contactArcRotationRadians[finger] = solved.distance * bakedAnchorNormalSign;
                result.contactArcRotationValid[finger] = 1;
            }
            if (allowSurfaceAimTargets && useTarget &&
                solved.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::FrontValid) {
                if (result.surfaceAimTargetValid[finger] == 0) {
                    if (solved.hasHitPoint) {
                        result.surfaceAimTarget[finger] =
                            RE::NiPoint3{ solved.hitPointX, solved.hitPointY, solved.hitPointZ };
                    } else {
                        result.surfaceAimTarget[finger] = fallbackHitPointValid ? fallbackHitPoint : fingerTargetWorld;
                    }
                    result.surfaceAimTargetValid[finger] = 1;
                }
                if (result.surfaceAimNormalValid[finger] == 0 && solved.hasHitNormal) {
                    result.surfaceAimNormal[finger] = normalizedOrFallback(
                        RE::NiPoint3{ solved.hitNormalX, solved.hitNormalY, solved.hitNormalZ }, curlNormalWorld);
                    result.surfaceAimNormalValid[finger] = 1;
                } else if (result.surfaceAimNormalValid[finger] == 0 && useTargetNormal &&
                    distanceSquared(fingerTargetNormalWorld, RE::NiPoint3{}) > 0.000001f) {
                    result.surfaceAimNormal[finger] = normalizedOrFallback(fingerTargetNormalWorld, curlNormalWorld);
                    result.surfaceAimNormalValid[finger] = 1;
                }
            }
            if (solved.hit) {
                result.hitCount++;
                if (solved.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::FrontValid &&
                    !solved.outOfReach) {
                    result.contactValidMask |= static_cast<std::uint8_t>(1u << finger);
                }
            }
        }

        result.solved = result.candidateTriangleCount > 0 && result.usedLiveRootFlattenedFingerBones;
        result.jointValues = grab_finger_pose_math::expandFingerCurlsToJointValues(result.values);
        result.hasJointValues = result.solved;
        result.spatialNodeVisitCount = spatialQueryStats.nodeVisits;
        result.spatialTriangleTestCount = spatialQueryStats.triangleTests;
        if (outSweepDebugCapture && useSpatialIndex) {
            outSweepDebugCapture->spatialNodeVisits = result.spatialNodeVisitCount;
            outSweepDebugCapture->spatialTriangleTests = result.spatialTriangleTestCount;
            outSweepDebugCapture->candidateTriangleCount =
                static_cast<std::uint32_t>((std::max)(result.candidateTriangleCount, 0));
            outSweepDebugCapture->isLeft = isLeft;
            outSweepDebugCapture->inPowerArmor = inPowerArmor;
            outSweepDebugCapture->valid = result.solved &&
                std::any_of(outSweepDebugCapture->fingers.begin(), outSweepDebugCapture->fingers.end(),
                    [](const FingerSweepDebugFingerCapture& finger) { return finger.valid; });
        }
        return result;
    }

    inline SolvedGrabFingerPose solveGrabFingerPoseFromTriangles(const std::vector<TriangleData>& triangles,
        const RE::NiTransform& handTransform, bool isLeft, const RE::NiPoint3& grabAnchorWorld,
        const RE::NiPoint3& grabGripPoint, float minValue, float maxTriangleDistanceSquared = 100.0f,
        bool useCurveSolver = true,
        const root_flattened_finger_skeleton_runtime::Snapshot* liveFingerSnapshot = nullptr,
        bool rejectBacksideHits = true, float surfacePlaneToleranceGameUnits = 1.5f, bool allowSurfaceAimTargets = true,
        float sweepContactRadiusGameUnits = 1.0f, float unreachableFingerOpenValue = -1.0f,
        float thumbSweepMaxOpenValue = grab_finger_pose_math::kMaxOverOpenValue,
        float fingerSweepMaxOpenValue = grab_finger_pose_math::kMaxOverOpenValue,
        const std::array<RE::NiPoint3, 5>* commandedOpenDirectionsWorld = nullptr,
        const FingerPoseTriangleSpatialIndex* spatialIndex = nullptr,
        const RE::NiTransform* spatialObjectWorldTransform = nullptr,
        FingerSweepDebugCapture* outSweepDebugCapture = nullptr,
        FingerPoseMeshRelation meshRelation = FingerPoseMeshRelation::CurrentMeshRequiresVirtualSeat)
    {
        return solveGrabFingerPoseFromTriangles(triangles, handTransform, isLeft, grabAnchorWorld,
            makeSharedGripPoseTarget(grabGripPoint), minValue, maxTriangleDistanceSquared, useCurveSolver,
            liveFingerSnapshot, rejectBacksideHits, surfacePlaneToleranceGameUnits, allowSurfaceAimTargets,
            sweepContactRadiusGameUnits, unreachableFingerOpenValue, thumbSweepMaxOpenValue, fingerSweepMaxOpenValue,
            commandedOpenDirectionsWorld, spatialIndex, spatialObjectWorldTransform, outSweepDebugCapture, meshRelation);
    }
}
