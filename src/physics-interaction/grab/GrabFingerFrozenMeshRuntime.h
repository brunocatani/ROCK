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
#include "physics-interaction/grab/GrabFingerLocalTransformRuntime.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

/*
 * The indexed frozen-mesh base owns the common authored-open anchor, bounded
 * local BVH, and calibrated sweep solve. Regular loose grabs layer their
 * thumb/index and pad-probe presentation policy on top. Equipped weapons use
 * the base directly because their multi-part surfaces and weapon-specific
 * opposition contract are not equivalent to a freely held object.
 */
namespace rock::grab_finger_pose_runtime
{
    struct FrozenMeshFingerPoseSolveOptions
    {
        float minValue = 0.2f;
        float maxTriangleDistanceSquared = 100.0f;
        bool rejectBacksideHits = true;
        float surfacePlaneToleranceGameUnits = 1.5f;
        bool allowSurfaceAimTargets = true;
        float sweepContactRadiusGameUnits = 1.0f;
        float thumbSweepMaxOpenValue = grab_finger_pose_math::kMaxOverOpenValue;
        float fingerSweepMaxOpenValue = grab_finger_pose_math::kMaxOverOpenValue;
        bool meshFingerPoseEnabled = true;
        bool captureSweepDebug = false;
        FingerPoseMeshRelation meshRelation = FingerPoseMeshRelation::AlreadyAtCommandedSeat;
    };

    struct FrozenMeshFingerPoseSolveResult
    {
        SolvedGrabFingerPose pose{};
        FingerSweepDebugCapture sweepDebug{};
        std::array<FingerPadSurfaceEvidence, 5> padEvidence{};
        root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshot{};
        std::array<RE::NiPoint3, 5> commandedOpenDirectionsWorld{};
        bool liveFingerSnapshotValid = false;
        bool commandedOpenDirectionsValid = false;
        bool spatialIndexBuilt = false;
    };

    inline bool resolveCommandedOpenDirectionsWorld(bool isLeft, const RE::NiTransform& handWorldTransform,
        std::array<RE::NiPoint3, 5>& outDirectionsWorld)
    {
        outDirectionsWorld = {};
        if (!std::isfinite(handWorldTransform.scale) || std::abs(handWorldTransform.scale) <= 0.000001f) {
            return false;
        }

        frik_visual_authority::FingerLocalTransformOverride openPoseLocals{};
        const auto openHandPose = frik_visual_authority::makeUniformHandPoseData(1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
        if (!frik_visual_authority::getHandPoseLocalTransformsForPose(frik_visual_authority::handFromBool(isLeft),
                openHandPose, &openPoseLocals) ||
            !grab_finger_local_transform_runtime::fingerLocalTransformOverrideIsSafeForPublication(openPoseLocals)) {
            return false;
        }

        const auto openDirectionsHandLocal = computeCommandedOpenDirectionsHandLocal(openPoseLocals.localTransforms);
        for (std::size_t finger = 0; finger < outDirectionsWorld.size(); ++finger) {
            const RE::NiPoint3 directionWorld =
                transform_math::localVectorToWorld(handWorldTransform, openDirectionsHandLocal[finger]);
            const float directionLengthSquared = distanceSquared(directionWorld, RE::NiPoint3{});
            if (!std::isfinite(directionLengthSquared) || directionLengthSquared <= 0.000001f) {
                outDirectionsWorld = {};
                return false;
            }
            outDirectionsWorld[finger] = directionWorld * (1.0f / std::sqrt(directionLengthSquared));
        }
        return true;
    }

    inline bool resolveSurfaceContactSplayValues(bool isLeft, const SolvedGrabFingerPose& pose,
        std::array<float, 5>& outSplayRadians, float maxSplayRadians)
    {
        outSplayRadians = {};
        if (!hasSurfaceContactSplayCandidates(pose)) {
            return false;
        }

        root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshot{};
        if (!root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, liveFingerSnapshot)) {
            return false;
        }

        RE::NiTransform handWorld{};
        std::array<RE::NiPoint3, 5> openDirections{};
        if (!frik_hand_world_authority::tryGetRawHandWorld(isLeft, handWorld) ||
            !resolveCommandedOpenDirectionsWorld(isLeft, handWorld, openDirections)) {
            return false;
        }
        // Splay is an absolute pose value. Measuring from the already splayed
        // displayed chord feeds the previous pose back into the next command.
        return buildSurfaceContactSplayValues(pose, liveFingerSnapshot, openDirections, outSplayRadians,
            maxSplayRadians);
    }

    template <class TriangleContainer>
    inline void rebuildBoundedWorldTriangles(const TriangleContainer& localTriangles,
        const RE::NiTransform& meshWorldTransform, std::vector<TriangleData>& outWorldTriangles)
    {
        outWorldTriangles.clear();
        outWorldTriangles.reserve((std::min)(localTriangles.size(), kMaxFingerPoseCandidateTriangles));
        for (const auto& localTriangle : localTriangles) {
            if (!isFinitePoint(localTriangle.v0) || !isFinitePoint(localTriangle.v1) ||
                !isFinitePoint(localTriangle.v2)) {
                continue;
            }
            outWorldTriangles.push_back(TriangleData{
                transform_math::localPointToWorld(meshWorldTransform, localTriangle.v0),
                transform_math::localPointToWorld(meshWorldTransform, localTriangle.v1),
                transform_math::localPointToWorld(meshWorldTransform, localTriangle.v2),
            });
            if (outWorldTriangles.size() >= kMaxFingerPoseCandidateTriangles) {
                break;
            }
        }
    }

    template <class TriangleContainer>
    inline FrozenMeshFingerPoseSolveResult solveFrozenMeshFingerPoseBase(const TriangleContainer& boundedLocalTriangles,
        const RE::NiTransform& frozenMeshWorldTransform, const RE::NiTransform& handWorldTransform, bool isLeft,
        const RE::NiPoint3& grabAnchorWorld, const GrabFingerPoseTargetSet& poseTargets,
        FingerPoseTriangleSpatialIndex& spatialIndex, std::vector<TriangleData>& worldTriangleScratch,
        const FrozenMeshFingerPoseSolveOptions& options,
        const root_flattened_finger_skeleton_runtime::Snapshot* capturedFingerSnapshot = nullptr,
        const std::array<RE::NiPoint3, 5>* capturedOpenDirectionsWorld = nullptr)
    {
        FrozenMeshFingerPoseSolveResult result{};
        rebuildBoundedWorldTriangles(boundedLocalTriangles, frozenMeshWorldTransform, worldTriangleScratch);
        result.spatialIndexBuilt = spatialIndex.buildFromLocalTriangles(boundedLocalTriangles);

        if (capturedFingerSnapshot) {
            if (capturedFingerSnapshot->valid) {
                result.liveFingerSnapshot = *capturedFingerSnapshot;
                result.liveFingerSnapshotValid = true;
            }
        } else {
            result.liveFingerSnapshotValid = root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(
                isLeft, result.liveFingerSnapshot);
        }
        const auto* liveFingerSnapshotPtr = result.liveFingerSnapshotValid ? &result.liveFingerSnapshot : nullptr;

        // Captured directions (offline replay) stand in for the FRIK query, as the captured snapshot does for the scene.
        if (capturedOpenDirectionsWorld) {
            result.commandedOpenDirectionsWorld = *capturedOpenDirectionsWorld;
            result.commandedOpenDirectionsValid = true;
        } else {
            result.commandedOpenDirectionsValid =
                resolveCommandedOpenDirectionsWorld(isLeft, handWorldTransform, result.commandedOpenDirectionsWorld);
        }
        result.pose = solveGrabFingerPoseFromTriangles(worldTriangleScratch, handWorldTransform, isLeft,
            grabAnchorWorld, poseTargets, options.minValue, options.maxTriangleDistanceSquared, true,
            liveFingerSnapshotPtr, options.rejectBacksideHits, options.surfacePlaneToleranceGameUnits,
            options.allowSurfaceAimTargets, options.sweepContactRadiusGameUnits, -1.0f, options.thumbSweepMaxOpenValue,
            options.fingerSweepMaxOpenValue,
            result.commandedOpenDirectionsValid ? &result.commandedOpenDirectionsWorld : nullptr,
            result.spatialIndexBuilt ? &spatialIndex : nullptr,
            result.spatialIndexBuilt ? &frozenMeshWorldTransform : nullptr,
            options.captureSweepDebug ? &result.sweepDebug : nullptr, options.meshRelation);

        return result;
    }

    template <class TriangleContainer>
    inline FrozenMeshFingerPoseSolveResult solveFrozenMeshFingerPose(const TriangleContainer& boundedLocalTriangles,
        const RE::NiTransform& frozenMeshWorldTransform, const RE::NiTransform& handWorldTransform, bool isLeft,
        const RE::NiPoint3& grabAnchorWorld, const GrabFingerPoseTargetSet& poseTargets,
        FingerPoseTriangleSpatialIndex& spatialIndex, std::vector<TriangleData>& worldTriangleScratch,
        const FrozenMeshFingerPoseSolveOptions& options,
        const root_flattened_finger_skeleton_runtime::Snapshot* capturedFingerSnapshot = nullptr)
    {
        auto result = solveFrozenMeshFingerPoseBase(boundedLocalTriangles, frozenMeshWorldTransform, handWorldTransform,
            isLeft, grabAnchorWorld, poseTargets, spatialIndex, worldTriangleScratch, options, capturedFingerSnapshot);
        useThumbIndexCurveOnlyPose(result.pose);
        if (result.liveFingerSnapshotValid) {
            (void)refineGrabFingerPoseWithPadProbes(result.pose, worldTriangleScratch, poseTargets,
                result.liveFingerSnapshot, frozenMeshWorldTransform, options.meshFingerPoseEnabled, true,
                result.padEvidence, true, {}, result.spatialIndexBuilt ? &spatialIndex : nullptr);
        }
        captureSurfaceAimObjectLocal(result.pose, frozenMeshWorldTransform);
        return result;
    }
}
