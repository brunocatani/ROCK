#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// Per-finger surface solve for support grip capture, called from
// capturePartGrip (SupportGrip.cpp) after the hand is seated on the grip.

namespace rock
{
    /*
     * July 9 calibrated slice against the contacted part only. The mesh stays
     * where it is and the finger bases are shifted by the pending seat (palm
     * pivot -> grip point), so the solve sees the hand as already seated.
     */
    void TwoHandedGrip::solveSupportGripFingerPose(
        const bool isLeft,
        RE::NiNode* weaponNode,
        const RE::NiTransform& handTransform,
        const RE::NiPoint3& gripWorldPoint,
        const bool cachedTrianglesFound,
        const WeaponCollision::SupportGripEvidenceView& evidenceView,
        WeaponPartGrip& grip)
    {
        auto& fingerScratch = _fingerPoseSolveScratch->hands[isLeft ? 0u : 1u];
        for (auto& ranking : fingerScratch.rankings) {
            ranking.clear();
        }
        fingerScratch.localTriangles.clear();
        fingerScratch.worldTriangles.clear();

        /*
         * Capture the root-flattened fingers once for this transaction. The
         * compact sweep snapshot and any exact-local thumb/surface correction
         * must describe the same pre-authority hand, not two scene reads split
         * by grip publication.
         */
        auto& capturedFingerBoneSnapshot = fingerScratch.capturedBones;
        const bool capturedFingerBoneSnapshotValid =
            rootFlattenedTwoHandedReader().capture(
                skeleton_bone_debug_math::DebugSkeletonBoneMode::
                    HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::
                    GameRootFlattenedBoneTree,
                SkeletonBoneCaptureSpace::Controller,
                capturedFingerBoneSnapshot);
        root_flattened_finger_skeleton_runtime::Snapshot
            capturedFingerSnapshot{};
        const bool capturedFingerSnapshotValid =
            capturedFingerBoneSnapshotValid &&
            root_flattened_finger_skeleton_runtime::
                buildFingerSkeletonSnapshot(
                    capturedFingerBoneSnapshot,
                    isLeft,
                    capturedFingerSnapshot, nullptr, &fingerScratch.capturedBoneNames);
        SupportGripFingerReferenceSet fingerReferenceSet{};
        fingerReferenceSet.seatPointWorld = gripWorldPoint;
        fingerReferenceSet.seatPointValid =
            grab_finger_pose_runtime::isFinitePoint(gripWorldPoint);

        const std::size_t sourceTriangleCount =
            cachedTrianglesFound ? evidenceView.localTriangles.size() : 0;
        if (g_rockConfig.rockGrabMeshFingerPoseEnabled && cachedTrianglesFound) {
            selectNearestSupportGripFingerTriangles(
                std::span<const WeaponCollision::SupportGripEvidenceView>(&evidenceView, 1),
                weaponNode->world,
                fingerReferenceSet,
                grab_finger_pose_runtime::
                    kMaxFingerPoseCandidateTriangles,
                fingerScratch.rankings,
                fingerScratch.localTriangles);
        }
        performance_profiler::observeValue(
            performance_profiler::ValueMetric::
                EquippedWeaponFingerPoseSourceTriangles,
            static_cast<std::uint64_t>(sourceTriangleCount));
        performance_profiler::observeValue(
            performance_profiler::ValueMetric::
                EquippedWeaponFingerPoseSelectedTriangles,
            static_cast<std::uint64_t>(
                fingerScratch.localTriangles.size()));

        grab_finger_pose_runtime::SolvedGrabFingerPose meshFingerPose{};
        const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPosePtr = nullptr;
        std::array<float, 5> capturedFingerSplayRadians{};
        const std::array<float, 5>* capturedFingerSplayRadiansPtr = nullptr;
        if (g_rockConfig.rockGrabMeshFingerPoseEnabled && !fingerScratch.localTriangles.empty()) {
            auto fingerPoseTargets = grab_finger_pose_runtime::makeSharedGripPoseTarget(gripWorldPoint, grip.grabNormalWorld);
            fingerPoseTargets.useSeatPointForMissingTargets = false;
            fingerPoseTargets.useWholeMeshForMissingTargets = true;
            const auto frozenSolve =
                grab_finger_pose_runtime::solveFrozenMeshFingerPose(
                    fingerScratch.localTriangles,
                    weaponNode->world,
                    handTransform,
                    isLeft,
                    computeGrabLegacyPalmPivotAWorldFromHandBasis(handTransform, isLeft),
                    fingerPoseTargets,
                    fingerScratch.worldTriangles,
                    grab_finger_pose_runtime::FrozenMeshFingerPoseSolveOptions{
                        .minValue = g_rockConfig.rockGrabFingerMinValue,
                        .maxTriangleDistanceSquared = g_rockConfig.rockGrabMaxTriangleDistance,
                        .rejectBacksideHits = g_rockConfig.rockGrabFingerRejectBacksideHits,
                        .surfacePlaneToleranceGameUnits = g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits,
                        .allowSurfaceAimTargets = true,
                        .meshRelation = grab_finger_pose_runtime::FingerPoseMeshRelation::CurrentMeshRequiresVirtualSeat,
                    },
                    capturedFingerSnapshotValid ?
                        &capturedFingerSnapshot :
                        nullptr);
            meshFingerPose = frozenSolve.pose;
            grab_finger_pose_runtime::captureSurfaceAimObjectLocal(
                meshFingerPose,
                weaponNode->world);
            if (meshFingerPose.solved) {
                meshFingerPosePtr = &meshFingerPose;
                if (frozenSolve.liveFingerSnapshotValid) {
                    const auto landmarks = root_flattened_finger_skeleton_runtime::buildLandmarkSet(
                        frozenSolve.liveFingerSnapshot);
                    std::array<RE::NiPoint3, 5> capturedDirections{};
                    for (std::size_t finger = 0; finger < capturedDirections.size(); ++finger) {
                        capturedDirections[finger] = landmarks.fingers[finger].openDirection;
                    }
                    if (grab_finger_pose_runtime::buildSurfaceContactSplayValues(
                            meshFingerPose, frozenSolve.liveFingerSnapshot, capturedDirections, capturedFingerSplayRadians)) {
                        capturedFingerSplayRadiansPtr = &capturedFingerSplayRadians;
                    }
                }
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: mesh finger pose hand={} values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} contactMask=0x{:02X} sourceTris={} candidateTris={} altThumb={} thumbLane={}",
                    isLeft ? "left" : "right",
                    meshFingerPose.values[0],
                    meshFingerPose.values[1],
                    meshFingerPose.values[2],
                    meshFingerPose.values[3],
                    meshFingerPose.values[4],
                    meshFingerPose.hitCount,
                    static_cast<unsigned>(
                        meshFingerPose.contactValidMask),
                    sourceTriangleCount,
                    meshFingerPose.candidateTriangleCount,
                    meshFingerPose.usedAlternateThumbCurve ? "yes" : "no",
                    grab_finger_pose_math::thumbLaneName(meshFingerPose.selectedThumbLane));
                if (meshFingerPose.hasThumbCurveDiagnostics) {
                    ROCK_LOG_DEBUG(Weapon,
                        "TwoHandedGrip: thumb curve primary(hit={} value={:.2f} behind={}) opposition(hit={} value={:.2f} behind={}) sidePad(hit={} value={:.2f} behind={}) selected={}",
                        meshFingerPose.thumbPrimaryCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbPrimaryCurve.value,
                        meshFingerPose.thumbPrimaryCurve.openedByBehindContact ? "yes" : "no",
                        meshFingerPose.thumbAlternateCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbAlternateCurve.value,
                        meshFingerPose.thumbAlternateCurve.openedByBehindContact ? "yes" : "no",
                        meshFingerPose.thumbSidePadCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbSidePadCurve.value,
                        meshFingerPose.thumbSidePadCurve.openedByBehindContact ? "yes" : "no",
                        grab_finger_pose_math::thumbLaneName(meshFingerPose.selectedThumbLane));
                }
            }
        }

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: weapon grab pose hand={} pose={} hits={} candidates={}",
            isLeft ? "left" : "right",
            meshFingerPosePtr ? "mesh" : "closed-fallback", meshFingerPose.hitCount, meshFingerPose.candidateTriangleCount);

        setSupportGripPose(
            isLeft,
            meshFingerPosePtr,
            capturedFingerSplayRadiansPtr,
            SupportGripPoseFallback::FullyClosed);
        if (meshFingerPosePtr && grip.hasFingerPose) {
            std::array<RE::NiTransform, 15> localTransforms{};
            std::uint16_t localTransformMask = 0;
            const auto handPose = grip.hasFingerSplay ?
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose, grip.fingerSplayRadians) :
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose);
            if (buildFullHandLocalTransformsForMeshPose(
                    isLeft,
                    *meshFingerPosePtr,
                    handPose,
                    capturedFingerBoneSnapshotValid ?
                        &capturedFingerBoneSnapshot :
                        nullptr,
                    localTransforms,
                    localTransformMask)) {
                grip.fingerLocalTransforms = localTransforms;
                grip.fingerLocalTransformMask = localTransformMask;
                grip.hasFingerLocalTransforms = true;
                double maxBasisError = 0.0;
                for (const auto& local : localTransforms) {
                    maxBasisError = (std::max)(maxBasisError,
                        transform_math::storedRotationOrthonormalityError(local.rotate));
                }
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: dynamic finger pose prepared hand={} mask=0x{:04X} lane={} curls=({:.3f},{:.3f},{:.3f},{:.3f},{:.3f}) maxBasisError={:.8f}",
                    isLeft ? "left" : "right",
                    grip.fingerLocalTransformMask,
                    grab_finger_pose_math::thumbLaneName(meshFingerPosePtr->selectedThumbLane),
                    meshFingerPosePtr->values[0], meshFingerPosePtr->values[1], meshFingerPosePtr->values[2],
                    meshFingerPosePtr->values[3], meshFingerPosePtr->values[4], maxBasisError);
            }
        }

    }
}
