#include "physics-interaction/weapon/TwoHandedGripInternal.h"

#include "physics-interaction/grab/GrabFingerCapsuleSweep.h"

// Per-finger surface solve for support grip capture, called from
// capturePartGrip (SupportGrip.cpp) after the hand is seated on the grip.

namespace rock
{
    /*
     * The support hand's fingers come from the same capsule sweep as loose
     * grabs (GrabFingerCapsuleSweep.h): FRIK's own finger motion, swept against
     * the weapon mesh with the hand collider radii, from the seated hand.
     * The live skeleton is captured once relative to the controller hand and
     * rebased onto the seated hand, so the sweep sees the final grip relation.
     * Triangle selection stays bounded: only weapon triangles nearest each
     * finger's sweep are indexed.
     */
    void TwoHandedGrip::solveSupportGripFingerPose(
        const bool isLeft,
        RE::NiNode* weaponNode,
        const WeaponInteractionDecision& decision,
        const WeaponCollision& weaponCollision,
        const RE::NiTransform& handTransform,
        const RE::NiTransform& adjustedHandTransform,
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
        fingerScratch.spatialIndex.clear();

        auto& capturedFingerBoneSnapshot = fingerScratch.capturedBones;
        root_flattened_finger_skeleton_runtime::Snapshot seatedSkeleton{};
        const bool seatedSkeletonValid =
            rootFlattenedTwoHandedReader().capture(
                skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                SkeletonBoneCaptureSpace::Controller,
                capturedFingerBoneSnapshot) &&
            root_flattened_finger_skeleton_runtime::buildFingerSkeletonSnapshot(
                capturedFingerBoneSnapshot, isLeft, seatedSkeleton, nullptr, &fingerScratch.capturedBoneNames) &&
            grab_finger_capsule_sweep::rebaseSkeleton(seatedSkeleton, handTransform, adjustedHandTransform);

        grab_finger_capsule_sweep::FrikFingerRig rig{};
        const bool rigValid = seatedSkeletonValid && grab_finger_capsule_sweep::resolveFrikFingerRig(isLeft, rig);
        const grab_finger_capsule_sweep::Options options{
            .minValue = g_rockConfig.rockGrabFingerMinValue,
            .fingerMaxOpenValue = g_rockConfig.rockGrabFingerSweepMaxOpenValue,
            .thumbMaxOpenValue = g_rockConfig.rockGrabThumbSweepMaxOpenValue,
        };

        std::size_t compositeEvidenceViewCount = 0;
        std::size_t sourceTriangleCount = 0;
        if (g_rockConfig.rockGrabMeshFingerPoseEnabled && rigValid) {
            const RE::NiTransform handBoneWorld = grab_finger_capsule_sweep::resolveHandBoneWorld(adjustedHandTransform, seatedSkeleton, rig);
            const auto tipLengths = grab_finger_capsule_sweep::tipLengths(seatedSkeleton);
            SupportGripFingerReferenceSet fingerReferenceSet{};
            fingerReferenceSet.seatPointWorld = gripWorldPoint;
            fingerReferenceSet.seatPointValid = grab_finger_pose_runtime::isFinitePoint(gripWorldPoint);
            for (std::size_t finger = 0; finger < kSupportGripFingerLaneCount; ++finger) {
                fingerReferenceSet.lanePointCounts[finger] = grab_finger_capsule_sweep::sweepReferencePoints(
                    rig, handBoneWorld, finger, tipLengths[finger], options.minValue, fingerReferenceSet.lanePointsWorld[finger]);
            }

            std::array<WeaponCollision::SupportGripEvidenceView, MAX_WEAPON_COLLISION_BODIES> compositeEvidenceViews{};
            const std::size_t discoveredViewCount = weaponCollision.findSupportGripEvidenceViews(weaponNode, compositeEvidenceViews);
            for (std::size_t index = 0; index < discoveredViewCount; ++index) {
                const auto& candidateView = compositeEvidenceViews[index];
                if (candidateView.weaponGenerationKey != decision.weaponGenerationKey ||
                    candidateView.weaponGenerationKey != _session.weaponGenerationKey || candidateView.localTriangles.empty()) {
                    continue;
                }
                if (compositeEvidenceViewCount != index) {
                    compositeEvidenceViews[compositeEvidenceViewCount] = candidateView;
                }
                sourceTriangleCount += candidateView.localTriangles.size();
                ++compositeEvidenceViewCount;
            }
            if (compositeEvidenceViewCount == 0 && cachedTrianglesFound) {
                compositeEvidenceViews[0] = evidenceView;
                compositeEvidenceViewCount = 1;
                sourceTriangleCount = evidenceView.localTriangles.size();
            }
            selectNearestSupportGripFingerTriangles(
                std::span<const WeaponCollision::SupportGripEvidenceView>(compositeEvidenceViews.data(), compositeEvidenceViewCount),
                weaponNode->world,
                fingerReferenceSet,
                grab_finger_pose_runtime::kMaxFingerPoseCandidateTriangles,
                fingerScratch.rankings,
                fingerScratch.localTriangles);
        }
        performance_profiler::observeValue(
            performance_profiler::ValueMetric::EquippedWeaponFingerPoseSourceTriangles, static_cast<std::uint64_t>(sourceTriangleCount));
        performance_profiler::observeValue(
            performance_profiler::ValueMetric::EquippedWeaponFingerPoseSelectedTriangles, static_cast<std::uint64_t>(fingerScratch.localTriangles.size()));

        grab_finger_pose_runtime::SolvedGrabFingerPose meshFingerPose{};
        const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPosePtr = nullptr;
        grab_finger_capsule_sweep::Result capsule{};
        if (rigValid && !fingerScratch.localTriangles.empty() && fingerScratch.spatialIndex.buildFromLocalTriangles(fingerScratch.localTriangles)) {
            capsule = grab_finger_capsule_sweep::solveForHand(rig, adjustedHandTransform, seatedSkeleton, fingerScratch.spatialIndex, weaponNode->world, options);
            meshFingerPose = grab_finger_capsule_sweep::toSolvedPose(capsule, static_cast<std::uint32_t>(fingerScratch.localTriangles.size()));
            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseSpatialNodeVisits, meshFingerPose.spatialNodeVisitCount);
            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseTriangleTests, meshFingerPose.spatialTriangleTestCount);
            // A sweep that touched nothing never reached the weapon: keep the closed fallback grip.
            if (meshFingerPose.contactValidMask != 0) {
                meshFingerPosePtr = &meshFingerPose;
            }
        }

        std::uint32_t openedMask = 0;
        std::uint32_t blockedMask = 0;
        for (std::size_t finger = 0; finger < 5; ++finger) {
            openedMask |= capsule.fingers[finger].openedToClear ? (1u << finger) : 0u;
            blockedMask |= capsule.fingers[finger].blocked ? (1u << finger) : 0u;
        }
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: support finger pose hand={} pose={} curls=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) contacts=0x{:02X} openedToClear=0x{:02X} "
            "blocked=0x{:02X} skeleton={} frikRig={} sources={} sourceTris={} candidateTris={} sphereQueries={} tests={}",
            isLeft ? "left" : "right", meshFingerPosePtr ? "mesh" : "closed-fallback", meshFingerPose.values[0], meshFingerPose.values[1],
            meshFingerPose.values[2], meshFingerPose.values[3], meshFingerPose.values[4], meshFingerPose.contactValidMask, openedMask, blockedMask,
            seatedSkeletonValid, rigValid, compositeEvidenceViewCount, sourceTriangleCount, fingerScratch.localTriangles.size(), capsule.sphereQueries,
            capsule.stats.triangleTests);

        setSupportGripPose(isLeft, meshFingerPosePtr, SupportGripPoseFallback::FullyClosed);
    }
}
