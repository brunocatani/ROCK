// Per-finger surface solve for support grip capture, called from
// capturePartGrip (SupportGrip.cpp) after the hand is seated on the grip.
#include "physics-interaction/weapon/grip/TwoHandedGripInternal.h"

#include "physics-interaction/grab/GrabFingerCapsuleSweep.h"

#include <chrono>

namespace rock
{
    void TwoHandedGrip::solveSupportGripFingerPose(const bool isLeft, RE::NiNode* weaponNode,
        const WeaponInteractionDecision& decision, const WeaponCollision& weaponCollision,
        const RE::NiTransform& handTransform, const RE::NiTransform& adjustedHandTransform,
        const RE::NiPoint3& gripWorldPoint, const bool cachedTrianglesFound,
        const WeaponCollision::SupportGripEvidenceView& evidenceView, WeaponPartGrip& grip)
    {
        const bool capsuleProbes = g_rockConfig.rockWeaponGrabCapsuleProbes;
        auto& fingerScratch = _fingerPoseSolveScratch->hands[isLeft ? 0u : 1u];
        for (auto& ranking : fingerScratch.rankings) {
            ranking.clear();
        }
        fingerScratch.localTriangles.clear();
        fingerScratch.worldTriangles.clear();
        fingerScratch.spatialIndex.clear();

        /*
         * Capture the root-flattened fingers once for this transaction. The
         * compact sweep snapshot and any exact-local thumb/surface correction
         * must describe the same pre-authority hand, not two scene reads split
         * by grip publication.
         */
        auto& capturedFingerBoneSnapshot = fingerScratch.capturedBones;
        const bool capturedFingerBoneSnapshotValid = rootFlattenedTwoHandedReader().capture(
            skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
            skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
            SkeletonBoneCaptureSpace::Controller, capturedFingerBoneSnapshot);
        root_flattened_finger_skeleton_runtime::Snapshot capturedFingerSnapshot{};
        const bool capturedFingerSnapshotValid = capturedFingerBoneSnapshotValid &&
            root_flattened_finger_skeleton_runtime::buildFingerSkeletonSnapshot(capturedFingerBoneSnapshot, isLeft,
                capturedFingerSnapshot, nullptr, &fingerScratch.capturedBoneNames);
        SupportGripFingerReferenceSet fingerReferenceSet{};
        fingerReferenceSet.seatPointWorld = gripWorldPoint;
        fingerReferenceSet.seatPointValid = grab_finger_pose_runtime::isFinitePoint(gripWorldPoint);
        if (capturedFingerSnapshotValid) {
            const RE::NiTransform rawToSeatedWorld = transform_math::composeTransforms(adjustedHandTransform,
                transform_math::invertTransform(handTransform));
            const auto liveLandmarks = root_flattened_finger_skeleton_runtime::buildLandmarkSet(capturedFingerSnapshot);
            std::array<RE::NiPoint3, kSupportGripFingerLaneCount> commandedOpenDirectionsWorld{};
            const bool commandedDirectionsValid = grab_finger_pose_runtime::resolveCommandedOpenDirectionsWorld(isLeft,
                adjustedHandTransform, commandedOpenDirectionsWorld);
            const RE::NiPoint3 seatedSweepNormal = liveLandmarks.valid
                ? transform_math::localVectorToWorld(rawToSeatedWorld, liveLandmarks.palmNormalWorld)
                : RE::NiPoint3{};
            const auto appendLanePoint = [&fingerReferenceSet](const std::size_t lane, const RE::NiPoint3& pointWorld) {
                if (lane >= kSupportGripFingerLaneCount || !grab_finger_pose_runtime::isFinitePoint(pointWorld)) {
                    return;
                }
                auto& count = fingerReferenceSet.lanePointCounts[lane];
                if (count >= kSupportGripFingerLaneReferenceCapacity) {
                    return;
                }
                fingerReferenceSet.lanePointsWorld[lane][count++] = pointWorld;
            };

            for (std::size_t finger = 0; finger < capturedFingerSnapshot.fingers.size(); ++finger) {
                const auto& chain = capturedFingerSnapshot.fingers[finger];
                if (!chain.valid) {
                    continue;
                }
                for (const auto& pointWorld : chain.points) {
                    appendLanePoint(finger, transform_math::localPointToWorld(rawToSeatedWorld, pointWorld));
                }

                if (!liveLandmarks.valid || !commandedDirectionsValid || finger >= liveLandmarks.fingers.size() ||
                    !liveLandmarks.fingers[finger].valid || !std::isfinite(liveLandmarks.fingers[finger].length) ||
                    liveLandmarks.fingers[finger].length <= 0.0001f) {
                    continue;
                }
                const RE::NiPoint3 seatedBase =
                    transform_math::localPointToWorld(rawToSeatedWorld, liveLandmarks.fingers[finger].base);
                const auto sweepCurve = grab_finger_pose_math::makeBakedCalibratedFingerCurve<RE::NiPoint3>(finger,
                    isLeft, capturedFingerSnapshot.inPowerArmor, seatedBase, seatedSweepNormal,
                    commandedOpenDirectionsWorld[finger], liveLandmarks.fingers[finger].length);
                const auto* tipProbe = sweepCurve.probeCount > 0 ? &sweepCurve.probes[0] : nullptr;
                if (!tipProbe || tipProbe->sampleCount == 0 || tipProbe->sampleCount > tipProbe->samples.size()) {
                    continue;
                }
                constexpr std::size_t kSweepSamples = 7;
                const RE::NiPoint3 curveNormal =
                    grab_finger_pose_runtime::normalizedOrFallback(sweepCurve.normal, seatedSweepNormal);
                const RE::NiPoint3 curveZero = grab_finger_pose_runtime::normalizedOrFallback(
                    sweepCurve.zeroAngleVector, commandedOpenDirectionsWorld[finger]);
                for (std::size_t sample = 0; sample < kSweepSamples; ++sample) {
                    const std::size_t row = sample * (tipProbe->sampleCount - 1) / (kSweepSamples - 1);
                    const auto& baked = tipProbe->samples[row];
                    const RE::NiPoint3 arm =
                        grab_finger_pose_math::rotateAroundUnitAxis(curveZero, curveNormal, baked.angleRadians);
                    appendLanePoint(finger,
                        grab_finger_pose_math::add(sweepCurve.center,
                            grab_finger_pose_math::scale(arm, baked.reachLength)));
                }
            }
        }

        std::array<WeaponCollision::SupportGripEvidenceView, MAX_WEAPON_COLLISION_BODIES> compositeEvidenceViews{};
        std::size_t compositeEvidenceViewCount = 0;
        std::size_t sourceTriangleCount = 0;
        {
            const std::size_t discoveredViewCount =
                weaponCollision.findSupportGripEvidenceViews(weaponNode, compositeEvidenceViews);
            for (std::size_t index = 0; index < discoveredViewCount; ++index) {
                const auto& candidateView = compositeEvidenceViews[index];
                if (candidateView.weaponGenerationKey != decision.weaponGenerationKey ||
                    candidateView.weaponGenerationKey != _session.weaponGenerationKey ||
                    candidateView.localTriangles.empty()) {
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
            selectNearestSupportGripFingerTriangles(std::span<const WeaponCollision::SupportGripEvidenceView>(
                                                        compositeEvidenceViews.data(), compositeEvidenceViewCount),
                weaponNode->world, fingerReferenceSet, grab_finger_pose_runtime::kMaxFingerPoseCandidateTriangles,
                fingerScratch.rankings, fingerScratch.localTriangles);
        }
        performance_profiler::observeValue(performance_profiler::ValueMetric::EquippedWeaponFingerPoseSourceTriangles,
            static_cast<std::uint64_t>(sourceTriangleCount));
        performance_profiler::observeValue(performance_profiler::ValueMetric::EquippedWeaponFingerPoseSelectedTriangles,
            static_cast<std::uint64_t>(fingerScratch.localTriangles.size()));

        if (capsuleProbes) {
            /*
             * Testing mode (FingerPoses.WeaponGrabCapsuleProbes): the fingers
             * are posed by the loose-object probes. Each finger's capsules
             * close until they meet the weapon mesh, then a thumb or index
             * that met nothing on its arc bends its joints separately to
             * bring its tip onto the mesh, so a bolt handle or slide smaller
             * than the fingers is held between them. Only the finger pose
             * differs: which part is grabbed and where the hand sits are
             * unchanged.
             */
            namespace sweep = grab_finger_capsule_sweep;
            const auto started = std::chrono::steady_clock::now();
            grab_finger_pose_runtime::SolvedGrabFingerPose capsulePose{};
            sweep::FrikFingerRig rig{};
            const bool rigValid = capturedFingerSnapshotValid && sweep::resolveFrikFingerRig(isLeft, rig);
            const bool indexBuilt = fingerScratch.spatialIndex.buildFromLocalTriangles(fingerScratch.localTriangles);
            sweep::PinchReach thumbReach{};
            sweep::PinchReach indexReach{};
            if (rigValid && indexBuilt) {
                // The mesh stands where it will be once the hand is seated.
                const RE::NiTransform meshWorld = weapon_two_handed_grip_math::virtualizeMeshForSeatedHand(
                    weaponNode->world, handTransform, adjustedHandTransform);
                const sweep::Options options{
                    .minValue = g_rockConfig.rockGrabFingerMinValue,
                };
                const auto capsule = sweep::solveForHand(rig, handTransform, capturedFingerSnapshot,
                    fingerScratch.spatialIndex, meshWorld, options);
                capsulePose =
                    sweep::toSolvedPose(capsule, static_cast<std::uint32_t>(fingerScratch.localTriangles.size()));
                const RE::NiTransform handBone = sweep::resolveHandBoneWorld(handTransform, capturedFingerSnapshot, rig);
                const auto tips = sweep::tipLengths(capturedFingerSnapshot);
                if ((capsulePose.contactValidMask & 0x01) == 0) {
                    thumbReach = sweep::reachPinchFinger(capsulePose, 0, rig, handBone, tips[0],
                        fingerScratch.spatialIndex, meshWorld, options, 0.0f);
                }
                if ((capsulePose.contactValidMask & 0x02) == 0) {
                    indexReach = sweep::reachPinchFinger(capsulePose, 1, rig, handBone, tips[1],
                        fingerScratch.spatialIndex, meshWorld, options, options.minValue);
                }
            }
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: weapon grab pose applied=Capsule probes (testing mode) hand={} part={} solved={} "
                "skeleton={} frikRig={} meshIndex={} contacts=0x{:02X} curls=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) "
                "thumbReach={:.2f}->{:.2f}gu({}) indexReach={:.2f}->{:.2f}gu({}) sourceTriangles={} triangles={} "
                "{:.2f}ms",
                isLeft ? "left" : "right", static_cast<int>(grip.partKind), capsulePose.solved ? "yes" : "no",
                capturedFingerSnapshotValid ? "yes" : "no", rigValid ? "yes" : "no", indexBuilt ? "yes" : "no",
                static_cast<unsigned>(capsulePose.contactValidMask), capsulePose.values[0], capsulePose.values[1],
                capsulePose.values[2], capsulePose.values[3], capsulePose.values[4], thumbReach.beforeGameUnits,
                thumbReach.afterGameUnits, thumbReach.applied ? "reached" : "kept", indexReach.beforeGameUnits,
                indexReach.afterGameUnits, indexReach.applied ? "reached" : "kept", sourceTriangleCount,
                fingerScratch.localTriangles.size(),
                std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - started).count());
            // Fail closed: an unsolved pose takes the closed fallback like the mesh solve.
            setSupportGripPose(isLeft, capsulePose.solved ? &capsulePose : nullptr, nullptr,
                SupportGripPoseFallback::FullyClosed);
            return;
        }

        grab_finger_pose_runtime::SolvedGrabFingerPose meshFingerPose{};
        const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPosePtr = nullptr;
        std::array<float, 5> capturedFingerSplayRadians{};
        const std::array<float, 5>* capturedFingerSplayRadiansPtr = nullptr;
        bool spatialIndexBuilt = false;
        bool commandedOpenDirectionsValid = false;
        if (!fingerScratch.localTriangles.empty()) {
            const RE::NiTransform seatedToRawWorld = transform_math::composeTransforms(handTransform,
                transform_math::invertTransform(adjustedHandTransform));
            const RE::NiTransform frozenMeshWorld = weapon_two_handed_grip_math::virtualizeMeshForSeatedHand(
                weaponNode->world, handTransform, adjustedHandTransform);
            const RE::NiPoint3 frozenGripPoint = weapon_two_handed_grip_math::virtualizeWorldPointForSeatedHand(
                gripWorldPoint, handTransform, adjustedHandTransform);
            const RE::NiPoint3 solveAnchorWorld = frozenGripPoint;
            const RE::NiPoint3 frozenGripNormal =
                transform_math::localVectorToWorld(seatedToRawWorld, grip.grabNormalWorld);
            auto fingerPoseTargets =
                grab_finger_pose_runtime::makeSharedGripPoseTarget(frozenGripPoint, frozenGripNormal);
            fingerPoseTargets.useSeatPointForMissingTargets = false;
            fingerPoseTargets.useWholeMeshForMissingTargets = true;
            /*
             * Equipped support keeps the indexed frozen base, but owns its
             * presentation policy. Loose-grab thumb/index clearing and generic
             * pad-probe refinement erased useful weapon-surface opposition.
             */
            auto frozenSolve = grab_finger_pose_runtime::solveFrozenMeshFingerPoseBase(fingerScratch.localTriangles,
                frozenMeshWorld, handTransform, isLeft, solveAnchorWorld, fingerPoseTargets, fingerScratch.spatialIndex,
                fingerScratch.worldTriangles,
                grab_finger_pose_runtime::FrozenMeshFingerPoseSolveOptions{
                    .minValue = g_rockConfig.rockGrabFingerMinValue,
                    .maxTriangleDistanceSquared = g_rockConfig.rockGrabMaxTriangleDistance,
                    .rejectBacksideHits = g_rockConfig.rockGrabFingerRejectBacksideHits,
                    .surfacePlaneToleranceGameUnits = g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits,
                    .allowSurfaceAimTargets = true,
                    .sweepContactRadiusGameUnits = g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits,
                    .thumbSweepMaxOpenValue = g_rockConfig.rockGrabThumbSweepMaxOpenValue,
                    .fingerSweepMaxOpenValue = g_rockConfig.rockGrabFingerSweepMaxOpenValue,
                    .meshFingerPoseEnabled = true,
                    .captureSweepDebug = false,
                    .meshRelation = grab_finger_pose_runtime::FingerPoseMeshRelation::AlreadyAtCommandedSeat,
                },
                capturedFingerSnapshotValid ? &capturedFingerSnapshot : nullptr);
            grab_finger_pose_runtime::captureSurfaceAimObjectLocal(frozenSolve.pose, frozenMeshWorld);
            spatialIndexBuilt = frozenSolve.spatialIndexBuilt;
            commandedOpenDirectionsValid = frozenSolve.commandedOpenDirectionsValid;
            meshFingerPose = frozenSolve.pose;
            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseSpatialNodeVisits,
                meshFingerPose.spatialNodeVisitCount);
            performance_profiler::observeValue(performance_profiler::ValueMetric::EquippedWeaponFingerPoseTriangleTests,
                meshFingerPose.spatialTriangleTestCount);
            if (meshFingerPose.solved) {
                const bool completeDirectFingerEvidence =
                    grab_finger_pose_runtime::hasCompleteFingerContactEvidence(meshFingerPose);
                const auto oppositionConfig = currentWeaponOppositionPocketConfig();
                const auto oppositionPocket = !completeDirectFingerEvidence &&
                        oppositionConfig.enabled && frozenSolve.liveFingerSnapshotValid
                    ? grab_finger_pose_runtime::findLocalOppositionPocketEvidence(fingerScratch.worldTriangles,
                          frozenSolve.liveFingerSnapshot, frozenGripPoint, meshFingerPose.contactValidMask,
                          oppositionConfig.minFingerGapGameUnits,
                          (std::max)(oppositionConfig.maxFingerGapGameUnits,
                              WEAPON_OPPOSITION_MAX_FINGER_GAP_GAME_UNITS),
                          oppositionConfig.maxPocketDistanceGameUnits,
                          (std::min)(WEAPON_OPPOSITION_SEGMENT_PROBE_RADIUS_GAME_UNITS,
                              (std::max)(0.0f, g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits)))
                    : grab_finger_pose_runtime::OppositionPocketEvidence{};
                if (oppositionPocket.valid) {
                    applyStableWeaponOppositionPose(meshFingerPose, oppositionConfig,
                        oppositionPocket.opposedFingerIndex);
                    ROCK_LOG_INFO(Weapon,
                        "TwoHandedGrip: local opposition pocket accepted hand={} kind={} directMask=0x{:02X} "
                        "endpointMask=0x{:02X} directEndpoints=0x{:02X} gap={:.3f} gripSurfaceDistance={:.3f}",
                        isLeft ? "left" : "right",
                        grab_finger_pose_runtime::oppositionPocketKindName(oppositionPocket.kind),
                        static_cast<unsigned>(meshFingerPose.contactValidMask),
                        static_cast<unsigned>(oppositionPocket.endpointMask),
                        static_cast<unsigned>(oppositionPocket.directEndpointMask), oppositionPocket.fingerGapGameUnits,
                        oppositionPocket.gripToSurfaceDistanceGameUnits);
                }
                const bool completeFingerEvidence = completeDirectFingerEvidence || oppositionPocket.valid;
                if (completeFingerEvidence) {
                    meshFingerPosePtr = &meshFingerPose;
                } else {
                    ROCK_LOG_INFO(Weapon,
                        "TwoHandedGrip: mesh finger pose failed closed hand={} contactMask=0x{:02X} "
                        "requiredMask=0x{:02X} hits={} sources={} sourceTriangles={} candidateTriangles={}",
                        isLeft ? "left" : "right", static_cast<unsigned>(meshFingerPose.contactValidMask),
                        static_cast<unsigned>(grab_finger_pose_runtime::kCompleteFingerContactMask),
                        meshFingerPose.hitCount, compositeEvidenceViewCount, sourceTriangleCount,
                        meshFingerPose.candidateTriangleCount);
                }
                if (completeDirectFingerEvidence && frozenSolve.liveFingerSnapshotValid &&
                    frozenSolve.commandedOpenDirectionsValid &&
                    grab_finger_pose_runtime::buildSurfaceContactSplayValues(meshFingerPose,
                        frozenSolve.liveFingerSnapshot, frozenSolve.commandedOpenDirectionsWorld,
                        capturedFingerSplayRadians)) {
                    capturedFingerSplayRadiansPtr = &capturedFingerSplayRadians;
                }
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: mesh finger pose hand={} values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} "
                    "contactMask=0x{:02X} sources={} sourceTris={} candidateTris={} spatial={} nodes={} tests={} "
                    "commandedAnchors={} altThumb={} thumbLane={}",
                    isLeft ? "left" : "right", meshFingerPose.values[0], meshFingerPose.values[1],
                    meshFingerPose.values[2], meshFingerPose.values[3], meshFingerPose.values[4],
                    meshFingerPose.hitCount, static_cast<unsigned>(meshFingerPose.contactValidMask),
                    compositeEvidenceViewCount, sourceTriangleCount, meshFingerPose.candidateTriangleCount,
                    spatialIndexBuilt ? "yes" : "no", meshFingerPose.spatialNodeVisitCount,
                    meshFingerPose.spatialTriangleTestCount, commandedOpenDirectionsValid ? "yes" : "no",
                    meshFingerPose.usedAlternateThumbCurve ? "yes" : "no",
                    grab_finger_pose_math::thumbLaneName(meshFingerPose.selectedThumbLane));
                if (meshFingerPose.hasThumbCurveDiagnostics) {
                    ROCK_LOG_DEBUG(Weapon,
                        "TwoHandedGrip: thumb curve primary(hit={} value={:.2f} behind={}) opposition(hit={} "
                        "value={:.2f} behind={}) sidePad(hit={} value={:.2f} behind={}) selected={}",
                        meshFingerPose.thumbPrimaryCurve.hit ? "yes" : "no", meshFingerPose.thumbPrimaryCurve.value,
                        meshFingerPose.thumbPrimaryCurve.openedByBehindContact ? "yes" : "no",
                        meshFingerPose.thumbAlternateCurve.hit ? "yes" : "no", meshFingerPose.thumbAlternateCurve.value,
                        meshFingerPose.thumbAlternateCurve.openedByBehindContact ? "yes" : "no",
                        meshFingerPose.thumbSidePadCurve.hit ? "yes" : "no", meshFingerPose.thumbSidePadCurve.value,
                        meshFingerPose.thumbSidePadCurve.openedByBehindContact ? "yes" : "no",
                        grab_finger_pose_math::thumbLaneName(meshFingerPose.selectedThumbLane));
                }
            }
        }

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: weapon grab pose applied=Current swept arc hand={} pose={} hits={} candidates={}",
            isLeft ? "left" : "right", meshFingerPosePtr ? "mesh" : "closed-fallback", meshFingerPose.hitCount,
            meshFingerPose.candidateTriangleCount);

        setSupportGripPose(isLeft, meshFingerPosePtr, capturedFingerSplayRadiansPtr,
            SupportGripPoseFallback::FullyClosed);
        if (meshFingerPosePtr && grip.hasFingerPose) {
            std::array<RE::NiTransform, 15> localTransforms{};
            std::uint16_t localTransformMask = 0;
            const auto handPose = grip.hasFingerSplay
                ? frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose, grip.fingerSplayRadians)
                : frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose);
            if (buildFullHandLocalTransformsForMeshPose(isLeft, *meshFingerPosePtr, handPose,
                    capturedFingerBoneSnapshotValid ? &capturedFingerBoneSnapshot : nullptr, localTransforms,
                    localTransformMask)) {
                grip.fingerLocalTransforms = localTransforms;
                grip.fingerLocalTransformMask = localTransformMask;
                grip.hasFingerLocalTransforms = true;
                double maxBasisError = 0.0;
                for (const auto& local : localTransforms) {
                    maxBasisError =
                        (std::max)(maxBasisError, transform_math::storedRotationOrthonormalityError(local.rotate));
                }
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: dynamic finger pose prepared hand={} mask=0x{:04X} lane={} "
                    "curls=({:.3f},{:.3f},{:.3f},{:.3f},{:.3f}) maxBasisError={:.8f}",
                    isLeft ? "left" : "right", grip.fingerLocalTransformMask,
                    grab_finger_pose_math::thumbLaneName(meshFingerPosePtr->selectedThumbLane),
                    meshFingerPosePtr->values[0], meshFingerPosePtr->values[1], meshFingerPosePtr->values[2],
                    meshFingerPosePtr->values[3], meshFingerPosePtr->values[4], maxBasisError);
            }
        }
    }
}
