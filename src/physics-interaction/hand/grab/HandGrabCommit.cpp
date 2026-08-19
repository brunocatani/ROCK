#include "physics-interaction/hand/Hand.h"

/*
 * COMMIT: the last two phases of acquisition, after the seat is frozen. Nothing
 * here may reject the grab - by this point _grabFrame is populated and the hold
 * is being made real.
 *
 *   commitGrabDrive      - collision suppression, inertia, the constraint or
 *                          proxy drive, body flag leases, nearby damping
 *   publishGrabFingerPose - the loose-weapon attach path or the mesh finger pose
 *
 * commitGrabDrive is the first phase that takes native ownership, so it is also
 * the first whose failure needs the Tier-C teardown: its one failure exit routes
 * through abortGrabAcquisition, which destroys the proxy and restores inertia
 * and collision as well as clearing the frame.
 */

#include "physics-interaction/hand/grab/HandGrabAcquisitionContext.h"

namespace rock
{
    using namespace hand_grab_detail;

    bool Hand::commitGrabDrive(hand_grab_detail::GrabAcquisitionContext& context)
    {
        if (!context.world || !context.bhkWorld || !context.selection.isValid() || !context.sharedContext) {
            return false;
        }

        auto* world = context.world;
        auto* bhkWorld = context.bhkWorld;
        const auto& sel = context.selection;
        const auto& sharedContext = *context.sharedContext;
        const auto* bodyBoneColliders = context.bodyBoneColliders;
        auto* rootNode = context.rootNode;
        auto& activeLifecycle = context.activeLifecycle;
        const auto& handWorldTransform = context.handWorldTransform;
        const float tau = context.tau;
        const float damping = context.damping;
        const float maxForce = context.maxForce;
        const float proportionalRecovery = context.proportionalRecovery;
        const float constantRecovery = context.constantRecovery;
        const auto objectBodyId = context.objectBodyId;
        const bool joiningPeerHeldObject = context.joiningPeerHeldObject;
        const bool looseWeaponGrab = context.looseWeaponGrab;
        const auto& proxyFrameWorldAtGrab = context.proxyFrameWorldAtGrab;
        const auto& grabGripPoint = context.grabGripPoint;
        const bool meshGrabFound = context.meshGrabFound;
        const char* grabPointMode = context.grabPointMode;
        const char* grabFallbackReason = context.grabFallbackReason;

        auto abortGrab = [&]() -> bool {
            abortGrabAcquisition(GrabAcquisitionUnwind{
                .world = world,
                .bhkWorld = bhkWorld,
                .rootNode = rootNode,
                .lifecycle = &activeLifecycle,
                .objectBodyId = objectBodyId,
                .targetKind = sel.targetKind,
                .originalMotionPropsId = context.selectedOriginalMotionPropsId,
                .joiningPeerHeldObject = joiningPeerHeldObject,
                .consumedPullPrepLifecycle = context.consumedPullPrepLifecycle,
            });
            return false;
        };

        {
            const RE::hkVector4f zeroVel{ 0.0f, 0.0f, 0.0f, 0.0f };
            havok_runtime::setBodyVelocityDeferred(world, objectBodyId.value, zeroVel, zeroVel);

            for (auto bid : _heldBodyIds) {
                if (bid != objectBodyId.value) {
                    havok_runtime::setBodyVelocityDeferred(world, bid, zeroVel, zeroVel);
                }
            }
        }

        const auto grabActivation = activateHeldObjectBodySet(world, objectBodyId.value, _heldBodyIds);
        if (grabActivation.failedActivationCount > 0) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB activation incomplete: primaryBody={} bodies={} activated={} failed={}",
                handName(),
                objectBodyId.value,
                grabActivation.bodyCount,
                grabActivation.activatedCount,
                grabActivation.failedActivationCount);
        }

        suppressHandCollisionForGrab(world, bodyBoneColliders);

        if (joiningPeerHeldObject && sharedContext.peerSavedObjectState) {
            copyPeerInertiaSnapshot(_savedObjectState, *sharedContext.peerSavedObjectState);
            ROCK_LOG_DEBUG(Hand,
                "{} hand joined peer-held object inertia snapshot: formID={:08X} peerMotions={} inertiaModified={}",
                handName(),
                sel.refr ? sel.refr->GetFormID() : 0,
                sharedContext.peerSavedObjectState->motionInertiaStates.size(),
                sharedContext.peerSavedObjectState->inertiaModified ? "yes" : "no");
        } else {
            normalizeGrabbedInertiaForBodies(world, objectBodyId, _heldBodyIds, _savedObjectState, looseWeaponGrab);
        }

        {
            RE::NiPoint3 legacyPalmPivotAWorld = computeGrabLegacyPalmPivotAWorldFromHandBasis(handWorldTransform, _isLeft);
            RE::NiPoint3 grabPivotAWorld =
                _grabFrame.hasTelemetryCapture ? _grabFrame.grabPivotWorldAtGrab : computeGrabPivotAWorld(world, handWorldTransform);
            const float gameToHkScale = gameToHavokScale();
            const RE::NiTransform initialDesiredBodyWorld = _grabFrame.desiredBodyWorldAtGrab;

            float pivotAHk[4];
            pivotAHk[0] = grabPivotAWorld.x * gameToHkScale;
            pivotAHk[1] = grabPivotAWorld.y * gameToHkScale;
            pivotAHk[2] = grabPivotAWorld.z * gameToHkScale;
            pivotAHk[3] = 0.0f;

            float gripWorldHk[4];
            gripWorldHk[0] = grabGripPoint.x * gameToHkScale;
            gripWorldHk[1] = grabGripPoint.y * gameToHkScale;
            gripWorldHk[2] = grabGripPoint.z * gameToHkScale;
            gripWorldHk[3] = 0.0f;

            {
                float pivotAToGrab = std::sqrt(
                    (pivotAHk[0] - gripWorldHk[0]) * (pivotAHk[0] - gripWorldHk[0]) + (pivotAHk[1] - gripWorldHk[1]) * (pivotAHk[1] - gripWorldHk[1]) +
                    (pivotAHk[2] - gripWorldHk[2]) * (pivotAHk[2] - gripWorldHk[2]));
                float legacyPalmPivotAToHandOrigin = std::sqrt((legacyPalmPivotAWorld.x - handWorldTransform.translate.x) * (legacyPalmPivotAWorld.x - handWorldTransform.translate.x) +
                    (legacyPalmPivotAWorld.y - handWorldTransform.translate.y) * (legacyPalmPivotAWorld.y - handWorldTransform.translate.y) +
                    (legacyPalmPivotAWorld.z - handWorldTransform.translate.z) * (legacyPalmPivotAWorld.z - handWorldTransform.translate.z));
                float pivotAToHandOrigin = std::sqrt((grabPivotAWorld.x - handWorldTransform.translate.x) * (grabPivotAWorld.x - handWorldTransform.translate.x) +
                    (grabPivotAWorld.y - handWorldTransform.translate.y) * (grabPivotAWorld.y - handWorldTransform.translate.y) +
                    (grabPivotAWorld.z - handWorldTransform.translate.z) * (grabPivotAWorld.z - handWorldTransform.translate.z));
                ROCK_LOG_DEBUG(Hand,
                    "GRAB DIAG {}: legacyPalmPivotAWorld=({:.1f},{:.1f},{:.1f}) handPos=({:.1f},{:.1f},{:.1f}) "
                    "pocket=({:.1f},{:.1f},{:.1f}) grip=({:.1f},{:.1f},{:.1f}) meshGrab={} grabPointMode={} fallbackReason={} "
                    "frozenPivotB=({:.2f},{:.2f},{:.2f}) contactPatchEvidence={} contactPatchPivot={} patchHits={} multiFinger={} mfGroups={} mfSpread={:.2f} "
                    "activePoint={} activeUsesFingerEvidence={} palmSeatPoint={} fingerEvidencePoint={} "
                    "pivotAToGrab_hk={:.4f} ({:.1f} game units) legacyPalmPivotAToHandOrigin={:.1f} pivotAToHandOrigin={:.1f} game units "
                    "selectionToGripEvidence={:.1f} fingerPoseAim={} fingerPoseAimReason={}",
                    handName(), legacyPalmPivotAWorld.x, legacyPalmPivotAWorld.y, legacyPalmPivotAWorld.z, handWorldTransform.translate.x, handWorldTransform.translate.y, handWorldTransform.translate.z, grabPivotAWorld.x,
                    grabPivotAWorld.y, grabPivotAWorld.z, grabGripPoint.x, grabGripPoint.y, grabGripPoint.z, meshGrabFound, grabPointMode, grabFallbackReason,
                    _grabFrame.pivotBBodyLocalGame.x, _grabFrame.pivotBBodyLocalGame.y, _grabFrame.pivotBBodyLocalGame.z,
                    _grabFrame.hasContactPatchEvidence ? "yes" : "no", "no",
                    _grabFrame.contactPatchSampleCount, _grabFrame.hasMultiFingerContactPatch ? "yes" : "no", _grabFrame.multiFingerContactGroupCount,
                    _grabFrame.multiFingerContactSpreadGameUnits, _grabFrame.activeGrabPointMode, _grabFrame.activeGrabPointUsesMultiFingerEvidence ? "yes" : "no",
                    _grabFrame.palmSeatPointMode, _grabFrame.fingerEvidencePointMode, pivotAToGrab,
                    pivotAToGrab * havokToGameScale(), legacyPalmPivotAToHandOrigin, pivotAToHandOrigin, _grabFrame.selectionToGripEvidenceDistanceGameUnits,
                    _grabFrame.fingerPoseAimValid ? "yes" : "no", _grabFrame.fingerPoseAimReason);
            }

            const char* driveReason = joiningPeerHeldObject ? "joining-peer-held-loose-object" : "ordinary-dynamic-loose-object";
            if (!createProxyConstraintGrabDrive(
                    bhkWorld,
                    world,
                    objectBodyId,
                    proxyFrameWorldAtGrab,
                    handWorldTransform,
                    grabPivotAWorld,
                    tau,
                    damping,
                    maxForce,
                    held_object_drive_policy::sanitizeMotorAuthorityScale(sharedGrabAuthorityForceScale(joiningPeerHeldObject)),
                    proportionalRecovery,
                    constantRecovery,
                    looseWeaponGrab,
                    driveReason)) {
                ROCK_LOG_ERROR(Hand,
                    "{} hand GRAB FAILED: proxy constraint creation failed bodyId={} targetBody=({:.2f},{:.2f},{:.2f}) pivotBConstraint=({:.2f},{:.2f},{:.2f}) joiningPeer={}",
                    handName(),
                    objectBodyId.value,
                    initialDesiredBodyWorld.translate.x,
                    initialDesiredBodyWorld.translate.y,
                    initialDesiredBodyWorld.translate.z,
                    activeProxyConstraintPivotBLocalGame().x,
                    activeProxyConstraintPivotBLocalGame().y,
                    activeProxyConstraintPivotBLocalGame().z,
                    joiningPeerHeldObject ? "yes" : "no");
            }
        }

        const bool driveCreated = _activeConstraint.isValid() && _grabAuthorityProxy.isValid();
        if (!driveCreated) {
            ROCK_LOG_ERROR(Hand, "{} hand GRAB FAILED: proxy-constraint dynamic grab creation failed", handName());
            return abortGrab();
        }

        _heldObjectIsLooseWeapon = looseWeaponGrab;
        if (_heldObjectIsLooseWeapon) {
            suppressBodyCollisionForHeldLooseWeapon(world, bodyBoneColliders);
        }
        const auto massSummaryAtGrab = readHeldBodyMassSummary(
            world,
            _savedObjectState.bodyId,
            _heldBodyIds,
            _heldDriveDecision.includeConnectedMass);
        const float effectiveMassAtGrab = effectiveGrabMotorMass(massSummaryAtGrab.motorMass());
        const float sharedLinearForce = _activeConstraint.linearMotor ?
            (std::max)(std::fabs(_activeConstraint.linearMotor->minForce), std::fabs(_activeConstraint.linearMotor->maxForce)) :
            maxForce;
        const float sharedAngularForce = _activeConstraint.angularMotor ?
            (std::max)(std::fabs(_activeConstraint.angularMotor->minForce), std::fabs(_activeConstraint.angularMotor->maxForce)) :
            0.0f;
        ROCK_LOG_DEBUG(Hand,
            "{} hand dynamic grab created: drive={} bodyDriveMode={} driveReason={} forceShare={:.2f} linearScope={} angularScope={} massScope={} looseWeapon={} constraint={} proxyBody={} handBody={} objBody={} heldBodies={} mass={:.2f} effectiveMotorMass={:.2f} primaryMass={:.2f} massBodies={} motions={} longLever={:.1f}gu linearTau={:.3f} angularTau={:.3f} linearDamping={:.2f} angularDamping={:.2f} linearForce={:.0f} angularForce={:.0f} propRecov={:.1f} constRecov={:.1f} rotRef={}",
            handName(),
            kHeldObjectDriveName,
            held_object_drive_policy::modeName(_heldDriveDecision.mode),
            _heldDriveDecision.reason,
            sharedGrabAuthorityForceScale(joiningPeerHeldObject),
            _heldDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
            _heldDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
            _heldDriveDecision.includeConnectedMass ? "bodySet" : "primaryOnly",
            _heldObjectIsLooseWeapon ? "yes" : "no",
            _activeConstraint.constraintId,
            _grabAuthorityProxy.isValid() ? _grabAuthorityProxy.getBodyId().value : INVALID_BODY_ID,
            _handBody.getBodyId().value,
            objectBodyId.value,
            _heldBodyIds.size(),
            massSummaryAtGrab.motorMass(),
            effectiveMassAtGrab,
            massSummaryAtGrab.primaryMass,
            massSummaryAtGrab.sampledBodies,
            massSummaryAtGrab.uniqueMotions,
            _grabFrame.longObjectLeverGameUnits,
            _activeConstraint.linearMotor ? _activeConstraint.linearMotor->tau : tau,
            _activeConstraint.angularMotor ? _activeConstraint.angularMotor->tau : g_rockConfig.rockGrabAngularTau,
            _activeConstraint.linearMotor ? _activeConstraint.linearMotor->damping : damping,
            _activeConstraint.angularMotor ? _activeConstraint.angularMotor->damping : g_rockConfig.rockGrabAngularDamping,
            sharedLinearForce,
            sharedAngularForce,
            _activeConstraint.linearMotor ? _activeConstraint.linearMotor->proportionalRecoveryVelocity : proportionalRecovery,
            _activeConstraint.linearMotor ? _activeConstraint.linearMotor->constantRecoveryVelocity : constantRecovery,
            kGrabObjectRotationReferenceName);

        const auto heldFlagLeases =
            acquireHeldObjectBodyFlagLeases(world, _savedObjectState.bodyId.value, _heldBodyIds, heldBodyFlagLeaseOwner(this));
        if (heldFlagLeases.failedLeaseCount > 0) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB held body flag lease incomplete: primaryBody={} bodies={} collision={} authority={} failed={}",
                handName(),
                _savedObjectState.bodyId.value,
                heldFlagLeases.bodyCount,
                heldFlagLeases.collisionLeaseCount,
                heldFlagLeases.authorityLeaseCount,
                heldFlagLeases.failedLeaseCount);
        } else {
            ROCK_LOG_DEBUG(Hand,
                "{} hand GRAB held body flag leases acquired: primaryBody={} bodies={} collision={} authority={}",
                handName(),
                _savedObjectState.bodyId.value,
                heldFlagLeases.bodyCount,
                heldFlagLeases.collisionLeaseCount,
                heldFlagLeases.authorityLeaseCount);
        }
        clearHeldBodyContactSnapshot();
        _activeGrabLifecycle = std::move(activeLifecycle);

        {
            int count = (std::min)(static_cast<int>(_heldBodyIds.size()), MAX_HELD_BODIES);
            for (int i = 0; i < count; i++) {
                _heldBodyIdsSnapshot[i] = _heldBodyIds[i];
            }
            _heldBodyIdsCount.store(count, std::memory_order_release);
            _isHoldingFlag.store(true, std::memory_order_release);
        }

        if (g_rockConfig.rockGrabNearbyDampingEnabled) {
            object_physics_body_set::BodySetScanOptions dampingOptions{};
            dampingOptions.mode = physics_body_classifier::InteractionMode::PassivePush;
            dampingOptions.rightHandBodyId = _isLeft ? INVALID_BODY_ID : _handBody.getBodyId().value;
            dampingOptions.leftHandBodyId = _isLeft ? _handBody.getBodyId().value : INVALID_BODY_ID;
            dampingOptions.heldBySameHand = &_heldBodyIds;
            dampingOptions.maxDepth = (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
            {
                performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabNearbyDampingBegin);
                _nearbyGrabDamping = nearby_grab_damping::beginNearbyGrabDamping(bhkWorld,
                    world,
                    sel.refr,
                    _heldBodyIds,
                    grabGripPoint,
                    g_rockConfig.rockGrabNearbyDampingRadius,
                    g_rockConfig.rockGrabNearbyDampingSeconds,
                    g_rockConfig.rockGrabNearbyLinearDamping,
                    g_rockConfig.rockGrabNearbyAngularDamping,
                    dampingOptions);
            }
            performance_profiler::observeValue(performance_profiler::ValueMetric::GrabNearbyDampingMotions, _nearbyGrabDamping.motions.size());
        } else {
            _nearbyGrabDamping.clear();
        }

        return true;
    }


    void Hand::publishGrabFingerPose(hand_grab_detail::GrabAcquisitionContext& context)
    {
        if (!context.world || !context.selection.isValid()) {
            return;
        }

        auto* world = context.world;
        const auto& sel = context.selection;
        const auto& handWorldTransform = context.handWorldTransform;
        const auto& objectWorldTransform = context.objectWorldTransform;
        const auto& resolvedGrabOffsetSource = context.resolvedGrabOffsetSource;
        const auto& grabFingerPoseMeshTriangles = context.grabFingerPoseMeshTriangles;

        stopSelectionHighlight();
        clearSelectedCloseFingerPose();
        _grabFingerPose = {};
        _grabFingerTriangleIndex.clear();
        const bool useLooseWeaponPrimaryAttachHandPose = _grabFrame.syntheticLooseWeaponPrimaryAttach;
        _hasGrabFingerPose = g_rockConfig.rockGrabMeshFingerPoseEnabled && !useLooseWeaponPrimaryAttachHandPose;
        _grabFingerProbeStart = {};
        _grabFingerProbeEnd = {};
        _hasGrabFingerProbeDebug = false;
        _grabFingerSweepDebugCapture = {};
        _grabFingerSweepDebugObjectWorld = {};
        _hasGrabFingerSweepDebug = false;
        _grabFingerPadProbeStart = {};
        _grabFingerPadProbeEnd = {};
        _grabFingerPadProbeHit = {};
        _grabFingerPadProbeHitValid = {};
        _hasGrabFingerPadProbeDebug = false;
        _grabFingerSurfaceTarget = {};
        _grabFingerSurfaceTargetValid = {};
        _hasGrabFingerSurfaceTargetDebug = false;
        _grabFingerPosePublished = false;
        if (useLooseWeaponPrimaryAttachHandPose) {
            const auto grabOffsetFingerPoseSource = g_rockConfig.rockGrabMeshFingerPoseEnabled ?
                resolveGrabOffsetFingerPoseSource(resolvedGrabOffsetSource) :
                GrabOffsetFingerPoseSource{};
            if (grabOffsetFingerPoseSource.valid) {
                /*
                 * Reuse the same calibrated-or-saved source that established
                 * the object attach. This synthetic attach has no mesh contact
                 * to solve fingers from, so without the resolved snapshot it
                 * would fall back to a generic canned pose. This is a one-time
                 * publish; mesh-resolve paths must not mutate it afterward.
                 */
                _grabFingerPose = grabOffsetFingerPoseSource.pose;
                applyRockGrabHandPose(_isLeft,
                    _grabFingerPose,
                    _grabFingerJointPose,
                    _hasGrabFingerJointPose,
                    _grabFingerLocalTransforms,
                    _grabFingerLocalTransformMask,
                    _hasGrabFingerLocalTransforms,
                    0.0f,
                    /*publishLocalTransforms=*/false);
                _grabFingerPosePublished = true;
                ROCK_LOG_INFO(Hand,
                    "{} hand loose weapon attach: applying {} finger pose",
                    handName(),
                    grabOffsetFingerPoseSource.reason);
            } else {
                _grabFingerPosePublished = publishLooseWeaponPrimaryAttachHandPose(_isLeft, sel.refr);
                if (!_grabFingerPosePublished) {
                    ROCK_LOG_WARN(Hand, "{} hand loose weapon attach: failed to publish FRIK weapon hand pose", handName());
                }
            }
        } else if (_hasGrabFingerPose) {
            const bool pinchFingerPose = _grabFrame.seatMode == GrabSeatMode::PinchPocket;
            const bool touchHeldAtCommit = _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld;
            if (pinchFingerPose && !touchHeldAtCommit) {
                ROCK_LOG_DEBUG(Hand, "{} THREE-PHASE GRAB POSE: pinch solve deferred until TouchHeld phase={} cachedTriangles={} poseTargets={}", handName(),
                    grab_three_phase::phaseName(_grabAcquisitionPhase),
                    _grabFrame.fingerPoseLocalMeshTriangles.empty() ? _grabFrame.localMeshTriangles.size() : _grabFrame.fingerPoseLocalMeshTriangles.size(),
                    _grabFrame.fingerPoseTargetCount);
            } else {
                /*
                 * Regular grabs solve exactly once against the already-frozen
                 * commanded seat, never against the object's transient approach
                 * pose. The endpoint therefore exists before the first approach
                 * frame and cannot visibly change at TouchHeld. Pinch keeps its
                 * established at-touch triangle path above.
                */
                const RE::NiTransform& targetObjectWorld = pinchFingerPose ? objectWorldTransform : _grabFrame.desiredObjectWorldAtGrab;
                std::vector<TriangleData> targetFingerPoseWorldTriangles = pinchFingerPose ? grabFingerPoseMeshTriangles : std::vector<TriangleData>{};
                const auto& localFingerPoseTriangles = !_grabFrame.fingerPoseLocalMeshTriangles.empty() ? _grabFrame.fingerPoseLocalMeshTriangles : _grabFrame.localMeshTriangles;

                const RE::NiTransform& targetFingerHandTransform = handWorldTransform;
                root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshotAtGrab{};
                const RE::NiPoint3 fingerPosePivotWorld =
                    _grabFrame.hasTelemetryCapture ? _grabFrame.grabPivotWorldAtGrab : computeGrabPivotAWorld(world, targetFingerHandTransform);
                const auto targetFingerPoseTargets = rebuildFingerPoseTargetsFromGrabFrame(_grabFrame, targetObjectWorld);
                grab_finger_pose_runtime::FingerSweepDebugCapture sweepDebugCapture{};
                grab_finger_pose_runtime::SolvedGrabFingerPose fingerPose{};
                bool spatialIndexBuilt = false;
                bool commandedOpenDirectionsValid = false;
                if (pinchFingerPose) {
                    const auto* liveFingerSnapshotAtGrabPtr =
                        root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft, liveFingerSnapshotAtGrab) ? &liveFingerSnapshotAtGrab : nullptr;
                    fingerPose = grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(targetFingerPoseWorldTriangles, targetFingerHandTransform, _isLeft, fingerPosePivotWorld,
                        targetFingerPoseTargets, g_rockConfig.rockGrabFingerMinValue, g_rockConfig.rockGrabMaxTriangleDistance, !pinchFingerPose, liveFingerSnapshotAtGrabPtr,
                        g_rockConfig.rockGrabFingerRejectBacksideHits, g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits, _grabFrame.fingerPoseAimValid,
                        g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits, -1.0f, g_rockConfig.rockGrabThumbSweepMaxOpenValue, g_rockConfig.rockGrabFingerSweepMaxOpenValue,
                        nullptr, nullptr, nullptr, nullptr, grab_finger_pose_runtime::FingerPoseMeshRelation::AlreadyAtCommandedSeat);
                    applyPinchFingerPosePolicy(fingerPose, _grabFrame, g_rockConfig.rockGrabFingerMinValue);
                    grab_finger_pose_runtime::useThumbIndexCurveOnlyPose(fingerPose);
                    std::array<grab_finger_pose_runtime::FingerPadSurfaceEvidence, 5> padCaptureEvidence{};
                    (void)grab_finger_pose_runtime::refineGrabFingerPoseWithPadProbes(fingerPose, targetFingerPoseWorldTriangles, targetFingerPoseTargets,
                        liveFingerSnapshotAtGrab, targetObjectWorld, g_rockConfig.rockGrabMeshFingerPoseEnabled, true, padCaptureEvidence, true);
                    grab_finger_pose_runtime::captureSurfaceAimObjectLocal(fingerPose, targetObjectWorld);
                } else {
                    const auto frozenSolve = grab_finger_pose_runtime::solveFrozenMeshFingerPose(
                        localFingerPoseTriangles,
                        targetObjectWorld,
                        targetFingerHandTransform,
                        _isLeft,
                        fingerPosePivotWorld,
                        targetFingerPoseTargets,
                        _grabFingerTriangleIndex,
                        targetFingerPoseWorldTriangles,
                        grab_finger_pose_runtime::FrozenMeshFingerPoseSolveOptions{
                            .minValue = g_rockConfig.rockGrabFingerMinValue,
                            .maxTriangleDistanceSquared = g_rockConfig.rockGrabMaxTriangleDistance,
                            .rejectBacksideHits = g_rockConfig.rockGrabFingerRejectBacksideHits,
                            .surfacePlaneToleranceGameUnits = g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits,
                            .allowSurfaceAimTargets = _grabFrame.fingerPoseAimValid,
                            .sweepContactRadiusGameUnits = g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits,
                            .thumbSweepMaxOpenValue = g_rockConfig.rockGrabThumbSweepMaxOpenValue,
                            .fingerSweepMaxOpenValue = g_rockConfig.rockGrabFingerSweepMaxOpenValue,
                            .meshFingerPoseEnabled = g_rockConfig.rockGrabMeshFingerPoseEnabled,
                            .captureSweepDebug = g_rockConfig.rockDebugShowGrabFingerSweptArc,
                        });
                    fingerPose = frozenSolve.pose;
                    sweepDebugCapture = frozenSolve.sweepDebug;
                    liveFingerSnapshotAtGrab = frozenSolve.liveFingerSnapshot;
                    spatialIndexBuilt = frozenSolve.spatialIndexBuilt;
                    commandedOpenDirectionsValid = frozenSolve.commandedOpenDirectionsValid;
                }
                _grabFingerSweepDebugCapture = sweepDebugCapture;
                _grabFingerSweepDebugObjectWorld = targetObjectWorld;
                _hasGrabFingerSweepDebug = sweepDebugCapture.valid;
                _grabFingerPose = fingerPose;
                _grabFingerProbeStart = fingerPose.probeStart;
                _grabFingerProbeEnd = fingerPose.probeEnd;
                _hasGrabFingerProbeDebug = fingerPose.candidateTriangleCount > 0;
                auto publishFingerPose = grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(_grabFingerPose, targetObjectWorld);
                if (g_rockConfig.rockDebugShowGrabFingerProbes) {
                    std::array<grab_finger_pose_runtime::FingerPadSurfaceEvidence, 5> padEvidence{};
                    (void)grab_finger_pose_runtime::refineGrabFingerPoseWithPadProbes(publishFingerPose, targetFingerPoseWorldTriangles, targetFingerPoseTargets,
                        liveFingerSnapshotAtGrab, targetObjectWorld, g_rockConfig.rockGrabMeshFingerPoseEnabled, true, padEvidence, false);
                    const auto padDebug = makeFingerPadPublishDebug(publishFingerPose, padEvidence);
                    _grabFingerPadProbeStart = padDebug.padProbeStart;
                    _grabFingerPadProbeEnd = padDebug.padProbeEnd;
                    _grabFingerPadProbeHit = padDebug.padProbeHit;
                    _grabFingerPadProbeHitValid = padDebug.padProbeHitValid;
                    _hasGrabFingerPadProbeDebug = padDebug.hasPadProbeDebug;
                    _grabFingerSurfaceTarget = padDebug.surfaceTarget;
                    _grabFingerSurfaceTargetValid = padDebug.surfaceTargetValid;
                    _hasGrabFingerSurfaceTargetDebug = padDebug.hasSurfaceTargetDebug;
                } else {
                    _hasGrabFingerPadProbeDebug = false;
                    _hasGrabFingerSurfaceTargetDebug = false;
                }
                if (!touchHeldAtCommit) {
                    publishFingerPose = buildAcquisitionFingerPose(publishFingerPose, 0.0f);
                }
                applyRockGrabHandPose(_isLeft, publishFingerPose, _grabFingerJointPose, _hasGrabFingerJointPose, _grabFingerLocalTransforms, _grabFingerLocalTransformMask,
                    _hasGrabFingerLocalTransforms, 0.0f, touchHeldAtCommit, touchHeldAtCommit);
                _grabFingerPosePublished = true;
                ROCK_LOG_DEBUG(Hand, "{} THREE-PHASE GRAB POSE TARGET: phase={} targetSpace=yes solved={} hits={} triangles={} spatial={} nodes={} tests={} commandedAnchors={}",
                    handName(), grab_three_phase::phaseName(_grabAcquisitionPhase), _grabFingerPose.solved ? "yes" : "no", _grabFingerPose.hitCount,
                    _grabFingerPose.candidateTriangleCount, _grabFingerPose.usedSpatialIndex ? "yes" : "no", _grabFingerPose.spatialNodeVisitCount,
                    _grabFingerPose.spatialTriangleTestCount, commandedOpenDirectionsValid ? "yes" : "no");
            }
        }


    }
}
