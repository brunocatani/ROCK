// Grab commit: freezes the grab authority relation, initializes the held state,
// solves and publishes the grab finger pose and creates the grab constraint
// (commitGrabConstraintAndPose). Also the hand-collision suppression a grab
// applies and restores.
#include "physics-interaction/hand/HandGrabInternal.h"

namespace rock
{

    namespace
    {
        bool publishLooseWeaponPrimaryAttachHandPose(bool isLeft, RE::TESObjectREFR* refr,
            const AuthoredWeaponGripPose& pose)
        {
            const auto* weapon = looseWeaponFormFromRef(refr);
            if (!pose.valid() || !weapon || pose.isLeft != isLeft || pose.weaponFormId != weapon->formID)
                return false;
            frik_visual_authority::FingerLocalTransformOverride exact{};
            exact.enabledMask = pose.fingerMask;
            std::copy(pose.fingerLocals.begin(), pose.fingerLocals.end(), std::begin(exact.localTransforms));
            constexpr const char* tag = "ROCK_Grab";
            constexpr int priority = 100;
            const char* blockTag = isLeft ? "ROCK_GrabPrimaryPoseLeft" : "ROCK_GrabPrimaryPoseRight";
            const bool blocked = frik_visual_authority::blockPrimaryHandWeaponPose(blockTag, true);
            const bool scalar = blocked &&
                frik_visual_authority::setHandPoseCustom(tag, handFromBool(isLeft),
                    frik_visual_authority::HandPoseData{}, priority);
            if (scalar &&
                frik_visual_authority::setHandPoseCustomLocalTransforms(tag, handFromBool(isLeft), &exact, priority)) {
                ROCK_LOG_INFO(Hand, "{} loose authored pose published role={} form={:08X}", isLeft ? "left" : "right",
                    static_cast<unsigned>(pose.role), pose.weaponFormId);
                return true;
            }
            (void)frik_visual_authority::clearHandPose(tag, handFromBool(isLeft));
            if (blocked)
                (void)frik_visual_authority::blockPrimaryHandWeaponPose(blockTag, false);
            return false;
        }

        /*
         * A resolved offset's finger pose is only meaningful for the loose-
         * weapon synthetic primary attach (pull-catch/force-grab): that path
         * has no mesh contact to solve fingers from, so it otherwise falls
         * back to a generic canned named pose (see
         * publishLooseWeaponPrimaryAttachHandPose / grabSelectedObject).
         */
        bool tryBuildSavedGrabOffsetFingerPose(const saved_grab_offset::HandOffset& handOffset,
            grab_finger_pose_runtime::SolvedGrabFingerPose& pose)
        {
            if (!handOffset.hasFingerPose) {
                return false;
            }

            pose.solved = true;
            pose.values = { handOffset.fingerValues[0], handOffset.fingerValues[1], handOffset.fingerValues[2],
                handOffset.fingerValues[3], handOffset.fingerValues[4] };
            pose.hasJointValues = handOffset.hasFingerJointValues;
            if (handOffset.hasFingerJointValues) {
                for (std::size_t i = 0; i < pose.jointValues.size(); ++i) {
                    pose.jointValues[i] = handOffset.fingerJointValues[i];
                }
            }
            return true;
        }

        void copyPeerInertiaSnapshot(SavedObjectState& target, const SavedObjectState& peer)
        {
            target.savedPackedInertia[0] = peer.savedPackedInertia[0];
            target.savedPackedInertia[1] = peer.savedPackedInertia[1];
            target.savedPackedInertia[2] = peer.savedPackedInertia[2];
            target.savedPackedMass = peer.savedPackedMass;
            target.inertiaModified = peer.inertiaModified;
            target.motionInertiaStates = peer.motionInertiaStates;
        }

        /*
         * Thumb and index keep the mesh solve, so they stop on the object's
         * surface instead of closing through it onto each other. The free
         * fingers tuck to the pinch curl unless the mesh stops them first.
         */
        void applyPinchFingerPosePolicy(grab_finger_pose_runtime::SolvedGrabFingerPose& pose,
            const CanonicalGrabFrame& frame)
        {
            const auto& tucked = frame.seat.pinchFingerPose;
            for (std::size_t finger = 2; finger < pose.values.size(); ++finger) {
                if (tucked.values[finger] <= pose.values[finger]) {
                    continue;
                }
                pose.values[finger] = tucked.values[finger];
                if (pose.hasJointValues) {
                    for (std::size_t segment = 0; segment < 3; ++segment) {
                        pose.jointValues[finger * 3 + segment] = tucked.jointValues[finger * 3 + segment];
                    }
                }
            }
        }

        bool rebaseFingerSkeletonSnapshot(root_flattened_finger_skeleton_runtime::Snapshot& snapshot,
            const RE::NiTransform& sourceHandWorld, const RE::NiTransform& targetHandWorld)
        {
            if (!snapshot.valid || !std::isfinite(sourceHandWorld.scale) ||
                std::abs(sourceHandWorld.scale) <= 0.000001f || !std::isfinite(targetHandWorld.scale) ||
                std::abs(targetHandWorld.scale) <= 0.000001f) {
                return false;
            }

            for (auto& finger : snapshot.fingers) {
                if (!finger.valid) {
                    return false;
                }
                for (auto& point : finger.points) {
                    point = transform_math::localPointToWorld(targetHandWorld,
                        transform_math::worldPointToLocal(sourceHandWorld, point));
                }
                if (finger.tipGeometryValid) {
                    finger.tipSegmentCenterWorld = transform_math::localPointToWorld(targetHandWorld,
                        transform_math::worldPointToLocal(sourceHandWorld, finger.tipSegmentCenterWorld));
                    finger.tipDirectionWorld = normalizeOrZero(transform_math::localVectorToWorld(targetHandWorld,
                        transform_math::worldVectorToLocal(sourceHandWorld, finger.tipDirectionWorld)));
                }
            }
            if (snapshot.palmNormalValid) {
                snapshot.palmNormalWorld = normalizeOrZero(transform_math::localVectorToWorld(targetHandWorld,
                    transform_math::worldVectorToLocal(sourceHandWorld, snapshot.palmNormalWorld)));
                if (lengthSquared(snapshot.palmNormalWorld) <= 0.000001f) {
                    return false;
                }
            }
            return true;
        }

        grab_finger_pose_runtime::GrabFingerPoseTargetSet rebuildFingerPoseTargetsFromGrabFrame(
            const CanonicalGrabFrame& frame, const RE::NiTransform& currentNodeWorld)
        {
            const RE::NiPoint3 seatNormalWorld =
                frame.gripEvidence.hasGripPoint ? gripEvidenceNormalWorld(frame, currentNodeWorld) : RE::NiPoint3{};
            auto targets = grab_finger_pose_runtime::makeSharedGripPoseTarget(
                gripEvidencePointWorld(frame, currentNodeWorld), seatNormalWorld);
            targets.useSeatPointForMissingTargets = false;
            targets.useWholeMeshForMissingTargets = true;
            for (std::size_t finger = 0; finger < frame.fingerPoseTargetLocal.size(); ++finger) {
                if (!frame.fingerPoseTargetValid[finger]) {
                    continue;
                }
                targets.targets[finger] =
                    transform_math::localPointToWorld(currentNodeWorld, frame.fingerPoseTargetLocal[finger]);
                targets.targetValid[finger] = 1;
                if (frame.fingerPoseTargetNormalValid[finger]) {
                    targets.targetNormals[finger] = normalizeOrZero(transform_math::localVectorToWorld(currentNodeWorld,
                        frame.fingerPoseTargetNormalLocal[finger]));
                    targets.targetNormalValid[finger] = 1;
                }
                ++targets.targetCount;
            }
            return targets;
        }

        constexpr const char* kGrabObjectRotationReferenceName = "generatedProxyAuthorityLocal";

        void applyFrozenGrabAuthorityFrameToGrabFrame(CanonicalGrabFrame& frame,
            const grab_authority_frame_math::FrozenGrabAuthorityFrame<RE::NiTransform>& frozen)
        {
            if (!frozen.valid) {
                return;
            }

            frame.rawHandSpace = frozen.rawHandSpace;
            frame.handBodyToRawHandAtGrab = frozen.handBodyToRawHandAtGrab;
            frame.proxyAuthorityHandSpace = frozen.proxyAuthorityHandSpace;
            frame.proxyAuthorityBodyHandSpace = frozen.proxyAuthorityBodyHandSpace;
            frame.authority.bodyLocal = frozen.bodyLocal;
            frame.authority.bodyWorldAtGrab = frozen.bodyWorldAtGrab;
            frame.rootBodyLocal = frozen.rootBodyLocal;
            frame.ownerBodyLocal = frozen.ownerBodyLocal;
            frame.gripEvidence.gripPointLocal = frozen.gripPointLocal;
            frame.authority.gripPointBodyLocalGame = frozen.pivotBBodyLocalGame;
            frame.authority.pivotBBodyLocalGame = frozen.pivotBBodyLocalGame;
            frame.authority.pivotBConstraintLocalGame = frozen.pivotBConstraintLocalGame;
            frame.authority.pivotAHandBodyLocalGame = frozen.pivotAHandBodyLocalGame;
            frame.authority.grabPivotWorldAtGrab = frozen.grabPivotWorldAtGrab;
            frame.gripEvidence.gripPointWorldAtGrab = frozen.gripPointWorldAtGrab;
            frame.authority.desiredObjectWorldAtGrab = frozen.desiredObjectWorld;
            frame.authority.desiredBodyWorldAtGrab = frozen.desiredBodyWorld;
            frame.authority.hasFrozenPivotB = true;
            frame.gripEvidence.hasGripPoint = true;
        }
    }

    void Hand::clearGrabHandCollisionSuppressionState() { _grabHandCollisionSuppression.clearTracking(); }

    void Hand::suppressHandCollisionForGrab(RE::hknpWorld* world)
    {
        /*
         * Held-object grabs suppress the grabbing hand's generated colliders
         * (palm and fingers) while the object is constrained to that hand. The
         * adjacent forearm and wrist body colliders lose only their pairs with
         * the held bodies (PhysicsInteraction::publishHeldBodyExclusion).
         */
        _grabHandCollisionSuppression.cancelDelayedRestore();

        if (!world || !hasCollisionBody())
            return;

        auto suppressBody = [&](std::uint32_t bodyId, const char* context) {
            if (bodyId == INVALID_BODY_ID) {
                return;
            }

            if (!_grabHandCollisionSuppression.contains(bodyId) && _grabHandCollisionSuppression.full()) {
                ROCK_LOG_WARN(Hand, "{} hand: grab collision suppression set full; bodyId={} context={} left active",
                    handName(), bodyId, context ? context : "unknown");
                return;
            }

            const auto registryResult = _grabHandCollisionSuppression.acquire(world, bodyId, context);

            if (registryResult.valid && (registryResult.filterChanged || registryResult.firstLeaseForBody)) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand: grab collision lease acquired bodyId={} context={} filter=0x{:08X}->0x{:08X} "
                    "wasDisabledBeforeGrab={} leases={}",
                    handName(), bodyId, context ? context : "unknown", registryResult.filterBefore,
                    registryResult.filterAfter, registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        };

        const std::uint32_t colliderCount = _boneColliders.getBodyCount();
        if (colliderCount > 0) {
            for (std::uint32_t i = 0; i < colliderCount; ++i) {
                suppressBody(_boneColliders.getBodyIdAtomic(i), "held-grab-hand-suite");
            }
        } else {
            suppressBody(_handBody.getBodyId().value, "held-grab-hand-anchor");
        }
    }

    void Hand::restoreHandCollisionAfterGrab(RE::hknpWorld* world)
    {
        if (_grabHandCollisionSuppression.empty())
            return;

        if (!world) {
            ROCK_LOG_WARN(Hand,
                "{} hand: cannot restore grab hand collision yet (world={}); preserving suppression state", handName(),
                static_cast<const void*>(world));
            return;
        }

        const bool restored = _grabHandCollisionSuppression.releaseAll(world, "held-grab-hand",
            [this](std::uint32_t bodyId,
                const collision_suppression_registry::RuntimeSuppressionResult& releaseResult) {
                if (releaseResult.readFailed) {
                    return;
                }
                ROCK_LOG_DEBUG(Hand,
                    "{} hand: grab hand collision lease released bodyId={} filter=0x{:08X}->0x{:08X} "
                    "restoreDisabled={} fullyReleased={}",
                    handName(), bodyId, releaseResult.filterBefore, releaseResult.filterAfter,
                    releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    releaseResult.bodyFullyReleased ? "yes" : "no");
            });
        if (!restored) {
            ROCK_LOG_WARN(Hand, "{} hand: grab hand collision restore deferred; suppression leases preserved",
                handName());
            return;
        }
        clearGrabHandCollisionSuppressionState();
    }

    void Hand::updateDelayedGrabHandCollisionRestore(RE::hknpWorld* world, float deltaTime)
    {
        if (!_grabHandCollisionSuppression.advanceDelayedRestore(deltaTime)) {
            return;
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand: delayed grab hand collision restore ready bodies={} firstBodyId={} delayRemaining={:.3f}",
            handName(), _grabHandCollisionSuppression.size(), _grabHandCollisionSuppression.firstBodyId(),
            _grabHandCollisionSuppression.delayedRestoreRemainingSeconds());
        restoreHandCollisionAfterGrab(world);
    }

    bool Hand::commitFrozenGrabAuthority(const GrabFrozenCommitInput& input)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabFrozenCommit);
        const auto& handWorldTransform = *input.handWorldTransform;
        const auto& proxy = *input.proxy;
        const auto& meshCapture = *input.meshCapture;
        const auto& bodyFrame = *input.bodyFrame;
        auto& seatCapture = *input.seatCapture;
        const auto& surface = *input.surface;
        const auto& proxyFrameWorldAtGrab = proxy.proxyFrameWorldAtGrab;
        const auto& proxyAuthorityFrameWorldAtGrab = proxy.proxyAuthorityFrameWorldAtGrab;
        const auto& objectWorldTransform = meshCapture.objectWorldTransform;
        const auto& grabBodyWorldAtGrab = bodyFrame.grabBodyWorld;
        const auto& constraintBodyWorldAtGrab = bodyFrame.constraintBodyWorld;
        const auto& rootBodyLocalAtGrab = bodyFrame.rootBodyLocal;
        const auto& ownerBodyLocalAtGrab = bodyFrame.ownerBodyLocal;
        const auto& grabPivotAWorld = bodyFrame.grabPivotAWorld;
        const auto& grabGripPoint = surface.gripPoint;
        const auto* grabPointMode = surface.pointMode;
        auto& desiredObjectWorld = seatCapture.desiredObjectWorld;
        auto& desiredBodyWorld = seatCapture.desiredBodyWorld;
        const auto resolvedAuthorityPivotSourceForFreeze = seatCapture.resolvedAuthoritySource;
        const auto visualHandObjectLocal = seatCapture.hasAuthoredHandRelation
            ? transform_math::composeTransforms(bodyFrame.objectToBody,
                  transform_math::composeTransforms(transform_math::invertTransform(rootBodyLocalAtGrab),
                      seatCapture.authoredHandWeaponLocal))
            : transform_math::makeIdentityTransform<RE::NiTransform>();
        const auto objectBodyId = input.objectBodyId;
        const auto grabTraceId = input.traceId;
        grab_authority_frame_math::FrozenGrabAuthorityFrame<RE::NiTransform> frozenAuthorityFrame{};
        const RE::NiPoint3 frozenVisualNormalWorld = gripEvidenceNormalWorld(_grabFrame, objectWorldTransform);
        frozenAuthorityFrame = grab_authority_frame_math::freezeGrabAuthorityFrame<RE::NiTransform>(
            grab_authority_frame_math::GrabAuthorityFrameFreezeInput<RE::NiTransform>{
                .rawHandWorld = handWorldTransform,
                .proxyWorld = proxyFrameWorldAtGrab,
                .proxyAuthorityFrameWorld = proxyAuthorityFrameWorldAtGrab,
                .objectWorld = objectWorldTransform,
                .bodyWorld = grabBodyWorldAtGrab,
                .constraintBodyWorld = constraintBodyWorldAtGrab,
                .rootBodyLocal = rootBodyLocalAtGrab,
                .ownerBodyLocal = ownerBodyLocalAtGrab,
                .desiredObjectWorld = desiredObjectWorld,
                .desiredBodyWorld = desiredBodyWorld,
                .visualHandObjectLocal = visualHandObjectLocal,
                .pivotAWorld = grabPivotAWorld,
                .gripPointWorld = grabGripPoint,
                .visualNormalWorld = frozenVisualNormalWorld,
                .source = resolvedAuthorityPivotSourceForFreeze,
                .hasDesiredObjectWorld = true,
                .hasDesiredBodyWorld = true,
                .hasVisualHandObjectLocal = seatCapture.hasAuthoredHandRelation,
                .visualNormalValid = lengthSquared(frozenVisualNormalWorld) > 0.000001f,
            });
        if (!frozenAuthorityFrame.valid) {
            ROCK_LOG_ERROR(Hand,
                "{} GRAB FAILED: unable to freeze coherent authority frame point=({:.2f},{:.2f},{:.2f}) "
                "pivotA=({:.2f},{:.2f},{:.2f}) mode={} bodyId={}",
                handName(), grabGripPoint.x, grabGripPoint.y, grabGripPoint.z, grabPivotAWorld.x, grabPivotAWorld.y,
                grabPivotAWorld.z, grabPointMode, objectBodyId.value);
            _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::Idle;
            _grabObjectGripAtGrab = {};
            _heldObjectIsLooseWeapon = false;
            _grabFrame.clear();
            _heldBodyIds.clear();
            _heldDriveDecision = {};
            _heldBodyIdsCount.store(0, std::memory_order_release);
            _grabFingerPosePublished = false;
            (void)frik_visual_authority::clearHandPose("ROCK_Grab", handFromBool(_isLeft));
            clearGrabExternalHandWorldTransform(_isLeft);
            input.rollback.rollback();
            return false;
        }
        applyFrozenGrabAuthorityFrameToGrabFrame(_grabFrame, frozenAuthorityFrame);
        desiredObjectWorld = frozenAuthorityFrame.desiredObjectWorld;
        desiredBodyWorld = frozenAuthorityFrame.desiredBodyWorld;
        const float objectScaleForLever = std::isfinite(objectWorldTransform.scale) && objectWorldTransform.scale > 0.0f
            ? objectWorldTransform.scale
            : 1.0f;
        _grabFrame.pivotAuthority.longLeverGameUnits =
            computeLocalMeshMaxDistanceFromPoint(_grabFrame.localMeshTriangles,
                _grabFrame.gripEvidence.gripPointLocal) *
            objectScaleForLever;
        _grabFrame.authority.liveHandWorldAtGrab = handWorldTransform;
        _grabFrame.authority.handBodyWorldAtGrab = proxyFrameWorldAtGrab;
        _grabFrame.authority.objectNodeWorldAtGrab = objectWorldTransform;
        _grabFrame.hasTelemetryCapture = true;
        _grabFrame.handScaleAtGrab = handWorldTransform.scale;
        _grabFrame.traceId = grabTraceId;
        _grabFrame.freezeCaptureTelemetry(objectBodyId.value);
        return true;
    }

    void Hand::initializePostFreezeGrab(RE::hknpWorld* world, const GrabPostFreezeInput& input)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabPostFreeze);
        const auto& handWorldTransform = *input.handWorldTransform;
        const auto& meshCapture = *input.meshCapture;
        const auto& bodyFrame = *input.bodyFrame;
        const auto& surface = *input.surface;
        const auto& sel = _currentSelection;
        const auto& objectWorldTransform = meshCapture.objectWorldTransform;
        const auto& palmPos = bodyFrame.palmPosition;
        const auto& grabPivotAWorld = bodyFrame.grabPivotAWorld;
        const auto& grabGripPoint = surface.gripPoint;
        const auto objectBodyId = input.objectBodyId;
        const auto& sharedContext = *input.sharedContext;
        const bool joiningPeerHeldObject = input.joiningPeerHeldObject;
        const bool looseWeaponGrab = input.looseWeaponGrab;
        clearGrabExternalHandWorldTransform(_isLeft);
        _grabVisualHandTransform = handWorldTransform;
        _hasGrabVisualHandTransform = false;
        _lastPublishedGrabVisualHandTransform = {};
        _hasLastPublishedGrabVisualHandTransform = false;
        _grabVisualHandLerpStartTransform = handWorldTransform;
        _grabVisualHandLerpElapsedSeconds = 0.0f;
        _grabVisualHandLerpDurationSeconds = 0.0f;
        const RE::NiPoint3 initialGrabDelta = grabPivotAWorld - grabGripPoint;
        const float initialGrabDistance = std::sqrt(initialGrabDelta.x * initialGrabDelta.x +
            initialGrabDelta.y * initialGrabDelta.y + initialGrabDelta.z * initialGrabDelta.z);
        const bool needsLargeInitialSync = initialGrabDistance >= g_rockConfig.rockGrabHandLerpMinDistance;
        _grabFrame.fadeInGrabConstraint = needsLargeInitialSync;
        if (needsLargeInitialSync) {
            _grabFrame.motorFadeReason = "largeInitialSync";
        } else {
            _grabFrame.motorFadeReason = "none";
        }
        _controllerReleaseHistory.clear();
        _objectReleaseHistory.clear();
        _releaseObjectNotBefore = release_velocity::sampleTimeSeconds();
        _releaseControllerFrame = 0;
        _previousHeldRawHandWorld = {};
        _previousHeldHandPositionHavok = {};
        _lastHeldHandPositionHavok = {};
        _hasPreviousHeldRawHandWorld = false;
        _hasLastHeldHandPositionHavok = false;
        _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);

        ROCK_LOG_DEBUG(Hand,
            "{} GRAB HAND SPACE: pos=({:.1f},{:.1f},{:.1f}) "
            "palmPos=({:.1f},{:.1f},{:.1f}) pivotA=({:.1f},{:.1f},{:.1f}) grabPt=({:.1f},{:.1f},{:.1f})",
            handName(), _grabFrame.rawHandSpace.translate.x, _grabFrame.rawHandSpace.translate.y,
            _grabFrame.rawHandSpace.translate.z, palmPos.x, palmPos.y, palmPos.z, grabPivotAWorld.x, grabPivotAWorld.y,
            grabPivotAWorld.z, grabGripPoint.x, grabGripPoint.y, grabGripPoint.z);
        ROCK_LOG_DEBUG(Hand, "{} BODY LOCAL: pos=({:.2f},{:.2f},{:.2f}) scale={:.3f}", handName(),
            _grabFrame.authority.bodyLocal.translate.x, _grabFrame.authority.bodyLocal.translate.y,
            _grabFrame.authority.bodyLocal.translate.z, _grabFrame.authority.bodyLocal.scale);

        {
            RE::NiTransform handBodyDiag{};
            RE::NiTransform objectBodyDiag{};
            const bool hasLiveDiag = tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), handBodyDiag) &&
                tryResolveLiveBodyWorldTransform(world, objectBodyId, objectBodyDiag);
            if (hasLiveDiag) {
                ROCK_LOG_TRACE(Hand,
                    "{} DIAG: handBodyLive pos=({:.1f},{:.1f},{:.1f}) objBodyLive pos=({:.1f},{:.1f},{:.1f})",
                    handName(), handBodyDiag.translate.x, handBodyDiag.translate.y, handBodyDiag.translate.z,
                    objectBodyDiag.translate.x, objectBodyDiag.translate.y, objectBodyDiag.translate.z);
            }
            ROCK_LOG_TRACE(Hand, "{} DIAG: handNi pos=({:.1f},{:.1f},{:.1f}) objNi pos=({:.1f},{:.1f},{:.1f})",
                handName(), handWorldTransform.translate.x, handWorldTransform.translate.y,
                handWorldTransform.translate.z, objectWorldTransform.translate.x, objectWorldTransform.translate.y,
                objectWorldTransform.translate.z);

            float comX, comY, comZ;
            if (getBodyCOMWorld(world, objectBodyId, comX, comY, comZ) && hasLiveDiag) {
                ROCK_LOG_TRACE(Hand, "{} B8 COM LIVE: comHk=({:.3f},{:.3f},{:.3f}) objBodyLive=({:.1f},{:.1f},{:.1f})",
                    handName(), comX, comY, comZ, objectBodyDiag.translate.x, objectBodyDiag.translate.y,
                    objectBodyDiag.translate.z);
            }
        }

        if (!grab_target::requiresNativeBodyOwnership(sel.targetKind)) {
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
            ROCK_LOG_WARN(Hand, "{} hand GRAB activation incomplete: primaryBody={} bodies={} activated={} failed={}",
                handName(), objectBodyId.value, grabActivation.bodyCount, grabActivation.activatedCount,
                grabActivation.failedActivationCount);
        }

        suppressHandCollisionForGrab(world);

        // Native ragdoll inertia belongs to its joints/solver.
        if (!grab_target::requiresNativeBodyOwnership(sel.targetKind)) {
            if (joiningPeerHeldObject && sharedContext.peerSavedObjectState) {
                copyPeerInertiaSnapshot(_savedObjectState, *sharedContext.peerSavedObjectState);
                ROCK_LOG_DEBUG(Hand,
                    "{} hand joined peer-held object inertia snapshot: formID={:08X} peerMotions={} inertiaModified={}",
                    handName(), sel.refr ? sel.refr->GetFormID() : 0,
                    sharedContext.peerSavedObjectState->motionInertiaStates.size(),
                    sharedContext.peerSavedObjectState->inertiaModified ? "yes" : "no");
            } else {
                normalizeGrabbedInertiaForBodies(world, objectBodyId, _heldBodyIds, _savedObjectState, looseWeaponGrab);
            }
        }
    }

    /*
     * The one mesh finger solve. Commit calls it when the seat is already
     * final; otherwise arrival at TouchHeld does. The live finger skeleton is
     * rebased from the live hand onto the hand pose being solved for.
     */
    /*
     * The one mesh finger solve. Commit calls it when the seat is already
     * final; otherwise arrival at TouchHeld does. The live finger skeleton is
     * rebased from the live hand onto the hand pose being solved for.
     *
     * Each finger is swept with per-bone capsules along FRIK's own finger
     * motion (GrabFingerCapsuleSweep.h) and published as FRIK joint values
     * only. The baked-curve solver, its pad-probe refinement and the
     * surface-aim local-transform layer were removed with it: bench replays
     * showed the baked curves leaving drawn fingers 1+ gu inside objects and
     * the local layer pushing them a further 0.1-0.6 gu in.
     */
    void Hand::solveAndPublishGrabFingerPose(const GrabFingerSolveFrame& frame)
    {
        const bool pinchFingerPose = _grabFrame.seat.mode == GrabSeatMode::PinchPocket;
        const auto& localFingerPoseTriangles = !_grabFrame.fingerPoseLocalMeshTriangles.empty()
            ? _grabFrame.fingerPoseLocalMeshTriangles
            : _grabFrame.localMeshTriangles;

        if (_grabFingerSweptArc) {
            /*
             * iLooseGrabFingerPoseMode = 1: the pre-capsule solve (finger pose
             * mode 1, Current), unchanged: frozen-mesh swept arc, pad-probe
             * refinement, surface-aim capture, joint publication now and the
             * local-transform layer on the next TouchHeld update. Not recorded
             * for bench replay (the replay record holds capsule solves).
             */
            const auto mode = grab_finger_pose_runtime::FingerPoseMode::Current;
            const auto fingerPoseTargets = rebuildFingerPoseTargetsFromGrabFrame(_grabFrame, frame.objectWorld);

            root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshot{};
            const bool liveFingerSnapshotValid =
                root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft,
                    liveFingerSnapshot) &&
                rebaseFingerSkeletonSnapshot(liveFingerSnapshot, frame.liveHandWorld, frame.solveHandWorld);
            if (!liveFingerSnapshotValid) {
                liveFingerSnapshot = {};
            }

            std::vector<TriangleData> worldTriangles;
            const grab_finger_pose_runtime::FrozenMeshFingerPoseSolveOptions solveOptions{
                .minValue = g_rockConfig.rockGrabFingerMinValue,
                .maxTriangleDistanceSquared = g_rockConfig.rockGrabMaxTriangleDistance,
                .rejectBacksideHits = g_rockConfig.rockGrabFingerRejectBacksideHits,
                .surfacePlaneToleranceGameUnits = g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits,
                .allowSurfaceAimTargets = _grabFrame.fingerPoseAimValid,
                .sweepContactRadiusGameUnits = g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits,
                .thumbSweepMaxOpenValue = g_rockConfig.rockGrabThumbSweepMaxOpenValue,
                .fingerSweepMaxOpenValue = g_rockConfig.rockGrabFingerSweepMaxOpenValue,
                .meshFingerPoseEnabled = g_rockConfig.rockGrabMeshFingerPoseEnabled,
                .captureSweepDebug = false,
                .mode = mode,
            };
            const auto frozenSolve = grab_finger_pose_runtime::solveFrozenMeshFingerPoseBase(localFingerPoseTriangles,
                frame.objectWorld, frame.solveHandWorld, _isLeft, frame.pivotWorld, fingerPoseTargets,
                _grabFingerTriangleIndex, worldTriangles, solveOptions, &liveFingerSnapshot);
            _grabFingerPose = frozenSolve.pose;
            _grabFingerPose.mode = mode;
            if (pinchFingerPose) {
                applyPinchFingerPosePolicy(_grabFingerPose, _grabFrame);
            }
            grab_finger_pose_runtime::useThumbIndexCurveOnlyPose(_grabFingerPose);
            const auto* padIndex = frozenSolve.spatialIndexBuilt ? &_grabFingerTriangleIndex : nullptr;
            std::array<grab_finger_pose_runtime::FingerPadSurfaceEvidence, 5> padCaptureEvidence{};
            (void)grab_finger_pose_runtime::refineGrabFingerPoseWithPadProbes(_grabFingerPose, worldTriangles,
                fingerPoseTargets, liveFingerSnapshot, frame.objectWorld, g_rockConfig.rockGrabMeshFingerPoseEnabled,
                true, padCaptureEvidence, true, {}, padIndex);
            grab_finger_pose_runtime::captureSurfaceAimObjectLocal(_grabFingerPose, frame.objectWorld);

            auto publishFingerPose =
                grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(_grabFingerPose, frame.objectWorld);
            if (!frame.touchHeld) {
                publishFingerPose = buildAcquisitionFingerPose(publishFingerPose, 0.0f);
            }
            applySweptArcGrabHandPose(_isLeft, publishFingerPose, _grabFingerJointPose, _hasGrabFingerJointPose,
                _grabFingerLocalTransforms, _grabFingerLocalTransformMask, _hasGrabFingerLocalTransforms, 0.0f,
                /*publishLocalTransforms=*/false, frame.touchHeld);
            _grabFingerPosePublished = true;
            _grabFingerLocalTransformFinalizePending = frame.touchHeld && _grabFingerPose.solved;
            ROCK_LOG_DEBUG(Hand,
                "{} THREE-PHASE GRAB POSE SOLVED (swept arc): stage={} phase={} pinch={} solved={} hits={} "
                "triangles={} spatial={} nodes={} tests={} commandedAnchors={}",
                handName(), frame.stage, grab_three_phase::phaseName(_grabAcquisitionPhase),
                pinchFingerPose ? "yes" : "no", _grabFingerPose.solved ? "yes" : "no", _grabFingerPose.hitCount,
                _grabFingerPose.candidateTriangleCount, _grabFingerPose.usedSpatialIndex ? "yes" : "no",
                _grabFingerPose.spatialNodeVisitCount, _grabFingerPose.spatialTriangleTestCount,
                frozenSolve.commandedOpenDirectionsValid ? "yes" : "no");
            return;
        }

        root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshot{};
        const bool liveFingerSnapshotValid =
            root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft, liveFingerSnapshot) &&
            grab_finger_capsule_sweep::rebaseSkeleton(liveFingerSnapshot, frame.liveHandWorld, frame.solveHandWorld);

        grab_finger_capsule_sweep::FrikFingerRig rig{};
        const bool rigValid = liveFingerSnapshotValid && grab_finger_capsule_sweep::resolveFrikFingerRig(_isLeft, rig);
        const bool indexBuilt = _grabFingerTriangleIndex.buildFromLocalTriangles(localFingerPoseTriangles);
        const grab_finger_capsule_sweep::Options options{
            .minValue = g_rockConfig.rockGrabFingerMinValue,
            .fingerMaxOpenValue = g_rockConfig.rockGrabFingerSweepMaxOpenValue,
            .thumbMaxOpenValue = g_rockConfig.rockGrabThumbSweepMaxOpenValue,
        };
        grab_finger_capsule_sweep::Result capsule{};
        if (rigValid && indexBuilt) {
            capsule = grab_finger_capsule_sweep::solveForHand(rig, frame.solveHandWorld, liveFingerSnapshot,
                _grabFingerTriangleIndex, frame.objectWorld, options);
            _grabFingerPose = grab_finger_capsule_sweep::toSolvedPose(capsule,
                static_cast<std::uint32_t>(localFingerPoseTriangles.size()));
            if (g_rockConfig.rockDebugTestbenchTelemetry) {
                beginGrabPoseReplay(frame, localFingerPoseTriangles, liveFingerSnapshot, capsule, options);
            }
        } else {
            // Fail closed: an unsolved pose publishes ROCK's neutral fallback grip.
            _grabFingerPose = {};
            ROCK_LOG_WARN(Hand, "{} GRAB FINGER POSE: not solved skeleton={} frikRig={} meshIndex={} triangles={}",
                handName(), liveFingerSnapshotValid, rigValid, indexBuilt, localFingerPoseTriangles.size());
        }
        if (pinchFingerPose && _grabFingerPose.solved) {
            applyPinchFingerPosePolicy(_grabFingerPose, _grabFrame);
        }

        auto publishFingerPose = _grabFingerPose;
        if (!frame.touchHeld) {
            publishFingerPose = buildAcquisitionFingerPose(publishFingerPose, 0.0f);
        }
        applyRockGrabHandPose(_isLeft, publishFingerPose, _grabFingerJointPose, _hasGrabFingerJointPose,
            _grabFingerLocalTransforms, _grabFingerLocalTransformMask, _hasGrabFingerLocalTransforms, 0.0f,
            frame.touchHeld);
        _grabFingerPosePublished = true;
        if (frame.touchHeld && _grabFingerPose.solved) {
            _grabPoseQualityUpdatesRemaining = g_rockConfig.rockDebugTestbenchTelemetry ? 60 : -1;
        }
        ROCK_LOG_DEBUG(
            Hand,
            "{} THREE-PHASE GRAB POSE SOLVED: stage={} phase={} pinch={} solved={} contacts=0x{:02X} "
            "curls=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) "
            "openedToClear=0x{:02X} blocked=0x{:02X} triangles={} sphereQueries={} tests={}",
            handName(), frame.stage, grab_three_phase::phaseName(_grabAcquisitionPhase), pinchFingerPose ? "yes" : "no",
            _grabFingerPose.solved ? "yes" : "no", _grabFingerPose.contactValidMask, _grabFingerPose.values[0],
            _grabFingerPose.values[1], _grabFingerPose.values[2], _grabFingerPose.values[3], _grabFingerPose.values[4],
            [&] {
                std::uint32_t m = 0;
                for (std::size_t f = 0; f < 5; ++f)
                    m |= capsule.fingers[f].openedToClear ? (1u << f) : 0u;
                return m;
            }(),
            [&] {
                std::uint32_t m = 0;
                for (std::size_t f = 0; f < 5; ++f)
                    m |= capsule.fingers[f].blocked ? (1u << f) : 0u;
                return m;
            }(),
            localFingerPoseTriangles.size(), capsule.sphereQueries, capsule.stats.triangleTests);
    }

    bool Hand::commitGrabConstraintAndPose(RE::hknpWorld* world, const GrabConstraintCommitInput& input)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabConstraintCommit);
        const auto& handWorldTransform = *input.handWorldTransform;
        const auto& proxy = *input.proxy;
        const auto& surface = *input.surface;
        auto* bhkWorld = input.bhkWorld;
        const auto objectBodyId = input.objectBodyId;
        auto& activeLifecycle = *input.activeLifecycle;
        const auto& savedGrabOffset = *input.savedGrabOffset;
        const bool hasSavedGrabOffset = input.hasSavedGrabOffset;
        const bool joiningPeerHeldObject = input.joiningPeerHeldObject;
        const bool looseWeaponGrab = input.looseWeaponGrab;
        const float tau = input.tau;
        const float damping = input.damping;
        const float maxForce = input.maxForce;
        const float proportionalRecovery = input.proportionalRecovery;
        const float constantRecovery = input.constantRecovery;
        const auto& sel = _currentSelection;
        const auto& proxyFrameWorldAtGrab = proxy.proxyFrameWorldAtGrab;
        const auto& grabGripPoint = surface.gripPoint;
        const bool meshGrabFound = surface.meshGrabFound;
        const char* grabPointMode = surface.pointMode;
        const char* grabFallbackReason = surface.fallbackReason;
        _grabOffsetAcquisition = {};
        _looseSharedAcquisition = {};
        _looseSharedAcquisitionPeer = 0;
        _looseSharedAcquisitionFrame = 0;
        _looseSharedGunstockAim = false;
        _looseWeaponReturnStartFrame = 0;
        _looseSharedHandBlendAlpha = -1.0f;
        _grabOffsetMaximumGripError = 0.0f;
        _grabOffsetMaximumRotationError = 0.0f;
        const bool authoredPoseReady = !_grabFrame.authoredWeaponPose.valid() ||
            publishLooseWeaponPrimaryAttachHandPose(_isLeft, sel.refr, _grabFrame.authoredWeaponPose);
        if (!authoredPoseReady) {
            ROCK_LOG_WARN(Hand, "{} authored weapon pose rejected before constraint commit", handName());
        } else if (grab_target::requiresNativeBodyOwnership(sel.targetKind) &&
            (!surface.surfaceHit.valid || !surface.surfaceHit.sourceShape ||
                surface.surfaceHit.sourceNode != _grabFrame.heldNode)) {
            // Pocket refinement can choose another surface after primary
            // resolution. Never commit that point to a different limb.
            ROCK_LOG_WARN(Hand, "{} ragdoll final surface rejected: body={} source='{}' owner='{}' valid={}",
                handName(), objectBodyId.value, nodeDebugName(surface.surfaceHit.sourceNode),
                nodeDebugName(_grabFrame.heldNode), surface.surfaceHit.valid);
        } else {
            RE::NiPoint3 grabPivotAWorld = _grabFrame.hasTelemetryCapture
                ? _grabFrame.authority.grabPivotWorldAtGrab
                : computeGrabPivotAWorld(world, handWorldTransform);
            const float gameToHkScale = gameToHavokScale();
            const RE::NiTransform initialDesiredBodyWorld = _grabFrame.authority.desiredBodyWorldAtGrab;

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
                float pivotAToGrab = std::sqrt((pivotAHk[0] - gripWorldHk[0]) * (pivotAHk[0] - gripWorldHk[0]) +
                    (pivotAHk[1] - gripWorldHk[1]) * (pivotAHk[1] - gripWorldHk[1]) +
                    (pivotAHk[2] - gripWorldHk[2]) * (pivotAHk[2] - gripWorldHk[2]));
                float pivotAToHandOrigin = std::sqrt((grabPivotAWorld.x - handWorldTransform.translate.x) *
                        (grabPivotAWorld.x - handWorldTransform.translate.x) +
                    (grabPivotAWorld.y - handWorldTransform.translate.y) *
                        (grabPivotAWorld.y - handWorldTransform.translate.y) +
                    (grabPivotAWorld.z - handWorldTransform.translate.z) *
                        (grabPivotAWorld.z - handWorldTransform.translate.z));
                ROCK_LOG_DEBUG(Hand,
                    "GRAB DIAG {}: handPos=({:.1f},{:.1f},{:.1f}) "
                    "pocket=({:.1f},{:.1f},{:.1f}) grip=({:.1f},{:.1f},{:.1f}) meshGrab={} grabPointMode={} "
                    "fallbackReason={} "
                    "frozenPivotB=({:.2f},{:.2f},{:.2f}) activePoint={} palmSeatPoint={} "
                    "pivotAToGrab_hk={:.4f} ({:.1f} game units) pivotAToHandOrigin={:.1f} game units "
                    "selectionToGripEvidence={:.1f}",
                    handName(), handWorldTransform.translate.x, handWorldTransform.translate.y,
                    handWorldTransform.translate.z, grabPivotAWorld.x, grabPivotAWorld.y, grabPivotAWorld.z,
                    grabGripPoint.x, grabGripPoint.y, grabGripPoint.z, meshGrabFound, grabPointMode, grabFallbackReason,
                    _grabFrame.authority.pivotBBodyLocalGame.x, _grabFrame.authority.pivotBBodyLocalGame.y,
                    _grabFrame.authority.pivotBBodyLocalGame.z, _grabFrame.seat.activeGrabPointMode,
                    _grabFrame.seat.palmSeatPointMode, pivotAToGrab, pivotAToGrab * havokToGameScale(),
                    pivotAToHandOrigin, _grabFrame.pivotAuthority.selectionDistanceGameUnits);
            }

            const char* driveReason =
                joiningPeerHeldObject ? "joining-peer-held-loose-object" : "ordinary-dynamic-loose-object";
            RE::NiTransform initialProxyWorld = proxyFrameWorldAtGrab;
            RE::NiPoint3 initialConstraintPivotWorld = grabPivotAWorld;
            // A local grasp seat can turn the object up to a quarter turn
            // about its grip. Ramp the target there like the other seats
            // that rotate at capture; the visual hand stays on the
            // controller until the ramp ends.
            if (!joiningPeerHeldObject && sel.targetKind == grab_target::Kind::LooseObject &&
                (_grabFrame.syntheticLooseWeaponPrimaryAttach || sel.forcedArrival || _grabFrame.seat.localGraspSeat)) {
                _grabOffsetAcquisition = looseWeaponGrab
                    ? (g_rockConfig.rockWeaponSupportGripHandLerpEnabled
                              ? grab_offset_acquisition::beginWeapon(proxyFrameWorldAtGrab,
                                    _grabFrame.authority.bodyWorldAtGrab, _grabFrame.proxyAuthorityBodyHandSpace,
                                    _grabFrame.authority.pivotBConstraintLocalGame, looseWeaponAcquireConfig())
                              : grab_offset_acquisition::Transition<RE::NiTransform>{})
                    : grab_offset_acquisition::begin(proxyFrameWorldAtGrab, _grabFrame.authority.bodyWorldAtGrab,
                          _grabFrame.proxyAuthorityBodyHandSpace, _grabFrame.authority.pivotBConstraintLocalGame);
                if (_grabOffsetAcquisition.active) {
                    initialProxyWorld = grab_offset_acquisition::advance(_grabOffsetAcquisition, proxyFrameWorldAtGrab,
                        _grabFrame.proxyAuthorityBodyHandSpace, _grabFrame.authority.pivotBConstraintLocalGame, 0.0f);
                    initialConstraintPivotWorld = generatedProxyLocalPointToWorld(initialProxyWorld,
                        _grabFrame.authority.pivotAHandBodyLocalGame);
                    // The initial constraint is already satisfied. Ramp its
                    // target, not its force budget, to avoid letting the
                    // body sag while a second startup fade is running.
                    _grabFrame.fadeInGrabConstraint = false;
                    _grabFrame.motorFadeReason = "offsetAcquisitionTarget";
                    const auto initialGrip = transform_math::localPointToWorld(_grabFrame.authority.bodyWorldAtGrab,
                        _grabFrame.authority.pivotBConstraintLocalGame);
                    ROCK_LOG_INFO(Hand,
                        "{} OFFSET_ACQUIRE begin trace={} ref={:08X} source={} gripCorrection={:.3f}gu "
                        "rotationCorrection={:.3f}deg duration={:.3f}s",
                        handName(), _grabFrame.traceId, sel.refr ? sel.refr->GetFormID() : 0, grabFallbackReason,
                        pointDistanceGameUnits(initialGrip, grabPivotAWorld),
                        grab_offset_acquisition::rotationAngleRadians(_grabFrame.authority.bodyWorldAtGrab.rotate,
                            initialDesiredBodyWorld.rotate) *
                            57.295779513f,
                        _grabOffsetAcquisition.durationSeconds);
                }
            }
            if (!createProxyConstraintGrabDrive(bhkWorld, world, objectBodyId, initialProxyWorld, handWorldTransform,
                    initialConstraintPivotWorld, tau, damping, maxForce,
                    held_object_drive_policy::sanitizeMotorAuthorityScale(
                        sharedGrabAuthorityForceScale(joiningPeerHeldObject)),
                    proportionalRecovery, constantRecovery, looseWeaponGrab, driveReason)) {
                ROCK_LOG_ERROR(Hand,
                    "{} hand GRAB FAILED: proxy constraint creation failed bodyId={} targetBody=({:.2f},{:.2f},{:.2f}) "
                    "pivotBConstraint=({:.2f},{:.2f},{:.2f}) joiningPeer={}",
                    handName(), objectBodyId.value, initialDesiredBodyWorld.translate.x,
                    initialDesiredBodyWorld.translate.y, initialDesiredBodyWorld.translate.z,
                    activeProxyConstraintPivotBLocalGame().x, activeProxyConstraintPivotBLocalGame().y,
                    activeProxyConstraintPivotBLocalGame().z, joiningPeerHeldObject ? "yes" : "no");
            }
        }

        const bool driveCreated = _activeConstraint.isValid() && _grabAuthorityProxy.isValid();
        if (!driveCreated) {
            if (_grabFrame.authoredWeaponPose.valid()) {
                (void)frik_visual_authority::clearHandPose("ROCK_Grab", handFromBool(_isLeft));
                (void)frik_visual_authority::blockPrimaryHandWeaponPose(
                    _isLeft ? "ROCK_GrabPrimaryPoseLeft" : "ROCK_GrabPrimaryPoseRight", false);
            }
            ROCK_LOG_ERROR(Hand, "{} hand GRAB FAILED: proxy-constraint dynamic grab creation failed", handName());
            destroyGrabAuthorityProxy(bhkWorld);
            clearGrabExternalHandWorldTransform(_isLeft);
            if (!joiningPeerHeldObject) {
                restoreGrabbedInertia(world, _savedObjectState);
            }
            input.rollback.rollback();
            restoreHandCollisionAfterGrab(world);
            _savedObjectState.clear();
            _heldBodyIds.clear();
            _heldDriveDecision = {};
            _heldObjectIsLooseWeapon = false;
            return false;
        }

        _ragdollGrabOwner.reset(grab_target::requiresNativeBodyOwnership(sel.targetKind)
                ? havok_runtime::getCollisionObjectFromBody(world, _savedObjectState.bodyId)
                : nullptr);
        _ragdollGrabSurface.reset(
            grab_target::requiresNativeBodyOwnership(sel.targetKind) ? input.surface->surfaceHit.sourceShape : nullptr);
        _ragdollGrabTriangle = input.surface->surfaceHit.sourceTriangleIndex;
        _heldObjectIsLooseWeapon = looseWeaponGrab;
        const auto massSummaryAtGrab = readHeldBodyMassSummary(world, _savedObjectState.bodyId, _heldBodyIds,
            _heldDriveDecision.includeConnectedMass);
        const float effectiveMassAtGrab = effectiveGrabMotorMass(massSummaryAtGrab.motorMass());
        const float sharedLinearForce = _activeConstraint.linearMotor
            ? (std::max)(std::fabs(_activeConstraint.linearMotor->minForce),
                  std::fabs(_activeConstraint.linearMotor->maxForce))
            : maxForce;
        const float sharedAngularForce = _activeConstraint.angularMotor
            ? (std::max)(std::fabs(_activeConstraint.angularMotor->minForce),
                  std::fabs(_activeConstraint.angularMotor->maxForce))
            : 0.0f;
        ROCK_LOG_DEBUG(Hand,
            "{} hand dynamic grab created: drive={} bodyDriveMode={} driveReason={} forceShare={:.2f} linearScope={} "
            "angularScope={} massScope={} looseWeapon={} constraint={} proxyBody={} handBody={} objBody={} "
            "heldBodies={} mass={:.2f} effectiveMotorMass={:.2f} primaryMass={:.2f} massBodies={} motions={} "
            "longLever={:.1f}gu linearTau={:.3f} angularTau={:.3f} linearDamping={:.2f} angularDamping={:.2f} "
            "linearForce={:.0f} angularForce={:.0f} propRecov={:.1f} constRecov={:.1f} rotRef={}",
            handName(), kHeldObjectDriveName, held_object_drive_policy::modeName(_heldDriveDecision.mode),
            _heldDriveDecision.reason, sharedGrabAuthorityForceScale(joiningPeerHeldObject),
            _heldDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
            _heldDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
            _heldDriveDecision.includeConnectedMass ? "bodySet" : "primaryOnly",
            _heldObjectIsLooseWeapon ? "yes" : "no", _activeConstraint.constraintId,
            _grabAuthorityProxy.isValid() ? _grabAuthorityProxy.getBodyId().value : INVALID_BODY_ID,
            _handBody.getBodyId().value, objectBodyId.value, _heldBodyIds.size(), massSummaryAtGrab.motorMass(),
            effectiveMassAtGrab, massSummaryAtGrab.primaryMass, massSummaryAtGrab.sampledBodies,
            massSummaryAtGrab.uniqueMotions, _grabFrame.pivotAuthority.longLeverGameUnits,
            _activeConstraint.linearMotor ? _activeConstraint.linearMotor->tau : tau,
            _activeConstraint.angularMotor ? _activeConstraint.angularMotor->tau : g_rockConfig.rockGrabAngularTau,
            _activeConstraint.linearMotor ? _activeConstraint.linearMotor->damping : damping,
            _activeConstraint.angularMotor ? _activeConstraint.angularMotor->damping
                                           : g_rockConfig.rockGrabAngularDamping,
            sharedLinearForce, sharedAngularForce,
            _activeConstraint.linearMotor ? _activeConstraint.linearMotor->proportionalRecoveryVelocity
                                          : proportionalRecovery,
            _activeConstraint.linearMotor ? _activeConstraint.linearMotor->constantRecoveryVelocity : constantRecovery,
            kGrabObjectRotationReferenceName);

        const auto heldFlagLeases = acquireHeldObjectBodyFlagLeases(world, _savedObjectState.bodyId.value, _heldBodyIds,
            _savedObjectState.targetKind, heldBodyFlagLeaseOwner(this));
        if (heldFlagLeases.failedLeaseCount > 0) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB held body flag lease incomplete: primaryBody={} bodies={} collision={} authority={} "
                "failed={}",
                handName(), _savedObjectState.bodyId.value, heldFlagLeases.bodyCount,
                heldFlagLeases.collisionLeaseCount, heldFlagLeases.authorityLeaseCount,
                heldFlagLeases.failedLeaseCount);
        } else {
            ROCK_LOG_DEBUG(Hand,
                "{} hand GRAB held body flag leases acquired: primaryBody={} bodies={} collision={} authority={}",
                handName(), _savedObjectState.bodyId.value, heldFlagLeases.bodyCount,
                heldFlagLeases.collisionLeaseCount, heldFlagLeases.authorityLeaseCount);
        }
        clearHeldBodyContactSnapshot();
        _activeGrabLifecycle = std::move(activeLifecycle);

        publishHeldBodyScope(world);
        _isHoldingFlag.store(true, std::memory_order_release);

        stopSelectionHighlight();
        clearSelectedCloseFingerPose();
        _grabFingerPose = {};
        _grabFingerTriangleIndex.clear();
        _grabFingerSweptArc = g_rockConfig.rockLooseGrabFingerPoseMode == 1;
        _grabFingerLocalTransformFinalizePending = false;
        const bool useLooseWeaponPrimaryAttachHandPose = _grabFrame.syntheticLooseWeaponPrimaryAttach;
        _hasGrabFingerPose = g_rockConfig.rockGrabMeshFingerPoseEnabled && !useLooseWeaponPrimaryAttachHandPose;
        _grabFingerPosePublished = false;
        if (useLooseWeaponPrimaryAttachHandPose) {
            grab_finger_pose_runtime::SolvedGrabFingerPose savedGrabFingerPose{};
            const bool fixedAidPose = g_rockConfig.rockImmersiveAidEnabled &&
                immersive_aid::usesStimpakPose(
                    immersive_aid::classify(sel.refr ? sel.refr->GetObjectReference() : nullptr));
            const bool hasSavedGrabFingerPose = (fixedAidPose || g_rockConfig.rockGrabMeshFingerPoseEnabled) &&
                hasSavedGrabOffset && tryBuildSavedGrabOffsetFingerPose(savedGrabOffset, savedGrabFingerPose);
            if (_grabFrame.authoredWeaponPose.valid()) {
                _grabFingerPosePublished = authoredPoseReady;
            } else if (hasSavedGrabFingerPose) {
                /*
                     * Reuse the same saved offset that established the object
                     * attach. This synthetic attach has no mesh contact
                     * to solve fingers from, so without the resolved snapshot it
                     * would fall back to a generic canned pose. This is a one-time
                     * publish; mesh-resolve paths must not mutate it afterward.
                     */
                _grabFingerPose = savedGrabFingerPose;
                applyRockGrabHandPose(_isLeft, _grabFingerPose, _grabFingerJointPose, _hasGrabFingerJointPose,
                    _grabFingerLocalTransforms, _grabFingerLocalTransformMask, _hasGrabFingerLocalTransforms, 0.0f);
                _grabFingerPosePublished = true;
                if (savedGrabOffset.hasExactFingerRotations) {
                    frik_visual_authority::FingerLocalTransformOverride exact{};
                    const auto handPose =
                        frik_visual_authority::makeHandPoseDataFromJointValues(savedGrabOffset.fingerJointValues);
                    bool applied = frik_visual_authority::getHandPoseLocalTransformsForPose(handFromBool(_isLeft),
                                       handPose, &exact) &&
                        exact.enabledMask == 0x7fff;
                    if (applied) {
                        for (std::size_t joint = 0; joint < 15; ++joint) {
                            for (std::size_t row = 0; row < 3; ++row) {
                                for (std::size_t column = 0; column < 3; ++column) {
                                    exact.localTransforms[joint].rotate.entry[row][column] =
                                        savedGrabOffset.exactFingerRotations[joint][row * 3 + column];
                                }
                            }
                        }
                        applied = frik_visual_authority::setHandPoseCustomLocalTransforms("ROCK_Grab",
                            handFromBool(_isLeft), &exact, 100);
                    }
                    if (applied) {
                        for (std::size_t joint = 0; joint < 15; ++joint)
                            _grabFingerLocalTransforms[joint] = exact.localTransforms[joint];
                        _grabFingerLocalTransformMask = exact.enabledMask;
                        _hasGrabFingerLocalTransforms = true;
                        ROCK_LOG_INFO(Hand, "{} saved exact grab pose applied: 15 normal-rig joints", handName());
                    } else {
                        // Never retain the scalar seed as if the exact pose
                        // succeeded. Existing release cleanup owns this tag.
                        (void)frik_visual_authority::clearHandPose("ROCK_Grab", handFromBool(_isLeft));
                        _grabFingerPosePublished = false;
                        ROCK_LOG_WARN(Hand,
                            "{} saved exact grab pose unavailable: FRIK rejected the normal-rig transform override",
                            handName());
                    }
                }
                ROCK_LOG_INFO(Hand, "{} hand loose weapon attach: applying savedGrabOffset finger pose", handName());
            } else {
                _grabFingerPosePublished =
                    publishLooseWeaponPrimaryAttachHandPose(_isLeft, sel.refr, _grabFrame.authoredWeaponPose);
                if (!_grabFingerPosePublished) {
                    ROCK_LOG_WARN(Hand, "{} hand loose weapon attach: failed to publish FRIK weapon hand pose",
                        handName());
                }
            }
        } else if (_hasGrabFingerPose) {
            ROCK_LOG_INFO(Hand, "{} dynamic finger pose solver={} trace={}", handName(),
                _grabFingerSweptArc ? "1 swept arc" : "0 capsule sweep", _grabFrame.traceId);
            const bool pinchFingerPose = _grabFrame.seat.mode == GrabSeatMode::PinchPocket;
            const bool touchHeldAtCommit = _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld;
            const bool deferFingerPoseUntilSettledTouch =
                !pinchFingerPose && !touchHeldAtCommit && _grabFrame.seat.requiresSettledVisualHandRelation;
            if (pinchFingerPose && !touchHeldAtCommit) {
                ROCK_LOG_DEBUG(Hand,
                    "{} THREE-PHASE GRAB POSE: contact-time solve deferred until TouchHeld phase={} cachedTriangles={}",
                    handName(), grab_three_phase::phaseName(_grabAcquisitionPhase),
                    _grabFrame.fingerPoseLocalMeshTriangles.empty() ? _grabFrame.localMeshTriangles.size()
                                                                    : _grabFrame.fingerPoseLocalMeshTriangles.size());
            } else if (deferFingerPoseUntilSettledTouch) {
                /*
                     * A programmatic arrival whose pivot still needs seated
                     * promotion has no final object-hand relation yet. Publish
                     * only the neutral pre-close pose during approach. The
                     * promotion path solves once against the final live seat,
                     * after the object reaches the hand.
                     */
                const grab_finger_pose_runtime::SolvedGrabFingerPose pendingTarget{};
                const auto acquisitionFingerPose = buildAcquisitionFingerPose(pendingTarget, 0.0f);
                if (_grabFingerSweptArc) {
                    applySweptArcGrabHandPose(_isLeft, acquisitionFingerPose, _grabFingerJointPose,
                        _hasGrabFingerJointPose, _grabFingerLocalTransforms, _grabFingerLocalTransformMask,
                        _hasGrabFingerLocalTransforms, 0.0f, /*publishLocalTransforms=*/false,
                        /*snapJointPoseToTarget=*/true);
                } else {
                    applyRockGrabHandPose(_isLeft, acquisitionFingerPose, _grabFingerJointPose, _hasGrabFingerJointPose,
                        _grabFingerLocalTransforms, _grabFingerLocalTransformMask, _hasGrabFingerLocalTransforms, 0.0f,
                        /*snapJointPoseToTarget=*/true);
                }
                ROCK_LOG_DEBUG(Hand,
                    "{} THREE-PHASE GRAB POSE: final mesh solve deferred until seated TouchHeld phase={} "
                    "cachedTriangles={}",
                    handName(), grab_three_phase::phaseName(_grabAcquisitionPhase),
                    _grabFrame.fingerPoseLocalMeshTriangles.empty() ? _grabFrame.localMeshTriangles.size()
                                                                    : _grabFrame.fingerPoseLocalMeshTriangles.size());
            } else {
                // The seat is final: solve once against the commanded relation.
                solveAndPublishGrabFingerPose(GrabFingerSolveFrame{
                    .objectWorld = _grabFrame.authority.desiredObjectWorldAtGrab,
                    .liveHandWorld = handWorldTransform,
                    .solveHandWorld = handWorldTransform,
                    .pivotWorld = _grabFrame.hasTelemetryCapture ? _grabFrame.authority.grabPivotWorldAtGrab
                                                                 : computeGrabPivotAWorld(world, handWorldTransform),
                    .touchHeld = touchHeldAtCommit,
                    .stage = "commit",
                });
            }
        }

        return true;
    }
}
