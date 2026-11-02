// Hand::releaseGrabbedObject: ends a grab. Computes and applies the throw
// velocity from the recent hand and object motion, restores the object's
// bodies, flags and hand collision, removes the grab authority proxy and starts
// the hand's visual return.
#include "physics-interaction/hand/HandGrabInternal.h"

namespace rock
{

    namespace
    {
        const char* releaseDispositionName(GrabReleaseDisposition disposition) noexcept
        {
            switch (disposition) {
            case GrabReleaseDisposition::PhysicalDrop:
                return "physical-drop";
            case GrabReleaseDisposition::PendingInventoryTransfer:
                return "pending-inventory-transfer";
            case GrabReleaseDisposition::TransferToInventory:
                return "transfer-to-inventory";
            case GrabReleaseDisposition::PendingConsumeTransfer:
                return "pending-consume-transfer";
            case GrabReleaseDisposition::OwnershipHandoff:
                return "ownership-handoff";
            }
            return "unknown";
        }

        RE::NiPoint3 clampAngularVelocityVector(const RE::NiPoint3& value, float maxRadiansPerSecond)
        {
            if (!std::isfinite(maxRadiansPerSecond) || maxRadiansPerSecond <= 0.0f) {
                return RE::NiPoint3{};
            }

            const float magnitude = vectorMagnitude(value);
            if (!std::isfinite(magnitude) || magnitude <= 0.000001f) {
                return RE::NiPoint3{};
            }
            if (magnitude <= maxRadiansPerSecond) {
                return value;
            }

            const float scale = maxRadiansPerSecond / magnitude;
            return RE::NiPoint3{ value.x * scale, value.y * scale, value.z * scale };
        }
    }

    GrabReleaseOutcome Hand::releaseGrabbedObject(RE::hknpWorld* world,
        GrabReleaseCollisionRestoreMode collisionRestoreMode, const GrabReleaseContext& releaseContext,
        const std::source_location& caller)
    {
        GrabReleaseOutcome outcome{};
        outcome.finalObjectRelease = releaseContext.finalObjectRelease;
        if (!isHolding()) {
            return outcome;
        }

        if (releaseContext.finalObjectRelease && _heldObjectIsLooseWeapon && world) {
            auto* physics = PhysicsInteraction::s_instance.load(std::memory_order_acquire);
            if (physics &&
                !physics->prepareLooseWeaponRelease(getHeldRef(), world,
                    releaseContext.disposition == GrabReleaseDisposition::PhysicalDrop)) {
                ROCK_LOG_SAMPLE_WARN(Hand, 2000, "Weapon release waiting for native collision restoration");
                return outcome;
            }
        }

        held_scene_presentation::clearHeldBodies(_isLeft);

        outcome.released = true;
        outcome.retainedRef = _savedObjectState.retainedRef;
        outcome.refr = outcome.retainedRef.get();
        outcome.formID = outcome.refr ? outcome.refr->GetFormID() : 0;

        const std::string_view callerPath{ caller.file_name() };
        const auto separator = callerPath.find_last_of("/\\");
        const auto callerFile = separator == std::string_view::npos ? callerPath : callerPath.substr(separator + 1);
        ROCK_LOG_INFO(Hand,
            "{} hand RELEASE: bodyId={} constraintId={} proxyBody={} finalObjectRelease={} disposition={} reason={} "
            "grab={} caller={}:{}",
            handName(), _savedObjectState.bodyId.value,
            _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
            _grabAuthorityProxy.isValid() ? _grabAuthorityProxy.getBodyId().value : INVALID_BODY_ID,
            releaseContext.finalObjectRelease ? "yes" : "no", releaseDispositionName(releaseContext.disposition),
            releaseContext.reason ? releaseContext.reason : "none", _grabFrame.traceId, callerFile, caller.line());

        const auto releaseTime = runtime_state::currentFrame().timing.elapsedGameSeconds;
        const auto objectHistory = _objectReleaseHistory.peaks(release_velocity::sampleTimeSeconds());
        const auto handHistory = _controllerReleaseHistory.peaks(releaseTime);
        const bool captureReleaseVelocity = !grab_target::requiresNativeBodyOwnership(_savedObjectState.targetKind) &&
            (releaseContext.disposition == GrabReleaseDisposition::PhysicalDrop ||
                releaseContext.disposition == GrabReleaseDisposition::PendingInventoryTransfer ||
                releaseContext.disposition == GrabReleaseDisposition::PendingConsumeTransfer) &&
            releaseContext.applyCapturedReleaseVelocity && releaseContext.finalObjectRelease && world &&
            (objectHistory.count > 0 || handHistory.count > 0);
        if (captureReleaseVelocity) {
            const RE::NiPoint3 objectLocalReleaseVelocity = objectHistory.linear;
            const RE::NiPoint3 handLocalReleaseVelocity = handHistory.linear;
            const RE::NiPoint3 handAngularVelocity = handHistory.angular;

            RE::NiPoint3 releaseLeverOriginHavok = _lastHeldHandPositionHavok;
            RE::NiTransform releaseBodyWorld{};
            bool hasReleaseBodyWorld = false;
            if (tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, releaseBodyWorld)) {
                hasReleaseBodyWorld = true;
                releaseLeverOriginHavok = gamePointToHavokPoint(
                    transform_math::localPointToWorld(releaseBodyWorld, activeProxyConstraintPivotBLocalGame()));
            }

            // Both histories contain world-space movement, including player
            // translation. Do not rotate the velocity by the hand's facing.
            const RE::NiPoint3 releaseVelocity = grab_held_response::composeControllerReleaseVelocity(
                grab_held_response::ReleaseVelocityInput<RE::NiPoint3>{
                    .controllerDerivedEnabled = g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled,
                    .hasHandLocalVelocity = handHistory.count > 0,
                    .hasObjectLocalVelocity = objectHistory.count > 0,
                    .handLocalVelocityHavok = handLocalReleaseVelocity,
                    .objectLocalVelocityHavok = objectLocalReleaseVelocity,
                    .throwMultiplier = g_rockConfig.rockThrowVelocityMultiplier,
                    .maxVelocityHavok = g_rockConfig.rockGrabThrowMaxVelocityHavok,
                });
            const RE::NiPoint3 rawReleaseAngularVelocity = grab_held_response::composeControllerReleaseAngularVelocity(
                grab_held_response::ReleaseAngularVelocityInput<RE::NiPoint3>{
                    .controllerDerivedEnabled = g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled,
                    .hasHandAngularVelocity = handHistory.count > 0,
                    .handAngularVelocityRadiansPerSecond = handAngularVelocity,
                    .angularVelocityScale = g_rockConfig.rockGrabThrowAngularVelocityScale,
                    .maxAngularVelocityRadiansPerSecond = g_rockConfig.rockGrabThrowMaxAngularVelocityRadiansPerSecond,
                });
            RE::NiPoint3 releaseAngularVelocity = rawReleaseAngularVelocity;
            const auto releaseContactSnapshot = readHeldBodyContactSnapshot();
            const bool releaseContactSoftening = releaseContactSnapshot.recent &&
                classifyHeldContactOtherMotion(world, releaseContactSnapshot.otherBodyId) !=
                    held_object_contact_policy::HeldContactOtherMotion::Dynamic;
            const auto releaseAuthority = grab_motion_controller::evaluateHeldAuthority(releaseContactSoftening);
            const float releaseLongObjectAngularScale = grab_motion_controller::computeLongObjectAngularSpeedScale(
                g_rockConfig.rockGrabLongObjectAngularScalingEnabled, _grabFrame.pivotAuthority.longLeverGameUnits,
                g_rockConfig.rockGrabLongObjectReferenceLeverGameUnits, g_rockConfig.rockGrabLongObjectMinAngularScale);
            const float releaseAngularVelocityCap = grab_motion_controller::computeAuthorityScaledAngularVelocityCap(
                g_rockConfig.rockGrabThrowMaxAngularVelocityRadiansPerSecond,
                releaseAuthority.releaseAngularVelocityScale, releaseLongObjectAngularScale);
            releaseAngularVelocity = clampAngularVelocityVector(releaseAngularVelocity, releaseAngularVelocityCap);
            const bool overrideAngularVelocity = g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled &&
                lengthSquared(releaseAngularVelocity) > 0.000001f;
            outcome.velocity.available = true;
            outcome.velocity.primaryBodyId = _savedObjectState.bodyId;
            outcome.velocity.linearVelocityHavok = releaseVelocity;
            outcome.velocity.angularVelocityRadiansPerSecond = releaseAngularVelocity;
            outcome.velocity.overrideAngularVelocity = overrideAngularVelocity;
            if (_heldDriveDecision.includeConnectedLinearVelocity ||
                _heldDriveDecision.includeConnectedAngularVelocity) {
                for (const auto bodyId : _heldBodyIds) {
                    if (outcome.velocity.bodyCount >= outcome.velocity.bodyIds.size()) {
                        break;
                    }
                    outcome.velocity.bodyIds[outcome.velocity.bodyCount++] = bodyId;
                }
            }

            const bool applyReleaseVelocity = releaseContext.disposition == GrabReleaseDisposition::PhysicalDrop;
            if (applyReleaseVelocity) {
                setHeldVelocity(world, _savedObjectState.bodyId, _heldBodyIds, releaseVelocity, releaseAngularVelocity,
                    overrideAngularVelocity, 1.0f, _heldDriveDecision.includeConnectedLinearVelocity,
                    _heldDriveDecision.includeConnectedAngularVelocity);
            }
            ROCK_LOG_DEBUG(Hand,
                "{} hand RELEASE VELOCITY: applied={} driveMode={} linearScope={} angularScope={} authority={} "
                "angularScale={:.2f} angularCap={:.3f} longScale={:.2f} handLocal=({:.3f},{:.3f},{:.3f}) "
                "objectLocal=({:.3f},{:.3f},{:.3f}) angularRaw=({:.3f},{:.3f},{:.3f}) "
                "angularFinal=({:.3f},{:.3f},{:.3f}) final=({:.3f},{:.3f},{:.3f}) lever=({:.3f},{:.3f},{:.3f}) "
                "objectHistory={} handHistory={} multiplier={:.2f}",
                handName(), applyReleaseVelocity ? "yes" : "no",
                held_object_drive_policy::modeName(_heldDriveDecision.mode),
                _heldDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
                _heldDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly", releaseAuthority.reason,
                releaseAuthority.releaseAngularVelocityScale, releaseAngularVelocityCap, releaseLongObjectAngularScale,
                handLocalReleaseVelocity.x, handLocalReleaseVelocity.y, handLocalReleaseVelocity.z,
                objectLocalReleaseVelocity.x, objectLocalReleaseVelocity.y, objectLocalReleaseVelocity.z,
                handAngularVelocity.x, handAngularVelocity.y, handAngularVelocity.z, releaseAngularVelocity.x,
                releaseAngularVelocity.y, releaseAngularVelocity.z, releaseVelocity.x, releaseVelocity.y,
                releaseVelocity.z, releaseLeverOriginHavok.x, releaseLeverOriginHavok.y, releaseLeverOriginHavok.z,
                objectHistory.count, handHistory.count, g_rockConfig.rockThrowVelocityMultiplier);
        }

        if (releaseContext.finalObjectRelease) {
            restoreGrabbedInertia(world, _savedObjectState);
        }

        if (releaseContext.finalObjectRelease && world && _activeGrabLifecycle.size() > 0) {
            const auto releaseRestorePolicy =
                active_grab_body_lifecycle::releaseRestorePolicyForTargetKind(_savedObjectState.targetKind);
            const auto releaseIntent = releaseIntentFromDisposition(releaseContext.disposition);
            const auto releasePlan = _activeGrabLifecycle.restorePlanForRelease(releaseRestorePolicy,
                _savedObjectState.targetKind, releaseIntent);
            restoreActiveGrabLifecycle(world, _activeGrabLifecycle, releasePlan, _savedObjectState.bodyId.value,
                handName(), "release");
            if (_activeGrabLifecycle.hasIncompleteNativeScan()) {
                auto* rootNode = _savedObjectState.refr ? _savedObjectState.refr->Get3D() : nullptr;
                if (active_grab_body_lifecycle::shouldSkipIncompleteScanRootRestore(releasePlan,
                        _savedObjectState.originalMotionPropsId)) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand release: skipped recursive root restore for converted loose-object physical drop "
                        "root='{}' motionProps={} preservedMotion={}",
                        handName(), nodeDebugName(rootNode), _savedObjectState.originalMotionPropsId,
                        releasePlan.preservedConvertedMotionCount);
                } else {
                    restoreIncompleteActivePrepRoot(rootNode, _savedObjectState.originalMotionPropsId, handName(),
                        "release-incomplete-scan");
                }
            }
        }

        _isHoldingFlag.store(false, std::memory_order_release);
        _heldBodyIdsCount.store(0, std::memory_order_release);
        clearHeldBodyContactSnapshot();

        if (world) {
            const auto heldFlagReleases = releaseHeldObjectBodyFlagLeases(world, _savedObjectState.bodyId.value,
                _heldBodyIds, _savedObjectState.targetKind, heldBodyFlagLeaseOwner(this),
                grab_target::requiresNativeBodyOwnership(_savedObjectState.targetKind) ||
                    releaseContext.finalObjectRelease);
            if (heldFlagReleases.failedLeaseCount > 0) {
                ROCK_LOG_WARN(Hand,
                    "{} hand RELEASE held body flag release incomplete: primaryBody={} bodies={} collision={} "
                    "authority={} failed={} finalObjectRelease={}",
                    handName(), _savedObjectState.bodyId.value, heldFlagReleases.bodyCount,
                    heldFlagReleases.collisionLeaseCount, heldFlagReleases.authorityLeaseCount,
                    heldFlagReleases.failedLeaseCount, releaseContext.finalObjectRelease ? "yes" : "no");
            } else {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand RELEASE held body flag leases released: primaryBody={} bodies={} collision={} "
                    "authority={} finalObjectRelease={}",
                    handName(), _savedObjectState.bodyId.value, heldFlagReleases.bodyCount,
                    heldFlagReleases.collisionLeaseCount, heldFlagReleases.authorityLeaseCount,
                    releaseContext.finalObjectRelease ? "yes" : "no");
            }

            // A pending pickup still owns a loose world object. Wake it even
            // without throw history, so rejection leaves an ordinary drop.
            if (releaseContext.finalObjectRelease && shouldActivateReleasedBodies(releaseContext.disposition)) {
                const auto releaseActivation =
                    activateHeldObjectBodySet(world, _savedObjectState.bodyId.value, _heldBodyIds);
                if (releaseActivation.failedActivationCount > 0) {
                    ROCK_LOG_WARN(Hand,
                        "{} hand RELEASE activation incomplete: primaryBody={} bodies={} activated={} failed={}",
                        handName(), _savedObjectState.bodyId.value, releaseActivation.bodyCount,
                        releaseActivation.activatedCount, releaseActivation.failedActivationCount);
                }
            }
        }

        {
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            if (_activeConstraint.isValid()) {
                destroyGrabConstraint(world, _activeConstraint);
            }
            destroyGrabAuthorityProxyLocked(nullptr);
        }

        const bool delayRestore = collisionRestoreMode == GrabReleaseCollisionRestoreMode::Delayed &&
            _grabHandCollisionSuppression.beginDelayedRestore(g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds);
        if (delayRestore) {
            ROCK_LOG_DEBUG(Hand, "{} hand: grab hand collision restore delayed bodies={} firstBodyId={} seconds={:.3f}",
                handName(), _grabHandCollisionSuppression.size(), _grabHandCollisionSuppression.firstBodyId(),
                _grabHandCollisionSuppression.delayedRestoreRemainingSeconds());
        } else {
            restoreHandCollisionAfterGrab(world);
        }

        beginGrabVisualReturn();
        (void)frik_visual_authority::clearHandPose("ROCK_Grab", handFromBool(_isLeft));
        clearGrabExternalHandWorldTransform(_isLeft);
        clearSelectedCloseFingerPose();
        _ragdollGrabOwner.reset();
        _ragdollGrabSurface.reset();
        _savedObjectState.clear();
        _activeGrabLifecycle.clear();
        _activeConstraint.clear();
        clearGrabAuthorityProxyRuntime();
        _heldBodyIds.clear();
        _grabFrame.clear();
        _grabOffsetAcquisition = {};
        _looseSharedAcquisition = {};
        _looseSharedAcquisitionPeer = 0;
        _looseSharedAcquisitionFrame = 0;
        _looseSharedGunstockAim = false;
        _looseWeaponReturnStartFrame = 0;
        _looseSharedHandBlendAlpha = -1.0f;
        _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::Idle;
        _grabObjectGripAtGrab = {};
        _heldDriveDecision = {};
        _heldObjectIsLooseWeapon = false;
        _grabFingerPosePublished = false;
        _grabConvergeStableInsidePocketSeconds = 0.0f;
        _grabConvergePreviousGripErrorGameUnits = std::numeric_limits<float>::max();
        _grabTransitPlacementElapsedSeconds = 0.0f;
        _grabDeviationHistory = {};
        _grabDeviationHistoryCount = 0;
        _grabDeviationHistoryNext = 0;
        _grabVisualHandTransform = {};
        _hasGrabVisualHandTransform = false;
        _lastPublishedGrabVisualHandTransform = {};
        _hasLastPublishedGrabVisualHandTransform = false;
        _grabVisualHandLerpStartTransform = {};
        _grabVisualHandLerpElapsedSeconds = 0.0f;
        _grabVisualHandLerpDurationSeconds = 0.0f;
        _grabFingerJointPose = {};
        _grabFingerLocalTransforms = {};
        _grabFingerLocalTransformMask = 0;
        _grabFingerPose = {};
        _grabFingerTriangleIndex.clear();
        _hasGrabFingerJointPose = false;
        _hasGrabFingerLocalTransforms = false;
        _grabFingerLocalTransformFinalizePending = false;
        _grabFingerSweptArc = false;
        _grabPoseQualityUpdatesRemaining = -1;
        _grabPoseReplay.reset();
        _hasGrabFingerPose = false;
        _controllerReleaseHistory.clear();
        _objectReleaseHistory.clear();
        _releaseObjectNotBefore = release_velocity::sampleTimeSeconds();
        _releaseControllerFrame = 0;
        _previousHeldRawHandWorld = {};
        _previousHeldHandPositionHavok = {};
        _lastHeldHandPositionHavok = {};
        _hasPreviousHeldRawHandWorld = false;
        _hasLastHeldHandPositionHavok = false;
        _currentSelection.clear();
        clearGrabAcquisitionCache("release-cleared-selection");
        HandInteractionEvent releaseEvent = HandInteractionEvent::ReleaseRequested;
        if (releaseContext.disposition == GrabReleaseDisposition::TransferToInventory &&
            _state == HandState::StashCandidate) {
            releaseEvent = HandInteractionEvent::CommitStash;
        }
        applyTransition(HandTransitionRequest{ .event = releaseEvent });

        ROCK_LOG_DEBUG(Hand, "{} hand: Idle", handName());
        return outcome;
    }
}
