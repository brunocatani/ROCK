#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// TwoHandedGrip lifecycle and per-frame state dispatch: construction, update(), reset(), core state transitions, firing-hand role switch, and the small shared part-grip coordinate resolvers.

namespace rock
{
    TwoHandedGrip::TwoHandedGrip() :
        _fingerPoseSolveScratch(std::make_unique<FingerPoseSolveScratch>())
    {
        _recoil.controllerRegistered =
            frik_visual_authority::registerWeaponHandRecoilController(
                WEAPON_RECOIL_CONTROLLER_TAG,
                &TwoHandedGrip::controlWeaponHandRecoil,
                this,
                GRIP_HAND_POSE_PRIORITY);
        if (!_recoil.controllerRegistered) {
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: FRIK weapon-hand recoil controller registration failed; regular FRIK recoil remains active");
        }
    }

    TwoHandedGrip::~TwoHandedGrip()
    {
        clearOneHandRecoilClaim();
        if (_recoil.controllerRegistered) {
            (void)frik_visual_authority::unregisterWeaponHandRecoilController(
                WEAPON_RECOIL_CONTROLLER_TAG);
            _recoil.controllerRegistered = false;
        }
    }

    bool TwoHandedGrip::tryGetSolverHandTransform(bool isLeft, RE::NiTransform& outTransform) const
    {
        const ScopeSafeHandFrameState& state = _scope.safeHandFrames[isLeft ? 0u : 1u];
        if (!state.currentHandWorldValid) {
            outTransform = {};
            return false;
        }
        outTransform = state.currentHandWorld;
        return true;
    }

    RE::NiPoint3 TwoHandedGrip::worldToWeaponLocal(const RE::NiPoint3& worldPos, const RE::NiAVObject* weaponNode)
    {
        if (!weaponNode) {
            return {};
        }
        return weapon_collision_geometry_math::worldPointToLocal(weaponNode->world.rotate, weaponNode->world.translate, weaponNode->world.scale, worldPos);
    }

    RE::NiPoint3 TwoHandedGrip::weaponLocalToWorld(const RE::NiPoint3& localPos, const RE::NiAVObject* weaponNode)
    {
        if (!weaponNode) {
            return {};
        }
        return weapon_collision_geometry_math::localPointToWorld(weaponNode->world.rotate, weaponNode->world.translate, weaponNode->world.scale, localPos);
    }

    RE::NiPoint3 TwoHandedGrip::resolvePartGripWorld(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(grip, weaponNode)) {
            return transform_math::localPointToWorld(supportAttachmentRoot->world, grip.gripSourceLocal);
        }
        return weaponLocalToWorld(grip.gripLocal, weaponNode);
    }

    RE::NiPoint3 TwoHandedGrip::resolvePartGripWeaponLocal(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        return worldToWeaponLocal(resolvePartGripWorld(grip, weaponNode), weaponNode);
    }

    RE::NiPoint3 TwoHandedGrip::resolvePartGripNormalWeaponLocal(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(grip, weaponNode)) {
            const RE::NiPoint3 supportNormalWorld = transform_math::localVectorToWorld(supportAttachmentRoot->world, grip.normalSourceLocal);
            return transform_math::worldVectorToLocal(weaponNode->world, supportNormalWorld);
        }
        return grip.normalLocal;
    }

    RE::NiTransform TwoHandedGrip::resolvePartGripHandWorld(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(grip, weaponNode)) {
            return transform_math::composeTransforms(supportAttachmentRoot->world, grip.handSourceLocal);
        }
        if (!weaponNode) {
            return RE::NiTransform{};
        }
        return weapon_support_authority_policy::buildVisualOnlySupportHandWorld(weaponNode->world, grip.handWeaponLocal);
    }

    RE::NiAVObject* TwoHandedGrip::resolveCurrentSupportAttachmentRoot(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (!grip.hasSourceFrames || !grip.attachmentRoot || !weaponNode) {
            return nullptr;
        }
        return actor_equipment_grab::nodeContainsNode(weaponNode, grip.attachmentRoot, 64) ? grip.attachmentRoot : nullptr;
    }

    TwoHandedGripUpdateResult TwoHandedGrip::update(
        RE::NiNode* weaponNode,
        const WeaponInteractionContact& leftWeaponContact,
        const WeaponInteractionContact& rightWeaponContact,
        const EquippedWeaponGripFrameInput& frameInput,
        float dt,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const WeaponCollision& weaponCollision,
        const WeaponInteractionRuntimeState& leftRuntimeState,
        const WeaponInteractionRuntimeState& rightRuntimeState,
        weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode,
        bool firingGripProximityAuthorityEnabled,
        const EquippedWeaponHandlingSettings& handlingSettings)
    {
        const EquippedWeaponGripOccupancy occupancyBefore =
            getGripOccupancy();
        authored_weapon_grip_activation_policy::IndicatorInput
            authoredIndicatorInput{};
        RE::NiTransform authoredIndicatorWeaponWorld{};
        std::uint64_t authoredIndicatorWeaponGenerationKey = 0;
        bool authoredIndicatorSupportHandIsLeft = true;
        bool authoredIndicatorWeaponWorldValid = false;
        const auto finishUpdate = [this,
                                      &occupancyBefore,
                                      &authoredIndicatorInput,
                                      &authoredIndicatorWeaponWorld,
                                      &authoredIndicatorWeaponGenerationKey,
                                      &authoredIndicatorSupportHandIsLeft,
                                      &authoredIndicatorWeaponWorldValid]() {
            const EquippedWeaponGripOccupancy occupancyAfter =
                getGripOccupancy();
            const auto& markerHand = _firing.reattachIndicatorFrame.handIsLeft ?
                occupancyAfter.left : occupancyAfter.right;
            if (markerHand.weaponEngaged()) {
                _firing.reattachIndicatorFrame = {};
            }
            authoredIndicatorInput.supportHandWeaponEngaged =
                authoredIndicatorSupportHandIsLeft ?
                occupancyAfter.left.weaponEngaged() :
                occupancyAfter.right.weaponEngaged();
            const auto indicator =
                authored_weapon_grip_activation_policy::evaluateIndicator(
                    authoredIndicatorInput);
            const RE::NiPoint3 indicatorWorld{
                indicator.markerWorld.x,
                indicator.markerWorld.y,
                indicator.markerWorld.z,
            };
            RE::NiPoint3 indicatorWeaponLocal{};
            bool indicatorWeaponLocalValid = false;
            if (indicator.visible && authoredIndicatorWeaponWorldValid) {
                indicatorWeaponLocal = transform_math::worldPointToLocal(
                    authoredIndicatorWeaponWorld,
                    indicatorWorld);
                indicatorWeaponLocalValid =
                    std::isfinite(indicatorWeaponLocal.x) &&
                    std::isfinite(indicatorWeaponLocal.y) &&
                    std::isfinite(indicatorWeaponLocal.z);
            }
            _support.authoredIndicatorFrame =
                AuthoredSupportGripIndicatorFrame{
                    .positionWeaponLocal = indicatorWeaponLocal,
                    .weaponGenerationKey =
                        authoredIndicatorWeaponGenerationKey,
                    .supportHandIsLeft =
                        authoredIndicatorSupportHandIsLeft,
                    .weaponLocalValid = indicatorWeaponLocalValid,
                    .visible = indicator.visible,
                };
            return TwoHandedGripUpdateResult{
                .before = occupancyBefore,
                .after = occupancyAfter,
                .releaseRetained = _gripReleaseRetained,
            };
        };
        clearOneHandRecoilClaim();
        _recoil.rightBaseValid = false;
        _recoil.weaponEvidence = frameInput.recoilWeapon;
        _recoil.equippedIdentity = {
            .formID = frameInput.recoilWeapon.formID,
            .family = weapon_recoil_policy::classifyFamily(frameInput.recoilWeapon),
            .weaponNode = reinterpret_cast<std::uintptr_t>(weaponNode),
            .weaponGeneration = currentWeaponGenerationKey,
            .equippedOwnership = currentEquippedWeaponOwnershipKey,
        };
        _recoil.ticket.beginUpdate(g_rockConfig.rockImmersiveRecoil);
        const bool authoredOnlyModeChanged =
            _handlingSettings.authoredOnlySupportGrabsEnabled !=
            handlingSettings.authoredOnlySupportGrabsEnabled;
        if (authoredOnlyModeChanged) {
            // The mode is acquisition-only: invalidate qualification so the
            // next grab uses the new contract, but never tear down a live grip.
            resetAuthoredSupportCapability("authored-only-mode-changed");
        }
        _handlingSettings = handlingSettings;
        _scope.currentHandDriverFrames[0] = frameInput.leftHandDriverFrame;
        _scope.currentHandDriverFrames[1] = frameInput.rightHandDriverFrame;
        setGrabbedObjectHandPoseOwnership(
            frameInput.leftHandHoldingObject,
            frameInput.rightHandHoldingObject);
        _hasSolvedWeaponTransform = false;
        _support.authoredDebugSnapshot = {};
        _scope.handAuthorityPublishedThisFrame = {};
        _firing.reattachHoverInsideZone = false;
        _firing.reattachHoverHandIsLeft = isFiringHandLeft();
        _firing.reattachIndicatorFrame = {};
        _firing.reattachDebugSnapshot = {};
        _gripReleaseRetained = {};
        if (g_rockConfig.rockDebugDrawNativeScopeActivation &&
            _scope.cameraDebugSnapshot.framesSinceApply != (std::numeric_limits<std::uint32_t>::max)()) {
            ++_scope.cameraDebugSnapshot.framesSinceApply;
        }

        refreshNativeScopeAnchor(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            weaponCollision.getCurrentObservedEquippedWeaponFormID(),
            weaponCollision);
        refreshScopeSafeHandFrames(weaponNode, frameInput, dt);

        if (!runtime_state::isLocalSkeletonReady() || !weaponNode) {
            resetAuthoredSupportCapability(
                "skeleton-or-weapon-unavailable");
            clearAllVisualReturns("skeleton-or-weapon-unavailable", true, true);
            if (_session.state != TwoHandedState::Inactive) {
                transitionToInactive(false);
            }
            reconcileDeferredScopeHandAuthority(weaponNode);
            return finishUpdate();
        }

        publishPhysicalRightNativeWeaponIntent(
            weaponNode,
            currentWeaponGenerationKey);

        // The publisher resets every update and reconstructs the native pose
        // before collision/recoil presentation. Seed aim while this exact
        // frame and weapon still own it, including persistent primary carry.
        // Sampling rendered roots later can be blocked for the whole hold.
        if (_recoil.rightBaseValid) {
            (void)captureRightNativeWeaponAimFrame(weaponNode,
                currentWeaponGenerationKey, currentEquippedWeaponOwnershipKey,
                &_recoil.rightWeaponBase);
        }

        refreshNaturalHandInWandFrames();
        refreshAuthoredSupportRightMirror();

        updateWeaponVisualReturn(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            dt);

        EquippedWeaponGripFrameInput stableFrameInput = frameInput;
        if (_firing.persistentCarryActive && isManualOwnershipActive()) {
            /*
             * A persistent carry may begin without a current logical hold.
             * Keep the firing grip virtually closed until acquisition is
             * committed by this physical hold or by post-update toggle
             * reconciliation. Only the armed hand's later release is allowed
             * through the normal debounce/drop machinery. This avoids an
             * immediate phantom drop while preserving two-hand, detach,
             * stash, and handoff gestures.
             */
            if (frameInput.primaryGripInput.held ||
                frameInput.primaryGripInput.pressed) {
                (void)commitPersistentEquippedCarryInputAcquisition(
                    isFiringHandLeft());
            }
            if (!_firing.persistentCarryDetachArmed) {
                stableFrameInput.primaryGripInput.held = true;
                stableFrameInput.primaryGripInput.pressed = false;
                stableFrameInput.primaryGripInput.released = false;
            }
        }
        // A closed logical grip ends that hand's refused-release episode so
        // the next refusal logs again.
        if (stableFrameInput.primaryGripInput.held) {
            _gripReleaseRetainedLogged[isFiringHandLeft() ? 1u : 0u] = false;
        }
        if (isSupportHandLeft() ? stableFrameInput.leftGripHeld :
                                  stableFrameInput.rightGripHeld) {
            _gripReleaseRetainedLogged[isSupportHandLeft() ? 1u : 0u] = false;
        }

        if (isManualOwnershipActive() &&
            !reconcileCollisionGeneration(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                weaponCollision)) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: clearing authority because equipped weapon instance changed active={:016X} current={:016X}",
                _session.equippedWeaponOwnershipKey,
                currentEquippedWeaponOwnershipKey);
            clearAllVisualReturns("equipped-weapon-identity-changed", true, true);
            transitionToInactive(false);
            reconcileDeferredScopeHandAuthority(weaponNode);
            return finishUpdate();
        }

        /*
         * Left-firing feed-forward pre-write: while ROCK owns the weapon node
         * (left-firing topology), FRIK's earlier skeleton pass has already
         * rewritten the node to its OFFHAND GLUE pose, so at this point
         * weaponNode->world is glue space, not the real carried pose. Every
         * world<->weapon-local conversion below (part-grip captures, mesh
         * grab points, promotion distances, the two-hand solver base) would
         * silently mix real-space palm/contact points with that glue frame -
         * the round-4 corrupted captures. Publishing the canonical
         * feed-forward pose FIRST makes the node a real-space basis for all
         * existing math with no per-call-site special cases; the state
         * handlers below re-publish their final solved pose as before.
         * Right-firing reads FRIK's authored carry and is untouched.
         * (PhysicsInteraction additionally publishes this before the frame's
         * weapon interaction probes - see the header note.) The pre-write is
         * a basis, not the rendered frame: it never updates the rendered
         * weapon record that visual returns, the part-carry handoff, and the
         * seat overlay read.
         */
        (void)publishLeftFiringFeedForwardWeaponPose(weaponNode);

        /*
         * Support-side routing follows the CURRENT firing hand: the support
         * hand is whichever physical hand does not own the firing grip. All
         * grip math below is weapon-relative; the hands only choose roles.
         */
        const bool supportHandIsLeft = isSupportHandLeft();
        const auto supportHandTopology =
            authored_weapon_grip_activation_policy::resolveHandTopology(
                isFiringHandLeft(),
                supportHandIsLeft);
        const WeaponInteractionContact& supportWeaponContact = supportHandIsLeft ? leftWeaponContact : rightWeaponContact;
        const WeaponInteractionRuntimeState& supportRuntimeState = supportHandIsLeft ? leftRuntimeState : rightRuntimeState;
        WeaponInteractionDecision decision = routeWeaponInteraction(supportWeaponContact, supportRuntimeState);
        /*
         * FRAME STAGE: the native graph-output and UpdateFirstPersonArm
         * captures occurred earlier, FRIK has completed its presentation pass,
         * BeforeRock providers have run, and AuthoredPrimaryFiringGripRuntime
         * has republished the current canonical/candidate. Qualification here
         * is therefore post-FRIK, inside ROCK's interaction update, before the
         * AfterRock/Complete provider phases. Authored capability comes only
         * from the current captured pose; generated collision is diagnostic.
         */
        if (_handlingSettings.authoredOnlySupportGrabsEnabled) {
            synchronizeAuthoredSupportCapabilityIdentity(
                weaponNode,
                currentEquippedWeaponOwnershipKey,
                currentWeaponGenerationKey,
                supportHandTopology);
            const auto& capabilityRuntime = runtime_state::currentFrame();
            const bool qualificationReady =
                _support.authoredCapability.initialized &&
                capabilityRuntime.weaponDrawn &&
                f4vr::isNodeVisible(weaponNode) &&
                !capabilityRuntime.localMenuBlocking &&
                !capabilityRuntime.compatibilityConfigBlocking &&
                !frameInput.animationBoundaryActive &&
                supportRuntimeState.supportGripAllowed &&
                !supportRuntimeState.providerPartAuthority.active;
            advanceAuthoredSupportCapabilityQualification(
                qualificationReady,
                dt);
        } else {
            if (_support.authoredCapability.initialized) {
                resetAuthoredSupportCapability("authored-only-mode-disabled");
            }
            _support.authoredCapability.reason =
                authored_support_grab_policy::CapabilityReason::ModeDisabled;
        }
        refreshAuthoredSupportGripActivationState(
            weaponNode,
            currentWeaponGenerationKey,
            weaponCollision);
        if (_handlingSettings.authoredOnlySupportGrabsEnabled &&
            !supportRuntimeState.providerPartAuthority.active) {
            observeAuthoredSupportCapability(
                weaponNode,
                currentWeaponGenerationKey);
        } else if (_support.authoredDebugSnapshot.valid) {
            _support.authoredDebugSnapshot.authoredCapability =
                authored_support_grab_policy::Capability::Pending;
            _support.authoredDebugSnapshot.authoredCapabilityReason =
                authored_support_grab_policy::CapabilityReason::ModeDisabled;
        }
        const bool routedSupportTouching =
            decision.kind == WeaponInteractionKind::SupportGrip;
        const bool supportGripHeld = supportHandIsLeft ? stableFrameInput.leftGripHeld : stableFrameInput.rightGripHeld;
        const bool supportHandHoldingObject = supportHandIsLeft ? stableFrameInput.leftHandHoldingObject : stableFrameInput.rightHandHoldingObject;
        const EquippedWeaponPrimaryGripInput& primaryGripInput = stableFrameInput.primaryGripInput;

        const auto& authoredActivation =
            _support.authoredDebugSnapshot;
        const bool authoredActivationStateMatches =
            authoredActivation.valid &&
            authoredActivation.supportHandIsLeft == supportHandIsLeft &&
            authoredActivation.weaponGenerationKey ==
                currentWeaponGenerationKey &&
            authoredActivation.captureSequence ==
                _support.authoredCandidate.captureSequence;
        using IndicatorVec3 =
            authored_weapon_grip_activation_policy::Vec3;
        const auto toIndicatorVector = [](const RE::NiPoint3& value) {
            return IndicatorVec3{ value.x, value.y, value.z };
        };
        authoredIndicatorSupportHandIsLeft = supportHandIsLeft;
        const bool authoredCapabilityAllowsIndicator =
            !_handlingSettings.authoredOnlySupportGrabsEnabled ||
            _support.authoredCapability.capability ==
                authored_support_grab_policy::Capability::Usable;
        const auto arrangement = authoredGripArrangement(weaponNode, currentWeaponGenerationKey);
        const bool sharedFiringZone = loose_weapon_authored_grab_policy::sharedFiringZone(arrangement);
        const bool authoredSeatAcquisitionAvailable = !sharedFiringZone &&
            authoredActivationStateMatches &&
            authoredActivation.activationSpatialPass &&
            authoredCapabilityAllowsIndicator &&
            supportRuntimeState.supportGripAllowed &&
            !supportRuntimeState.providerPartAuthority.active;
        firing_grip_reattach_zone_policy::ZoneInput handoffInput{};
        firing_grip_reattach_zone_policy::ZoneResult handoffZone{};
        // Evaluate the same cylinders as detached firing-grip reattachment,
        // even without a contact/probe or authored support seat. Keep the
        // debug boundary available while the firing grip is occupied.
        const auto& handoffRuntime = runtime_state::currentFrame();
        const bool handoffZoneEvaluated =
            _session.state != TwoHandedState::PartCarry &&
            handoffRuntime.weaponDrawn && f4vr::isNodeVisible(weaponNode) &&
            !handoffRuntime.localMenuBlocking && !handoffRuntime.compatibilityConfigBlocking &&
            !frameInput.animationBoundaryActive &&
            (_handlingSettings.ambidextrousHandoffEnabled || sharedFiringZone) &&
            arrangement != loose_weapon_authored_grab_policy::Arrangement::Pending &&
            firingGripProximityAuthorityEnabled &&
            canBeginPrimaryOnlyGripForHand(supportHandIsLeft) &&
            tryBuildFiringGripZoneInput(weaponNode, currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                _handlingSettings.firingGripReattachRadiusGameUnits, handoffInput) &&
            tryEvaluateFiringGripZoneForHand(supportHandIsLeft, handoffInput, handoffZone);
        // A station without a collision contact must still obey a provider's
        // exclusive part whitelist for this weapon generation.
        provider::RockProviderWeaponPartTargetResolutionV1 handoffPartResolution{};
        if (handoffZoneEvaluated && handoffZone.inside) {
            provider::RockProviderWeaponPartTargetQueryV1 handoffPartQuery{};
            handoffPartQuery.weaponGenerationKey = currentWeaponGenerationKey;
            handoffPartQuery.bodyId = weapon_part_runtime::kInvalidBodyId;
            (void)provider::resolveWeaponPartTargetV1(handoffPartQuery, handoffPartResolution);
        }
        const bool handoffAcquisitionAvailable =
            handoffZoneEvaluated && !partGrip(supportHandIsLeft).active &&
            !supportHandHoldingObject && canAcquireFiringGripHandoff(
                handoffZone.inside, supportRuntimeState,
                handoffPartResolution.whitelistActive != 0);
        if (handoffAcquisitionAvailable) {
            updateFiringGripZoneIndicator(weaponNode, currentWeaponGenerationKey,
                supportHandIsLeft, handoffZone);
        }
        if (supportGripHeld && handoffZoneEvaluated &&
            !partGrip(supportHandIsLeft).active && !supportHandHoldingObject) {
            ROCK_LOG_SAMPLE_DEBUG(Weapon, 1000,
                "TwoHandedGrip: firing-grip handoff acquisition hand={} inside={} along={:.2f} perp={:.2f} reach={:.2f} radius={:.2f} allowed={} contactRoute={} authoredRoute={} reserved={} provider={} whitelist={}",
                supportHandIsLeft ? "left" : "right", handoffZone.inside,
                handoffZone.alongAxisGameUnits, handoffZone.perpendicularDistanceGameUnits,
                handoffInput.reachGameUnits, handoffInput.radiusGameUnits,
                handoffAcquisitionAvailable, routedSupportTouching,
                authoredSeatAcquisitionAvailable, !supportRuntimeState.supportGripAllowed,
                supportRuntimeState.providerPartAuthority.active,
                handoffPartResolution.whitelistActive != 0);
        }
        if (handoffAcquisitionAvailable || authoredSeatAcquisitionAvailable) {
            decision = WeaponInteractionDecision{
                .kind = WeaponInteractionKind::SupportGrip,
                .partKind = WeaponPartKind::Other,
                .gripPose = WeaponGripPoseId::ReceiverSupport,
                .bodyId = 0x7FFF'FFFFu,
                .interactionRoot = weaponNode,
                .sourceRoot = weaponNode,
                .weaponGenerationKey = currentWeaponGenerationKey,
                .acquisitionSource =
                    handoffAcquisitionAvailable ? WeaponInteractionAcquisitionSource::FiringGripZone :
                        WeaponInteractionAcquisitionSource::AuthoredSeat,
            };
        }
        const bool supportTouchingSupport =
            routedSupportTouching || authoredSeatAcquisitionAvailable || handoffAcquisitionAvailable;
        RE::NiNode* interactionWeaponNode =
            (handoffAcquisitionAvailable || authoredSeatAcquisitionAvailable) ?
            weaponNode :
            sourceRootNodeOrFallback(decision.interactionRoot, weaponNode);
        authoredIndicatorInput =
            authored_weapon_grip_activation_policy::IndicatorInput{
                .weaponFamily = authoredActivation.weaponFamily,
                .authoredSeatWorld = toIndicatorVector(
                    authoredActivation.authoredPalmSeatWorld),
                .supportSideAxisWorld = toIndicatorVector(
                    authoredActivation.supportSideAxisWorld),
                .downAxisWorld = toIndicatorVector(
                    authoredActivation.downAxisWorld),
                .activationStateValid =
                    authoredActivationStateMatches &&
                    authoredCapabilityAllowsIndicator,
                .activationSpatialPass =
                    authoredActivation.activationSpatialPass && !sharedFiringZone && !handoffAcquisitionAvailable,
                .supportGripAllowed = supportRuntimeState.supportGripAllowed,
                .providerPartAuthorityActive =
                    supportRuntimeState.providerPartAuthority.active,
                .supportHandHoldingObject = supportHandHoldingObject,
            };
        authoredIndicatorWeaponWorld = authoredActivation.weaponWorld;
        authoredIndicatorWeaponGenerationKey = currentWeaponGenerationKey;
        authoredIndicatorWeaponWorldValid =
            authoredActivationStateMatches &&
            isInvertibleTransform(authoredIndicatorWeaponWorld);

        const bool supportOwned = partGrip(supportHandIsLeft).active && !partGrip(supportHandIsLeft).attachOnly;
        const bool freeSupportIndicator = !supportHandHoldingObject && !partGrip(supportHandIsLeft).active &&
            (handoffAcquisitionAvailable || authored_weapon_grip_activation_policy::evaluateIndicator(authoredIndicatorInput).visible);
        const auto releaseIntent = equipped_weapon_manual_ownership_policy::resolvePrimaryReleaseIntent(
            _firing.primaryReleaseIntent, {
                .ownershipKey = currentEquippedWeaponOwnershipKey,
                .firingHandIsLeft = isFiringHandLeft(),
                .logicalHeld = stableFrameInput.primaryGripInput.held,
                .logicalReleased = stableFrameInput.primaryGripInput.released,
                .supportGripActive = supportOwned,
                .freeSupportIndicatorActive = freeSupportIndicator,
                .primaryOwned = _session.state != TwoHandedState::PartCarry,
            });
        if (stableFrameInput.primaryGripInput.released) {
            ROCK_LOG_INFO(Weapon,
                "Weapon grip intent: owner={:016X} primary={} mode={} release={} supportOwned={} freeSupportIndicator={}",
                currentEquippedWeaponOwnershipKey, firingHandName(),
                equipped_weapon_toggle_grab_policy::modeName(frameInput.weaponGrabMode),
                releaseIntent.blockedBySupportHover ? "consumed-support-priority" : "accepted",
                supportOwned, freeSupportIndicator);
        }
        if (releaseIntent.blockedBySupportHover) {
            // Reconcile the toggle latch as retained. The rejected gesture
            // cannot turn into a delayed detach when the offhand leaves.
            if (isFiringHandLeft()) _gripReleaseRetained.left = true;
            else _gripReleaseRetained.right = true;
        }
        const auto primaryReleaseDecision = equipped_weapon_manual_ownership_policy::debouncePrimaryGripRelease(
            _firing.primaryReleaseDebounce, releaseIntent.retained);
        stableFrameInput.primaryGripInput.held = primaryReleaseDecision.retained;
        stableFrameInput.primaryGripInput.released = primaryReleaseDecision.releaseConfirmed;

        recordGripFailureFrame(
            weaponNode,
            frameInput,
            stableFrameInput,
            dt,
            weaponCollision.getCurrentObservedEquippedWeaponFormID(),
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey);

        const auto tryReleasedPrimaryHandoff = [&]() {
            if (!handlingSettings.ambidextrousHandoffEnabled || !primaryGripInput.released || !supportOwned) return false;
            const char* reason = "not-attempted";
            const auto result = tryPromoteSupportGripToFiringGrip(_session.weaponNode, dt, reason);
            if (result == weapon_support_authority_policy::FiringGripPromotionResult::Promoted) return true;
            if (result == weapon_support_authority_policy::FiringGripPromotionResult::Blocked) {
                // This gesture selected the firing station. Missing pose data
                // must not reinterpret it as a drop or replay it after the
                // support hand leaves. Retain ownership and require a fresh
                // release, reconciling both hold and toggle input modes.
                _firing.primaryReleaseIntent.pending = false;
                _firing.primaryReleaseDebounce = {};
                (isFiringHandLeft() ? _gripReleaseRetained.left : _gripReleaseRetained.right) = true;
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                    "Primary release handoff blocked: hand={} reason={} action=retain-grips generation={:016X} cleanIntent={} collisionPresentation={} weaponReturn={} scopeOpen={} rootRebase={}",
                    firingHandName(), reason, _session.weaponGenerationKey,
                    _recoil.rightBaseValid, _visuals.weaponCollisionHandPresentationFromPreviousFrame[1],
                    _visuals.returningWeapon.localTransition.active, _scope.menuOpenThisFrame,
                    _scope.safeHandFrames[1].rootRebaseActive);
                updateGripping(_session.weaponNode, dt);
                return true;
            }
            ROCK_LOG_SAMPLE_INFO(Weapon, 1000, "Primary release handoff not applied: hand={} reason={}; evaluating detach policy", firingHandName(), reason);
            return false;
        };

        switch (_session.state) {
        case TwoHandedState::Inactive:
            if (supportTouchingSupport && !supportHandHoldingObject) {
                transitionToTouching(interactionWeaponNode, decision);
            }
            break;

        case TwoHandedState::Touching:
            if (supportHandHoldingObject) {
                _session.state = TwoHandedState::Inactive;
                break;
            }
            if (supportTouchingSupport) {
                _touchAbsentSeconds = 0.0f;
            } else {
                // Measured elapsed time only: an unmeasurable frame holds the
                // timeout instead of advancing it by fabricated time.
                _touchAbsentSeconds +=
                    std::isfinite(dt) && dt > 0.0f ? dt : 0.0f;
                if (_touchAbsentSeconds > TOUCH_TIMEOUT_SECONDS) {
                    _session.state = TwoHandedState::Inactive;
                    break;
                }
            }
            if (weapon_two_handed_grip_math::canStartSupportGrip(supportTouchingSupport, supportGripHeld, supportHandHoldingObject)) {
                transitionToGripping(interactionWeaponNode,
                    decision,
                    weaponCollision,
                    supportAuthorityMode,
                    firingGripProximityAuthorityEnabled,
                    currentEquippedWeaponOwnershipKey,
                    supportRuntimeState.providerPartAuthority);
            }
            break;

        case TwoHandedState::Gripping:
            if (!_session.weaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(_session.weaponGenerationKey, currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because weapon generation changed during support grip");
                transitionToInactive(false);
            } else if (!providerPartAuthorityStillCurrent(partGrip(supportHandIsLeft), currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because provider weapon-part target is no longer current");
                transitionToInactive(false);
            } else if (providerPartTargetNewlyMatchesGrip(partGrip(supportHandIsLeft), currentWeaponGenerationKey)) {
                // The still-held grab recaptures next frame under the new
                // provider resolution (e.g. an AttachOnly whitelist armed
                // mid-hold).
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: releasing support grip to recapture under newly matched provider weapon-part target");
                transitionToInactive(ownsWeaponTransform());
            } else if (!supportRuntimeState.supportGripAllowed) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because offhand reservation disabled support grip");
                transitionToInactive(false);
            } else if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(supportGripHeld, supportHandHoldingObject)) {
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: support release predicate firingHand={} supportHand={} gripHeld={} holdingObject={} scopeMenu={}",
                    firingHandName(),
                    supportHandIsLeft ? "left" : "right",
                    supportGripHeld ? "yes" : "no",
                    supportHandHoldingObject ? "yes" : "no",
                    _scope.menuOpenThisFrame ? "open" : "closed");
                const auto releaseAction = weapon_two_handed_grip_math::resolveSupportReleaseManualAction(
                    weapon_two_handed_grip_math::SupportReleaseOwnershipInput{
                        .firingGripOwnershipEnabled = handlingSettings.firingGripOwnershipEnabled,
                    });
                if (releaseAction == weapon_two_handed_grip_math::SupportReleaseManualAction::KeepPrimaryOwnership) {
                    if (!frameInput.primaryGripInput.held || _firing.primaryReleaseIntent.pending) {
                        recordGripReleaseRetained(isFiringHandLeft(), "two-hand-release-retains-primary");
                        _firing.primaryReleaseIntent.pending = false;
                        _firing.primaryReleaseDebounce = {};
                    }
                    beginHandVisualReturn(supportHandIsLeft, "support-released-primary-held");
                    if (ownsWeaponTransform()) {
                        beginHandVisualReturn(isFiringHandLeft(), "two-hand-primary-return-to-native-carry");
                        if (usesNativeRightCarry()) {
                            beginWeaponVisualReturn("support-released-primary-held");
                        }
                    }
                    if (usesLeftFiringCarry() && ownsWeaponTransform()) {
                        // Left-carry counterpart of beginWeaponVisualReturn:
                        // ease from the two-hand pose to the wand aim instead
                        // of jumping there this frame.
                        beginLeftFiringSupportReleaseReturn(
                            "support-released-primary-held");
                    }
                    const bool primaryOnlyActive = transitionToPrimaryOnly(
                        _session.weaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        "support-released-primary-held");
                    if (primaryOnlyActive && usesLeftFiringCarry()) {
                        (void)solveLeftFiringWeaponCarry(
                            _session.weaponNode,
                            dt);
                    }
                } else {
                    beginHandVisualReturn(supportHandIsLeft, "support-released");
                    beginHandVisualReturn(isFiringHandLeft(), "primary-authority-cleared");
                    if (ownsWeaponTransform()) {
                        beginWeaponVisualReturn("support-released");
                    }
                    transitionToInactive(ownsWeaponTransform());
                }
            } else if (tryReleasedPrimaryHandoff()) {
                // A firing-station handoff owns this release even if its
                // preparation failed. Distant support reaches detach below.
            } else if (handlingSettings.primaryDetachEnabled &&
                       !primaryGripInput.held) {
                if (weapon_support_authority_policy::
                        canCarryAfterFiringGripDetach(_session.authorityMode)) {
                    if (transitionToPartCarry()) {
                        updatePartCarryGrip(
                            _session.weaponNode,
                            dt,
                            stableFrameInput,
                            leftWeaponContact,
                            rightWeaponContact,
                            weaponCollision,
                            currentWeaponGenerationKey,
                            currentEquippedWeaponOwnershipKey,
                            leftRuntimeState,
                            rightRuntimeState);
                    }
                } else {
                    // Visual-only support cannot carry, so the firing grip is
                    // the weapon's only carrier: the open firing hand keeps
                    // the two-hand hold instead of dropping the weapon.
                    recordGripReleaseRetained(
                        isFiringHandLeft(),
                        "two-hand-noncarry-support");
                    _firing.primaryReleaseIntent.pending = false;
                    _firing.primaryReleaseDebounce = {};
                    updateGripping(_session.weaponNode, dt);
                }
            } else {
                updateGripping(_session.weaponNode, dt);
            }
            break;

        case TwoHandedState::PartCarry:
            if (!_session.weaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing part-carry authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(_session.weaponGenerationKey, currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing part-carry authority because weapon generation changed");
                transitionToInactive(false);
            } else if (!handlingSettings.primaryDetachEnabled) {
                transitionToInactive(ownsWeaponTransform());
            } else {
                updatePartCarryGrip(
                    _session.weaponNode,
                    dt,
                    stableFrameInput,
                    leftWeaponContact,
                    rightWeaponContact,
                    weaponCollision,
                    currentWeaponGenerationKey,
                    currentEquippedWeaponOwnershipKey,
                    leftRuntimeState,
                    rightRuntimeState);
            }
            break;

        case TwoHandedState::PrimaryOnly:
            if (!_session.weaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing primary-only authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!handlingSettings.firingGripOwnershipEnabled) {
                transitionToInactive(false);
            } else if (supportTouchingSupport && weapon_two_handed_grip_math::canStartSupportGrip(supportTouchingSupport, supportGripHeld, supportHandHoldingObject)) {
                transitionToGripping(interactionWeaponNode,
                    decision,
                    weaponCollision,
                    supportAuthorityMode,
                    firingGripProximityAuthorityEnabled,
                    currentEquippedWeaponOwnershipKey,
                    supportRuntimeState.providerPartAuthority);
            } else {
                updatePrimaryOnlyGrip(
                    _session.weaponNode,
                    currentEquippedWeaponOwnershipKey,
                    primaryGripInput,
                    handlingSettings.primaryDetachEnabled,
                    dt);
            }
            break;
        }

        refreshRightNativeCanonicalFrame(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            weaponCollision.getCurrentEquippedWeaponInstanceContentKey());

        // Enforce the left-firing weapon-node ownership contract after every
        // state/role transition this frame (idempotent; also the parent
        // watchdog for engine-side re-attach).
        syncFiringHandWeaponNodeOwnership(weaponNode);
        updateHandVisualReturns(dt);
        // State transitions and their replacement publications must finish
        // before stale scoped roles are removed. This keeps hFRIK under one
        // continuous ROCK authority selection across scope and role edges.
        reconcileDeferredScopeHandAuthority(weaponNode);
        applyRightOneHandRecoil(weaponNode);
        return finishUpdate();
    }

    void TwoHandedGrip::reset()
    {
        _confirmedEquippedOwnershipKey = 0;
        _confirmedEquippedGripGenerationKey = 0;
        clearOneHandRecoilClaim();
        _recoil.equippedIdentity = {};
        _recoil.rightBaseValid = false;
        _recoil.rightNeedsNeutralFrame = false;
        (void)frik_visual_authority::clearHandWorld(
            WEAPON_COLLISION_HAND_TAG,
            frik_visual_authority::Hand::Left);
        (void)frik_visual_authority::clearHandWorld(
            WEAPON_COLLISION_HAND_TAG,
            frik_visual_authority::Hand::Right);
        _visuals.weaponCollisionHandAuthorityLive = {};
        _visuals.weaponCollisionHandPresentationFromPreviousFrame = {};
        clearDynamicSupportAcquisition("reset", true);
        clearAuthoredSupportGripCandidate();
        resetAuthoredSupportCapability("reset");
        _support.authoredIndicatorFrame = {};
        _support.authoredDebugSnapshot = {};
        _support.lastStableApproachDirectionWorld = {};
        _support.lastStableDirectionGenerationKey = 0;
        _support.lastStableDirectionCaptureSequence = 0;
        _support.lastStableDirectionHandTopology =
            authored_weapon_grip_activation_policy::HandTopology::Invalid;
        _support.lastStableApproachDirectionValid = false;
        clearAllVisualReturns("reset", false, true);
        clearNativeScopeOverlayAuthority(true);
        _equippedWeaponDropRequest = {};
        _firing.transferredPrimaryGrip = {};
        _hapticEvents = {};
        _gripReleaseRetained = {};
        _gripReleaseRetainedLogged = {};
        _firing.reattachHoverInsideZone = false;
        _firing.reattachIndicatorFrame = {};
        _firing.reattachDebugSnapshot = {};
        _scope.anchorWeaponNode = nullptr;
        _scope.anchorGenerationKey = 0;
        _scope.anchorOwnershipKey = 0;
        _scope.anchorWeaponFormID = 0;
        _scope.anchorWeaponLocal = {};
        _scope.anchorSource =
            native_scope_sight_anchor_policy::AnchorSource::None;
        _scope.anchorValid = false;
        _scope.fallbackRotationDegrees = {};
        _scope.cameraDebugSnapshot = {};
        _scope.activationDebugSnapshot = {};
        clearNativeScopeRigidFrame();
        _scope.safeHandFrames = {};
        resetGripFailureDiagnostics();
        _scope.driverFrameAuthorityActive = false;
        _scope.nativeRequestStateValid = false;
        _scope.nativeRequestActive = false;
        _scope.manualActivationRequested = false;
        _scope.handAuthorityPublishedThisFrame = {};
        if (_firing.authoredHandWorldActive) {
            clearAuthoredPrimaryFiringHandWorldAuthority();
        }
        clearPrimaryGripFingerPose(isFiringHandLeft());
        clearPrimaryGripWorldAuthority(isFiringHandLeft());
        clearPrimaryDetachVisualAuthority(isFiringHandLeft());
        clearSupportGripPose(true);
        clearSupportGripPose(false);
        restoreFrikPrimaryWeaponPose();
        clearRightFiringHandCanonicalFrame();
        _firing.rightNativeWeaponAimFrame = {};
        _firing.leftDampedFollowFrame = {};
        _leftCarry.supportReleaseReturn = {};
        _firing.rightNaturalBoneInWand = {};
        _firing.leftNaturalBoneInWand = {};
        _firing.rightNaturalBoneInDampedDriver = {};
        _firing.leftNaturalBoneInDampedDriver = {};
        _firing.hasRightNaturalBoneInWand = false;
        _firing.hasLeftNaturalBoneInWand = false;
        _firing.hasRightNaturalBoneInDampedDriver = false;
        _firing.hasLeftNaturalBoneInDampedDriver = false;
        _firing.authoredFingerPoseSuppressed = false;
        _firing.leftHandWorldActive = false;
        _firing.leftHandHoldingObjectForPose = false;
        _firing.rightHandHoldingObjectForPose = false;
        if (_session.state != TwoHandedState::Inactive) {
            transitionToInactive(false);
            _scope.menuOpenThisFrame = false;
            _scope.menuClosedThisFrame = false;
            return;
        }
        _scope.menuOpenThisFrame = false;
        _scope.menuClosedThisFrame = false;
        _session.state = TwoHandedState::Inactive;
        _touchAbsentSeconds = 0.0f;
        _support.rotationBlend = 0.0f;
        _support.partGrips = {};
        _partCarry.pivotIsLeft = true;
        _partCarry.detachAuthority =
            immersive_weapon_policy::DetachAuthority::None;
        _partCarry.gripSeparationWorld = 0.0f;
        _firing.primaryGripLocal = {};
        _support.lockedGripSeparationWorld = 0.0f;
        _session.authorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        _hasSolvedWeaponTransform = false;
        _session.weaponNode = nullptr;
        _session.weaponGenerationKey = 0;
        _session.equippedWeaponOwnershipKey = 0;
        _firing.primaryReleaseDebounce = {};
        _firing.primaryReleaseIntent = {};
        _firing.persistentCarryActive = false;
        _firing.persistentCarryDetachArmed = false;
        _firing.persistentCarryInputAcquisitionPending = false;
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;
        _firing.primaryHandWeaponLocal = {};
        _firing.hasPrimaryHandWeaponLocal = false;
        _firing.primaryGripConfidence = 0.0f;
        _visuals.lastPublishedHandWorld = {};
        _visuals.hasLastPublishedHandWorld = {};
        _visuals.lastRenderedWeaponWorld = {};
        _visuals.hasLastRenderedWeaponWorld = false;
        _visuals.weaponHandoff = {};
        _recoil.ticket.invalidate();
        resetLockedHandVisualLerp();
    }

    bool TwoHandedGrip::ownsWeaponTransform() const
    {
        return (_session.state == TwoHandedState::Gripping && _firing.transferredPrimaryGrip.valid()) ||
               ((_session.state == TwoHandedState::Gripping || _session.state == TwoHandedState::PartCarry) &&
                   weapon_support_authority_policy::supportGripOwnsWeaponTransform(_session.authorityMode));
    }

    bool TwoHandedGrip::blocksAuthoredPrimaryGripWeaponAlignment() const
    {
        /*
         * Right-firing PrimaryOnly is lifecycle/input ownership only: hFRIK
         * still publishes the native Weapon transform every frame. Treating
         * that state as a competing transform owner made the authored
         * calibration disappear immediately after a support-hand return.
         * Left-firing carry always remains ROCK-owned even in PrimaryOnly or
         * visual-only support mode, and the topology blocker is included as a
         * fail-closed witness if state and bridge cleanup ever diverge.
         */
        return usesLeftFiringCarry() ||
               _leftCarry.weaponNodeOwnershipBlockEngaged ||
               ownsWeaponTransform();
    }

    bool TwoHandedGrip::isWeaponVisualReturnActive() const
    {
        return _visuals.returningWeapon.localTransition.active;
    }

    bool TwoHandedGrip::getSolvedWeaponTransform(RE::NiTransform& outTransform) const
    {
        if (!_hasSolvedWeaponTransform) {
            return false;
        }
        outTransform = _lastSolvedWeaponTransform;
        return true;
    }

    bool TwoHandedGrip::getManualCycleRockGripBaselines(
        RE::NiTransform& outRightHandInWeapon,
        RE::NiTransform& outLeftHandInWeapon) const
    {
        outRightHandInWeapon = {};
        outLeftHandInWeapon = {};

        const WeaponPartGrip& supportGrip = partGrip(true);
        if (_session.state != TwoHandedState::Gripping ||
            usesLeftFiringCarry() ||
            !ownsWeaponTransform() ||
            !_hasSolvedWeaponTransform ||
            !_session.weaponNode ||
            !_firing.hasPrimaryHandWeaponLocal ||
            !supportGrip.active ||
            !supportGrip.hasHandWeaponLocal ||
            !isFiniteTransform(_lastSolvedWeaponTransform) ||
            !isFiniteTransform(_firing.primaryHandWeaponLocal)) {
            return false;
        }

        const RE::NiTransform supportHandWorld =
            resolvePartGripHandWorld(supportGrip, _session.weaponNode);
        if (!isFiniteTransform(supportHandWorld)) {
            return false;
        }

        outRightHandInWeapon = _firing.primaryHandWeaponLocal;
        outLeftHandInWeapon = transform_math::composeTransforms(
            transform_math::invertTransform(_lastSolvedWeaponTransform),
            supportHandWorld);
        if (!isFiniteTransform(outRightHandInWeapon) ||
            !isFiniteTransform(outLeftHandInWeapon)) {
            outRightHandInWeapon = {};
            outLeftHandInWeapon = {};
            return false;
        }
        return true;
    }

    void TwoHandedGrip::transitionToTouching(RE::NiNode* weaponNode, const WeaponInteractionDecision& decision)
    {
        if (!weaponNode) {
            _session.state = TwoHandedState::Inactive;
            return;
        }

        _session.state = TwoHandedState::Touching;
        _touchAbsentSeconds = 0.0f;
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: touching weapon='{}' bodyId={} partKind={} pose={} interactionRoot={:x} sourceRoot={:x} generation={:016X}",
            weaponNode->name.c_str(),
            decision.bodyId,
            static_cast<int>(decision.partKind),
            static_cast<int>(decision.gripPose),
            reinterpret_cast<std::uintptr_t>(decision.interactionRoot),
            reinterpret_cast<std::uintptr_t>(decision.sourceRoot),
            decision.weaponGenerationKey);
    }

    void TwoHandedGrip::transitionToInactive(bool publishRestoredWeaponTransform)
    {
        _firing.transferredPrimaryGrip = {};
        clearDynamicSupportAcquisition(
            "transition-to-inactive",
            true);
        clearWeaponPoseHandoffBlend("transition-to-inactive", true);
        const bool weaponReturnActive = _visuals.returningWeapon.localTransition.active;
        // Weapon-node topology always returns to native immediately. A visual
        // return owns only ROCK's later transform publication, never hFRIK's
        // external-left-carry topology switch.
        releaseFiringHandWeaponNodeOwnership(_session.weaponNode);
        clearPrimaryGripFingerPose(
            isFiringHandLeft(),
            weaponReturnActive &&
                _visuals.returningWeapon.followsAuthoredPrimaryGrip);
        clearPrimaryGripWorldAuthority(isFiringHandLeft());
        clearPrimaryDetachVisualAuthority(isFiringHandLeft());
        clearSupportGripPose(true);
        clearSupportGripPose(false);
        if (!weaponReturnActive) {
            restoreFrikPrimaryWeaponPose();
        }
        bool restoredWeaponTransformAvailable = false;
        RE::NiTransform restoredWeaponTransform{};
        if (publishRestoredWeaponTransform && _hasWeaponNodeLocalBaseline && _session.weaponNode) {
            if (_session.weaponNode->parent) {
                restoredWeaponTransform = transform_math::composeTransforms(_session.weaponNode->parent->world, _weaponNodeLocalBaseline);
            } else {
                restoredWeaponTransform = _weaponNodeLocalBaseline;
            }
            restoredWeaponTransformAvailable = true;
        }

        _session.state = TwoHandedState::Inactive;
        _touchAbsentSeconds = 0.0f;
        _support.rotationBlend = 0.0f;
        _support.partGrips = {};
        _partCarry.pivotIsLeft = true;
        _partCarry.detachAuthority =
            immersive_weapon_policy::DetachAuthority::None;
        _partCarry.gripSeparationWorld = 0.0f;
        _firing.primaryGripLocal = {};
        _support.lockedGripSeparationWorld = 0.0f;
        _session.authorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        _hasSolvedWeaponTransform = weaponReturnActive || (publishRestoredWeaponTransform && restoredWeaponTransformAvailable);
        if (weaponReturnActive && _visuals.hasLastRenderedWeaponWorld) {
            _lastSolvedWeaponTransform = _visuals.lastRenderedWeaponWorld;
        } else if (_hasSolvedWeaponTransform) {
            _lastSolvedWeaponTransform = restoredWeaponTransform;
        }
        _firing.primaryHandWeaponLocal = {};
        _firing.hasPrimaryHandWeaponLocal = false;
        _firing.primaryGripConfidence = 0.0f;
        _session.weaponNode = nullptr;
        _session.weaponGenerationKey = 0;
        _session.equippedWeaponOwnershipKey = 0;
        _firing.primaryReleaseDebounce = {};
        _firing.primaryReleaseIntent = {};
        _firing.persistentCarryActive = false;
        _firing.persistentCarryDetachArmed = false;
        _firing.persistentCarryInputAcquisitionPending = false;
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;
        resetLockedHandVisualLerp();
        if (!isHandVisualReturnActive(true)) {
            _visuals.hasLastPublishedHandWorld[0] = false;
        }
        if (!isHandVisualReturnActive(false)) {
            _visuals.hasLastPublishedHandWorld[1] = false;
        }
        if (!weaponReturnActive) {
            _visuals.hasLastRenderedWeaponWorld = false;
        }
        // The firing-hand role is grip-session state: outside manual
        // ownership the weapon is FRIK/native-carried by the right hand.
        _session.firingHandIsLeft = false;
        _firing.leftDampedFollowFrame = {};
        clearLeftFiringSupportReleaseReturn("grip-released");

        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: grip released");
    }

    void TwoHandedGrip::updateGripping(RE::NiNode* weaponNode, float dt)
    {
        if (_session.authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport) {
            updateVisualOnlySupportGrip(weaponNode, dt);
            return;
        }

        updateFullWeaponAuthorityGrip(weaponNode, dt);
    }

    void TwoHandedGrip::setFiringHand(const bool isLeft, const char* reason)
    {
        if (isFiringHand(isLeft)) {
            return;
        }

        _firing.transferredPrimaryGrip = {};
        // Drop the old hand's role-tagged FRIK publications; the new hand's
        // grip-frame capture and pose publication are owned by the caller.
        clearPrimaryGripFingerPose(isFiringHandLeft());
        clearPrimaryGripWorldAuthority(isFiringHandLeft());
        clearPrimaryDetachVisualAuthority(isFiringHandLeft());
        _visuals.primaryHandLerp = {};
        _firing.primaryReleaseDebounce = {};
        _firing.primaryReleaseIntent = {};
        if (_firing.persistentCarryActive) {
            _firing.persistentCarryDetachArmed = false;
        }
        _firing.leftDampedFollowFrame = {};
        clearLeftFiringSupportReleaseReturn("firing-hand-changed");
        _session.firingHandIsLeft = isLeft;
        // The authored support mirror and activation cone are role-specific.
        // Never carry a capability verdict across a firing/support hand swap.
        resetAuthoredSupportCapability("firing-hand-changed");
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: firing hand switched to {} reason={}", isLeft ? "left" : "right", reason ? reason : "unknown");
    }
}
