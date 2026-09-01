#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// TwoHandedGrip lifecycle and per-frame state dispatch: construction, update(), reset(), core state transitions, firing-hand role switch, and the small shared part-grip coordinate resolvers.

namespace rock
{
    TwoHandedGrip::TwoHandedGrip() :
        _fingerPoseSolveScratch(std::make_unique<FingerPoseSolveScratch>())
    {
        _leftCarry.recoilControllerRegistered =
            frik_visual_authority::registerWeaponHandRecoilController(
                WEAPON_RECOIL_CONTROLLER_TAG,
                &TwoHandedGrip::controlWeaponHandRecoil,
                this,
                GRIP_HAND_POSE_PRIORITY);
        if (!_leftCarry.recoilControllerRegistered) {
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: FRIK weapon-hand recoil controller registration failed; regular FRIK recoil remains active");
        }
    }

    TwoHandedGrip::~TwoHandedGrip()
    {
        if (_leftCarry.recoilControllerRegistered) {
            (void)frik_visual_authority::unregisterWeaponHandRecoilController(
                WEAPON_RECOIL_CONTROLLER_TAG);
            _leftCarry.recoilControllerRegistered = false;
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
        bool authoredIndicatorSupportHandIsLeft = true;
        const auto finishUpdate = [this,
                                      &occupancyBefore,
                                      &authoredIndicatorInput,
                                      &authoredIndicatorSupportHandIsLeft]() {
            const EquippedWeaponGripOccupancy occupancyAfter =
                getGripOccupancy();
            authoredIndicatorInput.supportHandWeaponEngaged =
                authoredIndicatorSupportHandIsLeft ?
                occupancyAfter.left.weaponEngaged() :
                occupancyAfter.right.weaponEngaged();
            const auto indicator =
                authored_weapon_grip_activation_policy::evaluateIndicator(
                    authoredIndicatorInput);
            _support.authoredIndicatorFrame =
                AuthoredSupportGripIndicatorFrame{
                    .positionWorld = RE::NiPoint3{
                        indicator.markerWorld.x,
                        indicator.markerWorld.y,
                        indicator.markerWorld.z,
                    },
                    .supportHandIsLeft =
                        authoredIndicatorSupportHandIsLeft,
                    .visible = indicator.visible,
                };
            return TwoHandedGripUpdateResult{
                .before = occupancyBefore,
                .after = occupancyAfter,
            };
        };
        _leftCarry.recoilReadyThisUpdate =
            _leftCarry.recoilSampleValid &&
            _leftCarry.recoilSampleSequence !=
                _leftCarry.observedRecoilSampleSequence;
        _leftCarry.recoilSupportConstrainedThisUpdate = false;
        _leftCarry.observedRecoilSampleSequence =
            _leftCarry.recoilSampleSequence;
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
        _firing.reattachHoverInsideRadius = false;
        _firing.reattachHoverHandIsLeft = isFiringHandLeft();
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

        publishCollisionIsolatedRightNativeWeaponIntent(
            weaponNode,
            currentWeaponGenerationKey);

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
        const auto primaryReleaseDecision = equipped_weapon_manual_ownership_policy::debouncePrimaryGripRelease(
            _firing.primaryReleaseDebounce,
            stableFrameInput.primaryGripInput.held);
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
        const bool authoredSeatAcquisitionAvailable =
            authoredActivationStateMatches &&
            authoredActivation.activationSpatialPass &&
            authoredCapabilityAllowsIndicator &&
            supportRuntimeState.supportGripAllowed &&
            !supportRuntimeState.providerPartAuthority.active;
        if (authoredSeatAcquisitionAvailable) {
            decision = WeaponInteractionDecision{
                .kind = WeaponInteractionKind::SupportGrip,
                .partKind = WeaponPartKind::Other,
                .gripPose = WeaponGripPoseId::ReceiverSupport,
                .bodyId = 0x7FFF'FFFFu,
                .interactionRoot = weaponNode,
                .sourceRoot = weaponNode,
                .weaponGenerationKey = currentWeaponGenerationKey,
                .acquisitionSource =
                    WeaponInteractionAcquisitionSource::AuthoredSeat,
            };
        }
        const bool supportTouchingSupport =
            routedSupportTouching || authoredSeatAcquisitionAvailable;
        RE::NiNode* interactionWeaponNode =
            authoredSeatAcquisitionAvailable ?
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
                    authoredActivation.activationSpatialPass,
                .supportGripAllowed = supportRuntimeState.supportGripAllowed,
                .providerPartAuthorityActive =
                    supportRuntimeState.providerPartAuthority.active,
                .supportHandHoldingObject = supportHandHoldingObject,
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
            _support.gripAgeSeconds +=
                std::isfinite(dt) && dt > 0.0f ? dt : 0.0f;
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
                        .primaryDetachEnabled = handlingSettings.primaryDetachEnabled,
                        .primaryGripHeld = primaryGripInput.held,
                    });
                if (releaseAction == weapon_two_handed_grip_math::SupportReleaseManualAction::KeepPrimaryOwnership) {
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
                } else if (releaseAction == weapon_two_handed_grip_math::SupportReleaseManualAction::DropEquippedWeapon) {
                    beginHandVisualReturn(supportHandIsLeft, "support-released-drop");
                    beginHandVisualReturn(isFiringHandLeft(), "primary-released-drop");
                    if (_handlingSettings.detachAuthority ==
                        immersive_weapon_policy::DetachAuthority::
                            IntegratedImmersive) {
                        recordFiringGripDetachedHaptic();
                    }
                    requestEquippedWeaponDrop(
                        "support-released-primary-not-held",
                        equipped_weapon_drop_policy::sourceForSupportRelease(
                            primaryGripInput.released,
                            isFiringHandLeft()));
                } else {
                    beginHandVisualReturn(supportHandIsLeft, "support-released");
                    beginHandVisualReturn(isFiringHandLeft(), "primary-authority-cleared");
                    if (ownsWeaponTransform()) {
                        beginWeaponVisualReturn("support-released");
                    }
                    transitionToInactive(ownsWeaponTransform());
                }
            } else if ((handlingSettings.primaryDetachEnabled || handlingSettings.ambidextrousHandoffEnabled) && !primaryGripInput.held &&
                       equipped_weapon_manual_ownership_policy::shouldDeferPrimaryReleaseActionForFreshSupportGrip(_support.gripAgeSeconds)) {
                /*
                 * The firing-grip release confirmed while the support grab is
                 * only a few frames old: same physical gesture or a
                 * grab-synchronized grip flicker, never an independent
                 * release. Hold the two-handed grip unchanged; a re-pressed
                 * grip resumes normally, and promotion/detach run below once
                 * the grab has aged. leftGripHeld/rightGripHeld in the log
                 * discriminate a physical flicker (both pipelines open) from
                 * an input-path divergence (normal pipeline still held).
                 */
                if (!_support.freshGripDeferLogged) {
                    _support.freshGripDeferLogged = true;
                    ROCK_LOG_INFO(Weapon,
                        "TwoHandedGrip: deferring firing-grip release action while support grip is fresh age={:.3f}s firingHand={} leftGripHeld={} rightGripHeld={}",
                        _support.gripAgeSeconds,
                        firingHandName(),
                        stableFrameInput.leftGripHeld ? "yes" : "no",
                        stableFrameInput.rightGripHeld ? "yes" : "no");
                }
                updateGripping(_session.weaponNode, dt);
            } else if (handlingSettings.ambidextrousHandoffEnabled && !primaryGripInput.held && tryPromoteSupportGripToFiringGrip(_session.weaponNode, dt)) {
                // The support hand was wrapped over the firing grip when the
                // firing hand opened: it takes over the SAME weapon-relative
                // grip in place (seamless hand switch, pistol shooting-cup
                // flow). State is PrimaryOnly under the new firing hand.
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
                    beginHandVisualReturn(
                        supportHandIsLeft,
                        "primary-released-noncarry-support-drop");
                    beginHandVisualReturn(
                        isFiringHandLeft(),
                        "primary-released-noncarry-support-drop");
                    if (_handlingSettings.detachAuthority ==
                        immersive_weapon_policy::DetachAuthority::
                            IntegratedImmersive) {
                        recordFiringGripDetachedHaptic();
                    }
                    requestEquippedWeaponDrop(
                        "primary-released-without-carry-authority",
                        isFiringHandLeft() ?
                            equipped_weapon_drop_policy::SourceHand::Left :
                            equipped_weapon_drop_policy::SourceHand::Right);
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
            currentEquippedWeaponOwnershipKey);

        // Enforce the left-firing weapon-node ownership contract after every
        // state/role transition this frame (idempotent; also the parent
        // watchdog for engine-side re-attach).
        syncFiringHandWeaponNodeOwnership(weaponNode);
        updateHandVisualReturns(dt);
        // State transitions and their replacement publications must finish
        // before stale scoped roles are removed. This keeps hFRIK under one
        // continuous ROCK authority selection across scope and role edges.
        reconcileDeferredScopeHandAuthority(weaponNode);
        (void)applyLeftFiringWeaponRecoil(weaponNode);
        traceNativeScopeTransitionFinalState(weaponNode);
        return finishUpdate();
    }

    void TwoHandedGrip::reset()
    {
        (void)frik_visual_authority::clearExternalHandWorldTransform(
            WEAPON_COLLISION_HAND_TAG,
            frik_visual_authority::Hand::Left);
        (void)frik_visual_authority::clearExternalHandWorldTransform(
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
        _hapticEvents = {};
        _firing.reattachHoverInsideRadius = false;
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
        _scope.transitionTraceSequence = 0;
        _scope.transitionTraceFramesRemaining = 0;
        _scope.transitionFinalTraceSequence = 0;
        _scope.transitionFinalTraceSample = 0;
        _scope.transitionFinalTracePending = false;
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
        _support.gripAgeSeconds = 0.0f;
        _support.freshGripDeferLogged = false;
        _session.authorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        _hasSolvedWeaponTransform = false;
        _session.weaponNode = nullptr;
        _session.weaponGenerationKey = 0;
        _session.equippedWeaponOwnershipKey = 0;
        _firing.primaryReleaseDebounce = {};
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
        _leftCarry.recoilWorldDelta =
            transform_math::makeIdentityTransform<RE::NiTransform>();
        _leftCarry.observedRecoilSampleSequence =
            _leftCarry.recoilSampleSequence;
        _leftCarry.recoilSampleValid = false;
        _leftCarry.recoilReadyThisUpdate = false;
        _leftCarry.recoilSupportConstrainedThisUpdate = false;
        resetLockedHandVisualLerp();
    }

    bool TwoHandedGrip::ownsWeaponTransform() const
    {
        return (_session.state == TwoHandedState::Gripping || _session.state == TwoHandedState::PartCarry) &&
               weapon_support_authority_policy::supportGripOwnsWeaponTransform(_session.authorityMode);
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
        clearDynamicSupportAcquisition(
            "transition-to-inactive",
            true);
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
        _support.gripAgeSeconds = 0.0f;
        _support.freshGripDeferLogged = false;
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

        // Drop the old hand's role-tagged FRIK publications; the new hand's
        // grip-frame capture and pose publication are owned by the caller.
        clearPrimaryGripFingerPose(isFiringHandLeft());
        clearPrimaryGripWorldAuthority(isFiringHandLeft());
        clearPrimaryDetachVisualAuthority(isFiringHandLeft());
        _visuals.primaryHandLerp = {};
        _firing.primaryReleaseDebounce = {};
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
