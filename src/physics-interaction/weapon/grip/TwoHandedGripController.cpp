#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// TwoHandedGrip lifecycle and per-frame state dispatch: construction, update(), reset(), core state transitions, firing-hand role switch, and the small shared part-grip coordinate resolvers.

namespace rock
{
    TwoHandedGrip::TwoHandedGrip() :
        _fingerPoseSolveScratch(std::make_unique<FingerPoseSolveScratch>())
    {
        _recoilControllerRegistered =
            frik_visual_authority::registerWeaponHandRecoilController(
                WEAPON_RECOIL_CONTROLLER_TAG,
                &TwoHandedGrip::controlWeaponHandRecoil,
                this,
                GRIP_HAND_POSE_PRIORITY);
        if (!_recoilControllerRegistered) {
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: FRIK weapon-hand recoil controller registration failed; regular FRIK recoil remains active");
        }
    }

    TwoHandedGrip::~TwoHandedGrip()
    {
        if (_recoilControllerRegistered) {
            (void)frik_visual_authority::unregisterWeaponHandRecoilController(
                WEAPON_RECOIL_CONTROLLER_TAG);
            _recoilControllerRegistered = false;
        }
    }

    bool TwoHandedGrip::tryGetSolverHandTransform(bool isLeft, RE::NiTransform& outTransform) const
    {
        const ScopeSafeHandFrameState& state = _scopeSafeHandFrames[isLeft ? 0u : 1u];
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
            _authoredSupportGripIndicatorFrame =
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
        _leftFiringWeaponRecoilReadyThisUpdate =
            _leftFiringWeaponRecoilSampleValid &&
            _weaponRecoilSampleSequence !=
                _observedWeaponRecoilSampleSequence;
        _leftFiringWeaponRecoilSupportConstrainedThisUpdate = false;
        _observedWeaponRecoilSampleSequence =
            _weaponRecoilSampleSequence;
        const bool authoredOnlyModeChanged =
            _handlingSettings.authoredOnlySupportGrabsEnabled !=
            handlingSettings.authoredOnlySupportGrabsEnabled;
        if (authoredOnlyModeChanged) {
            // The mode is acquisition-only: invalidate qualification so the
            // next grab uses the new contract, but never tear down a live grip.
            resetAuthoredSupportCapability("authored-only-mode-changed");
        }
        _handlingSettings = handlingSettings;
        _currentHandDriverFrames[0] = frameInput.leftHandDriverFrame;
        _currentHandDriverFrames[1] = frameInput.rightHandDriverFrame;
        setGrabbedObjectHandPoseOwnership(
            frameInput.leftHandHoldingObject,
            frameInput.rightHandHoldingObject);
        _hasSolvedWeaponTransform = false;
        _authoredSupportGripDebugSnapshot = {};
        _scopeHandAuthorityPublishedThisFrame = {};
        _firingGripReattachHoverInsideRadius = false;
        _firingGripReattachHoverHandIsLeft = isFiringHandLeft();
        if (g_rockConfig.rockDebugDrawNativeScopeActivation &&
            _nativeScopeCameraDebugSnapshot.framesSinceApply != (std::numeric_limits<std::uint32_t>::max)()) {
            ++_nativeScopeCameraDebugSnapshot.framesSinceApply;
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
            if (_state != TwoHandedState::Inactive) {
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
        if (_persistentEquippedCarryActive && isManualOwnershipActive()) {
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
            if (!_persistentEquippedCarryDetachArmed) {
                stableFrameInput.primaryGripInput.held = true;
                stableFrameInput.primaryGripInput.pressed = false;
                stableFrameInput.primaryGripInput.released = false;
            }
        }
        const auto primaryReleaseDecision = equipped_weapon_manual_ownership_policy::debouncePrimaryGripRelease(
            _primaryReleaseDebounce,
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
                _activeEquippedWeaponOwnershipKey,
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
         * weapon interaction probes - see the header note.)
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
                _authoredSupportCapability.initialized &&
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
            if (_authoredSupportCapability.initialized) {
                resetAuthoredSupportCapability("authored-only-mode-disabled");
            }
            _authoredSupportCapability.reason =
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
        } else if (_authoredSupportGripDebugSnapshot.valid) {
            _authoredSupportGripDebugSnapshot.authoredCapability =
                authored_support_grab_policy::Capability::Pending;
            _authoredSupportGripDebugSnapshot.authoredCapabilityReason =
                authored_support_grab_policy::CapabilityReason::ModeDisabled;
        }
        const bool routedSupportTouching =
            decision.kind == WeaponInteractionKind::SupportGrip;
        const bool supportGripHeld = supportHandIsLeft ? stableFrameInput.leftGripHeld : stableFrameInput.rightGripHeld;
        const bool supportHandHoldingObject = supportHandIsLeft ? stableFrameInput.leftHandHoldingObject : stableFrameInput.rightHandHoldingObject;
        const EquippedWeaponPrimaryGripInput& primaryGripInput = stableFrameInput.primaryGripInput;

        const auto& authoredActivation =
            _authoredSupportGripDebugSnapshot;
        const bool authoredActivationStateMatches =
            authoredActivation.valid &&
            authoredActivation.supportHandIsLeft == supportHandIsLeft &&
            authoredActivation.weaponGenerationKey ==
                currentWeaponGenerationKey &&
            authoredActivation.captureSequence ==
                _authoredSupportGripCandidate.captureSequence;
        using IndicatorVec3 =
            authored_weapon_grip_activation_policy::Vec3;
        const auto toIndicatorVector = [](const RE::NiPoint3& value) {
            return IndicatorVec3{ value.x, value.y, value.z };
        };
        authoredIndicatorSupportHandIsLeft = supportHandIsLeft;
        const bool authoredCapabilityAllowsIndicator =
            !_handlingSettings.authoredOnlySupportGrabsEnabled ||
            _authoredSupportCapability.capability ==
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

        switch (_state) {
        case TwoHandedState::Inactive:
            if (supportTouchingSupport && !supportHandHoldingObject) {
                transitionToTouching(interactionWeaponNode, decision);
            }
            break;

        case TwoHandedState::Touching:
            if (supportHandHoldingObject) {
                _state = TwoHandedState::Inactive;
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
                    _state = TwoHandedState::Inactive;
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
            _supportGripAgeSeconds +=
                std::isfinite(dt) && dt > 0.0f ? dt : 0.0f;
            if (!_activeWeaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(_activeWeaponGenerationKey, currentWeaponGenerationKey)) {
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
                    _scopeMenuOpenThisFrame ? "open" : "closed");
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
                    const bool primaryOnlyActive = transitionToPrimaryOnly(
                        _activeWeaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        "support-released-primary-held");
                    if (primaryOnlyActive && usesLeftFiringCarry()) {
                        (void)solveLeftFiringWeaponCarry(
                            _activeWeaponNode);
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
                       equipped_weapon_manual_ownership_policy::shouldDeferPrimaryReleaseActionForFreshSupportGrip(_supportGripAgeSeconds)) {
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
                if (!_freshSupportGripDeferLogged) {
                    _freshSupportGripDeferLogged = true;
                    ROCK_LOG_INFO(Weapon,
                        "TwoHandedGrip: deferring firing-grip release action while support grip is fresh age={:.3f}s firingHand={} leftGripHeld={} rightGripHeld={}",
                        _supportGripAgeSeconds,
                        firingHandName(),
                        stableFrameInput.leftGripHeld ? "yes" : "no",
                        stableFrameInput.rightGripHeld ? "yes" : "no");
                }
                updateGripping(_activeWeaponNode, dt);
            } else if (handlingSettings.ambidextrousHandoffEnabled && !primaryGripInput.held && tryPromoteSupportGripToFiringGrip(_activeWeaponNode)) {
                // The support hand was wrapped over the firing grip when the
                // firing hand opened: it takes over the SAME weapon-relative
                // grip in place (seamless hand switch, pistol shooting-cup
                // flow). State is PrimaryOnly under the new firing hand.
            } else if (handlingSettings.primaryDetachEnabled &&
                       !primaryGripInput.held) {
                if (weapon_support_authority_policy::
                        canCarryAfterFiringGripDetach(_authorityMode)) {
                    if (transitionToPartCarry()) {
                        updatePartCarryGrip(
                            _activeWeaponNode,
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
                updateGripping(_activeWeaponNode, dt);
            }
            break;

        case TwoHandedState::PartCarry:
            if (!_activeWeaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing part-carry authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(_activeWeaponGenerationKey, currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing part-carry authority because weapon generation changed");
                transitionToInactive(false);
            } else if (!handlingSettings.primaryDetachEnabled) {
                transitionToInactive(ownsWeaponTransform());
            } else {
                updatePartCarryGrip(
                    _activeWeaponNode,
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
            if (!_activeWeaponNode) {
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
                    _activeWeaponNode,
                    currentEquippedWeaponOwnershipKey,
                    primaryGripInput,
                    handlingSettings.primaryDetachEnabled);
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
        _weaponCollisionHandAuthorityLive = {};
        _weaponCollisionHandPresentationFromPreviousFrame = {};
        clearDynamicSupportAcquisition("reset", true);
        clearAuthoredSupportGripCandidate();
        resetAuthoredSupportCapability("reset");
        _authoredSupportGripIndicatorFrame = {};
        _authoredSupportGripDebugSnapshot = {};
        _authoredSupportLastStableApproachDirectionWorld = {};
        _authoredSupportLastStableDirectionGenerationKey = 0;
        _authoredSupportLastStableDirectionCaptureSequence = 0;
        _authoredSupportLastStableDirectionHandTopology =
            authored_weapon_grip_activation_policy::HandTopology::Invalid;
        _authoredSupportLastStableApproachDirectionValid = false;
        clearAllVisualReturns("reset", false, true);
        clearNativeScopeOverlayAuthority(true);
        _equippedWeaponDropRequest = {};
        _hapticEvents = {};
        _firingGripReattachHoverInsideRadius = false;
        _nativeScopeAnchorWeaponNode = nullptr;
        _nativeScopeAnchorGenerationKey = 0;
        _nativeScopeAnchorOwnershipKey = 0;
        _nativeScopeAnchorWeaponFormID = 0;
        _nativeScopeAnchorWeaponLocal = {};
        _nativeScopeAnchorSource =
            native_scope_sight_anchor_policy::AnchorSource::None;
        _nativeScopeAnchorValid = false;
        _nativeScopeFallbackRotationDegrees = {};
        _nativeScopeCameraDebugSnapshot = {};
        _nativeScopeActivationDebugSnapshot = {};
        clearNativeScopeRigidFrame();
        _scopeSafeHandFrames = {};
        resetGripFailureDiagnostics();
        _scopeDriverFrameAuthorityActive = false;
        _nativeScopeRequestStateValid = false;
        _nativeScopeRequestActive = false;
        _manualScopeActivationRequested = false;
        _nativeScopeTransitionTraceSequence = 0;
        _nativeScopeTransitionTraceFramesRemaining = 0;
        _nativeScopeTransitionFinalTraceSequence = 0;
        _nativeScopeTransitionFinalTraceSample = 0;
        _nativeScopeTransitionFinalTracePending = false;
        _scopeHandAuthorityPublishedThisFrame = {};
        if (_authoredPrimaryFiringHandWorldActive) {
            clearAuthoredPrimaryFiringHandWorldAuthority();
        }
        clearPrimaryGripFingerPose(isFiringHandLeft());
        clearPrimaryGripWorldAuthority(isFiringHandLeft());
        clearPrimaryDetachVisualAuthority(isFiringHandLeft());
        clearSupportGripPose(true);
        clearSupportGripPose(false);
        restoreFrikPrimaryWeaponPose();
        clearRightFiringHandCanonicalFrame();
        _rightNativeWeaponAimFrame = {};
        _leftFiringDampedFollowFrame = {};
        _rightNaturalBoneInWand = {};
        _leftNaturalBoneInWand = {};
        _rightNaturalBoneInDampedDriver = {};
        _leftNaturalBoneInDampedDriver = {};
        _hasRightNaturalBoneInWand = false;
        _hasLeftNaturalBoneInWand = false;
        _hasRightNaturalBoneInDampedDriver = false;
        _hasLeftNaturalBoneInDampedDriver = false;
        _authoredPrimaryFingerPoseSuppressed = false;
        _leftFiringHandWorldActive = false;
        _leftFiringPositionOnlyTracePending = false;
        _leftHandHoldingObjectForPose = false;
        _rightHandHoldingObjectForPose = false;
        if (_state != TwoHandedState::Inactive) {
            transitionToInactive(false);
            _scopeMenuOpenThisFrame = false;
            _scopeMenuClosedThisFrame = false;
            return;
        }
        _scopeMenuOpenThisFrame = false;
        _scopeMenuClosedThisFrame = false;
        _state = TwoHandedState::Inactive;
        _touchAbsentSeconds = 0.0f;
        _rotationBlend = 0.0f;
        _partGrips = {};
        _partCarryPivotIsLeft = true;
        _partCarryDetachAuthority =
            immersive_weapon_policy::DetachAuthority::None;
        _partCarryGripSeparationWorld = 0.0f;
        _primaryGripLocal = {};
        _lockedGripSeparationWorld = 0.0f;
        _supportGripAgeSeconds = 0.0f;
        _freshSupportGripDeferLogged = false;
        _authorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        _hasSolvedWeaponTransform = false;
        _activeWeaponNode = nullptr;
        _activeWeaponGenerationKey = 0;
        _activeEquippedWeaponOwnershipKey = 0;
        _primaryReleaseDebounce = {};
        _persistentEquippedCarryActive = false;
        _persistentEquippedCarryDetachArmed = false;
        _persistentEquippedCarryInputAcquisitionPending = false;
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;
        _primaryHandWeaponLocal = {};
        _hasFiringHandWeaponLocal = false;
        _primaryGripConfidence = 0.0f;
        _lastPublishedHandWorld = {};
        _hasLastPublishedHandWorld = {};
        _lastRenderedWeaponWorld = {};
        _hasLastRenderedWeaponWorld = false;
        _leftFiringWeaponRecoilWorldDelta =
            transform_math::makeIdentityTransform<RE::NiTransform>();
        _observedWeaponRecoilSampleSequence =
            _weaponRecoilSampleSequence;
        _leftFiringWeaponRecoilSampleValid = false;
        _leftFiringWeaponRecoilReadyThisUpdate = false;
        _leftFiringWeaponRecoilSupportConstrainedThisUpdate = false;
        resetLockedHandVisualLerp();
    }

    bool TwoHandedGrip::ownsWeaponTransform() const
    {
        return (_state == TwoHandedState::Gripping || _state == TwoHandedState::PartCarry) &&
               weapon_support_authority_policy::supportGripOwnsWeaponTransform(_authorityMode);
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
               _weaponNodeOwnershipBlockEngaged ||
               ownsWeaponTransform();
    }

    bool TwoHandedGrip::isWeaponVisualReturnActive() const
    {
        return _returningWeaponVisual.localTransition.active;
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
        if (_state != TwoHandedState::Gripping ||
            usesLeftFiringCarry() ||
            !ownsWeaponTransform() ||
            !_hasSolvedWeaponTransform ||
            !_activeWeaponNode ||
            !_hasFiringHandWeaponLocal ||
            !supportGrip.active ||
            !supportGrip.hasHandWeaponLocal ||
            !isFiniteTransform(_lastSolvedWeaponTransform) ||
            !isFiniteTransform(_primaryHandWeaponLocal)) {
            return false;
        }

        const RE::NiTransform supportHandWorld =
            resolvePartGripHandWorld(supportGrip, _activeWeaponNode);
        if (!isFiniteTransform(supportHandWorld)) {
            return false;
        }

        outRightHandInWeapon = _primaryHandWeaponLocal;
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
            _state = TwoHandedState::Inactive;
            return;
        }

        _state = TwoHandedState::Touching;
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
        const bool weaponReturnActive = _returningWeaponVisual.localTransition.active;
        // Weapon-node topology always returns to native immediately. A visual
        // return owns only ROCK's later transform publication, never hFRIK's
        // external-left-carry topology switch.
        releaseFiringHandWeaponNodeOwnership(_activeWeaponNode);
        clearPrimaryGripFingerPose(
            isFiringHandLeft(),
            weaponReturnActive &&
                _returningWeaponVisual.followsAuthoredPrimaryGrip);
        clearPrimaryGripWorldAuthority(isFiringHandLeft());
        clearPrimaryDetachVisualAuthority(isFiringHandLeft());
        clearSupportGripPose(true);
        clearSupportGripPose(false);
        if (!weaponReturnActive) {
            restoreFrikPrimaryWeaponPose();
        }
        bool restoredWeaponTransformAvailable = false;
        RE::NiTransform restoredWeaponTransform{};
        if (publishRestoredWeaponTransform && _hasWeaponNodeLocalBaseline && _activeWeaponNode) {
            if (_activeWeaponNode->parent) {
                restoredWeaponTransform = transform_math::composeTransforms(_activeWeaponNode->parent->world, _weaponNodeLocalBaseline);
            } else {
                restoredWeaponTransform = _weaponNodeLocalBaseline;
            }
            restoredWeaponTransformAvailable = true;
        }

        _state = TwoHandedState::Inactive;
        _touchAbsentSeconds = 0.0f;
        _rotationBlend = 0.0f;
        _partGrips = {};
        _partCarryPivotIsLeft = true;
        _partCarryDetachAuthority =
            immersive_weapon_policy::DetachAuthority::None;
        _partCarryGripSeparationWorld = 0.0f;
        _primaryGripLocal = {};
        _lockedGripSeparationWorld = 0.0f;
        _supportGripAgeSeconds = 0.0f;
        _freshSupportGripDeferLogged = false;
        _authorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        _hasSolvedWeaponTransform = weaponReturnActive || (publishRestoredWeaponTransform && restoredWeaponTransformAvailable);
        if (weaponReturnActive && _hasLastRenderedWeaponWorld) {
            _lastSolvedWeaponTransform = _lastRenderedWeaponWorld;
        } else if (_hasSolvedWeaponTransform) {
            _lastSolvedWeaponTransform = restoredWeaponTransform;
        }
        _primaryHandWeaponLocal = {};
        _hasFiringHandWeaponLocal = false;
        _primaryGripConfidence = 0.0f;
        _activeWeaponNode = nullptr;
        _activeWeaponGenerationKey = 0;
        _activeEquippedWeaponOwnershipKey = 0;
        _primaryReleaseDebounce = {};
        _persistentEquippedCarryActive = false;
        _persistentEquippedCarryDetachArmed = false;
        _persistentEquippedCarryInputAcquisitionPending = false;
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;
        resetLockedHandVisualLerp();
        if (!isHandVisualReturnActive(true)) {
            _hasLastPublishedHandWorld[0] = false;
        }
        if (!isHandVisualReturnActive(false)) {
            _hasLastPublishedHandWorld[1] = false;
        }
        if (!weaponReturnActive) {
            _hasLastRenderedWeaponWorld = false;
        }
        // The firing-hand role is grip-session state: outside manual
        // ownership the weapon is FRIK/native-carried by the right hand.
        _firingHandIsLeft = false;
        _leftFiringPositionOnlyTracePending = false;
        _leftFiringDampedFollowFrame = {};

        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: grip released");
    }

    void TwoHandedGrip::updateGripping(RE::NiNode* weaponNode, float dt)
    {
        if (_authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport) {
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
        _primaryHandVisualLerp = {};
        _primaryReleaseDebounce = {};
        if (_persistentEquippedCarryActive) {
            _persistentEquippedCarryDetachArmed = false;
        }
        _leftFiringDampedFollowFrame = {};
        _firingHandIsLeft = isLeft;
        // The authored support mirror and activation cone are role-specific.
        // Never carry a capability verdict across a firing/support hand swap.
        resetAuthoredSupportCapability("firing-hand-changed");
        _leftFiringPositionOnlyTracePending = isLeft;
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: firing hand switched to {} reason={}", isLeft ? "left" : "right", reason ? reason : "unknown");
    }
}
