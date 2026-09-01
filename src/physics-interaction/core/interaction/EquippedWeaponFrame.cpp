#include "physics-interaction/core/PhysicsInteractionInternal.h"

// Equipped-weapon frame: transitions, the per-frame equipped weapon update, authored primary grip runtime, handling settings, and shoulder sheath/retrieve.

namespace rock
{
    bool PhysicsInteraction::tryGetManualScopeDirectTransitionTarget(
        std::uint64_t& outWeaponGenerationKey,
        std::uint32_t& outNativeOverlayIndex) const
    {
        outWeaponGenerationKey = 0;
        outNativeOverlayIndex = 0;
        if (!_initialized.load(std::memory_order_acquire) || !runtime_state::isLocalSkeletonReady()) {
            return false;
        }
        const auto snapshot = _weaponCollision.getNativeScopeSightAnchorSnapshot();
        const native_scope_sight_anchor_policy::PublicationIdentity publishedIdentity{
            .weaponGenerationKey = snapshot.weaponGenerationKey,
            .equippedWeaponOwnershipKey = snapshot.equippedWeaponOwnershipKey,
            .weaponFormID = snapshot.weaponFormID,
        };
        const native_scope_sight_anchor_policy::PublicationIdentity currentIdentity{
            .weaponGenerationKey = _weaponCollision.getCurrentWeaponGenerationKey(),
            .equippedWeaponOwnershipKey = _weaponCollision.getCurrentEquippedWeaponOwnershipKey(),
            .weaponFormID = _weaponCollision.getCurrentObservedEquippedWeaponFormID(),
        };
        const NativeScopeResolvedAnchorSnapshot resolvedAnchor =
            _twoHandedGrip.getNativeScopeResolvedAnchorSnapshot();
        const native_scope_sight_anchor_policy::PublicationIdentity
            resolvedIdentity{
                .weaponGenerationKey = resolvedAnchor.weaponGenerationKey,
                .equippedWeaponOwnershipKey =
                    resolvedAnchor.equippedWeaponOwnershipKey,
                .weaponFormID = resolvedAnchor.weaponFormID,
            };
        if (!resolvedAnchor.valid ||
            !native_scope_sight_anchor_policy::matchesCurrentEquippedWeapon(
                resolvedIdentity,
                currentIdentity) ||
            !snapshot.manualDirectTransitionRequired || !snapshot.nativeScopeOverlayValid ||
            !native_scope_sight_anchor_policy::matchesCurrentEquippedWeapon(publishedIdentity, currentIdentity)) {
            return false;
        }
        outWeaponGenerationKey = snapshot.weaponGenerationKey;
        outNativeOverlayIndex = snapshot.nativeScopeOverlayIndex;
        return true;
    }

    void PhysicsInteraction::updateEquippedWeaponTransition()
    {
        const auto& runtime = runtime_state::currentFrame();
        auto* player = f4vr::getPlayer();
        const std::uint32_t nativeGunState =
            f4vr::getNativeGunState(player);
        const std::uint32_t nativeWeaponState =
            f4vr::getNativeWeaponState(player);
        native_equipped_weapon_draw::Identity currentIdentity{};
        const bool currentIdentityCaptured =
            native_equipped_weapon_draw::captureCurrentIdentity(
                currentIdentity);
        weapon_transition_animation_acceleration::service(
            weapon_transition_animation_acceleration::ServiceInput{
                .player = player,
                .identity = currentIdentityCaptured ?
                    weapon_transition_animation_acceleration::Identity{
                        .formID = currentIdentity.formID,
                        .instanceData = currentIdentity.instanceData,
                        .equipIndex = currentIdentity.equipIndex,
                    } :
                    weapon_transition_animation_acceleration::Identity{},
                .nativeWeaponState = nativeWeaponState,
                .runtimeAllowed =
                    runtime.visualAuthorityAvailable &&
                    runtime.localSkeletonReady &&
                    !runtime.localMenuBlocking &&
                    !runtime.compatibilityConfigBlocking,
            });
        const bool nativeWeaponAnimationActive =
            provider::currentNativeAnimationAuthorityFlagsV1() != 0 ||
            nativeGunState ==
                static_cast<std::uint32_t>(RE::GUN_STATE::kReloading);
        _equippedWeaponTransition.update(
            EquippedWeaponTransitionCoordinator::FrameInput{
                .deltaSeconds = runtime.deltaSeconds,
                .visualAuthorityAvailable = runtime.visualAuthorityAvailable,
                .localSkeletonReady = runtime.localSkeletonReady,
                .menuBlocking = runtime.localMenuBlocking,
                .compatibilityBlocking = runtime.compatibilityConfigBlocking,
                .nativeWeaponState = nativeWeaponState,
                .intentionalShoulderSheathActive =
                    _equippedWeaponShoulderSheath.active,
                .shoulderSheathFormID =
                    _equippedWeaponShoulderSheath.weaponFormID,
                .shoulderSheathInstanceData =
                    _equippedWeaponShoulderSheath.weaponInstanceData,
                .shoulderSheathEquipIndex =
                    _equippedWeaponShoulderSheath.equipIndex,
                .nativeWeaponAnimationActive = nativeWeaponAnimationActive,
            });
    }

    PhysicsInteraction::EquippedWeaponFrameResult PhysicsInteraction::updateEquippedWeaponFrame(
        const PhysicsFrameContext& frame,
        RE::bhkWorld* bhk,
        RE::hknpWorld* hknp)
    {
        const auto& runtime = runtime_state::currentFrame();

        RE::NiNode* weaponNode = resolveEquippedWeaponInteractionNode();
        /*
         * FRIK re-attaches the weapon node to the firing hand every frame
         * before ROCK runs, even in part-carry. Republish ROCK's solved carry
         * transform first so weapon-part probes, firing-grip zone checks, and
         * grip capture frames all read the weapon where the player sees it —
         * the same frame the generated colliders follow.
         */
        (void)_twoHandedGrip.republishPartCarryWeaponTransform(weaponNode);
        const bool rightHandWeaponEquipped = weaponNode != nullptr;
        const bool retainedWeaponCollisionActive =
            _weaponCollision.hasWeaponBody() && _weaponCollision.getCurrentWeaponGenerationKey() != 0;
        /*
         * Reload can temporarily remove the first-person weapon node while ROCK
         * deliberately retains the generated weapon body set. Keep the dominant
         * hand under weapon authority until those retained bodies are gone.
         */
        bool rightHandWeaponAuthorityActive = rightHandWeaponEquipped || retainedWeaponCollisionActive;
        /*
         * A visible part-carry (no hand at the firing grip) frees the right hand
         * even while generated weapon bodies exist: the free hand needs live
         * colliders for offhand-parity interaction, the same layer 43 vs 44
         * coexistence the left hand already has. Reload-retained bodies with no
         * visible weapon node keep the dominant-hand suppression because the
         * part-carry state cannot survive a missing weapon node anyway.
         */
        if (rightHandWeaponEquipped && _twoHandedGrip.isPartCarryActive()) {
            rightHandWeaponAuthorityActive = false;
        }
        /*
         * Left-firing carry frees the right hand the same way: the LEFT hand
         * owns the firing grip and the weapon transform, so the right hand has
         * support/free parity (its own part grip is leased separately below).
         */
        if (rightHandWeaponEquipped && _twoHandedGrip.isFiringHandLeft() && _twoHandedGrip.isFiringGripOccupied()) {
            rightHandWeaponAuthorityActive = false;
        }
        const bool rightHandWeaponAuthorityActiveBeforeGrip = rightHandWeaponAuthorityActive;
        const EquippedWeaponGripOccupancy weaponGripOccupancyBeforeUpdate =
            _twoHandedGrip.getGripOccupancy();
        bool leftSupportGripActive =
            weaponGripOccupancyBeforeUpdate.left.partGripActive;
        bool rightPartGripActive =
            weaponGripOccupancyBeforeUpdate.right.partGripActive;
        if (rightHandWeaponAuthorityActive) {
            suppressRightHandCollisionForDominantWeapon(hknp);
        } else {
            restoreRightHandCollisionAfterDominantWeapon(hknp);
        }
        // A part-gripping free hand is a transform driver like the support hand
        // and must not also solve contacts against the weapon package.
        if (rightPartGripActive) {
            suppressHandCollisionForWeaponSupport(hknp, false);
        } else {
            restoreHandCollisionAfterWeaponSupport(hknp, false);
        }

        updateHandCollisions(frame);
        /*
         * Hand collider creation/rebuild happens in updateHandCollisions. Re-run
         * active weapon-owner leases immediately so a new body ID cannot reach
         * the next physics step without the owning-hand suppression.
         */
        if (rightHandWeaponAuthorityActive) {
            suppressRightHandCollisionForDominantWeapon(hknp);
        }
        if (rightPartGripActive) {
            suppressHandCollisionForWeaponSupport(hknp, false);
        }
        logPalmClockSampleForHand("game-after-hand-collider-queue",
            _rightHand,
            hknp,
            frame.right.disabled ? nullptr : &frame.right.rawHandWorld,
            runtime.frameIndex,
            frame.deltaSeconds,
            nullptr);
        logPalmClockSampleForHand("game-after-hand-collider-queue",
            _leftHand,
            hknp,
            frame.left.disabled ? nullptr : &frame.left.rawHandWorld,
            runtime.frameIndex,
            frame.deltaSeconds,
            nullptr);
        updateBodyBoneCollisions(frame);
        updateNativePlayerCollisionSuppression(bhk, hknp);

        {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::WeaponCollision);

            if (g_rockConfig.rockDebugVerboseLogging) {
                if (++_wpnNodeLogCounter >= 90) {
                    _wpnNodeLogCounter = 0;
                    if (weaponNode) {
                        ROCK_LOG_DEBUG(Weapon, "WeaponNode: '{}' pos=({:.1f},{:.1f},{:.1f}) hasBody={} bodyCount={}", weaponNode->name.c_str(), weaponNode->world.translate.x,
                            weaponNode->world.translate.y, weaponNode->world.translate.z, _weaponCollision.hasWeaponBody(), _weaponCollision.getWeaponBodyCount());
                    } else {
                    }
                }
            }
            _weaponCollision.update(hknp, weaponNode, frame.deltaSeconds, runtime.weaponDrawn);
            const auto weaponClassification = _weaponCollision.getEquippedWeaponClassification();
            const bool realMeleeWeaponEquipped =
                weaponClassification.hasEquippedWeapon &&
                weaponClassification.classificationResolved &&
                weaponClassification.sizeClass == WeaponSizeClass::Melee;
            input_remap_runtime::setRealMeleeWeaponEquipped(realMeleeWeaponEquipped);
        }

        const std::uint64_t currentWeaponGenerationKey = _weaponCollision.getCurrentWeaponGenerationKey();
        const std::uint64_t currentEquippedWeaponOwnershipKey = _weaponCollision.getCurrentEquippedWeaponOwnershipKey();
        const std::uint64_t currentAuthoredGripGenerationKey =
            authored_support_grab_policy::resolveAuthoredGenerationKey(
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey);
        const bool suppressDefaultNativeWeaponIntent =
            _twoHandedGrip.previousWeaponCollisionPresentationWasLive();
        _dynamicWeaponCollision.beginFrame(
            runtime.frameIndex,
            hknp,
            bhk,
            weaponNode,
            currentWeaponGenerationKey,
            dynamic_weapon_collision_policy::kDynamicCompoundEnabled &&
                runtime.weaponDrawn &&
                !frame.menuBlocked &&
                physicsWritesAllowedForWorld(frame.hknpWorld),
            suppressDefaultNativeWeaponIntent);
        reconcileEquippedWeaponHandlingMode();

        {
            WeaponInteractionContact leftWeaponContact{};
            WeaponInteractionContact rightWeaponContact{};
            auto leftWeaponContactSource = weapon_debug_notification_policy::WeaponContactSource::None;

            auto publishWeaponInteractionContact = [&](bool isLeft, WeaponInteractionContact& contact) {
                auto& partKind = isLeft ? _leftWeaponContactPartKind : _rightWeaponContactPartKind;
                auto& reloadRole = isLeft ? _leftWeaponContactReloadRole : _rightWeaponContactReloadRole;
                auto& supportRole = isLeft ? _leftWeaponContactSupportRole : _rightWeaponContactSupportRole;
                auto& socketRole = isLeft ? _leftWeaponContactSocketRole : _rightWeaponContactSocketRole;
                auto& actionRole = isLeft ? _leftWeaponContactActionRole : _rightWeaponContactActionRole;
                auto& gripPose = isLeft ? _leftWeaponContactGripPose : _rightWeaponContactGripPose;
                auto& sequence = isLeft ? _leftWeaponContactSequence : _rightWeaponContactSequence;
                auto& missedFrames = isLeft ? _leftWeaponContactMissedFrames : _rightWeaponContactMissedFrames;

                partKind.store(static_cast<std::uint32_t>(contact.partKind), std::memory_order_release);
                reloadRole.store(static_cast<std::uint32_t>(contact.reloadRole), std::memory_order_release);
                supportRole.store(static_cast<std::uint32_t>(contact.supportGripRole), std::memory_order_release);
                socketRole.store(static_cast<std::uint32_t>(contact.socketRole), std::memory_order_release);
                actionRole.store(static_cast<std::uint32_t>(contact.actionRole), std::memory_order_release);
                gripPose.store(static_cast<std::uint32_t>(contact.fallbackGripPose), std::memory_order_release);
                contact.sequence = sequence.fetch_add(1, std::memory_order_acq_rel) + 1;
                missedFrames.store(0, std::memory_order_release);
            };

            auto clearWeaponContactForHand = [&](bool isLeft) {
                if (isLeft) {
                    clearLeftWeaponContact();
                } else {
                    clearRightWeaponContact();
                }
            };

            auto consumeWeaponContactForHand = [&](bool isLeft, const HandFrameInput& handInput, bool probeAllowed, WeaponInteractionContact& outContact) {
                auto& bodyIdAtomic = isLeft ? _leftWeaponContactBodyId : _rightWeaponContactBodyId;
                auto& missedFrames = isLeft ? _leftWeaponContactMissedFrames : _rightWeaponContactMissedFrames;
                auto& acquisitionState = _weaponInteractionAcquisitionStates[isLeft ? 0u : 1u];

                // Drain the physics-thread notification, but do not use an
                // arbitrary finger/body callback as palm-touch provenance.
                // Touch is the deterministic overlap below for both physical
                // hands and for either firing/support role.
                (void)bodyIdAtomic.exchange(INVALID_CONTACT_BODY_ID, std::memory_order_acquire);

                const RE::NiPoint3 legacyPalmPivotWorld =
                    computeGrabLegacyPalmPivotAWorldFromHandBasis(
                        handInput.rawHandWorld,
                        isLeft);
                const bool touchObserved = weaponNode &&
                    _weaponCollision.tryFindInteractionContactNearPoint(
                        weaponNode,
                        legacyPalmPivotWorld,
                        g_rockConfig.rockWeaponInteractionTouchRadius,
                        outContact);
                if (touchObserved) {
                    publishWeaponInteractionContact(isLeft, outContact);
                } else if (weaponNode && probeAllowed) {
                    if (_weaponCollision.tryFindInteractionContactNearPoint(
                            weaponNode,
                            handInput.grabAnchorWorld,
                            g_rockConfig.rockWeaponInteractionProbeRadius,
                            outContact)) {
                        publishWeaponInteractionContact(isLeft, outContact);
                        if (g_rockConfig.rockDebugVerboseLogging && ++_weaponInteractionProbeLogCounter >= 90) {
                            _weaponInteractionProbeLogCounter = 0;
                            ROCK_LOG_DEBUG(Weapon,
                                "WeaponInteractionProbe: hand={} bodyId={} partKind={} supportRole={} reloadRole={} actionRole={} radius={:.1f}",
                                isLeft ? "left" : "right",
                                outContact.bodyId,
                                static_cast<int>(outContact.partKind),
                                static_cast<int>(outContact.supportGripRole),
                                static_cast<int>(outContact.reloadRole),
                                static_cast<int>(outContact.actionRole),
                                g_rockConfig.rockWeaponInteractionProbeRadius);
                        }
                    } else {
                        const auto missed = missedFrames.fetch_add(1, std::memory_order_acq_rel) + 1;
                        if (missed > WEAPON_CONTACT_TIMEOUT_FRAMES) {
                            clearWeaponContactForHand(isLeft);
                        }
                    }
                } else {
                    const auto missed = missedFrames.fetch_add(1, std::memory_order_acq_rel) + 1;
                    if (missed > WEAPON_CONTACT_TIMEOUT_FRAMES) {
                        clearWeaponContactForHand(isLeft);
                    }
                }

                outContact.acquisitionSource = weapon_interaction_acquisition_policy::resolve(
                    acquisitionState,
                    touchObserved,
                    outContact.valid);
                switch (outContact.acquisitionSource) {
                case WeaponInteractionAcquisitionSource::PhysicalContact:
                    return weapon_debug_notification_policy::WeaponContactSource::Contact;
                case WeaponInteractionAcquisitionSource::ProximityProbe:
                    return weapon_debug_notification_policy::WeaponContactSource::Probe;
                case WeaponInteractionAcquisitionSource::None:
                default:
                    return weapon_debug_notification_policy::WeaponContactSource::None;
                }
            };

            // A loose-weapon equip carries the originating physical hand into
            // the first equipped frame; use it immediately so input/contact
            // routing never spends a frame under the default right-hand role.
            if (_pendingEquippedWeaponPrimaryOnlyGripStart.pending) {
                _pendingEquippedWeaponPrimaryOnlyGripStart.remainingSeconds -=
                    (std::max)(0.0f, frame.deltaSeconds);
                if (_pendingEquippedWeaponPrimaryOnlyGripStart.remainingSeconds <= 0.0f) {
                    ROCK_LOG_WARN(Weapon,
                        "Held weapon manual ownership handoff expired targetForm={:08X} targetInstance={:#x}",
                        _pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponFormID,
                        _pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponInstanceData);
                    _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                }
            }
            auto* observedEquippedWeapon = currentEquippedWeaponForm();
            const std::uint32_t observedEquippedWeaponFormID =
                observedEquippedWeapon ? observedEquippedWeapon->formID : 0;
            const auto observedEquippedWeaponInstanceData =
                reinterpret_cast<std::uintptr_t>(
                    currentEquippedWeaponInstanceData(observedEquippedWeapon));
            const bool equippedWeaponShoulderStashActive =
                equipped_weapon_drop_policy::equippedWeaponShoulderStashAvailable(
                    _equippedWeaponHandlingSettings.equippedWeaponShoulderStashEnabled);
            const bool inputBlockingMenuActive =
                input_remap_runtime::isMenuInputActive();
            const bool shoulderPendingPrimaryStartMatchesCurrentWeapon =
                _pendingEquippedWeaponPrimaryOnlyGripStart.pending &&
                (_pendingEquippedWeaponPrimaryOnlyGripStart.
                        targetWeaponFormID == 0 ||
                    equipped_weapon_transition_policy::
                        matchesExpectedIdentity(
                            observedEquippedWeaponFormID,
                            observedEquippedWeaponInstanceData,
                            _pendingEquippedWeaponPrimaryOnlyGripStart.
                                targetWeaponFormID,
                            _pendingEquippedWeaponPrimaryOnlyGripStart.
                                targetWeaponInstanceData,
                            _pendingEquippedWeaponPrimaryOnlyGripStart.
                                previousWeaponFormID,
                            _pendingEquippedWeaponPrimaryOnlyGripStart.
                                previousWeaponInstanceData));
            const bool shoulderFiringHandIsLeft =
                shoulderPendingPrimaryStartMatchesCurrentWeapon ?
                _pendingEquippedWeaponPrimaryOnlyGripStart.isLeft :
                _twoHandedGrip.isFiringHandLeft();
            const auto equippedWeaponShoulderFrame =
                advanceEquippedWeaponShoulderCoordinator(
                frame,
                equippedWeaponShoulderStashActive,
                inputBlockingMenuActive,
                observedEquippedWeaponFormID,
                observedEquippedWeaponInstanceData,
                weaponNode,
                currentEquippedWeaponOwnershipKey,
                shoulderFiringHandIsLeft);
            const bool pendingPrimaryStartMatchesCurrentWeapon =
                _pendingEquippedWeaponPrimaryOnlyGripStart.pending &&
                (_pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponFormID == 0 ||
                    equipped_weapon_transition_policy::matchesExpectedIdentity(
                        observedEquippedWeaponFormID,
                        observedEquippedWeaponInstanceData,
                        _pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponFormID,
                        _pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponInstanceData,
                        _pendingEquippedWeaponPrimaryOnlyGripStart.previousWeaponFormID,
                        _pendingEquippedWeaponPrimaryOnlyGripStart.previousWeaponInstanceData));
            const bool firingHandIsLeft = pendingPrimaryStartMatchesCurrentWeapon ?
                _pendingEquippedWeaponPrimaryOnlyGripStart.isLeft :
                _twoHandedGrip.isFiringHandLeft();
            const bool supportHandIsLeft = !firingHandIsLeft;
            const auto firingGripDecision =
                resolveEquippedWeaponDetachDecision(
                    _equippedWeaponHandlingSettings);

            /*
             * While the LEFT hand carries the weapon, the node still sits at
             * FRIK's offhand glue pose here; the ranked part probes below
             * convert real palm points into node-local space, so glue space
             * made a forend grab select the scope's sight body ~10gu away
             * (fallback wrap pose, grab churn). Publish the canonical carry
             * pose first so both hands probe the weapon where it actually is.
             */
            (void)_twoHandedGrip.publishLeftFiringFeedForwardWeaponPose(weaponNode);

            leftWeaponContactSource = consumeWeaponContactForHand(true, frame.left, weaponNode != nullptr, leftWeaponContact);
            // The free firing hand needs weapon-part probes for part grips and
            // for the reattach squeeze's proximity check, exactly like the
            // offhand; while the LEFT hand fires, the right hand is the
            // support/free hand and probes unconditionally.
            const bool rightWeaponContactProbeAllowed = weaponNode != nullptr &&
                (_twoHandedGrip.isPartCarryActive() || firingHandIsLeft);
            (void)consumeWeaponContactForHand(false, frame.right, rightWeaponContactProbeAllowed, rightWeaponContact);

            auto leftPhysicalGripState =
                peekGrabButtonState(true, input_remap_policy::kGrabButtonId);
            auto rightPhysicalGripState =
                peekGrabButtonState(false, input_remap_policy::kGrabButtonId);
            const auto maskShoulderGestureInput =
                [&](const bool isLeft, GrabButtonState& button) {
                    const bool consumed = isLeft ?
                        equippedWeaponShoulderFrame.decision.consumeLeftInput :
                        equippedWeaponShoulderFrame.decision.consumeRightInput;
                    if (!consumed) {
                        return;
                    }

                    if (equippedWeaponShoulderFrame.decision.gestureAction ==
                        equipped_weapon_shoulder::Action::SubmitRetrieve) {
                        // Retrieval starts a committed carry without forwarding
                        // that same button cycle into detach, toggle, or ordinary
                        // grab handling.
                        button = {};
                    } else if (equippedWeaponShoulderFrame.decision.gestureAction ==
                               equipped_weapon_shoulder::Action::SubmitSheath) {
                        // The coordinator consumed the physical tap/release.
                        // Keep the current carry logically closed until native
                        // sheathing captures its transfer and clears ownership.
                        button = GrabButtonState{ .held = true };
                    }
                };
            maskShoulderGestureInput(true, leftPhysicalGripState);
            maskShoulderGestureInput(false, rightPhysicalGripState);
            bool leftGripHeld = leftPhysicalGripState.held;
            bool rightGripHeld = rightPhysicalGripState.held;

            WeaponInteractionRuntimeState providerInteractionState{};

            ::rock::provider::RockProviderWeaponPartTargetResolutionV1 weaponPartResolution{};
            const auto weaponPartQuery = makeProviderWeaponPartTargetQuery(leftWeaponContact, _weaponCollision);
            const bool weaponPartResolved = leftWeaponContact.valid &&
                ::rock::provider::resolveWeaponPartTargetV1(weaponPartQuery, weaponPartResolution);
            const bool weaponPartWhitelistActive = weaponPartResolved && weaponPartResolution.whitelistActive != 0;
            const bool weaponPartMatched = weaponPartResolved && weaponPartResolution.matched != 0;
            if (weaponPartWhitelistActive && !weaponPartMatched) {
                providerInteractionState.supportGripAllowed = false;
            } else if (weaponPartMatched) {
                providerInteractionState.providerPartAuthority = makeWeaponProviderPartAuthority(weaponPartQuery, weaponPartResolution);
            }

            WeaponInteractionRuntimeState rightHandInteractionState{};
            ::rock::provider::RockProviderWeaponPartTargetResolutionV1 rightWeaponPartResolution{};
            const auto rightWeaponPartQuery = makeProviderWeaponPartTargetQuery(rightWeaponContact, _weaponCollision);
            const bool rightWeaponPartResolved = rightWeaponContact.valid &&
                ::rock::provider::resolveWeaponPartTargetV1(rightWeaponPartQuery, rightWeaponPartResolution);
            const bool rightWeaponPartWhitelistActive = rightWeaponPartResolved && rightWeaponPartResolution.whitelistActive != 0;
            const bool rightWeaponPartMatched = rightWeaponPartResolved && rightWeaponPartResolution.matched != 0;
            if (rightWeaponPartWhitelistActive && !rightWeaponPartMatched) {
                rightHandInteractionState.supportGripAllowed = false;
            } else if (rightWeaponPartMatched) {
                rightHandInteractionState.providerPartAuthority = makeWeaponProviderPartAuthority(rightWeaponPartQuery, rightWeaponPartResolution);
            }

            /*
             * The offhand reservation is a SUPPORT-ROLE gate, not a physical
             * left-hand gate: it constrains whichever hand currently plays the
             * support role. Part grips by the free firing hand stay gated by
             * the provider part whitelist alone so PAPER reload sessions still
             * constrain which parts the free hand may take.
             */
            const auto offhandReservation = offhand_interaction_reservation::fromProvider(::rock::provider::currentOffhandReservation());
            if (!offhand_interaction_reservation::allowsSupportGrip(offhandReservation)) {
                (supportHandIsLeft ? providerInteractionState : rightHandInteractionState).supportGripAllowed = false;
            }

            const WeaponInteractionDecision leftWeaponDecision = routeWeaponInteraction(leftWeaponContact, providerInteractionState);
            const auto weaponNotificationKey = weapon_debug_notification_policy::makeWeaponNotificationKey(
                leftWeaponContact,
                leftWeaponDecision,
                leftWeaponContactSource);

            const bool leftHandHoldingObject = _leftHand.isHolding();
            auto supportAuthorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
            bool supportAuthorityProviderOverride = false;
            // The grab-mode override follows the SUPPORT-ROLE hand's provider
            // resolution: that is the hand whose grip the mode describes.
            const bool supportWeaponPartMatched = supportHandIsLeft ? weaponPartMatched : rightWeaponPartMatched;
            const auto& supportWeaponPartResolution = supportHandIsLeft ? weaponPartResolution : rightWeaponPartResolution;
            if (supportWeaponPartMatched) {
                if (supportWeaponPartResolution.grabMode == ::rock::provider::RockProviderWeaponPartGrabModeV1::FullTwoHandAuthority) {
                    supportAuthorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
                    supportAuthorityProviderOverride = true;
                } else if (supportWeaponPartResolution.grabMode == ::rock::provider::RockProviderWeaponPartGrabModeV1::AttachOnly) {
                    supportAuthorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport;
                    supportAuthorityProviderOverride = true;
                }
            }
            const bool firingGripProximityAuthorityEnabled = weapon_support_authority_policy::canApplyFiringGripProximityAuthority(
                supportAuthorityProviderOverride);
            EquippedWeaponPrimaryGripInput primaryGripInput{};
            GrabButtonState primaryGrabState{};
            bool primaryGrabStateRead = false;
            _firingHandGrabButtonFrameState = {};
            auto readPrimaryGrabState = [&]() -> const GrabButtonState& {
                if (!primaryGrabStateRead) {
                    primaryGrabState = readGrabButtonState(firingHandIsLeft, input_remap_policy::kGrabButtonId);
                    // Menu rearm intentionally masks gameplay edges, but
                    // firing-grip ownership still follows the physical hand
                    // state after the menu closes.
                    primaryGrabState.held = input_remap_runtime::isRawButtonPhysicallyHeld(firingHandIsLeft, input_remap_policy::kGrabButtonId);
                    maskShoulderGestureInput(
                        firingHandIsLeft,
                        primaryGrabState);
                    primaryGrabStateRead = true;
                    (firingHandIsLeft ? leftPhysicalGripState :
                                        rightPhysicalGripState) =
                        primaryGrabState;
                    // Publish the consumed snapshot so the normal grab pipeline
                    // sees the same edges instead of re-consuming cleared ones.
                    _firingHandGrabButtonFrameState = SharedGrabButtonFrameState{
                        .valid = true,
                        .isLeft = firingHandIsLeft,
                        .held = primaryGrabState.held,
                        .pressed = primaryGrabState.pressed,
                        .released = primaryGrabState.released,
                    };
                }
                return primaryGrabState;
            };
            const bool primaryPoseBlockerAvailable = frik_visual_authority::canBlockPrimaryHandWeaponPose();
            const bool ambidextrousHandoffAvailable =
                _equippedWeaponHandlingSettings.ambidextrousHandoffEnabled &&
                TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true);
            const bool firingGripOwnershipFeatureAvailable = equipped_weapon_manual_ownership_policy::featureAvailable(
                !_equippedWeaponShoulderSheath.active &&
                    firingGripDecision.firingGripOwnershipEnabled,
                primaryPoseBlockerAvailable,
                weaponNode != nullptr,
                currentEquippedWeaponOwnershipKey);
            const bool primaryDetachFeatureAvailable = equipped_weapon_manual_ownership_policy::featureAvailable(
                !_equippedWeaponShoulderSheath.active &&
                    firingGripDecision.primaryDetachEnabled,
                primaryPoseBlockerAvailable,
                weaponNode != nullptr,
                currentEquippedWeaponOwnershipKey);
            bool pendingToggleCancelRequested = false;
            auto& pendingPrimaryStart =
                _pendingEquippedWeaponPrimaryOnlyGripStart;
            if (_equippedWeaponHandlingSettings.toggleGrabEnabled &&
                pendingPrimaryStart.toggleAcquisitionCommitted) {
                const auto& pendingHandPhysicalGrip =
                    firingHandIsLeft ?
                    leftPhysicalGripState :
                    rightPhysicalGripState;
                if (pendingHandPhysicalGrip.released ||
                    (!pendingHandPhysicalGrip.held &&
                        !pendingHandPhysicalGrip.pressed)) {
                    pendingPrimaryStart.toggleAcquisitionReleased = true;
                }
                pendingToggleCancelRequested =
                    pendingPrimaryStart.toggleAcquisitionReleased &&
                    pendingHandPhysicalGrip.pressed;
                if (pendingToggleCancelRequested) {
                    _equippedWeaponToggleGrabReleasePressConsumedThisFrame[
                        equipped_weapon_toggle_grab_policy::handIndex(
                            firingHandIsLeft)] = true;
                    ROCK_LOG_DEBUG(
                        Weapon,
                        "Pending firing-grip toggle acquisition cancelled by a second press hand={}",
                        firingHandIsLeft ? "left" : "right");
                    pendingPrimaryStart = {};
                }
            }
            const bool pendingToggleGripRetained =
                !pendingToggleCancelRequested &&
                _equippedWeaponHandlingSettings.toggleGrabEnabled &&
                _pendingEquippedWeaponPrimaryOnlyGripStart.
                    toggleAcquisitionCommitted;
            if (inputBlockingMenuActive) {
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            } else if (_pendingEquippedWeaponPrimaryOnlyGripStart.pending &&
                !equipped_weapon_manual_ownership_policy::shouldKeepPendingPrimaryOnlyStart(
                    equipped_weapon_manual_ownership_policy::PendingPrimaryOnlyStartInput{
                        .pending = _pendingEquippedWeaponPrimaryOnlyGripStart.pending,
                        .gripHeld =
                            input_remap_runtime::isRawButtonPhysicallyHeld(
                                firingHandIsLeft,
                                input_remap_policy::kGrabButtonId) ||
                            pendingToggleGripRetained,
                        .committedTransfer =
                            _pendingEquippedWeaponPrimaryOnlyGripStart.
                                committedTransfer,
                        .ownershipModeEnabled =
                            firingGripDecision.firingGripOwnershipEnabled,
                        .primaryPoseBlockerAvailable = primaryPoseBlockerAvailable,
                    })) {
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            }
            const input_remap_policy::EquippedWeaponFiringGripInputGate firingGripInputGate{
                .featureAvailable = firingGripOwnershipFeatureAvailable,
                .canUseFiringGripInput = _twoHandedGrip.canUseFiringGripInput(),
                .menuInputActive = inputBlockingMenuActive,
            };
            if (input_remap_policy::shouldConsumeEquippedWeaponFiringGripInput(firingGripInputGate)) {
                const auto& primaryState = readPrimaryGrabState();
                if (input_remap_policy::shouldUseEquippedWeaponFiringGripInput(firingGripInputGate)) {
                    primaryGripInput = EquippedWeaponPrimaryGripInput{
                        .held = primaryState.held,
                        .pressed = primaryState.pressed,
                        .released = primaryState.released,
                    };
                }
            }

            if (_equippedWeaponHandlingSettings.toggleGrabEnabled &&
                !primaryGrabStateRead) {
                static_cast<void>(readPrimaryGrabState());
            }

            auto toggleOccupancyBefore =
                _twoHandedGrip.getGripOccupancy();
            if (_twoHandedGrip.
                    isPersistentEquippedCarryInputAcquisitionPending()) {
                auto& pendingFiringOccupancy =
                    _twoHandedGrip.isFiringHandLeft() ?
                    toggleOccupancyBefore.left :
                    toggleOccupancyBefore.right;
                pendingFiringOccupancy.firingGripActive = false;
            }

            bool primaryOnlyGripStartedThisFrame = false;
            if (firingGripOwnershipFeatureAvailable && !inputBlockingMenuActive && !_twoHandedGrip.isManualOwnershipActive()) {
                const auto& primaryState = readPrimaryGrabState();
                if (_pendingEquippedWeaponPrimaryOnlyGripStart.pending &&
                    !primaryState.held &&
                    !_pendingEquippedWeaponPrimaryOnlyGripStart.
                        committedTransfer &&
                    !pendingToggleGripRetained) {
                    _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                }

                const bool pendingPrimaryOnlyStartRequested =
                    !pendingToggleCancelRequested &&
                    equipped_weapon_manual_ownership_policy::
                        shouldStartPendingPrimaryOnlyGrip(
                            pendingPrimaryStartMatchesCurrentWeapon,
                            primaryState.held || pendingToggleGripRetained,
                            _pendingEquippedWeaponPrimaryOnlyGripStart.
                                committedTransfer);
                const bool primaryOnlyStartRequested =
                    !pendingToggleCancelRequested &&
                    weaponNode != nullptr &&
                    currentEquippedWeaponOwnershipKey != 0 &&
                    ((primaryDetachFeatureAvailable && primaryState.held &&
                         primaryState.pressed) ||
                        pendingPrimaryOnlyStartRequested);
                if (pendingPrimaryStartMatchesCurrentWeapon &&
                    _pendingEquippedWeaponPrimaryOnlyGripStart.isLeft &&
                    (!_pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringHandWeaponLocal ||
                        !_pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringGripWeaponLocal)) {
                    _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringHandWeaponLocal =
                        _twoHandedGrip.tryBuildCurrentLeftFiringGripCapture(
                            weaponNode,
                            currentWeaponGenerationKey,
                            currentEquippedWeaponOwnershipKey,
                            _pendingEquippedWeaponPrimaryOnlyGripStart.firingHandWeaponLocal,
                            _pendingEquippedWeaponPrimaryOnlyGripStart.firingGripWeaponLocal);
                    _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringGripWeaponLocal =
                        _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringHandWeaponLocal;
                }
                const RE::NiTransform* capturedFiringHandWeaponLocal =
                    pendingPrimaryStartMatchesCurrentWeapon &&
                        _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringHandWeaponLocal ?
                    &_pendingEquippedWeaponPrimaryOnlyGripStart.firingHandWeaponLocal :
                    nullptr;
                const RE::NiPoint3* capturedFiringGripWeaponLocal =
                    pendingPrimaryStartMatchesCurrentWeapon &&
                        _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringGripWeaponLocal ?
                    &_pendingEquippedWeaponPrimaryOnlyGripStart.firingGripWeaponLocal :
                    nullptr;
                const bool retainUntilPhysicalGrip =
                    pendingPrimaryStartMatchesCurrentWeapon &&
                    (_pendingEquippedWeaponPrimaryOnlyGripStart.
                            committedTransfer ||
                        pendingToggleGripRetained);
                const auto leftTakeoverReadiness =
                    _twoHandedGrip.getLeftFiringTakeoverReadiness(
                        weaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        _equippedWeaponHandlingSettings.
                            authoredOnlySupportGrabsEnabled);
                const bool leftTakeoverBlocked =
                    pendingPrimaryStartMatchesCurrentWeapon &&
                    primaryOnlyStartRequested &&
                    firingHandIsLeft &&
                    !authored_support_grab_policy::
                        leftFiringTakeoverReady(
                            leftTakeoverReadiness);
                if (leftTakeoverBlocked) {
                    auto& pendingStart =
                        _pendingEquippedWeaponPrimaryOnlyGripStart;
                    if (pendingStart.takeoverWitness.observe(
                            leftTakeoverReadiness)) {
                        ROCK_LOG_DEBUG(
                            Weapon,
                            "Left firing-grip start waiting for authored support readiness={} collisionGeneration={:016X} authoredGeneration={:016X} ownership={:016X}",
                            authored_support_grab_policy::
                                leftFiringTakeoverReadinessName(
                                    leftTakeoverReadiness),
                            currentWeaponGenerationKey,
                            currentAuthoredGripGenerationKey,
                            currentEquippedWeaponOwnershipKey);
                    }
                } else if (primaryOnlyStartRequested &&
                    _twoHandedGrip.beginPrimaryOnlyGrip(
                        weaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        firingHandIsLeft,
                        capturedFiringHandWeaponLocal,
                        capturedFiringGripWeaponLocal,
                        retainUntilPhysicalGrip)) {
                    primaryOnlyGripStartedThisFrame = true;
                    _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                    primaryGripInput = EquippedWeaponPrimaryGripInput{
                        .held = primaryState.held,
                        .pressed = primaryState.pressed,
                        .released = primaryState.released,
                    };
                }
            } else if (inputBlockingMenuActive) {
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            }

            std::array<const RE::NiAVObject*, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1> drivenSourceNodes{};
            std::size_t drivenSourceNodeCount = 0;
            if (weaponNode) {
                drivenSourceNodeCount = applyProviderWeaponPartDrives(
                    weaponNode,
                    currentWeaponGenerationKey,
                    frame,
                    drivenSourceNodes);
            } else {
                _providerWeaponPartDriveResultCount = 0;
            }

            /*
             * Firing-grip reattach is the squeeze gesture (grab held with the
             * palm on the grip); distance is evaluated by TwoHandedGrip. This
             * only gates whether each free hand may be captured at all -
             * either hand can take the firing grip when ambidextrous takeover
             * is available.
             */
            bool leftReattachEligible = false;
            bool rightReattachEligible = false;
            if (_twoHandedGrip.isPartCarryActive() && primaryDetachFeatureAvailable) {
                leftReattachEligible = weapon_two_handed_grip_math::canAttemptFiringGripReattach(
                    weapon_two_handed_grip_math::FiringGripReattachInput{
                        .partCarryActive = true,
                        .menuInputActive = inputBlockingMenuActive,
                        .handHoldingObject = _leftHand.isHolding(),
                    });
                rightReattachEligible = weapon_two_handed_grip_math::canAttemptFiringGripReattach(
                    weapon_two_handed_grip_math::FiringGripReattachInput{
                        .partCarryActive = true,
                        .menuInputActive = inputBlockingMenuActive,
                        .handHoldingObject = _rightHand.isHolding(),
                    });
            }

            const bool nativeShoulderSheathRequested =
                equippedWeaponShoulderFrame.decision.action ==
                equipped_weapon_shoulder::Action::SubmitSheath;
            const auto nativeShoulderSheathSourceHand =
                equippedWeaponShoulderFrame.sourceHand;
            const auto& nativeShoulderSheathDecision =
                equippedWeaponShoulderFrame.detectorDecision;

            const auto toToggleButtonState = [](const GrabButtonState& state) {
                return equipped_weapon_toggle_grab_policy::ButtonState{
                    .held = state.held,
                    .pressed = state.pressed,
                    .released = state.released,
                };
            };
            const auto toggleGrabDecision =
                equipped_weapon_toggle_grab_policy::prepare(
                    _equippedWeaponToggleGrabState,
                    equipped_weapon_toggle_grab_policy::Input{
                        .enabled = _equippedWeaponHandlingSettings.
                            toggleGrabEnabled,
                        .inputAllowed = !inputBlockingMenuActive,
                        .weaponOwnershipKey =
                            currentEquippedWeaponOwnershipKey,
                        .occupancy = {
                            .left = toggleOccupancyBefore.left.
                                weaponEngaged(),
                            .right = toggleOccupancyBefore.right.
                                weaponEngaged(),
                        },
                        .left = toToggleButtonState(
                            leftPhysicalGripState),
                        .right = toToggleButtonState(
                            rightPhysicalGripState),
                    });
            if (_equippedWeaponHandlingSettings.toggleGrabEnabled) {
                leftGripHeld = toggleGrabDecision.left.held;
                rightGripHeld = toggleGrabDecision.right.held;
                const auto& logicalPrimaryGrip = firingHandIsLeft ?
                    toggleGrabDecision.left : toggleGrabDecision.right;
                primaryGripInput = EquippedWeaponPrimaryGripInput{
                    .held = logicalPrimaryGrip.held,
                    .pressed = logicalPrimaryGrip.pressed,
                    .released = logicalPrimaryGrip.released,
                };
                _equippedWeaponToggleGrabReleasePressConsumedThisFrame[
                    equipped_weapon_toggle_grab_policy::handIndex(true)] =
                    _equippedWeaponToggleGrabReleasePressConsumedThisFrame[
                        equipped_weapon_toggle_grab_policy::handIndex(true)] ||
                    toggleGrabDecision.leftReleasePressConsumed;
                _equippedWeaponToggleGrabReleasePressConsumedThisFrame[
                    equipped_weapon_toggle_grab_policy::handIndex(false)] =
                    _equippedWeaponToggleGrabReleasePressConsumedThisFrame[
                        equipped_weapon_toggle_grab_policy::handIndex(false)] ||
                    toggleGrabDecision.rightReleasePressConsumed;
            }

            const auto captureScopeHandDriverFrame = [](RE::NiNode* driverNode) {
                EquippedWeaponScopeHandDriverFrame result{};
                result.nodeAvailable = driverNode != nullptr;
                if (driverNode) {
                    result.world = driverNode->world;
                    result.worldFinite = finiteNiTransform(driverNode->world);
                    result.valid = result.worldFinite;
                }
                return result;
            };
            auto* playerNodes = f4vr::getPlayerNodes();
            const auto scopeHandDriverNode = [playerNodes](bool isLeft) -> RE::NiNode* {
                if (!playerNodes) {
                    return nullptr;
                }
                return isLeft ?
                    playerNodes->SecondaryMeleeWeaponOffsetNode2 :
                    playerNodes->primaryWeaponOffsetNOde;
            };
            const EquippedWeaponScopeHandDriverFrame leftHandDriverFrame = captureScopeHandDriverFrame(scopeHandDriverNode(true));
            const EquippedWeaponScopeHandDriverFrame rightHandDriverFrame = captureScopeHandDriverFrame(scopeHandDriverNode(false));
            bool nativeScopeRequestActive = false;
            const bool nativeScopeRequestStateValid =
                tryReadNativeScopeRequestState(nativeScopeRequestActive);
            const bool manualScopeActivationRequested =
                input_remap_runtime::isManualScopeActivationRequested();
            const EquippedWeaponGripFrameInput gripFrameInput{
                .leftGripHeld = leftGripHeld,
                .rightGripHeld = rightGripHeld,
                .leftHandHoldingObject = leftHandHoldingObject,
                .rightHandHoldingObject = _rightHand.isHolding(),
                .leftReattachEligible = leftReattachEligible,
                .rightReattachEligible = rightReattachEligible,
                .scopeMenuOpen = runtime.localScopeMenuOpen,
                .manualScopeActivationRequested = manualScopeActivationRequested,
                .nativeScopeRequestStateValid = nativeScopeRequestStateValid,
                .nativeScopeRequestActive = nativeScopeRequestActive,
                .leftHandDriverFrame = leftHandDriverFrame,
                .rightHandDriverFrame = rightHandDriverFrame,
                .primaryGripInput = primaryGripInput,
                .leftPhysicalGripInput = EquippedWeaponPrimaryGripInput{
                    .held = leftPhysicalGripState.held,
                    .pressed = leftPhysicalGripState.pressed,
                    .released = leftPhysicalGripState.released,
                },
                .rightPhysicalGripInput = EquippedWeaponPrimaryGripInput{
                    .held = rightPhysicalGripState.held,
                    .pressed = rightPhysicalGripState.pressed,
                    .released = rightPhysicalGripState.released,
                },
                .hmdPositionWorld = frame.hmdPositionWorld,
                .toggleGrabEnabled =
                    _equippedWeaponHandlingSettings.toggleGrabEnabled,
                .animationBoundaryActive = frame.reloadBoundaryActive,
                .hasHmdFrame = frame.hasHmdFrame,
            };
            auto effectiveHandlingSettings = _equippedWeaponHandlingSettings;
            effectiveHandlingSettings.firingGripOwnershipEnabled =
                firingGripOwnershipFeatureAvailable;
            effectiveHandlingSettings.ambidextrousHandoffEnabled =
                ambidextrousHandoffAvailable;
            effectiveHandlingSettings.primaryDetachEnabled =
                primaryDetachFeatureAvailable;
            effectiveHandlingSettings.detachAuthority =
                primaryDetachFeatureAvailable ?
                firingGripDecision.authority :
                immersive_weapon_policy::DetachAuthority::None;
            effectiveHandlingSettings.preserveWeaponPoseOnDetach =
                primaryDetachFeatureAvailable &&
                firingGripDecision.preserveWeaponPoseOnDetach;
            effectiveHandlingSettings.firingGripReattachRadiusGameUnits =
                firingGripDecision.reattachRadiusGameUnits;
            effectiveHandlingSettings.weaponGripHapticDurationSeconds =
                firingGripDecision.gripHapticDurationSeconds;
            effectiveHandlingSettings.firingGripAttachHapticIntensity =
                firingGripDecision.gripAttachHapticIntensity;
            effectiveHandlingSettings.firingGripDetachHapticIntensity =
                firingGripDecision.gripDetachHapticIntensity;
            const TwoHandedGripUpdateResult gripUpdateResult =
                _twoHandedGrip.update(
                    weaponNode,
                    leftWeaponContact,
                    rightWeaponContact,
                    gripFrameInput,
                    frame.deltaSeconds,
                    currentAuthoredGripGenerationKey,
                    currentEquippedWeaponOwnershipKey,
                    _weaponCollision,
                    providerInteractionState,
                    rightHandInteractionState,
                    supportAuthorityMode,
                    firingGripProximityAuthorityEnabled,
                    effectiveHandlingSettings);
            auto toggleOccupancyAfter = gripUpdateResult.after;
            if (_twoHandedGrip.
                    isPersistentEquippedCarryInputAcquisitionPending()) {
                auto& pendingFiringOccupancy =
                    _twoHandedGrip.isFiringHandLeft() ?
                    toggleOccupancyAfter.left :
                    toggleOccupancyAfter.right;
                pendingFiringOccupancy.firingGripActive = false;
            }
            const auto toggleReconcileDecision =
                equipped_weapon_toggle_grab_policy::reconcile(
                    _equippedWeaponToggleGrabState,
                    _equippedWeaponHandlingSettings.toggleGrabEnabled,
                    currentEquippedWeaponOwnershipKey,
                    equipped_weapon_toggle_grab_policy::GripOccupancy{
                        .left = toggleOccupancyAfter.left.weaponEngaged(),
                        .right = toggleOccupancyAfter.right.weaponEngaged(),
                    });
            const auto consumeToggleAcquisitionPress =
                [this](const bool isLeft, const bool acquired) {
                if (!acquired) {
                    return;
                }

                (void)_twoHandedGrip.
                    commitPersistentEquippedCarryInputAcquisition(isLeft);

                GrabButtonState acquisitionInput{};
                if (_firingHandGrabButtonFrameState.valid &&
                    _firingHandGrabButtonFrameState.isLeft == isLeft) {
                    acquisitionInput = GrabButtonState{
                        .held = _firingHandGrabButtonFrameState.held,
                        .pressed = _firingHandGrabButtonFrameState.pressed,
                        .released = _firingHandGrabButtonFrameState.released,
                    };
                    _firingHandGrabButtonFrameState.valid = false;
                } else {
                    acquisitionInput = readGrabButtonState(
                        isLeft,
                        input_remap_policy::kGrabButtonId);
                }

                ROCK_LOG_DEBUG(Weapon,
                    "Equipped weapon toggle grab armed hand={} acquisitionPress={} physicalHeld={} physicalReleased={}",
                    isLeft ? "left" : "right",
                    acquisitionInput.pressed ? "yes" : "no",
                    acquisitionInput.held ? "yes" : "no",
                    acquisitionInput.released ? "yes" : "no");
            };
            // A successful weapon grip owns the press that acquired it. Drain
            // that edge before normal-grab suppression can leave it pending;
            // only a later physical press may toggle this grip open.
            consumeToggleAcquisitionPress(
                true,
                toggleReconcileDecision.leftGripAcquired);
            consumeToggleAcquisitionPress(
                false,
                toggleReconcileDecision.rightGripAcquired);
            if (_twoHandedGrip.hasVisualAuthorityForHand(false)) {
                _rightHand.cancelGrabVisualReturn("equipped-weapon-visual-authority");
            }
            if (_twoHandedGrip.hasVisualAuthorityForHand(true)) {
                _leftHand.cancelGrabVisualReturn("equipped-weapon-visual-authority");
            }
            if (primaryOnlyGripStartedThisFrame) {
                ROCK_LOG_DEBUG(Weapon, "Equipped weapon firing-grip ownership started from grip input or held-weapon equip");
            }
            const auto gripHapticEvents = _twoHandedGrip.consumeHapticEvents();
            const auto queueGripHaptic = [this](
                                             const bool isLeft,
                                             const float durationSeconds,
                                             const float intensity) {
                (void)_feedbackHaptics.queue(
                    isLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                    durationSeconds,
                    intensity);
            };
            const auto queueFiringGripHaptic = [this, &queueGripHaptic](
                                                    const bool isLeft,
                                                    const bool attached) {
                const auto eventDecision =
                    resolveEquippedWeaponDetachDecision(
                        _equippedWeaponHandlingSettings);
                if (eventDecision.authority ==
                        immersive_weapon_policy::DetachAuthority::
                            IntegratedImmersive) {
                    queueGripHaptic(
                        isLeft,
                        eventDecision.gripHapticDurationSeconds,
                        attached ?
                            eventDecision.gripAttachHapticIntensity :
                            eventDecision.gripDetachHapticIntensity);
                    return;
                }

                if (_equippedWeaponHandlingSettings.externalAuthorityActive) {
                    queueGripHaptic(
                        isLeft,
                        _equippedWeaponHandlingSettings.
                            weaponGripHapticDurationSeconds,
                        attached ?
                            _equippedWeaponHandlingSettings.
                                firingGripAttachHapticIntensity :
                            _equippedWeaponHandlingSettings.
                                firingGripDetachHapticIntensity);
                }
            };
            if (gripHapticEvents.firingGripAttached) {
                queueFiringGripHaptic(
                    gripHapticEvents.firingGripAttachedHandIsLeft,
                    true);
            }
            if (gripHapticEvents.firingGripDetached) {
                queueFiringGripHaptic(
                    gripHapticEvents.firingGripDetachedHandIsLeft,
                    false);
            }
            if (_equippedWeaponHandlingSettings.externalAuthorityActive) {
                if (gripHapticEvents.leftPartGripCaptured) {
                    queueGripHaptic(
                        true,
                        _equippedWeaponHandlingSettings.
                            weaponGripHapticDurationSeconds,
                        _equippedWeaponHandlingSettings.supportGripHapticIntensity);
                }
                if (gripHapticEvents.rightPartGripCaptured) {
                    queueGripHaptic(
                        false,
                        _equippedWeaponHandlingSettings.
                            weaponGripHapticDurationSeconds,
                        _equippedWeaponHandlingSettings.supportGripHapticIntensity);
                }
            }
            /*
             * Continuous hover feedback while the open firing palm sits inside
             * the reattach radius during part carry: re-queued every frame so
             * the vibration holds until the squeeze reattaches (which flips
             * the state and hands off to the firingGripAttached pulse above).
             */
            if (_equippedWeaponHandlingSettings.gripZoneHoverHapticsEnabled &&
                _twoHandedGrip.isFiringGripReattachHoverInsideRadius()) {
                (void)_feedbackHaptics.queue(
                    _twoHandedGrip.isFiringGripReattachHoverHandLeft() ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                    grip_zone_hover_haptic_policy::kContinuousQueueSeconds,
                    _equippedWeaponHandlingSettings.gripZoneHoverHapticIntensity);
            }
            bool nativeShoulderSheathSelected = false;
            if (nativeShoulderSheathRequested) {
                nativeShoulderSheathSelected = true;
                bool sheathAccepted = false;
                if (nativeShoulderSheathSourceHand !=
                    equipped_weapon_drop_policy::SourceHand::None) {
                    sheathAccepted = submitEquippedWeaponShoulderSheath(
                        observedEquippedWeaponFormID,
                        observedEquippedWeaponInstanceData,
                        nativeShoulderSheathSourceHand,
                        nativeShoulderSheathDecision,
                        weaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey);
                } else {
                    ROCK_LOG_ERROR(
                        Weapon,
                        "Equipped shoulder coordinator selected a sheath without a source hand");
                }
                equipped_weapon_shoulder::reportExecutionResult(
                    _equippedWeaponShoulderCoordinatorState,
                    equipped_weapon_shoulder::Action::SubmitSheath,
                    sheathAccepted);
            }
            const auto equippedWeaponDropRequest = _twoHandedGrip.consumeEquippedWeaponDropRequest();
            if (equippedWeaponDropRequest.requested) {
                const auto sourceHand = equippedWeaponDropRequest.sourceHand;
                const bool sourceHandKnown = sourceHand == equipped_weapon_drop_policy::SourceHand::Right ||
                                              sourceHand == equipped_weapon_drop_policy::SourceHand::Left;
                const RE::NiPoint3 dropLoc = sourceHandKnown ?
                                                (equipped_weapon_drop_policy::isLeft(sourceHand) ? frame.left.grabAnchorWorld : frame.right.grabAnchorWorld) :
                                                (weaponNode ? weaponNode->world.translate : frame.right.grabAnchorWorld);
                if (inputBlockingMenuActive) {
                    ROCK_LOG_INFO(Weapon,
                        "Equipped weapon manual release suppressed because an input-blocking menu is active sourceHand={} releaseLoc=({:.1f},{:.1f},{:.1f})",
                        equipped_weapon_drop_policy::sourceHandName(sourceHand),
                        dropLoc.x,
                        dropLoc.y,
                        dropLoc.z);
                    _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                    clearEquippedWeaponFiringGripInputState();
                } else {
                    const bool stashCommitSelected =
                        nativeShoulderSheathSelected ||
                        equippedWeaponShoulderFrame.decision.
                            suppressEquippedDrop;
                    const bool physicalDropRequested =
                        equipped_weapon_drop_policy::shouldAttemptPhysicalDrop(stashCommitSelected);
                    const bool dropHandoffAvailable = hasAvailableEquippedWeaponDropHandoff();
                    if (physicalDropRequested && !dropHandoffAvailable) {
                        ROCK_LOG_WARN(Weapon,
                            "Equipped weapon physical drop blocked because all native handoffs are active: capacity={}",
                            _equippedWeaponDropMomentumHandoffs.size());
                        f4vr::showNotification("ROCK: Cannot drop weapon - drop handoff queue is full.");
                    }
                    if (physicalDropRequested && dropHandoffAvailable) {
                        /*
                         * Seamless drop: spawn the world ref at the weapon's last
                         * visually-published pose (equipped and dropped weapons
                         * share the same nif) and hand the captured release
                         * momentum to the spawned physics bodies once they
                         * resolve. The previous-frame capture is preferred over
                         * the live node because the release transition restores
                         * the weapon node to the FRIK hand baseline before this
                         * code runs.
                         */
                        RE::NiPoint3 releaseLoc = dropLoc;
                        RE::NiPoint3 releaseRot{};
                        RE::NiTransform releaseWeaponWorld{};
                        bool hasReleaseRot = false;
                        if (_equippedWeaponReleaseCapture.hasWeaponWorld &&
                            finiteNiTransform(_equippedWeaponReleaseCapture.weaponWorld)) {
                            releaseWeaponWorld = _equippedWeaponReleaseCapture.weaponWorld;
                            releaseLoc = _equippedWeaponReleaseCapture.weaponWorld.translate;
                            releaseRot = transform_math::matrixToEulerRadians<RE::NiMatrix3, RE::NiPoint3>(_equippedWeaponReleaseCapture.weaponWorld.rotate);
                            hasReleaseRot = true;
                        } else if (weaponNode && finiteNiTransform(weaponNode->world)) {
                            releaseWeaponWorld = weaponNode->world;
                            releaseLoc = weaponNode->world.translate;
                            releaseRot = transform_math::matrixToEulerRadians<RE::NiMatrix3, RE::NiPoint3>(weaponNode->world.rotate);
                            hasReleaseRot = true;
                        }
                        const std::size_t releaseHandIndex = equipped_weapon_drop_policy::isLeft(sourceHand) ? 1u : 0u;
                        const auto& releaseHandInput = releaseHandIndex == 1u ? frame.left : frame.right;
                        const RE::NiPoint3 releaseGripWorld = _equippedWeaponReleaseCapture.hasPreviousHandWorld[releaseHandIndex] ?
                                                                 _equippedWeaponReleaseCapture.previousHandWorld[releaseHandIndex].translate :
                                                                 releaseHandInput.grabAnchorWorld;
                        // Consume the equipped body's generated points before
                        // the drop transaction retires that bank. They only
                        // bound long-object angular release speed; the frozen
                        // transform is the native body's placement authority.
                        const auto releaseGeometry = hasReleaseRot ?
                                                         _weaponCollision.getCurrentWeaponReleaseGeometry(releaseGripWorld, releaseWeaponWorld) :
                                                         WeaponCollision::ReleaseGeometrySnapshot{};
                        if (!releaseGeometry.hasCapturedWeaponWorld) {
                            ROCK_LOG_WARN(Weapon,
                                "Equipped weapon physical drop blocked because no finite frozen release pose is available: sourceHand={}",
                                equipped_weapon_drop_policy::sourceHandName(sourceHand));
                            f4vr::showNotification("ROCK: Cannot drop weapon - release pose is not ready.");
                        } else {
                            const auto dropResult = weapon_equip_transfer::dropEquippedWeaponFromPlayer(weapon_equip_transfer::EquippedDropInput{
                                .dropLoc = releaseLoc,
                                .dropRot = releaseRot,
                                .hasDropLoc = true,
                                .hasDropRot = true,
                            });
                            const bool dropCommitted = equipped_weapon_drop_policy::physicalDropCommitted(
                                equipped_weapon_drop_policy::PhysicalDropCommitInput{
                                    .dropSucceeded = dropResult.success,
                                    .droppedReferenceUnavailable =
                                        dropResult.reason == weapon_equip_transfer::DropReason::DroppedReferenceUnavailable,
                                });
                            if (dropCommitted) {
                                enforceNoBareFistState(true);
                                /*
                                 * RemoveItem creates the native layer-5 weapon at
                                 * the last layer-44 equipped-collider pose. Retire
                                 * ROCK's generated representation in this same
                                 * transaction so no physics step can solve the two
                                 * coincident weapon body sets before the native
                                 * handoff takes ownership.
                                 */
                                _weaponCollision.destroyWeaponBody(hknp);
                            }
                            if (dropCommitted && dropResult.handle) {
                                armEquippedWeaponDropMomentumHandoff(
                                    dropResult.handle,
                                    dropResult.droppedFormID,
                                    sourceHand,
                                    releaseGeometry);
                            }
                            if (dropCommitted) {
                                ROCK_LOG_INFO(Weapon,
                                    "Equipped weapon manual release committed formID={:08X} dropped={:08X} reference={} sourceHand={} dropLoc=({:.1f},{:.1f},{:.1f}) lever={:.1f}gu stack={} instanceMatch={}",
                                    dropResult.formID,
                                    dropResult.droppedFormID,
                                    dropResult.success ? "ready" : "pending",
                                    equipped_weapon_drop_policy::sourceHandName(sourceHand),
                                    releaseLoc.x,
                                    releaseLoc.y,
                                    releaseLoc.z,
                                    releaseGeometry.leverGameUnits,
                                    dropResult.stackID,
                                    dropResult.matchedInstanceData ? "yes" : "no");
                            } else {
                                ROCK_LOG_WARN(Weapon,
                                    "Equipped weapon manual release drop failed formID={:08X} reason={} sourceHand={} attempted={} stack={} instanceMatch={}",
                                    dropResult.formID,
                                    weapon_equip_transfer::dropReasonName(dropResult.reason),
                                    equipped_weapon_drop_policy::sourceHandName(sourceHand),
                                    dropResult.attempted ? "yes" : "no",
                                    dropResult.stackID,
                                    dropResult.matchedInstanceData ? "yes" : "no");
                            }
                            if (sourceHandKnown && dropCommitted) {
                                suppressHandCollisionAfterEquippedWeaponDrop(hknp, sourceHand);
                            }
                        }
                    }
                    _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                    clearEquippedWeaponFiringGripInputState();
                }
            }
            updateEquippedWeaponReleaseCapture(frame, weaponNode);
            const bool weaponSupportGripActive =
                gripUpdateResult.after.left.partGripActive;
            const input_remap_policy::EquippedWeaponFiringGripInputGate updatedFiringGripInputGate{
                .featureAvailable = firingGripOwnershipFeatureAvailable,
                .canUseFiringGripInput = _twoHandedGrip.canUseFiringGripInput(),
                .menuInputActive = inputBlockingMenuActive,
            };
            input_remap_runtime::setEquippedWeaponFiringGripInputActive(
                input_remap_policy::shouldUseEquippedWeaponFiringGripInput(updatedFiringGripInputGate));
            input_remap_runtime::setEquippedWeaponPrimaryDetached(_twoHandedGrip.isPartCarryActive());
            /*
             * Left-hand fire publication: while the LEFT hand occupies the
             * firing grip, the OpenVR-level trigger remap presents the left
             * trigger to the game as the primary (right) wand's trigger.
             */
            const bool leftHandFiringActiveAfterGrip =
                gripUpdateResult.after.left.firingGripActive;
            input_remap_runtime::setEquippedWeaponLeftHandFiringActive(leftHandFiringActiveAfterGrip);
            ::rock::provider::setEquippedWeaponFiringHandIsLeft(_twoHandedGrip.isFiringHandLeft());

            bool rightHandWeaponAuthorityActiveAfterGrip = rightHandWeaponEquipped || retainedWeaponCollisionActive;
            // A visible part-carry frees the right hand even while weapon bodies exist (see the pre-grip gate).
            if (rightHandWeaponEquipped && _twoHandedGrip.isPartCarryActive()) {
                rightHandWeaponAuthorityActiveAfterGrip = false;
            }
            // Left-firing carry frees the right hand the same way (see the pre-grip gate).
            if (rightHandWeaponEquipped && leftHandFiringActiveAfterGrip) {
                rightHandWeaponAuthorityActiveAfterGrip = false;
            }
            if (rightHandWeaponAuthorityActiveAfterGrip != rightHandWeaponAuthorityActiveBeforeGrip) {
                if (rightHandWeaponAuthorityActiveAfterGrip) {
                    suppressRightHandCollisionForDominantWeapon(hknp);
                } else {
                    restoreRightHandCollisionAfterDominantWeapon(hknp);
                }
            }
            rightHandWeaponAuthorityActive = rightHandWeaponAuthorityActiveAfterGrip;
            const bool rightPartGripActiveAfterGrip =
                gripUpdateResult.after.right.partGripActive;
            if (rightPartGripActiveAfterGrip !=
                gripUpdateResult.before.right.partGripActive) {
                if (rightPartGripActiveAfterGrip) {
                    suppressHandCollisionForWeaponSupport(hknp, false);
                } else {
                    beginDelayedHandCollisionRestoreAfterWeaponSupport(
                        hknp,
                        false);
                }
            }
            rightPartGripActive = rightPartGripActiveAfterGrip;

            if (g_rockConfig.rockDebugShowWeaponNotifications) {
                const auto gripNotificationEvent =
                    weapon_debug_notification_policy::observeWeaponSupportGrip(_weaponDebugNotificationState, weaponSupportGripActive);
                if (gripNotificationEvent != weapon_debug_notification_policy::WeaponGripNotificationEvent::None) {
                    if (gripNotificationEvent == weapon_debug_notification_policy::WeaponGripNotificationEvent::Started) {
                        const auto weaponDebugInfo = makeWeaponInteractionDebugInfo(_weaponCollision, weaponNode, leftWeaponContact);
                        f4vr::showNotification(
                            weapon_debug_notification_policy::formatWeaponGripNotification(gripNotificationEvent, weaponNotificationKey, weaponDebugInfo));
                        ROCK_LOG_INFO(Weapon,
                            "WeaponGripDiagnostics: weapon='{}' formID={:08X} node='{}' driveRoot='{}' sourceRoot='{}' nif='{}' part={} route={} pose={} body={} source={}",
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.weaponName),
                            weaponDebugInfo.weaponFormId,
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.weaponNodeName),
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.interactionRootName),
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.sourceRootName),
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.sourceName),
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.partKind),
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.interactionKind),
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.gripPose),
                            weaponNotificationKey.bodyId,
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.source));
                    } else {
                        f4vr::showNotification(weapon_debug_notification_policy::formatWeaponGripNotification(gripNotificationEvent, weaponNotificationKey));
                    }
                }
            } else {
                _weaponDebugNotificationState.supportGripActive = weaponSupportGripActive;
            }
            leftSupportGripActive = weaponSupportGripActive;

            /*
             * A LEFT hand occupying the firing grip is weapon-engaged exactly
             * like a support hand from the collision standpoint: its generated
             * colliders must not become a second physical owner while the
             * weapon rides the hand. Reuses the per-hand support lease.
             */
            const bool leftWeaponGripActiveAfterUpdate =
                weaponSupportGripActive || leftHandFiringActiveAfterGrip;
            if (leftWeaponGripActiveAfterUpdate) {
                suppressHandCollisionForWeaponSupport(hknp, true);
            } else if (
                gripUpdateResult.before.left.weaponEngaged()) {
                beginDelayedHandCollisionRestoreAfterWeaponSupport(
                    hknp,
                    true);
            } else {
                restoreHandCollisionAfterWeaponSupport(hknp, true);
            }

            const auto dynamicWeaponFrame =
                _dynamicWeaponCollision.finishFrame(
                    frame,
                    physicsWritesAllowedForWorld(frame.hknpWorld),
                    weaponNode,
                    currentWeaponGenerationKey,
                    _weaponCollision);
            const bool dynamicWeaponDebugEnabled =
                g_rockConfig.rockDebugShowColliders &&
                g_rockConfig.rockDebugDrawDynamicWeaponColliders;
            if (dynamicWeaponFrame.contactEpisodeStarted &&
                dynamicWeaponDebugEnabled) {
                auto* otherRef = resolveBodyToRef(
                    frame.bhkWorld,
                    frame.hknpWorld,
                    RE::hknpBodyId{ dynamicWeaponFrame.otherBodyId });
                const auto* otherBase = otherRef ? otherRef->GetObjectReference() : nullptr;
                const auto otherNameView = otherBase ?
                    RE::TESFullName::GetFullName(*otherBase, false) :
                    std::string_view{};
                const std::string otherName = otherNameView.empty() ?
                    std::string("(unresolved)") :
                    std::string(otherNameView);
                const char* otherType = otherBase ?
                    otherBase->GetFormTypeString() :
                    "unresolved";

                WeaponCollision::WeaponSurfaceProximityWitness partWitness{};
                WeaponInteractionDebugInfo partInfo{};
                constexpr float kContactPartSearchRadiusGameUnits = 24.0f;
                const bool partWitnessValid =
                    dynamicWeaponFrame.rawContactPointValid &&
                    _weaponCollision.tryFindCurrentWeaponSurfaceNearPoint(
                        weaponNode,
                        dynamicWeaponFrame.rawContactPointGame,
                        kContactPartSearchRadiusGameUnits,
                        partWitness);
                const bool partInfoValid =
                    partWitnessValid &&
                    _weaponCollision.tryGetWeaponContactDebugInfo(
                        partWitness.bodyId,
                        partInfo);

                ROCK_LOG_INFO(
                    Weapon,
                    "DWC contact witness: episode={} solveAge={} generation={:016X} weaponForm={:08X} other(body/layer/motion/ref/form/type/name)=({}/{}/{}/{:p}/{:08X}/{}/{}) native(collObj/owner)=({:p}/{:p}) raw(valid/proxyWasA/points/index/weight)={}/{}/{}/{}/{:.3f} pointGame=({:.2f},{:.2f},{:.2f}) normalRaw=({:.3f},{:.3f},{:.3f}) nearestPart(valid/body/source/distance/current)={}/{}/{}/{:.3f}/{}",
                    dynamicWeaponFrame.contactEpisode,
                    dynamicWeaponFrame.contactSolveAge,
                    currentWeaponGenerationKey,
                    _weaponCollision.getCurrentObservedEquippedWeaponFormID(),
                    dynamicWeaponFrame.otherBodyId,
                    dynamicWeaponFrame.otherLayer,
                    dynamicWeaponFrame.otherMotionIndex,
                    static_cast<void*>(otherRef),
                    otherRef ? otherRef->GetFormID() : 0,
                    otherType,
                    otherName,
                    reinterpret_cast<void*>(dynamicWeaponFrame.otherCollisionObject),
                    reinterpret_cast<void*>(dynamicWeaponFrame.otherOwnerNode),
                    dynamicWeaponFrame.rawContactPointValid,
                    dynamicWeaponFrame.rawContactProxyWasBodyA,
                    dynamicWeaponFrame.rawContactPointCount,
                    dynamicWeaponFrame.rawContactPointIndex,
                    dynamicWeaponFrame.rawContactPointWeightSum,
                    dynamicWeaponFrame.rawContactPointGame.x,
                    dynamicWeaponFrame.rawContactPointGame.y,
                    dynamicWeaponFrame.rawContactPointGame.z,
                    dynamicWeaponFrame.rawContactNormalHavok.x,
                    dynamicWeaponFrame.rawContactNormalHavok.y,
                    dynamicWeaponFrame.rawContactNormalHavok.z,
                    partWitnessValid,
                    partWitnessValid ? partWitness.bodyId : 0x7FFF'FFFFu,
                    partInfoValid ? partInfo.sourceName : std::string("(unresolved)"),
                    partWitnessValid ? partWitness.distanceGameUnits : -1.0f,
                    partWitnessValid && partWitness.sourceNodeCurrent);
                ROCK_LOG_INFO(
                    Weapon,
                    "DWC contact transforms: episode={} requestedBody=({:.2f},{:.2f},{:.2f}) liveBody=({:.2f},{:.2f},{:.2f}) otherReadable={} otherBody=({:.2f},{:.2f},{:.2f}) correction=({:.2f}gu,{:.2f}deg)",
                    dynamicWeaponFrame.contactEpisode,
                    dynamicWeaponFrame.requestedContactBodyWorld.translate.x,
                    dynamicWeaponFrame.requestedContactBodyWorld.translate.y,
                    dynamicWeaponFrame.requestedContactBodyWorld.translate.z,
                    dynamicWeaponFrame.liveContactBodyWorld.translate.x,
                    dynamicWeaponFrame.liveContactBodyWorld.translate.y,
                    dynamicWeaponFrame.liveContactBodyWorld.translate.z,
                    dynamicWeaponFrame.otherBodyWorldValid,
                    dynamicWeaponFrame.otherBodyWorld.translate.x,
                    dynamicWeaponFrame.otherBodyWorld.translate.y,
                    dynamicWeaponFrame.otherBodyWorld.translate.z,
                    dynamicWeaponFrame.translationCorrectionGameUnits,
                    dynamicWeaponFrame.rotationCorrectionDegrees);
            }
            if (dynamicWeaponFrame.applyVisualCorrection) {
                const bool visualPublishSucceeded = _twoHandedGrip.applyWeaponCollisionResolvedAuthority(
                    weaponNode,
                    dynamicWeaponFrame.resolvedWeaponWorld,
                    currentWeaponGenerationKey);
                const float immediateTranslationError =
                    visualPublishSucceeded && weaponNode ?
                        dynamic_weapon_collision_policy::translationDeltaGameUnits(
                            weaponNode->world,
                            dynamicWeaponFrame.resolvedWeaponWorld) :
                        -1.0f;
                const float immediateRotationError =
                    visualPublishSucceeded && weaponNode ?
                        dynamic_weapon_collision_policy::rotationDeltaDegrees(
                            weaponNode->world,
                            dynamicWeaponFrame.resolvedWeaponWorld) :
                        -1.0f;
                if (dynamicWeaponDebugEnabled) {
                    ROCK_LOG_SAMPLE_INFO(
                        Weapon,
                        500,
                        "DWC visual publication: bodyActive={} publishSucceeded={} immediateError=({:.3f}gu,{:.3f}deg) requestedCorrection=({:.3f}gu,{:.3f}deg)",
                        dynamicWeaponFrame.proxyActive,
                        visualPublishSucceeded,
                        immediateTranslationError,
                        immediateRotationError,
                        dynamicWeaponFrame.translationCorrectionGameUnits,
                        dynamicWeaponFrame.rotationCorrectionDegrees);
                }
            }
            if (weaponNode) {
                performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::WeaponCollisionTransforms);
                _weaponCollision.updateBodiesFromCurrentSourceTransforms(
                    hknp,
                    weaponNode,
                    frame.deltaSeconds,
                    drivenSourceNodes.data(),
                    drivenSourceNodeCount);
            }
            if (f4vr::isNodeVisible(weaponNode)) {
                applyFinalWeaponMuzzleAuthority();
            }
        }

        return EquippedWeaponFrameResult{
            .rightHandWeaponAuthorityActive = rightHandWeaponAuthorityActive,
            .leftSupportGripActive = leftSupportGripActive,
            .rightPartGripActive = rightPartGripActive,
        };
    }

    void PhysicsInteraction::updateAuthoredSupportGripIndicator()
    {
        const auto frame =
            _twoHandedGrip.getAuthoredSupportGripIndicatorFrame();
        const Hand& supportHand =
            frame.supportHandIsLeft ? _leftHand : _rightHand;
        if (!frame.visible || supportHand.isHolding()) {
            _authoredSupportGripIndicator.hide();
            return;
        }
        (void)_authoredSupportGripIndicator.update(frame.positionWorld);
    }

    void PhysicsInteraction::updateAuthoredPrimaryFiringGrip()
    {
        const auto& runtime = runtime_state::currentFrame();
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        // The authored/native canonical weapon frame is always ROCK's
        // physical-right primary controller, independent of FO4VR settings.
        const bool leftHandHoldingObject = _leftHand.isHolding();
        const bool rightHandHoldingObject = _rightHand.isHolding();
        _twoHandedGrip.setGrabbedObjectHandPoseOwnership(
            leftHandHoldingObject,
            rightHandHoldingObject);
        const auto nativeAuthorityFlags =
            provider::currentNativeAnimationAuthorityFlagsV1();
        auto* equippedWeapon = currentEquippedWeaponForm();
        const auto weaponClassification =
            _weaponCollision.getEquippedWeaponClassification();
        const std::uint64_t weaponKeywordFlags =
            equippedWeapon &&
                weaponClassification.hasEquippedWeapon &&
                weaponClassification.formID == equippedWeapon->formID ?
            weaponClassification.keywordFlags :
            0;
        std::uint64_t weaponOwnershipKey =
            weaponNode ? _weaponCollision.getCurrentEquippedWeaponOwnershipKey() : 0;
        if (weaponNode && weaponOwnershipKey == 0) {
            // Keep authored grip alignment independent of generated weapon
            // collision.
            // The richer stack/instance key wins when available; the equipped
            // form remains a stable freshness boundary when collision is off.
            weaponOwnershipKey = currentEquippedWeaponFormId();
        }
        const std::uint64_t collisionGenerationKey =
            weaponNode ? _weaponCollision.getCurrentWeaponGenerationKey() : 0;
        const std::uint64_t weaponGenerationKey =
            authored_support_grab_policy::resolveAuthoredGenerationKey(
                collisionGenerationKey,
                weaponOwnershipKey);

        const bool equippedGenerationMatchesForm =
            weaponNode &&
            equippedWeapon &&
            weaponGenerationKey != 0 &&
            weaponOwnershipKey != 0 &&
            _weaponCollision.getCurrentObservedEquippedWeaponFormID() == equippedWeapon->formID;
        const bool equippedWeaponTransitionActive =
            _equippedWeaponTransition.getPublicSnapshot().active;

        native_idle_grip_preharvest::observeEquippedWeapon(
            equippedGenerationMatchesForm ? equippedWeapon : nullptr,
            equippedGenerationMatchesForm ? weaponNode : nullptr,
            equippedGenerationMatchesForm ? currentEquippedWeaponInstanceData(equippedWeapon) : nullptr,
            equippedGenerationMatchesForm ? _weaponCollision.getCurrentEquippedWeaponInstanceContentKey() : 0);

        _twoHandedGrip.beginAuthoredPrimaryFiringGripFrame();
        _authoredPrimaryFiringGrip.update(AuthoredPrimaryFiringGripFrameInput{
            .weaponNode = weaponNode,
            .weapon = equippedWeapon,
            .weaponOwnershipKey = weaponOwnershipKey,
            .weaponGenerationKey = weaponGenerationKey,
            .weaponInstanceContentKey = equippedGenerationMatchesForm ? _weaponCollision.getCurrentEquippedWeaponInstanceContentKey() : 0,
            .weaponKeywordFlags = weaponKeywordFlags,
            .weaponInstanceContentKnown = equippedGenerationMatchesForm,
            .runtimeInitialized = _initialized.load(std::memory_order_acquire),
            .visualAuthorityAvailable = runtime.visualAuthorityAvailable,
            .localSkeletonReady = runtime.localSkeletonReady,
            .menuBlocking = runtime.localMenuBlocking,
            .compatibilityBlocking = runtime.compatibilityConfigBlocking,
            .weaponDrawn = runtime.weaponDrawn,
            .weaponVisible = weaponNode && f4vr::isNodeVisible(weaponNode),
            // Arms/hands-only manual cycling must retain ROCK's authored
            // weapon-to-controller alignment. Only a native Weapon transform
            // lease (the full reload path) suspends that owner.
            .nativeReloadAuthorityActive =
                (nativeAuthorityFlags &
                    authored_weapon_grip_capture_policy::kWeapon) != 0,
            .conflictingWeaponTransformAuthorityActive =
                _twoHandedGrip.blocksAuthoredPrimaryGripWeaponAlignment(),
            .weaponVisualReturnActive = _twoHandedGrip.isWeaponVisualReturnActive(),
            .equippedWeaponTransitionActive =
                equippedWeaponTransitionActive,
            .primaryHandHoldingObject = rightHandHoldingObject,
            .rockFiringHandIsLeft = _twoHandedGrip.isFiringHandLeft(),
            .inPowerArmor = f4vr::isInPowerArmor(),
        }, _twoHandedGrip);
        _twoHandedGrip.finishAuthoredPrimaryFiringGripFrame();
        if (_equippedWeaponTransition.isHandPoseHandoffActive()) {
            const bool handoffHandIsLeft = _equippedWeaponTransition.handPoseHandoffIsLeft();
            if (nativeAuthorityFlags != 0 ||
                runtime.localMenuBlocking ||
                runtime.compatibilityConfigBlocking) {
                _equippedWeaponTransition.completeHandPoseHandoff("authored-pose-unavailable");
            } else if (equippedWeapon && equippedWeapon->formID != _equippedWeaponTransition.bridgeWeaponBaseFormID()) {
                _equippedWeaponTransition.completeHandPoseHandoff("equipped-weapon-changed");
            } else if (_twoHandedGrip.hasPublishedAuthoredPrimaryFiringGripFingerPose(handoffHandIsLeft)) {
                _equippedWeaponTransition.completeHandPoseHandoff("equipped-authored-pose-acquired");
            }
        }
    }

    void PhysicsInteraction::refreshEquippedWeaponHandlingSettings()
    {
        ::rock::provider::RockProviderEquippedWeaponHandlingRequestV1 request{};
        const bool externalAuthorityActive =
            ::rock::provider::getEquippedWeaponHandlingAuthorityV1(request);
        const RockEquippedWeaponHandlingBaseline rockBaseline{
            .ambidextrousHandoffEnabled =
                g_rockConfig.rockAmbidextrousFiringGripEnabled,
            .authoredOnlySupportGrabsEnabled =
                g_rockConfig.
                    rockAuthoredOnlyEquippedWeaponSupportGrabsEnabled,
            .toggleGrabEnabled =
                g_rockConfig.rockEquippedWeaponToggleGrabEnabled,
            .equippedWeaponShoulderStashEnabled =
                g_rockConfig.rockEquippedWeaponShoulderStashEnabled,
            .immersiveWeapon = {
                .firingGripDetachEnabled =
                    g_rockConfig.
                        rockFiringGripDetachEnabled,
                .firingGripDetachPosePreservationEnabled =
                    g_rockConfig.
                        rockFiringGripDetachPosePreservationEnabled,
                .firingGripReattachRadiusGameUnits =
                    g_rockConfig.
                        rockFiringGripReattachRadiusGameUnits,
                .firingGripHapticDurationSeconds =
                    g_rockConfig.
                        rockFiringGripHapticDurationSeconds,
                .firingGripAttachHapticIntensity =
                    g_rockConfig.
                        rockFiringGripAttachHapticIntensity,
                .firingGripDetachHapticIntensity =
                    g_rockConfig.
                        rockFiringGripDetachHapticIntensity,
            },
            .firingGripProximitySupportRadiusGameUnits =
                g_rockConfig.rockFiringGripProximitySupportRadius,
            .firingGripPromotionRadiusGameUnits =
                g_rockConfig.rockFiringGripPromotionRadius,
            .leftFiringAimYawDegrees =
                g_rockConfig.rockLeftFiringAimYawDegrees,
            .leftFiringAimPitchDegrees =
                g_rockConfig.rockLeftFiringAimPitchDegrees,
            .leftFiringAimOffsetXGameUnits =
                g_rockConfig.rockLeftFiringAimOffsetXGameUnits,
            .leftFiringAimOffsetYGameUnits =
                g_rockConfig.rockLeftFiringAimOffsetYGameUnits,
            .leftFiringAimOffsetZGameUnits =
                g_rockConfig.rockLeftFiringAimOffsetZGameUnits,
        };
        auto settings = makeEquippedWeaponHandlingSettings(
            rockBaseline,
            externalAuthorityActive ? &request : nullptr);

        if (_equippedWeaponHandlingModeInitialized) {
            if (requiresEquippedWeaponHandlingModeReconcile(
                    _equippedWeaponHandlingSettings,
                    settings)) {
                _equippedWeaponHandlingModeReconcilePending = true;
            }
        }

        _equippedWeaponHandlingSettings = settings;
        _equippedWeaponHandlingModeInitialized = true;
        equipped_weapon_handling_runtime::publish(settings);
    }

    void PhysicsInteraction::reconcileEquippedWeaponHandlingMode()
    {
        if (!_equippedWeaponHandlingModeReconcilePending) {
            return;
        }

        _twoHandedGrip.restoreNativeRightEquippedCarry(
            "equipped-weapon-handling-mode-changed");
        _pendingEquippedWeaponPrimaryOnlyGripStart = {};
        _equippedWeaponHandlingModeReconcilePending = false;
    }

    bool PhysicsInteraction::submitEquippedWeaponShoulderSheath(
        const std::uint32_t observedWeaponFormID,
        const std::uintptr_t observedWeaponInstanceData,
        const equipped_weapon_drop_policy::SourceHand sourceHand,
        const shoulder_stash::Decision& stashDecision,
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey)
    {
        const bool stashHandIsLeft =
            equipped_weapon_drop_policy::isLeft(sourceHand);
        const std::size_t stashHandIndex = stashHandIsLeft ? 1u : 0u;
        native_equipped_weapon_draw::Identity sheathIdentity{};
        const bool identityCaptured =
            native_equipped_weapon_draw::captureCurrentIdentity(
                sheathIdentity);
        const bool identityMatchesObserved =
            identityCaptured &&
            sheathIdentity.formID == observedWeaponFormID &&
            sheathIdentity.instanceData == observedWeaponInstanceData;
        RE::NiTransform leftFiringHandWeaponLocal{};
        RE::NiPoint3 leftFiringGripWeaponLocal{};
        const bool hasLeftFiringGripTransfer =
            identityMatchesObserved &&
            _twoHandedGrip.tryCaptureLeftFiringGripTransfer(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                leftFiringHandWeaponLocal,
                leftFiringGripWeaponLocal);
        native_equipped_weapon_draw::Result sheathResult{};
        sheathResult.result = identityCaptured ?
            native_equipped_weapon_draw::SubmitResult::IdentityChanged :
            native_equipped_weapon_draw::SubmitResult::MissingEquippedWeapon;
        if (identityMatchesObserved) {
            sheathResult = native_equipped_weapon_draw::
                submitSheatheExactCurrent(sheathIdentity);
        }
        const bool sheathAccepted =
            (sheathResult.result ==
                    native_equipped_weapon_draw::SubmitResult::Submitted ||
                sheathResult.result ==
                    native_equipped_weapon_draw::SubmitResult::
                        AlreadySheathingOrSheathed) &&
            held_weapon_equip_state_policy::
                isShoulderStashedPresentationState(
                    sheathResult.stateAfter);
        if (sheathAccepted) {
            equipped_weapon_toggle_grab_policy::reset(
                _equippedWeaponToggleGrabState);
            _twoHandedGrip.restoreNativeRightEquippedCarry(
                "shoulder-weapon-sheathed");
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            _equippedWeaponShoulderSheath =
                EquippedWeaponShoulderSheathState{
                    .active = true,
                    .stashedByLeftHand = stashHandIsLeft,
                    .weaponFormID = sheathIdentity.formID,
                    .weaponInstanceData = sheathIdentity.instanceData,
                    .equipIndex = sheathIdentity.equipIndex,
                    .weaponOwnershipKey =
                        currentEquippedWeaponOwnershipKey,
                    .zone = stashDecision.zone,
                    .hasLeftFiringGripTransfer =
                        hasLeftFiringGripTransfer,
                    .leftFiringHandWeaponLocal =
                        leftFiringHandWeaponLocal,
                    .leftFiringGripWeaponLocal =
                        leftFiringGripWeaponLocal,
                };
            input_remap_runtime::setEquippedWeaponShoulderSheathActive(true);
            _equippedWeaponSheathRetrievalStates = {};
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon shoulder sheathed formID={:08X} instance={:#x} equipIndex={} sourceHand={} zone={} confidence={:.2f} leftTransfer={} state={}({})->{}({}) result={}",
                sheathIdentity.formID,
                sheathIdentity.instanceData,
                sheathIdentity.equipIndex,
                equipped_weapon_drop_policy::sourceHandName(sourceHand),
                body_zone::bodyZoneName(stashDecision.zone),
                stashDecision.confidence,
                hasLeftFiringGripTransfer ? "captured" : "unavailable",
                sheathResult.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateBefore),
                sheathResult.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateAfter),
                native_equipped_weapon_draw::submitResultName(
                    sheathResult.result));
            if (g_rockConfig.rockShoulderStashHapticsEnabled) {
                (void)_feedbackHaptics.queue(
                    stashHandIsLeft ? feedback_haptics::FeedbackHand::Left :
                                      feedback_haptics::FeedbackHand::Right,
                    g_rockConfig.rockShoulderStashCommitHapticDurationSeconds,
                    g_rockConfig.rockShoulderStashCommitHapticIntensity);
            }
            if (g_rockConfig.rockShoulderStashShowCollectedNotifications) {
                f4vr::showNotification(
                    shoulder_stash_notification_policy::
                        formatStowedNotification(
                            shoulderStashItemName(
                                currentEquippedWeaponForm()),
                            sheathIdentity.formID));
            }
        } else {
            ROCK_LOG_WARN(Weapon,
                "Equipped weapon shoulder sheathe failed formID={:08X} instance={:#x} sourceHand={} identityMatch={} state={}({})->{}({}) result={} -- weapon stays equipped and physical drop is suppressed",
                sheathIdentity.formID,
                sheathIdentity.instanceData,
                equipped_weapon_drop_policy::sourceHandName(sourceHand),
                identityMatchesObserved ? "yes" : "no",
                sheathResult.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateBefore),
                sheathResult.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateAfter),
                native_equipped_weapon_draw::submitResultName(
                    sheathResult.result));
        }

        shoulder_stash::resetRuntime(
            _equippedWeaponStashStates[stashHandIndex]);
        return sheathAccepted;
    }

    void PhysicsInteraction::clearEquippedWeaponShoulderSheath(
        const char* reason,
        const bool resetCoordinator)
    {
        if (_equippedWeaponShoulderSheath.active) {
            ROCK_LOG_INFO(
                Weapon,
                "Equipped weapon shoulder sheath cleared reason={} formID={:08X} instance={:#x} zone={}",
                reason ? reason : "unknown",
                _equippedWeaponShoulderSheath.weaponFormID,
                _equippedWeaponShoulderSheath.weaponInstanceData,
                body_zone::bodyZoneName(
                    _equippedWeaponShoulderSheath.zone));
        }
        _equippedWeaponShoulderSheath = {};
        input_remap_runtime::setEquippedWeaponShoulderSheathActive(false);
        _equippedWeaponSheathRetrievalStates = {};
        if (resetCoordinator) {
            equipped_weapon_shoulder::reset(
                _equippedWeaponShoulderCoordinatorState);
        }
    }

    PhysicsInteraction::EquippedWeaponShoulderFrameResult
        PhysicsInteraction::advanceEquippedWeaponShoulderCoordinator(
            const PhysicsFrameContext& frame,
            const bool handlingEnabled,
            const bool menuInputActive,
            const std::uint32_t observedWeaponFormID,
            const std::uintptr_t observedWeaponInstanceData,
            RE::NiNode* weaponNode,
            const std::uint64_t currentEquippedWeaponOwnershipKey,
            const bool firingHandIsLeft)
    {
        EquippedWeaponShoulderFrameResult result{};
        auto* player = f4vr::getPlayer();
        const std::uint32_t nativeWeaponState =
            f4vr::getNativeWeaponState(player);
        const auto nativePresentation =
            shoulderNativePresentation(nativeWeaponState);

        native_equipped_weapon_draw::Identity currentIdentity{};
        const bool capturedCurrentIdentity =
            native_equipped_weapon_draw::captureCurrentIdentity(
                currentIdentity);
        if (_equippedWeaponShoulderSheath.active) {
            const bool identityMatches = capturedCurrentIdentity &&
                currentIdentity.formID ==
                    _equippedWeaponShoulderSheath.weaponFormID &&
                currentIdentity.instanceData ==
                    _equippedWeaponShoulderSheath.weaponInstanceData &&
                currentIdentity.equipIndex ==
                    _equippedWeaponShoulderSheath.equipIndex &&
                observedWeaponFormID == currentIdentity.formID &&
                observedWeaponInstanceData == currentIdentity.instanceData;
            if (!handlingEnabled) {
                clearEquippedWeaponShoulderSheath(
                    "shoulder-stash-authority-lost");
            } else if (!identityMatches) {
                clearEquippedWeaponShoulderSheath(
                    "equipped-weapon-identity-changed");
            } else if (nativePresentation ==
                           equipped_weapon_shoulder::NativePresentation::Invalid ||
                       nativePresentation ==
                           equipped_weapon_shoulder::NativePresentation::WantDraw ||
                       nativePresentation ==
                           equipped_weapon_shoulder::NativePresentation::Drawing ||
                       nativePresentation ==
                           equipped_weapon_shoulder::NativePresentation::StableDrawn) {
                clearEquippedWeaponShoulderSheath(
                    nativePresentation ==
                            equipped_weapon_shoulder::NativePresentation::Invalid ?
                        "invalid-native-weapon-state" :
                        "native-weapon-no-longer-sheathed");
            }
        }

        auto leftPhysicalGrip = peekGrabButtonState(
            true,
            input_remap_policy::kGrabButtonId);
        auto rightPhysicalGrip = peekGrabButtonState(
            false,
            input_remap_policy::kGrabButtonId);
        leftPhysicalGrip.held =
            input_remap_runtime::isRawButtonPhysicallyHeld(
                true,
                input_remap_policy::kGrabButtonId);
        rightPhysicalGrip.held =
            input_remap_runtime::isRawButtonPhysicallyHeld(
                false,
                input_remap_policy::kGrabButtonId);
        const std::uint64_t shoulderOwnershipKey =
            _equippedWeaponShoulderSheath.active ?
            _equippedWeaponShoulderSheath.weaponOwnershipKey :
            _equippedWeaponShoulderCoordinatorState.activeAction !=
                    equipped_weapon_shoulder::Action::None ?
                _equippedWeaponShoulderCoordinatorState.weaponOwnershipKey :
                currentEquippedWeaponOwnershipKey;

        const auto sheathInputMode =
            equipped_weapon_shoulder::resolveSheathInputMode(
                _equippedWeaponHandlingSettings.immersiveWeapon.
                    firingGripDetachEnabled,
                _equippedWeaponHandlingSettings.toggleGrabEnabled);
        equipped_weapon_shoulder::FrameInput coordinatorInput{
            .enabled = handlingEnabled,
            .inputAllowed = !menuInputActive,
            .sheathInputMode = sheathInputMode,
            .storedActive = _equippedWeaponShoulderSheath.active,
            .stashedByLeftHand =
                _equippedWeaponShoulderSheath.stashedByLeftHand,
            .weaponOwnershipKey = shoulderOwnershipKey,
            .presentation = nativePresentation,
            .storedZone = _equippedWeaponShoulderSheath.zone,
            .right = {
                .button = {
                    .held = rightPhysicalGrip.held,
                    .pressed = rightPhysicalGrip.pressed,
                    .released = rightPhysicalGrip.released,
                },
            },
            .left = {
                .button = {
                    .held = leftPhysicalGrip.held,
                    .pressed = leftPhysicalGrip.pressed,
                    .released = leftPhysicalGrip.released,
                },
            },
        };

        std::array<shoulder_stash::Decision, 2> detectorDecisions{};
        const auto detectorConfig =
            makeEquippedWeaponStashDetectorConfig(handlingEnabled);
        const auto queueCandidateHaptic = [&](const bool isLeft,
                                              const bool sameStoredZone,
                                              const shoulder_stash::Decision& decision,
                                              shoulder_stash::RuntimeState& detectorState) {
            if (!decision.candidate || !sameStoredZone ||
                !g_rockConfig.rockShoulderStashHapticsEnabled) {
                return;
            }
            const bool pulseDue = _dynamicPushElapsedSeconds >=
                detectorState.nextCandidatePulseTimeSeconds;
            if (!decision.enteredCandidate &&
                !decision.changedCandidate && !pulseDue) {
                return;
            }
            (void)_feedbackHaptics.queue(
                isLeft ? feedback_haptics::FeedbackHand::Left :
                         feedback_haptics::FeedbackHand::Right,
                g_rockConfig.rockShoulderStashCandidateHapticDurationSeconds,
                shoulder_stash_haptic_policy::computeCandidatePulseIntensity(
                    decision.confidence,
                    shoulder_stash_haptic_policy::CandidatePulseConfig{
                        .enabled = true,
                        .baseIntensity = g_rockConfig.
                            rockShoulderStashCandidateHapticBaseIntensity,
                        .maxIntensity = g_rockConfig.
                            rockShoulderStashCandidateHapticIntensity,
                    }));
            detectorState.nextCandidatePulseTimeSeconds =
                _dynamicPushElapsedSeconds +
                (std::max)(0.02f,
                    g_rockConfig.
                        rockShoulderStashCandidateHapticIntervalSeconds);
        };
        const auto publishDetectorDecision = [&](const bool isLeft,
                                                 const shoulder_stash::Decision& decision) {
            auto& handInput = isLeft ?
                coordinatorInput.left : coordinatorInput.right;
            handInput.detector = {
                .candidate = decision.candidate,
                .confirmed = decision.confirmedForCommit,
                .zone = decision.zone,
                .confidence = decision.confidence,
            };
        };

        if (_equippedWeaponShoulderSheath.active) {
            _equippedWeaponStashStates = {};
            const bool roleNeutralFiringGripOwnership =
                resolveEquippedWeaponDetachDecision(
                    _equippedWeaponHandlingSettings).
                    firingGripOwnershipEnabled;
            for (const bool isLeft : { false, true }) {
                const std::size_t handIndex = isLeft ? 1u : 0u;
                Hand& hand = isLeft ? _leftHand : _rightHand;
                const HandFrameInput& handInput =
                    isLeft ? frame.left : frame.right;
                auto& detectorState =
                    _equippedWeaponSheathRetrievalStates[handIndex];
                const bool handEmpty = !hand.isHolding() &&
                    !_touchGrabRuntime.isHandActive(isLeft) &&
                    !_pendingForceGrabCommits[handIndex].active &&
                    !hand.hasActivePullCatchIntent() &&
                    !hand.hasPendingActorEquipmentDropHandoff() &&
                    !_twoHandedGrip.isHandPartGripping(isLeft) &&
                    !(_twoHandedGrip.isFiringGripOccupied() &&
                        _twoHandedGrip.isFiringHandLeft() == isLeft);
                const bool handAllowedByHandlingMode =
                    roleNeutralFiringGripOwnership || !isLeft;
                const bool handCanOwnFiringGrip =
                    handAllowedByHandlingMode &&
                    TwoHandedGrip::canBeginPrimaryOnlyGripForHand(isLeft);
                auto& policyHand = isLeft ?
                    coordinatorInput.left : coordinatorInput.right;
                policyHand.disabled = handInput.disabled;
                policyHand.eligible = !handInput.disabled && handEmpty &&
                    handCanOwnFiringGrip;
                if (!policyHand.eligible || menuInputActive) {
                    shoulder_stash::resetRuntime(detectorState);
                    continue;
                }

                const auto decision = shoulder_stash::evaluate(
                    shoulder_stash::DetectorInput{
                        .isLeftHand = isLeft,
                        .probe = shoulder_stash::Probe{
                            .pointGame = handInput.grabAnchorWorld,
                        },
                        .hmdProbe = makeShoulderStashHmdProbe(handInput),
                        .hasHmdProbe = true,
                        .hasHmdFrame = frame.hasHmdFrame,
                        .hmdPositionWorld = frame.hmdPositionWorld,
                        .hmdForwardWorld = frame.hmdForwardWorld,
                        .deltaSeconds = frame.deltaSeconds,
                        .config = detectorConfig,
                    },
                    detectorState);
                detectorDecisions[handIndex] = decision;
                publishDetectorDecision(isLeft, decision);
                queueCandidateHaptic(
                    isLeft,
                    decision.zone ==
                        _equippedWeaponShoulderSheath.zone,
                    decision,
                    detectorState);
            }
        } else {
            _equippedWeaponSheathRetrievalStates = {};
            const auto manualStashCarryHand =
                equipped_weapon_drop_policy::
                    resolveEquippedWeaponStashCarryHand(
                        _twoHandedGrip.isPrimaryOnlyActive(),
                        _twoHandedGrip.isPartCarryActive(),
                        _twoHandedGrip.isHandPartCarryGripping(true),
                        _twoHandedGrip.isHandPartCarryGripping(false),
                        _twoHandedGrip.isFiringHandLeft());
            const auto attachedShoulderGestureHand = firingHandIsLeft ?
                equipped_weapon_drop_policy::SourceHand::Left :
                equipped_weapon_drop_policy::SourceHand::Right;
            const bool manualCarryActive =
                _twoHandedGrip.isPrimaryOnlyActive() ||
                _twoHandedGrip.isPartCarryActive();
            const auto stashCarryHand = manualStashCarryHand !=
                    equipped_weapon_drop_policy::SourceHand::None ?
                manualStashCarryHand :
                handlingEnabled && !manualCarryActive ?
                    attachedShoulderGestureHand :
                    equipped_weapon_drop_policy::SourceHand::None;

            for (const bool isLeft : { false, true }) {
                const std::size_t handIndex = isLeft ? 1u : 0u;
                const bool selectedCarryHand =
                    stashCarryHand !=
                        equipped_weapon_drop_policy::SourceHand::None &&
                    equipped_weapon_drop_policy::isLeft(stashCarryHand) ==
                        isLeft;
                auto& detectorState =
                    _equippedWeaponStashStates[handIndex];
                auto& policyHand = isLeft ?
                    coordinatorInput.left : coordinatorInput.right;
                policyHand.carriesWeapon = selectedCarryHand;
                if (!selectedCarryHand || menuInputActive) {
                    shoulder_stash::resetRuntime(detectorState);
                    continue;
                }

                Hand& hand = isLeft ? _leftHand : _rightHand;
                const HandFrameInput& handInput =
                    isLeft ? frame.left : frame.right;
                const bool handEmpty = !hand.isHolding() &&
                    !_touchGrabRuntime.isHandActive(isLeft) &&
                    !_pendingForceGrabCommits[handIndex].active &&
                    !hand.hasActivePullCatchIntent() &&
                    !hand.hasPendingActorEquipmentDropHandoff();
                policyHand.disabled = handInput.disabled;
                policyHand.eligible = handlingEnabled &&
                    !handInput.disabled && handEmpty && weaponNode != nullptr &&
                    currentEquippedWeaponOwnershipKey != 0;
                if (!policyHand.eligible) {
                    shoulder_stash::resetRuntime(detectorState);
                    continue;
                }

                const auto decision = shoulder_stash::evaluate(
                    shoulder_stash::DetectorInput{
                        .isLeftHand = isLeft,
                        .probe = shoulder_stash::Probe{
                            .pointGame = handInput.grabAnchorWorld,
                        },
                        .hmdProbe = makeShoulderStashHmdProbe(handInput),
                        .hasHmdProbe = true,
                        .hasHmdFrame = frame.hasHmdFrame,
                        .hmdPositionWorld = frame.hmdPositionWorld,
                        .hmdForwardWorld = frame.hmdForwardWorld,
                        .deltaSeconds = frame.deltaSeconds,
                        .config = detectorConfig,
                    },
                    detectorState);
                detectorDecisions[handIndex] = decision;
                publishDetectorDecision(isLeft, decision);
                queueCandidateHaptic(
                    isLeft,
                    true,
                    decision,
                    detectorState);
            }
        }

        const auto previousPhase =
            _equippedWeaponShoulderCoordinatorState.phase;
        result.decision = equipped_weapon_shoulder::advance(
            _equippedWeaponShoulderCoordinatorState,
            coordinatorInput);
        _equippedWeaponShoulderGestureConsumedThisFrame[0] =
            result.decision.consumeRightInput;
        _equippedWeaponShoulderGestureConsumedThisFrame[1] =
            result.decision.consumeLeftInput;

        if (result.decision.action !=
                equipped_weapon_shoulder::Action::None) {
            const auto& selectedDetector = detectorDecisions[
                result.decision.hand ==
                        equipped_weapon_shoulder::Hand::Left ?
                    1u : 0u];
            ROCK_LOG_INFO(
                Weapon,
                "Equipped shoulder coordinator action={} reason={} phase={}->{} hand={} gesture={} zone={} candidate={} confirmed={} source={} confidence={:.2f} speed={:.1f} nativeState={}({}) sheathInput={} immersive={} toggle={}",
                equipped_weapon_shoulder::actionName(
                    result.decision.action),
                equipped_weapon_shoulder::reasonName(
                    result.decision.reason),
                equipped_weapon_shoulder::phaseName(previousPhase),
                equipped_weapon_shoulder::phaseName(
                    _equippedWeaponShoulderCoordinatorState.phase),
                equipped_weapon_shoulder::handName(result.decision.hand),
                result.decision.gestureSerial,
                body_zone::bodyZoneName(result.decision.zone),
                (result.decision.hand ==
                        equipped_weapon_shoulder::Hand::Left ?
                        coordinatorInput.left.detector.candidate :
                        coordinatorInput.right.detector.candidate) ?
                    "yes" : "no",
                (result.decision.hand ==
                        equipped_weapon_shoulder::Hand::Left ?
                        coordinatorInput.left.detector.confirmed :
                        coordinatorInput.right.detector.confirmed) ?
                    "yes" : "no",
                shoulder_stash::evidenceSourceName(
                    selectedDetector.source),
                result.decision.confidence,
                selectedDetector.speedGameUnitsPerSecond,
                nativeWeaponState,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    nativeWeaponState),
                equipped_weapon_shoulder::sheathInputModeName(
                    sheathInputMode),
                _equippedWeaponHandlingSettings.immersiveWeapon.
                        firingGripDetachEnabled ?
                    "yes" : "no",
                _equippedWeaponHandlingSettings.toggleGrabEnabled ?
                    "yes" : "no");
        } else if (previousPhase !=
                   _equippedWeaponShoulderCoordinatorState.phase) {
            ROCK_LOG_DEBUG(
                Weapon,
                "Equipped shoulder coordinator phase={}->{} reason={} gestureHand={} gesture={} nativeState={}({}) stored={}",
                equipped_weapon_shoulder::phaseName(previousPhase),
                equipped_weapon_shoulder::phaseName(
                    _equippedWeaponShoulderCoordinatorState.phase),
                equipped_weapon_shoulder::reasonName(
                    result.decision.reason),
                equipped_weapon_shoulder::handName(
                    _equippedWeaponShoulderCoordinatorState.actionHand),
                _equippedWeaponShoulderCoordinatorState.actionGestureSerial,
                nativeWeaponState,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    nativeWeaponState),
                _equippedWeaponShoulderSheath.active ? "yes" : "no");
        }

        const auto toSourceHand = [](const equipped_weapon_shoulder::Hand hand) {
            return hand == equipped_weapon_shoulder::Hand::Left ?
                equipped_weapon_drop_policy::SourceHand::Left :
                hand == equipped_weapon_shoulder::Hand::Right ?
                equipped_weapon_drop_policy::SourceHand::Right :
                equipped_weapon_drop_policy::SourceHand::None;
        };
        result.sourceHand = toSourceHand(result.decision.hand);
        if (result.decision.hand !=
            equipped_weapon_shoulder::Hand::None) {
            result.detectorDecision = detectorDecisions[
                equipped_weapon_shoulder::isLeft(result.decision.hand) ?
                    1u : 0u];
        }

        if (result.decision.action !=
            equipped_weapon_shoulder::Action::SubmitRetrieve) {
            return result;
        }

        const bool retrieveWithLeftHand =
            result.decision.hand ==
            equipped_weapon_shoulder::Hand::Left;
        const std::size_t handIndex = retrieveWithLeftHand ? 1u : 0u;
        if (!capturedCurrentIdentity ||
            !_equippedWeaponShoulderSheath.active) {
            equipped_weapon_shoulder::reportExecutionResult(
                _equippedWeaponShoulderCoordinatorState,
                result.decision.action,
                false);
            ROCK_LOG_WARN(
                Weapon,
                "Equipped weapon shoulder retrieval rejected after coordinator selection because exact stored identity is unavailable");
            return result;
        }

        const auto storedZone = _equippedWeaponShoulderSheath.zone;
        const auto drawResult =
            native_equipped_weapon_draw::submitExactCurrent(
                currentIdentity);
        const bool drawAccepted =
            drawResult.result ==
                native_equipped_weapon_draw::SubmitResult::Submitted ||
            drawResult.result ==
                native_equipped_weapon_draw::SubmitResult::
                    AlreadyDrawingOrDrawn;
        equipped_weapon_shoulder::reportExecutionResult(
            _equippedWeaponShoulderCoordinatorState,
            result.decision.action,
            drawAccepted);
        if (!drawAccepted) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Equipped weapon shoulder unsheath failed formID={:08X} hand={} gesture={} zone={} state={}({})->{}({}) result={}",
                currentIdentity.formID,
                equipped_weapon_shoulder::handName(result.decision.hand),
                result.decision.gestureSerial,
                body_zone::bodyZoneName(storedZone),
                drawResult.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    drawResult.stateBefore),
                drawResult.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    drawResult.stateAfter),
                native_equipped_weapon_draw::submitResultName(
                    drawResult.result));
            shoulder_stash::resetRuntime(
                _equippedWeaponSheathRetrievalStates[handIndex]);
            return result;
        }

        _pendingEquippedWeaponPrimaryOnlyGripStart =
            PendingEquippedWeaponPrimaryOnlyGripStart{
                .pending = true,
                .isLeft = retrieveWithLeftHand,
                .targetWeaponFormID = currentIdentity.formID,
                .targetWeaponInstanceData = currentIdentity.instanceData,
                .remainingSeconds = 10.0f,
                .committedTransfer = true,
                .hasFiringHandWeaponLocal = retrieveWithLeftHand &&
                    _equippedWeaponShoulderSheath.
                        hasLeftFiringGripTransfer,
                .firingHandWeaponLocal =
                    _equippedWeaponShoulderSheath.
                        leftFiringHandWeaponLocal,
                .hasFiringGripWeaponLocal = retrieveWithLeftHand &&
                    _equippedWeaponShoulderSheath.
                        hasLeftFiringGripTransfer,
                .firingGripWeaponLocal =
                    _equippedWeaponShoulderSheath.
                        leftFiringGripWeaponLocal,
            };
        if (g_rockConfig.rockShoulderStashHapticsEnabled) {
            (void)_feedbackHaptics.queue(
                retrieveWithLeftHand ?
                    feedback_haptics::FeedbackHand::Left :
                    feedback_haptics::FeedbackHand::Right,
                g_rockConfig.rockShoulderStashCommitHapticDurationSeconds,
                g_rockConfig.rockShoulderStashCommitHapticIntensity);
        }
        ROCK_LOG_INFO(
            Weapon,
            "Equipped weapon shoulder unsheathed formID={:08X} hand={} gesture={} zone={} confidence={:.2f} state={}({})->{}({}) result={}",
            currentIdentity.formID,
            equipped_weapon_shoulder::handName(result.decision.hand),
            result.decision.gestureSerial,
            body_zone::bodyZoneName(storedZone),
            result.detectorDecision.confidence,
            drawResult.stateBefore,
            held_weapon_equip_state_policy::nativeWeaponStateName(
                drawResult.stateBefore),
            drawResult.stateAfter,
            held_weapon_equip_state_policy::nativeWeaponStateName(
                drawResult.stateAfter),
            native_equipped_weapon_draw::submitResultName(
                drawResult.result));
        clearEquippedWeaponShoulderSheath(
            "physical-hand-unsheath-committed",
            false);
        return result;
    }
}
