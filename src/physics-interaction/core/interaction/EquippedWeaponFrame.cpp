#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapture.h"
#include "physics-interaction/weapon/telemetry/VanillaWeaponAlignmentTelemetry.h"
#include "physics-interaction/weapon/ManualScopeTargetPolicy.h"
#include "physics-interaction/weapon/telemetry/ScopeTransitionTelemetry.h"

// Equipped-weapon frame: transitions, the per-frame equipped weapon update, authored primary grip runtime, handling settings, and shoulder sheath/retrieve.

namespace rock
{
    namespace
    {
        void maskProviderWeaponGrabInput(bool isLeft, bool holding, GrabButtonState& button)
        {
            if (input_remap_runtime::ownsBareFistInput()) {
                button = {};
                return;
            }
            using Flag = provider::RockProviderHandInputSuppressionFlagV1;
            const auto flags = provider::currentHandInputSuppressionFlagsV1(
                isLeft ? provider::RockProviderHand::Left : provider::RockProviderHand::Right);
            const bool press = provider::hasHandInputSuppressionFlagV1(flags, Flag::SuppressNormalGrabPress);
            const bool release = provider::hasHandInputSuppressionFlagV1(flags, Flag::SuppressGrabRelease);
            if (!press && !release) return;
            const auto filtered = input_remap_policy::suppressGrabInput(
                {.grabHeld = button.held, .grabPressed = button.pressed, .grabReleased = button.released},
                press, release, holding);
            button = {.held = filtered.grabHeld, .pressed = filtered.grabPressed, .released = filtered.grabReleased};
            // The UI reads raw levels. Drain gameplay edges so opening the
            // wheel cannot become a delayed detach after its lease is cleared.
            static_cast<void>(input_remap_runtime::consumeRawButtonState(isLeft, input_remap_policy::kGrabButtonId));
        }
    }

    void PhysicsInteraction::traceScopeColliderState() const
    {
        if (!scope_transition_telemetry::activeLogger()) return;
        DynamicWeaponCollisionRuntime::DebugSnapshot snapshot{};
        const bool valid = _dynamicWeaponCollision.getDebugSnapshot(snapshot);
        scope_transition_telemetry::write(
            "SCT collider frame={} source=cached-finish-frame valid={} physicsReadable={} physicsValid={} identityCurrent={} generation={:X} solve={} body={} authorityBody={} contact={} teleported={} visualCorrection={} correction=({:.4f}gu,{:.4f}deg) requestedT=({:.4f},{:.4f},{:.4f}) liveT=({:.4f},{:.4f},{:.4f}) resolvedT=({:.4f},{:.4f},{:.4f})",
            scope_transition_telemetry::sequence(), valid, snapshot.physicsSnapshotReadable, snapshot.physicsSnapshotValid,
            snapshot.physicsSnapshotIdentityCurrent, snapshot.generationKey, snapshot.solveSequence, snapshot.bodyId,
            snapshot.authorityBodyId, snapshot.contactActive, snapshot.physicsSnapshotTeleported, snapshot.visualCorrectionActive,
            snapshot.translationCorrectionGameUnits, snapshot.rotationCorrectionDegrees,
            snapshot.requestedWeaponWorld.translate.x, snapshot.requestedWeaponWorld.translate.y, snapshot.requestedWeaponWorld.translate.z,
            snapshot.liveWeaponWorld.translate.x, snapshot.liveWeaponWorld.translate.y, snapshot.liveWeaponWorld.translate.z,
            snapshot.resolvedWeaponWorld.translate.x, snapshot.resolvedWeaponWorld.translate.y, snapshot.resolvedWeaponWorld.translate.z);
    }

    bool PhysicsInteraction::tryGetManualScopePresentationTarget(
        std::uint64_t& outWeaponGenerationKey,
        std::uint32_t& outNativeOverlayIndex,
        bool& outDirectTransitionRequired,
        const void* expectedWeapon,
        const void* expectedInstance) const
    {
        outWeaponGenerationKey = 0;
        outNativeOverlayIndex = 0;
        outDirectTransitionRequired = false;
        if (!g_rockConfig.rockEnableImmersiveScopes ||
            !_lifecycle.initialized.load(std::memory_order_acquire) || !runtime_state::isLocalSkeletonReady()) {
            return false;
        }
        const auto snapshot = _weaponCollision.getNativeScopeSightAnchorSnapshot();
        if (!snapshot.scopeEligible) {
            // Native classification also runs while idle. Ordinary sights
            // are an expected negative result, not a failed scope anchor.
            return false;
        }
        if (expectedWeapon && !manual_scope_target_policy::matchesNativeIdentity(
                snapshot.scopeWeaponIdentity, snapshot.scopeInstanceIdentity,
                reinterpret_cast<std::uintptr_t>(expectedWeapon), reinterpret_cast<std::uintptr_t>(expectedInstance))) {
            ROCK_LOG_SAMPLE_DEBUG(Weapon, 1000,
                "Native scope admission identity rejected: publishedWeapon=0x{:X} instance=0x{:X} nativeWeapon=0x{:X} instance=0x{:X}",
                snapshot.scopeWeaponIdentity, snapshot.scopeInstanceIdentity,
                reinterpret_cast<std::uintptr_t>(expectedWeapon), reinterpret_cast<std::uintptr_t>(expectedInstance));
            return false;
        }
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
            !snapshot.nativeScopeOverlayValid ||
            !native_scope_sight_anchor_policy::matchesCurrentEquippedWeapon(publishedIdentity, currentIdentity)) {
            if (expectedWeapon) {
                ROCK_LOG_SAMPLE_DEBUG(Weapon, 1000,
                    "Native scope admission target rejected: anchorValid={} eligible={} overlayValid={} publishedGeneration={:016X} resolvedGeneration={:016X} currentGeneration={:016X}",
                    resolvedAnchor.valid, snapshot.scopeEligible, snapshot.nativeScopeOverlayValid,
                    snapshot.weaponGenerationKey, resolvedAnchor.weaponGenerationKey, currentIdentity.weaponGenerationKey);
            }
            return false;
        }
        outWeaponGenerationKey = snapshot.weaponGenerationKey;
        outNativeOverlayIndex = snapshot.nativeScopeOverlayIndex;
        outDirectTransitionRequired = snapshot.manualDirectTransitionRequired;
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
        RE::NiTransform leftCarrySolvedWeaponWorld{};
        _equipped.transition.update(
            EquippedWeaponTransitionCoordinator::FrameInput{
                .deltaSeconds = runtime.deltaSeconds,
                .visualAuthorityAvailable = runtime.visualAuthorityAvailable,
                .localSkeletonReady = runtime.localSkeletonReady,
                .menuBlocking = runtime.localMenuBlocking,
                .compatibilityBlocking = runtime.compatibilityConfigBlocking,
                .nativeWeaponState = nativeWeaponState,
                .intentionalShoulderSheathActive =
                    _equipped.shoulderSheath.active,
                .shoulderSheathFormID =
                    _equipped.shoulderSheath.weaponFormID,
                .shoulderSheathInstanceData =
                    _equipped.shoulderSheath.weaponInstanceData,
                .shoulderSheathEquipIndex =
                    _equipped.shoulderSheath.equipIndex,
                .nativeWeaponAnimationActive = nativeWeaponAnimationActive,
                // Read one frame behind the grip solve by design: this update
                // runs before TwoHandedGrip::update each frame, so the bridge
                // receives the previous frame's solved carry pose - the pose
                // that was actually rendered.
                .leftCarrySolvedWeaponWorldValid =
                    _twoHandedGrip.tryGetSolvedLeftFiringWeaponWorld(
                        leftCarrySolvedWeaponWorld),
                .leftCarrySolvedWeaponWorld = leftCarrySolvedWeaponWorld,
            });
    }

    PhysicsInteraction::EquippedWeaponFrameResult PhysicsInteraction::updateEquippedWeaponFrame(
        const PhysicsFrameContext& frame,
        RE::bhkWorld* bhk,
        RE::hknpWorld* hknp)
    {
        performance_profiler::ScopedTimer equippedFrameTimer(performance_profiler::Scope::EquippedWeaponInteraction);
        const auto& runtime = runtime_state::currentFrame();
        const bool leftHandAvailableForAcquisition = force_grab_policy::availableForEquippedGrip(
            forceGrabHandBlockerMask(_leftHand, true, frame.left.disabled, true));
        const bool rightHandAvailableForAcquisition = force_grab_policy::availableForEquippedGrip(
            forceGrabHandBlockerMask(_rightHand, false, frame.right.disabled, true));
        weapon_recoil_policy::WeaponEvidence recoilWeapon{};

        RE::NiNode* weaponNode = resolveEquippedWeaponInteractionNode();
        /*
         * The weapon node does not keep ROCK's carry pose across frames, even
         * in part-carry. Republish ROCK's solved carry transform first so
         * weapon-part probes, firing-grip zone checks, and grip capture frames
         * all read the weapon where the player sees it - the same frame the
         * generated colliders follow.
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
        logPalmClockSampleForHand("game-after-hand-collider-prepare",
            _rightHand,
            hknp,
            frame.right.disabled ? nullptr : &frame.right.rawHandWorld,
            runtime.frameIndex,
            frame.deltaSeconds,
            nullptr);
        logPalmClockSampleForHand("game-after-hand-collider-prepare",
            _leftHand,
            hknp,
            frame.left.disabled ? nullptr : &frame.left.rawHandWorld,
            runtime.frameIndex,
            frame.deltaSeconds,
            nullptr);
        updateBodyBoneCollisions(frame);
        updateNativePlayerCollisionFilter(bhk, hknp);

        {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::WeaponCollision);

            if (g_rockConfig.rockDebugVerboseLogging) {
                if (++_diagnostics.wpnNodeLogCounter >= 90) {
                    _diagnostics.wpnNodeLogCounter = 0;
                    if (weaponNode) {
                        ROCK_LOG_DEBUG(Weapon, "WeaponNode: '{}' pos=({:.1f},{:.1f},{:.1f}) hasBody={} bodyCount={}", weaponNode->name.c_str(), weaponNode->world.translate.x,
                            weaponNode->world.translate.y, weaponNode->world.translate.z, _weaponCollision.hasWeaponBody(), _weaponCollision.getWeaponBodyCount());
                    } else {
                    }
                }
            }
            _weaponCollision.update(hknp, weaponNode, frame.deltaSeconds, runtime.weaponDrawn);
            const auto weaponClassification = _weaponCollision.getEquippedWeaponClassification();
            recoilWeapon = {
                .formID = weaponClassification.formID,
                .keywordFlags = weaponClassification.keywordFlags,
                .sizeClass = weaponClassification.sizeClass,
                .source = weaponClassification.classificationSource,
                .resolved = weaponClassification.hasEquippedWeapon && weaponClassification.classificationResolved,
            };
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
        _dynamicWeaponCollision.beginFrame(
            runtime.frameIndex,
            hknp,
            bhk,
            weaponNode,
            currentWeaponGenerationKey,
            dynamic_weapon_collision_policy::kDynamicCompoundEnabled &&
                runtime.weaponDrawn &&
                !frame.menuBlocked &&
                physicsWritesAllowedForWorld(frame.hknpWorld));
        reconcileEquippedWeaponHandlingMode();
        // WeaponCollision has just read the actual drawn inventory instance;
        // its observed identity exists before geometry/grab-session readiness.
        _twoHandedGrip.observeEquippedOwnership(
            runtime.weaponDrawn && _weaponCollision.getCurrentObservedEquippedWeaponFormID() != 0 ?
                currentEquippedWeaponOwnershipKey : 0,
            currentAuthoredGripGenerationKey);

        {
            WeaponInteractionContact leftWeaponContact{};
            WeaponInteractionContact rightWeaponContact{};
            auto leftWeaponContactSource = weapon_debug_notification_policy::WeaponContactSource::None;

            auto publishWeaponInteractionContact = [&](bool isLeft, WeaponInteractionContact& contact) {
                auto& partKind = isLeft ? _weaponContact.left.partKind : _weaponContact.right.partKind;
                auto& reloadRole = isLeft ? _weaponContact.left.reloadRole : _weaponContact.right.reloadRole;
                auto& supportRole = isLeft ? _weaponContact.left.supportRole : _weaponContact.right.supportRole;
                auto& socketRole = isLeft ? _weaponContact.left.socketRole : _weaponContact.right.socketRole;
                auto& actionRole = isLeft ? _weaponContact.left.actionRole : _weaponContact.right.actionRole;
                auto& gripPose = isLeft ? _weaponContact.left.gripPose : _weaponContact.right.gripPose;
                auto& sequence = isLeft ? _weaponContact.left.sequence : _weaponContact.right.sequence;
                auto& missedFrames = isLeft ? _weaponContact.left.missedFrames : _weaponContact.right.missedFrames;

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
                auto& bodyIdAtomic = isLeft ? _weaponContact.left.bodyId : _weaponContact.right.bodyId;
                auto& missedFrames = isLeft ? _weaponContact.left.missedFrames : _weaponContact.right.missedFrames;
                auto& acquisitionState = _equipped.weaponInteractionAcquisitionStates[isLeft ? 0u : 1u];

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
                        if (g_rockConfig.rockDebugVerboseLogging && ++_diagnostics.weaponInteractionProbeLogCounter >= 90) {
                            _diagnostics.weaponInteractionProbeLogCounter = 0;
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
            if (_equipped.pendingPrimaryOnlyGripStart.pending) {
                _equipped.pendingPrimaryOnlyGripStart.remainingSeconds -=
                    (std::max)(0.0f, frame.deltaSeconds);
                if (_equipped.pendingPrimaryOnlyGripStart.remainingSeconds <= 0.0f) {
                    ROCK_LOG_WARN(Weapon,
                        "Held weapon manual ownership handoff expired targetForm={:08X} targetInstance={:#x} hand={} lastStartFailure={} supportReadiness={}",
                        _equipped.pendingPrimaryOnlyGripStart.targetWeaponFormID,
                        _equipped.pendingPrimaryOnlyGripStart.targetWeaponInstanceData,
                        _equipped.pendingPrimaryOnlyGripStart.isLeft ? "left" : "right",
                        _equipped.pendingPrimaryOnlyGripStart.lastStartFailureReason ?
                            _equipped.pendingPrimaryOnlyGripStart.lastStartFailureReason : "none",
                        authored_support_grab_policy::leftFiringTakeoverReadinessName(
                            _equipped.pendingPrimaryOnlyGripStart.takeoverWitness.last));
                    _equipped.pendingPrimaryOnlyGripStart = {};
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
                    _equipped.handlingSettings.equippedWeaponShoulderStashEnabled);
            const bool inputBlockingMenuActive =
                input_remap_runtime::isMenuInputActive();
            const bool shoulderPendingPrimaryStartMatchesCurrentWeapon =
                _equipped.pendingPrimaryOnlyGripStart.pending &&
                (_equipped.pendingPrimaryOnlyGripStart.
                        targetWeaponFormID == 0 ||
                    equipped_weapon_transition_policy::
                        matchesExpectedIdentity(
                            observedEquippedWeaponFormID,
                            observedEquippedWeaponInstanceData,
                            _equipped.pendingPrimaryOnlyGripStart.
                                targetWeaponFormID,
                            _equipped.pendingPrimaryOnlyGripStart.
                                targetWeaponInstanceData,
                            _equipped.pendingPrimaryOnlyGripStart.
                                previousWeaponFormID,
                            _equipped.pendingPrimaryOnlyGripStart.
                                previousWeaponInstanceData));
            const bool shoulderFiringHandIsLeft =
                shoulderPendingPrimaryStartMatchesCurrentWeapon ?
                _equipped.pendingPrimaryOnlyGripStart.isLeft :
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
                _equipped.pendingPrimaryOnlyGripStart.pending &&
                (_equipped.pendingPrimaryOnlyGripStart.targetWeaponFormID == 0 ||
                    equipped_weapon_transition_policy::matchesExpectedIdentity(
                        observedEquippedWeaponFormID,
                        observedEquippedWeaponInstanceData,
                        _equipped.pendingPrimaryOnlyGripStart.targetWeaponFormID,
                        _equipped.pendingPrimaryOnlyGripStart.targetWeaponInstanceData,
                        _equipped.pendingPrimaryOnlyGripStart.previousWeaponFormID,
                        _equipped.pendingPrimaryOnlyGripStart.previousWeaponInstanceData));
            const bool firingHandIsLeft = pendingPrimaryStartMatchesCurrentWeapon ?
                _equipped.pendingPrimaryOnlyGripStart.isLeft :
                _twoHandedGrip.isFiringHandLeft();
            const bool supportHandIsLeft = !firingHandIsLeft;
            const auto firingGripDecision =
                resolveEquippedWeaponDetachDecision(
                    _equipped.handlingSettings);

            /*
             * While the LEFT hand carries the weapon, the node does not hold
             * last frame's carry pose here; the ranked part probes below
             * convert real palm points into node-local space, so a stale frame
             * made a forend grab select the wrong part. Publish the canonical
             * carry pose first so both hands probe the weapon where it
             * actually is.
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
            const auto providerGripOccupancy = _twoHandedGrip.getGripOccupancy();
            maskProviderWeaponGrabInput(true, providerGripOccupancy.left.weaponEngaged(), leftPhysicalGripState);
            maskProviderWeaponGrabInput(false, providerGripOccupancy.right.weaponEngaged(), rightPhysicalGripState);
            const auto holsterSnapshot = inputBlockingMenuActive || frame.menuBlocked ?
                virtual_holsters::Snapshot{} : virtual_holsters::readSnapshot();
            const auto holsterOccupancy = _twoHandedGrip.getGripOccupancy();
            const auto advanceHolsterInput = [&](const bool isLeft, const GrabButtonState& button) {
                const auto handIndex = equipped_weapon_toggle_grab_policy::handIndex(isLeft);
                const auto& occupancy = isLeft ? holsterOccupancy.left : holsterOccupancy.right;
                const bool toggleGrab = occupancy.usesToggleGrab(_equipped.handlingSettings.weaponGrabMode);
                const auto decision = virtual_holsters::advance(
                    _equipped.holsterInputStates[handIndex],
                    virtual_holsters::Input{
                        .holster = holsterSnapshot,
                        .ownershipKey = currentEquippedWeaponOwnershipKey,
                        .grabButtonId = input_remap_policy::kGrabButtonId,
                        .isLeft = isLeft,
                        .weaponEngaged = occupancy.firingGripActive || _twoHandedGrip.isHandPartCarryGripping(isLeft),
                        .toggleGrab = toggleGrab,
                        .held = button.held,
                        .pressed = button.pressed,
                        .released = button.released,
                    });
                _equipped.holsterInputConsumedThisFrame[handIndex] = decision.consumeInput;
                if (decision.started) {
                    ROCK_LOG_INFO(Weapon,
                        "VirtualHolsters weapon release deferred: hand={} slot={} mode={} held={} pressed={} released={} ownership={:016X}",
                        isLeft ? "left" : "right", holsterSnapshot.slot,
                        toggleGrab ? "toggle" : "hold",
                        button.held, button.pressed, button.released, currentEquippedWeaponOwnershipKey);
                }
                return decision;
            };
            const auto leftHolsterInput = advanceHolsterInput(true, leftPhysicalGripState);
            const auto rightHolsterInput = advanceHolsterInput(false, rightPhysicalGripState);
            const auto maskHolsterInput = [&](const bool isLeft, GrabButtonState& button) {
                const auto& decision = isLeft ? leftHolsterInput : rightHolsterInput;
                if (decision.consumeInput) {
                    // Preserve an existing grip before any detach/toggle/drop
                    // decision. Empty hands cannot acquire from this gesture.
                    button = GrabButtonState{ .held = decision.retainGrip };
                }
            };
            maskHolsterInput(true, leftPhysicalGripState);
            maskHolsterInput(false, rightPhysicalGripState);
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

            const auto indicatorBodyId = [&](const WeaponInteractionContact& contact,
                                             const WeaponInteractionRuntimeState& state) {
                return contact.valid && state.supportGripAllowed &&
                    contact.weaponGenerationKey == currentWeaponGenerationKey ?
                    contact.bodyId : weapon_part_runtime::kInvalidBodyId;
            };
            _equipped.partIndicatorBodyIds = {
                !leftHandAvailableForAcquisition ? weapon_part_runtime::kInvalidBodyId :
                    indicatorBodyId(leftWeaponContact, providerInteractionState),
                !rightHandAvailableForAcquisition ? weapon_part_runtime::kInvalidBodyId :
                    indicatorBodyId(rightWeaponContact, rightHandInteractionState),
            };
            _equipped.partIndicatorFrame = runtime.frameIndex;
            _equipped.partIndicatorGeneration = currentWeaponGenerationKey;

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
            _grabInput.firingHandButtonFrame = {};
            auto readPrimaryGrabState = [&]() -> const GrabButtonState& {
                if (!primaryGrabStateRead) {
                    primaryGrabState = readGrabButtonState(firingHandIsLeft, input_remap_policy::kGrabButtonId);
                    // Menu rearm intentionally masks gameplay edges, but
                    // firing-grip ownership still follows the physical hand
                    // state after the menu closes.
                    primaryGrabState.held = input_remap_runtime::isRawButtonPhysicallyHeld(firingHandIsLeft, input_remap_policy::kGrabButtonId);
                    const auto physicalPrimaryGrabState = primaryGrabState;
                    maskProviderWeaponGrabInput(firingHandIsLeft,
                        (firingHandIsLeft ? providerGripOccupancy.left : providerGripOccupancy.right).weaponEngaged(), primaryGrabState);
                    maskHolsterInput(firingHandIsLeft, primaryGrabState);
                    maskShoulderGestureInput(
                        firingHandIsLeft,
                        primaryGrabState);
                    primaryGrabStateRead = true;
                    (firingHandIsLeft ? leftPhysicalGripState :
                                        rightPhysicalGripState) =
                        primaryGrabState;
                    // Publish the consumed snapshot so the normal grab pipeline
                    // sees the same edges instead of re-consuming cleared ones.
                    _grabInput.firingHandButtonFrame = SharedGrabButtonFrameState{
                        .valid = true,
                        .isLeft = firingHandIsLeft,
                        .held = physicalPrimaryGrabState.held,
                        .pressed = physicalPrimaryGrabState.pressed,
                        .released = physicalPrimaryGrabState.released,
                    };
                }
                return primaryGrabState;
            };
            const bool primaryPoseBlockerAvailable = frik_visual_authority::canBlockPrimaryHandWeaponPose();
            const bool ambidextrousHandoffAvailable =
                _equipped.handlingSettings.ambidextrousHandoffEnabled &&
                TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true);
            const bool firingGripOwnershipFeatureAvailable = equipped_weapon_manual_ownership_policy::featureAvailable(
                !_equipped.shoulderSheath.active &&
                    firingGripDecision.firingGripOwnershipEnabled,
                primaryPoseBlockerAvailable,
                weaponNode != nullptr,
                currentEquippedWeaponOwnershipKey);
            const bool primaryDetachFeatureAvailable = equipped_weapon_manual_ownership_policy::featureAvailable(
                !_equipped.shoulderSheath.active &&
                    firingGripDecision.primaryDetachEnabled,
                primaryPoseBlockerAvailable,
                weaponNode != nullptr,
                currentEquippedWeaponOwnershipKey);
            bool pendingToggleCancelRequested = false;
            auto& pendingPrimaryStart =
                _equipped.pendingPrimaryOnlyGripStart;
            if (pendingPrimaryStart.pairedGrips.valid()) {
                for (const bool isLeft : { false, true }) {
                    const auto& physical = isLeft ? leftPhysicalGripState : rightPhysicalGripState;
                    pendingPrimaryStart.pairedRelease[equipped_weapon_toggle_grab_policy::handIndex(isLeft)].observe(
                        _equipped.handlingSettings.weaponGrabMode, isLeft == pendingPrimaryStart.isLeft,
                        { .held = physical.held, .pressed = physical.pressed, .released = physical.released });
                }
            } else if (pendingPrimaryStart.toggleAcquisitionCommitted) {
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
                    _equipped.toggleGrabReleasePressConsumedThisFrame[
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
                _equipped.pendingPrimaryOnlyGripStart.
                    toggleAcquisitionCommitted;
            if (inputBlockingMenuActive) {
                _equipped.pendingPrimaryOnlyGripStart = {};
            } else if (_equipped.pendingPrimaryOnlyGripStart.pending &&
                !equipped_weapon_manual_ownership_policy::shouldKeepPendingPrimaryOnlyStart(
                    equipped_weapon_manual_ownership_policy::PendingPrimaryOnlyStartInput{
                        .pending = _equipped.pendingPrimaryOnlyGripStart.pending,
                        .gripHeld =
                            input_remap_runtime::isRawButtonPhysicallyHeld(
                                firingHandIsLeft,
                                input_remap_policy::kGrabButtonId) ||
                            pendingToggleGripRetained,
                        .source =
                            _equipped.pendingPrimaryOnlyGripStart.
                                source,
                        .ownershipModeEnabled =
                            firingGripDecision.firingGripOwnershipEnabled,
                        .primaryPoseBlockerAvailable = primaryPoseBlockerAvailable,
                    })) {
                _equipped.pendingPrimaryOnlyGripStart = {};
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

            if (!primaryGrabStateRead) {
                static_cast<void>(readPrimaryGrabState());
            }

            auto toggleOccupancyBefore = _twoHandedGrip.getGrabInputOccupancy();

            bool primaryOnlyGripStartedThisFrame = false;
            bool pairedGripStartedThisFrame = false;
            const auto pairedRelease = pendingPrimaryStart.pairedRelease;
            bool nativeFiringGripTransfer = false;
            const bool firingHandAvailableForAcquisition = firingHandIsLeft ?
                leftHandAvailableForAcquisition : rightHandAvailableForAcquisition;
            const bool pairedHandsAvailable = !pendingPrimaryStart.pairedGrips.valid() ||
                (leftHandAvailableForAcquisition && rightHandAvailableForAcquisition);
            if (firingGripOwnershipFeatureAvailable && !inputBlockingMenuActive &&
                firingHandAvailableForAcquisition && pairedHandsAvailable && !_twoHandedGrip.isManualOwnershipActive()) {
                const auto& primaryState = readPrimaryGrabState();
                if (_equipped.pendingPrimaryOnlyGripStart.pending &&
                    !primaryState.held &&
                    _equipped.pendingPrimaryOnlyGripStart.source ==
                        equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource::GripInput &&
                    !pendingToggleGripRetained) {
                    _equipped.pendingPrimaryOnlyGripStart = {};
                }

                const bool pendingPrimaryOnlyStartRequested =
                    !pendingToggleCancelRequested &&
                    equipped_weapon_manual_ownership_policy::
                        shouldStartPendingPrimaryOnlyGrip(
                            pendingPrimaryStartMatchesCurrentWeapon,
                            primaryState.held || pendingToggleGripRetained,
                            _equipped.pendingPrimaryOnlyGripStart.
                                source);
                const bool primaryOnlyStartRequested =
                    !pendingToggleCancelRequested &&
                    weaponNode != nullptr &&
                    currentEquippedWeaponOwnershipKey != 0 &&
                    ((primaryDetachFeatureAvailable && primaryState.held &&
                         primaryState.pressed) ||
                        pendingPrimaryOnlyStartRequested);
                /*
                 * The whole grip subsystem (session keys, reconcile/rebind,
                 * authored canonical, support capability and candidate) runs
                 * on the AUTHORED generation key, which exists the moment the
                 * weapon node does. The takeover entry must use the same key:
                 * keying it to the raw collision generation made every
                 * left-hand equip wait for the collider build and present the
                 * weapon at the native right-hand attach until takeover.
                 */
                if (pendingPrimaryStartMatchesCurrentWeapon &&
                    _equipped.pendingPrimaryOnlyGripStart.isLeft &&
                    (!_equipped.pendingPrimaryOnlyGripStart.hasFiringHandWeaponLocal ||
                        !_equipped.pendingPrimaryOnlyGripStart.hasFiringGripWeaponLocal)) {
                    _equipped.pendingPrimaryOnlyGripStart.hasFiringHandWeaponLocal =
                        _twoHandedGrip.tryBuildCurrentLeftFiringGripCapture(
                            weaponNode,
                            currentAuthoredGripGenerationKey,
                            currentEquippedWeaponOwnershipKey,
                            _equipped.pendingPrimaryOnlyGripStart.firingHandWeaponLocal,
                            _equipped.pendingPrimaryOnlyGripStart.firingGripWeaponLocal);
                    _equipped.pendingPrimaryOnlyGripStart.hasFiringGripWeaponLocal =
                        _equipped.pendingPrimaryOnlyGripStart.hasFiringHandWeaponLocal;
                }
                const RE::NiTransform* capturedFiringHandWeaponLocal =
                    pendingPrimaryStartMatchesCurrentWeapon &&
                        _equipped.pendingPrimaryOnlyGripStart.hasFiringHandWeaponLocal ?
                    &_equipped.pendingPrimaryOnlyGripStart.firingHandWeaponLocal :
                    nullptr;
                const RE::NiPoint3* capturedFiringGripWeaponLocal =
                    pendingPrimaryStartMatchesCurrentWeapon &&
                        _equipped.pendingPrimaryOnlyGripStart.hasFiringGripWeaponLocal ?
                    &_equipped.pendingPrimaryOnlyGripStart.firingGripWeaponLocal :
                    nullptr;
                const bool retainUntilPhysicalGrip =
                    pendingPrimaryStartMatchesCurrentWeapon &&
                    (_equipped.pendingPrimaryOnlyGripStart.source ==
                            equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource::ShoulderRetrieval ||
                        pendingToggleGripRetained);
                const auto leftTakeoverReadiness =
                    _twoHandedGrip.getLeftFiringTakeoverReadiness(
                        weaponNode,
                        currentAuthoredGripGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        _equipped.handlingSettings.
                            authoredOnlySupportGrabsEnabled);
                const bool leftTakeoverBlocked =
                    pendingPrimaryStartMatchesCurrentWeapon &&
                    primaryOnlyStartRequested &&
                    firingHandIsLeft && !_equipped.pendingPrimaryOnlyGripStart.pairedGrips.valid() &&
                    !authored_support_grab_policy::
                        leftFiringTakeoverReady(
                            leftTakeoverReadiness);
                if (leftTakeoverBlocked) {
                    auto& pendingStart =
                        _equipped.pendingPrimaryOnlyGripStart;
                    if (pendingStart.takeoverWitness.observe(
                            leftTakeoverReadiness)) {
                        ROCK_LOG_DEBUG(
                            Weapon,
                            "Left firing-grip start waiting weapon='{}' formID={:08X} authored support readiness={} collisionGeneration={:016X} authoredGeneration={:016X} ownership={:016X}",
                            observedEquippedWeapon ? RE::TESFullName::GetFullName(*observedEquippedWeapon, false) : "unknown",
                            pendingStart.targetWeaponFormID,
                            authored_support_grab_policy::
                                leftFiringTakeoverReadinessName(
                                    leftTakeoverReadiness),
                            currentWeaponGenerationKey,
                            currentAuthoredGripGenerationKey,
                            currentEquippedWeaponOwnershipKey);
                    }
                } else if (primaryOnlyStartRequested) {
                    const char* startFailureReason = nullptr;
                    const bool pairedTransfer = pendingPrimaryStartMatchesCurrentWeapon &&
                        _equipped.pendingPrimaryOnlyGripStart.pairedGrips.valid();
                    const bool started = pairedTransfer ? _twoHandedGrip.beginTransferredTwoHandGrip(
                        weaponNode, currentAuthoredGripGenerationKey, currentEquippedWeaponOwnershipKey,
                        _equipped.pendingPrimaryOnlyGripStart.pairedGrips, &startFailureReason) :
                        _twoHandedGrip.beginPrimaryOnlyGrip(
                        weaponNode,
                        currentAuthoredGripGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        firingHandIsLeft,
                        capturedFiringHandWeaponLocal,
                        capturedFiringGripWeaponLocal,
                        retainUntilPhysicalGrip,
                        true,
                        &startFailureReason);
                    if (started) {
                        pairedGripStartedThisFrame = pairedTransfer;
                        if (pairedTransfer) {
                            (void)_twoHandedGrip.commitPersistentEquippedCarryInputAcquisition(firingHandIsLeft);
                            toggleOccupancyBefore = _twoHandedGrip.getGrabInputOccupancy();
                        }
                        if (_equipped.pendingPrimaryOnlyGripStart.pending) {
                            ROCK_LOG_INFO(Weapon,
                                "Equipped hand transfer completed weapon='{}' formID={:08X} hand={} source={}",
                                observedEquippedWeapon ? RE::TESFullName::GetFullName(*observedEquippedWeapon, false) : "unknown",
                                observedEquippedWeaponFormID,
                                firingHandIsLeft ? "left" : "right",
                                static_cast<unsigned>(_equipped.pendingPrimaryOnlyGripStart.source));
                        }
                        primaryOnlyGripStartedThisFrame = true;
                        nativeFiringGripTransfer = !pendingPrimaryOnlyStartRequested &&
                            equipped_weapon_toggle_grab_policy::firingUsesToggle(_equipped.handlingSettings.weaponGrabMode) &&
                            _equipped.handlingSettings.lastGripReleaseDropEnabled;
                        if (nativeFiringGripTransfer) {
                            (firingHandIsLeft ? toggleOccupancyBefore.left : toggleOccupancyBefore.right).
                                firingGripActive = true;
                        }
                        _equipped.pendingPrimaryOnlyGripStart = {};
                        primaryGripInput = EquippedWeaponPrimaryGripInput{
                            .held = primaryState.held,
                            .pressed = primaryState.pressed,
                            .released = primaryState.released,
                        };
                    } else {
                        _equipped.pendingPrimaryOnlyGripStart.lastStartFailureReason = startFailureReason;
                        ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                            "Equipped hand transfer blocked weapon='{}' formID={:08X} hand={} reason={} generation={:016X} ownership={:016X} physicalHeld={} toggleRetained={}",
                            observedEquippedWeapon ? RE::TESFullName::GetFullName(*observedEquippedWeapon, false) : "unknown",
                            observedEquippedWeaponFormID,
                            firingHandIsLeft ? "left" : "right",
                            startFailureReason ? startFailureReason : "unknown",
                            currentAuthoredGripGenerationKey,
                            currentEquippedWeaponOwnershipKey,
                            primaryState.held,
                            pendingToggleGripRetained);
                    }
                }
            } else if (inputBlockingMenuActive) {
                _equipped.pendingPrimaryOnlyGripStart = {};
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
                _providerDrives.resultCount = 0;
            }

            /*
             * Firing-grip reattach is the squeeze gesture (grab held with the
             * palm on the grip); the reattach zone is evaluated by
             * TwoHandedGrip. This
             * only gates whether each free hand may be captured at all -
             * either hand can take the firing grip when ambidextrous takeover
             * is available.
             */
            bool leftReattachEligible = false;
            bool rightReattachEligible = false;
            if (_twoHandedGrip.isPartCarryActive() && primaryDetachFeatureAvailable) {
                leftReattachEligible = leftHandAvailableForAcquisition && weapon_two_handed_grip_math::canAttemptFiringGripReattach(
                    weapon_two_handed_grip_math::FiringGripReattachInput{
                        .partCarryActive = true,
                        .menuInputActive = inputBlockingMenuActive,
                        .handHoldingObject = _leftHand.isHolding(),
                    });
                rightReattachEligible = rightHandAvailableForAcquisition && weapon_two_handed_grip_math::canAttemptFiringGripReattach(
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
            auto toggleGrabDecision =
                equipped_weapon_toggle_grab_policy::prepare(
                    _equipped.toggleGrabState,
                    equipped_weapon_toggle_grab_policy::Input{
                        .weaponGrabMode = _equipped.handlingSettings.
                            weaponGrabMode,
                        .inputAllowed = !inputBlockingMenuActive,
                        .weaponOwnershipKey =
                            currentEquippedWeaponOwnershipKey,
                        .occupancy = toggleOccupancyBefore,
                        .nativeFiringGripTransfer = nativeFiringGripTransfer,
                        .left = toToggleButtonState(
                            leftPhysicalGripState),
                        .right = toToggleButtonState(
                            rightPhysicalGripState),
                    });
            if (pairedGripStartedThisFrame) {
                equipped_weapon_toggle_grab_policy::adoptTransferredGrips(_equipped.toggleGrabState,
                    _equipped.handlingSettings.weaponGrabMode, currentEquippedWeaponOwnershipKey,
                    _twoHandedGrip.getGrabInputOccupancy());
                for (const bool isLeft : { false, true }) {
                    const auto index = equipped_weapon_toggle_grab_policy::handIndex(isLeft);
                    const bool released = pairedRelease[index].releaseRequested;
                    (isLeft ? toggleGrabDecision.left : toggleGrabDecision.right) = { .held = !released, .released = released };
                    (isLeft ? toggleGrabDecision.leftReleasePressConsumed : toggleGrabDecision.rightReleasePressConsumed) = released;
                    if (released) _equipped.toggleGrabState.hands[index] = equipped_weapon_toggle_grab_policy::HandState::ReleasePending;
                }
            }
            // A release may already be pending when the sphere becomes active.
            // Override that logical open state too, then reconcile it below so
            // leaving the sphere cannot replay the refused toggle release.
            if (leftHolsterInput.consumeInput) {
                toggleGrabDecision.left = { .held = leftHolsterInput.retainGrip };
            }
            if (rightHolsterInput.consumeInput) {
                toggleGrabDecision.right = { .held = rightHolsterInput.retainGrip };
            }
            leftGripHeld = toggleGrabDecision.left.held;
            rightGripHeld = toggleGrabDecision.right.held;
            const auto& logicalPrimaryGrip = firingHandIsLeft ?
                toggleGrabDecision.left : toggleGrabDecision.right;
            primaryGripInput = EquippedWeaponPrimaryGripInput{
                .held = logicalPrimaryGrip.held,
                .pressed = logicalPrimaryGrip.pressed,
                .released = logicalPrimaryGrip.released,
            };
            _equipped.toggleGrabReleasePressConsumedThisFrame[
                equipped_weapon_toggle_grab_policy::handIndex(true)] =
                _equipped.toggleGrabReleasePressConsumedThisFrame[
                    equipped_weapon_toggle_grab_policy::handIndex(true)] ||
                toggleGrabDecision.leftReleasePressConsumed;
            _equipped.toggleGrabReleasePressConsumedThisFrame[
                equipped_weapon_toggle_grab_policy::handIndex(false)] =
                _equipped.toggleGrabReleasePressConsumedThisFrame[
                    equipped_weapon_toggle_grab_policy::handIndex(false)] ||
                toggleGrabDecision.rightReleasePressConsumed;

            const auto captureScopeHandDriverFrame = [](const bool isLeft) {
                EquippedWeaponScopeHandDriverFrame result{};
                result.nodeAvailable = frik_hand_world_authority::tryGetInputDriverWorld(isLeft, result.world);
                result.worldFinite = result.nodeAvailable && finiteNiTransform(result.world);
                result.valid = result.worldFinite;
                return result;
            };
            const EquippedWeaponScopeHandDriverFrame leftHandDriverFrame = captureScopeHandDriverFrame(true);
            const EquippedWeaponScopeHandDriverFrame rightHandDriverFrame = captureScopeHandDriverFrame(false);
            bool nativeScopeRequestActive = false;
            const bool nativeScopeRequestStateValid =
                tryReadNativeScopeRequestState(nativeScopeRequestActive);
            const bool manualScopeActivationRequested =
                input_remap_runtime::isManualScopeActivationRequested();
            const bool pairedGripPending = _equipped.pendingPrimaryOnlyGripStart.pairedGrips.valid();
            const EquippedWeaponGripFrameInput gripFrameInput{
                .leftGripHeld = !pairedGripPending && leftGripHeld,
                .rightGripHeld = !pairedGripPending && rightGripHeld,
                .leftHandHoldingObject = leftHandHoldingObject,
                .rightHandHoldingObject = _rightHand.isHolding(),
                .leftHandAvailableForAcquisition = leftHandAvailableForAcquisition,
                .rightHandAvailableForAcquisition = rightHandAvailableForAcquisition,
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
                .weaponGrabMode =
                    _equipped.handlingSettings.weaponGrabMode,
                .animationBoundaryActive = frame.reloadBoundaryActive,
                .hasHmdFrame = frame.hasHmdFrame,
                .recoilWeapon = recoilWeapon,
            };
            auto effectiveHandlingSettings = _equipped.handlingSettings;
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
            TwoHandedGripUpdateResult gripUpdateResult =
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
            const auto toggleOccupancyAfter = _twoHandedGrip.getGrabInputOccupancy();
            const auto toggleReconcileDecision =
                equipped_weapon_toggle_grab_policy::reconcile(
                    _equipped.toggleGrabState,
                    _equipped.handlingSettings.weaponGrabMode,
                    currentEquippedWeaponOwnershipKey,
                    toggleOccupancyAfter,
                    equipped_weapon_toggle_grab_policy::GripReleaseRetention{
                        .left = gripUpdateResult.releaseRetained.left || leftHolsterInput.retainGrip,
                        .right = gripUpdateResult.releaseRetained.right || rightHolsterInput.retainGrip,
                    });
            const auto consumeToggleAcquisitionPress =
                [this](const bool isLeft, const bool acquired) {
                if (!acquired) {
                    return;
                }

                (void)_twoHandedGrip.
                    commitPersistentEquippedCarryInputAcquisition(isLeft);

                GrabButtonState acquisitionInput{};
                if (_grabInput.firingHandButtonFrame.valid &&
                    _grabInput.firingHandButtonFrame.isLeft == isLeft) {
                    acquisitionInput = GrabButtonState{
                        .held = _grabInput.firingHandButtonFrame.held,
                        .pressed = _grabInput.firingHandButtonFrame.pressed,
                        .released = _grabInput.firingHandButtonFrame.released,
                    };
                    _grabInput.firingHandButtonFrame.valid = false;
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
                toggleReconcileDecision.leftGripAcquired || pairedGripStartedThisFrame);
            consumeToggleAcquisitionPress(
                false,
                toggleReconcileDecision.rightGripAcquired || pairedGripStartedThisFrame);
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
                        _equipped.handlingSettings);
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

                if (_equipped.handlingSettings.externalAuthorityActive) {
                    queueGripHaptic(
                        isLeft,
                        _equipped.handlingSettings.
                            weaponGripHapticDurationSeconds,
                        attached ?
                            _equipped.handlingSettings.
                                firingGripAttachHapticIntensity :
                            _equipped.handlingSettings.
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
            if (_equipped.handlingSettings.externalAuthorityActive) {
                if (gripHapticEvents.leftPartGripCaptured) {
                    queueGripHaptic(
                        true,
                        _equipped.handlingSettings.
                            weaponGripHapticDurationSeconds,
                        _equipped.handlingSettings.supportGripHapticIntensity);
                }
                if (gripHapticEvents.rightPartGripCaptured) {
                    queueGripHaptic(
                        false,
                        _equipped.handlingSettings.
                            weaponGripHapticDurationSeconds,
                        _equipped.handlingSettings.supportGripHapticIntensity);
                }
            }
            /*
             * Continuous hover feedback while an open free palm sits inside
             * the firing-grip reattach zone during part carry: re-queued
             * every frame so the vibration holds until the squeeze reattaches
             * (which flips the state and hands off to the firingGripAttached
             * pulse above).
             */
            if (_equipped.handlingSettings.gripZoneHoverHapticsEnabled &&
                _twoHandedGrip.isFiringGripReattachHoverInsideZone()) {
                (void)_feedbackHaptics.queue(
                    _twoHandedGrip.isFiringGripReattachHoverHandLeft() ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                    grip_zone_hover_haptic_policy::kContinuousQueueSeconds,
                    _equipped.handlingSettings.gripZoneHoverHapticIntensity);
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
                    _equipped.shoulderCoordinator,
                    equipped_weapon_shoulder::Action::SubmitSheath,
                    sheathAccepted);
            }
            const auto equippedWeaponDropRequest = _twoHandedGrip.consumeEquippedWeaponDropRequest();
            if (equippedWeaponDropRequest.requested) {
                bool transferCommitted = false;
                // Keep the old scene alive until its presentation authorities
                // are cleared after the synchronous inventory mutation.
                RE::NiPointer<RE::NiNode> transferSourceNode(weaponNode);
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
                } else {
                    const bool stashCommitSelected =
                        nativeShoulderSheathSelected ||
                        equippedWeaponShoulderFrame.decision.
                            suppressEquippedDrop;
                    const bool physicalDropRequested =
                        equipped_weapon_drop_policy::shouldAttemptPhysicalDrop(stashCommitSelected);
                    const bool transferIsLeft = equipped_weapon_drop_policy::isLeft(sourceHand);
                    const auto transferHandIndex = transferIsLeft ? 1u : 0u;
                    Hand& transferHand = transferIsLeft ? _leftHand : _rightHand;
                    const auto& transferInput = transferIsLeft ? frame.left : frame.right;
                    const bool dropHandoffAvailable = sourceHandKnown && equippedWeaponDropRequest.pose.valid() &&
                        equippedWeaponDropRequest.pose.weaponFormId == observedEquippedWeaponFormID && hasAvailableEquippedWeaponDropHandoff() &&
                        (forceGrabHandBlockerMask(transferHand, transferIsLeft, transferInput.disabled, true) &
                            ~static_cast<std::uint32_t>(force_grab_policy::HandBlocker::EquippedWeapon)) == 0;
                    if (physicalDropRequested && !dropHandoffAvailable) {
                        ROCK_LOG_WARN(Weapon,
                            "Equipped weapon transfer blocked: source hand unavailable or native handoff capacity exhausted capacity={}",
                            _drop.nativeHandoffs.size());
                        f4vr::showNotification("ROCK: Cannot take weapon - hand or transfer queue is busy.");
                    }
                    if (physicalDropRequested && dropHandoffAvailable) {
                        // Preserve the equipped pose while the native loose bodies
                        // appear. Toggle Drop then seats the exact reference using
                        // the same authored weapon resolver as a far-grab catch.
                        RE::NiPoint3 releaseLoc = dropLoc;
                        RE::NiPoint3 releaseRot{};
                        const auto& releaseWeaponWorld = equippedWeaponDropRequest.weaponWorld;
                        const bool hasReleaseRot = finiteNiTransform(releaseWeaponWorld);
                        if (hasReleaseRot) {
                            releaseLoc = releaseWeaponWorld.translate;
                            releaseRot = transform_math::matrixToReferenceEulerRadians<RE::NiMatrix3, RE::NiPoint3>(releaseWeaponWorld.rotate);
                        }
                        const std::size_t releaseHandIndex = equipped_weapon_drop_policy::isLeft(sourceHand) ? 1u : 0u;
                        const auto& releaseHandInput = releaseHandIndex == 1u ? frame.left : frame.right;
                        const RE::NiPoint3 releaseGripWorld = releaseHandInput.grabAnchorWorld;
                        // Capture the native placement basis before retiring the
                        // generated equipped representation.
                        const auto releaseGeometry = hasReleaseRot ?
                                                         _weaponCollision.getCurrentWeaponReleaseGeometry(releaseGripWorld, releaseWeaponWorld) :
                                                         WeaponCollision::ReleaseGeometrySnapshot{};
                        if (!releaseGeometry.hasCapturedWeaponWorld) {
                            ROCK_LOG_WARN(Weapon,
                                "Equipped weapon physical drop blocked because no finite frozen release pose is available: sourceHand={}",
                                equipped_weapon_drop_policy::sourceHandName(sourceHand));
                            f4vr::showNotification("ROCK: Cannot drop weapon - release pose is not ready.");
                        } else {
                            const bool toggleDrop = equipped_weapon_drop_policy::fromSetting(g_rockConfig.rockWeaponDropMode) ==
                                equipped_weapon_drop_policy::Mode::ToggleDrop;
                            const auto sourceVisual = toggleDrop ?
                                equipped_weapon_visual_state::observe(observedEquippedWeaponFormID) :
                                equipped_weapon_visual_state::Snapshot{};
                            RE::NiPointer<RE::NiAVObject> dropVisualModel(
                                sourceVisual.ancestorPathVisible && sourceVisual.instanceLocallyVisible ?
                                    sourceVisual.exactInstance : nullptr);
                            const auto dropVisualInWeapon = dropVisualModel ?
                                transform_math::composeTransforms(transform_math::invertTransform(releaseWeaponWorld),
                                    dropVisualModel->world) : RE::NiTransform{};
                            _twoHandedGrip.prepareEquippedWeaponDropCommit();
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
                            transferCommitted = dropCommitted;
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
                                _forceGrab.pendingCommits[transferHandIndex] = PendingForceGrabCommit{
                                    .active = true,
                                    .isLeft = transferIsLeft,
                                    .phase = dropResult.equippedSlotReleased ?
                                        PendingForceGrabCommitPhase::WaitingForNativePlacement :
                                        PendingForceGrabCommitPhase::EquippedSlotReleaseFailed,
                                    .targetHandle = dropResult.handle,
                                    .inventoryTransfer = true,
                                    .equippedWeaponDropMode = equipped_weapon_drop_policy::fromSetting(g_rockConfig.rockWeaponDropMode),
                                    .weaponGripPose = equippedWeaponDropRequest.pose,
                                    .maxDistanceGame = 96.0f,
                                };
                                if (_forceGrab.pendingCommits[transferHandIndex].equippedWeaponDropMode ==
                                    equipped_weapon_drop_policy::Mode::ToggleDrop) {
                                    _forceGrab.retainedWeaponGrabs[transferHandIndex] = {
                                        .inputState = transferred_weapon_grab_policy::State::AwaitInitialRelease,
                                    };
                                }
                                vanilla_weapon_alignment_telemetry::recordTransferPose(
                                    dropResult.droppedFormID, transferIsLeft, "release",
                                    releaseGeometry.capturedWeaponWorld,
                                    (transferIsLeft ? frame.left : frame.right).rawHandWorld);
                                if (dropResult.equippedSlotReleased) {
                                    if (toggleDrop) {
                                        _drop.visuals[transferHandIndex].begin(std::move(dropVisualModel),
                                            dropVisualInWeapon, equippedWeaponDropRequest.pose, dropResult.droppedFormID);
                                    }
                                    armEquippedWeaponNativeHandoff(
                                        dropResult.handle,
                                        dropResult.droppedFormID,
                                        sourceHand,
                                        releaseGeometry);
                                }
                            }
                            if (dropCommitted) {
                                ROCK_LOG_INFO(Weapon,
                                    "Equipped weapon drop queued mode={} formID={:08X} dropped={:08X} reference={} sourceHand={} dropLoc=({:.1f},{:.1f},{:.1f}) lever={:.1f}gu stack={} instanceMatch={}",
                                    g_rockConfig.rockWeaponDropMode,
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
                    transferCommitted = transferCommitted || stashCommitSelected;
                }
                _twoHandedGrip.completeEquippedWeaponDrop(equippedWeaponDropRequest, transferCommitted);
                gripUpdateResult.after = _twoHandedGrip.getGripOccupancy();
                if (transferCommitted) {
                    _equipped.pendingPrimaryOnlyGripStart = {};
                    clearEquippedWeaponFiringGripInputState();
                } else {
                    (void)equipped_weapon_toggle_grab_policy::reconcile(_equipped.toggleGrabState,
                        _equipped.handlingSettings.weaponGrabMode, currentEquippedWeaponOwnershipKey,
                        _twoHandedGrip.getGrabInputOccupancy(), {
                            .left = sourceHand == equipped_weapon_drop_policy::SourceHand::Left,
                            .right = sourceHand == equipped_weapon_drop_policy::SourceHand::Right,
                        });
                }
            }
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
                    weapon_debug_notification_policy::observeWeaponSupportGrip(_diagnostics.weaponDebugNotification, weaponSupportGripActive);
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
                _diagnostics.weaponDebugNotification.supportGripActive = weaponSupportGripActive;
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

            RE::NiPoint3 surfaceSupportPrimaryGripLocal{};
            const bool surfaceSupportPrimaryValid =
                _twoHandedGrip.tryGetSurfaceSupportPrimaryGripLocal(
                    weaponNode, currentWeaponGenerationKey, surfaceSupportPrimaryGripLocal);
            const auto dynamicWeaponFrame =
                _dynamicWeaponCollision.finishFrame(
                    frame,
                    physicsWritesAllowedForWorld(frame.hknpWorld),
                    weaponNode,
                    currentWeaponGenerationKey,
                    _weaponCollision,
                    surfaceSupportPrimaryValid ? &surfaceSupportPrimaryGripLocal : nullptr);
            if (_dynamicWeaponCollision.compoundSourcesUnavailable()) {
                _weaponCollision.requestRebuildForReplacedSources();
            }
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
            // Final source transforms are queued after all animation/FRIK writers.
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

    void PhysicsInteraction::publishGripZoneIndicatorRenderFrame(
        const std::uint64_t gameFrameIndex)
    {
        const auto& runtime = runtime_state::currentFrame();
        if (!_lifecycle.initialized.load(std::memory_order_acquire) ||
            !runtime.visualAuthorityAvailable ||
            !runtime.localSkeletonReady || runtime.localMenuBlocking ||
            runtime.compatibilityConfigBlocking || gameFrameIndex == 0 ||
            gameFrameIndex != runtime.frameIndex) {
            debug::ClearGripZoneIndicators();
            return;
        }

        debug::GripZoneIndicatorOverlayFrame overlayFrame{};
        overlayFrame.gameFrameIndex = gameFrameIndex;
        overlayFrame.diameterGameUnits =
            g_rockConfig.rockGripZoneIndicatorDiameterGameUnits;

        if (_powerArmorCandidateFrame == gameFrameIndex) {
            for (const bool isLeft : {false, true}) {
                const auto& candidate = _powerArmorCandidates[isLeft ? 1u : 0u];
                if (candidate.valid && !_touchGrabRuntime.isHandActive(isLeft) &&
                    !(isLeft ? _leftHand : _rightHand).isHolding()) {
                    overlayFrame.positions[overlayFrame.count++] = candidate.positionGame;
                }
            }
            if (overlayFrame.count) debug::Install();
        }

        for (const bool isLeft : {false, true}) {
            const auto& hand = isLeft ? _leftHand : _rightHand;
            if (!hand.isHolding() && hand.hasSelection() && !_touchGrabRuntime.isHandActive(isLeft)) {
                overlayFrame.count += static_cast<std::uint32_t>(loose_weapon_grip_zone::collectIndicators(
                    isLeft, hand.getSelection().refr,
                    std::span<RE::NiPoint3>(overlayFrame.positions).subspan(overlayFrame.count)));
            }
        }
        if (overlayFrame.count) {
            debug::Install();
        }

        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        const std::uint64_t currentWeaponGenerationKey =
            _weaponCollision.getCurrentWeaponGenerationKey();
        if (!weaponNode || !f4vr::isNodeVisible(weaponNode) ||
            currentWeaponGenerationKey == 0 ||
            !dynamic_weapon_collision_policy::isFiniteTransform(
                weaponNode->world) ||
            std::abs(weaponNode->world.scale) <= 0.0001f) {
            debug::PublishGripZoneIndicators(overlayFrame);
            return;
        }

        const auto appendIndicator = [&](const auto& indicatorFrame,
                                         const Hand& hand) {
            if (!indicatorFrame.visible || hand.isHolding() ||
                !indicatorFrame.weaponLocalValid ||
                indicatorFrame.weaponGenerationKey !=
                    currentWeaponGenerationKey ||
                overlayFrame.count >= overlayFrame.positions.size()) {
                return;
            }

            const RE::NiPoint3 positionWorld =
                transform_math::localPointToWorld(
                    weaponNode->world,
                    indicatorFrame.positionWeaponLocal);
            if (!std::isfinite(positionWorld.x) ||
                !std::isfinite(positionWorld.y) ||
                !std::isfinite(positionWorld.z)) {
                return;
            }
            overlayFrame.positions[overlayFrame.count++] = positionWorld;
        };

        const auto authoredSupportFrame =
            _twoHandedGrip.getAuthoredSupportGripIndicatorFrame();
        appendIndicator(
            authoredSupportFrame,
            authoredSupportFrame.supportHandIsLeft ?
                _leftHand :
                _rightHand);

        const auto firingReattachFrame =
            _twoHandedGrip.getFiringGripReattachIndicatorFrame();
        appendIndicator(
            firingReattachFrame,
            firingReattachFrame.handIsLeft ? _leftHand : _rightHand);

        if (_equipped.partIndicatorFrame == gameFrameIndex &&
            _equipped.partIndicatorGeneration == currentWeaponGenerationKey) {
            auto candidateBodyIds = _equipped.partIndicatorBodyIds;
            const auto occupancy = _twoHandedGrip.getGripOccupancy();
            for (const bool isLeft : { true, false }) {
                const auto& hand = isLeft ? _leftHand : _rightHand;
                const auto& grip = isLeft ? occupancy.left : occupancy.right;
                if (hand.isHolding() || _touchGrabRuntime.isHandActive(isLeft) ||
                    grip.weaponEngaged() ||
                    (isLeft == _twoHandedGrip.isFiringHandLeft() &&
                        !_twoHandedGrip.isPartCarryActive())) {
                    candidateBodyIds[isLeft ? 0u : 1u] = weapon_part_runtime::kInvalidBodyId;
                }
            }
            if (candidateBodyIds[0] != weapon_part_runtime::kInvalidBodyId ||
                candidateBodyIds[1] != weapon_part_runtime::kInvalidBodyId) {
                std::array<weapon_part_runtime::Target,
                    provider::ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1> partTargets{};
                const auto targetCount = provider::copyWeaponPartTargets(partTargets);
                overlayFrame.count += static_cast<std::uint32_t>(
                    _weaponCollision.collectAttachOnlyGripIndicators(weaponNode,
                        std::span(partTargets).first(targetCount), candidateBodyIds,
                        std::span(overlayFrame.positions).subspan(overlayFrame.count)));
            }
        }

        if (overlayFrame.count > 0) {
            debug::Install();
        }
        debug::PublishGripZoneIndicators(overlayFrame);
    }

    /*
     * AfterWeaponPosition. FRIK's weapon pass writes the whole Weapon local
     * (offset and scale) after ROCK's frame, so the presentation baseline is
     * re-applied here: the render sees it, and the offset latch taken next
     * carries it into ROCK's next frame. Animation graph scale and an explicit
     * Weapon animation owner remain authoritative over this default. A
     * transition can remain active after the exact model is visible; waiting
     * for completion leaves its first equipped frames undersized.
     */
    void PhysicsInteraction::normalizeWeaponPresentationScaleAfterFrikWeaponPass()
    {
        const auto& runtime = runtime_state::currentFrame();
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        auto* equippedWeapon = currentEquippedWeaponForm();
        if (!weaponNode || !equippedWeapon ||
            !_lifecycle.initialized.load(std::memory_order_acquire) || !runtime.visualAuthorityAvailable ||
            !runtime.localSkeletonReady || runtime.localMenuBlocking || runtime.compatibilityConfigBlocking ||
            !runtime.weaponDrawn) {
            return;
        }
        // Same freshness boundary as the authored grip: the richer
        // stack/instance key when generated collision is on, the equipped form otherwise.
        std::uint64_t weaponOwnershipKey = _weaponCollision.getCurrentEquippedWeaponOwnershipKey();
        if (weaponOwnershipKey == 0) {
            weaponOwnershipKey = currentEquippedWeaponFormId();
        }
        const std::uint64_t weaponGenerationKey =
            authored_support_grab_policy::resolveAuthoredGenerationKey(
                _weaponCollision.getCurrentWeaponGenerationKey(),
                weaponOwnershipKey);
        const bool equippedGenerationMatchesForm =
            weaponGenerationKey != 0 &&
            weaponOwnershipKey != 0 &&
            _weaponCollision.getCurrentObservedEquippedWeaponFormID() == equippedWeapon->formID;
        const auto nativeAuthorityFlags = provider::currentNativeAnimationAuthorityFlagsV1();
        if (!equippedGenerationMatchesForm || !weaponNode->parent || !f4vr::isNodeVisible(weaponNode) ||
            (nativeAuthorityFlags & authored_weapon_grip_capture_policy::kWeapon) != 0) {
            return;
        }
        const float parentScale = weaponNode->parent->world.scale;
        float animationScale = 1.0f;
        const bool animationAvailable = authored_weapon_grip_capture::tryGetAnimationWeaponScale(
            weaponNode, equippedWeapon->formID, animationScale);
        const float desiredScale = authored_weapon_grip_capture_policy::resolveWeaponPresentationScale(animationAvailable, animationScale);
        if (std::isfinite(parentScale) && std::abs(parentScale) > 0.0001f &&
            std::abs(weaponNode->world.scale - desiredScale) > 0.00001f) {
            const float previousScale = weaponNode->world.scale;
            weaponNode->local.scale = desiredScale / parentScale;
            f4vr::updateDown(weaponNode, true);
            ROCK_LOG_SAMPLE_INFO(Animation, 2000,
                "Weapon presentation scale form={:08X} incoming={:.6f} target={:.6f} result={:.6f} source={}",
                equippedWeapon->formID, previousScale, desiredScale, weaponNode->world.scale,
                animationAvailable ? "animation-graph" : "unit-default");
        }
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
            _equipped.transition.getPublicSnapshot().active;

        native_idle_grip_preharvest::observeEquippedWeapon(
            equippedGenerationMatchesForm ? equippedWeapon : nullptr,
            equippedGenerationMatchesForm ? weaponNode : nullptr,
            equippedGenerationMatchesForm ? currentEquippedWeaponInstanceData(equippedWeapon) : nullptr,
            equippedGenerationMatchesForm ? _weaponCollision.getCurrentEquippedWeaponInstanceContentKey() : 0);

        _twoHandedGrip.beginAuthoredPrimaryFiringGripFrame();
        _equipped.authoredPrimaryFiringGrip.update(AuthoredPrimaryFiringGripFrameInput{
            .weaponNode = weaponNode,
            .weapon = equippedWeapon,
            .weaponOwnershipKey = weaponOwnershipKey,
            .weaponGenerationKey = weaponGenerationKey,
            .weaponInstanceContentKey = equippedGenerationMatchesForm ? _weaponCollision.getCurrentEquippedWeaponInstanceContentKey() : 0,
            .weaponKeywordFlags = weaponKeywordFlags,
            .weaponInstanceContentKnown = equippedGenerationMatchesForm,
            .runtimeInitialized = _lifecycle.initialized.load(std::memory_order_acquire),
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
        if (_equipped.transition.isHandPoseHandoffActive()) {
            const bool handoffHandIsLeft = _equipped.transition.handPoseHandoffIsLeft();
            const bool pairedSupportOwned = !_twoHandedGrip.isPartCarryActive() &&
                _twoHandedGrip.isHandPartGripping(!handoffHandIsLeft);
            if (nativeAuthorityFlags != 0 ||
                runtime.localMenuBlocking ||
                runtime.compatibilityConfigBlocking) {
                _equipped.transition.completeHandPoseHandoff("authored-pose-unavailable");
            } else if (equippedWeapon && equippedWeapon->formID != _equipped.transition.bridgeWeaponBaseFormID()) {
                _equipped.transition.completeHandPoseHandoff("equipped-weapon-changed");
            } else if (_equipped.transition.hasPairedHandPoseHandoff() &&
                !_equipped.pendingPrimaryOnlyGripStart.pairedGrips.valid() && !pairedSupportOwned) {
                _equipped.transition.completeHandPoseHandoff("paired-transfer-ended");
            } else if (_twoHandedGrip.hasPublishedAuthoredPrimaryFiringGripFingerPose(handoffHandIsLeft) &&
                (!_equipped.transition.hasPairedHandPoseHandoff() || pairedSupportOwned)) {
                _equipped.transition.completeHandPoseHandoff("equipped-authored-pose-acquired");
            }
        }
    }

    void PhysicsInteraction::refreshEquippedWeaponHandlingSettings()
    {
        // Frame updates begin after GameLoaded, when F4SE has loaded every
        // plugin. Cache module presence for this process without polling or
        // changing the user's configuration on reloads or new game sessions.
        const bool virtualHolstersLoaded = virtual_holsters::isLoaded();

        ::rock::provider::RockProviderEquippedWeaponHandlingRequestV1 request{};
        const bool externalAuthorityActive =
            ::rock::provider::getEquippedWeaponHandlingAuthorityV1(request);
        const RockEquippedWeaponHandlingBaseline rockBaseline{
            .ambidextrousHandoffEnabled =
                g_rockConfig.rockAmbidextrousFiringGripEnabled,
            .authoredOnlySupportGrabsEnabled =
                !g_rockConfig.rockGrabAnywhereOnWeapon,
            .weaponGrabMode =
                equipped_weapon_toggle_grab_policy::fromSetting(g_rockConfig.rockWeaponGrabMode),
            .equippedWeaponShoulderStashEnabled =
                g_rockConfig.rockEquippedWeaponShoulderStashEnabled &&
                !virtualHolstersLoaded,
            .lastGripReleaseDropEnabled =
                equipped_weapon_drop_policy::fromSetting(g_rockConfig.rockWeaponDropMode) != equipped_weapon_drop_policy::Mode::Off,
            .immersiveWeapon = {
                .firingGripDetachEnabled =
                    g_rockConfig.
                        rockDetachEitherHand,
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
            .firingGripReattachCylinderRadiusGameUnits =
                g_rockConfig.rockFiringGripReattachCylinderRadiusGameUnits,
            .firingGripProximitySupportRadiusGameUnits =
                g_rockConfig.rockFiringGripProximitySupportRadius,
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

        if (_equipped.handlingModeInitialized) {
            if (requiresEquippedWeaponHandlingModeReconcile(
                    _equipped.handlingSettings,
                    settings)) {
                _equipped.handlingModeReconcilePending = true;
            }
        }

        _equipped.handlingSettings = settings;
        _equipped.handlingModeInitialized = true;
        equipped_weapon_handling_runtime::publish(settings);
    }

    void PhysicsInteraction::reconcileEquippedWeaponHandlingMode()
    {
        if (!_equipped.handlingModeReconcilePending) {
            return;
        }

        _twoHandedGrip.restoreNativeRightEquippedCarry(
            "equipped-weapon-handling-mode-changed");
        _equipped.pendingPrimaryOnlyGripStart = {};
        _equipped.handlingModeReconcilePending = false;
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
                _equipped.toggleGrabState);
            _twoHandedGrip.restoreNativeRightEquippedCarry(
                "shoulder-weapon-sheathed");
            _equipped.pendingPrimaryOnlyGripStart = {};
            _equipped.shoulderSheath =
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
            _equipped.sheathRetrievalStates = {};
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
            _equipped.stashStates[stashHandIndex]);
        return sheathAccepted;
    }

    void PhysicsInteraction::clearEquippedWeaponShoulderSheath(
        const char* reason,
        const bool resetCoordinator)
    {
        if (_equipped.shoulderSheath.active) {
            ROCK_LOG_INFO(
                Weapon,
                "Equipped weapon shoulder sheath cleared reason={} formID={:08X} instance={:#x} zone={}",
                reason ? reason : "unknown",
                _equipped.shoulderSheath.weaponFormID,
                _equipped.shoulderSheath.weaponInstanceData,
                body_zone::bodyZoneName(
                    _equipped.shoulderSheath.zone));
        }
        _equipped.shoulderSheath = {};
        input_remap_runtime::setEquippedWeaponShoulderSheathActive(false);
        _equipped.sheathRetrievalStates = {};
        if (resetCoordinator) {
            equipped_weapon_shoulder::reset(
                _equipped.shoulderCoordinator);
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
        if (_equipped.shoulderSheath.active) {
            const bool identityMatches = capturedCurrentIdentity &&
                currentIdentity.formID ==
                    _equipped.shoulderSheath.weaponFormID &&
                currentIdentity.instanceData ==
                    _equipped.shoulderSheath.weaponInstanceData &&
                currentIdentity.equipIndex ==
                    _equipped.shoulderSheath.equipIndex &&
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
        const auto providerGripOccupancy = _twoHandedGrip.getGripOccupancy();
        maskProviderWeaponGrabInput(true, providerGripOccupancy.left.weaponEngaged(), leftPhysicalGrip);
        maskProviderWeaponGrabInput(false, providerGripOccupancy.right.weaponEngaged(), rightPhysicalGrip);
        const std::uint64_t shoulderOwnershipKey =
            _equipped.shoulderSheath.active ?
            _equipped.shoulderSheath.weaponOwnershipKey :
            _equipped.shoulderCoordinator.activeAction !=
                    equipped_weapon_shoulder::Action::None ?
                _equipped.shoulderCoordinator.weaponOwnershipKey :
                currentEquippedWeaponOwnershipKey;

        const auto sheathModeForHand = [&](bool isLeft) {
            const bool firingRole = _equipped.shoulderSheath.active || isLeft == firingHandIsLeft;
            return equipped_weapon_shoulder::resolveSheathInputMode(
                _equipped.handlingSettings.immersiveWeapon.firingGripDetachEnabled,
                equipped_weapon_toggle_grab_policy::usesToggleForRole(_equipped.handlingSettings.weaponGrabMode, firingRole));
        };
        equipped_weapon_shoulder::FrameInput coordinatorInput{
            .enabled = handlingEnabled,
            .inputAllowed = !menuInputActive,
            .storedActive = _equipped.shoulderSheath.active,
            .stashedByLeftHand =
                _equipped.shoulderSheath.stashedByLeftHand,
            .weaponOwnershipKey = shoulderOwnershipKey,
            .presentation = nativePresentation,
            .storedZone = _equipped.shoulderSheath.zone,
            .right = {
                .sheathInputMode = sheathModeForHand(false),
                .button = {
                    .held = rightPhysicalGrip.held,
                    .pressed = rightPhysicalGrip.pressed,
                    .released = rightPhysicalGrip.released,
                },
            },
            .left = {
                .sheathInputMode = sheathModeForHand(true),
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
            const bool pulseDue = _contacts.dynamicPushElapsedSeconds >=
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
                _contacts.dynamicPushElapsedSeconds +
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

        if (_equipped.shoulderSheath.active) {
            _equipped.stashStates = {};
            const bool roleNeutralFiringGripOwnership =
                resolveEquippedWeaponDetachDecision(
                    _equipped.handlingSettings).
                    firingGripOwnershipEnabled;
            for (const bool isLeft : { false, true }) {
                const std::size_t handIndex = isLeft ? 1u : 0u;
                Hand& hand = isLeft ? _leftHand : _rightHand;
                const HandFrameInput& handInput =
                    isLeft ? frame.left : frame.right;
                auto& detectorState =
                    _equipped.sheathRetrievalStates[handIndex];
                const bool handEmpty = !hand.isHolding() &&
                    !_touchGrabRuntime.isHandActive(isLeft) &&
                    !_forceGrab.pendingCommits[handIndex].active &&
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
                        _equipped.shoulderSheath.zone,
                    decision,
                    detectorState);
            }
        } else {
            _equipped.sheathRetrievalStates = {};
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
                    _equipped.stashStates[handIndex];
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
                    !_forceGrab.pendingCommits[handIndex].active &&
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
            _equipped.shoulderCoordinator.phase;
        result.decision = equipped_weapon_shoulder::advance(
            _equipped.shoulderCoordinator,
            coordinatorInput);
        _equipped.shoulderGestureConsumedThisFrame[0] =
            result.decision.consumeRightInput;
        _equipped.shoulderGestureConsumedThisFrame[1] =
            result.decision.consumeLeftInput;

        if (result.decision.action !=
                equipped_weapon_shoulder::Action::None) {
            const auto& selectedDetector = detectorDecisions[
                result.decision.hand ==
                        equipped_weapon_shoulder::Hand::Left ?
                    1u : 0u];
            ROCK_LOG_INFO(
                Weapon,
                "Equipped shoulder coordinator action={} reason={} phase={}->{} hand={} gesture={} zone={} candidate={} confirmed={} source={} confidence={:.2f} speed={:.1f} nativeState={}({}) sheathInput={} immersive={} grabMode={}",
                equipped_weapon_shoulder::actionName(
                    result.decision.action),
                equipped_weapon_shoulder::reasonName(
                    result.decision.reason),
                equipped_weapon_shoulder::phaseName(previousPhase),
                equipped_weapon_shoulder::phaseName(
                    _equipped.shoulderCoordinator.phase),
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
                    result.decision.hand == equipped_weapon_shoulder::Hand::Left ?
                        coordinatorInput.left.sheathInputMode : coordinatorInput.right.sheathInputMode),
                _equipped.handlingSettings.immersiveWeapon.
                        firingGripDetachEnabled ?
                    "yes" : "no",
                equipped_weapon_toggle_grab_policy::modeName(_equipped.handlingSettings.weaponGrabMode));
        } else if (previousPhase !=
                   _equipped.shoulderCoordinator.phase) {
            ROCK_LOG_DEBUG(
                Weapon,
                "Equipped shoulder coordinator phase={}->{} reason={} gestureHand={} gesture={} nativeState={}({}) stored={}",
                equipped_weapon_shoulder::phaseName(previousPhase),
                equipped_weapon_shoulder::phaseName(
                    _equipped.shoulderCoordinator.phase),
                equipped_weapon_shoulder::reasonName(
                    result.decision.reason),
                equipped_weapon_shoulder::handName(
                    _equipped.shoulderCoordinator.actionHand),
                _equipped.shoulderCoordinator.actionGestureSerial,
                nativeWeaponState,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    nativeWeaponState),
                _equipped.shoulderSheath.active ? "yes" : "no");
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
            !_equipped.shoulderSheath.active) {
            equipped_weapon_shoulder::reportExecutionResult(
                _equipped.shoulderCoordinator,
                result.decision.action,
                false);
            ROCK_LOG_WARN(
                Weapon,
                "Equipped weapon shoulder retrieval rejected after coordinator selection because exact stored identity is unavailable");
            return result;
        }

        const auto storedZone = _equipped.shoulderSheath.zone;
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
            _equipped.shoulderCoordinator,
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
                _equipped.sheathRetrievalStates[handIndex]);
            return result;
        }

        _equipped.pendingPrimaryOnlyGripStart =
            PendingEquippedWeaponPrimaryOnlyGripStart{
                .pending = true,
                .isLeft = retrieveWithLeftHand,
                .targetWeaponFormID = currentIdentity.formID,
                .targetWeaponInstanceData = currentIdentity.instanceData,
                .remainingSeconds = 10.0f,
                .source = equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource::ShoulderRetrieval,
                .hasFiringHandWeaponLocal = retrieveWithLeftHand &&
                    _equipped.shoulderSheath.
                        hasLeftFiringGripTransfer,
                .firingHandWeaponLocal =
                    _equipped.shoulderSheath.
                        leftFiringHandWeaponLocal,
                .hasFiringGripWeaponLocal = retrieveWithLeftHand &&
                    _equipped.shoulderSheath.
                        hasLeftFiringGripTransfer,
                .firingGripWeaponLocal =
                    _equipped.shoulderSheath.
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
