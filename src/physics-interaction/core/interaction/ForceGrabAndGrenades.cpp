#include "physics-interaction/core/PhysicsInteractionInternal.h"

// Provider force-grab commits, fuse service, and held-grenade arming.

namespace rock
{
    namespace
    {
        void rollbackInventoryTransfer(const PendingForceGrabCommit& commit)
        {
            if (!commit.inventoryTransfer) {
                return;
            }
            const auto ref = commit.targetHandle.get();
            const bool returned = ref && loose_grenade_runtime::returnDroppedReferenceToInventory(ref.get());
            f4vr::showNotification(returned ?
                "ROCK: Item handoff failed; returned to inventory." :
                "ROCK: Item handoff failed; check the dropped item near your hand.");
            ROCK_LOG_WARN(Hand, "Inventory handoff rollback: command={} ref={:08X} returned={}",
                commit.providerResultTemplate.commandId, ref ? ref->GetFormID() : 0, returned);
        }
    }

    void PhysicsInteraction::clearLooseGrenadeImpactWatches()
    {
        for (auto& bodyId : _forceGrab.grenadeImpactBodyIds) {
            bodyId.store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
        }
        _forceGrab.pendingGrenadeImpactPair.store(INVALID_HELD_IMPACT_PAIR, std::memory_order_release);
    }

    void PhysicsInteraction::clearPendingForceGrabCommits()
    {
        _forceGrab.retainedWeaponGrabs = {};
        for (auto& commit : _forceGrab.pendingCommits) {
            if (!commit.active) continue;
            if (provider::isInteractionCommandActiveV1(commit.providerResultTemplate.ownerToken,commit.providerResultTemplate.commandId)) {
                commit.providerResultTemplate.state = provider::RockProviderInteractionCommandStateV1::Cancelled;
                commit.providerResultTemplate.failure = provider::RockProviderInteractionFailureV1::ProviderNotReady;
                provider::completeInteractionCommandV1(commit.providerResultTemplate);
            }
            rollbackInventoryTransfer(commit);
            commit = {};
        }
    }

    void PhysicsInteraction::clearLooseGrenadeRuntimeState()
    {
        _forceGrab.grenadeFuses = {};
        clearLooseGrenadeImpactWatches();
    }

    std::uint32_t PhysicsInteraction::forceGrabHandBlockerMask(
        const Hand& hand,
        bool isLeft,
        bool handDisabled,
        bool includePendingCommit) const
    {
        const auto state = hand.getState();
        const bool openInteractionState =
            state == HandState::Idle || state == HandState::SelectedClose || state == HandState::SelectedFar;
        // Equip data remains authoritative while menus temporarily hide or
        // detach the weapon's 3D node.
        const bool equippedWeaponPresent = currentEquippedWeaponOccupiesHand();
        const bool equippedWeaponOccupiesHand = _equipped.pendingPrimaryOnlyGripStart.pairedGrips.valid() ||
            force_grab_policy::equippedWeaponOccupiesHand(
            isLeft,
            equippedWeaponPresent,
            _twoHandedGrip.isPartCarryActive(),
            _twoHandedGrip.isFiringHandLeft(),
            _twoHandedGrip.isHandPartGripping(isLeft));

        return force_grab_policy::blockerMask(force_grab_policy::HandAvailabilityInput{
            .disabled = handDisabled,
            .openInteractionState = openInteractionState,
            .holding = hand.isHolding(),
            .activePullCatch = hand.hasActivePullCatchIntent(),
            .actorEquipmentHandoff = hand.hasPendingActorEquipmentDropHandoff(),
            .pendingForceGrab = includePendingCommit && _forceGrab.pendingCommits[isLeft ? 1u : 0u].active,
            .equippedWeaponOccupiesHand = equippedWeaponOccupiesHand,
            .touchGrabActive = _touchGrabRuntime.isHandActive(isLeft),
            .inputReserved = input_remap_runtime::ownsBareFistInput(),
        });
    }

    bool PhysicsInteraction::canHandAcceptForceGrab(const Hand& hand, bool isLeft, bool handDisabled) const
    {
        return forceGrabHandBlockerMask(hand, isLeft, handDisabled, false) == 0;
    }

    bool PhysicsInteraction::handHoldsLooseGrenade(const Hand& hand) const
    {
        return hand.isHolding() && loose_grenade_runtime::isThrowableRef(hand.getHeldRef());
    }

    bool PhysicsInteraction::hasActiveLooseGrenadeCommit() const
    {
        for (const auto& commit : _forceGrab.pendingCommits) {
            if (commit.active && commit.targetIsLooseThrowable) {
                return true;
            }
        }
        return false;
    }

    bool PhysicsInteraction::isPendingForceGrabTarget(RE::TESObjectREFR* ref) const
    {
        if (!ref) {
            return false;
        }
        for (const auto& commit : _forceGrab.pendingCommits) {
            if (!commit.active) {
                continue;
            }
            const auto targetRefPtr = commit.targetHandle.get();
            if (targetRefPtr.get() == ref) {
                return true;
            }
        }
        return false;
    }

    void PhysicsInteraction::pruneInactiveProviderForceGrabCommits()
    {
        for (auto& commit : _forceGrab.pendingCommits) {
            if (commit.active && !commit.internallyOwned() &&
                !provider::isInteractionCommandActiveV1(
                    commit.providerResultTemplate.ownerToken,
                    commit.providerResultTemplate.commandId)) {
                rollbackInventoryTransfer(commit);
                commit = {};
            }
        }
    }

    void PhysicsInteraction::serviceLooseGrenadeQuickDraw(const PhysicsFrameContext& frame)
    {
        if (!input_remap_runtime::consumeGrenadeQuickDrawHoldRequest()) return;
        if (!frame.worldReady || !frame.bhkWorld || !frame.hknpWorld ||
            !physicsWritesAllowedForWorld(frame.hknpWorld)) return;
        if (handHoldsLooseGrenade(_rightHand) || handHoldsLooseGrenade(_leftHand) || hasActiveLooseGrenadeCommit()) {
            f4vr::showNotification("ROCK: A throwable is already held or attaching.");
            return;
        }
        const bool rightFree = forceGrabHandBlockerMask(_rightHand, false, frame.right.disabled, true) == 0;
        const bool leftFree = forceGrabHandBlockerMask(_leftHand, true, frame.left.disabled, true) == 0;
        if (!rightFree && !leftFree) {
            f4vr::showNotification("ROCK: Cannot draw throwable - both hands are occupied.");
            return;
        }
        const bool isLeft = force_grab_policy::selectGrenadeHand(false, rightFree, leftFree).hand == force_grab_policy::HandChoice::Left;
        auto location = (isLeft ? frame.left : frame.right).grabAnchorWorld;
        location.z -= 3.0f;
        const auto drop = loose_grenade_runtime::dropEquippedThrowableToWorld(location);
        if (!drop.success) {
            f4vr::showNotification("ROCK: Equip a supported grenade or throwable in the Pip-Boy first.");
            ROCK_LOG_WARN(Hand, "Grenade mode draw rejected: {}", drop.reason);
            return;
        }
        _forceGrab.pendingCommits[isLeft ? 1u : 0u] = PendingForceGrabCommit{
            .active = true,
            .isLeft = isLeft,
            .phase = PendingForceGrabCommitPhase::WaitingForReference,
            .targetHandle = drop.handle,
            .targetIsLooseThrowable = true,
            .inventoryTransfer = true,
            .grenadeQuickDraw = true,
            .maxDistanceGame = 96.0f,
        };
        ROCK_LOG_INFO(Hand, "Grenade mode draw: ref={:08X} stack={} hand={}",
            drop.droppedRef ? drop.droppedRef->GetFormID() : 0, drop.stackId, isLeft ? "left" : "right");
    }

    void PhysicsInteraction::servicePendingForceGrabCommits(const PhysicsFrameContext& frame)
    {
        if (!frame.worldReady || !frame.bhkWorld || !frame.hknpWorld) {
            return;
        }

        for (auto& commit : _forceGrab.pendingCommits) {
            if (!commit.active) {
                continue;
            }

            Hand& hand = commit.isLeft ? _leftHand : _rightHand;
            const auto& handInput = commit.isLeft ? frame.left : frame.right;

            auto abandon = [&](const char* reason,
                               provider::RockProviderInteractionFailureV1 providerFailure,
                               RE::TESObjectREFR*) {
                commit.providerResultTemplate.state = provider::RockProviderInteractionCommandStateV1::Rejected;
                commit.providerResultTemplate.failure = providerFailure;
                if (!commit.internallyOwned()) {
                    provider::completeInteractionCommandV1(commit.providerResultTemplate);
                }
                if (commit.equippedWeaponTransfer) {
                    for (auto& handoff : _drop.nativeHandoffs) {
                        if (handoff.active && handoff.handle == commit.targetHandle) {
                            reportEquippedWeaponPlacementFailure(handoff, reason);
                            handoff = {};
                        }
                    }
                    _forceGrab.retainedWeaponGrabs[commit.isLeft ? 1u : 0u] = {};
                }
                rollbackInventoryTransfer(commit);
                ROCK_LOG_WARN(Hand,
                    "Pending force-grab commit abandoned ({}): hand={}",
                    reason,
                    commit.isLeft ? "left" : "right");
                commit = {};
            };

            if (!commit.internallyOwned() && !provider::isInteractionCommandActiveV1(
                    commit.providerResultTemplate.ownerToken,
                    commit.providerResultTemplate.commandId)) {
                ROCK_LOG_INFO(Hand,
                    "Pending provider force-grab commit cancelled because its command is no longer active: command={} hand={}",
                    commit.providerResultTemplate.commandId,
                    commit.isLeft ? "left" : "right");
                rollbackInventoryTransfer(commit);
                commit = {};
                continue;
            }

            commit.elapsedTotalSeconds += (std::max)(0.0f, frame.deltaSeconds);
            const bool timedOut = commit.elapsedTotalSeconds >= commit.maxTotalSeconds;

            auto targetRefPtr = commit.targetHandle.get();
            auto* targetRef = targetRefPtr.get();
            if (!targetRef) {
                if (timedOut) {
                    abandon("target ref did not resolve", provider::RockProviderInteractionFailureV1::TargetUnavailable, nullptr);
                }
                continue;
            }
            if (targetRef->IsDeleted() || targetRef->IsDisabled()) {
                abandon("target ref disappeared", provider::RockProviderInteractionFailureV1::TargetUnavailable, targetRef);
                continue;
            }

            if (commit.phase == PendingForceGrabCommitPhase::EquippedSlotReleaseFailed) {
                abandon("equipped weapon slot was not released", provider::RockProviderInteractionFailureV1::HandBusy, targetRef);
                continue;
            }
            if (commit.phase == PendingForceGrabCommitPhase::NativePlacementFailed) {
                abandon("native weapon placement failed", provider::RockProviderInteractionFailureV1::TargetUnavailable, targetRef);
                continue;
            }
            if (commit.phase == PendingForceGrabCommitPhase::WaitingForNativePlacement) {
                if (timedOut) {
                    abandon("native weapon placement timed out", provider::RockProviderInteractionFailureV1::TargetUnavailable, targetRef);
                }
                continue;
            }

            if (commit.phase == PendingForceGrabCommitPhase::WaitingForReference) {
                if (commit.inventoryTransfer) {
                    commit.providerResultTemplate.targetFormId = targetRef->GetFormID();
                }
                commit.phase = PendingForceGrabCommitPhase::WaitingForSettle;
            }
            if (!canHandAcceptForceGrab(hand, commit.isLeft, handInput.disabled)) {
                if (timedOut) {
                    abandon("hand busy", provider::RockProviderInteractionFailureV1::HandBusy, targetRef);
                }
                continue;
            }

            if (commit.phase == PendingForceGrabCommitPhase::WaitingForSettle) {
                commit.elapsedSettleSeconds += (std::max)(0.0f, frame.deltaSeconds);
                if (commit.elapsedSettleSeconds < g_rockConfig.rockForceGrabAttachSettleSeconds) {
                    continue;
                }
            }

            if (hand.hasSelection()) {
                hand.clearSelectionState(false);
            }
            const RE::NiPoint3 sourcePoint = commit.hasSourcePointOverride ? commit.sourcePointOverride : handInput.grabAnchorWorld;
            if (!hand.acquireForceGrabLooseSelection(frame.bhkWorld,
                    frame.hknpWorld,
                    targetRef,
                    sourcePoint,
                    commit.preferredBodyId,
                    commit.maxDistanceGame,
                    commit.inventoryTransfer &&
                        commit.targetIsLooseThrowable &&
                        loose_grenade_runtime::isThrowableRef(targetRef),
                    commit.equippedWeaponTransfer)) {
                commit.phase = PendingForceGrabCommitPhase::WaitingForSettle;
                if (timedOut) {
                    abandon("failed to resolve physics body", provider::RockProviderInteractionFailureV1::TargetBodyMissing, targetRef);
                }
                continue;
            }

            if (hand.getSelection().refr != targetRef) {
                hand.clearSelectionState(false);
                commit.phase = PendingForceGrabCommitPhase::WaitingForSettle;
                if (timedOut) {
                    abandon("selection did not retain exact target", provider::RockProviderInteractionFailureV1::TargetUnavailable, targetRef);
                }
                continue;
            }
            if (commit.preferredBodyId != INVALID_BODY_ID && hand.getSelection().bodyId.value != commit.preferredBodyId) {
                const std::uint32_t resolvedBodyId = hand.getSelection().bodyId.value;
                hand.clearSelectionState(false);

                commit.providerResultTemplate.targetBodyId = resolvedBodyId;

                abandon("resolved body does not match requested body", provider::RockProviderInteractionFailureV1::TargetBodyMissing, targetRef);
                continue;
            }

            /*
             * Acquire and commit the exact handle target in one update. Never
             * persist a ready phase that organic selection can replace before
             * the retry. Saved/calibrated offsets remain applied by the normal
             * grabSelectedObject commit path through forcedArrival.
             */
            commit.phase = PendingForceGrabCommitPhase::AcquireAndCommitExactTarget;

            const auto sharedContext = makeGrabSharedObjectContext(hand, commit.isLeft);
            prepareDynamicWorldCarCollisionForGrab(frame.bhkWorld, frame.hknpWorld, targetRef);
            const bool grabbed = hand.grabSelectedObject(frame.hknpWorld,
                handInput.rawHandWorld,
                g_rockConfig.rockGrabLinearTau,
                g_rockConfig.rockGrabLinearDamping,
                g_rockConfig.rockGrabConstraintMaxForce,
                g_rockConfig.rockGrabLinearProportionalRecovery,
                g_rockConfig.rockGrabLinearConstantRecovery,
                &_bodyBoneColliders,
                sharedContext,
                commit.equippedWeaponTransfer ? &commit.weaponGripPose : nullptr) == GrabAttemptResult::Grabbed;
            if (!grabbed) {
                hand.clearSelectionState(false);
                commit.phase = PendingForceGrabCommitPhase::WaitingForSettle;
                if (timedOut) {
                    abandon("failed to commit grab", provider::RockProviderInteractionFailureV1::TargetUnavailable, targetRef);
                }
                continue;
            }

            auto* heldRef = hand.getHeldRef();
            const std::uint32_t primaryBodyId = hand.getSavedObjectState().bodyId.value;
            const bool exactBody = commit.preferredBodyId == INVALID_BODY_ID || primaryBodyId == commit.preferredBodyId;
            if (heldRef != targetRef || !exactBody) {
                hand.releaseGrabbedObject(
                    frame.hknpWorld,
                    GrabReleaseCollisionRestoreMode::Immediate,
                    makeGrabReleaseContext(hand, commit.isLeft));
                abandon("grab postcondition did not match exact target", provider::RockProviderInteractionFailureV1::TargetUnavailable, targetRef);
                continue;
            }

            commit.providerResultTemplate.targetBodyId = primaryBodyId;
            commit.providerResultTemplate.state = provider::RockProviderInteractionCommandStateV1::Succeeded;
            commit.providerResultTemplate.failure = provider::RockProviderInteractionFailureV1::None;
            if (!commit.internallyOwned() && !provider::completeInteractionCommandV1(commit.providerResultTemplate)) {
                /*
                 * Owner/provider loss can race the final main-thread
                 * commit. Do not publish or retain a grab whose command
                 * reservation was cancelled before the terminal result.
                 */
                hand.releaseGrabbedObject(
                    frame.hknpWorld,
                    GrabReleaseCollisionRestoreMode::Immediate,
                    makeGrabReleaseContext(hand, commit.isLeft));
                ROCK_LOG_INFO(Hand,
                    "Provider force-grab rolled back because command ownership ended during commit: command={} hand={}",
                    commit.providerResultTemplate.commandId,
                    commit.isLeft ? "left" : "right");
                rollbackInventoryTransfer(commit);
                commit = {};
                continue;
            }

            if (commit.equippedWeaponTransfer) {
                _forceGrab.retainedWeaponGrabs[commit.isLeft ? 1u : 0u].grabIdentity = hand.heldGrabIdentity();
                ROCK_LOG_INFO(Weapon,
                    "Equipped weapon acquired as retained loose grab: hand={} ref={:08X} body={} grab={} handleMatches={} inputState={}",
                    commit.isLeft ? "left" : "right", heldRef->GetFormID(), primaryBodyId, hand.heldGrabIdentity(),
                    heldRef->GetHandle() == commit.targetHandle,
                    static_cast<unsigned>(_forceGrab.retainedWeaponGrabs[commit.isLeft ? 1u : 0u].inputState));
            }
            claimObject(heldRef, claimOwnerForHand(commit.isLeft));
            dispatchPhysicsMessage(kPhysMsg_OnGrab, commit.isLeft, heldRef, heldRef ? heldRef->GetFormID() : 0, 0);
            dispatchGrabCommittedEvent(commit.isLeft, heldRef, primaryBodyId, frame.hknpWorld);
            input_remap_runtime::setHandHeldWeapon(commit.isLeft, (commit.isLeft ? _leftHand : _rightHand).isHoldingLooseWeapon());

            _forceGrab.committedThisFrame[commit.isLeft ? 1u : 0u] = true;
            commit = {};
        }
    }

    bool PhysicsInteraction::armHeldLooseGrenade(Hand& hand, const PhysicsFrameContext& frame)
    {
        auto* heldRef = hand.getHeldRef();
        if (!heldRef || !loose_grenade_runtime::isThrowableRef(heldRef)) {
            return false;
        }

        for (const auto& fuse : _forceGrab.grenadeFuses) {
            const auto fuseRefPtr = fuse.handle.get();
            if (fuse.active && fuseRefPtr.get() == heldRef) {
                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} hand loose throwable trigger ignored because activation is already active: ref={:08X} remaining={:.3f}s",
                    hand.handName(),
                    heldRef->GetFormID(),
                    fuse.remainingSeconds);
                return true;
            }
        }

        loose_grenade_runtime::GrenadeRuntimeData runtime{};
        if (!loose_grenade_runtime::resolveGrenadeRuntimeDataForReference(heldRef, runtime)) {
            ROCK_LOG_WARN(Hand,
                "{} hand loose throwable trigger could not arm because its authored runtime data is unsupported: ref={:08X}",
                hand.handName(),
                heldRef->GetFormID());
            return true;
        }

        std::uint32_t impactBodyId = INVALID_CONTACT_BODY_ID;
        if (runtime.detonationMode == loose_grenade_runtime::GrenadeDetonationMode::Impact) {
            const std::uint32_t primaryBodyId = hand.getSavedObjectState().bodyId.value;
            if (!isInvalidGrabBodyId(primaryBodyId)) {
                impactBodyId = primaryBodyId;
            } else {
                for (const auto bodyId : hand.getHeldBodyIds()) {
                    if (!isInvalidGrabBodyId(bodyId)) {
                        impactBodyId = bodyId;
                        break;
                    }
                }
            }

            if (impactBodyId == INVALID_CONTACT_BODY_ID) {
                ROCK_LOG_WARN(Hand,
                    "{} hand loose impact throwable could not arm because no held body id was available: ref={:08X}",
                    hand.handName(),
                    heldRef->GetFormID());
                return true;
            }
        }

        for (std::size_t slotIndex = 0; slotIndex < _forceGrab.grenadeFuses.size(); ++slotIndex) {
            auto& fuse = _forceGrab.grenadeFuses[slotIndex];
            if (fuse.active) {
                continue;
            }

            fuse = ArmedLooseGrenadeFuseState{
                .active = true,
                .handle = heldRef->GetHandle(),
                .refFormID = heldRef->GetFormID(),
                .runtime = runtime,
                .remainingSeconds = runtime.fuseSeconds,
                .impactBodyId = impactBodyId,
                .releasedSinceArming = false,
            };
            _forceGrab.grenadeImpactBodyIds[slotIndex].store(
                runtime.detonationMode == loose_grenade_runtime::GrenadeDetonationMode::Impact ? impactBodyId : INVALID_CONTACT_BODY_ID,
                std::memory_order_release);
            ROCK_LOG_INFO(Hand,
                "{} hand armed loose throwable: ref={:08X} projectile={:08X} explosion={:08X} mode={} delay={:.3f}s proximity={:.1f}gu preserveRef={} impactBody={} frameDt={:.4f}",
                hand.handName(),
                heldRef->GetFormID(),
                runtime.projectile ? runtime.projectile->GetFormID() : 0,
                runtime.explosion ? runtime.explosion->GetFormID() : 0,
                loose_grenade_runtime::detonationModeName(runtime.detonationMode),
                runtime.fuseSeconds,
                runtime.proximityRadiusGameUnits,
                runtime.preserveReferenceAfterDetonation ? "yes" : "no",
                impactBodyId,
                frame.deltaSeconds);
            const bool feedbackPlayed = loose_grenade_runtime::playPinPulledFeedbackAtReference(heldRef);
            ROCK_LOG_DEBUG(Hand,
                "{} hand loose throwable activation feedback: ref={:08X} played={}",
                hand.handName(),
                heldRef->GetFormID(),
                feedbackPlayed ? "yes" : "no");
            return true;
        }

        ROCK_LOG_WARN(Hand,
            "{} hand loose throwable trigger could not arm because armed throwable capacity is full: ref={:08X}",
            hand.handName(),
            heldRef->GetFormID());
        return true;
    }

    void PhysicsInteraction::updateLooseGrenadeFuses(const PhysicsFrameContext& frame)
    {
        constexpr float kPlacedMineProximityPollSeconds = 0.10f;
        const float deltaSeconds = (std::max)(0.0f, frame.deltaSeconds);
        if (deltaSeconds <= 0.0f) {
            return;
        }

        std::uint32_t pendingImpactBodyId = INVALID_CONTACT_BODY_ID;
        std::uint32_t pendingImpactOtherBodyId = INVALID_CONTACT_BODY_ID;
        const bool pendingImpact =
            unpackHeldImpactPair(
                _forceGrab.pendingGrenadeImpactPair.exchange(INVALID_HELD_IMPACT_PAIR, std::memory_order_acq_rel),
                pendingImpactBodyId,
                pendingImpactOtherBodyId);

        auto releaseHandIfHolding = [&](Hand& hand, bool isLeft, RE::TESObjectREFR* ref, std::uint32_t formID) {
            if (!ref || !hand.isHolding() || hand.getHeldRef() != ref) {
                return;
            }

            const auto releaseContext = makeGrabReleaseContext(hand, isLeft);
            static_cast<void>(hand.releaseGrabbedObject(frame.hknpWorld, GrabReleaseCollisionRestoreMode::Immediate, releaseContext));
            releaseObject(ref, claimOwnerForHand(isLeft));
            dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, ref, formID, 0);
            dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, ref);
        };

        auto detonateLooseGrenade = [&](ArmedLooseGrenadeFuseState& fuse,
                                         std::size_t slotIndex,
                                         RE::TESObjectREFR* ref,
                                         const char* reason,
                                         std::uint32_t triggerActorFormID = 0) {
            const std::uint32_t formID = ref ? ref->GetFormID() : fuse.refFormID;
            releaseHandIfHolding(_rightHand, false, ref, formID);
            releaseHandIfHolding(_leftHand, true, ref, formID);

            const bool explosionCreated = loose_grenade_runtime::createExplosionAtReference(ref, fuse.runtime.explosion);
            if (explosionCreated) {
                if (!fuse.runtime.preserveReferenceAfterDetonation) {
                    loose_grenade_runtime::disableAndDeleteReference(ref);
                }
                ROCK_LOG_INFO(Hand,
                    "Loose throwable activated: ref={:08X} explosion={:08X} mode={} reason={} consumed={} triggerActor={:08X} impactBody={} otherBody={}",
                    formID,
                    fuse.runtime.explosion ? fuse.runtime.explosion->GetFormID() : 0,
                    loose_grenade_runtime::detonationModeName(fuse.runtime.detonationMode),
                    reason ? reason : "unknown",
                    fuse.runtime.preserveReferenceAfterDetonation ? "no" : "yes",
                    triggerActorFormID,
                    fuse.impactBodyId,
                    pendingImpactOtherBodyId);
            } else {
                ROCK_LOG_WARN(Hand,
                    "Loose throwable activation failed; leaving ref loose: ref={:08X} explosion={:08X} mode={} reason={}",
                    formID,
                    fuse.runtime.explosion ? fuse.runtime.explosion->GetFormID() : 0,
                    loose_grenade_runtime::detonationModeName(fuse.runtime.detonationMode),
                    reason ? reason : "unknown");
            }
            _forceGrab.grenadeImpactBodyIds[slotIndex].store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
            fuse = {};
        };

        for (std::size_t slotIndex = 0; slotIndex < _forceGrab.grenadeFuses.size(); ++slotIndex) {
            auto& fuse = _forceGrab.grenadeFuses[slotIndex];
            if (!fuse.active) {
                continue;
            }

            auto refPtr = fuse.handle.get();
            auto* ref = refPtr.get();
            if (!ref || ref->IsDeleted() || ref->IsDisabled()) {
                ROCK_LOG_DEBUG(Hand,
                    "Loose throwable activation cleared because ref is gone: ref={:08X} remaining={:.3f}s",
                    fuse.refFormID,
                    fuse.remainingSeconds);
                _forceGrab.grenadeImpactBodyIds[slotIndex].store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
                fuse = {};
                continue;
            }

            if (fuse.runtime.detonationMode == loose_grenade_runtime::GrenadeDetonationMode::Impact) {
                if (!pendingImpact || pendingImpactBodyId != fuse.impactBodyId) {
                    continue;
                }
                if ((_rightHand.isHolding() && _rightHand.getHeldRef() == ref) ||
                    (_leftHand.isHolding() && _leftHand.getHeldRef() == ref)) {
                    ROCK_LOG_SAMPLE_DEBUG(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Loose impact throwable contact ignored while still held: ref={:08X} impactBody={} otherBody={}",
                        fuse.refFormID,
                        pendingImpactBodyId,
                        pendingImpactOtherBodyId);
                    continue;
                }

                auto* impactOtherRef = !isInvalidGrabBodyId(pendingImpactOtherBodyId) ?
                                           resolveBodyToRef(
                                               frame.bhkWorld,
                                               frame.hknpWorld,
                                               RE::hknpBodyId{ pendingImpactOtherBodyId }) :
                                           nullptr;
                auto* player = f4vr::getPlayer();
                if (impactOtherRef && (impactOtherRef == ref || impactOtherRef == player)) {
                    ROCK_LOG_SAMPLE_DEBUG(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Loose impact throwable ignored self/player contact: ref={:08X} impactBody={} otherBody={} otherRef={:08X}",
                        fuse.refFormID,
                        pendingImpactBodyId,
                        pendingImpactOtherBodyId,
                        impactOtherRef->GetFormID());
                    continue;
                }

                std::uint32_t impactActorFormID = 0;
                if (impactOtherRef && player &&
                    impactOtherRef->formType == RE::ENUM_FORM_ID::kACHR &&
                    std::isfinite(fuse.runtime.directImpactDamage) &&
                    fuse.runtime.directImpactDamage > 0.0f) {
                    auto* actor = static_cast<RE::Actor*>(impactOtherRef);
                    if (!actor->IsDead(false)) {
                        actor->HandleHealthDamage(player, fuse.runtime.directImpactDamage);
                        impactActorFormID = actor->GetFormID();
                        ROCK_LOG_INFO(Hand,
                            "Loose impact throwable applied direct authored weapon damage: ref={:08X} target={:08X} damage={:.1f}",
                            fuse.refFormID,
                            impactActorFormID,
                            fuse.runtime.directImpactDamage);
                    }
                }

                detonateLooseGrenade(
                    fuse,
                    slotIndex,
                    ref,
                    "impact",
                    impactActorFormID);
                continue;
            }

            if (fuse.runtime.detonationMode == loose_grenade_runtime::GrenadeDetonationMode::Proximity) {
                if ((_rightHand.isHolding() && _rightHand.getHeldRef() == ref) ||
                    (_leftHand.isHolding() && _leftHand.getHeldRef() == ref)) {
                    continue;
                }
                if (!fuse.releasedSinceArming) {
                    fuse.releasedSinceArming = true;
                    ROCK_LOG_INFO(Hand,
                        "Placed mine released; proximity arming delay started: ref={:08X} delay={:.3f}s radius={:.1f}gu",
                        fuse.refFormID,
                        fuse.remainingSeconds,
                        fuse.runtime.proximityRadiusGameUnits);
                    continue;
                }
                if (fuse.remainingSeconds > 0.0f) {
                    fuse.remainingSeconds -= deltaSeconds;
                    if (fuse.remainingSeconds > 0.0f) {
                        continue;
                    }
                    fuse.remainingSeconds = 0.0f;
                }

                const auto proximity = loose_grenade_runtime::scanHostileActorsWithinProximity(
                    ref,
                    fuse.runtime.proximityRadiusGameUnits);
                if (proximity.truncated) {
                    ROCK_LOG_SAMPLE_WARN(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Placed mine proximity scan reached its actor-handle bound: ref={:08X} scanned={}",
                        fuse.refFormID,
                        proximity.actorHandlesScanned);
                }
                if (proximity.targetFound) {
                    detonateLooseGrenade(
                        fuse,
                        slotIndex,
                        ref,
                        "proximity",
                        proximity.targetActorFormID);
                    continue;
                }

                fuse.remainingSeconds = kPlacedMineProximityPollSeconds;
                continue;
            }

            if (fuse.runtime.detonationMode != loose_grenade_runtime::GrenadeDetonationMode::TimedFuse) {
                ROCK_LOG_WARN(Hand,
                    "Cleared loose throwable with unsupported live mode: ref={:08X} mode={}",
                    fuse.refFormID,
                    loose_grenade_runtime::detonationModeName(fuse.runtime.detonationMode));
                _forceGrab.grenadeImpactBodyIds[slotIndex].store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
                fuse = {};
                continue;
            }

            fuse.remainingSeconds -= deltaSeconds;
            if (fuse.remainingSeconds > 0.0f) {
                continue;
            }

            detonateLooseGrenade(fuse, slotIndex, ref, "timed-fuse");
        }
    }
}
