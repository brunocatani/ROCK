/*
 * FORCE GRAB and LOOSE GRENADES: provider-driven grabs, and the grenade quick-draw
 * that reuses the same commit machinery.
 *
 * Read in order: the bare-fist guard and blocker mask decide whether a hand may
 * accept a force grab, servicePendingForceGrabCommits completes the ones that may,
 * and the loose-grenade block below drives arm/fuse state for a held grenade.
 *
 * makeGrabReleaseContext and makeGrabSharedObjectContext are declared here but
 * DEFINED in core/frame/PhysicsInteractionGrabInput.cpp. Grab input owns them.
 */

#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"

#include <algorithm>
#include <cmath>

#include "RockConfig.h"
#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/api/InteractionCommandQueue.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/object/CarInteractionPolicy.h"
#include "physics-interaction/weapon/BareFistGuardPolicy.h"
#include "physics-interaction/weapon/equip/HeldWeaponEquipStatePolicy.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock
{
    using namespace physics_interaction_detail;

    void PhysicsInteraction::prepareDynamicWorldCarCollisionForGrab(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        RE::TESObjectREFR* ref)
    {
        if (!ref) {
            return;
        }
        auto* baseObject = ref->GetObjectReference();
        const bool targetIsCar = fo4vr::isExplodableCar(baseObject);
        if (!targetIsCar) {
            return;
        }
        const auto decision = car_interaction_policy::evaluateGrab(car_interaction_policy::GrabPolicyInput{
            .targetIsCar = targetIsCar,
            .playerInPowerArmor = fo4vr::isInPowerArmor(),
        });
        if (decision.allowed) {
            _dynamicWorldCarCollision.restoreReference(bhkWorld, hknpWorld, ref, "grab-commit");
        }
    }


    void PhysicsInteraction::enforceNoBareFistState(bool forceRecheck)
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* legacyPlayer = f4vr::getPlayer();
        if (!player || !legacyPlayer) {
            _bareFistGuardState = {};
            return;
        }

        const bool weaponDrawn = f4vr::IsWeaponDrawn();
        const std::uint32_t equippedWeaponFormId = currentEquippedWeaponFormId();
        const bool actorUsingMelee = weaponDrawn && f4vr::CombatUtilities_IsActorUsingMelee(legacyPlayer);
        if (!bare_fist_guard_policy::shouldRefreshWitness(
                _bareFistGuardState,
                forceRecheck,
                weaponDrawn,
                equippedWeaponFormId,
                actorUsingMelee)) {
            return;
        }

        // Inventory stack scanning is transition-only; never run it every
        // frame while a legitimate melee weapon remains drawn.
        const bool realMeleeWeaponEquipped = actorUsingMelee && f4vr::isMeleeWeaponEquipped();

        _bareFistGuardState = bare_fist_guard_policy::RecheckState{
            .initialized = true,
            .weaponDrawn = weaponDrawn,
            .equippedWeaponFormId = equippedWeaponFormId,
            .actorUsingMelee = actorUsingMelee,
            .realMeleeWeaponEquipped = realMeleeWeaponEquipped,
        };
        if (!weaponDrawn) {
            return;
        }

        if (!bare_fist_guard_policy::shouldHolster(bare_fist_guard_policy::Witness{
                .rockEnabled = g_rockConfig.rockEnabled,
                .weaponDrawn = weaponDrawn,
                .actorUsingMelee = actorUsingMelee,
                .realMeleeWeaponEquipped = realMeleeWeaponEquipped,
            })) {
            return;
        }

        /*
         * FO4 represents an unarmed fallback as drawn melee even though no
         * inventory weapon owns the hand. ROCK has no manual unequip action,
         * so close that state centrally without filtering real hand-to-hand
         * weapons such as knuckles or power fists.
         */
        player->DrawWeaponMagicHands(false);
        _bareFistGuardState.weaponDrawn = false;
        ROCK_LOG_SAMPLE_INFO(Weapon,
            g_rockConfig.rockLogSampleMilliseconds,
            "Bare-fist draw state blocked: holstering unarmed fallback (equippedForm={:08X})",
            equippedWeaponFormId);
    }

    void PhysicsInteraction::clearLooseGrenadeImpactWatches()
    {
        for (auto& bodyId : _armedLooseGrenadeImpactBodyIds) {
            bodyId.store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
        }
        _pendingLooseGrenadeImpactPair.store(INVALID_HELD_IMPACT_PAIR, std::memory_order_release);
    }

    void PhysicsInteraction::clearPendingForceGrabCommitsForOrigin(PendingForceGrabCommitOrigin origin)
    {
        for (auto& commit : _pendingForceGrabCommits) {
            if (commit.active && commit.origin == origin) {
                if (origin == PendingForceGrabCommitOrigin::ProviderForceGrabCommand &&
                    provider::isInteractionCommandActiveV1(
                        commit.providerResultTemplate.ownerToken,
                        commit.providerResultTemplate.commandId)) {
                    commit.providerResultTemplate.state = provider::RockProviderInteractionCommandStateV1::Cancelled;
                    commit.providerResultTemplate.failure = provider::RockProviderInteractionFailureV1::ProviderNotReady;
                    provider::completeInteractionCommandV1(commit.providerResultTemplate);
                } else if (origin == PendingForceGrabCommitOrigin::LooseGrenadeQuickDraw) {
                    const auto targetRefPtr = commit.targetHandle.get();
                    auto* targetRef = targetRefPtr.get();
                    if (targetRef && !loose_grenade_runtime::returnDroppedReferenceToInventory(targetRef)) {
                        ROCK_LOG_WARN(Hand,
                            "Loose grenade shutdown cleanup left the physical drop in world: ref={:08X} request={}",
                            targetRef->GetFormID(),
                            commit.grenadeRequestId);
                    }
                }
                commit = {};
            }
        }
    }

    void PhysicsInteraction::clearLooseGrenadeRuntimeState()
    {
        clearPendingForceGrabCommitsForOrigin(PendingForceGrabCommitOrigin::LooseGrenadeQuickDraw);
        _armedLooseGrenadeFuses = {};
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
        // Equip identity remains authoritative while menus temporarily hide or
        // detach the weapon's 3D node. Native presentation state separates a
        // fully sheathed weapon from one that still reserves its firing hand.
        const bool equippedWeaponPresent = currentEquippedWeaponFormId() != 0;
        const std::uint32_t nativeWeaponState =
            f4vr::getNativeWeaponState(f4vr::getPlayer());
        const bool equippedWeaponReservesFiringHand =
            equippedWeaponPresent &&
            held_weapon_equip_state_policy::weaponStateReservesFiringHand(
                nativeWeaponState);
        const bool equippedWeaponOccupiesHand = force_grab_policy::equippedWeaponOccupiesHand(
            isLeft,
            equippedWeaponReservesFiringHand,
            _twoHandedGrip.isPartCarryActive(),
            _twoHandedGrip.isFiringHandLeft(),
            _twoHandedGrip.isHandPartGripping(isLeft));

        return force_grab_policy::blockerMask(force_grab_policy::HandAvailabilityInput{
            .disabled = handDisabled,
            .openInteractionState = openInteractionState,
            .holding = hand.isHolding(),
            .activePullCatch = hand.hasActivePullCatchIntent(),
            .actorEquipmentHandoff = hand.hasPendingActorEquipmentDropHandoff(),
            .pendingForceGrab = includePendingCommit && _pendingForceGrabCommits[isLeft ? 1u : 0u].active,
            .equippedWeaponOccupiesHand = equippedWeaponOccupiesHand,
            .touchGrabActive = _touchGrabRuntime.isHandActive(isLeft),
        });
    }

    bool PhysicsInteraction::canHandAcceptForceGrab(const Hand& hand, bool isLeft, bool handDisabled) const
    {
        return forceGrabHandBlockerMask(hand, isLeft, handDisabled, false) == 0;
    }

    bool PhysicsInteraction::handHoldsLooseGrenade(const Hand& hand) const
    {
        return hand.isHolding() && loose_grenade_runtime::isGrenadeRef(hand.getHeldRef());
    }

    bool PhysicsInteraction::hasActiveLooseGrenadeCommit() const
    {
        for (const auto& commit : _pendingForceGrabCommits) {
            if (commit.active && commit.targetIsLooseGrenade) {
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
        for (const auto& commit : _pendingForceGrabCommits) {
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
        for (auto& commit : _pendingForceGrabCommits) {
            if (commit.active &&
                commit.origin == PendingForceGrabCommitOrigin::ProviderForceGrabCommand &&
                !provider::isInteractionCommandActiveV1(
                    commit.providerResultTemplate.ownerToken,
                    commit.providerResultTemplate.commandId)) {
                commit = {};
            }
        }
    }

    void PhysicsInteraction::serviceLooseGrenadeQuickDraw(const PhysicsFrameContext& frame)
    {
        constexpr float kLooseGrenadeQuickDrawMaxDistanceGame = 96.0f;

        const auto buttonState = input_remap_runtime::consumeRawButtonState(
            false,
            input_remap_policy::kOpenVrGrenadeQuickDrawButtonId);
        if (!g_rockConfig.rockEnabled || !buttonState.available ||
            !buttonState.pressed ||
            input_remap_runtime::isConfiguratorChordInputReserved(false)) {
            return;
        }

        if (!frame.worldReady || !frame.bhkWorld || !frame.hknpWorld) {
            ROCK_LOG_WARN(Hand, "Ignored grenade quick draw because the physics world is unavailable");
            return;
        }

        pruneInactiveProviderForceGrabCommits();

        if (handHoldsLooseGrenade(_rightHand) || handHoldsLooseGrenade(_leftHand) ||
            hasActiveLooseGrenadeCommit()) {
            ROCK_LOG_INFO(Hand, "Ignored grenade quick draw because a loose grenade is already held or attaching");
            return;
        }

        loose_grenade_runtime::EquippedGrenadeSelection equippedSelection{};
        const auto equippedStatus =
            loose_grenade_runtime::resolveEquippedGrenadeSelection(equippedSelection);
        if (equippedStatus != loose_grenade_runtime::EquippedGrenadeSelectionStatus::Selected) {
            f4vr::showNotification(
                equippedStatus == loose_grenade_runtime::EquippedGrenadeSelectionStatus::NoneEquipped ?
                    "ROCK: No grenade is selected." :
                    "ROCK: The selected grenade cannot be drawn.");
            ROCK_LOG_WARN(Hand,
                "Grenade quick draw could not resolve one native equipped stack: status={}",
                loose_grenade_runtime::selectionStatusName(equippedStatus));
            return;
        }

        const std::uint32_t rightBlockers = forceGrabHandBlockerMask(_rightHand, false, frame.right.disabled, true);
        const std::uint32_t leftBlockers = forceGrabHandBlockerMask(_leftHand, true, frame.left.disabled, true);
        const auto handSelection = force_grab_policy::selectGrenadeHand(
            false,
            rightBlockers == 0,
            leftBlockers == 0);
        if (handSelection.failure == force_grab_policy::GrenadeSelectionFailure::HandsBlocked) {
            f4vr::showNotification("ROCK: Cannot draw grenade - both hands are occupied.");
            ROCK_LOG_WARN(Hand,
                "Blocked grenade quick draw before inventory removal: request={} rightBlockers=0x{:02X} leftBlockers=0x{:02X}",
                equippedSelection.requestId,
                rightBlockers,
                leftBlockers);
            return;
        }

        const bool isLeft = handSelection.hand == force_grab_policy::HandChoice::Left;
        auto& commit = _pendingForceGrabCommits[isLeft ? 1u : 0u];
        const auto& handInput = isLeft ? frame.left : frame.right;

        /*
         * Spawn pose is irrelevant: the force-grab commit snaps the
         * grenade to a canonical attach pose, so the drop only needs a
         * location with enough clearance that the spawned body does not
         * start intersecting the hand collider and get ejected before
         * the grab commits.
         */
        constexpr float kLooseGrenadeSpawnHandClearanceGameUnits = 3.0f;
        RE::NiPoint3 dropLocation = handInput.grabAnchorWorld;
        dropLocation.z -= kLooseGrenadeSpawnHandClearanceGameUnits;

        const auto dropResult = loose_grenade_runtime::dropEquippedGrenadeSelectionToWorld(
            equippedSelection,
            dropLocation);
        if (!dropResult.success) {
            f4vr::showNotification("ROCK: The selected grenade could not be drawn.");
            ROCK_LOG_WARN(Hand,
                "Grenade quick-draw inventory drop failed: weapon={:08X} request={} stack={} reason={}",
                equippedSelection.weapon ? equippedSelection.weapon->GetFormID() : 0,
                equippedSelection.requestId,
                equippedSelection.stackId,
                dropResult.reason ? dropResult.reason : "unknown");
            return;
        }

        commit = PendingForceGrabCommit{
            .active = true,
            .isLeft = isLeft,
            .origin = PendingForceGrabCommitOrigin::LooseGrenadeQuickDraw,
            .phase = PendingForceGrabCommitPhase::WaitingForReference,
            .targetHandle = dropResult.handle,
            .targetIsLooseGrenade = true,
            .preferredBodyId = INVALID_BODY_ID,
            .maxDistanceGame = kLooseGrenadeQuickDrawMaxDistanceGame,
            .grenadeRequestId = equippedSelection.requestId,
            .grenadeRuntime = equippedSelection.runtime,
        };
        ROCK_LOG_INFO(Hand,
            "Grenade quick draw created ref={:08X} weapon={:08X} stack={} request={} hand={}",
            dropResult.droppedRef ? dropResult.droppedRef->GetFormID() : 0,
            equippedSelection.weapon ? equippedSelection.weapon->GetFormID() : 0,
            dropResult.stackId,
            equippedSelection.requestId,
            isLeft ? "left" : "right");
    }

    void PhysicsInteraction::servicePendingForceGrabCommits(const PhysicsFrameContext& frame)
    {
        if (!frame.worldReady || !frame.bhkWorld || !frame.hknpWorld) {
            return;
        }

        for (auto& commit : _pendingForceGrabCommits) {
            if (!commit.active) {
                continue;
            }

            Hand& hand = commit.isLeft ? _leftHand : _rightHand;
            const auto& handInput = commit.isLeft ? frame.left : frame.right;

            auto abandon = [&](const char* reason,
                               provider::RockProviderInteractionFailureV1 providerFailure,
                               RE::TESObjectREFR* targetRef) {
                if (commit.origin == PendingForceGrabCommitOrigin::ProviderForceGrabCommand) {
                    commit.providerResultTemplate.state = provider::RockProviderInteractionCommandStateV1::Rejected;
                    commit.providerResultTemplate.failure = providerFailure;
                    provider::completeInteractionCommandV1(commit.providerResultTemplate);
                } else {
                    const bool returnedToInventory = targetRef && loose_grenade_runtime::returnDroppedReferenceToInventory(targetRef);
                    if (targetRef) {
                        f4vr::showNotification(returnedToInventory ?
                                "ROCK: Grenade attach failed; returned to inventory." :
                                "ROCK: Grenade attach failed; it remains at your hand.");
                    }
                    ROCK_LOG_WARN(Hand,
                        "Loose grenade force-grab cleanup: ref={:08X} request={} returnedToInventory={}",
                        targetRef ? targetRef->GetFormID() : 0,
                        commit.grenadeRequestId,
                        returnedToInventory ? "yes" : "no");
                }
                ROCK_LOG_WARN(Hand,
                    "Pending force-grab commit abandoned ({}): hand={} origin={}",
                    reason,
                    commit.isLeft ? "left" : "right",
                    static_cast<int>(commit.origin));
                commit = {};
            };

            if (commit.origin == PendingForceGrabCommitOrigin::ProviderForceGrabCommand &&
                !provider::isInteractionCommandActiveV1(
                    commit.providerResultTemplate.ownerToken,
                    commit.providerResultTemplate.commandId)) {
                ROCK_LOG_INFO(Hand,
                    "Pending provider force-grab commit cancelled because its command is no longer active: command={} hand={}",
                    commit.providerResultTemplate.commandId,
                    commit.isLeft ? "left" : "right");
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

            if (commit.phase == PendingForceGrabCommitPhase::WaitingForReference) {
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
                    commit.maxDistanceGame)) {
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
                if (commit.origin == PendingForceGrabCommitOrigin::ProviderForceGrabCommand) {
                    commit.providerResultTemplate.targetBodyId = resolvedBodyId;
                }
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
                sharedContext);
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
            if (commit.origin == PendingForceGrabCommitOrigin::ProviderForceGrabCommand) {
                commit.providerResultTemplate.targetBodyId = primaryBodyId;
                commit.providerResultTemplate.state = provider::RockProviderInteractionCommandStateV1::Succeeded;
                commit.providerResultTemplate.failure = provider::RockProviderInteractionFailureV1::None;
                if (!provider::completeInteractionCommandV1(commit.providerResultTemplate)) {
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
                    commit = {};
                    continue;
                }
            }

            claimObject(heldRef, claimOwnerForHand(commit.isLeft));
            dispatchPhysicsMessage(kPhysMsg_OnGrab, commit.isLeft, heldRef, heldRef ? heldRef->GetFormID() : 0, 0);
            dispatchGrabCommittedEvent(commit.isLeft, heldRef, primaryBodyId, frame.hknpWorld);
            input_remap_runtime::setHandHeldWeapon(commit.isLeft, (commit.isLeft ? _leftHand : _rightHand).isHoldingLooseWeapon());

            if (commit.origin == PendingForceGrabCommitOrigin::LooseGrenadeQuickDraw) {
                ROCK_LOG_INFO(Hand,
                    "Grenade quick draw force-grabbed: ref={:08X} body={} request={}",
                    heldRef ? heldRef->GetFormID() : 0,
                    primaryBodyId,
                    commit.grenadeRequestId);
            }
            _forceGrabCommittedThisFrame[commit.isLeft ? 1u : 0u] = true;
            commit = {};
        }
    }


    bool PhysicsInteraction::armHeldLooseGrenade(Hand& hand, const PhysicsFrameContext& frame)
    {
        auto* heldRef = hand.getHeldRef();
        if (!heldRef || !loose_grenade_runtime::isGrenadeRef(heldRef)) {
            return false;
        }

        for (const auto& fuse : _armedLooseGrenadeFuses) {
            const auto fuseRefPtr = fuse.handle.get();
            if (fuse.active && fuseRefPtr.get() == heldRef) {
                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} hand loose grenade trigger ignored because fuse is already active: ref={:08X} remaining={:.3f}s",
                    hand.handName(),
                    heldRef->GetFormID(),
                    fuse.remainingSeconds);
                return true;
            }
        }

        loose_grenade_runtime::GrenadeRuntimeData runtime{};
        if (!loose_grenade_runtime::resolveGrenadeRuntimeDataForReference(heldRef, runtime)) {
            ROCK_LOG_WARN(Hand,
                "{} hand loose grenade trigger could not arm because projectile/explosion/fuse data was missing: ref={:08X}",
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
                    "{} hand loose Molotov trigger could not arm impact detonation because no held body id was available: ref={:08X}",
                    hand.handName(),
                    heldRef->GetFormID());
                return true;
            }
        }

        for (std::size_t slotIndex = 0; slotIndex < _armedLooseGrenadeFuses.size(); ++slotIndex) {
            auto& fuse = _armedLooseGrenadeFuses[slotIndex];
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
            };
            _armedLooseGrenadeImpactBodyIds[slotIndex].store(
                runtime.detonationMode == loose_grenade_runtime::GrenadeDetonationMode::Impact ? impactBodyId : INVALID_CONTACT_BODY_ID,
                std::memory_order_release);
            ROCK_LOG_INFO(Hand,
                "{} hand armed loose grenade: ref={:08X} projectile={:08X} explosion={:08X} mode={} fuse={:.3f}s impactBody={} frameDt={:.4f}",
                hand.handName(),
                heldRef->GetFormID(),
                runtime.projectile ? runtime.projectile->GetFormID() : 0,
                runtime.explosion ? runtime.explosion->GetFormID() : 0,
                loose_grenade_runtime::detonationModeName(runtime.detonationMode),
                runtime.fuseSeconds,
                impactBodyId,
                frame.deltaSeconds);
            const bool feedbackPlayed = loose_grenade_runtime::playPinPulledFeedbackAtReference(heldRef);
            ROCK_LOG_DEBUG(Hand,
                "{} hand loose grenade pin-pull feedback: ref={:08X} played={}",
                hand.handName(),
                heldRef->GetFormID(),
                feedbackPlayed ? "yes" : "no");
            return true;
        }

        ROCK_LOG_WARN(Hand,
            "{} hand loose grenade trigger could not arm because armed grenade capacity is full: ref={:08X}",
            hand.handName(),
            heldRef->GetFormID());
        return true;
    }

    void PhysicsInteraction::updateLooseGrenadeFuses(const PhysicsFrameContext& frame)
    {
        const float deltaSeconds = (std::max)(0.0f, frame.deltaSeconds);
        if (deltaSeconds <= 0.0f) {
            return;
        }

        std::uint32_t pendingImpactBodyId = INVALID_CONTACT_BODY_ID;
        std::uint32_t pendingImpactOtherBodyId = INVALID_CONTACT_BODY_ID;
        const bool pendingImpact =
            unpackHeldImpactPair(
                _pendingLooseGrenadeImpactPair.exchange(INVALID_HELD_IMPACT_PAIR, std::memory_order_acq_rel),
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
                                        const char* reason) {
            const std::uint32_t formID = ref ? ref->GetFormID() : fuse.refFormID;
            releaseHandIfHolding(_rightHand, false, ref, formID);
            releaseHandIfHolding(_leftHand, true, ref, formID);

            const bool explosionCreated = loose_grenade_runtime::createExplosionAtReference(ref, fuse.runtime.explosion);
            if (explosionCreated) {
                loose_grenade_runtime::disableAndDeleteReference(ref);
                ROCK_LOG_INFO(Hand,
                    "Loose grenade detonated: ref={:08X} explosion={:08X} mode={} reason={} impactBody={} otherBody={}",
                    formID,
                    fuse.runtime.explosion ? fuse.runtime.explosion->GetFormID() : 0,
                    loose_grenade_runtime::detonationModeName(fuse.runtime.detonationMode),
                    reason ? reason : "unknown",
                    fuse.impactBodyId,
                    pendingImpactOtherBodyId);
            } else {
                ROCK_LOG_WARN(Hand,
                    "Loose grenade detonation failed; leaving ref loose: ref={:08X} explosion={:08X} mode={} reason={}",
                    formID,
                    fuse.runtime.explosion ? fuse.runtime.explosion->GetFormID() : 0,
                    loose_grenade_runtime::detonationModeName(fuse.runtime.detonationMode),
                    reason ? reason : "unknown");
            }
            _armedLooseGrenadeImpactBodyIds[slotIndex].store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
            fuse = {};
        };

        for (std::size_t slotIndex = 0; slotIndex < _armedLooseGrenadeFuses.size(); ++slotIndex) {
            auto& fuse = _armedLooseGrenadeFuses[slotIndex];
            if (!fuse.active) {
                continue;
            }

            auto refPtr = fuse.handle.get();
            auto* ref = refPtr.get();
            if (!ref || ref->IsDeleted() || ref->IsDisabled()) {
                ROCK_LOG_DEBUG(Hand,
                    "Loose grenade fuse cleared because ref is gone: ref={:08X} remaining={:.3f}s",
                    fuse.refFormID,
                    fuse.remainingSeconds);
                _armedLooseGrenadeImpactBodyIds[slotIndex].store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
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
                        "Loose Molotov impact ignored while still held: ref={:08X} impactBody={} otherBody={}",
                        fuse.refFormID,
                        pendingImpactBodyId,
                        pendingImpactOtherBodyId);
                    continue;
                }

                detonateLooseGrenade(fuse, slotIndex, ref, "impact");
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
