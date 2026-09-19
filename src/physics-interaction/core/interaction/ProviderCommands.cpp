#include "physics-interaction/core/PhysicsInteractionInternal.h"

// Provider interaction commands and provider weapon-part drives.

namespace rock
{
    void PhysicsInteraction::processProviderInteractionCommands(const PhysicsFrameContext& frame)
    {
        using namespace provider;

        /*
         * Unregister/provider-loss clears API reservations immediately. Prune
         * their deferred runtime slots before admitting a replacement command
         * so a cancelled owner cannot cause a one-frame false HandBusy.
         */
        pruneInactiveProviderForceGrabCommits();

        QueuedInteractionCommandV1 command{};
        std::uint32_t processed = 0;
        while (processed++ < ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1 && provider::dequeueInteractionCommandV1(command)) {
            pruneInactiveProviderForceGrabCommits();
            if (command.kind == RockProviderInteractionCommandKindV1::ForceGrab &&
                !provider::isInteractionCommandActiveV1(command.ownerToken, command.commandId)) {
                continue;
            }
            auto requestHand = [&]() {
                switch (command.kind) {
                case RockProviderInteractionCommandKindV1::ForceGrab:
                    return command.forceGrab.hand;
                case RockProviderInteractionCommandKindV1::ForceRelease:
                    return command.forceRelease.hand;
                case RockProviderInteractionCommandKindV1::ThrownDrop:
                    return command.thrownDrop.hand;
                default:
                    return RockProviderHand::None;
                }
            }();
            const auto requestTargetFormId = [&]() -> std::uint32_t {
                switch (command.kind) {
                case RockProviderInteractionCommandKindV1::ForceGrab:
                    return command.forceGrab.targetFormId;
                case RockProviderInteractionCommandKindV1::ForceRelease:
                    return command.forceRelease.targetFormId;
                case RockProviderInteractionCommandKindV1::ThrownDrop:
                    return command.thrownDrop.targetFormId;
                default:
                    return 0;
                }
            }();
            const auto requestTargetBodyId = [&]() -> std::uint32_t {
                switch (command.kind) {
                case RockProviderInteractionCommandKindV1::ForceGrab:
                    return command.forceGrab.targetBodyId;
                case RockProviderInteractionCommandKindV1::ForceRelease:
                    return command.forceRelease.targetBodyId;
                case RockProviderInteractionCommandKindV1::ThrownDrop:
                    return command.thrownDrop.targetBodyId;
                default:
                    return INVALID_BODY_ID;
                }
            }();
            const auto requestWorldGeneration = [&]() -> std::uint32_t {
                switch (command.kind) {
                case RockProviderInteractionCommandKindV1::ForceGrab:
                    return command.forceGrab.worldGeneration;
                case RockProviderInteractionCommandKindV1::ForceRelease:
                    return command.forceRelease.worldGeneration;
                case RockProviderInteractionCommandKindV1::ThrownDrop:
                    return command.thrownDrop.worldGeneration;
                default:
                    return 0;
                }
            }();
            const auto requestSkeletonGeneration = [&]() -> std::uint32_t {
                switch (command.kind) {
                case RockProviderInteractionCommandKindV1::ForceGrab:
                    return command.forceGrab.skeletonGeneration;
                case RockProviderInteractionCommandKindV1::ForceRelease:
                    return command.forceRelease.skeletonGeneration;
                case RockProviderInteractionCommandKindV1::ThrownDrop:
                    return command.thrownDrop.skeletonGeneration;
                default:
                    return 0;
                }
            }();
            const auto requestProviderGeneration = [&]() -> std::uint32_t {
                switch (command.kind) {
                case RockProviderInteractionCommandKindV1::ForceGrab:
                    return command.forceGrab.providerGeneration;
                case RockProviderInteractionCommandKindV1::ForceRelease:
                    return command.forceRelease.providerGeneration;
                case RockProviderInteractionCommandKindV1::ThrownDrop:
                    return command.thrownDrop.providerGeneration;
                default:
                    return 0;
                }
            }();

            RockProviderInteractionCommandResultV1 result{};
            result.size = sizeof(RockProviderInteractionCommandResultV1);
            result.version = ROCK_PROVIDER_API_VERSION;
            result.ownerToken = command.ownerToken;
            result.commandId = command.commandId;
            result.kind = command.kind;
            result.state = RockProviderInteractionCommandStateV1::Rejected;
            result.failure = RockProviderInteractionFailureV1::InvalidRequest;
            result.hand = requestHand;
            result.targetFormId = requestTargetFormId;
            result.targetBodyId = requestTargetBodyId;
            result.frameIndex = _frame.palmClockGameFrameIndex.load(std::memory_order_acquire);
            result.worldGeneration = _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire);
            result.skeletonGeneration = _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire);
            result.providerGeneration = _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire);

            auto complete = [&](RockProviderInteractionCommandStateV1 state, RockProviderInteractionFailureV1 failure) {
                result.state = state;
                result.failure = failure;
                return provider::completeInteractionCommandV1(result);
            };

            const bool isForceGrabCommand = command.kind == RockProviderInteractionCommandKindV1::ForceGrab;
            const bool isForceReleaseCommand = command.kind == RockProviderInteractionCommandKindV1::ForceRelease;
            const bool isThrownDropCommand = command.kind == RockProviderInteractionCommandKindV1::ThrownDrop;
            if (!isForceGrabCommand && !isForceReleaseCommand && !isThrownDropCommand) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::InvalidRequest);
                continue;
            }

            if (!frame.worldReady || !frame.bhkWorld || !frame.hknpWorld) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::ProviderNotReady);
                continue;
            }
            if (!physicsWritesAllowedForWorld(frame.hknpWorld)) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::PhysicsWritesBlocked);
                continue;
            }
            if (requestWorldGeneration != 0 && requestWorldGeneration != result.worldGeneration) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::StaleWorldGeneration);
                continue;
            }
            if (requestSkeletonGeneration != 0 && requestSkeletonGeneration != result.skeletonGeneration) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::StaleSkeletonGeneration);
                continue;
            }
            if (requestProviderGeneration != 0 && requestProviderGeneration != result.providerGeneration) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::StaleProviderGeneration);
                continue;
            }

            const bool inventoryTransfer = isForceGrabCommand &&
                (command.forceGrab.flags & static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::FromPlayerInventory)) != 0;
            if (inventoryTransfer) {
                const bool rightFree = forceGrabHandBlockerMask(_rightHand, false, frame.right.disabled, true) == 0;
                const bool leftFree = forceGrabHandBlockerMask(_leftHand, true, frame.left.disabled, true) == 0;
                const auto choice = force_grab_policy::selectGrenadeHand(false, rightFree, leftFree);
                if (!rightFree && !leftFree) {
                    f4vr::showNotification("ROCK: Cannot take item - neither hand is free.");
                    complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandBusy);
                    continue;
                }
                requestHand = choice.hand == force_grab_policy::HandChoice::Right ? RockProviderHand::Right : RockProviderHand::Left;
                result.hand = requestHand;
            }
            const bool isLeft = requestHand == RockProviderHand::Left;
            if (requestHand != RockProviderHand::Left && requestHand != RockProviderHand::Right) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandInvalid);
                continue;
            }
            Hand& hand = isLeft ? _leftHand : _rightHand;
            const auto& handInput = isLeft ? frame.left : frame.right;

            auto hasTargetIdentity = [&]() {
                return requestTargetFormId != 0 || requestTargetBodyId != INVALID_BODY_ID;
            };
            auto heldObjectMatchesRequest = [&](std::uint32_t heldFormId, std::uint32_t primaryBodyId) {
                if (!hasTargetIdentity()) {
                    return true;
                }
                if (requestTargetFormId != 0 && heldFormId != requestTargetFormId) {
                    return false;
                }
                if (requestTargetBodyId != INVALID_BODY_ID && primaryBodyId != requestTargetBodyId && !hand.isHeldBodyId(requestTargetBodyId)) {
                    return false;
                }
                return true;
            };
            auto clearProviderReleaseInputState = [&]() {
                _forceGrab.retainedWeaponGrabs[isLeft ? 1u : 0u] = {};
                grab_input_intent_policy::reset(_grabInput.intentStates[isLeft ? 1u : 0u]);
                peer_held_join_retry_policy::reset(_grabInput.peerHeldJoinRetryStates[isLeft ? 1u : 0u]);
                shoulder_stash::resetRuntime(_grabInput.shoulderStashStates[isLeft ? 1u : 0u]);
                mouth_consume::resetRuntime(_grabInput.mouthConsumeStates[isLeft ? 1u : 0u]);
                hand.cancelStashCandidate();
                hand.cancelConsumeCandidate();
                input_remap_runtime::setHandHeldWeapon(isLeft, hand.isHoldingLooseWeapon());
            };

            if (isForceReleaseCommand || isThrownDropCommand) {
                TouchGrabRuntime::HandReport surface{};
                if (isForceReleaseCommand && _touchGrabRuntime.getHandReport(isLeft, surface) && surface.globalSurface) {
                    if (!heldObjectMatchesRequest(surface.referenceFormId, surface.bodyId)) {
                        complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HeldObjectMismatch);
                        continue;
                    }
                    if ((command.forceRelease.flags & static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::UseVelocityHavok)) != 0) {
                        complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::InvalidRequest);
                        continue;
                    }
                    result.targetFormId = surface.referenceFormId;
                    result.targetBodyId = surface.bodyId;
                    _touchGrabRuntime.releaseHand(isLeft, frame.bhkWorld, frame.hknpWorld,
                        RockProviderTouchGrabReleaseReasonV1::HandUnavailable,
                        _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire));
                    clearProviderReleaseInputState();
                    complete(RockProviderInteractionCommandStateV1::Succeeded, RockProviderInteractionFailureV1::None);
                    continue;
                }
                if (!hand.isHolding()) {
                    complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandNotHolding);
                    continue;
                }

                auto* heldRef = hand.getHeldRef();
                const std::uint32_t heldFormId = heldRef ? heldRef->GetFormID() : 0u;
                const std::uint32_t primaryBodyId = hand.getSavedObjectState().bodyId.value;
                result.targetFormId = heldFormId;
                result.targetBodyId = primaryBodyId;
                if (!heldObjectMatchesRequest(heldFormId, primaryBodyId)) {
                    complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HeldObjectMismatch);
                    continue;
                }

                const Hand& peer = isLeft ? _rightHand : _leftHand;
                if (isThrownDropCommand && heldRef && peer.isHolding() && peer.getHeldRef() == heldRef) {
                    complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandBusy);
                    continue;
                }

                GrabReleaseOutcome::VelocitySnapshot requestedVelocity{};
                const bool forceReleaseUsesVelocity = isForceReleaseCommand &&
                    (command.forceRelease.flags & static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::UseVelocityHavok)) != 0;
                const bool thrownDropUsesVelocity = isThrownDropCommand &&
                    (command.thrownDrop.flags & static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::UseVelocityHavok)) != 0;
                const bool applyRequestedVelocity = forceReleaseUsesVelocity || thrownDropUsesVelocity;
                if (applyRequestedVelocity) {
                    const float* linearVelocityHavok = forceReleaseUsesVelocity ?
                        command.forceRelease.linearVelocityHavok :
                        command.thrownDrop.linearVelocityHavok;
                    const float* angularVelocityRadiansPerSecond = forceReleaseUsesVelocity ?
                        command.forceRelease.angularVelocityRadiansPerSecond :
                        command.thrownDrop.angularVelocityRadiansPerSecond;

                    requestedVelocity.available = true;
                    requestedVelocity.primaryBodyId = RE::hknpBodyId{ primaryBodyId };
                    requestedVelocity.linearVelocityHavok = RE::NiPoint3{
                        linearVelocityHavok[0],
                        linearVelocityHavok[1],
                        linearVelocityHavok[2],
                    };
                    requestedVelocity.angularVelocityRadiansPerSecond = RE::NiPoint3{
                        angularVelocityRadiansPerSecond[0],
                        angularVelocityRadiansPerSecond[1],
                        angularVelocityRadiansPerSecond[2],
                    };
                    requestedVelocity.overrideAngularVelocity = true;
                    for (const auto bodyId : hand.getHeldBodyIds()) {
                        if (requestedVelocity.bodyCount >= requestedVelocity.bodyIds.size()) {
                            break;
                        }
                        requestedVelocity.bodyIds[requestedVelocity.bodyCount++] = bodyId;
                    }
                }

                if (isThrownDropCommand && !applyRequestedVelocity) {
                    hand.captureHeldReleaseMotion(frame.hknpWorld, handInput.rawHandWorld, frame.deltaSeconds);
                }

                const std::uint32_t flags = isThrownDropCommand ? command.thrownDrop.flags : command.forceRelease.flags;
                const bool immediateCollisionRestore = isThrownDropCommand ?
                    (flags & static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::ImmediateCollisionRestore)) != 0 :
                    (flags & static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::ImmediateCollisionRestore)) != 0;
                auto releaseContext = makeGrabReleaseContext(hand, isLeft);
                releaseContext.disposition = GrabReleaseDisposition::PhysicalDrop;
                releaseContext.applyCapturedReleaseVelocity = isThrownDropCommand && !applyRequestedVelocity;
                releaseContext.reason = isThrownDropCommand ? "provider-thrown-drop" : "provider-force-release";
                const auto releaseOutcome = hand.releaseGrabbedObject(frame.hknpWorld,
                    immediateCollisionRestore ? GrabReleaseCollisionRestoreMode::Immediate : GrabReleaseCollisionRestoreMode::Delayed,
                    releaseContext);
                if (!releaseOutcome.released) {
                    complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandNotHolding);
                    continue;
                }

                if (heldRef) {
                    releaseObject(heldRef, claimOwnerForHand(isLeft));
                }
                if (applyRequestedVelocity && releaseContext.finalObjectRelease) {
                    hand.applyReleaseVelocitySnapshot(frame.hknpWorld, requestedVelocity);
                }
                dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormId, 0);
                dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, heldRef);
                clearProviderReleaseInputState();
                complete(RockProviderInteractionCommandStateV1::Succeeded, RockProviderInteractionFailureV1::None);
                continue;
            }

            if (handInput.disabled) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandDisabled);
                continue;
            }
            if (forceGrabHandBlockerMask(hand, isLeft, false, true) != 0) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandBusy);
                continue;
            }

            if (command.forceGrab.targetFormId == 0) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::InvalidRequest);
                continue;
            }

            if (command.powerArmorPoint != RockProviderPowerArmorPointV1::None) {
                RockProviderReferenceQueryV1 query{};
                query.referenceFormId = command.forceGrab.targetFormId;
                query.referenceNativeHandle = command.powerArmorReferenceNativeHandle;
                RockProviderPowerArmorTargetV1 target{};
                RE::NiTransform presented{};
                if (frame.menuBlocked || !reference_interaction::describePowerArmor(reference_interaction::resolveQuery(query), target) ||
                    target.frameReference.referenceFormId == 0 || !_dynamicHandCollision.getLastPresentedHandWorld(isLeft, presented)) {
                    complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::TargetUnavailable);
                    continue;
                }
                const auto candidate = _touchGrabRuntime.findPowerArmorCandidate(frame.hknpWorld, presented.translate,
                    target.frameReference.referenceFormId, command.powerArmorPoint,
                    command.forceGrab.maxDistanceGame > 0 ? command.forceGrab.maxDistanceGame : TouchGrabRuntime::kPowerArmorProximityRadiusGame);
                if (!candidate.valid || !_touchGrabRuntime.tryAcquirePowerArmor(isLeft, candidate, frame.bhkWorld,
                    frame.hknpWorld, result.worldGeneration, result.skeletonGeneration, result.providerGeneration,
                    _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire), command.ownerToken)) {
                    complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::TargetUnavailable);
                    continue;
                }
                result.targetFormId = candidate.referenceFormId;
                result.targetBodyId = candidate.bodyId;
                if (hand.hasSelection()) hand.clearSelectionState(false);
                clearProviderReleaseInputState();
                if (!complete(RockProviderInteractionCommandStateV1::Succeeded, RockProviderInteractionFailureV1::None)) {
                    _touchGrabRuntime.releaseHand(isLeft, frame.bhkWorld, frame.hknpWorld,
                        RockProviderTouchGrabReleaseReasonV1::OwnerYield,
                        _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire));
                }
                continue;
            }

            if (inventoryTransfer) {
                auto* weapon = RE::TESForm::GetFormByID<RE::TESObjectWEAP>(command.forceGrab.targetFormId);
                const bool throwable = weapon && loose_grenade_runtime::isThrowableWeapon(weapon);
                if (throwable && (handHoldsLooseGrenade(_rightHand) || handHoldsLooseGrenade(_leftHand) || hasActiveLooseGrenadeCommit())) {
                    f4vr::showNotification("ROCK: A throwable is already held or attaching.");
                    complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::TargetAlreadyOwned);
                    continue;
                }
                RE::NiPoint3 dropLocation = handInput.grabAnchorWorld;
                dropLocation.z -= 3.0f;
                const auto drop = loose_grenade_runtime::dropInventoryItemToWorld(command.forceGrab.targetFormId, dropLocation);
                if (!drop.success) {
                    ROCK_LOG_WARN(Hand, "Inventory handoff rejected: item={:08X} reason={}", command.forceGrab.targetFormId, drop.reason);
                    f4vr::showNotification("ROCK: This inventory item could not be taken.");
                    complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::TargetUnavailable);
                    continue;
                }
                result.targetFormId = drop.droppedRef ? drop.droppedRef->GetFormID() : 0;
                auto& commit = _forceGrab.pendingCommits[isLeft ? 1u : 0u];
                commit = PendingForceGrabCommit{
                    .active = true,
                    .isLeft = isLeft,
                    .phase = PendingForceGrabCommitPhase::WaitingForReference,
                    .targetHandle = drop.handle,
                    .targetIsLooseThrowable = throwable,
                    .inventoryTransfer = true,
                    .maxDistanceGame = 96.0f,
                    .providerResultTemplate = result,
                };
                // Keep the pending transfer even if cancellation won the race:
                // the normal prune path owns rollback of this exact handle.
                complete(RockProviderInteractionCommandStateV1::Queued, RockProviderInteractionFailureV1::None);
                ROCK_LOG_INFO(Hand, "Inventory handoff queued: item={:08X} command={} hand={}", command.forceGrab.targetFormId, command.commandId, isLeft ? "left" : "right");
                continue;
            }

            auto* targetRef = RE::TESForm::GetFormByID<RE::TESObjectREFR>(command.forceGrab.targetFormId);
            if (!targetRef) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::TargetMissing);
                continue;
            }
            result.targetFormId = targetRef->GetFormID();
            if (targetRef->IsDeleted() || targetRef->IsDisabled()) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::TargetUnavailable);
                continue;
            }
            if (command.forceGrab.targetFormId != 0 && targetRef->GetFormID() != command.forceGrab.targetFormId) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::TargetUnavailable);
                continue;
            }
            if (physicsModOwnsObject(targetRef)) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::TargetAlreadyOwned);
                continue;
            }

            const bool targetIsLooseThrowable = loose_grenade_runtime::isThrowableRef(targetRef);
            if (targetIsLooseThrowable &&
                (handHoldsLooseGrenade(_rightHand) ||
                    handHoldsLooseGrenade(_leftHand) ||
                    hasActiveLooseGrenadeCommit())) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandBusy);
                continue;
            }

            auto& commit = _forceGrab.pendingCommits[isLeft ? 1u : 0u];
            if (commit.active) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandBusy);
                continue;
            }

            const bool hasSourcePointOverride =
                (command.forceGrab.flags & static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::UsePreferredGrabPointGame)) != 0;

            commit = PendingForceGrabCommit{
                .active = true,
                .isLeft = isLeft,
                .phase = PendingForceGrabCommitPhase::WaitingForSettle,
                .targetHandle = targetRef->GetHandle(),
                .targetIsLooseThrowable = targetIsLooseThrowable,
                .preferredBodyId = command.forceGrab.targetBodyId,
                .maxDistanceGame = command.forceGrab.maxDistanceGame,
                .hasSourcePointOverride = hasSourcePointOverride,
                .sourcePointOverride = hasSourcePointOverride ?
                    RE::NiPoint3{
                        command.forceGrab.preferredGrabPointGame[0],
                        command.forceGrab.preferredGrabPointGame[1],
                        command.forceGrab.preferredGrabPointGame[2],
                    } :
                    RE::NiPoint3{},
                .providerResultTemplate = result,
            };
            if (!complete(RockProviderInteractionCommandStateV1::Queued, RockProviderInteractionFailureV1::None)) {
                commit = {};
                continue;
            }
        }
    }

    std::size_t PhysicsInteraction::applyProviderWeaponPartDrives(
        RE::NiNode* weaponNode,
        std::uint64_t currentWeaponGenerationKey,
        const PhysicsFrameContext& frame,
        std::array<const RE::NiAVObject*, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1>& outDrivenSourceNodes)
    {
        outDrivenSourceNodes = {};
        _providerDrives.resultCount = 0;
        if (!weaponNode || currentWeaponGenerationKey == 0 || !frame.worldReady) {
            if (weaponNode && currentWeaponGenerationKey != 0) {
                restoreExpiredProviderWeaponPartDriveNodes(weaponNode, currentWeaponGenerationKey);
            }
            return 0;
        }

        if (_providerDrives.generationKey != 0 && _providerDrives.generationKey != currentWeaponGenerationKey) {
            _providerDrives.nodeStates = {};
            _providerDrives.generationKey = 0;
        }
        for (auto& state : _providerDrives.nodeStates) {
            state.activeThisFrame = false;
        }

        std::array<::rock::provider::RockProviderWeaponPartDriveTargetV1, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1> driveTargets{};
        std::array<std::uint64_t,
            ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1>
            driveOwners{};
        const std::uint32_t driveCount = ::rock::provider::copyWeaponPartDriveTargetsV1(
            driveTargets.data(),
            static_cast<std::uint32_t>(driveTargets.size()),
            driveOwners.data());
        if (driveCount == 0) {
            restoreExpiredProviderWeaponPartDriveNodes(weaponNode, currentWeaponGenerationKey);
            return 0;
        }

        struct AppliedNode
        {
            RE::NiAVObject* node{ nullptr };
            std::uint32_t priority{ 0 };
            std::uint32_t resultIndex{ 0 };
        };

        std::array<AppliedNode, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1> appliedNodes{};
        std::size_t appliedNodeCount = 0;
        const auto evidenceDescriptors = _weaponCollision.getProfileEvidenceDescriptors();

        auto resolveDriveNode = [&](const ::rock::provider::RockProviderWeaponPartDriveTargetV1& drive) -> RE::NiAVObject* {
            if (drive.weaponGenerationKey != 0 && drive.weaponGenerationKey != currentWeaponGenerationKey) {
                return nullptr;
            }

            RE::NiAVObject* candidate = nullptr;
            auto acceptResolvedNode = [&](RE::NiAVObject* node) {
                if (!node || !actor_equipment_grab::nodeContainsNode(weaponNode, node, 64)) {
                    return false;
                }
                if (candidate && candidate != node) {
                    return false;
                }
                candidate = node;
                return true;
            };

            if ((drive.flags & static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponPartTargetFlagV1::MatchSourceRoot)) != 0 && drive.sourceRoot != 0) {
                auto* node = reinterpret_cast<RE::NiAVObject*>(drive.sourceRoot);
                if (!acceptResolvedNode(node)) {
                    return nullptr;
                }
            }

            if ((drive.flags & static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponPartTargetFlagV1::MatchBodyId)) != 0 && drive.bodyId != INVALID_CONTACT_BODY_ID) {
                const auto* descriptor = evidenceDescriptors.find(drive.bodyId);
                if (descriptor && descriptor->sourceRootAddress != 0 &&
                    descriptor->weaponGenerationKey == currentWeaponGenerationKey &&
                    acceptResolvedNode(reinterpret_cast<RE::NiAVObject*>(descriptor->sourceRootAddress))) {
                } else {
                    return nullptr;
                }
            }

            const auto sourceName = providerFixedStringView(drive.sourceName, ::rock::provider::ROCK_PROVIDER_MAX_EVIDENCE_NAME);
            if ((drive.flags & static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponPartTargetFlagV1::MatchSourceName)) != 0 && !sourceName.empty()) {
                RE::NiAVObject* matchedSourceNode = nullptr;
                for (const auto& descriptor : evidenceDescriptors) {
                    if (!descriptor.valid || descriptor.weaponGenerationKey != currentWeaponGenerationKey || descriptor.sourceName != sourceName) {
                        continue;
                    }
                    matchedSourceNode = reinterpret_cast<RE::NiAVObject*>(descriptor.sourceRootAddress);
                    break;
                }
                if (!matchedSourceNode) {
                    matchedSourceNode = findWeaponNodeBySourceName(weaponNode, sourceName, 32);
                }
                if (!acceptResolvedNode(matchedSourceNode)) {
                    return nullptr;
                }
            }

            return candidate;
        };

        auto shouldApplyPriority = [&](
                                       RE::NiAVObject* node,
                                       std::uint32_t priority,
                                       const std::uint32_t resultIndex) {
            for (std::size_t i = 0; i < appliedNodeCount; ++i) {
                if (appliedNodes[i].node != node) {
                    continue;
                }
                if (priority < appliedNodes[i].priority) {
                    return false;
                }
                _providerDrives.results[
                    appliedNodes[i].resultIndex].result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::LostPriority;
                appliedNodes[i].priority = priority;
                appliedNodes[i].resultIndex = resultIndex;
                return true;
            }
            if (appliedNodeCount < appliedNodes.size()) {
                appliedNodes[appliedNodeCount++] = AppliedNode{
                    .node = node,
                    .priority = priority,
                    .resultIndex = resultIndex,
                };
                return true;
            }
            return false;
        };

        auto markDrivenNode = [&](
                                  RE::NiAVObject* node,
                                  const ::rock::provider::RockProviderWeaponPartDriveApplicationResultV1& result) {
            if (!node) {
                return false;
            }
            if (_providerDrives.generationKey == 0) {
                _providerDrives.generationKey = currentWeaponGenerationKey;
            }
            for (auto& state : _providerDrives.nodeStates) {
                if (state.node == node) {
                    state.activeThisFrame = true;
                    state.ownerToken = result.ownerToken;
                    state.bodyId = result.bodyId;
                    state.groupId = result.groupId;
                    state.priority = result.priority;
                    std::memcpy(
                        state.sourceName.data(),
                        result.sourceName,
                        state.sourceName.size());
                    return true;
                }
            }
            for (auto& state : _providerDrives.nodeStates) {
                if (!state.node) {
                    state.node = node;
                    state.baselineLocal = node->local;
                    state.ownerToken = result.ownerToken;
                    state.bodyId = result.bodyId;
                    state.groupId = result.groupId;
                    state.priority = result.priority;
                    std::memcpy(
                        state.sourceName.data(),
                        result.sourceName,
                        state.sourceName.size());
                    state.activeThisFrame = true;
                    return true;
                }
            }
            return false;
        };

        std::size_t drivenSourceNodeCount = 0;
        for (std::uint32_t i = 0; i < driveCount && i < driveTargets.size(); ++i) {
            const auto& drive = driveTargets[i];
            auto& applicationResult = _providerDrives.results[
                _providerDrives.resultCount++];
            applicationResult = {};
            applicationResult.frameIndex =
                _frame.palmClockGameFrameIndex.load(std::memory_order_acquire);
            applicationResult.ownerToken = driveOwners[i];
            applicationResult.weaponGenerationKey =
                currentWeaponGenerationKey;
            applicationResult.bodyId = drive.bodyId;
            applicationResult.groupId = drive.groupId;
            applicationResult.priority = drive.priority;
            std::memcpy(
                applicationResult.sourceName,
                drive.sourceName,
                sizeof(applicationResult.sourceName));
            applicationResult.sourceName[
                sizeof(applicationResult.sourceName) - 1] = '\0';
            if (drive.weaponGenerationKey != 0 &&
                drive.weaponGenerationKey != currentWeaponGenerationKey) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::StaleGeneration;
                continue;
            }
            auto* sourceNode = resolveDriveNode(drive);
            if (!sourceNode) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::Unresolved;
                continue;
            }
            if (!sourceNode->parent) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::MissingParent;
                continue;
            }
            const RE::NiTransform requestedLocal = providerTransformToNi(drive.targetTransform);
            if (!finiteNiTransform(requestedLocal)) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::InvalidTransform;
                continue;
            }

            RE::NiTransform desiredWorld{};
            switch (drive.driveSpace) {
            case ::rock::provider::RockProviderWeaponPartDriveSpaceV1::SourceParentLocal:
                desiredWorld = transform_math::composeTransforms(sourceNode->parent->world, requestedLocal);
                break;
            case ::rock::provider::RockProviderWeaponPartDriveSpaceV1::WeaponRootLocal:
            default:
                desiredWorld = transform_math::composeTransforms(weaponNode->world, requestedLocal);
                break;
            }
            if (!finiteNiTransform(desiredWorld)) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::InvalidTransform;
                continue;
            }

            const RE::NiTransform requestedNodeLocal = transform_math::composeTransforms(transform_math::invertTransform(sourceNode->parent->world), desiredWorld);
            if (!finiteNiTransform(requestedNodeLocal)) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::InvalidTransform;
                continue;
            }
            if (!shouldApplyPriority(
                    sourceNode,
                    drive.priority,
                    _providerDrives.resultCount - 1)) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::LostPriority;
                continue;
            }
            if (!markDrivenNode(sourceNode, applicationResult)) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::CapacityRejected;
                continue;
            }
            sourceNode->local = requestedNodeLocal;
            f4vr::updateTransformsDown(sourceNode, true);
            applicationResult.result =
                ::rock::provider::RockProviderWeaponPartDriveApplicationV1::Applied;
            fillProviderTransform(
                requestedNodeLocal,
                applicationResult.appliedSourceParentLocal);

            if (drivenSourceNodeCount < outDrivenSourceNodes.size()) {
                outDrivenSourceNodes[drivenSourceNodeCount++] = sourceNode;
            }
        }

        restoreExpiredProviderWeaponPartDriveNodes(weaponNode, currentWeaponGenerationKey);
        return drivenSourceNodeCount;
    }

    void PhysicsInteraction::restoreExpiredProviderWeaponPartDriveNodes(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey)
    {
        if (_providerDrives.generationKey == 0) {
            return;
        }
        if (!weaponNode || currentWeaponGenerationKey == 0 || _providerDrives.generationKey != currentWeaponGenerationKey) {
            _providerDrives.nodeStates = {};
            _providerDrives.generationKey = 0;
            return;
        }

        bool anyActive = false;
        for (auto& state : _providerDrives.nodeStates) {
            if (!state.node) {
                continue;
            }
            if (state.activeThisFrame) {
                anyActive = true;
                continue;
            }
            if (actor_equipment_grab::nodeContainsNode(weaponNode, state.node, 64)) {
                state.node->local = state.baselineLocal;
                f4vr::updateTransformsDown(state.node, true);
                if (_providerDrives.resultCount <
                    _providerDrives.results.size()) {
                    auto& result = _providerDrives.results[
                        _providerDrives.resultCount++];
                    result = {};
                    result.frameIndex =
                        _frame.palmClockGameFrameIndex.load(
                            std::memory_order_acquire);
                    result.ownerToken = state.ownerToken;
                    result.weaponGenerationKey =
                        currentWeaponGenerationKey;
                    result.bodyId = state.bodyId;
                    result.groupId = state.groupId;
                    result.priority = state.priority;
                    result.result =
                        ::rock::provider::RockProviderWeaponPartDriveApplicationV1::Restored;
                    fillProviderTransform(
                        state.baselineLocal,
                        result.appliedSourceParentLocal);
                    std::memcpy(
                        result.sourceName,
                        state.sourceName.data(),
                        sizeof(result.sourceName));
                    result.sourceName[sizeof(result.sourceName) - 1] = '\0';
                }
            }
            state = {};
        }
        if (!anyActive) {
            _providerDrives.generationKey = 0;
        }
    }
}
