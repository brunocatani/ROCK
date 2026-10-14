#include "physics-interaction/native/SceneRelation.h"
#include "api/StorageService.h"
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
            if (command.kind == api::grab::InteractionCommandKind::ForceGrab &&
                !provider::isInteractionCommandActiveV1(command.ownerToken, command.commandId)) {
                continue;
            }
            auto requestHand = [&]() {
                switch (command.kind) {
                case api::grab::InteractionCommandKind::ForceGrab:
                    return command.forceGrab.hand;
                case api::grab::InteractionCommandKind::ForceRelease:
                    return command.forceRelease.hand;
                case api::grab::InteractionCommandKind::ThrownDrop:
                    return command.thrownDrop.hand;
                default:
                    return api::Hand::None;
                }
            }();
            const auto requestTargetFormId = [&]() -> std::uint32_t {
                switch (command.kind) {
                case api::grab::InteractionCommandKind::ForceGrab:
                    return command.forceGrab.targetFormId;
                case api::grab::InteractionCommandKind::ForceRelease:
                    return command.forceRelease.targetFormId;
                case api::grab::InteractionCommandKind::ThrownDrop:
                    return command.thrownDrop.targetFormId;
                default:
                    return 0;
                }
            }();
            const auto requestTargetBodyId = [&]() -> std::uint32_t {
                switch (command.kind) {
                case api::grab::InteractionCommandKind::ForceGrab:
                    return command.forceGrab.targetBodyId;
                case api::grab::InteractionCommandKind::ForceRelease:
                    return command.forceRelease.targetBodyId;
                case api::grab::InteractionCommandKind::ThrownDrop:
                    return command.thrownDrop.targetBodyId;
                default:
                    return INVALID_BODY_ID;
                }
            }();
            const auto requestWorldGeneration = [&]() -> std::uint32_t {
                switch (command.kind) {
                case api::grab::InteractionCommandKind::ForceGrab:
                    return command.forceGrab.worldGeneration;
                case api::grab::InteractionCommandKind::ForceRelease:
                    return command.forceRelease.worldGeneration;
                case api::grab::InteractionCommandKind::ThrownDrop:
                    return command.thrownDrop.worldGeneration;
                default:
                    return 0;
                }
            }();
            const auto requestSkeletonGeneration = [&]() -> std::uint32_t {
                switch (command.kind) {
                case api::grab::InteractionCommandKind::ForceGrab:
                    return command.forceGrab.skeletonGeneration;
                case api::grab::InteractionCommandKind::ForceRelease:
                    return command.forceRelease.skeletonGeneration;
                case api::grab::InteractionCommandKind::ThrownDrop:
                    return command.thrownDrop.skeletonGeneration;
                default:
                    return 0;
                }
            }();
            const auto requestProviderGeneration = [&]() -> std::uint32_t {
                switch (command.kind) {
                case api::grab::InteractionCommandKind::ForceGrab:
                    return command.forceGrab.providerGeneration;
                case api::grab::InteractionCommandKind::ForceRelease:
                    return command.forceRelease.providerGeneration;
                case api::grab::InteractionCommandKind::ThrownDrop:
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
            result.state = api::grab::InteractionCommandState::Rejected;
            result.failure = api::grab::InteractionFailure::InvalidRequest;
            result.hand = requestHand;
            result.targetFormId = requestTargetFormId;
            result.targetBodyId = requestTargetBodyId;
            result.frameIndex = _frame.palmClockGameFrameIndex.load(std::memory_order_acquire);
            result.worldGeneration = _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire);
            result.skeletonGeneration = _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire);
            result.providerGeneration = _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire);

            auto complete = [&](api::grab::InteractionCommandState state, api::grab::InteractionFailure failure) {
                result.state = state;
                result.failure = failure;
                return provider::completeInteractionCommandV1(result);
            };

            const bool isForceGrabCommand = command.kind == api::grab::InteractionCommandKind::ForceGrab;
            const bool isForceReleaseCommand = command.kind == api::grab::InteractionCommandKind::ForceRelease;
            const bool isThrownDropCommand = command.kind == api::grab::InteractionCommandKind::ThrownDrop;
            if (!isForceGrabCommand && !isForceReleaseCommand && !isThrownDropCommand) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::InvalidRequest);
                continue;
            }

            if (!frame.worldReady || !frame.bhkWorld || !frame.hknpWorld) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::ProviderNotReady);
                continue;
            }
            if (!physicsWritesAllowedForWorld(frame.hknpWorld)) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::PhysicsWritesBlocked);
                continue;
            }
            if (requestWorldGeneration != 0 && requestWorldGeneration != result.worldGeneration) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::StaleWorldGeneration);
                continue;
            }
            if (requestSkeletonGeneration != 0 && requestSkeletonGeneration != result.skeletonGeneration) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::StaleSkeletonGeneration);
                continue;
            }
            if (requestProviderGeneration != 0 && requestProviderGeneration != result.providerGeneration) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::StaleProviderGeneration);
                continue;
            }

            const bool inventoryTransfer = isForceGrabCommand &&
                (command.forceGrab.flags & static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::FromPlayerInventory)) != 0;
            if (isForceGrabCommand && !inventoryTransfer && !storage_service::allowsGrab(requestTargetFormId, command.ownerToken)) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::TargetUnavailable);
                continue;
            }
            const bool inventoryThrowable = inventoryTransfer && loose_grenade_runtime::isThrowableWeapon(
                RE::TESForm::GetFormByID<RE::TESObjectWEAP>(command.forceGrab.targetFormId));
            if (inventoryTransfer) {
                const bool rightFree = forceGrabHandBlockerMask(_rightHand, false, frame.right.disabled, true) == 0;
                const bool leftFree = forceGrabHandBlockerMask(_leftHand, true, frame.left.disabled, true) == 0;
                const auto choice = force_grab_policy::selectGrenadeHand(
                    false, rightFree, leftFree, inventoryThrowable && g_rockConfig.rockLeftHandedMode);
                if (!rightFree && !leftFree) {
                    f4vr::showNotification("ROCK: Cannot take item - neither hand is free.");
                    complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::HandBusy);
                    continue;
                }
                requestHand = choice.hand == force_grab_policy::HandChoice::Right ? api::Hand::Right : api::Hand::Left;
                result.hand = requestHand;
            }
            const bool isLeft = requestHand == api::Hand::Left;
            if (requestHand != api::Hand::Left && requestHand != api::Hand::Right) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::HandInvalid);
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
                        complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::HeldObjectMismatch);
                        continue;
                    }
                    if ((command.forceRelease.flags & static_cast<std::uint32_t>(api::grab::ForceReleaseFlag::UseVelocityHavok)) != 0) {
                        complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::InvalidRequest);
                        continue;
                    }
                    result.targetFormId = surface.referenceFormId;
                    result.targetBodyId = surface.bodyId;
                    _touchGrabRuntime.releaseHand(isLeft, frame.bhkWorld, frame.hknpWorld,
                        ::rock::api::touch::TouchGrabReleaseReason::HandUnavailable,
                        _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire));
                    clearProviderReleaseInputState();
                    complete(api::grab::InteractionCommandState::Succeeded, api::grab::InteractionFailure::None);
                    continue;
                }
                if (!hand.isHolding()) {
                    complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::HandNotHolding);
                    continue;
                }

                auto* heldRef = hand.getHeldRef();
                const std::uint32_t heldFormId = heldRef ? heldRef->GetFormID() : 0u;
                const std::uint32_t primaryBodyId = hand.getSavedObjectState().bodyId.value;
                result.targetFormId = heldFormId;
                result.targetBodyId = primaryBodyId;
                if (!heldObjectMatchesRequest(heldFormId, primaryBodyId)) {
                    complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::HeldObjectMismatch);
                    continue;
                }

                const Hand& peer = isLeft ? _rightHand : _leftHand;
                if (isThrownDropCommand && heldRef && peer.isHolding() && peer.getHeldRef() == heldRef) {
                    complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::HandBusy);
                    continue;
                }

                GrabReleaseOutcome::VelocitySnapshot requestedVelocity{};
                const bool forceReleaseUsesVelocity = isForceReleaseCommand &&
                    (command.forceRelease.flags & static_cast<std::uint32_t>(api::grab::ForceReleaseFlag::UseVelocityHavok)) != 0;
                const bool thrownDropUsesVelocity = isThrownDropCommand &&
                    (command.thrownDrop.flags & static_cast<std::uint32_t>(api::grab::ThrownDropFlag::UseVelocityHavok)) != 0;
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
                    hand.captureHeldReleaseMotion(frame.hknpWorld, handInput.rawHandWorld, frame.timing);
                }

                const std::uint32_t flags = isThrownDropCommand ? command.thrownDrop.flags : command.forceRelease.flags;
                const bool immediateCollisionRestore = isThrownDropCommand ?
                    (flags & static_cast<std::uint32_t>(api::grab::ThrownDropFlag::ImmediateCollisionRestore)) != 0 :
                    (flags & static_cast<std::uint32_t>(api::grab::ForceReleaseFlag::ImmediateCollisionRestore)) != 0;
                auto releaseContext = makeGrabReleaseContext(hand, isLeft);
                releaseContext.disposition = GrabReleaseDisposition::PhysicalDrop;
                releaseContext.applyCapturedReleaseVelocity = isThrownDropCommand && !applyRequestedVelocity;
                releaseContext.reason = isThrownDropCommand ? "provider-thrown-drop" : "provider-force-release";
                const auto releaseOutcome = hand.releaseGrabbedObject(frame.hknpWorld,
                    immediateCollisionRestore ? GrabReleaseCollisionRestoreMode::Immediate : GrabReleaseCollisionRestoreMode::Delayed,
                    releaseContext);
                if (!releaseOutcome.released) {
                    complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::HandNotHolding);
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
                complete(api::grab::InteractionCommandState::Succeeded, api::grab::InteractionFailure::None);
                continue;
            }

            if (handInput.disabled) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::HandDisabled);
                continue;
            }
            if (forceGrabHandBlockerMask(hand, isLeft, false, true) != 0) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::HandBusy);
                continue;
            }

            if (command.forceGrab.targetFormId == 0) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::InvalidRequest);
                continue;
            }

            if (command.powerArmorPoint != api::PowerArmorPoint::None) {
                RockProviderReferenceQueryV1 query{};
                query.referenceFormId = command.forceGrab.targetFormId;
                query.referenceNativeHandle = command.powerArmorReferenceNativeHandle;
                RockProviderPowerArmorTargetV1 target{};
                RE::NiTransform presented{};
                if (frame.menuBlocked || !reference_interaction::describePowerArmor(reference_interaction::resolveQuery(query), target) ||
                    target.frameReference.referenceFormId == 0 || !_dynamicHandCollision.getLastPresentedHandWorld(isLeft, presented)) {
                    complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::TargetUnavailable);
                    continue;
                }
                const auto candidate = _touchGrabRuntime.findPowerArmorCandidate(frame.hknpWorld, presented.translate,
                    target.frameReference.referenceFormId, command.powerArmorPoint,
                    command.forceGrab.maxDistanceGame > 0 ? command.forceGrab.maxDistanceGame : TouchGrabRuntime::kPowerArmorProximityRadiusGame);
                if (!candidate.valid || !_touchGrabRuntime.tryAcquirePowerArmor(isLeft, candidate, frame.bhkWorld,
                    frame.hknpWorld, result.worldGeneration, result.skeletonGeneration, result.providerGeneration,
                    _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire), command.ownerToken)) {
                    complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::TargetUnavailable);
                    continue;
                }
                result.targetFormId = candidate.referenceFormId;
                result.targetBodyId = candidate.bodyId;
                if (hand.hasSelection()) hand.clearSelectionState(false);
                clearProviderReleaseInputState();
                if (!complete(api::grab::InteractionCommandState::Succeeded, api::grab::InteractionFailure::None)) {
                    _touchGrabRuntime.releaseHand(isLeft, frame.bhkWorld, frame.hknpWorld,
                        ::rock::api::touch::TouchGrabReleaseReason::OwnerYield,
                        _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire));
                }
                continue;
            }

            if (inventoryTransfer) {
                if (inventoryThrowable && (handHoldsLooseGrenade(_rightHand) || handHoldsLooseGrenade(_leftHand) || hasActiveLooseGrenadeCommit())) {
                    f4vr::showNotification("ROCK: A throwable is already held or attaching.");
                    complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::TargetAlreadyOwned);
                    continue;
                }
                RE::NiPoint3 dropLocation = handInput.grabAnchorWorld;
                dropLocation.z -= 3.0f;
                const auto drop = loose_grenade_runtime::dropInventoryItemToWorld(command.forceGrab.targetFormId, dropLocation);
                if (!drop.success) {
                    ROCK_LOG_WARN(Hand, "Inventory handoff rejected: item={:08X} reason={}", command.forceGrab.targetFormId, drop.reason);
                    f4vr::showNotification("ROCK: This inventory item could not be taken.");
                    complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::TargetUnavailable);
                    continue;
                }
                result.targetFormId = drop.droppedRef ? drop.droppedRef->GetFormID() : 0;
                auto& commit = _forceGrab.pendingCommits[isLeft ? 1u : 0u];
                commit = PendingForceGrabCommit{
                    .active = true,
                    .isLeft = isLeft,
                    .phase = PendingForceGrabCommitPhase::WaitingForReference,
                    .targetHandle = drop.handle,
                    .targetIsLooseThrowable = inventoryThrowable,
                    .inventoryTransfer = true,
                    .maxDistanceGame = 96.0f,
                    .providerResultTemplate = result,
                };
                // Keep the pending transfer even if cancellation won the race:
                // the normal prune path owns rollback of this exact handle.
                complete(api::grab::InteractionCommandState::Queued, api::grab::InteractionFailure::None);
                ROCK_LOG_INFO(Hand, "Inventory handoff queued: item={:08X} command={} hand={}", command.forceGrab.targetFormId, command.commandId, isLeft ? "left" : "right");
                continue;
            }

            auto* targetRef = RE::TESForm::GetFormByID<RE::TESObjectREFR>(command.forceGrab.targetFormId);
            if (!targetRef) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::TargetMissing);
                continue;
            }
            result.targetFormId = targetRef->GetFormID();
            if (targetRef->IsDeleted() || targetRef->IsDisabled()) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::TargetUnavailable);
                continue;
            }
            if (command.forceGrab.targetFormId != 0 && targetRef->GetFormID() != command.forceGrab.targetFormId) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::TargetUnavailable);
                continue;
            }
            if (physicsModOwnsObject(targetRef)) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::TargetAlreadyOwned);
                continue;
            }

            const bool targetIsLooseThrowable = loose_grenade_runtime::isThrowableRef(targetRef);
            if (targetIsLooseThrowable &&
                (handHoldsLooseGrenade(_rightHand) ||
                    handHoldsLooseGrenade(_leftHand) ||
                    hasActiveLooseGrenadeCommit())) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::HandBusy);
                continue;
            }

            auto& commit = _forceGrab.pendingCommits[isLeft ? 1u : 0u];
            if (commit.active) {
                complete(api::grab::InteractionCommandState::Rejected, api::grab::InteractionFailure::HandBusy);
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
            if (!complete(api::grab::InteractionCommandState::Queued, api::grab::InteractionFailure::None)) {
                commit = {};
                continue;
            }
        }
    }

    RE::NiAVObject* PhysicsInteraction::resolveProviderWeaponPartDriveNode(
        RE::NiNode* weaponNode,std::uint64_t currentWeaponGenerationKey,
        const provider::PartDrive& drive,
        const WeaponEvidenceSnapshot& evidenceDescriptors) const
    {
        if (drive.weaponGenerationKey != 0 && drive.weaponGenerationKey != currentWeaponGenerationKey) {
            return nullptr;
        }

        RE::NiAVObject* candidate = nullptr;
        auto acceptResolvedNode = [&](RE::NiAVObject* node) {
            if (!node || !scene_relation::nodeContainsNode(weaponNode, node, 64)) {
                return false;
            }
            if (candidate && candidate != node) {
                return false;
            }
            candidate = node;
            return true;
        };

        if ((drive.flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSourceKey)) != 0 && drive.sourceRoot != 0) {
            auto* node = reinterpret_cast<RE::NiAVObject*>(drive.sourceRoot);
            if (!acceptResolvedNode(node)) {
                return nullptr;
            }
        }

        if ((drive.flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchBodyId)) != 0 && drive.bodyId != INVALID_CONTACT_BODY_ID) {
            const auto* descriptor = evidenceDescriptors.find(drive.bodyId);
            if (descriptor && descriptor->sourceRootAddress != 0 &&
                descriptor->weaponGenerationKey == currentWeaponGenerationKey &&
                acceptResolvedNode(reinterpret_cast<RE::NiAVObject*>(descriptor->sourceRootAddress))) {
            } else {
                return nullptr;
            }
        }

        const auto sourceName = providerFixedStringView(drive.sourceName, ::rock::provider::ROCK_PROVIDER_MAX_EVIDENCE_NAME);
        if ((drive.flags & static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSourceName)) != 0 && !sourceName.empty()) {
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
    }

    weapon_drive_pose::NativeFrame PhysicsInteraction::captureProviderPartQueryPose(RE::NiNode* weaponNode,std::uint64_t generation) const
    {
        weapon_drive_pose::NativeFrame result(weaponNode,weaponNode->world);
        std::array<provider::PartDrive,provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1> drives{};
        const auto count=provider::copyWeaponPartDrives(drives.data(),static_cast<unsigned>(drives.size()),nullptr,false);
        const auto evidence=_weaponCollision.getProfileEvidenceDescriptors();
        std::array<RE::NiAVObject*,provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1> slots{};
        if(_providerDrives.generationKey==generation)for(std::size_t i=0;i<slots.size();++i)slots[i]=_providerDrives.nodeStates[i].node;
        for(unsigned i=0;i<count;++i)if(auto* node=resolveProviderWeaponPartDriveNode(weaponNode,generation,drives[i],evidence)){
            auto slot=std::find(slots.begin(),slots.end(),node);
            if(slot==slots.end())slot=std::find(slots.begin(),slots.end(),nullptr);
            if(slot!=slots.end()&&result.apply(node,providerTransformToNi(drives[i].targetTransform),
                drives[i].driveSpace==api::weaponparts::DriveSpace::WeaponRootLocal,drives[i].priority))*slot=node;
        }
        // The writer restores expired/revoked nodes after active drives. Their
        // old driven locals may still be present if native animation left them alone.
        if(_providerDrives.generationKey==generation)for(const auto& state:_providerDrives.nodeStates)
            if(state.node&&!result.contains(state.node)&&scene_relation::nodeContainsNode(weaponNode,state.node,64))
                result.apply(state.node,state.baselineLocal,false,0);
        return result;
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

        std::array<provider::PartDrive, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1> driveTargets{};
        std::array<std::uint64_t,
            ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1>
            driveOwners{};
        const std::uint32_t driveCount = ::rock::provider::copyWeaponPartDrives(
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
                    api::weaponparts::DriveApplication::LostPriority;
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
                                  const api::weaponparts::DriveResult& result) {
            if (!node) {
                return false;
            }
            if (_providerDrives.generationKey == 0) {
                _providerDrives.generationKey = currentWeaponGenerationKey;
            }
            for (auto& state : _providerDrives.nodeStates) {
                if (state.node == node) {
                    state.activeThisFrame = true;
                    state.ownerToken = result.owner;
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
                    state.ownerToken = result.owner;
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
            applicationResult.owner = driveOwners[i];
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
                    api::weaponparts::DriveApplication::StaleGeneration;
                continue;
            }
            auto* sourceNode = resolveProviderWeaponPartDriveNode(weaponNode,currentWeaponGenerationKey,drive,evidenceDescriptors);
            if (!sourceNode) {
                applicationResult.result =
                    api::weaponparts::DriveApplication::Unresolved;
                continue;
            }
            if (!sourceNode->parent) {
                applicationResult.result =
                    api::weaponparts::DriveApplication::MissingParent;
                continue;
            }
            const RE::NiTransform requestedLocal = providerTransformToNi(drive.targetTransform);
            if (!finiteNiTransform(requestedLocal)) {
                applicationResult.result =
                    api::weaponparts::DriveApplication::InvalidTransform;
                continue;
            }

            RE::NiTransform desiredWorld{};
            switch (drive.driveSpace) {
            case api::weaponparts::DriveSpace::SourceParentLocal:
                desiredWorld = transform_math::composeTransforms(sourceNode->parent->world, requestedLocal);
                break;
            case api::weaponparts::DriveSpace::WeaponRootLocal:
            default:
                desiredWorld = transform_math::composeTransforms(weaponNode->world, requestedLocal);
                break;
            }
            if (!finiteNiTransform(desiredWorld)) {
                applicationResult.result =
                    api::weaponparts::DriveApplication::InvalidTransform;
                continue;
            }

            const RE::NiTransform requestedNodeLocal = transform_math::composeTransforms(transform_math::invertTransform(sourceNode->parent->world), desiredWorld);
            if (!finiteNiTransform(requestedNodeLocal)) {
                applicationResult.result =
                    api::weaponparts::DriveApplication::InvalidTransform;
                continue;
            }
            if (!shouldApplyPriority(
                    sourceNode,
                    drive.priority,
                    _providerDrives.resultCount - 1)) {
                applicationResult.result =
                    api::weaponparts::DriveApplication::LostPriority;
                continue;
            }
            if (!markDrivenNode(sourceNode, applicationResult)) {
                applicationResult.result =
                    api::weaponparts::DriveApplication::CapacityRejected;
                continue;
            }
            sourceNode->local = requestedNodeLocal;
            f4vr::updateTransformsDown(sourceNode, true);
            applicationResult.result =
                api::weaponparts::DriveApplication::Applied;
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
            if (scene_relation::nodeContainsNode(weaponNode, state.node, 64)) {
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
                    result.owner = state.ownerToken;
                    result.weaponGenerationKey =
                        currentWeaponGenerationKey;
                    result.bodyId = state.bodyId;
                    result.groupId = state.groupId;
                    result.priority = state.priority;
                    result.result =
                        api::weaponparts::DriveApplication::Restored;
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
