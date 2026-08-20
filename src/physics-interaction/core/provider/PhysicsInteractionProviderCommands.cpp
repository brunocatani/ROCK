/*
 * PROVIDER COMMANDS: the write half of the provider surface.
 *
 * Drains the interaction command queue, applies weapon-part drives, and restores
 * part nodes when a drive lease expires. Runs on the main thread inside the frame,
 * which is why it may touch scene and Havok state where the query file may not.
 */

#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/core/PhysicsInteractionTransformValidation.h"

#include <algorithm>
#include <cstring>
#include <string_view>

#include "RockConfig.h"
#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/api/InteractionCommandQueue.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock
{
    using namespace physics_interaction_detail;

    namespace
    {
        template <std::size_t Capacity>
        void copyDriveSourceName(
            std::array<char, Capacity>& destination,
            const char* source)
        {
            std::memcpy(destination.data(), source, destination.size());
        }

        std::string_view providerFixedStringView(const char* value, std::size_t capacity)
        {
            if (!value) {
                return {};
            }
            for (std::size_t i = 0; i < capacity; ++i) {
                if (value[i] == '\0') {
                    return std::string_view(value, i);
                }
            }
            return std::string_view(value, capacity);
        }

        bool nodeNameEquals(RE::NiAVObject* node, std::string_view name)
        {
            if (!node || name.empty()) {
                return false;
            }
            const char* nodeName = node->name.c_str();
            return nodeName && std::string_view(nodeName) == name;
        }

        RE::NiAVObject* findWeaponNodeBySourceName(RE::NiAVObject* root, std::string_view sourceName, int maxDepth = 32)
        {
            if (!root || sourceName.empty() || maxDepth < 0) {
                return nullptr;
            }
            if (nodeNameEquals(root, sourceName)) {
                return root;
            }
            auto* node = root->IsNode();
            if (!node) {
                return nullptr;
            }
            auto& children = node->GetRuntimeData().children;
            for (std::uint16_t i = 0; i < children.size(); ++i) {
                if (auto* found = findWeaponNodeBySourceName(children[i].get(), sourceName, maxDepth - 1)) {
                    return found;
                }
            }
            return nullptr;
        }

    }

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
            const auto requestHand = [&]() {
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
            result.frameIndex = _palmClockGameFrameIndex.load(std::memory_order_acquire);
            result.worldGeneration = _worldGenerationAtomic.load(std::memory_order_acquire);
            result.skeletonGeneration = _skeletonGenerationAtomic.load(std::memory_order_acquire);
            result.providerGeneration = _providerGenerationAtomic.load(std::memory_order_acquire);

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
                grab_input_intent_policy::reset(_grabInputIntentStates[isLeft ? 1u : 0u]);
                peer_held_join_retry_policy::reset(_peerHeldJoinRetryStates[isLeft ? 1u : 0u]);
                shoulder_stash::resetRuntime(_shoulderStashStates[isLeft ? 1u : 0u]);
                mouth_consume::resetRuntime(_mouthConsumeStates[isLeft ? 1u : 0u]);
                hand.cancelStashCandidate();
                hand.cancelConsumeCandidate();
                input_remap_runtime::setHandHeldWeapon(isLeft, hand.isHoldingLooseWeapon());
            };

            if (isForceReleaseCommand || isThrownDropCommand) {
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

            const bool targetIsLooseGrenade = loose_grenade_runtime::isGrenadeRef(targetRef);
            if (targetIsLooseGrenade &&
                (handHoldsLooseGrenade(_rightHand) ||
                    handHoldsLooseGrenade(_leftHand) ||
                    hasActiveLooseGrenadeCommit())) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandBusy);
                continue;
            }

            auto& commit = _pendingForceGrabCommits[isLeft ? 1u : 0u];
            if (commit.active) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandBusy);
                continue;
            }

            const bool hasSourcePointOverride =
                (command.forceGrab.flags & static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::UsePreferredGrabPointGame)) != 0;

            commit = PendingForceGrabCommit{
                .active = true,
                .isLeft = isLeft,
                .origin = PendingForceGrabCommitOrigin::ProviderForceGrabCommand,
                .phase = PendingForceGrabCommitPhase::WaitingForSettle,
                .targetHandle = targetRef->GetHandle(),
                .targetIsLooseGrenade = targetIsLooseGrenade,
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
        _providerWeaponPartDriveResultCount = 0;
        if (!weaponNode || currentWeaponGenerationKey == 0 || !frame.worldReady) {
            if (weaponNode && currentWeaponGenerationKey != 0) {
                restoreExpiredProviderWeaponPartDriveNodes(weaponNode, currentWeaponGenerationKey);
            }
            return 0;
        }

        if (_providerWeaponPartDriveGenerationKey != 0 && _providerWeaponPartDriveGenerationKey != currentWeaponGenerationKey) {
            _providerWeaponPartDriveNodeStates = {};
            _providerWeaponPartDriveGenerationKey = 0;
        }
        for (auto& state : _providerWeaponPartDriveNodeStates) {
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
                WeaponCollisionProfileEvidenceDescriptor descriptor{};
                RE::NiAVObject* sourceNode = nullptr;
                if (_weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(drive.bodyId, descriptor, sourceNode) &&
                    sourceNode &&
                    descriptor.weaponGenerationKey == currentWeaponGenerationKey &&
                    acceptResolvedNode(sourceNode)) {
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
                _providerWeaponPartDriveResults[
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
            if (_providerWeaponPartDriveGenerationKey == 0) {
                _providerWeaponPartDriveGenerationKey = currentWeaponGenerationKey;
            }
            for (auto& state : _providerWeaponPartDriveNodeStates) {
                if (state.node == node) {
                    state.activeThisFrame = true;
                    state.ownerToken = result.ownerToken;
                    state.bodyId = result.bodyId;
                    state.groupId = result.groupId;
                    state.priority = result.priority;
                    copyDriveSourceName(
                        state.sourceName,
                        result.sourceName);
                    return true;
                }
            }
            for (auto& state : _providerWeaponPartDriveNodeStates) {
                if (!state.node) {
                    state.node = node;
                    state.baselineLocal = node->local;
                    state.ownerToken = result.ownerToken;
                    state.bodyId = result.bodyId;
                    state.groupId = result.groupId;
                    state.priority = result.priority;
                    copyDriveSourceName(
                        state.sourceName,
                        result.sourceName);
                    state.activeThisFrame = true;
                    return true;
                }
            }
            return false;
        };

        std::size_t drivenSourceNodeCount = 0;
        for (std::uint32_t i = 0; i < driveCount && i < driveTargets.size(); ++i) {
            const auto& drive = driveTargets[i];
            auto& applicationResult = _providerWeaponPartDriveResults[
                _providerWeaponPartDriveResultCount++];
            applicationResult = {};
            applicationResult.frameIndex =
                _palmClockGameFrameIndex.load(std::memory_order_acquire);
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
                    _providerWeaponPartDriveResultCount - 1)) {
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
        if (_providerWeaponPartDriveGenerationKey == 0) {
            return;
        }
        if (!weaponNode || currentWeaponGenerationKey == 0 || _providerWeaponPartDriveGenerationKey != currentWeaponGenerationKey) {
            _providerWeaponPartDriveNodeStates = {};
            _providerWeaponPartDriveGenerationKey = 0;
            return;
        }

        bool anyActive = false;
        for (auto& state : _providerWeaponPartDriveNodeStates) {
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
                if (_providerWeaponPartDriveResultCount <
                    _providerWeaponPartDriveResults.size()) {
                    auto& result = _providerWeaponPartDriveResults[
                        _providerWeaponPartDriveResultCount++];
                    result = {};
                    result.frameIndex =
                        _palmClockGameFrameIndex.load(
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
            _providerWeaponPartDriveGenerationKey = 0;
        }
    }

}
