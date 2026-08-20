#define ROCK_API_EXPORTS
#include "ROCKProviderApiInternal.h"
#include "api/ProviderColliderVisualizationRuntime.h"
#include "api/ProviderDebugOverlayRuntime.h"
#include "api/ProviderLeasePolicy.h"
#include "api/TouchGrabRegistry.h"
#include "api/detail/ProviderApiEntryPoints.h"
#include "api/detail/ProviderApiState.h"
#include "api/detail/ProviderAnimationAuthority.h"
#include "api/detail/ProviderAuthorityLeases.h"
#include "api/detail/ProviderApiCore.h"
#include "api/detail/ProviderCommandMarshal.h"
#include "api/detail/ProviderFrameDiff.h"
#include "api/detail/ProviderInteractionCommands.h"
#include "api/detail/ProviderTransformMath.h"
#include "api/detail/ProviderWeaponPartValidation.h"
#include "api/detail/ProviderWeaponParts.h"

#include <array>
#include <atomic>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string_view>

#include "physics-interaction/object/ExternalBodyRegistry.h"
#include "physics-interaction/api/InteractionCommandQueue.h"
#include "physics-interaction/api/InteractionCommandPolicy.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/weapon/parts/WeaponPartRuntime.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "monitor/GrabClockMonitor.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RockConfig.h"

#ifdef DrawText
#undef DrawText
#endif

namespace rock::provider::detail
{
    using namespace rock::provider;
    using namespace rock;

    template <std::size_t Capacity>
    void addUniqueOwner(
        std::array<std::uint64_t, Capacity>& owners,
        std::uint32_t& count,
        const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return;
        }
        for (std::uint32_t index = 0; index < count; ++index) {
            if (owners[index] == ownerToken) {
                return;
            }
        }
        if (count < owners.size()) {
            owners[count++] = ownerToken;
        }
    }

    std::uint64_t currentProviderFrameIndex()
    {
        const auto nextFrameIndex = s_nextFrameIndex.load(std::memory_order_acquire);
        return nextFrameIndex > 0 ? nextFrameIndex - 1 : 0;
    }

    void publishProviderEvent(RockProviderEventV1 event)
    {
        event.size = sizeof(RockProviderEventV1);
        event.version = ROCK_PROVIDER_API_VERSION;
        if (event.frameIndex == 0) {
            event.frameIndex = currentProviderFrameIndex();
        }
        if (s_generationStateAvailable.load(std::memory_order_acquire)) {
            if (event.worldGeneration == 0) {
                event.worldGeneration = s_currentWorldGeneration.load(
                    std::memory_order_acquire);
            }
            if (event.skeletonGeneration == 0) {
                event.skeletonGeneration = s_currentSkeletonGeneration.load(
                    std::memory_order_acquire);
            }
            if (event.providerGeneration == 0) {
                event.providerGeneration = s_currentProviderGeneration.load(
                    std::memory_order_acquire);
            }
        }

        std::scoped_lock lock(s_providerEventMutex);
        event.sequence = s_nextProviderEventSequence++;
        if (s_providerEventCount < s_providerEvents.size()) {
            const auto index =
                (s_providerEventHead + s_providerEventCount) %
                s_providerEvents.size();
            s_providerEvents[index] = event;
            ++s_providerEventCount;
        } else {
            s_providerEvents[s_providerEventHead] = event;
            s_providerEventHead =
                (s_providerEventHead + 1) % s_providerEvents.size();
            ++s_overwrittenProviderEventCount;
        }
    }

    [[nodiscard]] const RockProviderEventV1& providerEventAtLocked(
        const std::uint32_t logicalIndex)
    {
        return s_providerEvents[
            (s_providerEventHead + logicalIndex) % s_providerEvents.size()];
    }

    void publishAuthorityLostEvent(
        const std::uint64_t ownerToken,
        const RockProviderAuthorityKindV1 authorityKind,
        const std::uint32_t reason)
    {
        RockProviderEventV1 event{};
        event.kind = RockProviderEventKindV1::AuthorityLost;
        event.ownerToken = ownerToken;
        event.result = reason;
        event.data[0] = static_cast<std::uint32_t>(authorityKind);
        publishProviderEvent(event);
    }

    RockProviderResultV1 validateRegisteredOwnerCapabilityLocked(
        std::uint64_t ownerToken,
        RockProviderConsumerCapabilityV1 capability)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        if (!findConsumerSlotLocked(ownerToken)) {
            return RockProviderResultV1::OwnerNotRegistered;
        }
        if (!consumerHasCapabilityLocked(ownerToken, capability)) {
            return RockProviderResultV1::PermissionDenied;
        }
        return RockProviderResultV1::Ok;
    }

    [[nodiscard]] RockProviderResultV1 revokeOwner(
        const std::uint64_t ownerToken,
        const RevokeReason revokeReason,
        const bool alsoUnregister)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto invalidationReason =
            revokeReason == RevokeReason::CallbackFault ?
                RockProviderSuppressionInvalidationReasonV1::CallbackFault :
                RockProviderSuppressionInvalidationReasonV1::OwnerUnregistered;
        const auto commandFailure =
            revokeReason == RevokeReason::CallbackFault ?
                RockProviderInteractionFailureV1::InvalidRequest :
                RockProviderInteractionFailureV1::OwnerNotRegistered;

        {
            // Keep all owner registries in one deadlock-safe acquisition.
            std::scoped_lock lock(
                s_consumerMutex,
                s_interactionCommandMutex,
                s_handInputSuppressionMutex,
                s_weaponPartMutex,
                s_nativeAnimationAuthorityMutex,
                s_equippedWeaponHandlingAuthorityMutex);

            if (alsoUnregister) {
                auto* consumer = findConsumerSlotLocked(ownerToken);
                if (!consumer) {
                    return RockProviderResultV1::OwnerNotRegistered;
                }
                *consumer = {};
            }

            clearInteractionCommandsForOwnerLocked(
                ownerToken,
                commandFailure);
            clearHandInputSuppressionsForOwnerLocked(
                ownerToken,
                RockProviderHand::None,
                invalidationReason);
            clearWeaponPartTargetsForOwnerLocked(ownerToken);
            clearWeaponPartDrivesForOwnerLocked(ownerToken);
            clearNativeAnimationAuthorityForOwnerLocked(ownerToken);
            clearEquippedWeaponHandlingAuthorityForOwnerLocked(ownerToken);
        }
        {
            std::scoped_lock lock(s_externalBodyMutex);
            s_externalBodies.clearOwner(ownerToken);
        }
        {
            std::scoped_lock lock(s_touchGrabMutex);
            s_touchGrabTargets.clearOwner(ownerToken);
        }
        {
            std::scoped_lock lock(s_offhandReservationMutex);
            if (s_offhandReservationSlot.ownerToken == ownerToken) {
                clearOffhandReservationLocked(invalidationReason);
            }
        }
        {
            std::scoped_lock lock(s_callbackMutex);
            for (auto& callback : s_callbacks) {
                if (callback.ownerToken == ownerToken) {
                    callback = {};
                }
            }
        }
        {
            std::scoped_lock lock(s_animationPhaseCallbackMutex);
            clearAnimationPhaseCallbacksForOwnerLocked(ownerToken);
        }
        (void)clearHandVisualAuthorityForOwner(
            ownerToken,
            RockProviderHand::None,
            true);
        clearNativeAnimationRuntimePublicationForOwner(ownerToken);
        provider_debug_overlay::clear(ownerToken);
        provider_collider_visualization::clear(ownerToken);
        if (revokeReason == RevokeReason::CallbackFault) {
            publishAuthorityLostEvent(
                ownerToken,
                RockProviderAuthorityKindV1::Unknown,
                static_cast<std::uint32_t>(invalidationReason));
        }
        return RockProviderResultV1::Ok;
    }

    void clearOwnerStateAfterCallbackFault(const std::uint64_t ownerToken)
    {
        (void)revokeOwner(ownerToken, RevokeReason::CallbackFault, false);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiUnregisterConsumerV1(std::uint64_t ownerToken)
    {
        return revokeOwner(
            ownerToken,
            RevokeReason::OwnerUnregistered,
            true);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetHandInteractionStateV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        RockProviderHandInteractionStateV1* outState)
    {
        const auto entryResult = validateOutputEntry(ownerToken, outState);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::HandInteractionState);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        std::scoped_lock lock(s_snapshotMutex);
        if (!s_hasSnapshot) {
            return RockProviderResultV1::NotReady;
        }
        *outState = s_lastHandInteractionStates[
            hand == RockProviderHand::Left ? 1u : 0u];
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyProviderEventsSinceV1(
        const std::uint64_t ownerToken,
        const std::uint64_t afterSequence,
        RockProviderEventV1* outEvents,
        const std::uint32_t maxEvents,
        RockProviderEventStreamStateV1* outStreamState)
    {
        const auto entryResult = validateOutputEntry(
            ownerToken,
            outStreamState);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (maxEvents != 0 && !outEvents) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::ProviderEvents);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        std::scoped_lock lock(s_providerEventMutex);
        *outStreamState = {};
        if (s_providerEventCount != 0) {
            outStreamState->oldestRetainedSequence =
                providerEventAtLocked(0).sequence;
        }
        outStreamState->latestEmittedSequence =
            s_nextProviderEventSequence > 1 ?
                s_nextProviderEventSequence - 1 :
                0;
        outStreamState->overwrittenCount = s_overwrittenProviderEventCount;
        if (afterSequence != 0 &&
            outStreamState->oldestRetainedSequence > 1 &&
            afterSequence <
                outStreamState->oldestRetainedSequence - 1) {
            outStreamState->flags |= static_cast<std::uint32_t>(
                RockProviderEventStreamFlagV1::GapBeforeFirstCopied);
        }
        if (s_overwrittenProviderEventCount != 0) {
            outStreamState->flags |= static_cast<std::uint32_t>(
                RockProviderEventStreamFlagV1::RingOverwroteRecords);
        }

        std::uint32_t copied = 0;
        for (std::uint32_t i = 0;
             i < s_providerEventCount && copied < maxEvents;
             ++i) {
            const auto& event = providerEventAtLocked(i);
            if (event.sequence <= afterSequence ||
                (event.ownerToken != 0 && event.ownerToken != ownerToken)) {
                continue;
            }
            outEvents[copied++] = event;
        }
        outStreamState->copiedCount = copied;
        if (copied != 0) {
            outStreamState->firstCopiedSequence = outEvents[0].sequence;
            outStreamState->lastCopiedSequence =
                outEvents[copied - 1].sequence;
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetEquippedWeaponStateV1(
        const std::uint64_t ownerToken,
        RockProviderEquippedWeaponStateV1* outState)
    {
        const auto entryResult = validateOutputEntry(ownerToken, outState);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::FrameSnapshots);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        std::scoped_lock lock(s_snapshotMutex);
        if (!s_hasSnapshot) {
            return RockProviderResultV1::NotReady;
        }
        *outState = s_lastEquippedWeaponState;
        return RockProviderResultV1::Ok;
    }

    template <class Output, class Query>
    RockProviderResultV1 queryPhysicsInteractionValueV1(
        const std::uint64_t ownerToken,
        const RockProviderConsumerCapabilityV1 capability,
        Output* output,
        Query&& query,
        const bool requireAnimationThread = false)
    {
        const auto entryResult = validateOutputEntry(
            ownerToken,
            output,
            requireAnimationThread ?
                EntryThreadPolicy::AnimationOwner :
                EntryThreadPolicy::AnyThread);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        const auto ownerResult = validateReadCapability(ownerToken, capability);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        return query(*pi, *output) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetScopeSightStateV1(
        const std::uint64_t ownerToken,
        RockProviderScopeSightStateV1* outState)
    {
        return queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::ScopeSightState,
            outState,
            [](PhysicsInteraction& pi, RockProviderScopeSightStateV1& state) {
                return pi.queryProviderScopeSightStateV1(state);
            },
            true);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetWeaponCompositionStateV1(
        const std::uint64_t ownerToken,
        RockProviderWeaponCompositionStateV1* outState)
    {
        return queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::WeaponComposition,
            outState,
            [](PhysicsInteraction& pi, RockProviderWeaponCompositionStateV1& state) {
                return pi.queryProviderWeaponCompositionStateV1(state);
            });
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyWeaponCompositionEntriesV1(
        const std::uint64_t ownerToken,
        RockProviderWeaponCompositionEntryV1* outEntries,
        const std::uint32_t maxEntries,
        std::uint32_t* outEntryCount)
    {
        if (ownerToken == 0 || !outEntryCount ||
            (maxEntries != 0 && !outEntries)) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outEntryCount = 0;
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::WeaponComposition);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        *outEntryCount = pi->copyProviderWeaponCompositionEntriesV1(
            outEntries,
            maxEntries);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetSelectedAuthoredGripPoseV1(
        const std::uint64_t ownerToken,
        RockProviderAuthoredGripPoseV1* outPose)
    {
        return queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::PoseReadback,
            outPose,
            [](PhysicsInteraction& pi, RockProviderAuthoredGripPoseV1& pose) {
                return pi.queryProviderSelectedAuthoredGripPoseV1(pose);
            },
            true);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetPresentedHandPoseV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        RockProviderPresentedHandPoseV1* outPose)
    {
        const auto entryResult = validateOutputEntry(
            ownerToken,
            outPose,
            EntryThreadPolicy::AnimationOwner);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        const auto result = queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::PoseReadback,
            outPose,
            [hand](PhysicsInteraction& pi, RockProviderPresentedHandPoseV1& pose) {
                return pi.queryProviderPresentedHandPoseV1(hand, pose);
            },
            true);
        if (result == RockProviderResultV1::Ok) {
            outPose->frameIndex = currentProviderFrameIndex();
            outPose->presentationSequence =
                s_activeAnimationPhaseFrameIndex.load(std::memory_order_acquire);
        }
        return result;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopySemanticHandContactsV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        const std::uint32_t maxFramesSinceContact,
        RockProviderSemanticHandContactV1* outContacts,
        const std::uint32_t maxContacts,
        std::uint32_t* outContactCount)
    {
        const auto entryResult = validateArrayEntry(
            ownerToken,
            outContacts,
            maxContacts,
            outContactCount,
            EntryThreadPolicy::AnimationOwner,
            hand == RockProviderHand::Right || hand == RockProviderHand::Left);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        *outContactCount = 0;
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::SemanticHandContacts);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        *outContactCount = pi->copyProviderSemanticHandContactsV1(
            hand,
            maxFramesSinceContact,
            outContacts,
            maxContacts);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyPlayerColliderDescriptorsV1(
        const std::uint64_t ownerToken,
        RockProviderPlayerColliderDescriptorV1* outDescriptors,
        const std::uint32_t maxDescriptors,
        std::uint32_t* outDescriptorCount)
    {
        const auto entryResult = validateArrayEntry(
            ownerToken,
            outDescriptors,
            maxDescriptors,
            outDescriptorCount,
            EntryThreadPolicy::AnimationOwner);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        *outDescriptorCount = 0;
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::PlayerColliderDescriptors);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        *outDescriptorCount = pi->copyProviderPlayerColliderDescriptorsV1(
            outDescriptors,
            maxDescriptors);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetHandCollisionAvailabilityV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        RockProviderHandCollisionAvailabilityV1* outState)
    {
        const auto entryResult = validateOutputEntry(
            ownerToken,
            outState,
            EntryThreadPolicy::AnimationOwner);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        return queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::PlayerColliderDescriptors,
            outState,
            [hand](PhysicsInteraction& pi, RockProviderHandCollisionAvailabilityV1& state) {
                return pi.queryProviderHandCollisionAvailabilityV1(hand, state);
            },
            true);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiQueryWorldRaycastV1(
        const std::uint64_t ownerToken,
        const RockProviderWorldRaycastRequestV1* request,
        RockProviderWorldRaycastResultV1* outResult)
    {
        auto entryResult = validateEntry(
            request,
            EntryThreadPolicy::AnimationOwner);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        entryResult = validateEntry(outResult);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!finiteProviderPoint(request->startGame) ||
            !finiteProviderPoint(request->directionGame) ||
            !std::isfinite(request->maxDistanceGame) ||
            request->maxDistanceGame <= 0.0f ||
            request->maxDistanceGame >
                ROCK_PROVIDER_MAX_WORLD_RAYCAST_DISTANCE_GAME_V1) {
            return RockProviderResultV1::InvalidArgument;
        }
        const float directionLengthSquared =
            request->directionGame.x * request->directionGame.x +
            request->directionGame.y * request->directionGame.y +
            request->directionGame.z * request->directionGame.z;
        if (!std::isfinite(directionLengthSquared) ||
            directionLengthSquared <= 1.0e-8f) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }
        if (!apiIsProviderReady()) {
            return RockProviderResultV1::NotReady;
        }

        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }

        const auto frameIndex = currentProviderFrameIndex();
        {
            std::scoped_lock lock(s_consumerMutex);
            auto* slot = findConsumerSlotLocked(ownerToken);
            if (!slot) {
                return RockProviderResultV1::OwnerNotRegistered;
            }
            if (!hasConsumerCapabilityV1(
                    slot->grantedCapabilities,
                    RockProviderConsumerCapabilityV1::WorldRaycasts)) {
                return RockProviderResultV1::PermissionDenied;
            }
            if (slot->worldRaycastFrameIndex != frameIndex) {
                slot->worldRaycastFrameIndex = frameIndex;
                slot->worldRaycastCount = 0;
            }
            if (slot->worldRaycastCount >=
                ROCK_PROVIDER_MAX_WORLD_RAYCASTS_PER_OWNER_PER_FRAME_V1) {
                return RockProviderResultV1::CapacityFull;
            }
            ++slot->worldRaycastCount;
        }

        *outResult = {};
        outResult->frameIndex = frameIndex;
        if (!pi->queryProviderWorldRaycastV1(*request, *outResult)) {
            return RockProviderResultV1::WorldNotReady;
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
        apiSetColliderVisualizationOverrideV1(
            const std::uint64_t ownerToken,
            const RockProviderColliderVisualizationRequestV1* request)
    {
        const auto entryResult = validateEntry(
            request,
            EntryThreadPolicy::AnimationOwner);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->weaponGenerationKey == 0 ||
            request->bodyId == 0x7FFF'FFFF ||
            request->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }
        if (!apiIsProviderReady()) {
            return RockProviderResultV1::NotReady;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi) {
            return RockProviderResultV1::NotReady;
        }
        const auto capabilityResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::
                ColliderVisualizationOverride);
        if (capabilityResult != RockProviderResultV1::Ok) {
            return capabilityResult;
        }

        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot ||
                request->weaponGenerationKey !=
                    s_lastSnapshot.weaponGenerationKey) {
                return RockProviderResultV1::TargetUnavailable;
            }
        }
        if (!pi->isProviderWeaponBodyCurrentV1(
                request->weaponGenerationKey,
                request->bodyId)) {
            return RockProviderResultV1::TargetInvalid;
        }

        return provider_collider_visualization::set(
            ownerToken,
            *request,
            currentProviderFrameIndex());
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
        apiClearColliderVisualizationOverrideV1(
            const std::uint64_t ownerToken)
    {
        const auto entryResult = validateOwnerEntry(
            ownerToken,
            EntryThreadPolicy::AnimationOwner);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        const auto capabilityResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::
                ColliderVisualizationOverride);
        if (capabilityResult != RockProviderResultV1::Ok) {
            return capabilityResult;
        }
        provider_collider_visualization::clear(ownerToken);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiPublishDebugOverlayV1(
        const std::uint64_t ownerToken,
        const RockProviderDebugOverlayPublicationV1* publication)
    {
        const auto entryResult = validateEntry(
            publication,
            EntryThreadPolicy::AnimationOwner);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (publication->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            publication->worldGeneration,
            publication->skeletonGeneration,
            publication->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }
        if (!apiIsProviderReady()) {
            return RockProviderResultV1::NotReady;
        }
        {
            std::scoped_lock lock(s_consumerMutex);
            const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::DebugOverlayPublication);
            if (ownerResult != RockProviderResultV1::Ok) {
                return ownerResult;
            }
        }
        return provider_debug_overlay::publish(
            ownerToken,
            *publication,
            currentProviderFrameIndex());
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearDebugOverlayV1(
        const std::uint64_t ownerToken)
    {
        const auto entryResult = validateOwnerEntry(
            ownerToken,
            EntryThreadPolicy::AnimationOwner);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        {
            std::scoped_lock lock(s_consumerMutex);
            const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::DebugOverlayPublication);
            if (ownerResult != RockProviderResultV1::Ok) {
                return ownerResult;
            }
        }
        provider_debug_overlay::clear(ownerToken);
        return RockProviderResultV1::Ok;
    }

    bool ROCK_PROVIDER_CALL apiQueryWeaponContactAtPoint(
        const RockProviderWeaponContactQuery* query,
        RockProviderWeaponContactResult* outResult)
    {
        if (!query || !outResult ||
            query->size != sizeof(RockProviderWeaponContactQuery) ||
            outResult->size != sizeof(RockProviderWeaponContactResult)) {
            return false;
        }

        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return false;
        }

        return pi->queryProviderWeaponContactAtPoint(*query, *outResult);
    }

    bool ROCK_PROVIDER_CALL apiQueryEquippedWeaponClassificationV1(RockProviderWeaponClassificationV1* outResult)
    {
        if (!outResult || outResult->size != sizeof(RockProviderWeaponClassificationV1)) {
            return false;
        }

        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return false;
        }

        return pi->queryProviderEquippedWeaponClassificationV1(*outResult);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetWeaponEvidenceDetailCountV1()
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->getProviderWeaponEvidenceDetailCountV1();
    }

    std::uint32_t ROCK_PROVIDER_CALL apiCopyWeaponEvidenceDetailsV1(
        RockProviderWeaponEvidenceDetailV1* outDetails,
        std::uint32_t maxDetails)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderWeaponEvidenceDetailsV1(outDetails, maxDetails);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetWeaponEvidenceDetailPointCountV1(std::uint32_t bodyId)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->getProviderWeaponEvidenceDetailPointCountV1(bodyId);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiCopyWeaponEvidenceDetailPointsV1(
        std::uint32_t bodyId,
        RockProviderPoint3* outPoints,
        std::uint32_t maxPoints)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderWeaponEvidenceDetailPointsV1(bodyId, outPoints, maxPoints);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetBodyContactSnapshotV1(
        RockProviderBodyContactV1* outContacts,
        std::uint32_t maxContacts)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderBodyContacts(outContacts, maxContacts);
    }

    bool ROCK_PROVIDER_CALL apiGetRawWandButtonStateV1(RockProviderHand hand, std::uint32_t buttonId, RockProviderRawWandButtonStateV1* outState)
    {
        if (!outState || outState->size != sizeof(RockProviderRawWandButtonStateV1)) {
            return false;
        }
        if (hand != RockProviderHand::Left && hand != RockProviderHand::Right) {
            return false;
        }
        if (!rock::input_remap_policy::isValidButtonId(static_cast<int>(buttonId))) {
            return false;
        }

        // Level state only by design: ROCK consumes its press/release edge queues internally each frame, so exposing them would race consumers.
        const auto raw = rock::input_remap_runtime::peekRawButtonState(hand == RockProviderHand::Left, static_cast<int>(buttonId));
        *outState = {};
        outState->size = sizeof(RockProviderRawWandButtonStateV1);
        outState->version = ROCK_PROVIDER_API_VERSION;
        outState->available = raw.available ? 1u : 0u;
        outState->held = raw.held ? 1u : 0u;
        outState->sampleSequence = raw.sampleSequence;
        outState->sampleAgeMilliseconds = raw.sampleAgeMilliseconds;
        outState->availabilityReason =
            static_cast<RockProviderInputAvailabilityReasonV1>(
                raw.availabilityReason);
        return true;
    }

    bool ROCK_PROVIDER_CALL apiIsNativePipboyInputSuppressedV1()
    {
        return rock::input_remap_runtime::isNativePipboyInputSuppressionActive();
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetWeaponEmitterCountV1()
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }
        return pi->getProviderWeaponEmitterCountV1();
    }

    std::uint32_t ROCK_PROVIDER_CALL apiCopyWeaponEmittersV1(
        RockProviderWeaponEmitterV1* outEmitters,
        std::uint32_t maxEmitters)
    {
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return 0;
        }
        return pi->copyProviderWeaponEmittersV1(outEmitters, maxEmitters);
    }

}

namespace rock::provider
{
    using namespace detail;

    void setPhysicsInteractionInstance(rock::PhysicsInteraction* pi)
    {
        s_physicsInteraction.store(pi, std::memory_order_release);
    }

    void dispatchFrameCallbacks(rock::PhysicsInteraction& pi)
    {
        RockProviderFrameSnapshot snapshot{};
        snapshot.frameIndex = s_nextFrameIndex.fetch_add(1, std::memory_order_acq_rel);
        pi.fillProviderFrameSnapshot(snapshot);
        snapshot.externalBodyCount = currentExternalBodyCount();

        // These atomics are the allocation-free generation authority used by
        // lease validation during this frame. Publishing them before pruning
        // makes a generation transition revoke stale state in the same frame.
        s_currentWorldGeneration.store(
            snapshot.worldGeneration,
            std::memory_order_release);
        s_currentSkeletonGeneration.store(
            snapshot.skeletonGeneration,
            std::memory_order_release);
        s_currentProviderGeneration.store(
            snapshot.providerGeneration,
            std::memory_order_release);
        s_generationStateAvailable.store(true, std::memory_order_release);

        {
            std::scoped_lock lock(s_handInputSuppressionMutex);
            pruneExpiredHandInputSuppressionsLocked(snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_nativeAnimationAuthorityMutex);
            pruneExpiredNativeAnimationAuthorityLocked(snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_nativeAnimationRuntimePublicationMutex);
            pruneExpiredNativeAnimationRuntimePublicationLocked(
                snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_equippedWeaponHandlingAuthorityMutex);
            pruneExpiredEquippedWeaponHandlingAuthorityLocked(
                snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_weaponPartMutex);
            pruneExpiredWeaponPartDrivesLocked(snapshot.frameIndex);
        }
        {
            std::scoped_lock lock(s_offhandReservationMutex);
            pruneExpiredOffhandReservationLocked(snapshot.frameIndex);
        }
        snapshot.offhandReservation = currentOffhandReservation();

        provider_debug_overlay::PruneResult overlayPrune{};
        provider_debug_overlay::prune(
            snapshot.frameIndex,
            snapshot.worldGeneration,
            snapshot.skeletonGeneration,
            snapshot.providerGeneration,
            overlayPrune);
        for (std::uint32_t index = 0; index < overlayPrune.count; ++index) {
            publishAuthorityLostEvent(
                overlayPrune.publishers[index].ownerToken,
                RockProviderAuthorityKindV1::DebugOverlay,
                static_cast<std::uint32_t>(
                    overlayPrune.publishers[index].reason));
        }

        provider_collider_visualization::Invalidation
            colliderVisualizationInvalidation{};
        provider_collider_visualization::Snapshot
            colliderVisualizationSnapshot{};
        const bool colliderVisualizationActive =
            provider_collider_visualization::copySnapshot(
                colliderVisualizationSnapshot);
        const bool colliderVisualizationBodyCurrent =
            !colliderVisualizationActive ||
            pi.isProviderWeaponBodyCurrentV1(
                colliderVisualizationSnapshot.weaponGenerationKey,
                colliderVisualizationSnapshot.bodyId);
        provider_collider_visualization::prune(
            snapshot.frameIndex,
            snapshot.worldGeneration,
            snapshot.skeletonGeneration,
            snapshot.providerGeneration,
            snapshot.weaponGenerationKey,
            colliderVisualizationBodyCurrent,
            colliderVisualizationInvalidation);
        if (colliderVisualizationInvalidation.ownerToken != 0) {
            publishAuthorityLostEvent(
                colliderVisualizationInvalidation.ownerToken,
                RockProviderAuthorityKindV1::ColliderVisualization,
                static_cast<std::uint32_t>(
                    colliderVisualizationInvalidation.reason));
        }

        std::array<RockProviderWeaponPartGripStateV1, 2> partGripStates{};
        pi.fillProviderWeaponPartGripStates(partGripStates);

        std::array<RockProviderHandInteractionStateV1, 2>
            handInteractionStates{};
        pi.fillProviderHandInteractionStates(handInteractionStates);
        for (auto& state : handInteractionStates) {
            state.frameIndex = snapshot.frameIndex;
        }

        RockProviderEquippedWeaponStateV1 equippedWeaponState{};
        (void)pi.queryProviderEquippedWeaponStateV1(equippedWeaponState);
        equippedWeaponState.frameIndex = snapshot.frameIndex;

        bool hadPrevious = false;
        bool lifecycleChanged = false;
        bool equippedTerminalChanged = false;
        std::array<bool, 2> handChanged{};

        {
            std::scoped_lock lock(s_snapshotMutex);
            hadPrevious = s_hasSnapshot;
            const auto previousSnapshot = s_lastSnapshot;
            const auto previousHands = s_lastHandInteractionStates;
            const auto previousEquipped = s_lastEquippedWeaponState;

            for (std::size_t index = 0;
                 index < handInteractionStates.size();
                 ++index) {
                auto& currentHand = handInteractionStates[index];
                const auto& previousHand = previousHands[index];
                if (hadPrevious &&
                    handGripActive(previousHand) &&
                    !handGripActive(currentHand) &&
                    currentHand.phase ==
                        RockProviderHandInteractionPhaseV1::Idle) {
                    currentHand.phase =
                        RockProviderHandInteractionPhaseV1::Releasing;
                    currentHand.targetKind = previousHand.targetKind;
                    currentHand.reservedTargetIdentity =
                        previousHand.reservedTargetIdentity;
                    currentHand.targetFormId = previousHand.targetFormId;
                    currentHand.primaryBodyId = previousHand.primaryBodyId;
                    constexpr std::uint32_t touchGrabClassificationFlags =
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::TouchGrab) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                FixedSurfaceLatch) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                GlobalSurfaceLatch) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                SurfaceAnchorValid) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                MeshSurfaceAnchor) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                MeshFingerPose) |
                        static_cast<std::uint32_t>(
                            RockProviderHandInteractionFlagV1::
                                MeshCollisionFallback);
                    currentHand.flags |=
                        previousHand.flags &
                        touchGrabClassificationFlags;
                    currentHand.surfaceAnchorGame =
                        previousHand.surfaceAnchorGame;
                    currentHand.surfaceGripMode =
                        previousHand.surfaceGripMode;
                    currentHand.heldBodyCount = previousHand.heldBodyCount;
                    std::copy(
                        std::begin(previousHand.heldBodyIds),
                        std::end(previousHand.heldBodyIds),
                        std::begin(currentHand.heldBodyIds));
                }
                assignHandInteractionSequences(
                    currentHand,
                    previousHand,
                    hadPrevious);
                handChanged[index] = !hadPrevious ||
                    !sameHandInteractionPayload(
                        currentHand,
                        previousHand);
            }

            lifecycleChanged = !hadPrevious ||
                !sameLifecyclePayload(snapshot, previousSnapshot);
            const bool weaponChanged = !hadPrevious ||
                !sameWeaponPayload(snapshot, previousSnapshot) ||
                !sameEquippedWeaponPayload(
                    equippedWeaponState,
                    previousEquipped);
            const bool transitionChanged = !hadPrevious ||
                equippedWeaponState.transitionSequence !=
                    previousEquipped.transitionSequence ||
                equippedWeaponState.terminalSequence !=
                    previousEquipped.terminalSequence;
            const bool collisionChanged = !hadPrevious ||
                snapshot.collisionGeneration !=
                    previousSnapshot.collisionGeneration;
            const bool handRolesChanged = !hadPrevious ||
                snapshot.primaryHand != previousSnapshot.primaryHand ||
                snapshot.offhandHand != previousSnapshot.offhandHand ||
                snapshot.offhandReservation !=
                    previousSnapshot.offhandReservation;

            if (lifecycleChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::Lifecycle);
            }
            if (handChanged[0]) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::RightHand);
            }
            if (handChanged[1]) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::LeftHand);
            }
            if (weaponChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::Weapon);
            }
            if (transitionChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::EquippedTransition);
            }
            if (collisionChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::Collision);
            }
            if (handRolesChanged) {
                snapshot.stateChangeMask |= static_cast<std::uint32_t>(
                    RockProviderFrameStateChangeFlagV1::HandRoles);
            }
            snapshot.stateSequence = !hadPrevious ?
                1 :
                (snapshot.stateChangeMask != 0 ?
                        advanceSequence(previousSnapshot.stateSequence) :
                        previousSnapshot.stateSequence);
            snapshot.enrichmentFlags |= static_cast<std::uint32_t>(
                RockProviderFrameEnrichmentFlagV1::StateSequenceValid);
            snapshot.equippedWeaponTransitionSequence =
                equippedWeaponState.transitionSequence;

            equippedTerminalChanged = hadPrevious &&
                equippedWeaponState.terminalSequence != 0 &&
                equippedWeaponState.terminalSequence !=
                    previousEquipped.terminalSequence;
            s_lastSnapshot = snapshot;
            s_hasSnapshot = true;
            s_lastPartGripStates = partGripStates;
            s_lastHandInteractionStates = handInteractionStates;
            s_lastEquippedWeaponState = equippedWeaponState;
        }

        if (hadPrevious && lifecycleChanged) {
            RockProviderEventV1 event{};
            event.kind = RockProviderEventKindV1::LifecycleChanged;
            event.result = static_cast<std::uint32_t>(
                snapshot.lastLifecycleReason);
            event.subjectSequence = snapshot.stateSequence;
            event.data[0] = snapshot.lifecycleFlags;
            event.data[1] = snapshot.providerReady;
            event.data[2] = snapshot.stateChangeMask;
            publishProviderEvent(event);
        }
        if (hadPrevious) {
            for (std::size_t index = 0;
                 index < handInteractionStates.size();
                 ++index) {
                if (!handChanged[index]) {
                    continue;
                }
                const auto& state = handInteractionStates[index];
                RockProviderEventV1 event{};
                event.kind = RockProviderEventKindV1::GrabStateChanged;
                event.hand = state.hand;
                event.formId = state.targetFormId;
                event.result = static_cast<std::uint32_t>(state.phase);
                event.subjectSequence = state.stateSequence;
                event.data[0] = static_cast<std::uint32_t>(
                    state.targetKind);
                event.data[1] = state.primaryBodyId;
                event.data[2] = state.flags;
                publishProviderEvent(event);
            }
        }
        if (equippedTerminalChanged) {
            RockProviderEventV1 event{};
            event.kind =
                RockProviderEventKindV1::EquippedWeaponTransitionTerminal;
            event.weaponGenerationKey =
                equippedWeaponState.weaponGenerationKey;
            event.formId = equippedWeaponState.weaponFormId;
            event.result = static_cast<std::uint32_t>(
                equippedWeaponState.terminalResult);
            event.subjectSequence =
                equippedWeaponState.terminalSequence;
            event.data[0] = static_cast<std::uint32_t>(
                equippedWeaponState.transitionSource);
            event.data[1] = equippedWeaponState.flags;
            publishProviderEvent(event);
        }

        for (std::size_t index = 0; index < s_callbacks.size(); ++index) {
            CallbackSlot slot{};
            {
                std::scoped_lock lock(s_callbackMutex);
                slot = s_callbacks[index];
            }
            if (slot.callback) {
                FrameCallbackInvocationResult callbackResult{};
                try {
                    callbackResult = invokeFrameCallbackSafely(
                        slot.callback,
                        &snapshot,
                        slot.userData);
                } catch (...) {
                    callbackResult.healthy = false;
                }

                if (!callbackResult.healthy) {
                    HMODULE faultModule = nullptr;
                    constexpr auto moduleFlags =
                        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                        GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT;
                    const auto faultAddress =
                        callbackResult.exceptionAddress;
                    const bool hasFaultModule =
                        faultAddress != 0 &&
                        GetModuleHandleExA(
                            moduleFlags,
                            reinterpret_cast<LPCSTR>(faultAddress),
                            &faultModule) != FALSE;
                    if (hasFaultModule) {
                        std::array<char, MAX_PATH> modulePath{};
                        const auto pathLength = GetModuleFileNameA(
                            faultModule,
                            modulePath.data(),
                            static_cast<DWORD>(modulePath.size()));
                        const char* moduleName = modulePath.data();
                        if (pathLength != 0) {
                            if (const auto* slash =
                                    std::strrchr(moduleName, '\\')) {
                                moduleName = slash + 1;
                            }
                        } else {
                            moduleName = "<unknown-module>";
                        }
                        const auto moduleBase =
                            reinterpret_cast<std::uintptr_t>(
                                faultModule);
                        logger::error(
                            "ROCK provider frame callback token {} faulted: "
                            "exception=0x{:08X} instruction={}+0x{:X} "
                            "(0x{:016X}); unregistering the callback.",
                            slot.token,
                            callbackResult.exceptionCode,
                            moduleName,
                            faultAddress - moduleBase,
                            faultAddress);
                    } else {
                        logger::error(
                            "ROCK provider frame callback token {} faulted: "
                            "exception=0x{:08X} instruction=0x{:016X}; "
                            "unregistering the callback.",
                            slot.token,
                            callbackResult.exceptionCode,
                            faultAddress);
                    }
                    if (slot.ownerToken != 0) {
                        clearOwnerStateAfterCallbackFault(slot.ownerToken);
                    } else {
                        clearCallbackSlot(slot.token);
                    }
                }
            }
        }

        // Embedded grab-clock monitor panel consumes the same fully built
        // snapshot as external consumers, but in-process with direct access
        // to ROCK internals (game thread, every producer frame including
        // menu/config-blocked ones so the panel can hide itself).
        rock::monitor::onProviderFrame(snapshot);
    }

    void clearExternalBodiesForProviderLoss()
    {
        std::array<std::uint64_t, ROCK_PROVIDER_MAX_CONSUMERS_V1>
            suppressionOwners{};
        std::array<std::uint64_t, ROCK_PROVIDER_MAX_CONSUMERS_V1>
            suppressionSlotOwners{};
        std::array<std::uint64_t, ROCK_PROVIDER_MAX_CONSUMERS_V1>
            targetOwners{};
        std::array<std::uint64_t, ROCK_PROVIDER_MAX_CONSUMERS_V1>
            driveOwners{};
        std::array<std::uint64_t, ROCK_PROVIDER_MAX_CONSUMERS_V1>
            nativeAnimationOwners{};
        std::array<std::uint64_t, ROCK_PROVIDER_MAX_CONSUMERS_V1>
            handVisualOwners{};
        std::uint32_t suppressionOwnerCount = 0;
        std::uint32_t suppressionSlotOwnerCount = 0;
        std::uint32_t targetOwnerCount = 0;
        std::uint32_t driveOwnerCount = 0;
        std::uint32_t nativeAnimationOwnerCount = 0;
        std::uint32_t handVisualOwnerCount = 0;
        std::uint64_t nativeRuntimeOwner = 0;
        std::uint64_t equippedHandlingOwner = 0;

        {
            std::scoped_lock lock(s_externalBodyMutex);
            s_externalBodies.clearAll();
        }
        {
            std::scoped_lock lock(s_touchGrabMutex);
            s_touchGrabTargets.clearAll();
        }
        clearInteractionCommandsForProviderLossV1(RockProviderInteractionFailureV1::ProviderNotReady);
        {
            std::scoped_lock lock(s_handInputSuppressionMutex);
            for (const auto& slot : s_handInputSuppressions) {
                if (slot.ownerToken == 0) {
                    continue;
                }
                addUniqueOwner(
                    suppressionSlotOwners,
                    suppressionSlotOwnerCount,
                    slot.ownerToken);
                if (slot.active) {
                    addUniqueOwner(
                        suppressionOwners,
                        suppressionOwnerCount,
                        slot.ownerToken);
                }
            }
            for (std::uint32_t index = 0;
                 index < suppressionSlotOwnerCount;
                 ++index) {
                clearHandInputSuppressionsForOwnerLocked(
                    suppressionSlotOwners[index],
                    RockProviderHand::None,
                    RockProviderSuppressionInvalidationReasonV1::ProviderLost,
                    true);
            }
        }
        {
            std::scoped_lock lock(s_weaponPartMutex);
            for (const auto& slot : s_weaponPartTargets) {
                if (slot.active) {
                    addUniqueOwner(
                        targetOwners,
                        targetOwnerCount,
                        slot.ownerToken);
                }
            }
            for (const auto& slot : s_weaponPartDrives) {
                if (slot.active) {
                    addUniqueOwner(
                        driveOwners,
                    driveOwnerCount,
                    slot.ownerToken);
                }
            }
            for (std::uint32_t index = 0; index < targetOwnerCount; ++index) {
                clearWeaponPartTargetsForOwnerLocked(targetOwners[index]);
            }
            for (std::uint32_t index = 0; index < driveOwnerCount; ++index) {
                clearWeaponPartDrivesForOwnerLocked(driveOwners[index]);
            }
        }
        {
            std::scoped_lock lock(s_nativeAnimationAuthorityMutex);
            for (const auto& slot : s_nativeAnimationAuthoritySlots) {
                if (slot.active) {
                    addUniqueOwner(
                        nativeAnimationOwners,
                        nativeAnimationOwnerCount,
                    slot.ownerToken);
                }
            }
            for (std::uint32_t index = 0;
                 index < nativeAnimationOwnerCount;
                 ++index) {
                clearNativeAnimationAuthorityForOwnerLocked(
                    nativeAnimationOwners[index],
                    false);
            }
            publishNativeAnimationAuthorityAggregateLocked();
        }
        {
            std::scoped_lock lock(s_handVisualAuthorityMutex);
            for (auto& slot : s_handVisualAuthoritySlots) {
                addUniqueOwner(
                    handVisualOwners,
                    handVisualOwnerCount,
                    slot.ownerToken);
                (void)clearHandVisualAuthoritySlotLocked(slot, true);
            }
        }
        {
            std::scoped_lock lock(s_nativeAnimationRuntimePublicationMutex);
            nativeRuntimeOwner = s_nativeAnimationRuntimeProviderOwner;
            clearNativeAnimationRuntimePublicationLocked();
        }
        {
            std::scoped_lock lock(s_equippedWeaponHandlingAuthorityMutex);
            equippedHandlingOwner =
                s_equippedWeaponHandlingAuthority.ownerToken;
            clearEquippedWeaponHandlingAuthorityForOwnerLocked(
                equippedHandlingOwner);
        }
        provider_debug_overlay::PruneResult overlayLost{};
        provider_debug_overlay::clearAll(
            overlayLost,
            RockProviderSuppressionInvalidationReasonV1::ProviderLost);
        provider_collider_visualization::Invalidation
            colliderVisualizationLost{};
        provider_collider_visualization::clearAll(
            colliderVisualizationLost,
            RockProviderSuppressionInvalidationReasonV1::ProviderLost);
        {
            std::scoped_lock lock(s_offhandReservationMutex);
            clearOffhandReservationLocked(
                RockProviderSuppressionInvalidationReasonV1::ProviderLost);
        }

        constexpr auto providerLost = static_cast<std::uint32_t>(
            RockProviderSuppressionInvalidationReasonV1::ProviderLost);
        for (std::uint32_t index = 0;
             index < suppressionOwnerCount;
             ++index) {
            publishAuthorityLostEvent(
                suppressionOwners[index],
                RockProviderAuthorityKindV1::HandInputSuppression,
                providerLost);
        }
        for (std::uint32_t index = 0; index < targetOwnerCount; ++index) {
            publishAuthorityLostEvent(
                targetOwners[index],
                RockProviderAuthorityKindV1::WeaponPartTargets,
                providerLost);
        }
        for (std::uint32_t index = 0; index < driveOwnerCount; ++index) {
            publishAuthorityLostEvent(
                driveOwners[index],
                RockProviderAuthorityKindV1::WeaponPartDrive,
                providerLost);
        }
        for (std::uint32_t index = 0;
             index < nativeAnimationOwnerCount;
             ++index) {
            publishAuthorityLostEvent(
                nativeAnimationOwners[index],
                RockProviderAuthorityKindV1::NativeAnimation,
                providerLost);
        }
        for (std::uint32_t index = 0;
             index < handVisualOwnerCount;
             ++index) {
            publishAuthorityLostEvent(
                handVisualOwners[index],
                RockProviderAuthorityKindV1::HandVisual,
                providerLost);
        }
        if (nativeRuntimeOwner != 0) {
            publishAuthorityLostEvent(
                nativeRuntimeOwner,
                RockProviderAuthorityKindV1::NativeAnimationRuntime,
                providerLost);
        }
        if (equippedHandlingOwner != 0) {
            publishAuthorityLostEvent(
                equippedHandlingOwner,
                RockProviderAuthorityKindV1::EquippedWeaponHandling,
                providerLost);
        }
        for (std::uint32_t index = 0; index < overlayLost.count; ++index) {
            publishAuthorityLostEvent(
                overlayLost.publishers[index].ownerToken,
                RockProviderAuthorityKindV1::DebugOverlay,
                providerLost);
        }
        if (colliderVisualizationLost.ownerToken != 0) {
            publishAuthorityLostEvent(
                colliderVisualizationLost.ownerToken,
                RockProviderAuthorityKindV1::ColliderVisualization,
                providerLost);
        }
        s_generationStateAvailable.store(false, std::memory_order_release);
        s_currentWorldGeneration.store(0, std::memory_order_release);
        s_currentSkeletonGeneration.store(0, std::memory_order_release);
        s_currentProviderGeneration.store(0, std::memory_order_release);
    }

}
