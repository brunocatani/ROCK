#define ROCK_API_EXPORTS
// Build the DLL side of the public provider ABI.
#include "api/detail/ProviderApiEntryPoints.h"
#include "api/detail/ProviderApiCore.h"
#include "api/detail/ProviderApiState.h"
#include "api/detail/ProviderTransformMath.h"

#include "api/ProviderColliderVisualizationRuntime.h"
#include "api/ProviderDebugOverlayRuntime.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>

#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/InputRemapRuntime.h"

namespace rock::provider::detail
{
    using namespace rock;

    [[nodiscard]] const RockProviderEventV1& providerEventAtLocked(
        const std::uint32_t logicalIndex)
    {
        return s_providerEvents[
            (s_providerEventHead + logicalIndex) % s_providerEvents.size()];
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
