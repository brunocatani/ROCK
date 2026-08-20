#pragma once

#include "api/ProviderLeasePolicy.h"
#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/api/InteractionCommandPolicy.h"
#include "physics-interaction/api/InteractionCommandQueue.h"
#include "physics-interaction/object/ExternalBodyRegistry.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace rock::provider::detail
{
    struct CallbackSlot
    {
        std::uint64_t token{ 0 };
        std::uint64_t ownerToken{ 0 };
        RockProviderFrameCallback callback{ nullptr };
        void* userData{ nullptr };
    };

    struct ConsumerSlot
    {
        std::uint64_t token{ 0 };
        std::uint32_t grantedCapabilities{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint64_t worldRaycastFrameIndex{ 0 };
        std::uint32_t worldRaycastCount{ 0 };
        char modName[64]{};
    };

    struct InteractionCommandSlot
    {
        bool active{ false };
        QueuedInteractionCommandV1 command{};
    };

    struct InteractionCommandResultSlot
    {
        bool active{ false };
        RockProviderInteractionCommandResultV1 result{};
    };

    struct HandInputSuppressionSlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t flags{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        RockProviderSuppressionInvalidationReasonV1 lastInvalidationReason{
            RockProviderSuppressionInvalidationReasonV1::None
        };
        std::uint64_t lastInvalidatedFrame{ 0 };
    };

    struct NativeAnimationAuthoritySlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        std::uint32_t flags{ 0 };
        std::uint64_t expiresAtFrame{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    struct AnimationPhaseCallbackSlot
    {
        std::uint64_t token{ 0 };
        std::uint64_t ownerToken{ 0 };
        RockProviderAnimationPhaseCallbackV1 callback{ nullptr };
        void* userData{ nullptr };
    };

    struct HandVisualAuthoritySlot
    {
        std::uint64_t ownerToken{ 0 };
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t publishedFlags{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        char tag[64]{};
    };

    struct EquippedWeaponHandlingAuthoritySlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        RockProviderEquippedWeaponHandlingRequestV1 request{};
    };

    struct WeaponPartTargetSlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        RockProviderWeaponPartTargetV1 target{};
    };

    struct WeaponPartDriveSlot
    {
        bool active{ false };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        RockProviderWeaponPartDriveTargetV1 target{};
    };

    struct OffhandReservationSlot
    {
        std::uint64_t ownerToken{ 0 };
        RockProviderOffhandReservation reservation{
            RockProviderOffhandReservation::Normal
        };
        std::uint64_t expiresAfterFrame{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    // s_callbackMutex guards callback registration and dispatch copies.
    extern std::atomic<rock::PhysicsInteraction*> s_physicsInteraction;
    extern std::atomic<std::uint64_t> s_nextFrameIndex;
    extern std::atomic<std::uint64_t> s_nextCallbackToken;
    extern std::mutex s_callbackMutex;
    extern std::array<CallbackSlot, 16> s_callbacks;

    // s_snapshotMutex guards game-thread frame publications and readers.
    extern std::mutex s_snapshotMutex;
    extern RockProviderFrameSnapshot s_lastSnapshot;
    extern bool s_hasSnapshot;
    extern std::atomic<bool> s_generationStateAvailable;
    extern std::atomic<std::uint32_t> s_currentWorldGeneration;
    extern std::atomic<std::uint32_t> s_currentSkeletonGeneration;
    extern std::atomic<std::uint32_t> s_currentProviderGeneration;
    extern std::array<RockProviderWeaponPartGripStateV1, 2>
        s_lastPartGripStates;
    extern std::array<RockProviderHandInteractionStateV1, 2>
        s_lastHandInteractionStates;
    extern RockProviderEquippedWeaponStateV1 s_lastEquippedWeaponState;

    // s_consumerMutex guards owner registration and capability checks.
    extern std::mutex s_consumerMutex;
    extern std::array<ConsumerSlot, ROCK_PROVIDER_MAX_CONSUMERS_V1>
        s_consumers;
    extern std::atomic<std::uint64_t> s_nextConsumerTokenSequence;

    // s_interactionCommandMutex guards game-to-physics command state.
    extern std::mutex s_interactionCommandMutex;
    extern std::array<InteractionCommandSlot,
        ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1>
        s_interactionCommands;
    extern std::array<InteractionCommandResultSlot,
        ROCK_PROVIDER_MAX_COMPLETED_INTERACTION_COMMANDS_V1>
        s_interactionResults;
    extern std::size_t s_nextInteractionResultSlot;
    extern std::atomic<std::uint64_t> s_nextInteractionCommandId;
    extern interaction_command_policy::ForceGrabReservations
        s_forceGrabReservations;

    // s_providerEventMutex guards the bounded provider event ring.
    extern std::mutex s_providerEventMutex;
    extern std::array<RockProviderEventV1, ROCK_PROVIDER_MAX_PROVIDER_EVENTS_V1>
        s_providerEvents;
    extern std::uint32_t s_providerEventCount;
    extern std::uint32_t s_providerEventHead;
    extern std::uint64_t s_nextProviderEventSequence;
    extern std::uint64_t s_overwrittenProviderEventCount;

    // s_handInputSuppressionMutex guards owner lease publications.
    extern std::mutex s_handInputSuppressionMutex;
    extern std::array<HandInputSuppressionSlot,
        ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSIONS_V1>
        s_handInputSuppressions;

    // s_nativeAnimationAuthorityMutex guards selective animation leases.
    extern std::mutex s_nativeAnimationAuthorityMutex;
    extern std::array<NativeAnimationAuthoritySlot,
        ROCK_PROVIDER_MAX_CONSUMERS_V1>
        s_nativeAnimationAuthoritySlots;
    extern std::atomic<std::uint32_t> s_nativeAnimationAuthorityFlags;
    extern std::atomic<std::uint32_t> s_nativeAnimationAuthorityOwnerCount;

    // s_animationPhaseCallbackMutex guards animation-thread callbacks.
    extern std::mutex s_animationPhaseCallbackMutex;
    extern std::array<AnimationPhaseCallbackSlot,
        ROCK_PROVIDER_MAX_ANIMATION_PHASE_CALLBACKS_V1>
        s_animationPhaseCallbacks;
    extern std::atomic<std::uint64_t> s_nextAnimationPhaseCallbackToken;
    extern std::atomic<std::uint64_t> s_nextAnimationPhaseFrameIndex;
    extern std::atomic<std::uint64_t> s_activeAnimationPhaseFrameIndex;
    extern std::atomic<std::uint32_t> s_animationOwnerThreadId;
    extern std::atomic<bool> s_animationThreadMismatchLogged;

    // s_handVisualAuthorityMutex guards hFRIK hand publications.
    extern std::mutex s_handVisualAuthorityMutex;
    extern std::array<HandVisualAuthoritySlot,
        ROCK_PROVIDER_MAX_CONSUMERS_V1 * 2>
        s_handVisualAuthoritySlots;

    // s_nativeAnimationRuntimePublicationMutex guards one runtime provider.
    extern std::mutex s_nativeAnimationRuntimePublicationMutex;
    extern std::uint64_t s_nativeAnimationRuntimeProviderOwner;
    extern RockProviderNativeAnimationRuntimePublicationV1
        s_nativeAnimationRuntimePublication;
    extern bool s_hasNativeAnimationRuntimePublication;
    extern std::uint64_t s_nativeAnimationRuntimeExpiresAfterFrame;

    // s_equippedWeaponHandlingAuthorityMutex guards one policy owner.
    extern std::mutex s_equippedWeaponHandlingAuthorityMutex;
    extern EquippedWeaponHandlingAuthoritySlot
        s_equippedWeaponHandlingAuthority;

    // s_weaponPartMutex guards target and drive publications.
    extern std::mutex s_weaponPartMutex;
    extern std::array<WeaponPartTargetSlot,
        ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1>
        s_weaponPartTargets;
    extern std::array<WeaponPartDriveSlot,
        ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1>
        s_weaponPartDrives;

    // s_externalBodyMutex guards game and physics-thread body evidence.
    extern std::mutex s_externalBodyMutex;
    extern ExternalBodyRegistry s_externalBodies;

    // s_touchGrabMutex guards game and physics-thread touch targets.
    extern std::mutex s_touchGrabMutex;
    extern TouchGrabRegistry s_touchGrabTargets;

    // s_offhandReservationMutex guards the owner lease and atomic mirror.
    extern std::mutex s_offhandReservationMutex;
    extern OffhandReservationSlot s_offhandReservationSlot;
    extern std::atomic<std::uint64_t> s_offhandReservationOwner;
    extern std::atomic<std::uint64_t> s_offhandReservationExpiry;
    extern std::atomic<std::uint32_t> s_offhandReservation;
    extern std::atomic<bool> s_equippedWeaponFiringHandIsLeft;

    enum class EntryThreadPolicy
    {
        AnyThread,
        AnimationOwner,
    };

    struct EntryValidationChecks
    {
        bool threadValid{ true };
        bool argumentPresent{ true };
        bool sizeValid{ true };
        bool versionValid{ true };
        bool semanticValid{ true };
    };

    [[nodiscard]] constexpr RockProviderResultV1 validateEntryOrder(
        const EntryValidationChecks& checks)
    {
        if (!checks.threadValid) {
            return RockProviderResultV1::WrongThread;
        }
        if (!checks.argumentPresent) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!checks.sizeValid) {
            return RockProviderResultV1::InvalidSize;
        }
        if (!checks.versionValid) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (!checks.semanticValid) {
            return RockProviderResultV1::InvalidArgument;
        }
        return RockProviderResultV1::Ok;
    }

    [[nodiscard]] bool entryThreadValid(EntryThreadPolicy policy);

    template <class Entry, class SemanticValidator>
    [[nodiscard]] RockProviderResultV1 validateEntry(
        const Entry* entry,
        const EntryThreadPolicy threadPolicy,
        SemanticValidator&& semanticValidator)
    {
        if (!entryThreadValid(threadPolicy)) {
            return RockProviderResultV1::WrongThread;
        }
        if (!entry) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto structuralResult = validateEntryOrder(
            EntryValidationChecks{
                .sizeValid = entry->size == sizeof(Entry),
                .versionValid = entry->version != 0 &&
                                entry->version <= ROCK_PROVIDER_API_VERSION,
            });
        if (structuralResult != RockProviderResultV1::Ok) {
            return structuralResult;
        }
        return semanticValidator(*entry) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::InvalidArgument;
    }

    template <class Entry>
    [[nodiscard]] RockProviderResultV1 validateEntry(
        const Entry* entry,
        const EntryThreadPolicy threadPolicy = EntryThreadPolicy::AnyThread)
    {
        return validateEntry(
            entry,
            threadPolicy,
            [](const Entry&) { return true; });
    }

    [[nodiscard]] inline RockProviderResultV1 validateOwnerEntry(
        const std::uint64_t ownerToken,
        const EntryThreadPolicy threadPolicy = EntryThreadPolicy::AnyThread)
    {
        return validateEntryOrder(EntryValidationChecks{
            .threadValid = entryThreadValid(threadPolicy),
            .semanticValid = ownerToken != 0,
        });
    }

    template <class Output>
    [[nodiscard]] RockProviderResultV1 validateOutputEntry(
        const std::uint64_t ownerToken,
        const Output* output,
        const EntryThreadPolicy threadPolicy = EntryThreadPolicy::AnyThread)
    {
        if (!entryThreadValid(threadPolicy)) {
            return RockProviderResultV1::WrongThread;
        }
        if (!output) {
            return RockProviderResultV1::InvalidArgument;
        }
        return validateEntryOrder(EntryValidationChecks{
            .sizeValid = output->size >= sizeof(Output),
            .semanticValid = ownerToken != 0,
        });
    }

    template <class Value>
    [[nodiscard]] RockProviderResultV1 validateArrayEntry(
        const std::uint64_t ownerToken,
        const Value* values,
        const std::uint32_t maxValues,
        const std::uint32_t* outValueCount,
        const EntryThreadPolicy threadPolicy = EntryThreadPolicy::AnyThread,
        const bool semanticValid = true)
    {
        return validateEntryOrder(EntryValidationChecks{
            .threadValid = entryThreadValid(threadPolicy),
            .argumentPresent = outValueCount != nullptr &&
                               (maxValues == 0 || values != nullptr),
            .semanticValid = ownerToken != 0 && semanticValid,
        });
    }

    template <class IsActive, class GenerationChanged, class ExpiresAfter,
              class Revoke>
    [[nodiscard]] bool pruneExpiredSlots(
        const std::size_t slotCount,
        const std::uint64_t frameIndex,
        IsActive&& isActive,
        GenerationChanged&& generationChanged,
        ExpiresAfter&& expiresAfter,
        Revoke&& revoke)
    {
        bool changed = false;
        for (std::size_t index = 0; index < slotCount; ++index) {
            if (!isActive(index)) {
                continue;
            }

            const bool staleGeneration = generationChanged(index);
            if (!staleGeneration && provider_lease_policy::isActive(
                    frameIndex,
                    expiresAfter(index))) {
                continue;
            }

            const auto reason = staleGeneration ?
                RockProviderSuppressionInvalidationReasonV1::GenerationChanged :
                RockProviderSuppressionInvalidationReasonV1::Expired;
            revoke(index, reason);
            changed = true;
        }
        return changed;
    }

    enum class RevokeReason
    {
        CallbackFault,
        OwnerUnregistered,
    };

    ConsumerSlot* findConsumerSlotLocked(std::uint64_t ownerToken);
    RockProviderResultV1 validateRegisteredOwnerCapabilityLocked(
        std::uint64_t ownerToken,
        RockProviderConsumerCapabilityV1 capability);
    RockProviderResultV1 validateGenerationGuards(
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration);
    RockProviderResultV1 validateReadCapability(
        std::uint64_t ownerToken,
        RockProviderConsumerCapabilityV1 capability);
    [[nodiscard]] bool generationGuardsStale(
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration);
    std::uint64_t nextConsumerToken();
    std::uint64_t currentProviderFrameIndex();
    void publishProviderEvent(RockProviderEventV1 event);
    void publishAuthorityLostEvent(
        std::uint64_t ownerToken,
        RockProviderAuthorityKindV1 authorityKind,
        std::uint32_t reason);
    RockProviderResultV1 revokeOwner(
        std::uint64_t ownerToken,
        RevokeReason revokeReason,
        bool alsoUnregister);
}
