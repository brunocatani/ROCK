#define ROCK_API_EXPORTS
/*
 * Build the DLL side of the public provider ABI.
 *
 * OWNER LIFECYCLE: one revocation path for every way an owner can go away.
 *
 * revokeOwner is the single implementation. apiUnregisterConsumerV1 and
 * clearOwnerStateAfterCallbackFault are thin wrappers over it, and they differ only
 * in the invalidation reason and whether the consumer slot is cleared too.
 *
 * LOCK ORDER. revokeOwner takes six mutexes in ONE std::scoped_lock. That single
 * simultaneous acquisition is what makes it deadlock-safe. Never split it into
 * sequential locks, and never move part of it into another function.
 *
 * clearExternalBodiesForProviderLoss below takes its per-family locks one at a time,
 * in a fixed order. That order is load-bearing. Keep it.
 */
#include "ROCKProviderApiInternal.h"
#include "api/ProviderColliderVisualizationRuntime.h"
#include "api/ProviderDebugOverlayRuntime.h"
#include "api/TouchGrabRegistry.h"
#include "api/detail/ProviderApiEntryPoints.h"
#include "api/detail/ProviderAnimationAuthority.h"
#include "api/detail/ProviderApiState.h"
#include "api/detail/ProviderAuthorityLeases.h"
#include "api/detail/ProviderInteractionCommands.h"
#include "api/detail/ProviderOwnerLifecycle.h"
#include "api/detail/ProviderWeaponParts.h"

#include <array>
#include <atomic>
#include <mutex>

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

}

namespace rock::provider
{
    using namespace detail;

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
