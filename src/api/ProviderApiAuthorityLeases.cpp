#define ROCK_API_EXPORTS
/*
 * Build the DLL side of the public provider ABI.
 *
 * AUTHORITY LEASES: hand input suppression, equipped-weapon handling authority and
 * its hand request, and the offhand reservation.
 *
 * Every family expires the same way, through the shared prune machinery in
 * ProviderLeasePolicy.h. A lease that outlives its frame budget is a bug, so a
 * family must never grow its own expiry rule.
 *
 * Entry points that take both a consumer token and a family slot lock both mutexes
 * in one scoped_lock. Keep every one of those a single acquisition.
 */
#include "api/detail/ProviderAuthorityLeases.h"

#include "api/detail/ProviderApiCore.h"
#include "api/detail/ProviderApiEntryPoints.h"

#include <cmath>
#include <mutex>

#include "physics-interaction/core/PhysicsInteraction.h"

namespace rock::provider::detail
{
    using namespace rock;

    constexpr std::uint32_t kImplementedHandInputSuppressionFlagsV1 =
        static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressConfigModeChord) |
        static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressOpenVrGameInput) |
        static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressNativeVats) |
        static_cast<std::uint32_t>(RockProviderHandInputSuppressionFlagV1::SuppressNativeVans);

    void clearEquippedWeaponHandlingAuthorityForOwnerLocked(
        const std::uint64_t ownerToken)
    {
        if (s_equippedWeaponHandlingAuthority.active &&
            s_equippedWeaponHandlingAuthority.ownerToken == ownerToken) {
            s_equippedWeaponHandlingAuthority = {};
        }
    }

    void pruneExpiredEquippedWeaponHandlingAuthorityLocked(
        const std::uint64_t frameIndex)
    {
        (void)pruneExpiredSlots(
            1,
            frameIndex,
            [](const std::size_t) {
                return s_equippedWeaponHandlingAuthority.active;
            },
            [](const std::size_t) {
                return generationGuardsStale(
                    s_equippedWeaponHandlingAuthority.request.worldGeneration,
                    s_equippedWeaponHandlingAuthority.request.skeletonGeneration,
                    s_equippedWeaponHandlingAuthority.request.providerGeneration);
            },
            [](const std::size_t) {
                return s_equippedWeaponHandlingAuthority.expiresAfterFrame;
            },
            [](const std::size_t, const auto reason) {
                publishAuthorityLostEvent(
                    s_equippedWeaponHandlingAuthority.ownerToken,
                    RockProviderAuthorityKindV1::EquippedWeaponHandling,
                    static_cast<std::uint32_t>(reason));
                s_equippedWeaponHandlingAuthority = {};
            });
    }


    void publishOffhandReservationLocked(const OffhandReservationSlot& slot)
    {
        s_offhandReservationOwner.store(
            slot.ownerToken,
            std::memory_order_release);
        s_offhandReservation.store(
            static_cast<std::uint32_t>(slot.reservation),
            std::memory_order_release);
        s_offhandReservationExpiry.store(
            slot.expiresAfterFrame,
            std::memory_order_release);
    }

    void clearOffhandReservationLocked(
        const RockProviderSuppressionInvalidationReasonV1 reason)
    {
        const auto ownerToken = s_offhandReservationSlot.ownerToken;
        s_offhandReservationSlot = {};
        publishOffhandReservationLocked(s_offhandReservationSlot);
        if (ownerToken != 0) {
            publishAuthorityLostEvent(
                ownerToken,
                RockProviderAuthorityKindV1::OffhandReservation,
                static_cast<std::uint32_t>(reason));
        }
    }

    void pruneExpiredOffhandReservationLocked(const std::uint64_t frameIndex)
    {
        (void)pruneExpiredSlots(
            1,
            frameIndex,
            [](const std::size_t) {
                return s_offhandReservationSlot.ownerToken != 0;
            },
            [](const std::size_t) {
                return generationGuardsStale(
                    s_offhandReservationSlot.worldGeneration,
                    s_offhandReservationSlot.skeletonGeneration,
                    s_offhandReservationSlot.providerGeneration);
            },
            [](const std::size_t) {
                return s_offhandReservationSlot.expiresAfterFrame;
            },
            [](const std::size_t, const auto reason) {
                clearOffhandReservationLocked(reason);
            });
    }

    void pruneExpiredHandInputSuppressionsLocked(std::uint64_t frameIndex)
    {
        (void)pruneExpiredSlots(
            s_handInputSuppressions.size(),
            frameIndex,
            [](const std::size_t index) {
                return s_handInputSuppressions[index].active;
            },
            [](const std::size_t index) {
                const auto& slot = s_handInputSuppressions[index];
                return generationGuardsStale(
                    slot.worldGeneration,
                    slot.skeletonGeneration,
                    slot.providerGeneration);
            },
            [](const std::size_t index) {
                return s_handInputSuppressions[index].expiresAfterFrame;
            },
            [frameIndex](const std::size_t index, const auto reason) {
                auto& slot = s_handInputSuppressions[index];
                publishAuthorityLostEvent(
                    slot.ownerToken,
                    RockProviderAuthorityKindV1::HandInputSuppression,
                    static_cast<std::uint32_t>(reason));
                slot.active = false;
                slot.flags = 0;
                slot.lastInvalidationReason = reason;
                slot.lastInvalidatedFrame = frameIndex;
            });
    }

    void clearHandInputSuppressionsForOwnerLocked(
        std::uint64_t ownerToken,
        RockProviderHand hand,
        RockProviderSuppressionInvalidationReasonV1 reason =
            RockProviderSuppressionInvalidationReasonV1::ExplicitClear,
        const bool includeInactive)
    {
        for (auto& slot : s_handInputSuppressions) {
            if (slot.ownerToken != ownerToken ||
                (!slot.active && !includeInactive)) {
                continue;
            }
            if (hand == RockProviderHand::None || slot.hand == hand) {
                slot.active = false;
                slot.flags = 0;
                slot.lastInvalidationReason = reason;
                slot.lastInvalidatedFrame = currentProviderFrameIndex();
            }
        }
    }


    RockProviderResultV1 validateGenerationGuards(
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration)
    {
        if (worldGeneration == 0 && skeletonGeneration == 0 && providerGeneration == 0) {
            return RockProviderResultV1::Ok;
        }

        if (!s_generationStateAvailable.load(std::memory_order_acquire)) {
            return RockProviderResultV1::NotReady;
        }
        if (worldGeneration != 0 &&
            worldGeneration != s_currentWorldGeneration.load(
                std::memory_order_acquire)) {
            return RockProviderResultV1::WorldNotReady;
        }
        if (skeletonGeneration != 0 &&
            skeletonGeneration != s_currentSkeletonGeneration.load(
                std::memory_order_acquire)) {
            return RockProviderResultV1::NotReady;
        }
        if (providerGeneration != 0 &&
            providerGeneration != s_currentProviderGeneration.load(
                std::memory_order_acquire)) {
            return RockProviderResultV1::NotReady;
        }
        return RockProviderResultV1::Ok;
    }


    bool generationGuardsStale(
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration)
    {
        return validateGenerationGuards(
                   worldGeneration,
                   skeletonGeneration,
                   providerGeneration) != RockProviderResultV1::Ok;
    }


    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetHandInputSuppressionV1(
        std::uint64_t ownerToken,
        const RockProviderHandInputSuppressionRequestV1* request)
    {
        const auto entryResult = validateEntry(request);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if (request->flags == 0 || (request->flags & ~kImplementedHandInputSuppressionFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }

        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request->leaseFrames,
            ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSION_LEASE_FRAMES_V1);
        const auto frameIndex = currentProviderFrameIndex();
        const auto expiresAfterFrame =
            provider_lease_policy::exclusiveExpiryFrame(frameIndex, leaseFrames);

        std::scoped_lock lock(s_consumerMutex, s_handInputSuppressionMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::HandInputSuppression);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        pruneExpiredHandInputSuppressionsLocked(frameIndex);
        for (auto& slot : s_handInputSuppressions) {
            if (slot.active && slot.ownerToken == ownerToken && slot.hand == request->hand) {
                slot.flags = request->flags;
                slot.expiresAfterFrame = expiresAfterFrame;
                slot.worldGeneration = request->worldGeneration;
                slot.skeletonGeneration = request->skeletonGeneration;
                slot.providerGeneration = request->providerGeneration;
                slot.lastInvalidationReason =
                    RockProviderSuppressionInvalidationReasonV1::None;
                slot.lastInvalidatedFrame = 0;
                return RockProviderResultV1::Ok;
            }
        }

        for (auto& slot : s_handInputSuppressions) {
            if (!slot.active) {
                slot = HandInputSuppressionSlot{
                    .active = true,
                    .ownerToken = ownerToken,
                    .hand = request->hand,
                    .flags = request->flags,
                    .expiresAfterFrame = expiresAfterFrame,
                    .worldGeneration = request->worldGeneration,
                    .skeletonGeneration = request->skeletonGeneration,
                    .providerGeneration = request->providerGeneration,
                    .lastInvalidationReason =
                        RockProviderSuppressionInvalidationReasonV1::None,
                    .lastInvalidatedFrame = 0,
                };
                return RockProviderResultV1::Ok;
            }
        }

        return RockProviderResultV1::CapacityFull;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearHandInputSuppressionV1(
        std::uint64_t ownerToken,
        RockProviderHand hand)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (hand != RockProviderHand::None && hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }

        std::scoped_lock lock(s_consumerMutex, s_handInputSuppressionMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::HandInputSuppression);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        clearHandInputSuppressionsForOwnerLocked(ownerToken, hand);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetHandInputSuppressionStateV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        RockProviderHandInputSuppressionStateV1* outState)
    {
        const auto entryResult = validateOutputEntry(ownerToken, outState);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_handInputSuppressionMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::InputObservability);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        pruneExpiredHandInputSuppressionsLocked(frameIndex);

        *outState = {};
        outState->frameIndex = frameIndex;
        outState->hand = hand;
        for (const auto& slot : s_handInputSuppressions) {
            if (slot.hand != hand) {
                continue;
            }
            if (slot.active) {
                outState->effectiveFlags |= slot.flags;
            }
            if (slot.ownerToken != ownerToken) {
                continue;
            }
            if (slot.active) {
                outState->callerFlags = slot.flags;
                outState->callerLeaseActive = 1;
                outState->callerExpiresAfterFrame = slot.expiresAfterFrame;
                outState->callerRemainingFrames =
                    provider_lease_policy::remainingFrames(
                        frameIndex,
                        slot.expiresAfterFrame);
            } else if (outState->lastInvalidationReason ==
                       RockProviderSuppressionInvalidationReasonV1::None) {
                outState->lastInvalidationReason =
                    slot.lastInvalidationReason;
            }
        }
        {
            std::scoped_lock snapshotLock(s_snapshotMutex);
            if (s_hasSnapshot) {
                outState->worldGeneration = s_lastSnapshot.worldGeneration;
                outState->skeletonGeneration = s_lastSnapshot.skeletonGeneration;
                outState->providerGeneration = s_lastSnapshot.providerGeneration;
            }
        }
        return RockProviderResultV1::Ok;
    }


    [[nodiscard]] bool equippedWeaponHandlingRequestValuesValid(
        const RockProviderEquippedWeaponHandlingRequestV1& request)
    {
        const auto inRange = [](const float value, const float minimum, const float maximum) {
            return std::isfinite(value) && value >= minimum && value <= maximum;
        };
        return inRange(request.gripZoneEquipRadiusGameUnits, 0.25f, 30.0f) &&
               inRange(request.gripZoneEquipSettleSeconds, 0.0f, 5.0f) &&
               inRange(request.firingGripReattachRadiusGameUnits, 0.25f, 30.0f) &&
               inRange(request.gripZoneHoverHapticIntensity, 0.0f, 1.0f) &&
               inRange(request.firingGripProximitySupportRadiusGameUnits, 0.25f, 30.0f) &&
               inRange(request.weaponGripHapticDurationSeconds, 0.01f, 0.50f) &&
               inRange(request.firingGripAttachHapticIntensity, 0.0f, 1.0f) &&
               inRange(request.firingGripDetachHapticIntensity, 0.0f, 1.0f) &&
               inRange(request.supportGripHapticIntensity, 0.0f, 1.0f) &&
               inRange(request.firingGripPromotionRadiusGameUnits, 0.25f, 30.0f) &&
               inRange(request.leftFiringAimYawDegrees, -30.0f, 30.0f) &&
               inRange(request.leftFiringAimPitchDegrees, -30.0f, 30.0f) &&
               inRange(request.leftFiringAimOffsetGameUnits[0], -15.0f, 15.0f) &&
               inRange(request.leftFiringAimOffsetGameUnits[1], -15.0f, 15.0f) &&
               inRange(request.leftFiringAimOffsetGameUnits[2], -15.0f, 15.0f) &&
               inRange(request.equipVisualBridgeTimeoutSeconds, 0.25f, 5.0f) &&
               inRange(request.equipVisualBridgeBlendSeconds, 0.0f, 1.0f);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetEquippedWeaponHandlingAuthorityV1(
        const std::uint64_t ownerToken,
        const RockProviderEquippedWeaponHandlingRequestV1* request)
    {
        const auto entryResult = validateEntry(request);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        constexpr std::uint32_t implementedFlags =
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::AmbidextrousHandoff) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::GripZoneEquip) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::GripZoneHoverHaptics) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::FiringGripProximitySupport) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::EquippedWeaponShoulderStash) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::PipboyTriggerHandEquip) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::EquipVisualBridge);
        const auto ownershipFlag = static_cast<std::uint32_t>(
            RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership);
        const auto ownershipDependentFlags =
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::AmbidextrousHandoff) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::GripZoneEquip) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::EquippedWeaponShoulderStash) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::PipboyTriggerHandEquip);
        const auto primaryDetachFlag = static_cast<std::uint32_t>(
            RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach);
        const auto shoulderStashFlag = static_cast<std::uint32_t>(
            RockProviderEquippedWeaponHandlingFlagV1::EquippedWeaponShoulderStash);
        if (request->flags == 0 ||
            (request->flags & ~implementedFlags) != 0 ||
            request->leaseFrames == 0 ||
            ((request->flags & ownershipDependentFlags) != 0 &&
                (request->flags & ownershipFlag) == 0) ||
            ((request->flags & shoulderStashFlag) != 0 &&
                (request->flags & primaryDetachFlag) == 0) ||
            !equippedWeaponHandlingRequestValuesValid(*request)) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(
            s_consumerMutex,
            s_equippedWeaponHandlingAuthorityMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::EquippedWeaponHandlingAuthority);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        pruneExpiredEquippedWeaponHandlingAuthorityLocked(frameIndex);
        if (s_equippedWeaponHandlingAuthority.active &&
            s_equippedWeaponHandlingAuthority.ownerToken != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }

        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request->leaseFrames,
            ROCK_PROVIDER_MAX_EQUIPPED_WEAPON_HANDLING_LEASE_FRAMES_V1);
        s_equippedWeaponHandlingAuthority.active = true;
        s_equippedWeaponHandlingAuthority.ownerToken = ownerToken;
        s_equippedWeaponHandlingAuthority.expiresAfterFrame =
            provider_lease_policy::exclusiveExpiryFrame(
                frameIndex,
                leaseFrames);
        s_equippedWeaponHandlingAuthority.request = *request;
        s_equippedWeaponHandlingAuthority.request.size =
            sizeof(RockProviderEquippedWeaponHandlingRequestV1);
        s_equippedWeaponHandlingAuthority.request.version =
            ROCK_PROVIDER_API_VERSION;
        s_equippedWeaponHandlingAuthority.request.leaseFrames = leaseFrames;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearEquippedWeaponHandlingAuthorityV1(
        const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(
            s_consumerMutex,
            s_equippedWeaponHandlingAuthorityMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::EquippedWeaponHandlingAuthority);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (s_equippedWeaponHandlingAuthority.active &&
            s_equippedWeaponHandlingAuthority.ownerToken != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }
        clearEquippedWeaponHandlingAuthorityForOwnerLocked(ownerToken);
        return RockProviderResultV1::Ok;
    }

    bool ROCK_PROVIDER_CALL apiGetEquippedWeaponHandlingStateV1(
        RockProviderEquippedWeaponHandlingStateV1* outState)
    {
        if (!outState ||
            outState->size != sizeof(RockProviderEquippedWeaponHandlingStateV1)) {
            return false;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized() ||
            !pi->queryProviderEquippedWeaponHandlingStateV1(*outState)) {
            return false;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_equippedWeaponHandlingAuthorityMutex);
        pruneExpiredEquippedWeaponHandlingAuthorityLocked(frameIndex);
        if (s_equippedWeaponHandlingAuthority.active) {
            outState->authorityFlags =
                s_equippedWeaponHandlingAuthority.request.flags;
            outState->ownerToken =
                s_equippedWeaponHandlingAuthority.ownerToken;
            outState->expiresAfterFrame =
                s_equippedWeaponHandlingAuthority.expiresAfterFrame;
            outState->runtimeFlags |= static_cast<std::uint32_t>(
                RockProviderEquippedWeaponHandlingRuntimeFlagV1::AuthorityActive);
        }
        return true;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
    apiRequestEquippedWeaponHandV1(
        const std::uint64_t ownerToken,
        const RockProviderEquippedWeaponHandRequestV1* request)
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
        if (request->hand != RockProviderHand::Right &&
            request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if (request->flags != 0) {
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

        {
            const auto frameIndex = currentProviderFrameIndex();
            std::scoped_lock lock(
                s_consumerMutex,
                s_equippedWeaponHandlingAuthorityMutex);
            const auto ownerResult =
                validateRegisteredOwnerCapabilityLocked(
                    ownerToken,
                    RockProviderConsumerCapabilityV1::
                        EquippedWeaponHandlingAuthority);
            if (ownerResult != RockProviderResultV1::Ok) {
                return ownerResult;
            }
            pruneExpiredEquippedWeaponHandlingAuthorityLocked(
                frameIndex);
            if (!s_equippedWeaponHandlingAuthority.active) {
                return RockProviderResultV1::PermissionDenied;
            }
            if (s_equippedWeaponHandlingAuthority.ownerToken !=
                ownerToken) {
                return RockProviderResultV1::OwnerConflict;
            }

            std::uint32_t requiredFlags =
                static_cast<std::uint32_t>(
                    RockProviderEquippedWeaponHandlingFlagV1::
                        FiringGripOwnership);
            if (request->hand == RockProviderHand::Left) {
                requiredFlags |= static_cast<std::uint32_t>(
                    RockProviderEquippedWeaponHandlingFlagV1::
                        AmbidextrousHandoff);
            }
            if ((s_equippedWeaponHandlingAuthority.request.flags &
                    requiredFlags) != requiredFlags) {
                return RockProviderResultV1::PermissionDenied;
            }
        }

        auto* pi =
            s_physicsInteraction.load(std::memory_order_acquire);
        return pi && pi->isInitialized() ?
            pi->requestProviderEquippedWeaponHandV1(
                ownerToken,
                *request) :
            RockProviderResultV1::NotReady;
    }


    bool ROCK_PROVIDER_CALL apiSetOffhandInteractionReservation(std::uint64_t ownerToken, RockProviderOffhandReservation reservation)
    {
        if (ownerToken == 0 ||
            (reservation != RockProviderOffhandReservation::Normal &&
                reservation != RockProviderOffhandReservation::ReloadReserved &&
                reservation != RockProviderOffhandReservation::ReloadPoseOverride)) {
            return false;
        }
        std::scoped_lock lock(s_consumerMutex, s_offhandReservationMutex);
        if (!consumerHasCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::OffhandReservation)) {
            return false;
        }
        pruneExpiredOffhandReservationLocked(currentProviderFrameIndex());
        if (reservation == RockProviderOffhandReservation::Normal) {
            if (s_offhandReservationSlot.ownerToken != 0 &&
                s_offhandReservationSlot.ownerToken != ownerToken) {
                return false;
            }
            clearOffhandReservationLocked(
                RockProviderSuppressionInvalidationReasonV1::ExplicitClear);
            return true;
        }
        if (s_offhandReservationSlot.ownerToken != 0 &&
            s_offhandReservationSlot.ownerToken != ownerToken) {
            return false;
        }
        s_offhandReservationSlot = OffhandReservationSlot{
            .ownerToken = ownerToken,
            .reservation = reservation,
            .expiresAfterFrame = provider_lease_policy::exclusiveExpiryFrame(
                currentProviderFrameIndex(),
                ROCK_PROVIDER_MAX_OFFHAND_RESERVATION_LEASE_FRAMES_V1),
        };
        publishOffhandReservationLocked(s_offhandReservationSlot);
        return true;
    }

    [[nodiscard]] bool validOffhandReservationRequest(
        const RockProviderOffhandReservationRequestV1& request)
    {
        return request.reservation != RockProviderOffhandReservation::Normal &&
               (request.reservation == RockProviderOffhandReservation::ReloadReserved ||
                   request.reservation == RockProviderOffhandReservation::ReloadPoseOverride) &&
               request.leaseFrames != 0;
    }

    RockProviderResultV1 setOffhandReservationLeaseV1(
        const std::uint64_t ownerToken,
        const RockProviderOffhandReservationRequestV1* request,
        const bool requireExisting)
    {
        const auto entryResult = validateEntry(request);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!validOffhandReservationRequest(*request)) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_offhandReservationMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::OffhandReservation);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        pruneExpiredOffhandReservationLocked(frameIndex);
        if (requireExisting &&
            s_offhandReservationSlot.ownerToken != ownerToken) {
            return RockProviderResultV1::TargetUnavailable;
        }
        if (s_offhandReservationSlot.ownerToken != 0 &&
            s_offhandReservationSlot.ownerToken != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }
        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request->leaseFrames,
            ROCK_PROVIDER_MAX_OFFHAND_RESERVATION_LEASE_FRAMES_V1);
        s_offhandReservationSlot = OffhandReservationSlot{
            .ownerToken = ownerToken,
            .reservation = request->reservation,
            .expiresAfterFrame = provider_lease_policy::exclusiveExpiryFrame(
                frameIndex,
                leaseFrames),
            .worldGeneration = request->worldGeneration,
            .skeletonGeneration = request->skeletonGeneration,
            .providerGeneration = request->providerGeneration,
        };
        publishOffhandReservationLocked(s_offhandReservationSlot);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiAcquireOffhandReservationV1(
        const std::uint64_t ownerToken,
        const RockProviderOffhandReservationRequestV1* request)
    {
        return setOffhandReservationLeaseV1(ownerToken, request, false);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRenewOffhandReservationV1(
        const std::uint64_t ownerToken,
        const RockProviderOffhandReservationRequestV1* request)
    {
        return setOffhandReservationLeaseV1(ownerToken, request, true);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiReleaseOffhandReservationV1(
        const std::uint64_t ownerToken)
    {
        const auto entryResult = validateOwnerEntry(ownerToken);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        std::scoped_lock lock(s_consumerMutex, s_offhandReservationMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::OffhandReservation);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        pruneExpiredOffhandReservationLocked(currentProviderFrameIndex());
        if (s_offhandReservationSlot.ownerToken != 0 &&
            s_offhandReservationSlot.ownerToken != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }
        clearOffhandReservationLocked(
            RockProviderSuppressionInvalidationReasonV1::ExplicitClear);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetOffhandReservationStateV1(
        const std::uint64_t ownerToken,
        RockProviderOffhandReservationStateV1* outState)
    {
        const auto entryResult = validateOutputEntry(ownerToken, outState);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_offhandReservationMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::OffhandReservation);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        pruneExpiredOffhandReservationLocked(frameIndex);
        *outState = {};
        outState->reservation = s_offhandReservationSlot.reservation;
        outState->active = s_offhandReservationSlot.ownerToken != 0 ? 1u : 0u;
        outState->ownerToken = s_offhandReservationSlot.ownerToken;
        outState->expiresAfterFrame =
            s_offhandReservationSlot.expiresAfterFrame;
        outState->remainingFrames = provider_lease_policy::remainingFrames(
            frameIndex,
            outState->expiresAfterFrame);
        {
            std::scoped_lock snapshotLock(s_snapshotMutex);
            if (s_hasSnapshot) {
                outState->worldGeneration = s_lastSnapshot.worldGeneration;
                outState->skeletonGeneration =
                    s_lastSnapshot.skeletonGeneration;
                outState->providerGeneration =
                    s_lastSnapshot.providerGeneration;
            }
        }
        return RockProviderResultV1::Ok;
    }
}

namespace rock::provider
{
    using namespace detail;

    RockProviderOffhandReservation currentOffhandReservation()
    {
        const auto expiry =
            s_offhandReservationExpiry.load(std::memory_order_acquire);
        if (expiry != 0 && !provider_lease_policy::isActive(
                currentProviderFrameIndex(),
                expiry)) {
            return RockProviderOffhandReservation::Normal;
        }
        return static_cast<RockProviderOffhandReservation>(s_offhandReservation.load(std::memory_order_acquire));
    }

    void setEquippedWeaponFiringHandIsLeft(const bool isLeft)
    {
        s_equippedWeaponFiringHandIsLeft.store(isLeft, std::memory_order_release);
    }

    bool getEquippedWeaponHandlingAuthorityV1(
        RockProviderEquippedWeaponHandlingRequestV1& outRequest)
    {
        outRequest = {};
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_equippedWeaponHandlingAuthorityMutex);
        pruneExpiredEquippedWeaponHandlingAuthorityLocked(frameIndex);
        if (!s_equippedWeaponHandlingAuthority.active) {
            return false;
        }
        outRequest = s_equippedWeaponHandlingAuthority.request;
        return true;
    }

    bool ownsEquippedWeaponHandlingAuthorityV1(
        const std::uint64_t ownerToken,
        const std::uint32_t requiredFlags)
    {
        if (ownerToken == 0) {
            return false;
        }
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(
            s_equippedWeaponHandlingAuthorityMutex);
        pruneExpiredEquippedWeaponHandlingAuthorityLocked(
            frameIndex);
        return s_equippedWeaponHandlingAuthority.active &&
               s_equippedWeaponHandlingAuthority.ownerToken ==
                   ownerToken &&
               (s_equippedWeaponHandlingAuthority.request.flags &
                   requiredFlags) == requiredFlags;
    }

    std::uint32_t currentHandInputSuppressionFlagsV1(RockProviderHand hand)
    {
        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return 0;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::uint32_t flags = 0;
        std::scoped_lock lock(s_handInputSuppressionMutex);
        pruneExpiredHandInputSuppressionsLocked(frameIndex);
        for (const auto& slot : s_handInputSuppressions) {
            if (slot.active && slot.hand == hand) {
                flags |= slot.flags;
            }
        }
        return flags;
    }

}
