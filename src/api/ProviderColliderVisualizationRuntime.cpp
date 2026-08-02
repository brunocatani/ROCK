#include "api/ProviderColliderVisualizationRuntime.h"

#include "api/ProviderLeasePolicy.h"

#include <atomic>
#include <mutex>

namespace rock::provider_collider_visualization
{
    namespace
    {
        using namespace provider;

        struct OverrideSlot
        {
            Snapshot snapshot{};
            std::uint64_t expiresAfterFrame{ 0 };
            std::uint32_t worldGeneration{ 0 };
            std::uint32_t skeletonGeneration{ 0 };
            std::uint32_t providerGeneration{ 0 };
        };

        OverrideSlot s_slot{};
        std::mutex s_mutex;
        std::atomic_bool s_active{ false };

    }

    provider::RockProviderResultV1 set(
        const std::uint64_t ownerToken,
        const provider::RockProviderColliderVisualizationRequestV1& request,
        const std::uint64_t frameIndex)
    {
        using namespace provider;
        if (ownerToken == 0 || request.weaponGenerationKey == 0 ||
            request.bodyId == 0x7FFF'FFFF || request.leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        std::scoped_lock lock(s_mutex);
        if (s_slot.snapshot.ownerToken != 0 &&
            s_slot.snapshot.ownerToken != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }

        s_slot = {};
        s_slot.snapshot.ownerToken = ownerToken;
        s_slot.snapshot.weaponGenerationKey = request.weaponGenerationKey;
        s_slot.snapshot.bodyId = request.bodyId;
        s_slot.snapshot.partKind = request.partKind;
        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request.leaseFrames,
            ROCK_PROVIDER_MAX_COLLIDER_VISUALIZATION_OVERRIDE_LEASE_FRAMES_V1);
        s_slot.expiresAfterFrame =
            provider_lease_policy::exclusiveExpiryFrame(
                frameIndex,
                leaseFrames);
        s_slot.worldGeneration = request.worldGeneration;
        s_slot.skeletonGeneration = request.skeletonGeneration;
        s_slot.providerGeneration = request.providerGeneration;
        s_active.store(true, std::memory_order_release);
        return RockProviderResultV1::Ok;
    }

    void prune(
        const std::uint64_t frameIndex,
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration,
        const std::uint64_t weaponGenerationKey,
        const bool weaponBodyCurrent,
        Invalidation& outInvalidation)
    {
        outInvalidation = {};
        if (!s_active.load(std::memory_order_acquire)) {
            return;
        }

        std::scoped_lock lock(s_mutex);
        if (s_slot.snapshot.ownerToken == 0) {
            s_active.store(false, std::memory_order_release);
            return;
        }

        const bool generationChanged =
            (s_slot.worldGeneration != 0 &&
                s_slot.worldGeneration != worldGeneration) ||
            (s_slot.skeletonGeneration != 0 &&
                s_slot.skeletonGeneration != skeletonGeneration) ||
            (s_slot.providerGeneration != 0 &&
                s_slot.providerGeneration != providerGeneration) ||
            s_slot.snapshot.weaponGenerationKey != weaponGenerationKey ||
            !weaponBodyCurrent;
        const bool expired = !provider_lease_policy::isActive(
            frameIndex,
            s_slot.expiresAfterFrame);
        if (!generationChanged && !expired) {
            return;
        }

        outInvalidation.ownerToken = s_slot.snapshot.ownerToken;
        outInvalidation.reason = generationChanged ?
            RockProviderSuppressionInvalidationReasonV1::GenerationChanged :
            RockProviderSuppressionInvalidationReasonV1::Expired;
        s_slot = {};
        s_active.store(false, std::memory_order_release);
    }

    void clear(const std::uint64_t ownerToken)
    {
        if (ownerToken == 0 ||
            !s_active.load(std::memory_order_acquire)) {
            return;
        }
        std::scoped_lock lock(s_mutex);
        if (s_slot.snapshot.ownerToken != ownerToken) {
            return;
        }
        s_slot = {};
        s_active.store(false, std::memory_order_release);
    }

    void clearAll(
        Invalidation& outInvalidation,
        const provider::RockProviderSuppressionInvalidationReasonV1 reason)
    {
        outInvalidation = {};
        std::scoped_lock lock(s_mutex);
        if (s_slot.snapshot.ownerToken != 0) {
            outInvalidation.ownerToken = s_slot.snapshot.ownerToken;
            outInvalidation.reason = reason;
        }
        s_slot = {};
        s_active.store(false, std::memory_order_release);
    }

    void clearAll()
    {
        std::scoped_lock lock(s_mutex);
        s_slot = {};
        s_active.store(false, std::memory_order_release);
    }

    bool hasOverride()
    {
        return s_active.load(std::memory_order_acquire);
    }

    bool copySnapshot(Snapshot& outSnapshot)
    {
        outSnapshot = {};
        if (!s_active.load(std::memory_order_acquire)) {
            return false;
        }
        std::scoped_lock lock(s_mutex);
        if (s_slot.snapshot.ownerToken == 0) {
            return false;
        }
        outSnapshot = s_slot.snapshot;
        return true;
    }
}
