#pragma once

#include "api/ROCKProviderApi.h"
#include "physics-interaction/melee/PhysicalMeleePolicy.h"

#include <cstdint>

namespace RE
{
    class bhkWorld;
}

namespace rock::physical_melee
{
    struct ContactFrameContext
    {
        RE::bhkWorld* bhkWorld{ nullptr };
        WeaponIdentityWitness currentWeapon{};
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint64_t processingFrameIndex{ 0 };
        bool damageSubmissionAllowed{ false };
    };

    [[nodiscard]] bool initializeRuntime();
    void resetRuntime() noexcept;
    void drainCompletedOutcomes(std::uint64_t currentFrameIndex);
    void beginContactFrame(const ContactFrameContext& context);
    void processContactObservation(
        const provider::RockProviderExternalContactV1& observation,
        const WeaponIdentityWitness& expectedWeapon,
        std::uint32_t targetCollisionLayer);
    void finishContactFrame();
}
