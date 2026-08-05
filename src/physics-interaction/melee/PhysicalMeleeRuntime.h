#pragma once

#include "api/ROCKProviderApi.h"

#include <cstdint>

namespace RE
{
    class bhkWorld;
}

namespace rock::physical_melee
{
    [[nodiscard]] bool initializeRuntime();
    void resetRuntime() noexcept;
    void drainCompletedOutcomes(std::uint64_t currentFrameIndex);
    void processContactObservation(
        RE::bhkWorld* bhkWorld,
        const provider::RockProviderExternalContactV1& observation,
        std::uint32_t worldGeneration,
        std::uint32_t skeletonGeneration,
        std::uint32_t providerGeneration,
        std::uint64_t processingFrameIndex);
}
