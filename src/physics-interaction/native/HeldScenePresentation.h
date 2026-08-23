#pragma once

#include "RE/NetImmerse/NiTransform.h"

#include <cstddef>
#include <cstdint>

namespace RE
{
    class hknpWorld;
    class NiAVObject;
    class NiCollisionObject;
}

namespace rock::held_scene_presentation
{
    inline constexpr std::size_t kMaxRegisteredBodies = 64;

    struct RegisteredBody
    {
        RE::NiCollisionObject* collisionObject = nullptr;
        RE::hknpWorld* world = nullptr;
        std::uint32_t bodyId = 0x7FFF'FFFFu;
    };

    struct Registration
    {
        RegisteredBody bodies[kMaxRegisteredBodies]{};
        std::size_t count = 0;
        std::uint64_t traceId = 0;
        RE::NiAVObject* visibleGeometry = nullptr;
    };

    struct RenderConsumptionSample
    {
        std::uint64_t ordinal = 0;
        std::uint64_t traceId = 0;
        std::uint64_t captureMicroseconds = 0;
        std::uintptr_t renderPass = 0;
        std::uint32_t threadId = 0;
        std::uint32_t technique = 0;
        RE::NiAVObject* geometry = nullptr;
        RE::NiTransform world{};
        std::uint64_t shaderFlagsBefore = 0;
        std::uint64_t shaderFlagsAfter = 0;
    };

    [[nodiscard]] bool install() noexcept;
    [[nodiscard]] bool installRenderConsumptionProbe() noexcept;
    void publishHeldBodies(bool isLeft, const Registration& registration) noexcept;
    void clearHeldBodies(bool isLeft) noexcept;
    [[nodiscard]] bool readLatestRenderConsumption(
        bool isLeft,
        std::uint64_t afterOrdinal,
        RenderConsumptionSample& sample) noexcept;
}
