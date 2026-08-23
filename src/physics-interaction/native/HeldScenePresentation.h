#pragma once

#include "RE/NetImmerse/NiTransform.h"

#include <cstddef>
#include <cstdint>

namespace RE
{
    class hknpWorld;
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
    };

    [[nodiscard]] bool install() noexcept;
    void publishHeldBodies(bool isLeft, const Registration& registration) noexcept;
    void clearHeldBodies(bool isLeft) noexcept;
    void publishTargetTransport(
        bool isLeft,
        RE::hknpWorld* world,
        std::uint32_t bodyId,
        std::uint64_t traceId,
        const RE::NiTransform& targetBodyWorld,
        const RE::NiTransform& solvedBodyWorld) noexcept;
}
