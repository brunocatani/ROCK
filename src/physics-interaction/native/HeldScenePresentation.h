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

    /*
     * Result of one game-frame target-transport publication. When applied,
     * presentedBodyWorld is the exact BODY-frame pose the scene writer will
     * hand the held node during this frame's post-physics update, so the
     * game-frame hand visual can be posed on the same clock instead of on the
     * node's previous-frame value.
     */
    struct TargetTransportPublication
    {
        bool applied = false;
        RE::NiTransform presentedBodyWorld{};
    };

    [[nodiscard]] bool install() noexcept;
    void publishHeldBodies(bool isLeft, const Registration& registration) noexcept;
    void clearHeldBodies(bool isLeft) noexcept;
    TargetTransportPublication publishTargetTransport(
        bool isLeft,
        RE::hknpWorld* world,
        std::uint32_t bodyId,
        std::uint64_t traceId,
        const RE::NiTransform& targetBodyWorld,
        const RE::NiTransform& solvedBodyWorld) noexcept;
    /*
     * True when the scene writer would present THIS hand's published target
     * transport for the body. A body published by both hands resolves to the
     * earlier grab trace, exactly as the writer selects it.
     */
    [[nodiscard]] bool ownsPublishedTargetTransport(
        bool isLeft,
        RE::hknpWorld* world,
        std::uint32_t bodyId) noexcept;
}
