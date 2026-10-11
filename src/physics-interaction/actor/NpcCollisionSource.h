#pragma once

#include "physics-interaction/collision/CollisionLayerPolicy.h"

#include <cstdint>
#include <cstddef>

namespace rock::npc_collision
{
    inline constexpr std::size_t kActorCapacity = 8;
    inline constexpr std::size_t kBodyCapacity = 64;
    inline constexpr std::size_t kDebugBodyCapacity = kActorCapacity * kBodyCapacity * 2;

    struct SourceIdentity
    {
        std::uint32_t bodyId = 0x7FFF'FFFF;
        std::uint32_t motionId = 0x7FFF'FFFF;
        std::uintptr_t collisionObject = 0;
        std::uintptr_t ownerNode = 0;
        std::uintptr_t shape = 0;
        std::uint32_t group = 0;

        bool operator==(const SourceIdentity&) const = default;

        [[nodiscard]] bool valid() const noexcept
        {
            return bodyId != 0x7FFF'FFFF && motionId != 0 && motionId != 0x7FFF'FFFF && collisionObject && ownerNode && shape;
        }
    };

    struct SourceState
    {
        SourceIdentity identity{};
        std::uint32_t filter = 0;
        std::uint32_t flags = 0;
        std::int32_t presence = -1;

        [[nodiscard]] bool animatedLimb() const noexcept
        {
            using namespace collision_layer_policy;
            const auto layer = filter & FO4_LAYER_FILTER_MASK;
            // Only native animated limb bodies. Excludes the separate pose-bound
            // body, disabled filters, corpses, and dynamic ragdoll ownership.
            return identity.valid() && (layer == FO4_LAYER_BIPED || layer == FO4_LAYER_BIPED_NO_CC) &&
                (flags & 7u) == 6u && (flags & 0x80000u) == 0 && (filter & 0x4000u) == 0;
        }

        [[nodiscard]] bool needsSupplement() const noexcept
        {
            // FO4VR 140E3CD92..DE7 and 14156589D..8F8 use body+6C != -1
            // or flags bit 18 for native presence. Removal at 141544E20..E32
            // writes -1 and clears that bit. Allocated IDs alone are insufficient.
            return animatedLimb() && !hasNativePresence();
        }

        [[nodiscard]] bool hasNativePresence() const noexcept { return presence != -1 || (flags & 0x40000u) != 0; }

        [[nodiscard]] bool canDrive(const SourceIdentity& expected) const noexcept
        {
            return needsSupplement() && identity == expected;
        }
    };
}
