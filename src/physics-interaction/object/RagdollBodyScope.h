#pragma once

#include "physics-interaction/object/RagdollComponentPolicy.h"

namespace RE
{
    class NiCollisionObject;
    class NiAVObject;
    class hknpWorld;
}

namespace rock::ragdoll
{
    struct BodyOwner
    {
        BodyNode body{};
        RE::NiCollisionObject* collision = nullptr;
        RE::NiAVObject* node = nullptr;
    };

    // A frame-local read result. No native pointer in this value is retained by a hold.
    struct Component
    {
        std::array<BodyOwner, kMaxBodies> bodies{};
        std::size_t count = 0;
        bool valid = false;
        bool fixedAttached = false;
        const char* reason = "not-read";

        bool contains(std::uint32_t id) const noexcept
        {
            for (std::size_t i = 0; i < count; ++i) if (bodies[i].body.id == id) return true;
            return false;
        }
    };

    // Reads only the primary's native system, under the existing world read lock.
    Component readComponent(RE::hknpWorld* world, std::uint32_t primary);
}
