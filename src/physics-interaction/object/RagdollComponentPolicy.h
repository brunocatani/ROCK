#pragma once

#include <array>
#include <cstdint>
#include <span>

namespace rock::ragdoll
{
    inline constexpr std::size_t kMaxBodies = 96;
    inline constexpr std::size_t kMaxConstraints = 192;
    inline constexpr std::uint32_t kInvalidId = 0x7FFF'FFFF;

    struct BodyNode
    {
        std::uint32_t id = kInvalidId;
        std::uint32_t motion = kInvalidId;
        bool dynamic = false;
    };

    struct Joint
    {
        std::uint32_t a = kInvalidId;
        std::uint32_t b = kInvalidId;
        bool enabled = false;
    };

    struct Membership
    {
        std::array<bool, kMaxBodies> included{};
        bool valid = false;
        bool fixedAttached = false;
    };

    inline Membership connectedComponent(std::span<const BodyNode> bodies,
        std::span<const Joint> joints, std::uint32_t primary) noexcept
    {
        Membership result{};
        if (bodies.size() > kMaxBodies || joints.size() > kMaxConstraints || primary == kInvalidId) return result;
        std::array<std::size_t, kMaxBodies> queue{};
        std::size_t read = 0, count = 0;
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            if (bodies[i].id == primary && bodies[i].dynamic) {
                result.included[i] = true;
                queue[count++] = i;
                break;
            }
        }
        if (count == 0) return result;
        auto include = [&](std::size_t i) {
            if (result.included[i] || bodies[i].id == kInvalidId) return;
            result.included[i] = true;
            if (bodies[i].dynamic) queue[count++] = i;
            else result.fixedAttached = true;
        };
        while (read < count) {
            const auto& from = bodies[queue[read++]];
            for (std::size_t i = 0; i < bodies.size(); ++i) {
                const auto& to = bodies[i];
                // Motion zero is the fixed world; it does not connect unrelated statics.
                if (from.dynamic && to.dynamic && from.motion != 0 && from.motion != kInvalidId &&
                    from.motion == to.motion) include(i);
            }
            for (const auto& joint : joints) {
                if (!joint.enabled) continue;
                const auto neighbor = joint.a == from.id ? joint.b : joint.b == from.id ? joint.a : kInvalidId;
                if (neighbor == kInvalidId) continue;
                for (std::size_t i = 0; i < bodies.size(); ++i) if (bodies[i].id == neighbor) { include(i); break; }
            }
        }
        result.valid = true;
        return result;
    }
}
