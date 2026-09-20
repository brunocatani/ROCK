#pragma once

#include <cstdint>
#include <span>

namespace RE { class hknpWorld; }
namespace rock::native_player_collision { struct BodyPair; }
namespace rock::havok_physics_timing { struct PhysicsTimingSample; }

namespace rock::shell_casing_grace
{
    // Process-lifetime native hooks. Only the native player's shell-ejection
    // call is admitted; other debris creation retains native behavior.
    bool install() noexcept;

    // Game-thread publication from the existing generated-weapon bank owner.
    void publishWeapon(std::uint32_t formId, std::uint64_t generation,
        std::span<const std::uint32_t> bodyIds) noexcept;
    void prepareFrame(RE::hknpWorld* world, float milliseconds,
        double simulatedSeconds, std::uint64_t completedSolve) noexcept;
    void abandon() noexcept;

    // Native world mutation is excluded during pair callbacks. Birth is
    // recorded under the engine's existing world write lock; expiry and cache
    // invalidation run before collision, never inside the pair callback.
    void beforeCollide(RE::hknpWorld* world,
        const havok_physics_timing::PhysicsTimingSample& timing) noexcept;
    void afterSolve(RE::hknpWorld* world,
        const havok_physics_timing::PhysicsTimingSample& timing) noexcept;
    int filterPairs(RE::hknpWorld* world,
        native_player_collision::BodyPair* pairs, int count) noexcept;
}
