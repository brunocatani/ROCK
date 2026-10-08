#pragma once

#include "physics-interaction/collision/NativePlayerCollisionPolicy.h"
#include "physics-interaction/hand/WeaponReleaseCollisionLease.h"

#include <cstddef>
#include <span>

namespace RE { class hknpWorld; }
namespace rock::havok_runtime { struct BodySnapshot; }

namespace rock::native_player_collision
{
    inline constexpr std::size_t kMaximumPlayerBodies = 64;

    // Process-lifetime hook, installed once on the game thread. Only simulation
    // body pairs are filtered; native ray/shape query entry points stay intact.
    bool install() noexcept;
    bool installed() noexcept;

    // Called only with a live callback-local snapshot. Equipped weapons resolve
    // to their actor; only a positive world WEAP reference is a loose weapon.
    // Missing/stale ownership preserves native contacts and records a counter.
    bool isLooseWeaponBody(const havok_runtime::BodySnapshot& body) noexcept;

    // Single game-thread publisher; physics workers only read a quiesced value
    // snapshot. IDs are checked against live identity before any suppression.
    // An empty publication restores physical pairs in the supplied live world.
    void publish(RE::hknpWorld* world, std::span<const BodyIdentity> bodies);
    // Exact dynamic-hand bodies owned by an authored handle. Only their world
    // surface simulation pairs are removed; publication changes rebuild caches.
    void publishAuthoredHands(RE::hknpWorld* world, std::span<const BodyIdentity> bodies);

    // Single owner: the equipped BladePenetrationRuntime. Game-thread mutation
    // while its physics callbacks are quiesced. The existing simulation hook
    // consumes this exact pair even when the native-player body list is empty.
    bool publishBladePair(RE::hknpWorld* world, std::uint32_t weaponBody, std::uint32_t targetBody);
    bool hasBladePair(RE::hknpWorld* world, std::uint32_t weaponBody, std::uint32_t targetBody) noexcept;
    std::uint64_t bladePairRejectedCount() noexcept;
    // A null liveWorld clears ownership after world loss without native calls.
    void clearBladePair(RE::hknpWorld* liveWorld);

    // DynamicHandCollision owns the timer on the game thread. Publish before
    // detaching, clear on expiry/invalidation; simulation workers read only a
    // quiesced identity. Does not require an optional constraint pair filter.
    bool publishWeaponRelease(RE::hknpWorld* world, weapon_release_collision_lease::Identity weapon);
    void clearWeaponRelease(RE::hknpWorld* liveWorld);

    // EjectionContacts owns these exact magazine/equipped-weapon sets. Single
    // game-thread publisher under the world write lock; empty sets restore pairs.
    bool publishMagazineEjection(RE::hknpWorld* world, std::span<const std::uint32_t> magazines,
        std::span<const std::uint32_t> weapons);
    void clearMagazineEjection(RE::hknpWorld* liveWorld);
    std::uint64_t magazineEjectionRejectedCount() noexcept;

    // Skeleton/world teardown: invalidate the snapshot without touching a world
    // whose lifetime may have ended. No collision bits or pair leases to restore.
    void abandon() noexcept;
}
