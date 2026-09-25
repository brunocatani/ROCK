#pragma once

#include "physics-interaction/collision/NativePlayerCollisionPolicy.h"

#include <cstddef>
#include <span>

namespace RE { class hknpWorld; class TESObjectREFR; }
namespace rock::havok_runtime { struct BodySnapshot; }

namespace rock::native_player_collision
{
    inline constexpr std::size_t kMaximumPlayerBodies = 64;

    // Process-lifetime hook, installed once on the game thread. Only simulation
    // body pairs are filtered; native ray/shape query entry points stay intact.
    bool install() noexcept;

    // Called only with a live callback-local snapshot. Equipped weapons resolve
    // to their actor; only a positive world WEAP reference is a loose weapon.
    // Missing/stale ownership preserves native contacts and records a counter.
    bool isLooseWeaponBody(const havok_runtime::BodySnapshot& body) noexcept;

    // Single game-thread publisher; physics workers only read a quiesced value
    // snapshot. IDs are checked against live identity before any suppression.
    // An empty publication restores physical pairs in the supplied live world.
    void publish(RE::hknpWorld* world, std::span<const BodyIdentity> bodies);

    // Single owner: the equipped BladePenetrationRuntime. Game-thread mutation
    // while its physics callbacks are quiesced. The existing simulation hook
    // consumes this exact pair even when the native-player body list is empty.
    bool publishBladePair(RE::hknpWorld* world, std::uint32_t weaponBody, std::uint32_t targetBody);
    bool hasBladePair(RE::hknpWorld* world, std::uint32_t weaponBody, std::uint32_t targetBody) noexcept;
    std::uint64_t bladePairRejectedCount() noexcept;
    // A null liveWorld clears ownership after world loss without native calls.
    void clearBladePair(RE::hknpWorld* liveWorld);

    // PhysicalWeaponPhysics pins the exact reference until withdrawal. These
    // simulation-only rules cover native bodies published outside the 3D tree
    // and bodies created later; generated replacement bodies stay collidable.
    bool publishPhysicalWeapon(unsigned slot, RE::hknpWorld* world,
        RE::TESObjectREFR* reference, std::span<const std::uint32_t> knownBodies);
    bool setPhysicalWeaponReady(unsigned slot, bool ready);
    void clearPhysicalWeapon(unsigned slot, RE::hknpWorld* liveWorld);
    bool isReplacedWeaponBody(RE::hknpWorld* world, std::uint32_t body) noexcept;

    // Skeleton/world teardown: invalidate the snapshot without touching a world
    // whose lifetime may have ended. No collision bits or pair leases to restore.
    void abandon() noexcept;
}
