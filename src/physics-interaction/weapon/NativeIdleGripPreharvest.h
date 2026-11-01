#pragma once

#include "RE/NetImmerse/NiSmartPointer.h"

#include <cstddef>
#include <cstdint>
#include <span>

namespace RE
{
    class NiAVObject;
    class TESObjectREFR;
    class TESObjectWEAP;
    class TBO_InstanceData;
}

namespace rock::native_idle_grip_preharvest
{
    /*
     * Advance the one-slot off-screen native animation probe, then offer one
     * retained loose weapon reference as the next candidate. Frame-thread
     * only. A null candidate still advances an in-flight asynchronous engine
     * load. Ownership by value prevents native progress from invalidating the
     * reference before its scene identity is copied.
     */
    void observeCandidate(RE::NiPointer<RE::TESObjectREFR> candidate, bool isLeft, bool held) noexcept;
    // Keeps at most one loaded resource set per held weapon/physical hand.
    // These are off-screen animation resources, never equipment or firing state.
    [[nodiscard]] bool isSubgraphWarm(std::uint64_t handle) noexcept;
    void clearHeldWarmups() noexcept;
    // Inventory-owned display: retain the exact instance data, but identify
    // authored poses in the display model's same local variant domain.
    void observeInventoryDisplay(RE::TESObjectWEAP*,RE::NiAVObject*,RE::TBO_InstanceData*) noexcept;

    /*
     * Inventory firearms a consumer keeps ready to equip (Weapon
     * setWarmInventoryWeapons): their full first-person subgraphs stay loaded
     * in off-screen managers, so the native equip reuses the resources
     * instead of waiting on the engine's first-use load. Pointers are
     * borrowed for the call; the instance is retained. Frame-thread only.
     */
    inline constexpr std::size_t kConsumerWarmCapacity = 8;
    struct ConsumerWarmRequest
    {
        RE::TESObjectWEAP* weapon{ nullptr };
        RE::TBO_InstanceData* instance{ nullptr };
    };
    // Replaces owner's set, keeping what stays loaded; false (nothing
    // changed) when every owner's sets together would exceed the capacity.
    [[nodiscard]] bool setConsumerWarmSet(std::uint64_t owner, std::span<const ConsumerWarmRequest> requests) noexcept;
    void clearConsumerWarmSet(std::uint64_t owner) noexcept;
    // Once per frame: reload after a race or power-armor change and start the
    // next requested weapon that is not loaded yet (one load at a time).
    void observeConsumerWarmups() noexcept;

    /*
     * Offer the currently equipped weapon after its stable generation exists.
     * All pointers are frame-borrowed: the implementation value-copies the
     * variant and deterministic instance-content identities and retains
     * instance data with Bethesda's smart pointer before asynchronous work.
     */
    void observeEquippedWeapon(
        RE::TESObjectWEAP* weapon,
        RE::NiAVObject* weaponRoot,
        RE::TBO_InstanceData* instanceData,
        std::uint64_t instanceContentKey) noexcept;
}
