#pragma once

#include "RE/NetImmerse/NiSmartPointer.h"

#include <cstdint>

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
    // Stored references are disabled and need not have their own loaded 3D.
    // Identify the exact item from its handle and the frame-borrowed visible
    // display. The existing one-slot probe owns/copies everything it retains.
    void observeStoredDisplay(RE::NiPointer<RE::TESObjectREFR> candidate,RE::NiAVObject* displayRoot) noexcept;
    // Inventory-owned display: retain the exact instance data, but identify
    // authored poses in the display model's same local variant domain.
    void observeInventoryDisplay(RE::TESObjectWEAP*,RE::NiAVObject*,RE::TBO_InstanceData*) noexcept;

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
