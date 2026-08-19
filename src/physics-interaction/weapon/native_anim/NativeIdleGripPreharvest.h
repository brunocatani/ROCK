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
    void observeCandidate(RE::NiPointer<RE::TESObjectREFR> candidate) noexcept;

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
