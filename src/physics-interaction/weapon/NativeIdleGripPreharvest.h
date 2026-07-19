#pragma once

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
     * loose weapon reference as the next candidate. Frame-thread only. A null
     * candidate still advances an in-flight asynchronous engine load.
     */
    void observeCandidate(RE::TESObjectREFR* candidate) noexcept;

    /*
     * Offer the currently equipped weapon after its stable generation exists.
     * All pointers are frame-borrowed: the implementation value-copies the
     * variant identity and retains instance data with Bethesda's smart pointer
     * before starting asynchronous work.
     */
    void observeEquippedWeapon(
        RE::TESObjectWEAP* weapon,
        RE::NiAVObject* weaponRoot,
        RE::TBO_InstanceData* instanceData) noexcept;
}
