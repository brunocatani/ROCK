#pragma once

namespace RE
{
    class TESObjectREFR;
}

namespace rock::native_idle_grip_preharvest
{
    /*
     * Advance the one-slot off-screen native animation probe, then offer one
     * loose weapon reference as the next candidate. Frame-thread only. A null
     * candidate still advances an in-flight asynchronous engine load.
     */
    void observeCandidate(RE::TESObjectREFR* candidate) noexcept;
}
