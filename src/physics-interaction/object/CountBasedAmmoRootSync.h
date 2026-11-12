#pragma once

#include <array>
#include <cstdint>

#include "RE/Bethesda/BSPointerHandle.h"
#include "RE/NetImmerse/NiTransform.h"

namespace RE
{
    class hknpWorld;
    class TESObjectREFR;
}

/*
 * Keeps the geometry-less root body of count-based loose ammo on its visible
 * branch body outside a grab (CountBasedAmmo.h explains the two bodies).
 *
 * Nothing in the model joins the two bodies, so a round the hand pushed, or
 * one released from a hand, leaves its root body behind: an invisible cube the
 * hand bumps into, and the place the reference reloads at. A tracked round has
 * its root body written onto the branch body every frame, with the branch
 * body's linear velocity, until the round has rested for kRestSeconds. While a
 * hand holds the round, that hand carries the root body instead
 * (Hand::carryCountBasedAmmoAnchor) and the entry only stays alive.
 *
 * A round is tracked from the frames either hand selects or holds it. Entries
 * hold reference handles, never pointers, and every frame re-resolves the
 * reference, its 3D and both bodies; anything missing drops the entry.
 * Frame thread only.
 */
namespace rock::count_based_ammo
{
    class RootBodySync
    {
    public:
        static constexpr std::size_t kCapacity = 16;
        static constexpr float kRestSeconds = 1.0f;

        // Starts or refreshes tracking. Ignores anything that is not
        // count-based loose ammo.
        void track(RE::TESObjectREFR* refr, bool held);

        // Writes every tracked, unheld root body onto its branch body.
        void update(RE::hknpWorld* world, float deltaSeconds);

        void clear() noexcept { _entries = {}; }

    private:
        struct Entry
        {
            RE::ObjectRefHandle handle{};
            std::uint32_t formId = 0;
            // The root body's frame in the branch body's frame at rest,
            // measured when tracking starts.
            RE::NiTransform rootFromBranch{};
            bool measured = false;
            // Set by track() for the current frame; cleared by update().
            bool held = false;
            bool touched = false;
            float restSeconds = 0.0f;

            [[nodiscard]] bool active() const noexcept { return formId != 0; }
        };

        std::array<Entry, kCapacity> _entries{};
    };
}
