#pragma once

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiSmartPointer.h"

#include <array>
#include <cstdint>
#include <span>

namespace RE { class BSTriShape; }

namespace rock::weapon_material_visibility
{
    [[nodiscard]] bool isHidden(const RE::BSTriShape* shape);

    // Updated with a live weapon scene on the game thread. Equipped weapons run
    // before collider preparation; loose references share the same material rule.
    // Strong references keep hidden shapes alive until restoration.
    class State
    {
    public:
        [[nodiscard]] bool update(std::span<RE::NiAVObject* const> roots, std::uint32_t weaponFormID, bool traceDetails = true);
        void clear();

    private:
        struct Entry
        {
            RE::NiPointer<RE::NiAVObject> node;
            bool seen = false;
            bool owned = false;
        };
        std::array<Entry, 512> _culled{};
        std::size_t _count = 0;
        // Bounded first/settled snapshots for each equipped root set. Numeric
        // identities are used only for trace deduplication, never dereferenced.
        std::array<std::uintptr_t, 512> _tracedShapes{};
        std::size_t _traceCount = 0;
        std::uint64_t _traceRoots = 0;
        unsigned _traceFrame = 0;
    };
}
