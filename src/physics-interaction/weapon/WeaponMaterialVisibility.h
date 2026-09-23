#pragma once

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiSmartPointer.h"

#include <array>
#include <span>

namespace RE { class BSTriShape; }

namespace rock::weapon_material_visibility
{
    [[nodiscard]] bool isHidden(const RE::BSTriShape* shape);

    // Updated with the live equipped scene on the game thread, before collider
    // preparation. Strong references keep hidden shapes alive until restoration.
    class State
    {
    public:
        [[nodiscard]] bool update(std::span<RE::NiAVObject* const> roots);
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
    };
}
