#pragma once

#include <cstdint>

namespace rock::weapon_material_visibility
{
    struct AlphaState
    {
        std::uint16_t flags = 0;
        std::uint8_t threshold = 0;
        bool present = false;
    };

    // NiAlphaProperty: blending bit 0, source/destination factors at bits 1/5,
    // testing bit 9, test function at bit 10. A zero-alpha texture is not proof
    // of invisibility if the material ignores alpha or uses it differently.
    constexpr bool zeroAlphaIsInvisible(AlphaState alpha)
    {
        if (!alpha.present) return false;
        if (alpha.flags & 0x200u) {
            bool passes = true;
            switch ((alpha.flags >> 10) & 7u) {
            case 0: break; // always
            case 1: passes = alpha.threshold > 0; break; // less
            case 2: passes = alpha.threshold == 0; break; // equal
            case 3: break; // less/equal
            case 4: passes = false; break; // greater
            case 5: passes = alpha.threshold != 0; break; // not equal
            case 6: passes = alpha.threshold == 0; break; // greater/equal
            case 7: passes = false; break; // never
            }
            if (!passes) return true;
        }
        const auto source = (alpha.flags >> 1) & 15u;
        const auto destination = (alpha.flags >> 5) & 15u;
        return (alpha.flags & 1u) && source == 6u && (destination == 7u || destination == 0u);
    }

    struct CullDecision
    {
        bool culled;
        bool owned;
    };

    constexpr CullDecision decideCull(bool hiddenMaterial, bool currentlyCulled, bool owned)
    {
        if (hiddenMaterial) return { true, owned || !currentlyCulled };
        return { currentlyCulled && !owned, false };
    }
}
