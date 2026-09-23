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

    // FO4VR lighting uses the enabled alpha-test threshold, not NiAlphaProperty's
    // legacy comparison-mode bits: 1428B6F4A..81 uploads +2A directly. BGSM
    // application at 14279239D..3C8 sets bit 9 and +2A independently of that mode.
    // In particular, 02EC (BGSM-created rail) and 12EC (authored sticker) agree.
    constexpr bool zeroAlphaIsInvisible(AlphaState alpha)
    {
        if (!alpha.present) return false;
        if ((alpha.flags & 0x200u) && alpha.threshold > 0) return true;
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
