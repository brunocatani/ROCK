#pragma once

#include "RE/NetImmerse/NiTransform.h"

#include <cstdint>

namespace RE
{
    class TESObjectWEAP;
}

namespace rock::knife_reverse_grip
{
    // One-handed melee weapons (sword, dagger, axe and mace animation types:
    // knives, switchblade, baton, pipes and modded one-handers) can be held in
    // a reverse grip. The trigger of the hand carrying one toggles it; any
    // change of that hand's weapon, including holstering, returns it to the
    // normal grip. Automatic melee keeps the native trigger attack.
    // Frame-thread state only.
    [[nodiscard]] bool appliesTo(const RE::TESObjectWEAP* weapon) noexcept;

    // Once per hand per input frame: the knife the hand carries now (0 when
    // none) and whether its trigger was pressed this frame.
    void observeHand(bool isLeft, std::uint32_t knifeFormID, bool triggerPressed) noexcept;

    [[nodiscard]] bool reversed(bool isLeft, std::uint32_t formID) noexcept;

    // The hand-in-weapon seat with the knife turned 180 degrees about its
    // edge axis (weapon X) through the handle centre under the palm: the
    // handle stays in the fist, the edge keeps its facing and the blade
    // leaves the hand on the other side. flatPlaneZ is the model's flat-side
    // centre (WeaponCollision mirror offsets). The authored seat, its
    // left-hand mirror and every finger pose are kept as they are.
    [[nodiscard]] RE::NiTransform reverseSeat(const RE::NiTransform& seat, bool isLeft, float flatPlaneZ) noexcept;

    // The seat a placement uses: reversed while this hand's knife is.
    [[nodiscard]] RE::NiTransform presentedSeat(const RE::NiTransform& seat, bool isLeft, std::uint32_t formID,
        float flatPlaneZ) noexcept;
}
