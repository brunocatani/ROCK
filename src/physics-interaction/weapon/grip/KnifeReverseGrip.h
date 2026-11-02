// Reverse grip for one-handed melee weapons: the trigger of the carrying hand
// toggles it (see KnifeReverseGrip.cpp).
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
    // a reverse grip. The trigger of the hand carrying one toggles it. The grip
    // stays while that weapon stays equipped, through menus (which release the
    // live grip) and save loads (weapon_hand_save); the hand taking another
    // one-handed melee weapon or the weapon leaving both equip indices returns
    // it to the normal grip. Automatic melee keeps the native trigger attack.
    // Frame-thread state; the saved snapshot and a pending restore are atomics.
    [[nodiscard]] bool appliesTo(const RE::TESObjectWEAP* weapon) noexcept;

    // Once per hand per input frame: the knife the hand carries now (0 when
    // none or not held right now), the weapon forms at the two native equip
    // indices, and whether its trigger was pressed this frame.
    void observeHand(bool isLeft, std::uint32_t knifeFormID, std::uint32_t wornForm0, std::uint32_t wornForm1,
        bool triggerPressed) noexcept;

    // The reverse grip of one hand for the save game: the knife (0 none) and its grip.
    struct SavedGrip
    {
        std::uint32_t formID = 0;
        bool reversed = false;
    };

    // Any thread.
    [[nodiscard]] SavedGrip savedGrip(bool isLeft) noexcept;
    // Any thread (save-load callbacks): the grip the hand takes when it next
    // carries that knife; another knife in that hand first drops it.
    void restoreGrip(bool isLeft, SavedGrip grip) noexcept;

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
