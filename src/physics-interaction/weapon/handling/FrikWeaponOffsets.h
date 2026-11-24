// FRIK's weapon offsets for a weapon (see FrikWeaponOffsetPolicy.h and
// FrikWeaponOffsets.cpp).
#pragma once

#include "physics-interaction/weapon/handling/FrikWeaponOffsetPolicy.h"

#include "RE/NetImmerse/NiTransform.h"

#include <optional>

namespace RE
{
    class NiAVObject;
    class TESObjectWEAP;
}

/*
 * FRIK's weapon offsets (see FrikWeaponOffsetPolicy.h), read through FRIK's
 * API and used only while WeaponHandling.FrikUserWeaponOffsets is on; the
 * offsets FRIK ships with count only while
 * WeaponHandling.FrikBuiltInWeaponOffsets is on too. A weapon with one is
 * seated and scaled from it instead of ROCK's authored grip, keeps the
 * animation's support seat scaled with it, and is published with
 * WeaponStateFlag::UserWeaponOffset so PAPER skips its authored reload and
 * cycle animations, which do not fit the moved hands.
 *
 * FRIK is asked on every call, so an offset saved in FRIK's weapon adjustment
 * applies on the next frame. Needs FRIK API v2.5; with an older FRIK no
 * weapon has an offset. Thread model: game frame thread only.
 */
namespace rock::frik_weapon_offsets
{
    using frik_weapon_offset_policy::Override;

    // The override for this weapon model, or none. weaponRoot is the model
    // whose P-Grip child picks FRIK's pistol/rifle name variant.
    [[nodiscard]] std::optional<Override> find(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot,
        bool inPowerArmor);

    // RArm_Hand in Weapon for the override, from the weapon's native seat.
    [[nodiscard]] RE::NiTransform seat(const Override& override, const RE::NiTransform& nativeHandInWeapon);
}
