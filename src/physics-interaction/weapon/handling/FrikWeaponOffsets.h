// Player-made FRIK weapon offsets (see FrikWeaponOffsetPolicy.h and
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
 * Player-made FRIK weapon offset files (see FrikWeaponOffsetPolicy.h), used
 * only while WeaponHandling.FrikUserWeaponOffsets is on. A weapon with
 * one is seated and scaled from it instead of ROCK's authored
 * grip, takes only dynamic support grabs, and is published with
 * WeaponStateFlag::UserWeaponOffset so PAPER skips its authored reload and
 * cycle animations, which do not fit the moved hands.
 *
 * Thread model: load(), processPendingReload() and find() run on the game
 * frame thread. The directory watcher thread only publishes a change flag;
 * the files are re-read on the frame thread after the burst settles.
 */
namespace rock::frik_weapon_offsets
{
    using frik_weapon_offset_policy::Override;

    // Startup, after FRIK initialised its config folders.
    void load();

    // Frame start: re-read the folder once a change burst has settled.
    void processPendingReload();

    // The override for this weapon model, or none. weaponRoot is the model
    // whose P-Grip child picks FRIK's pistol/rifle name variant.
    [[nodiscard]] std::optional<Override> find(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot,
        bool inPowerArmor);

    // RArm_Hand in Weapon for the override, from the weapon's native seat.
    [[nodiscard]] RE::NiTransform seat(const Override& override, const RE::NiTransform& nativeHandInWeapon);
}
