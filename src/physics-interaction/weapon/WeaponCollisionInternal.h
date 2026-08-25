#pragma once

#include "physics-interaction/weapon/WeaponCollision.h"

namespace rock::weapon_collision_internal
{
    [[nodiscard]] const RE::BGSObjectInstanceExtra* findEquippedWeaponObjectInstanceExtra(
        const RE::PlayerCharacter* player,
        const RE::TESForm* weaponForm,
        const RE::TBO_InstanceData* instanceData);
    [[nodiscard]] const char* safeNodeName(const RE::NiAVObject* node);
}
