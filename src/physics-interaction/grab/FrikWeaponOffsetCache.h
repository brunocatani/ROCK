#pragma once

#include "RE/NetImmerse/NiTransform.h"

namespace RE
{
    class NiAVObject;
    class TESObjectWEAP;
}

namespace rock::frik_weapon_offset_cache
{
    struct LookupResult
    {
        bool found = false;
        RE::NiTransform offset{};
        const char* reason = "notEvaluated";
    };

    void preload();

    [[nodiscard]] LookupResult findPrimaryWeaponOffset(
        const RE::TESObjectWEAP* weapon,
        const RE::NiAVObject* weaponRoot);
}
