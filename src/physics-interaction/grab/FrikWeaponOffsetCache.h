#pragma once

#include "RE/NetImmerse/NiTransform.h"

#include <cstdint>

namespace RE
{
    class NiAVObject;
    class TESObjectWEAP;
}

namespace rock::frik_weapon_offset_cache
{
    enum class OffsetSource : std::uint8_t
    {
        None,
        EmbeddedResource,
        CustomFile,
        LiveWeaponNodeFallback,
    };

    struct LookupResult
    {
        bool found = false;
        RE::NiTransform offset{};
        OffsetSource source = OffsetSource::None;
        const char* reason = "notEvaluated";
    };

    void preload();
    [[nodiscard]] std::uint64_t currentRevision() noexcept;

    [[nodiscard]] LookupResult findPrimaryWeaponOffset(
        const RE::TESObjectWEAP* weapon,
        const RE::NiAVObject* weaponRoot);
}
