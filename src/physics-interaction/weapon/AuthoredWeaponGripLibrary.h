#pragma once

#include "RE/NetImmerse/NiTransform.h"

#include <cstdint>

namespace RE
{
    class NiAVObject;
    class TESObjectWEAP;
}

namespace rock::authored_weapon_grip_library
{
    enum class CaptureSource : std::uint8_t
    {
        Unknown,
        LiveEquippedGraph,
        NativeIdlePreharvest,
    };

    struct LookupResult
    {
        bool found{ false };
        RE::NiTransform rightHandWeaponLocal{};
        std::uint64_t captureSequence{ 0 };
        CaptureSource source{ CaptureSource::Unknown };
        bool usedVariantFallback{ false };
        const char* reason{ "notEvaluated" };
    };

    /*
     * Main-thread, process-local library of Bethesda's exact
     * RArm_Hand-in-Weapon relation. Entries are keyed by runtime weapon form,
     * power-armor topology, and the P-Grip child used by hFRIK to distinguish
     * stock variants. Storage is fixed and bounded: publication and lookup do
     * not allocate in the animation or interaction hot paths.
     */
    [[nodiscard]] bool publish(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot, bool inPowerArmor, const RE::NiTransform& rightHandWeaponLocal,
        std::uint64_t captureSequence, CaptureSource source);

    [[nodiscard]] LookupResult find(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot, bool inPowerArmor);
}
