#pragma once

#include "RE/NetImmerse/NiTransform.h"
#include "physics-interaction/weapon/AuthoredWeaponGripAuthorityPolicy.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace RE
{
    class NiAVObject;
    class TESObjectWEAP;
}

namespace rock::authored_weapon_grip_library
{
    inline constexpr std::size_t kFiringFingerBoneCount = 15;
    inline constexpr std::uint16_t kCompleteFiringFingerMask = authored_weapon_grip_authority_policy::kCompleteFiringFingerMask;

    enum class CaptureSource : std::uint8_t
    {
        Unknown,
        LiveEquippedGraph,
        NativeIdlePreharvest,
    };

    struct FiringFingerPose
    {
        std::array<RE::NiTransform, kFiringFingerBoneCount> localTransforms{};
        std::uint16_t enabledMask{ 0 };

        [[nodiscard]] bool complete() const noexcept { return authored_weapon_grip_authority_policy::completeFiringFingerPose(enabledMask); }
    };

    struct LookupResult
    {
        bool found{ false };
        RE::NiTransform rightHandWeaponLocal{};
        FiringFingerPose rightFiringFingerPose{};
        std::uint64_t captureSequence{ 0 };
        CaptureSource source{ CaptureSource::Unknown };
        bool usedVariantFallback{ false };
        const char* reason{ "notEvaluated" };
    };

    struct WeaponVariantIdentity
    {
        std::uint64_t key{ 0 };
    };

    /*
     * Resolve the fixed, value-only identity used by the cache. Callers that
     * start asynchronous work must capture this value while the scene root is
     * frame-valid; they must never retain the NiAVObject solely to re-identify
     * the variant later.
     */
    [[nodiscard]] WeaponVariantIdentity identifyWeaponVariant(const RE::NiAVObject* weaponRoot) noexcept;

    /*
     * Main-thread, process-local library of Bethesda's exact
     * RArm_Hand-in-Weapon relation. Entries are keyed by runtime weapon form,
     * power-armor topology, and the P-Grip child used by hFRIK to distinguish
     * stock variants. Storage is fixed and bounded: publication and lookup do
     * not allocate in the animation or interaction hot paths.
     */
    [[nodiscard]] bool publish(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot, bool inPowerArmor, const RE::NiTransform& rightHandWeaponLocal,
        std::uint64_t captureSequence, CaptureSource source, const FiringFingerPose* rightFiringFingerPose = nullptr);

    [[nodiscard]] bool publishResolvedVariant(const RE::TESObjectWEAP* weapon, WeaponVariantIdentity variant, bool inPowerArmor,
        const RE::NiTransform& rightHandWeaponLocal, std::uint64_t captureSequence, CaptureSource source,
        const FiringFingerPose* rightFiringFingerPose = nullptr);

    [[nodiscard]] LookupResult find(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot, bool inPowerArmor);
}
