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
        PersistedNativeIdle,
        NativeIdlePreharvest,
    };

    [[nodiscard]] constexpr bool isNativeIdleAuthority(const CaptureSource source) noexcept
    {
        return source == CaptureSource::PersistedNativeIdle || source == CaptureSource::NativeIdlePreharvest;
    }

    struct FiringFingerPose
    {
        std::array<RE::NiTransform, kFiringFingerBoneCount> localTransforms{};
        std::uint16_t enabledMask{ 0 };

        [[nodiscard]] bool complete() const noexcept { return authored_weapon_grip_authority_policy::completeFiringFingerPose(enabledMask); }
    };

    struct LookupResult
    {
        bool found{ false };
        // Exact animation-authored wrist relation retained for hand and
        // finger presentation.
        RE::NiTransform rightHandWeaponLocal{};
        // Physical right-hand relation measured from ROCK's final
        // position-only equipped pose. Loose weapon placement consumes this
        // when available so it keeps the native weapon aim while reproducing
        // the same authored grip translation.
        RE::NiTransform rightPositionOnlyHandWeaponLocal{};
        FiringFingerPose rightFiringFingerPose{};
        // Paired support-arm relation for this exact authored canonical:
        // LArm_Hand in Weapon plus the left finger locals. A native-idle
        // source sampled it from the same idle clip as the primary pose; a
        // live source converged it from Bethesda's native right-primary
        // graph. It lets a direct physical-left equip seat authored support
        // grabs without ever seeing a native-right frame.
        RE::NiTransform supportHandWeaponLocal{};
        FiringFingerPose supportFingerPose{};
        std::uint64_t supportCaptureSequence{ 0 };
        std::uint64_t captureSequence{ 0 };
        std::uint64_t positionOnlyFrikOffsetRevision{ 0 };
        CaptureSource source{ CaptureSource::Unknown };
        CaptureSource supportSource{ CaptureSource::Unknown };
        bool hasSupportRelation{ false };
        bool supportPoseAbsent{ false };
        bool hasRightPositionOnlyHandWeaponLocal{ false };
        bool usedVariantFallback{ false };
        bool vanillaPipePose{ false };
        const char* reason{ "notEvaluated" };
    };

    struct WeaponVariantIdentity
    {
        std::uint64_t key{ 0 };
        std::uint64_t instanceContentKey{ 0 };
        bool instanceContentKnown{ false };
    };

    /*
     * Resolve the fixed, value-only identity used by the cache. Callers that
     * start asynchronous work must capture this value while the scene root is
     * frame-valid; they must never retain the NiAVObject solely to re-identify
     * the variant later.
     */
    [[nodiscard]] WeaponVariantIdentity identifyWeaponVariant(
        const RE::NiAVObject* weaponRoot,
        std::uint64_t instanceContentKey = 0,
        bool instanceContentKnown = false) noexcept;

    /*
     * Main-thread, process-local library of Bethesda's exact
     * RArm_Hand-in-Weapon relation plus ROCK's separately measured physical
     * position-only hold. Entries are keyed by runtime weapon form,
     * deterministic equipped-instance content when available, power-armor
     * topology, and the P-Grip child used by hFRIK to distinguish stock
     * variants. Storage is fixed and bounded: publication and lookup do not
     * allocate in the animation or interaction hot paths.
     */
    [[nodiscard]] bool publish(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot, bool inPowerArmor, const RE::NiTransform& rightHandWeaponLocal,
        std::uint64_t captureSequence, CaptureSource source, const FiringFingerPose* rightFiringFingerPose = nullptr);

    [[nodiscard]] bool publishResolvedVariant(const RE::TESObjectWEAP* weapon, WeaponVariantIdentity variant, bool inPowerArmor,
        const RE::NiTransform& rightHandWeaponLocal, std::uint64_t captureSequence, CaptureSource source,
        const FiringFingerPose* rightFiringFingerPose = nullptr, bool vanillaPipePose = false);

    // Apply only to a physical-right consumer copy. Stored poses and left
    // firing/support consumers retain the original animation data.
    void applyPipeDefaultOffset(LookupResult& result, bool isLeft) noexcept;

    /*
     * Attach the physical-hand relation measured from the final
     * position-only equipped solve to the exact authored capture that
     * produced it. This never replaces the animation-authored wrist relation.
     */
    [[nodiscard]] bool publishPositionOnlyHold(
        const RE::TESObjectWEAP* weapon,
        bool inPowerArmor,
        std::uint64_t authoredCaptureSequence,
        std::uint64_t frikOffsetRevision,
        const RE::NiTransform& rightPositionOnlyHandWeaponLocal);

    /*
     * Attach the paired support-arm relation to the entry for this exact
     * variant. The entry's authored canonical must already exist. Source
     * authority follows the primary pose: a native-idle clip sample is never
     * replaced by a live equipped-graph capture, which only fills an entry
     * the preharvest could not serve. A same-source, value-equivalent
     * republication keeps the stored sequence; a materially different
     * relation of accepted authority replaces it.
     */
    [[nodiscard]] bool publishSupportRelation(
        const RE::TESObjectWEAP* weapon,
        WeaponVariantIdentity variant,
        bool inPowerArmor,
        const RE::NiTransform& supportHandWeaponLocal,
        const FiringFingerPose& supportFingerPose,
        std::uint64_t supportCaptureSequence,
        CaptureSource source);

    [[nodiscard]] bool publishSupportAbsence(const RE::TESObjectWEAP* weapon,
        WeaponVariantIdentity variant, bool inPowerArmor, std::uint64_t captureSequence,
        CaptureSource source);

    // Resolve the raw animation relation into this model's registration frame.
    // findResolvedVariant and all publication calls retain raw animation space.
    [[nodiscard]] LookupResult find(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot, bool inPowerArmor);
    [[nodiscard]] LookupResult findResolvedVariant(const RE::TESObjectWEAP* weapon, WeaponVariantIdentity variant, bool inPowerArmor);
}
