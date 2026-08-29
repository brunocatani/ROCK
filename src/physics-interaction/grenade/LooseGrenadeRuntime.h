#pragma once

#include "RE/Bethesda/BSTSmartPointer.h"
#include "RE/Bethesda/BSPointerHandle.h"

#include "physics-interaction/grenade/LooseThrowablePolicy.h"

#include <cstdint>

namespace RE
{
    class BGSExplosion;
    class BGSProjectile;
    class TESObjectREFR;
    class TESObjectWEAP;
    class TBO_InstanceData;
    class NiPoint3;
}

namespace rock::loose_grenade_runtime
{
    enum class GrenadeDetonationMode : std::uint8_t
    {
        Unsupported,
        TimedFuse,
        Impact,
        Proximity,
    };

    struct GrenadeRuntimeData
    {
        RE::BGSProjectile* projectile{ nullptr };
        RE::BGSExplosion* explosion{ nullptr };
        float fuseSeconds{ 0.0f };
        float proximityRadiusGameUnits{ 0.0f };
        float directImpactDamage{ 0.0f };
        bool preserveReferenceAfterDetonation{ false };
        GrenadeDetonationMode detonationMode{ GrenadeDetonationMode::TimedFuse };
    };

    struct ProximityScanResult
    {
        bool targetFound{ false };
        bool truncated{ false };
        std::uint32_t targetActorFormID{ 0 };
        std::uint32_t actorHandlesScanned{ 0 };
    };

    enum class EquippedGrenadeSelectionStatus : std::uint8_t
    {
        Selected,
        PlayerUnavailable,
        NoneEquipped,
        AmbiguousEquipped,
        InvalidRuntimeData
    };

    struct EquippedGrenadeSelection
    {
        std::uint64_t requestId{ 0 };
        RE::TESObjectWEAP* weapon{ nullptr };
        RE::BSTSmartPointer<RE::TBO_InstanceData> instanceData{};
        std::uint32_t stackId{ 0xFFFF'FFFFu };
        GrenadeRuntimeData runtime{};
    };

    struct DropResult
    {
        bool success{ false };
        const char* reason{ "not-attempted" };
        RE::ObjectRefHandle handle{};
        RE::TESObjectREFR* droppedRef{ nullptr };
        std::uint32_t stackId{ 0 };
    };

    [[nodiscard]] bool isThrowableWeapon(const RE::TESObjectWEAP* weapon) noexcept;
    [[nodiscard]] bool isThrowableRef(RE::TESObjectREFR* ref) noexcept;
    [[nodiscard]] bool resolveGrenadeRuntimeData(
        RE::TESObjectWEAP* weapon,
        RE::TBO_InstanceData* instanceData,
        GrenadeRuntimeData& outRuntime) noexcept;
    [[nodiscard]] bool resolveGrenadeRuntimeDataForReference(
        RE::TESObjectREFR* ref,
        GrenadeRuntimeData& outRuntime) noexcept;

    [[nodiscard]] EquippedGrenadeSelectionStatus resolveEquippedGrenadeSelection(
        EquippedGrenadeSelection& outSelection) noexcept;
    [[nodiscard]] const char* selectionStatusName(EquippedGrenadeSelectionStatus status) noexcept;
    [[nodiscard]] DropResult dropEquippedGrenadeSelectionToWorld(
        const EquippedGrenadeSelection& selection,
        const RE::NiPoint3& dropLocation);

    [[nodiscard]] bool createExplosionAtReference(RE::TESObjectREFR* ref, RE::BGSExplosion* explosion);
    [[nodiscard]] const char* detonationModeName(GrenadeDetonationMode mode) noexcept;
    [[nodiscard]] ProximityScanResult scanHostileActorsWithinProximity(
        RE::TESObjectREFR* ref,
        float radiusGameUnits) noexcept;
    [[nodiscard]] bool playPinPulledFeedbackAtReference(RE::TESObjectREFR* ref);
    [[nodiscard]] bool returnDroppedReferenceToInventory(RE::TESObjectREFR* ref);
    void disableAndDeleteReference(RE::TESObjectREFR* ref);
}
