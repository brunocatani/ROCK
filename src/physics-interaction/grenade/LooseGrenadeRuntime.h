#pragma once

#include "RE/Bethesda/BSTSmartPointer.h"
#include "RE/Bethesda/BSPointerHandle.h"

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
        TimedFuse,
        Impact
    };

    enum class GrenadeKind : std::uint8_t
    {
        NotGrenade,
        Generic,
        Molotov
    };

    struct GrenadeRuntimeData
    {
        RE::BGSProjectile* projectile{ nullptr };
        RE::BGSExplosion* explosion{ nullptr };
        float fuseSeconds{ 0.0f };
        GrenadeDetonationMode detonationMode{ GrenadeDetonationMode::TimedFuse };
    };

    struct PendingEquipRequest
    {
        bool active{ false };
        std::uint64_t requestId{ 0 };
        RE::TESObjectWEAP* weapon{ nullptr };
        RE::BSTSmartPointer<RE::TBO_InstanceData> instanceData{};
        std::uint32_t stackId{ 0 };
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

    [[nodiscard]] bool installEquipHook();

    [[nodiscard]] bool isGrenadeWeapon(const RE::TESObjectWEAP* weapon) noexcept;
    [[nodiscard]] bool isGrenadeRef(RE::TESObjectREFR* ref) noexcept;
    [[nodiscard]] GrenadeKind classifyGrenadeRef(RE::TESObjectREFR* ref) noexcept;
    [[nodiscard]] bool resolveGrenadeRuntimeData(
        RE::TESObjectWEAP* weapon,
        RE::TBO_InstanceData* instanceData,
        GrenadeRuntimeData& outRuntime) noexcept;
    [[nodiscard]] bool resolveGrenadeRuntimeDataForReference(
        RE::TESObjectREFR* ref,
        GrenadeRuntimeData& outRuntime) noexcept;

    [[nodiscard]] bool copyPendingEquipRequest(PendingEquipRequest& outRequest);
    [[nodiscard]] bool hasPendingEquipRequest();
    void discardPendingEquipRequest(std::uint64_t requestId);
    void clearPendingEquipRequest();

    [[nodiscard]] DropResult dropPendingEquipRequestToWorld(
        const PendingEquipRequest& request,
        const RE::NiPoint3& dropLocation);

    [[nodiscard]] bool createExplosionAtReference(RE::TESObjectREFR* ref, RE::BGSExplosion* explosion);
    [[nodiscard]] const char* detonationModeName(GrenadeDetonationMode mode) noexcept;
    [[nodiscard]] bool playPinPulledFeedbackAtReference(RE::TESObjectREFR* ref);
    [[nodiscard]] bool returnDroppedReferenceToInventory(RE::TESObjectREFR* ref);
    void disableAndDeleteReference(RE::TESObjectREFR* ref);
}
