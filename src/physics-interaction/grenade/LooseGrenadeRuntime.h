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
    struct GrenadeRuntimeData
    {
        RE::BGSProjectile* projectile{ nullptr };
        RE::BGSExplosion* explosion{ nullptr };
        float fuseSeconds{ 0.0f };
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
    [[nodiscard]] bool resolveGrenadeRuntimeData(
        RE::TESObjectWEAP* weapon,
        RE::TBO_InstanceData* instanceData,
        GrenadeRuntimeData& outRuntime) noexcept;
    [[nodiscard]] bool resolveGrenadeRuntimeDataForReference(
        RE::TESObjectREFR* ref,
        GrenadeRuntimeData& outRuntime) noexcept;

    [[nodiscard]] bool copyOldestPendingEquipRequest(PendingEquipRequest& outRequest);
    void discardPendingEquipRequest(std::uint64_t requestId);

    [[nodiscard]] DropResult dropPendingEquipRequestToWorld(
        const PendingEquipRequest& request,
        const RE::NiPoint3& dropLocation,
        const RE::NiPoint3* dropRotation);

    [[nodiscard]] bool createExplosionAtReference(RE::TESObjectREFR* ref, RE::BGSExplosion* explosion);
    void disableAndDeleteReference(RE::TESObjectREFR* ref);
}
