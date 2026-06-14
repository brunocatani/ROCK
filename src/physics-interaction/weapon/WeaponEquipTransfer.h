#pragma once

#include "RE/Bethesda/TESForms.h"

#include <cstdint>

namespace RE
{
    class TESObjectREFR;
    class TESObjectWEAP;
}

namespace rock::weapon_equip_transfer
{
    enum class EquipReason : std::uint8_t
    {
        NotAttempted = 0,
        MissingRef,
        MissingPlayer,
        MissingEquipManager,
        DeletedOrDisabled,
        PlayerRef,
        MissingBaseForm,
        UnsupportedBaseForm,
        MissingEquipSlot,
        ActivateRefFailed,
        MissingInventoryList,
        InventoryStackNotFound,
        EquipObjectFailed,
        ActivateRefThenEquipObject,
    };

    struct EquipInput
    {
        RE::TESObjectREFR* heldRef = nullptr;
    };

    struct EquipResult
    {
        bool attempted = false;
        bool transferredToInventory = false;
        bool success = false;
        bool matchedInstanceData = false;
        EquipReason reason = EquipReason::NotAttempted;
        std::int32_t count = 1;
        std::uint32_t formID = 0;
        std::uint32_t stackID = 0;
        RE::TESObjectWEAP* weapon = nullptr;
    };

    [[nodiscard]] const char* equipReasonName(EquipReason reason) noexcept;
    [[nodiscard]] EquipResult transferHeldWeaponToPlayerAndEquip(const EquipInput& input) noexcept;
}
