#pragma once

#include "physics-interaction/weapon/NativeEquippedWeapon.h"
#include "physics-interaction/weapon/WeaponEquipTransfer.h"

namespace rock
{
    // Frame-thread transaction. The caller owns hand/grip continuity. This
    // object owns the exact inventory selection and magazine across native
    // reslotting, including compensation when the second equip is declined.
    class NativeEquippedAdmission
    {
    public:
        enum class Result { Ready, Unavailable, OriginalRestored, RecoveryRequired };
        [[nodiscard]] Result begin(RE::TESObjectWEAP* incoming) noexcept;
        [[nodiscard]] Result finish(bool incomingCommitted) noexcept;
        [[nodiscard]] Result recover() noexcept;
        [[nodiscard]] bool pending() const noexcept { return _pending; }
        const native_equipped_weapon::Snapshot& original() const noexcept { return _original; }
        std::uint32_t originalLoaded() const noexcept { return _loaded; }
        void abandonAfterGameLoad() noexcept;

    private:
        [[nodiscard]] bool restoreOriginalMagazine() noexcept;
        native_equipped_weapon::Snapshot _original{};
        weapon_equip_transfer::InventorySelection _selection{};
        std::uint32_t _incoming{}, _ammo{}, _loaded{};
        bool _pending{}, _reslotted{};
    };
}
