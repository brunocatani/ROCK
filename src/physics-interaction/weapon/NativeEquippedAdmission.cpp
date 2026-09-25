#include "physics-interaction/weapon/NativeEquippedAdmission.h"
#include "physics-interaction/weapon/NativeEquippedModelSlot.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Bethesda/TESBoundObjects.h"

namespace rock
{
    bool NativeEquippedAdmission::restoreOriginalMagazine() noexcept
    {
        native_equipped_weapon::Snapshot current;
        if (!native_equipped_weapon::read(0, current) || current.identity.form != _original.identity.form ||
            current.identity.instance != _original.identity.instance) return false;
        const auto exactStack = weapon_equip_transfer::captureEquippedInventoryWeapon(0);
        if (!exactStack.stack || exactStack.stack != _selection.stack) return false;
        return native_equipped_weapon::restoreMagazine(current.identity, _ammo, _loaded);
    }

    NativeEquippedAdmission::Result NativeEquippedAdmission::begin(RE::TESObjectWEAP* incoming) noexcept
    {
        if (_pending || !incoming || incoming->weaponData.type != RE::WEAPON_TYPE::kGun ||
            !frik_visual_authority::canBlockSecondaryWeaponNodeOwnership() ||
            !weapon_equip_transfer::canRecoverHeldEquip() || !native_equipped_weapon::slotEmpty(1)) return Result::Unavailable;
        native_equipped_weapon::Snapshot original;
        if (!native_equipped_weapon::read(0, original) || !original.attached) return Result::Unavailable;
        auto* weapon = static_cast<RE::TESObjectWEAP*>(original.item.item.object);
        auto* data = static_cast<RE::EquippedWeaponData*>(original.item.data.get());
        if (weapon->weaponData.type != RE::WEAPON_TYPE::kGun || !data->ammo) return Result::Unavailable;
        const auto selection = weapon_equip_transfer::captureEquippedInventoryWeapon(0);
        if (!selection.stack || !native_equipped_weapon::handSlot(0) || !native_equipped_weapon::handSlot(1) ||
            !native_equipped_model_slot::reserve(incoming)) return Result::Unavailable;
        _original = std::move(original);
        _selection = selection;
        _ammo = data->ammo->formID;
        _loaded = data->ammoCount;
        _incoming = incoming->formID;
        _pending = true;
        _reslotted = _original.item.equipSlot != native_equipped_weapon::handSlot(0);
        if (_reslotted) {
            // E72DA0 resolves slot conflicts through E70560, creates the new
            // native record, then applies its inventory flags through E71920.
            // One complete manager transaction keeps that native ordering;
            // an extra preliminary unequip could merge the selected stack.
            const auto result = weapon_equip_transfer::equipInventoryWeapon(_selection, 0);
            if (!result.success || !restoreOriginalMagazine()) return recover();
        }
        ROCK_LOG_INFO(Weapon, "Native akimbo admission prepared original={:08X} instance={:#x} loaded={} incoming={:08X} reslotted={}",
            _original.identity.form, _original.identity.instance, _loaded, _incoming, _reslotted);
        return Result::Ready;
    }

    NativeEquippedAdmission::Result NativeEquippedAdmission::finish(bool incomingCommitted) noexcept
    {
        if (!_pending) return Result::Unavailable;
        native_equipped_weapon::Snapshot first, second;
        const bool pair = incomingCommitted && native_equipped_weapon::read(0, first) && native_equipped_weapon::read(1, second) &&
            first.identity.form == _original.identity.form && first.identity.instance == _original.identity.instance &&
            second.identity.form == _incoming && first.identity.data != second.identity.data &&
            first.item.equipSlot == native_equipped_weapon::handSlot(0) && second.item.equipSlot == native_equipped_weapon::handSlot(1);
        if (!pair || !restoreOriginalMagazine()) return recover();
        const auto leftSelection = weapon_equip_transfer::captureEquippedInventoryWeapon(1);
        if (!leftSelection.stack || leftSelection.stack == _selection.stack) return recover();
        const auto* secondData = static_cast<const RE::EquippedWeaponData*>(second.item.data.get());
        ROCK_LOG_INFO(Weapon, "Native akimbo inventory committed records=2 first={:08X}/{:#x}/{} second={:08X}/{:#x}/{} stacks=distinct",
            first.identity.form, first.identity.data, _loaded, second.identity.form, second.identity.data, secondData->ammoCount);
        _pending = false;
        _reslotted = false;
        // Keep the original model pinned until the caller installs its visual
        // continuity bridge. abandonAfterGameLoad also serves as explicit clear.
        return Result::Ready;
    }

    NativeEquippedAdmission::Result NativeEquippedAdmission::recover() noexcept
    {
        if (!_pending) return Result::Unavailable;
        native_equipped_weapon::Snapshot second;
        if (native_equipped_weapon::read(1, second)) {
            if (second.identity.form != _incoming || !weapon_equip_transfer::unequipExactIndexedWeapon(
                    1, second.identity.form, second.identity.instance)) return Result::RecoveryRequired;
        } else if (!native_equipped_weapon::slotEmpty(1)) return Result::RecoveryRequired;
        if (_reslotted) {
            native_equipped_weapon::Snapshot first;
            if (native_equipped_weapon::read(0, first)) {
                if (first.identity.form != _original.identity.form || first.identity.instance != _original.identity.instance) return Result::RecoveryRequired;
            } else if (!native_equipped_weapon::slotEmpty(0)) return Result::RecoveryRequired;
            if (!weapon_equip_transfer::equipInventoryWeapon(_selection,UINT32_MAX).success) return Result::RecoveryRequired;
        }
        if (!restoreOriginalMagazine()) return Result::RecoveryRequired;
        ROCK_LOG_WARN(Weapon, "Native akimbo admission compensated original={:08X} loaded={}; incoming remains in inventory if pickup committed",
            _original.identity.form, _loaded);
        _pending = false;
        _reslotted = false;
        (void)native_equipped_model_slot::releaseIfUnused();
        return Result::OriginalRestored;
    }

    void NativeEquippedAdmission::abandonAfterGameLoad() noexcept
    {
        _original = {};
        _selection = {};
        _incoming = _ammo = _loaded = 0;
        _pending = _reslotted = false;
    }
}
