#include "physics-interaction/weapon/NativeEquippedWeaponDraw.h"

#include "physics-interaction/weapon/HeldWeaponEquipStatePolicy.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/Actor.h"

namespace rock::native_equipped_weapon_draw
{
    Result submitExactCurrent(const Identity& expected) noexcept
    {
        Result result{};
        auto* player = f4vr::getPlayer();
        if (!player) {
            result.result = SubmitResult::MissingPlayer;
            return result;
        }

        auto* equipped = f4vr::getEquippedItem();
        auto* object = equipped ? equipped->item.object : nullptr;
        auto* instanceData = equipped ? equipped->item.instanceData.get() : nullptr;
        if (!object || object->formType != RE::ENUM_FORM_ID::kWEAP) {
            result.result = SubmitResult::MissingEquippedWeapon;
            return result;
        }
        if (object->formID != expected.formID ||
            reinterpret_cast<std::uintptr_t>(instanceData) != expected.instanceData ||
            equipped->equipIndex.index != expected.equipIndex) {
            result.result = SubmitResult::IdentityChanged;
            return result;
        }

        result.stateBefore = static_cast<std::uint32_t>(player->weaponState);
        result.stateAfter = result.stateBefore;
        if (!held_weapon_equip_state_policy::isValidNativeWeaponState(result.stateBefore)) {
            result.result = SubmitResult::InvalidWeaponState;
            return result;
        }
        if (!held_weapon_equip_state_policy::shouldSubmitDrawFollowup(result.stateBefore)) {
            result.result = SubmitResult::AlreadyDrawingOrDrawn;
            return result;
        }

        player->DrawWeaponMagicHands(true);
        result.stateAfter = static_cast<std::uint32_t>(player->weaponState);
        result.result = SubmitResult::Submitted;
        return result;
    }

    const char* submitResultName(const SubmitResult result) noexcept
    {
        switch (result) {
        case SubmitResult::Submitted:
            return "submitted";
        case SubmitResult::AlreadyDrawingOrDrawn:
            return "already-drawing-or-drawn";
        case SubmitResult::MissingPlayer:
            return "missing-player";
        case SubmitResult::MissingEquippedWeapon:
            return "missing-equipped-weapon";
        case SubmitResult::IdentityChanged:
            return "identity-changed";
        case SubmitResult::InvalidWeaponState:
            return "invalid-weapon-state";
        default:
            return "unknown";
        }
    }
}
