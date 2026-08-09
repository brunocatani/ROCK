#include "physics-interaction/weapon/NativeEquippedWeaponDraw.h"

#include "physics-interaction/weapon/HeldWeaponEquipStatePolicy.h"
#include "physics-interaction/weapon/WeaponTransitionAnimationAcceleration.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/Actor.h"

namespace rock::native_equipped_weapon_draw
{
    namespace
    {
        struct ExactCurrent
        {
            RE::PlayerCharacter* player{ nullptr };
            SubmitResult result{ SubmitResult::MissingPlayer };
        };

        [[nodiscard]] ExactCurrent resolveExactCurrent(
            const Identity& expected) noexcept
        {
            ExactCurrent current{};
            current.player = f4vr::getPlayer();
            if (!current.player) {
                return current;
            }

            Identity observed{};
            if (!captureCurrentIdentity(observed)) {
                current.result = SubmitResult::MissingEquippedWeapon;
                return current;
            }
            if (observed.formID != expected.formID ||
                observed.instanceData != expected.instanceData ||
                observed.equipIndex != expected.equipIndex) {
                current.result = SubmitResult::IdentityChanged;
                return current;
            }

            current.result = SubmitResult::Submitted;
            return current;
        }

        void requestAnimationAcceleration(
            RE::PlayerCharacter* player,
            const Identity& identity,
            const weapon_transition_animation_acceleration_policy::Direction
                direction) noexcept
        {
            (void)weapon_transition_animation_acceleration::request(
                weapon_transition_animation_acceleration::RequestInput{
                    .player = player,
                    .identity = {
                        .formID = identity.formID,
                        .instanceData = identity.instanceData,
                        .equipIndex = identity.equipIndex,
                    },
                    .direction = direction,
                });
        }
    }

    bool captureCurrentIdentity(Identity& outIdentity) noexcept
    {
        outIdentity = {};
        auto* equipped = f4vr::getEquippedWeaponItem();
        auto* object = equipped ? equipped->item.object : nullptr;
        auto* instanceData = equipped ? equipped->item.instanceData.get() : nullptr;
        if (!object || object->formType != RE::ENUM_FORM_ID::kWEAP) {
            return false;
        }

        outIdentity = Identity{
            .formID = object->formID,
            .instanceData = reinterpret_cast<std::uintptr_t>(instanceData),
            .equipIndex = equipped->equipIndex.index,
        };
        return true;
    }

    Result submitExactCurrent(const Identity& expected) noexcept
    {
        Result result{};
        const auto current = resolveExactCurrent(expected);
        result.result = current.result;
        if (current.result != SubmitResult::Submitted) {
            return result;
        }

        result.stateBefore = f4vr::getNativeWeaponState(current.player);
        result.stateAfter = result.stateBefore;
        if (!held_weapon_equip_state_policy::isValidNativeWeaponState(result.stateBefore)) {
            result.result = SubmitResult::InvalidWeaponState;
            return result;
        }
        if (!held_weapon_equip_state_policy::shouldSubmitDrawFollowup(result.stateBefore)) {
            if (result.stateBefore == static_cast<std::uint32_t>(
                    held_weapon_equip_state_policy::NativeWeaponState::
                        Drawing)) {
                requestAnimationAcceleration(
                    current.player,
                    expected,
                    weapon_transition_animation_acceleration_policy::
                        Direction::Draw);
            }
            result.result = SubmitResult::AlreadyDrawingOrDrawn;
            return result;
        }

        requestAnimationAcceleration(
            current.player,
            expected,
            weapon_transition_animation_acceleration_policy::Direction::Draw);
        current.player->DrawWeaponMagicHands(true);
        result.stateAfter = f4vr::getNativeWeaponState(current.player);
        result.result = SubmitResult::Submitted;
        return result;
    }

    Result submitSheatheExactCurrent(const Identity& expected) noexcept
    {
        Result result{};
        const auto current = resolveExactCurrent(expected);
        result.result = current.result;
        if (current.result != SubmitResult::Submitted) {
            return result;
        }

        result.stateBefore = f4vr::getNativeWeaponState(current.player);
        result.stateAfter = result.stateBefore;
        if (!held_weapon_equip_state_policy::isValidNativeWeaponState(result.stateBefore)) {
            result.result = SubmitResult::InvalidWeaponState;
            return result;
        }
        if (!held_weapon_equip_state_policy::shouldSubmitSheatheFollowup(result.stateBefore)) {
            if (result.stateBefore == static_cast<std::uint32_t>(
                    held_weapon_equip_state_policy::NativeWeaponState::
                        Sheathing)) {
                requestAnimationAcceleration(
                    current.player,
                    expected,
                    weapon_transition_animation_acceleration_policy::
                        Direction::Sheathe);
            }
            result.result = SubmitResult::AlreadySheathingOrSheathed;
            return result;
        }

        requestAnimationAcceleration(
            current.player,
            expected,
            weapon_transition_animation_acceleration_policy::
                Direction::Sheathe);
        current.player->DrawWeaponMagicHands(false);
        result.stateAfter = f4vr::getNativeWeaponState(current.player);
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
        case SubmitResult::AlreadySheathingOrSheathed:
            return "already-sheathing-or-sheathed";
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
