#include "physics-interaction/weapon/NativeEquippedWeaponDraw.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/HeldWeaponEquipStatePolicy.h"
#include "physics-interaction/weapon/WeaponTransitionAnimationAcceleration.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/Actor.h"

#include <array>

namespace rock::native_equipped_weapon_draw
{
    namespace
    {
        constexpr std::array<std::uint8_t, 7> kExpectedPrepareDrawEntry{
            0x80, 0x89, 0xA3, 0x12, 0x00, 0x00, 0x80,
        };

        struct ExactCurrent
        {
            RE::PlayerCharacter* player{ nullptr };
            SubmitResult result{ SubmitResult::MissingPlayer };
        };

        [[nodiscard]] bool prepareDrawEntryMatchesVerifiedRuntime() noexcept
        {
            static const bool matches = []() noexcept {
                if (!REL::Module::IsVR() ||
                    REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
                    return false;
                }

                std::array<std::uint8_t, kExpectedPrepareDrawEntry.size()> actual{};
                const auto address = REL::Offset(
                    offsets::kFunc_PrepareEquippedWeaponDraw).address();
                return native_memory::guardedCopyFromMemory(
                           reinterpret_cast<const void*>(address),
                           actual.data(),
                           actual.size()) &&
                       actual == kExpectedPrepareDrawEntry;
            }();
            return matches;
        }

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

        [[nodiscard]] Result submitResolvedDraw(
            const ExactCurrent& current,
            const Identity& expected,
            const bool prepareEquipRecovery) noexcept
        {
            Result result{};
            result.result = current.result;
            if (current.result != SubmitResult::Submitted) {
                return result;
            }

            result.stateBefore = f4vr::getNativeWeaponState(current.player);
            result.stateAfter = result.stateBefore;
            if (!held_weapon_equip_state_policy::isValidNativeWeaponState(
                    result.stateBefore)) {
                result.result = SubmitResult::InvalidWeaponState;
                return result;
            }
            if (!held_weapon_equip_state_policy::shouldSubmitDrawFollowup(
                    result.stateBefore)) {
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

            if (prepareEquipRecovery) {
                if (result.stateBefore != static_cast<std::uint32_t>(
                        held_weapon_equip_state_policy::NativeWeaponState::
                            Sheathed)) {
                    result.result = SubmitResult::RecoveryStateChanged;
                    return result;
                }
                if (!prepareDrawEntryMatchesVerifiedRuntime()) {
                    result.result =
                        SubmitResult::RecoveryPreparationUnavailable;
                    return result;
                }

                using PrepareEquippedWeaponDraw = void (*)(
                    RE::PlayerCharacter*);
                const auto prepare = reinterpret_cast<PrepareEquippedWeaponDraw>(
                    REL::Offset(
                        offsets::kFunc_PrepareEquippedWeaponDraw).address());
                prepare(current.player);
            }

            requestAnimationAcceleration(
                current.player,
                expected,
                weapon_transition_animation_acceleration_policy::Direction::
                    Draw);
            current.player->DrawWeaponMagicHands(true);
            result.stateAfter = f4vr::getNativeWeaponState(current.player);
            /*
             * FO4VR 1.2.72 at 0x140F78D10 changes the native weapon state
             * synchronously whenever ActionDraw is accepted. An unchanged
             * retryable state therefore means the action was rejected; the
             * void CommonLib boundary must not be reported as success.
             */
            result.result = result.stateAfter == result.stateBefore ?
                SubmitResult::NativeActionRejected :
                SubmitResult::Submitted;
            return result;
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
        const auto current = resolveExactCurrent(expected);
        return submitResolvedDraw(current, expected, false);
    }

    Result submitPreparedExactCurrent(const Identity& expected) noexcept
    {
        const auto current = resolveExactCurrent(expected);
        return submitResolvedDraw(current, expected, true);
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
        result.result = result.stateAfter == result.stateBefore ?
            SubmitResult::NativeActionRejected :
            SubmitResult::Submitted;
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
        case SubmitResult::RecoveryPreparationUnavailable:
            return "recovery-preparation-unavailable";
        case SubmitResult::RecoveryStateChanged:
            return "recovery-state-changed";
        case SubmitResult::NativeActionRejected:
            return "native-action-rejected";
        default:
            return "unknown";
        }
    }
}
