#pragma once

#include <cstdint>

namespace rock::held_weapon_equip_state_policy
{
    /*
     * Numeric values mirror RE::WEAPON_STATE. Keeping the policy independent
     * of engine headers lets the transition rules remain constexpr-testable;
     * the runtime converts the typed CommonLib value at the call boundary.
     */
    enum class NativeWeaponState : std::uint32_t
    {
        Sheathed = 0,
        WantToDraw = 1,
        Drawing = 2,
        Drawn = 3,
        WantToSheathe = 4,
        Sheathing = 5,
    };

    enum class EquipReadiness : std::uint8_t
    {
        Stable,
        Transitioning,
        Invalid,
    };

    [[nodiscard]] inline constexpr EquipReadiness classifyForEquip(const std::uint32_t nativeState) noexcept
    {
        switch (static_cast<NativeWeaponState>(nativeState)) {
        case NativeWeaponState::Sheathed:
        case NativeWeaponState::Drawn:
            return EquipReadiness::Stable;
        case NativeWeaponState::WantToDraw:
        case NativeWeaponState::Drawing:
        case NativeWeaponState::WantToSheathe:
        case NativeWeaponState::Sheathing:
            return EquipReadiness::Transitioning;
        default:
            return EquipReadiness::Invalid;
        }
    }

    [[nodiscard]] inline constexpr bool canBeginEquip(const std::uint32_t nativeState) noexcept
    {
        return classifyForEquip(nativeState) == EquipReadiness::Stable;
    }

    [[nodiscard]] inline constexpr bool shouldRearmTrigger(
        const std::uint32_t nativeState,
        const bool sameHandTriggerRequest) noexcept
    {
        return sameHandTriggerRequest && classifyForEquip(nativeState) == EquipReadiness::Transitioning;
    }

    [[nodiscard]] inline constexpr bool isValidNativeWeaponState(
        const std::uint32_t nativeState) noexcept
    {
        return nativeState <= static_cast<std::uint32_t>(NativeWeaponState::Sheathing);
    }

    /*
     * FO4VR PlayerCharacter::DrawWeaponMagicHands(true) returns immediately
     * only for Drawing/Drawn. Sheathed, WantToSheathe, and Sheathing are
     * valid submissions; the latter two reverse an in-progress holster. The
     * coordinator retries this request only through a bounded state machine.
     */
    [[nodiscard]] inline constexpr bool shouldSubmitDrawFollowup(
        const std::uint32_t nativeState) noexcept
    {
        switch (static_cast<NativeWeaponState>(nativeState)) {
        case NativeWeaponState::Sheathed:
        case NativeWeaponState::WantToDraw:
        case NativeWeaponState::WantToSheathe:
        case NativeWeaponState::Sheathing:
            return true;
        case NativeWeaponState::Drawing:
        case NativeWeaponState::Drawn:
        default:
            return false;
        }
    }

    /*
     * FO4VR PlayerCharacter::DrawWeaponMagicHands(false) returns immediately
     * only for Sheathed/Sheathing. Raw 1.2.72 disassembly at 0x140F78D10
     * shows that WantToDraw, Drawing, Drawn, and WantToSheathe all submit
     * native state 5, so a physical stash can safely reverse an in-flight
     * draw without removing the equipped inventory instance.
     */
    [[nodiscard]] inline constexpr bool shouldSubmitSheatheFollowup(
        const std::uint32_t nativeState) noexcept
    {
        switch (static_cast<NativeWeaponState>(nativeState)) {
        case NativeWeaponState::WantToDraw:
        case NativeWeaponState::Drawing:
        case NativeWeaponState::Drawn:
        case NativeWeaponState::WantToSheathe:
            return true;
        case NativeWeaponState::Sheathed:
        case NativeWeaponState::Sheathing:
        default:
            return false;
        }
    }

    [[nodiscard]] inline constexpr bool isShoulderStashedPresentationState(
        const std::uint32_t nativeState) noexcept
    {
        switch (static_cast<NativeWeaponState>(nativeState)) {
        case NativeWeaponState::Sheathed:
        case NativeWeaponState::WantToSheathe:
        case NativeWeaponState::Sheathing:
            return true;
        default:
            return false;
        }
    }

    [[nodiscard]] inline constexpr const char* nativeWeaponStateName(const std::uint32_t nativeState) noexcept
    {
        switch (static_cast<NativeWeaponState>(nativeState)) {
        case NativeWeaponState::Sheathed:
            return "sheathed";
        case NativeWeaponState::WantToDraw:
            return "want-to-draw";
        case NativeWeaponState::Drawing:
            return "drawing";
        case NativeWeaponState::Drawn:
            return "drawn";
        case NativeWeaponState::WantToSheathe:
            return "want-to-sheathe";
        case NativeWeaponState::Sheathing:
            return "sheathing";
        default:
            return "invalid";
        }
    }
}
