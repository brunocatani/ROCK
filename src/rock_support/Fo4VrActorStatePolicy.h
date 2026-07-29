#pragma once

#include <cstddef>
#include <cstdint>

namespace rock::fo4vr_actor_state_policy
{
    /*
     * Verified against Fallout4VR.exe 1.2.72:
     *
     * - PlayerCharacter::DrawWeaponMagicHands (0x140F78D10) reads the dword at
     *   PlayerCharacter + 0x134, shifts right by two, and masks with 0x7.
     * - ActorState::SetWeaponState (0x140E77090) writes the same three bits
     *   through mask 0x1C at ActorState + 0x0C.
     * - The native gun-state setter (0x140E77100) reads and writes mask
     *   0x78000 at shift 15. ReloadStateChangeHandler (0x140FF2B90)
     *   independently submits native gun state 4 for reload begin.
     *
     * CommonLibF4VR currently declares the named weaponState bitfield one bit
     * earlier and its later gunState field is consequently shifted as well.
     * Runtime code must decode the verified storage instead of reading either
     * bitfield or GetWeaponMagicDrawn().
     */
    inline constexpr std::size_t kWeaponStateStorageOffset = 0x0C;
    inline constexpr std::uint32_t kWeaponStateShift = 2;
    inline constexpr std::uint32_t kWeaponStateValueMask = 0x7;
    inline constexpr std::uint32_t kGunStateShift = 15;
    inline constexpr std::uint32_t kGunStateValueMask = 0xF;
    inline constexpr std::uint32_t kInvalidWeaponState = 0xFFFFFFFFu;
    inline constexpr std::uint32_t kInvalidGunState = 0xFFFFFFFFu;

    [[nodiscard]] inline constexpr std::uint32_t decodeWeaponState(
        const std::uint32_t actorStateStorage) noexcept
    {
        return (actorStateStorage >> kWeaponStateShift) &
               kWeaponStateValueMask;
    }

    [[nodiscard]] inline constexpr std::uint32_t decodeGunState(
        const std::uint32_t actorStateStorage) noexcept
    {
        return (actorStateStorage >> kGunStateShift) &
               kGunStateValueMask;
    }

    [[nodiscard]] inline constexpr bool isWeaponMagicDrawn(
        const std::uint32_t weaponState) noexcept
    {
        // Native states 3..5 are Drawn, WantToSheathe, and Sheathing.
        return weaponState >= 3 && weaponState <= 5;
    }
}
