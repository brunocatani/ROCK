#pragma once

#include <cstdint>

namespace rock::native_equipped_weapon_draw
{
    struct Identity
    {
        std::uint32_t formID{ 0 };
        std::uintptr_t instanceData{ 0 };
        std::uint32_t equipIndex{ 0 };
    };

    enum class SubmitResult : std::uint8_t
    {
        Submitted = 0,
        AlreadyDrawingOrDrawn,
        MissingPlayer,
        MissingEquippedWeapon,
        IdentityChanged,
        InvalidWeaponState,
    };

    struct Result
    {
        SubmitResult result{ SubmitResult::MissingPlayer };
        std::uint32_t stateBefore{ 0 };
        std::uint32_t stateAfter{ 0 };
    };

    [[nodiscard]] Result submitExactCurrent(const Identity& expected) noexcept;
    [[nodiscard]] const char* submitResultName(SubmitResult result) noexcept;
}
