#pragma once

#include <cstdint>

namespace rock::native_equipped_weapon_attach
{
    struct Identity
    {
        std::uint32_t formID{ 0 };
        std::uintptr_t instanceData{ 0 };
        std::uint32_t equipIndex{ 0 };
    };

    enum class SubmitResult : std::uint8_t
    {
        Submitted,
        UnsupportedRuntime,
        ContractMismatch,
        MissingPlayer,
        MissingEquippedWeapon,
        IdentityChanged,
        MissingManager,
    };

    [[nodiscard]] SubmitResult submitExactCurrent(const Identity& expected) noexcept;
    [[nodiscard]] const char* submitResultName(SubmitResult result) noexcept;
}
