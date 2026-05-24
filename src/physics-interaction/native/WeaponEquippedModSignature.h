#pragma once

#include <cstdint>

namespace rock
{
    struct EquippedWeaponModSignature
    {
        std::uint64_t key{ 0 };
        std::uint32_t formID{ 0 };
        std::uintptr_t instanceDataAddress{ 0 };
        std::uintptr_t objectInstanceExtraAddress{ 0 };
        std::uint64_t objectIndexDataSignature{ 0 };
        std::uint32_t objectIndexDataCount{ 0 };
        std::uint32_t activeModCount{ 0 };
        std::uint32_t disabledModCount{ 0 };
        std::uintptr_t equippedDataAddress{ 0 };
        std::uintptr_t equippedObjectAddress{ 0 };
        bool hasEquippedWeapon{ false };
    };

    [[nodiscard]] EquippedWeaponModSignature readEquippedWeaponModSignature();
}
