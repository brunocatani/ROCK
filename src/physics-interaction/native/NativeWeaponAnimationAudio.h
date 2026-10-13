#pragma once

#include <string_view>

namespace rock::native_weapon_animation_audio
{
    [[nodiscard]] constexpr bool equalName(std::string_view a, std::string_view b) noexcept
    {
        if (a.size() != b.size()) return false;
        for (std::size_t i = 0; i < a.size(); ++i) {
            const auto lower = [](char c) { return c >= 'A' && c <= 'Z' ? c + ('a' - 'A') : c; };
            if (lower(a[i]) != lower(b[i])) return false;
        }
        return true;
    }

    [[nodiscard]] constexpr bool isSoundStart(std::string_view tag) noexcept
    {
        return equalName(tag, "SoundPlay") || equalName(tag, "SoundPlayAt");
    }

    [[nodiscard]] constexpr bool isEquipClip(std::string_view path) noexcept
    {
        const auto separator = path.find_last_of("/\\");
        if (separator != std::string_view::npos) path.remove_prefix(separator + 1);
        // The live graph reports HKT clip names even for resolved HKX assets;
        // its 1HM transition also uses EquipFast without the WPN prefix.
        if (path.size() < 4) return false;
        const auto extension = path.substr(path.size() - 4);
        if (!equalName(extension, ".hkx") && !equalName(extension, ".hkt")) return false;
        path.remove_suffix(4);
        if (path.size() >= 3 && equalName(path.substr(0, 3), "WPN")) path.remove_prefix(3);
        return equalName(path, "Equip") || equalName(path, "EquipFast") || equalName(path, "UnEquip");
    }

    // Filter the external sound notification copy, after Havok has queued it.
    // The original behavior events and their payload ownership remain native.
    [[nodiscard]] bool install() noexcept;
}
