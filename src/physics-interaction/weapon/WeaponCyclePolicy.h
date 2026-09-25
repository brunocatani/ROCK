#pragma once
#include <string_view>
#include <span>
#include <cstdint>
#include <algorithm>
#include <cmath>

namespace rock::weapon_cycle_policy
{
    inline float playbackRate(float clipSeconds, float shotSeconds) noexcept
    {
        if (!std::isfinite(clipSeconds) || !std::isfinite(shotSeconds) || clipSeconds <= 0 || shotSeconds <= 0) return 1.0f;
        const float rate = clipSeconds / shotSeconds;
        return std::isfinite(rate) ? (std::max)(1.0f, rate) : 1.0f;
    }
    // Single-shot clips contain the complete mechanical stroke. Replaying an
    // auto forward/back blend clip as a stroke would leave the slide displaced.
    constexpr unsigned fireClipPriority(std::string_view path) noexcept
    {
        path.remove_prefix(path.find_last_of("/\\") == path.npos ? 0 : path.find_last_of("/\\") + 1);
        const auto dot = path.find_last_of('.');
        if (dot != path.npos) path = path.substr(0, dot);
        const auto equals = [path](std::string_view target) {
            if (path.size() != target.size()) return false;
            for (std::size_t i = 0; i < path.size(); ++i) {
                const char c = path[i] >= 'A' && path[i] <= 'Z' ? path[i] + ('a' - 'A') : path[i];
                if (c != target[i]) return false;
            }
            return true;
        };
        if (equals("wpnfiresingleready")) return 3;
        if (equals("wpnfiresinglereadya")) return 2;
        if (equals("wpnfiresinglereadyb")) return 1;
        return 0;
    }

    // Reject the Weapon root, body bones, malformed parents and cycles.
    constexpr bool isWeaponPart(int bone, int weapon, std::span<const std::int16_t> parents) noexcept
    {
        if (bone < 0 || bone == weapon || weapon < 0) return false;
        for (std::size_t depth = 0; depth < 64; ++depth) {
            if (bone < 0 || static_cast<std::size_t>(bone) >= parents.size()) return false;
            bone = parents[bone];
            if (bone == weapon) return true;
        }
        return false;
    }
}
