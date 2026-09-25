#pragma once
#include <string_view>
#include <span>
#include <cstdint>
#include "physics-interaction/TransformMath.h"

namespace rock::weapon_cycle_policy
{
    enum class PresentationEvent { Ignore, Sound, HidePart, ShowPart };
    constexpr PresentationEvent presentationEvent(std::string_view tag) noexcept
    {
        if (tag == "CullBone") return PresentationEvent::HidePart;
        if (tag == "UnCullBone" || tag == "UncullBone") return PresentationEvent::ShowPart;
        if (tag == "SoundPlay" || tag == "SoundPlayAt" || tag == "SoundStop" || tag == "SoundFadeOut") return PresentationEvent::Sound;
        return PresentationEvent::Ignore;
    }
    template<class Transform>
    Transform retargetPart(const Transform& modelRest, const Transform& inverseClipRest, const Transform& sample)
    {
        return transform_math::composeTransforms(modelRest, transform_math::composeTransforms(inverseClipRest, sample));
    }
    constexpr bool clipNameIs(std::string_view path, std::string_view target) noexcept
    {
        path.remove_prefix(path.find_last_of("/\\") == path.npos ? 0 : path.find_last_of("/\\") + 1);
        if (const auto dot = path.find_last_of('.'); dot != path.npos) path = path.substr(0, dot);
        if (path.size() != target.size()) return false;
        for (std::size_t i = 0; i < path.size(); ++i) {
            const char c = path[i] >= 'A' && path[i] <= 'Z' ? path[i] + ('a' - 'A') : path[i];
            if (c != target[i]) return false;
        }
        return true;
    }
    constexpr unsigned reloadClipPriority(std::string_view path) noexcept
    {
        return clipNameIs(path, "wpnreload") ? 2 : clipNameIs(path, "wpnreloadready") ? 1 : 0;
    }
    constexpr unsigned emptyReloadClipPriority(std::string_view path) noexcept
    {
        return clipNameIs(path, "wpnreloadempty") ? 2 : clipNameIs(path, "wpnreloademptyready") ? 1 : 0;
    }
    constexpr unsigned reserveReloadClipPriority(std::string_view path) noexcept
    {
        return clipNameIs(path, "wpnreloadreserve") ? 1 : 0;
    }
    // Single-shot clips contain the complete mechanical stroke. Replaying an
    // auto forward/back blend clip as a stroke would leave the slide displaced.
    constexpr unsigned fireClipPriority(std::string_view path) noexcept
    {
        const auto equals = [path](std::string_view target) { return clipNameIs(path, target); };
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
