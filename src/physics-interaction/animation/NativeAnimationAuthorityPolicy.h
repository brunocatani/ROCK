#pragma once

#include <cstdint>
#include <string_view>

namespace rock::native_animation_authority_policy
{
    inline constexpr std::uint32_t kArms = 1u << 0;
    inline constexpr std::uint32_t kHands = 1u << 1;
    inline constexpr std::uint32_t kWeapon = 1u << 2;
    inline constexpr std::uint32_t kReloadPose = kArms | kHands | kWeapon;

    [[nodiscard]] constexpr char asciiLower(char value)
    {
        return value >= 'A' && value <= 'Z' ? static_cast<char>(value - 'A' + 'a') : value;
    }

    [[nodiscard]] constexpr bool equalsIgnoreCase(std::string_view lhs, std::string_view rhs)
    {
        if (lhs.size() != rhs.size()) {
            return false;
        }
        for (std::size_t i = 0; i < lhs.size(); ++i) {
            if (asciiLower(lhs[i]) != asciiLower(rhs[i])) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] constexpr bool startsWithIgnoreCase(std::string_view value, std::string_view prefix)
    {
        return value.size() >= prefix.size() && equalsIgnoreCase(value.substr(0, prefix.size()), prefix);
    }

    /*
     * Only the two arm chains and the two weapon roots are eligible. The hand
     * root participates in both Arms and Hands so either partial request has a
     * stable hierarchy boundary. Finger/thumb descendants belong to Hands;
     * no root, COM, spine, head, or leg transform can enter this authority.
     */
    [[nodiscard]] constexpr std::uint32_t classifyBone(std::string_view name)
    {
        if (equalsIgnoreCase(name, "Weapon") || equalsIgnoreCase(name, "WeaponLeft")) {
            return kWeapon;
        }

        constexpr std::string_view leftArmPrefix = "LArm_";
        constexpr std::string_view rightArmPrefix = "RArm_";
        std::string_view suffix;
        if (startsWithIgnoreCase(name, leftArmPrefix)) {
            suffix = name.substr(leftArmPrefix.size());
        } else if (startsWithIgnoreCase(name, rightArmPrefix)) {
            suffix = name.substr(rightArmPrefix.size());
        } else {
            return 0;
        }

        if (equalsIgnoreCase(suffix, "Hand")) {
            return kArms | kHands;
        }
        if (startsWithIgnoreCase(suffix, "Finger") || startsWithIgnoreCase(suffix, "Thumb")) {
            return kHands;
        }
        return kArms;
    }

    [[nodiscard]] constexpr bool isRequested(std::uint32_t boneFlags, std::uint32_t requestedFlags)
    {
        return (boneFlags & requestedFlags & kReloadPose) != 0;
    }
}
