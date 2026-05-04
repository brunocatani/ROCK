#pragma once

/*
 * Equipped sidearm support grip is split from full two-handed weapon authority
 * because pistols need the visual stability of a left-hand mesh grab without
 * turning the left controller into a weapon steering input. HIGGS-style full
 * two-hand solving remains the correct model for long guns, while sidearms use
 * wrist from the stored weapon-local grab frame. Runtime classification uses
 * the weapon animation grip keywords because Fallout 4 only exposes "gun" at
 * the weapon-type enum level, while actual vanilla and modded pistol records
 * carry `AnimsGripPistol`. Long-gun grip keywords are checked before the
 * sidearm signal so rifle-like weapons keep the solver that owns weapon
 * transform authority. `BGSKeywordForm::HasKeyword(..., TBO_InstanceData*)`
 * preserves weapon-mod instance changes. Name fallback tokens only cover
 * vanilla sidearms whose records do not expose a stronger semantic signal.
 */

#include "TransformMath.h"

#include <array>
#include <cstdint>
#include <string_view>

namespace frik::rock::weapon_support_authority_policy
{
    enum class WeaponSupportAuthorityMode
    {
        FullTwoHandedSolver = 0,
        VisualOnlySupport = 1,
    };

    enum class WeaponSupportWeaponClass
    {
        Unknown = 0,
        Sidearm = 1,
        LongGun = 2,
    };

    struct EquippedWeaponIdentity
    {
        std::uint32_t formID{ 0 };
        std::string_view displayName{};
        std::string_view nodeName{};
        bool hasPistolGripKeyword{ false };
        bool hasInstancePistolGripKeyword{ false };
        bool hasLongGunGripKeyword{ false };
        bool hasInstanceLongGunGripKeyword{ false };
    };

    inline constexpr bool hasPistolGripSemantic(const EquippedWeaponIdentity& identity)
    {
        return identity.hasPistolGripKeyword || identity.hasInstancePistolGripKeyword;
    }

    inline constexpr bool hasLongGunGripSemantic(const EquippedWeaponIdentity& identity)
    {
        return identity.hasLongGunGripKeyword || identity.hasInstanceLongGunGripKeyword;
    }

    inline constexpr WeaponSupportAuthorityMode resolveSupportAuthorityMode(
        bool visualOnlySidearmSupportEnabled,
        WeaponSupportWeaponClass weaponClass)
    {
        return visualOnlySidearmSupportEnabled && weaponClass == WeaponSupportWeaponClass::Sidearm ?
                   WeaponSupportAuthorityMode::VisualOnlySupport :
                   WeaponSupportAuthorityMode::FullTwoHandedSolver;
    }

    inline constexpr bool supportGripOwnsWeaponTransform(WeaponSupportAuthorityMode mode)
    {
        return mode == WeaponSupportAuthorityMode::FullTwoHandedSolver;
    }

    inline constexpr bool supportGripAppliesPrimaryHandAuthority(WeaponSupportAuthorityMode mode)
    {
        return mode == WeaponSupportAuthorityMode::FullTwoHandedSolver;
    }

    inline constexpr bool supportGripAppliesSupportHandAuthority(WeaponSupportAuthorityMode)
    {
        return true;
    }

    template <class Transform>
    inline Transform buildVisualOnlySupportHandWorld(const Transform& weaponWorld, const Transform& supportHandWeaponLocal)
    {
        return transform_math::composeTransforms(weaponWorld, supportHandWeaponLocal);
    }

    inline char lowerAscii(char value)
    {
        return value >= 'A' && value <= 'Z' ? static_cast<char>(value - 'A' + 'a') : value;
    }

    inline std::string_view trimAscii(std::string_view value)
    {
        while (!value.empty() && (value.front() == ' ' || value.front() == '\t' || value.front() == '\r' || value.front() == '\n')) {
            value.remove_prefix(1);
        }
        while (!value.empty() && (value.back() == ' ' || value.back() == '\t' || value.back() == '\r' || value.back() == '\n')) {
            value.remove_suffix(1);
        }
        return value;
    }

    inline bool containsIgnoreCase(std::string_view haystack, std::string_view needle)
    {
        needle = trimAscii(needle);
        if (needle.empty() || needle.size() > haystack.size()) {
            return false;
        }

        for (std::size_t start = 0; start + needle.size() <= haystack.size(); ++start) {
            bool matched = true;
            for (std::size_t index = 0; index < needle.size(); ++index) {
                if (lowerAscii(haystack[start + index]) != lowerAscii(needle[index])) {
                    matched = false;
                    break;
                }
            }
            if (matched) {
                return true;
            }
        }
        return false;
    }

    inline bool identityNameMatchesAny(const EquippedWeaponIdentity& identity, const std::string_view* tokens, std::size_t tokenCount)
    {
        for (std::size_t index = 0; index < tokenCount; ++index) {
            if (containsIgnoreCase(identity.displayName, tokens[index]) || containsIgnoreCase(identity.nodeName, tokens[index])) {
                return true;
            }
        }
        return false;
    }

    inline WeaponSupportWeaponClass classifyEquippedWeaponForSupportGrip(const EquippedWeaponIdentity& identity)
    {
        if (hasLongGunGripSemantic(identity)) {
            return WeaponSupportWeaponClass::LongGun;
        }
        if (hasPistolGripSemantic(identity)) {
            return WeaponSupportWeaponClass::Sidearm;
        }

        static constexpr std::array<std::string_view, 8> kSidearmNameFallbackTokens{
            "pistol",
            "revolver",
            "deliverer",
            "alien blaster",
            "gamma gun",
            "flare gun",
            "the gainer",
            "western revolver",
        };
        if (identityNameMatchesAny(identity, kSidearmNameFallbackTokens.data(), kSidearmNameFallbackTokens.size())) {
            return WeaponSupportWeaponClass::Sidearm;
        }

        static constexpr std::array<std::string_view, 11> kLongGunTokens{
            "rifle",
            "shotgun",
            "musket",
            "launcher",
            "minigun",
            "fat man",
            "flamer",
            "gatling",
            "harpoon",
            "submachine",
            "machine gun",
        };
        if (identityNameMatchesAny(identity, kLongGunTokens.data(), kLongGunTokens.size())) {
            return WeaponSupportWeaponClass::LongGun;
        }

        return WeaponSupportWeaponClass::Unknown;
    }
}
