#pragma once

#include "physics-interaction/weapon/WeaponTypes.h"

namespace rock::weapon_recoil_policy
{
    enum class Profile : std::uint8_t
    {
        OneHand,
        FullTwoHand,
        CloseSupport,
        PowerArmor,
        Bipod,
    };

    struct ProfileGains
    {
        float translation;
        float rotation;
    };

    // Independent profiles: tuning one hold must never retune another.
    inline constexpr ProfileGains kOneHand{ 1.0f, 1.0f };
    inline constexpr ProfileGains kFullTwoHand{ 1.0f, 1.0f };
    inline constexpr ProfileGains kCloseSupport{ 0.45f, 0.30f };
    inline constexpr ProfileGains kPowerArmor{ 0.45f, 0.30f };
    inline constexpr ProfileGains kBipod{ 0.10f, 0.10f };

    [[nodiscard]] inline constexpr Profile selectProfile(
        const bool inPowerArmor, const bool closeSupport, const bool fullTwoHanded,
        const bool surfaceLatched = false) noexcept
    {
        return surfaceLatched ? Profile::Bipod :
               inPowerArmor ? Profile::PowerArmor :
               closeSupport ? Profile::CloseSupport :
               fullTwoHanded ? Profile::FullTwoHand : Profile::OneHand;
    }

    [[nodiscard]] inline constexpr ProfileGains gainsFor(const Profile profile) noexcept
    {
        switch (profile) {
        case Profile::Bipod: return kBipod;
        case Profile::PowerArmor: return kPowerArmor;
        case Profile::CloseSupport: return kCloseSupport;
        case Profile::FullTwoHand: return kFullTwoHand;
        default: return kOneHand;
        }
    }

    [[nodiscard]] inline constexpr const char* name(const Profile profile) noexcept
    {
        switch (profile) {
        case Profile::Bipod: return "bipod";
        case Profile::PowerArmor: return "power-armor";
        case Profile::CloseSupport: return "close-support";
        case Profile::FullTwoHand: return "full-two-hand";
        default: return "one-hand";
        }
    }

    enum class Family : std::uint8_t { Default, Pistol, Rifle, Shotgun, Heavy };

    struct WeaponEvidence
    {
        std::uint32_t formID{ 0 };
        std::uint64_t keywordFlags{ 0 };
        WeaponSizeClass sizeClass{ WeaponSizeClass::Rifle };
        WeaponClassificationSource source{ WeaponClassificationSource::None };
        bool resolved{ false };
    };

    [[nodiscard]] inline constexpr Family classifyFamily(const WeaponEvidence& weapon) noexcept
    {
        if (!weapon.resolved || weapon.sizeClass == WeaponSizeClass::Melee) {
            return Family::Default;
        }
        if (weapon.sizeClass == WeaponSizeClass::Heavy) {
            return Family::Heavy;
        }
        if (hasWeaponKeywordFlag(weapon.keywordFlags, WeaponKeywordFlag::Shotgun)) {
            return Family::Shotgun;
        }
        return weapon.sizeClass == WeaponSizeClass::Pistol ? Family::Pistol : Family::Rifle;
    }

    [[nodiscard]] inline constexpr const char* name(const Family family) noexcept
    {
        switch (family) {
        case Family::Pistol: return "pistol";
        case Family::Rifle: return "rifle";
        case Family::Shotgun: return "shotgun";
        case Family::Heavy: return "heavy";
        default: return "default";
        }
    }

    // The physical support state selects the percentage independently of the
    // recoil delivery/profile (provider visual support can retain OneHand's
    // geometric profile while still being a supported hold).
    [[nodiscard]] inline constexpr float selectHoldPercent(const bool oneHanded,
        const float oneHandPercent, const float twoHandPercent) noexcept
    {
        return oneHanded ? oneHandPercent : twoHandPercent;
    }

    // Percent has already been validated by ROCK's central INI loader.
    // Armor and a latched bipod are umbrella overrides. Neither inherits a
    // family/hand multiplier; bipod is always 10% of the native kick.
    [[nodiscard]] inline constexpr ProfileGains effectiveGains(const Profile profile, const float percent) noexcept
    {
        const auto base = gainsFor(profile);
        if (profile == Profile::PowerArmor || profile == Profile::Bipod) {
            return base;
        }
        const float multiplier = percent * 0.01f;
        return { base.translation * multiplier, base.rotation * multiplier };
    }
}
