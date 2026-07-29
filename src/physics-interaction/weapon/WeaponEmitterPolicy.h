#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>

#include "physics-interaction/weapon/WeaponEffectGeometryPolicy.h"

namespace rock::weapon_emitter_policy
{
    enum class Kind : std::uint32_t
    {
        Unknown = 0,
        Flashlight = 1,
        Laser = 2,
        Reticle = 3,
    };

    enum class Source : std::uint32_t
    {
        Unknown = 0,
        EffectGeometry = 1,
        AddOnNode = 2,
    };

    struct ClassificationEvidence
    {
        std::string_view nodeName{};
        bool effectGeometry{ false };
        bool valueNode{ false };
        bool laserContext{ false };
        bool flashlightContext{ false };
        bool sightContext{ false };
    };

    struct AddOnNodeValue
    {
        std::uint32_t value{ 0 };
        bool valid{ false };
    };

    [[nodiscard]] inline constexpr bool hasLaserRoleName(std::string_view name) noexcept
    {
        using weapon_effect_geometry_policy::containsAsciiInsensitive;
        return containsAsciiInsensitive(name, "lasersight") ||
               containsAsciiInsensitive(name, "laserbeam") ||
               containsAsciiInsensitive(name, "laserdot") ||
               containsAsciiInsensitive(name, "laserray");
    }

    [[nodiscard]] inline constexpr bool hasFlashlightRoleName(std::string_view name) noexcept
    {
        using weapon_effect_geometry_policy::containsAsciiInsensitive;
        return containsAsciiInsensitive(name, "flashlight") ||
               containsAsciiInsensitive(name, "weaponlight") ||
               containsAsciiInsensitive(name, "screengloweffect") ||
               containsAsciiInsensitive(name, "gloweffect") ||
               containsAsciiInsensitive(name, "lightfx");
    }

    [[nodiscard]] inline constexpr bool hasReticleRoleName(std::string_view name) noexcept
    {
        using weapon_effect_geometry_policy::containsAsciiInsensitive;
        return containsAsciiInsensitive(name, "reticle") ||
               containsAsciiInsensitive(name, "parallax") ||
               containsAsciiInsensitive(name, "tritium") ||
               containsAsciiInsensitive(name, "trit");
    }

    [[nodiscard]] inline constexpr Kind classify(const ClassificationEvidence& evidence) noexcept
    {
        const bool directLaser = hasLaserRoleName(evidence.nodeName);
        const bool directFlashlight = hasFlashlightRoleName(evidence.nodeName);
        const bool directReticle = hasReticleRoleName(evidence.nodeName);

        if (evidence.valueNode) {
            if (directLaser || evidence.laserContext) {
                return Kind::Laser;
            }
            if (directFlashlight || evidence.flashlightContext) {
                return Kind::Flashlight;
            }
            return Kind::Unknown;
        }

        if (!evidence.effectGeometry) {
            return Kind::Unknown;
        }
        if (directLaser) {
            return Kind::Laser;
        }
        if (directFlashlight) {
            return Kind::Flashlight;
        }
        if (directReticle) {
            return Kind::Reticle;
        }
        if (evidence.laserContext) {
            return Kind::Laser;
        }
        if (evidence.flashlightContext) {
            return Kind::Flashlight;
        }
        if (evidence.sightContext) {
            return Kind::Reticle;
        }
        return Kind::Unknown;
    }

    /*
     * Weapon NIFs conventionally encode the BSValueNode payload in names such
     * as AddOnNode130. CommonLibF4VR exposes the RTTI but no verified class
     * layout, so the authored decimal suffix is the safe V1 source of truth.
     */
    [[nodiscard]] inline constexpr AddOnNodeValue parseAddOnNodeValue(std::string_view name) noexcept
    {
        constexpr std::string_view prefix = "AddOnNode";
        if (name.size() <= prefix.size() ||
            !weapon_effect_geometry_policy::equalsAsciiInsensitive(name.substr(0, prefix.size()), prefix)) {
            return {};
        }

        std::uint64_t value = 0;
        std::size_t digitCount = 0;
        for (std::size_t i = prefix.size(); i < name.size(); ++i) {
            const char ch = name[i];
            if (ch < '0' || ch > '9') {
                break;
            }
            value = value * 10u + static_cast<std::uint64_t>(ch - '0');
            ++digitCount;
            if (value > (std::numeric_limits<std::uint32_t>::max)()) {
                return {};
            }
        }
        return digitCount != 0 ? AddOnNodeValue{ static_cast<std::uint32_t>(value), true } : AddOnNodeValue{};
    }

    [[nodiscard]] inline constexpr std::uint32_t transformPriority(Kind kind, Source source, std::string_view nodeName) noexcept
    {
        if (source == Source::AddOnNode) {
            return 400;
        }
        if (kind == Kind::Laser) {
            if (weapon_effect_geometry_policy::containsAsciiInsensitive(nodeName, "beam") ||
                weapon_effect_geometry_policy::containsAsciiInsensitive(nodeName, "ray")) {
                return 300;
            }
            if (weapon_effect_geometry_policy::containsAsciiInsensitive(nodeName, "dot")) {
                return 100;
            }
        }
        if (kind == Kind::Reticle) {
            if (weapon_effect_geometry_policy::containsAsciiInsensitive(nodeName, "reticle")) {
                return 300;
            }
            if (weapon_effect_geometry_policy::containsAsciiInsensitive(nodeName, "trit")) {
                return 250;
            }
            if (weapon_effect_geometry_policy::containsAsciiInsensitive(nodeName, "parallax")) {
                return 150;
            }
            return 100;
        }
        return source == Source::EffectGeometry ? 200 : 0;
    }
}
