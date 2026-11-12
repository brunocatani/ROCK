// Which weapons get ROCK's manual scope: native scope identity, magnified scope
// structure, overlay index and magnification.
#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>
#include <string_view>

#include "physics-interaction/weapon/classification/WeaponEffectGeometryPolicy.h"

namespace rock::manual_scope_target_policy
{
    inline constexpr std::uint32_t kMaximumNativeOverlayIndex = 16;
    // Fallout4.esm:000AF2A9, ZM_Standard_Scope_x4.
    inline constexpr std::uint32_t kDefaultOverlayIndex = 6;
    inline constexpr float kDefaultMagnification = 4.0f;
    // Authored ZOOM magnification below 2 is iron-sight or red-dot aim zoom
    // (bench census: 1.0 to 1.5), never a scope's own value. From 2 up it is
    // a scope's magnification, which also admits a weapon that lacks the
    // native HasScope flag; below it the scope view shows kDefaultMagnification.
    inline constexpr float kMinimumScopeMagnification = 2.0f;

    [[nodiscard]] inline constexpr bool matchesNativeIdentity(std::uintptr_t publishedWeapon,
        std::uintptr_t publishedInstance, std::uintptr_t nativeWeapon, std::uintptr_t nativeInstance) noexcept
    {
        return publishedWeapon != 0 && publishedWeapon == nativeWeapon && publishedInstance == nativeInstance;
    }

    struct StructuralMarkerEvidence
    {
        bool scopeAiming{ false };
        bool scopeViewParts{ false };
        bool scopeFade{ false };
    };

    [[nodiscard]] inline constexpr bool nodeBaseNameEquals(std::string_view nodeName,
        std::string_view expectedBaseName) noexcept
    {
        const auto suffix = nodeName.find(':');
        const std::string_view baseName = nodeName.substr(0, suffix);
        if (baseName.size() != expectedBaseName.size()) {
            return false;
        }

        for (std::size_t i = 0; i < baseName.size(); ++i) {
            const auto lowerAscii = [](const char c) constexpr {
                return c >= 'A' && c <= 'Z' ? static_cast<char>(c + ('a' - 'A')) : c;
            };
            if (lowerAscii(baseName[i]) != lowerAscii(expectedBaseName[i])) {
                return false;
            }
        }
        return true;
    }

    inline constexpr void observeStructuralNodeName(StructuralMarkerEvidence& evidence,
        std::string_view nodeName) noexcept
    {
        evidence.scopeAiming = evidence.scopeAiming || nodeBaseNameEquals(nodeName, "ScopeAiming");
        evidence.scopeViewParts = evidence.scopeViewParts || nodeBaseNameEquals(nodeName, "ScopeViewParts");
        evidence.scopeFade = evidence.scopeFade || nodeBaseNameEquals(nodeName, "ScopeFade");
    }

    [[nodiscard]] inline constexpr bool hasMagnifiedScopeStructure(const StructuralMarkerEvidence& evidence) noexcept
    {
        return evidence.scopeAiming && (evidence.scopeViewParts || evidence.scopeFade);
    }

    [[nodiscard]] inline constexpr bool isValidNativeOverlayIndex(const std::uint32_t overlayIndex) noexcept
    {
        return overlayIndex > 0 && overlayIndex <= kMaximumNativeOverlayIndex;
    }

    [[nodiscard]] inline constexpr std::uint32_t resolveOverlay(std::uint32_t authoredOverlay) noexcept
    {
        return isValidNativeOverlayIndex(authoredOverlay) ? authoredOverlay : kDefaultOverlayIndex;
    }

    [[nodiscard]] inline bool isValidMagnification(float magnification) noexcept
    {
        return std::isfinite(magnification) && magnification >= kMinimumScopeMagnification;
    }

    [[nodiscard]] inline float resolveMagnification(float authoredMagnification) noexcept
    {
        return isValidMagnification(authoredMagnification) ? authoredMagnification : kDefaultMagnification;
    }

    [[nodiscard]] inline float magnifiedFieldOfView(float fieldOfViewDegrees, float magnification) noexcept
    {
        if (!std::isfinite(fieldOfViewDegrees) || fieldOfViewDegrees <= 0.0f || fieldOfViewDegrees >= 180.0f) {
            return 0.0f;
        }
        constexpr float radiansPerDegree = std::numbers::pi_v<float> / 180.0f;
        return 2.0f *
            std::atan(std::tan(fieldOfViewDegrees * radiansPerDegree * 0.5f) / resolveMagnification(magnification)) /
            radiansPerDegree;
    }

    [[nodiscard]] inline bool hasScopeMagnification(float authoredMagnification) noexcept
    {
        return std::isfinite(authoredMagnification) && authoredMagnification >= kMinimumScopeMagnification;
    }

    [[nodiscard]] inline constexpr bool isScopeEligible(bool nativeHasScope, bool explicitScopeModelInstalled,
        bool magnifiedScopeStructurePresent, bool scopeMagnificationAuthored) noexcept
    {
        return nativeHasScope || explicitScopeModelInstalled || magnifiedScopeStructurePresent ||
            scopeMagnificationAuthored;
    }

    [[nodiscard]] inline constexpr bool nativeHasScope(bool hasInstance, bool instanceHasScope,
        bool baseHasScope) noexcept
    {
        return hasInstance ? instanceHasScope : baseHasScope;
    }

    /*
     * Some weapon authors ship a magnified optic model but omit Fallout's
     * native HasScope flag. The resolved instance flag remains the authority
     * for native activation. An installed attachment with a model whose record
     * name says "scope" is the cheap path; the structural check above
     * recognizes the native ScopeAiming plus ScopeViewParts/ScopeFade contract
     * of generically named magnified optics. Model paths are not evidence:
     * Fallout keeps iron and reflex sights in "Scopes" folders, and the vanilla
     * reflex sight model is HandmadeScope.nif.
     */
    [[nodiscard]] inline constexpr bool hasExplicitScopeIdentity(std::string_view recordName,
        std::string_view modelPath) noexcept
    {
        return !modelPath.empty() && weapon_effect_geometry_policy::containsAsciiInsensitive(recordName, "scope");
    }

    [[nodiscard]] inline constexpr bool requiresDirectNativeTransition(bool nativeScopeMetadataAuthored,
        bool explicitScopeModelInstalled, bool magnifiedScopeStructurePresent, bool scopeMagnificationAuthored,
        bool validNativeOverlay) noexcept
    {
        return !nativeScopeMetadataAuthored && validNativeOverlay &&
            (explicitScopeModelInstalled || magnifiedScopeStructurePresent || scopeMagnificationAuthored);
    }
}
