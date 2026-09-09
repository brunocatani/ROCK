#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>
#include <string_view>

#include "physics-interaction/weapon/WeaponEffectGeometryPolicy.h"

namespace rock::manual_scope_target_policy
{
    inline constexpr std::uint32_t kMaximumNativeOverlayIndex = 16;
    // Fallout4.esm:000AF2A9, ZM_Standard_Scope_x4.
    inline constexpr std::uint32_t kDefaultOverlayIndex = 6;
    inline constexpr float kDefaultMagnification = 4.0f;

    struct StructuralMarkerEvidence
    {
        bool scopeAiming{ false };
        bool scopeViewParts{ false };
        bool scopeFade{ false };
    };

    [[nodiscard]] inline constexpr bool nodeBaseNameEquals(
        std::string_view nodeName,
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

    inline constexpr void observeStructuralNodeName(
        StructuralMarkerEvidence& evidence,
        std::string_view nodeName) noexcept
    {
        evidence.scopeAiming = evidence.scopeAiming || nodeBaseNameEquals(nodeName, "ScopeAiming");
        evidence.scopeViewParts = evidence.scopeViewParts || nodeBaseNameEquals(nodeName, "ScopeViewParts");
        evidence.scopeFade = evidence.scopeFade || nodeBaseNameEquals(nodeName, "ScopeFade");
    }

    [[nodiscard]] inline constexpr bool hasMagnifiedScopeStructure(
        const StructuralMarkerEvidence& evidence) noexcept
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
        return std::isfinite(magnification) && magnification > 1.0f;
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
        return 2.0f * std::atan(std::tan(fieldOfViewDegrees * radiansPerDegree * 0.5f) /
                   resolveMagnification(magnification)) / radiansPerDegree;
    }

    [[nodiscard]] inline constexpr bool isScopeEligible(
        bool nativeHasScope, bool explicitScopeModelInstalled, bool magnifiedScopeStructurePresent) noexcept
    {
        return nativeHasScope || explicitScopeModelInstalled || magnifiedScopeStructurePresent;
    }

    [[nodiscard]] inline constexpr bool nativeHasScope(
        bool hasInstance, bool instanceHasScope, bool baseHasScope) noexcept
    {
        return hasInstance ? instanceHasScope : baseHasScope;
    }

    /*
     * Some weapon authors ship a magnified optic model but omit Fallout's
     * native HasScope flag. The resolved instance flag remains the authority
     * for native activation. Explicit scope naming is the cheap path; the
     * structural fallback above recognizes the native ScopeAiming plus
     * ScopeViewParts/ScopeFade contract used by generically named magnified
     * optics. Generic sights and red dots satisfy neither contract.
     */
    [[nodiscard]] inline constexpr bool hasExplicitScopeIdentity(
        std::string_view recordName,
        std::string_view modelPath) noexcept
    {
        if (modelPath.empty()) {
            return false;
        }

        using weapon_effect_geometry_policy::containsAsciiInsensitive;
        if (containsAsciiInsensitive(recordName, "scope")) {
            return true;
        }
        // A weapon-folder name containing "scope" is not evidence that every
        // receiver/barrel/stock underneath it is an installed optic.
        const auto lastSeparator = modelPath.find_last_of("/\\");
        const auto fileName = lastSeparator == std::string_view::npos ? modelPath : modelPath.substr(lastSeparator + 1);
        if (containsAsciiInsensitive(fileName, "scope")) {
            return true;
        }
        while (lastSeparator != std::string_view::npos && !modelPath.empty()) {
            const auto separator = modelPath.find_first_of("/\\");
            const auto component = modelPath.substr(0, separator);
            if (nodeBaseNameEquals(component, "scope") || nodeBaseNameEquals(component, "scopes")) {
                return true;
            }
            if (separator == std::string_view::npos) {
                break;
            }
            modelPath.remove_prefix(separator + 1);
        }
        return false;
    }

    [[nodiscard]] inline constexpr bool requiresDirectNativeTransition(
        bool nativeScopeMetadataAuthored,
        bool explicitScopeModelInstalled,
        bool magnifiedScopeStructurePresent,
        bool validNativeOverlay) noexcept
    {
        return !nativeScopeMetadataAuthored &&
               validNativeOverlay &&
               (explicitScopeModelInstalled || magnifiedScopeStructurePresent);
    }
}
