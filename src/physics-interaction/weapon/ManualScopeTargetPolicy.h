#pragma once

#include <cstdint>
#include <string_view>

#include "physics-interaction/weapon/WeaponEffectGeometryPolicy.h"

namespace rock::manual_scope_target_policy
{
    inline constexpr std::uint32_t kMaximumNativeOverlayIndex = 16;

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
        return overlayIndex <= kMaximumNativeOverlayIndex;
    }

    /*
     * Some weapon authors ship a magnified optic model but omit Fallout's
     * native scope-overlay OMOD property. The native property remains the
     * authority when present. Explicit scope naming is the cheap path; the
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
        return containsAsciiInsensitive(recordName, "scope") ||
               containsAsciiInsensitive(modelPath, "scope");
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
