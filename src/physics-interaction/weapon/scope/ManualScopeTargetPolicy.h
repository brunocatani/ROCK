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
    // A weapon without native HasScope is a scope only from this authored
    // ZOOM magnification, whatever its names or structure say: below it are
    // iron sights and red dots, including red dots whose record says "scope"
    // ("[Scope] Aimpoint PRO" at 1.5). Native scopes below it render at
    // kDefaultMagnification.
    inline constexpr float kMinimumScopeMagnification = 2.0f;
    // ZOOM alone admits a weapon without HasScope, a scope name or ScopeAiming
    // structure only from 3. Bench census of 197 guns: red dot, laser and
    // holo sights author 2.0 to 2.4; the magnified optics found by ZOOM
    // alone author 4.0 to 4.5 (hybrid, ACOG, revolver scope).
    inline constexpr float kMinimumZoomOnlyScopeMagnification = 3.0f;

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
        return std::isfinite(authoredMagnification) && authoredMagnification >= kMinimumZoomOnlyScopeMagnification;
    }

    // ROCK's own scope evidence for a weapon without native HasScope.
    [[nodiscard]] inline bool hasManualScopeEvidence(bool explicitScopeModelInstalled,
        bool magnifiedScopeStructurePresent, float authoredMagnification) noexcept
    {
        return hasScopeMagnification(authoredMagnification) ||
            ((explicitScopeModelInstalled || magnifiedScopeStructurePresent) &&
                isValidMagnification(authoredMagnification));
    }

    [[nodiscard]] inline constexpr bool isScopeEligible(bool nativeHasScope, bool manualScopeEvidence) noexcept
    {
        return nativeHasScope || manualScopeEvidence;
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
        bool manualScopeEvidence, bool validNativeOverlay) noexcept
    {
        return !nativeScopeMetadataAuthored && validNativeOverlay && manualScopeEvidence;
    }
}
