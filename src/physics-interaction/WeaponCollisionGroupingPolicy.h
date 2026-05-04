#pragma once

#include "WeaponPartClassifier.h"
#include "WeaponSemanticTypes.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace frik::rock::weapon_collision_grouping_policy
{
    /*
     * Weapon mesh collision is generated from runtime visual geometry because
     * FO4VR firearm collision is often incomplete. Mode 1 is the production
     * default because the working pre-refactor build grouped render chunks under
     * semantic part nodes without making large cross-weapon convex hulls. Mode 2
     * remains available for explicit PAPER evidence experiments, but it must not
     * be the fallback for missing or invalid config because it can merge distant
     * authored helpers into slanted-looking generated bodies.
     */
    enum class WeaponCollisionGroupingMode : std::uint8_t
    {
        LegacyTriShape = 0,
        SemanticPartNode = 1,
        CompoundSemanticPart = 2
    };

    inline constexpr int kDefaultWeaponCollisionGroupingMode = static_cast<int>(WeaponCollisionGroupingMode::SemanticPartNode);

    inline constexpr WeaponCollisionGroupingMode sanitizeWeaponCollisionGroupingMode(int mode)
    {
        switch (mode) {
        case static_cast<int>(WeaponCollisionGroupingMode::LegacyTriShape):
            return WeaponCollisionGroupingMode::LegacyTriShape;
        case static_cast<int>(WeaponCollisionGroupingMode::SemanticPartNode):
            return WeaponCollisionGroupingMode::SemanticPartNode;
        case static_cast<int>(WeaponCollisionGroupingMode::CompoundSemanticPart):
            return WeaponCollisionGroupingMode::CompoundSemanticPart;
        default:
            return WeaponCollisionGroupingMode::SemanticPartNode;
        }
    }

    inline constexpr const char* weaponCollisionGroupingModeName(WeaponCollisionGroupingMode mode)
    {
        switch (mode) {
        case WeaponCollisionGroupingMode::LegacyTriShape:
            return "LegacyTriShape";
        case WeaponCollisionGroupingMode::SemanticPartNode:
            return "SemanticPartNode";
        case WeaponCollisionGroupingMode::CompoundSemanticPart:
            return "CompoundSemanticPart";
        default:
            return "CompoundSemanticPart";
        }
    }

    inline constexpr bool weaponCollisionGroupingModeChanged(int cachedMode, int currentMode)
    {
        return sanitizeWeaponCollisionGroupingMode(cachedMode) != sanitizeWeaponCollisionGroupingMode(currentMode);
    }

    inline constexpr bool usesSingleWeaponPackageDriveRoot(WeaponCollisionGroupingMode mode)
    {
        switch (sanitizeWeaponCollisionGroupingMode(static_cast<int>(mode))) {
        case WeaponCollisionGroupingMode::LegacyTriShape:
        case WeaponCollisionGroupingMode::SemanticPartNode:
        case WeaponCollisionGroupingMode::CompoundSemanticPart:
        default:
            return true;
        }
    }

    inline constexpr std::uint64_t weaponCollisionCandidateScore(std::size_t hullCount, std::uint32_t triangleCount)
    {
        constexpr std::uint64_t kTriangleCoverageWeight = 1'000'000ull;
        constexpr std::uint64_t kMaxFragmentationPenalty = 999'999ull;
        const std::uint64_t cappedHullPenalty = (std::min)(static_cast<std::uint64_t>(hullCount), kMaxFragmentationPenalty);
        return static_cast<std::uint64_t>(triangleCount) * kTriangleCoverageWeight + (kMaxFragmentationPenalty - cappedHullPenalty);
    }

    inline constexpr bool excludeFromSemanticWeaponCollision(const WeaponPartClassification& semantic)
    {
        return semantic.partKind == WeaponPartKind::Round || semantic.partKind == WeaponPartKind::Shell || semantic.partKind == WeaponPartKind::CosmeticAmmo;
    }

    inline constexpr bool startsSemanticWeaponPartGroup(const WeaponPartClassification& semantic)
    {
        return semantic.partKind != WeaponPartKind::Other && !excludeFromSemanticWeaponCollision(semantic);
    }

    inline constexpr bool semanticChildSplitsFromParent(const WeaponPartClassification& parent, const WeaponPartClassification& child)
    {
        if (!startsSemanticWeaponPartGroup(child)) {
            return false;
        }
        if (!startsSemanticWeaponPartGroup(parent)) {
            return true;
        }
        return parent.partKind != child.partKind;
    }

    inline bool asciiStartsWithNoCase(std::string_view value, std::string_view prefix)
    {
        if (prefix.empty() || value.size() < prefix.size()) {
            return false;
        }

        for (std::size_t i = 0; i < prefix.size(); ++i) {
            if (std::tolower(static_cast<unsigned char>(value[i])) != std::tolower(static_cast<unsigned char>(prefix[i]))) {
                return false;
            }
        }
        return true;
    }

    inline bool isAuthoredWeaponExtraNode(std::string_view sourceName)
    {
        return asciiStartsWithNoCase(sourceName, "WeaponExtra");
    }

    inline bool isMagazineMeshAlias(std::string_view sourceName)
    {
        if (!asciiStartsWithNoCase(sourceName, "Mag")) {
            return false;
        }
        if (asciiStartsWithNoCase(sourceName, "Magnum")) {
            return false;
        }
        if (weaponPartNameContains(sourceName, "bullet") || weaponPartNameContains(sourceName, "casing") ||
            weaponPartNameContains(sourceName, "ammo") || weaponPartNameContains(sourceName, "shell") ||
            weaponPartNameContains(sourceName, "round") || weaponPartNameContains(sourceName, "moonclip")) {
            return false;
        }
        return true;
    }

    inline WeaponPartClassification classifyCompoundSemanticWeaponPartName(std::string_view sourceName)
    {
        const auto lexical = classifyWeaponPartName(sourceName);
        if (excludeFromSemanticWeaponCollision(lexical)) {
            return lexical;
        }
        if (weaponPartNameContains(sourceName, "moonclip")) {
            return classifyWeaponPartKind(WeaponPartKind::CosmeticAmmo);
        }
        if (isAuthoredWeaponExtraNode(sourceName)) {
            return classifyWeaponPartKind(WeaponPartKind::Other);
        }
        if (isMagazineMeshAlias(sourceName)) {
            return classifyWeaponPartKind(WeaponPartKind::Magazine);
        }
        return lexical;
    }

    inline bool excludeFromCompoundSemanticWeaponCollision(const WeaponPartClassification& semantic)
    {
        return excludeFromSemanticWeaponCollision(semantic);
    }

    inline bool startsCompoundSemanticWeaponPartGroup(const WeaponPartClassification& semantic)
    {
        return startsSemanticWeaponPartGroup(semantic);
    }

    inline bool compoundSemanticChildSplitsFromParent(const WeaponPartClassification& parent, const WeaponPartClassification& child)
    {
        if (!startsCompoundSemanticWeaponPartGroup(child)) {
            return false;
        }
        if (!startsCompoundSemanticWeaponPartGroup(parent)) {
            return true;
        }
        return parent.partKind != child.partKind;
    }

    inline bool compoundSemanticPartsMayShareBody(const WeaponPartClassification& existing, const WeaponPartClassification& incoming)
    {
        if (!startsCompoundSemanticWeaponPartGroup(existing) || !startsCompoundSemanticWeaponPartGroup(incoming)) {
            return false;
        }
        return existing.partKind == incoming.partKind && existing.reloadRole == incoming.reloadRole &&
               existing.supportGripRole == incoming.supportGripRole && existing.socketRole == incoming.socketRole &&
               existing.actionRole == incoming.actionRole;
    }

}
