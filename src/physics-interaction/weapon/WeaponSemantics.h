#pragma once

/*
 * Weapon semantic classification helpers are grouped here because they form one pipeline from node/body evidence to collision grouping and hull budgets.
 */


// ---- WeaponPartClassifier.h ----

#include "physics-interaction/weapon/WeaponTypes.h"

#include <algorithm>
#include <cctype>
#include <string_view>

namespace rock
{
    /*
     * The generated mesh path sees weapon parts through FO4VR NIF node names,
     * not authored ROCK data yet. This classifier is deliberately conservative:
     * it only assigns high-value semantic roles when names match common firearm
     * part terms, leaving unknown geometry as Other instead of forcing reload or
     * grip behavior onto decorative mesh.
     */

    inline bool weaponPartNameContains(std::string_view haystack, std::string_view needle)
    {
        if (needle.empty() || haystack.size() < needle.size()) {
            return false;
        }

        return std::search(haystack.begin(), haystack.end(), needle.begin(), needle.end(), [](char lhs, char rhs) {
            return std::tolower(static_cast<unsigned char>(lhs)) == std::tolower(static_cast<unsigned char>(rhs));
        }) != haystack.end();
    }

    inline WeaponPartClassification classifyWeaponPartKind(WeaponPartKind kind)
    {
        WeaponPartClassification result{};
        result.partKind = kind;

        switch (kind) {
        case WeaponPartKind::Foregrip:
            result.supportGripRole = WeaponSupportGripRole::Foregrip;
            result.fallbackGripPose = WeaponGripPoseId::VerticalForegrip;
            result.priority = 95;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Pump:
            result.supportGripRole = WeaponSupportGripRole::PumpGrip;
            result.actionRole = WeaponActionRole::Pump;
            result.fallbackGripPose = WeaponGripPoseId::PumpGrip;
            result.priority = 92;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Handguard:
            result.supportGripRole = WeaponSupportGripRole::SupportSurface;
            result.fallbackGripPose = WeaponGripPoseId::HandguardClamp;
            result.priority = 88;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Barrel:
            result.supportGripRole = WeaponSupportGripRole::SupportSurface;
            result.fallbackGripPose = WeaponGripPoseId::BarrelWrap;
            result.priority = 84;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Magwell:
            result.supportGripRole = WeaponSupportGripRole::MagwellHold;
            result.socketRole = WeaponSocketRole::Magwell;
            result.fallbackGripPose = WeaponGripPoseId::MagwellHold;
            result.priority = 82;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Magazine:
            result.reloadRole = WeaponReloadRole::MagazineBody;
            result.priority = 80;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Bolt:
            result.actionRole = WeaponActionRole::Bolt;
            result.priority = 78;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Slide:
            result.actionRole = WeaponActionRole::Slide;
            result.priority = 78;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::ChargingHandle:
            result.actionRole = WeaponActionRole::ChargingHandle;
            result.priority = 78;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::BreakAction:
            result.actionRole = WeaponActionRole::BreakAction;
            result.priority = 76;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Cylinder:
            result.actionRole = WeaponActionRole::Cylinder;
            result.socketRole = WeaponSocketRole::Cylinder;
            result.priority = 76;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Chamber:
            result.socketRole = WeaponSocketRole::Chamber;
            result.priority = 74;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::LaserCell:
            result.socketRole = WeaponSocketRole::LaserCell;
            result.reloadRole = WeaponReloadRole::AmmoPiece;
            result.priority = 74;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Lever:
            result.actionRole = WeaponActionRole::Lever;
            result.socketRole = WeaponSocketRole::LoadingGate;
            result.priority = 72;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Stock:
            result.supportGripRole = WeaponSupportGripRole::StockForward;
            result.fallbackGripPose = WeaponGripPoseId::ReceiverSupport;
            result.priority = 66;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Receiver:
            result.reloadRole = WeaponReloadRole::Receiver;
            result.supportGripRole = WeaponSupportGripRole::ReceiverSupport;
            result.fallbackGripPose = WeaponGripPoseId::ReceiverSupport;
            result.priority = 62;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Grip:
            result.priority = 56;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Shell:
        case WeaponPartKind::Round:
            result.reloadRole = WeaponReloadRole::AmmoPiece;
            result.priority = 46;
            result.gameplayCritical = true;
            break;
        case WeaponPartKind::Sight:
        case WeaponPartKind::Accessory:
            result.priority = 28;
            break;
        case WeaponPartKind::CosmeticAmmo:
            result.reloadRole = WeaponReloadRole::CosmeticAmmo;
            result.cosmetic = true;
            result.priority = 12;
            break;
        case WeaponPartKind::Other:
        default:
            result.priority = 10;
            break;
        }

        return result;
    }

    inline WeaponPartClassification classifyWeaponPartName(std::string_view sourceName)
    {
        if (weaponPartNameContains(sourceName, "magwell")) {
            return classifyWeaponPartKind(WeaponPartKind::Magwell);
        }
        if (weaponPartNameContains(sourceName, "foregrip") || weaponPartNameContains(sourceName, "verticalgrip") ||
            weaponPartNameContains(sourceName, "angledgrip")) {
            return classifyWeaponPartKind(WeaponPartKind::Foregrip);
        }
        if (weaponPartNameContains(sourceName, "handguard") || weaponPartNameContains(sourceName, "forearm")) {
            return classifyWeaponPartKind(WeaponPartKind::Handguard);
        }
        if (weaponPartNameContains(sourceName, "pump")) {
            return classifyWeaponPartKind(WeaponPartKind::Pump);
        }
        if (weaponPartNameContains(sourceName, "shell")) {
            return classifyWeaponPartKind(WeaponPartKind::Shell);
        }
        if (weaponPartNameContains(sourceName, "round") || weaponPartNameContains(sourceName, "cartridge")) {
            return classifyWeaponPartKind(WeaponPartKind::Round);
        }
        if (weaponPartNameContains(sourceName, "casing") || weaponPartNameContains(sourceName, "bullet") ||
            weaponPartNameContains(sourceName, "ammo")) {
            return classifyWeaponPartKind(WeaponPartKind::CosmeticAmmo);
        }
        if (weaponPartNameContains(sourceName, "magazine") || weaponPartNameContains(sourceName, " clip")) {
            return classifyWeaponPartKind(WeaponPartKind::Magazine);
        }
        if (weaponPartNameContains(sourceName, "charging")) {
            return classifyWeaponPartKind(WeaponPartKind::ChargingHandle);
        }
        if (weaponPartNameContains(sourceName, "bolt")) {
            return classifyWeaponPartKind(WeaponPartKind::Bolt);
        }
        if (weaponPartNameContains(sourceName, "slide")) {
            return classifyWeaponPartKind(WeaponPartKind::Slide);
        }
        if (weaponPartNameContains(sourceName, "break") || weaponPartNameContains(sourceName, "hinge")) {
            return classifyWeaponPartKind(WeaponPartKind::BreakAction);
        }
        if (weaponPartNameContains(sourceName, "cylinder")) {
            return classifyWeaponPartKind(WeaponPartKind::Cylinder);
        }
        if (weaponPartNameContains(sourceName, "chamber")) {
            return classifyWeaponPartKind(WeaponPartKind::Chamber);
        }
        if (weaponPartNameContains(sourceName, "fusioncell") || weaponPartNameContains(sourceName, "fusion cell") ||
            weaponPartNameContains(sourceName, "lasercell") || weaponPartNameContains(sourceName, "battery")) {
            return classifyWeaponPartKind(WeaponPartKind::LaserCell);
        }
        if (weaponPartNameContains(sourceName, "lever")) {
            return classifyWeaponPartKind(WeaponPartKind::Lever);
        }
        if (weaponPartNameContains(sourceName, "stock") || weaponPartNameContains(sourceName, "butt")) {
            return classifyWeaponPartKind(WeaponPartKind::Stock);
        }
        if (weaponPartNameContains(sourceName, "barrel") || weaponPartNameContains(sourceName, "muzzle") ||
            weaponPartNameContains(sourceName, "suppressor") || weaponPartNameContains(sourceName, "silencer")) {
            return classifyWeaponPartKind(WeaponPartKind::Barrel);
        }
        if (weaponPartNameContains(sourceName, "sight") || weaponPartNameContains(sourceName, "scope")) {
            return classifyWeaponPartKind(WeaponPartKind::Sight);
        }
        if (weaponPartNameContains(sourceName, "accessory") || weaponPartNameContains(sourceName, "rail")) {
            return classifyWeaponPartKind(WeaponPartKind::Accessory);
        }
        if (weaponPartNameContains(sourceName, "receiver") || weaponPartNameContains(sourceName, "frame") ||
            weaponPartNameContains(sourceName, "body") || weaponPartNameContains(sourceName, "rifle") ||
            weaponPartNameContains(sourceName, "pistol") || weaponPartNameContains(sourceName, "weapon")) {
            return classifyWeaponPartKind(WeaponPartKind::Receiver);
        }
        if (weaponPartNameContains(sourceName, "grip") || weaponPartNameContains(sourceName, "handle")) {
            return classifyWeaponPartKind(WeaponPartKind::Grip);
        }

        return classifyWeaponPartKind(WeaponPartKind::Other);
    }
}

// ---- WeaponSemanticHullBudget.h ----

#include "physics-interaction/weapon/WeaponTypes.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace rock
{
    /*
     * Weapon body budgets must preserve gameplay parts before decorative
     * coverage. A balanced geometric spread is useful for anonymous collision,
     * but reload/support/action routing depends on keeping semantic bodies such
     * as magwells, magazines, bolts, slides, pumps, and handguards alive.
     */
    struct WeaponSemanticHullBudgetInput
    {
        WeaponPartClassification semantic{};
        std::size_t sourceIndex{ 0 };
        std::uintptr_t sourceGroupId{ 0 };
        std::size_t pointCount{ 0 };
    };

    inline std::vector<std::size_t> selectSemanticWeaponHullIndices(const std::vector<WeaponSemanticHullBudgetInput>& inputs, std::size_t budget)
    {
        std::vector<WeaponSemanticHullBudgetInput> ranked = inputs;
        if (ranked.size() <= budget) {
            std::vector<std::size_t> allIndices;
            allIndices.reserve(ranked.size());
            for (const auto& input : ranked) {
                allIndices.push_back(input.sourceIndex);
            }
            return allIndices;
        }

        std::stable_sort(ranked.begin(), ranked.end(), [](const WeaponSemanticHullBudgetInput& lhs, const WeaponSemanticHullBudgetInput& rhs) {
            if (lhs.semantic.gameplayCritical != rhs.semantic.gameplayCritical) {
                return lhs.semantic.gameplayCritical && !rhs.semantic.gameplayCritical;
            }
            if (lhs.semantic.cosmetic != rhs.semantic.cosmetic) {
                return !lhs.semantic.cosmetic && rhs.semantic.cosmetic;
            }
            if (lhs.semantic.priority != rhs.semantic.priority) {
                return lhs.semantic.priority > rhs.semantic.priority;
            }
            if (lhs.pointCount != rhs.pointCount) {
                return lhs.pointCount > rhs.pointCount;
            }
            return lhs.sourceIndex < rhs.sourceIndex;
        });

        std::vector<std::size_t> selected;
        selected.reserve(budget);
        std::unordered_set<std::size_t> selectedSourceIndices;
        std::unordered_set<std::uintptr_t> selectedSourceGroups;

        auto groupIdFor = [](const WeaponSemanticHullBudgetInput& input) {
            return input.sourceGroupId != 0 ? input.sourceGroupId : (0x5747500000000000ull | static_cast<std::uintptr_t>(input.semantic.partKind));
        };

        auto trySelect = [&](const WeaponSemanticHullBudgetInput& input) {
            if (selected.size() >= budget) {
                return;
            }
            if (selectedSourceIndices.find(input.sourceIndex) != selectedSourceIndices.end()) {
                return;
            }
            selected.push_back(input.sourceIndex);
            selectedSourceIndices.insert(input.sourceIndex);
            selectedSourceGroups.insert(groupIdFor(input));
        };

        for (const auto& input : ranked) {
            const auto groupId = groupIdFor(input);
            if (!input.semantic.gameplayCritical || selectedSourceGroups.find(groupId) != selectedSourceGroups.end()) {
                continue;
            }
            trySelect(input);
        }

        for (const auto& input : ranked) {
            if (selected.size() >= budget) {
                break;
            }
            trySelect(input);
        }
        return selected;
    }
}

// ---- WeaponCollisionGroupingPolicy.h ----

#include "physics-interaction/weapon/WeaponTypes.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace rock::weapon_collision_grouping_policy
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
