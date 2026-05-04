#pragma once

#include "WeaponSemanticTypes.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace frik::rock
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
