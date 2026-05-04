#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstddef>

namespace frik::rock::hand_collision_suppression_math
{
    /*
     * ROCK suppresses its own hand bodies during physical grabs because the hand body
     * is the constraint driver, not an obstacle the held object should keep solving
     * against. HIGGS gates this with collision-filter bit 14; FO4VR Ghidra analysis
     * confirms that bit still disables hknp pair filtering. ROCK deliberately
     * avoids the native body deactivation path here because generated bodies do
     * not always have the internal table entry that function expects.
     */
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;
    inline constexpr std::uint32_t kNoCollideBit = 1u << 14;

    struct SuppressionState
    {
        bool active = false;
        bool wasNoCollideBeforeSuppression = false;
        std::uint32_t bodyId = kInvalidBodyId;
    };

    struct DelayedRestoreState
    {
        bool pending = false;
        float remainingSeconds = 0.0f;
        std::uint32_t bodyId = kInvalidBodyId;
        std::uint32_t bodyCount = 0;
    };

    struct SuppressionBeginResult
    {
        bool stored = false;
        bool firstSuppression = false;
        bool wasNoCollideBeforeSuppression = false;
        std::uint32_t disabledFilter = 0;
    };

    template <std::size_t Count>
    struct SuppressionSet
    {
        std::array<SuppressionState, Count> entries{};
    };

    inline float sanitizeDelaySeconds(float seconds)
    {
        return std::isfinite(seconds) && seconds > 0.0f ? seconds : 0.0f;
    }

    inline std::uint32_t beginSuppression(SuppressionState& state, std::uint32_t bodyId, std::uint32_t currentFilter)
    {
        if (!state.active || state.bodyId != bodyId) {
            state.active = true;
            state.wasNoCollideBeforeSuppression = (currentFilter & kNoCollideBit) != 0;
            state.bodyId = bodyId;
        }

        return currentFilter | kNoCollideBit;
    }

    inline std::uint32_t restoreFilter(const SuppressionState& state, std::uint32_t currentFilter)
    {
        return state.wasNoCollideBeforeSuppression ? (currentFilter | kNoCollideBit) : (currentFilter & ~kNoCollideBit);
    }

    inline void clear(SuppressionState& state)
    {
        state.active = false;
        state.wasNoCollideBeforeSuppression = false;
        state.bodyId = kInvalidBodyId;
    }

    inline void clear(DelayedRestoreState& state)
    {
        state.pending = false;
        state.remainingSeconds = 0.0f;
        state.bodyId = kInvalidBodyId;
        state.bodyCount = 0;
    }

    template <std::size_t Count>
    inline void clear(SuppressionSet<Count>& set)
    {
        for (auto& entry : set.entries) {
            clear(entry);
        }
    }

    template <std::size_t Count>
    inline std::size_t activeCount(const SuppressionSet<Count>& set)
    {
        std::size_t count = 0;
        for (const auto& entry : set.entries) {
            if (entry.active) {
                ++count;
            }
        }
        return count;
    }

    template <std::size_t Count>
    inline bool hasActive(const SuppressionSet<Count>& set)
    {
        return activeCount(set) > 0;
    }

    template <std::size_t Count>
    inline SuppressionState* findSuppressionState(SuppressionSet<Count>& set, std::uint32_t bodyId)
    {
        if (bodyId == kInvalidBodyId) {
            return nullptr;
        }
        for (auto& entry : set.entries) {
            if (entry.active && entry.bodyId == bodyId) {
                return &entry;
            }
        }
        return nullptr;
    }

    template <std::size_t Count>
    inline const SuppressionState* findSuppressionState(const SuppressionSet<Count>& set, std::uint32_t bodyId)
    {
        if (bodyId == kInvalidBodyId) {
            return nullptr;
        }
        for (const auto& entry : set.entries) {
            if (entry.active && entry.bodyId == bodyId) {
                return &entry;
            }
        }
        return nullptr;
    }

    template <std::size_t Count>
    inline SuppressionBeginResult beginSuppression(SuppressionSet<Count>& set, std::uint32_t bodyId, std::uint32_t currentFilter)
    {
        SuppressionBeginResult result{};
        result.disabledFilter = currentFilter | kNoCollideBit;
        if (bodyId == kInvalidBodyId) {
            return result;
        }

        if (auto* existing = findSuppressionState(set, bodyId)) {
            result.stored = true;
            result.firstSuppression = false;
            result.wasNoCollideBeforeSuppression = existing->wasNoCollideBeforeSuppression;
            return result;
        }

        for (auto& entry : set.entries) {
            if (!entry.active) {
                entry.active = true;
                entry.wasNoCollideBeforeSuppression = (currentFilter & kNoCollideBit) != 0;
                entry.bodyId = bodyId;
                result.stored = true;
                result.firstSuppression = true;
                result.wasNoCollideBeforeSuppression = entry.wasNoCollideBeforeSuppression;
                return result;
            }
        }

        return result;
    }

    template <std::size_t Count>
    inline std::uint32_t restoreFilter(const SuppressionSet<Count>& set, std::uint32_t bodyId, std::uint32_t currentFilter)
    {
        const auto* entry = findSuppressionState(set, bodyId);
        return entry ? restoreFilter(*entry, currentFilter) : currentFilter;
    }

    template <std::size_t Count>
    inline bool wasNoCollideBeforeSuppression(const SuppressionSet<Count>& set, std::uint32_t bodyId)
    {
        const auto* entry = findSuppressionState(set, bodyId);
        return entry && entry->wasNoCollideBeforeSuppression;
    }

    template <std::size_t Count>
    inline std::uint32_t firstActiveBodyId(const SuppressionSet<Count>& set)
    {
        for (const auto& entry : set.entries) {
            if (entry.active) {
                return entry.bodyId;
            }
        }
        return kInvalidBodyId;
    }

    inline bool beginDelayedRestore(DelayedRestoreState& restoreState, const SuppressionState& suppressionState, float delaySeconds)
    {
        const float sanitizedDelay = sanitizeDelaySeconds(delaySeconds);
        if (!suppressionState.active || suppressionState.bodyId == kInvalidBodyId || sanitizedDelay <= 0.0f) {
            clear(restoreState);
            return false;
        }

        restoreState.pending = true;
        restoreState.remainingSeconds = sanitizedDelay;
        restoreState.bodyId = suppressionState.bodyId;
        restoreState.bodyCount = 1;
        return true;
    }

    template <std::size_t Count>
    inline bool beginDelayedRestore(DelayedRestoreState& restoreState, const SuppressionSet<Count>& suppressionSet, float delaySeconds)
    {
        const float sanitizedDelay = sanitizeDelaySeconds(delaySeconds);
        const std::size_t count = activeCount(suppressionSet);
        if (count == 0 || sanitizedDelay <= 0.0f) {
            clear(restoreState);
            return false;
        }

        restoreState.pending = true;
        restoreState.remainingSeconds = sanitizedDelay;
        restoreState.bodyId = firstActiveBodyId(suppressionSet);
        restoreState.bodyCount = static_cast<std::uint32_t>(count);
        return true;
    }

    inline bool advanceDelayedRestore(DelayedRestoreState& restoreState, const SuppressionState& suppressionState, float deltaSeconds)
    {
        if (!restoreState.pending) {
            return false;
        }

        if (!suppressionState.active || suppressionState.bodyId != restoreState.bodyId) {
            clear(restoreState);
            return false;
        }

        const float sanitizedDelta = sanitizeDelaySeconds(deltaSeconds);
        restoreState.remainingSeconds = (std::max)(0.0f, restoreState.remainingSeconds - sanitizedDelta);
        return restoreState.remainingSeconds <= 0.0f;
    }

    template <std::size_t Count>
    inline bool advanceDelayedRestore(DelayedRestoreState& restoreState, const SuppressionSet<Count>& suppressionSet, float deltaSeconds)
    {
        if (!restoreState.pending) {
            return false;
        }

        if (!hasActive(suppressionSet)) {
            clear(restoreState);
            return false;
        }

        const float sanitizedDelta = sanitizeDelaySeconds(deltaSeconds);
        restoreState.remainingSeconds = (std::max)(0.0f, restoreState.remainingSeconds - sanitizedDelta);
        return restoreState.remainingSeconds <= 0.0f;
    }
}
