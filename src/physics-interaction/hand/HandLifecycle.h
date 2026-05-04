#pragma once

/*
 * Hand lifecycle helpers are grouped here so lifecycle state, semantic contact state, and collision suppression share one hand-state policy surface.
 */


// ---- HandLifecyclePolicy.h ----

#include <cstddef>

namespace frik::rock::hand_lifecycle_policy
{
    /*
     * Hand::reset is only safe after native Havok ownership has already been
     * released. Constraint IDs, held body IDs, saved inertia, and suppression
     * filters are not plain cache data; dropping them loses the only state ROCK
     * has for restoring the world.
     */
    inline constexpr bool requiresHavokCleanupBeforeReset(
        bool activeConstraintValid,
        bool handCollisionSuppressed,
        std::size_t heldBodyCount,
        bool savedObjectStateValid,
        bool handCollisionBodyValid)
    {
        return activeConstraintValid || handCollisionSuppressed || heldBodyCount > 0 || savedObjectStateValid || handCollisionBodyValid;
    }
}

// ---- HandState.h ----

#include <cstdint>

namespace frik::rock
{
    enum class HandState : std::uint8_t
    {
        Idle,
        SelectedClose,
        SelectedFar,
        SelectionLocked,
        HeldInit,
        HeldBody,
        Pulled,
        GrabFromOtherHand,
        SelectedTwoHand,
        HeldTwoHanded,
    };
}

// ---- HandSemanticContactState.h ----

#include "physics-interaction/hand/HandColliderTypes.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace frik::rock::hand_semantic_contact_state
{
    /*
     * Semantic hand contacts are stored separately from generic touch state so
     * ROCK can preserve HIGGS-style dynamic body ownership while still knowing
     * which generated hand part actually touched the object. Runtime code uses
     * this as evidence for source-body routing and as a safe pivot candidate
     * only when the contact is fresh and belongs to the same target body.
     */

    inline constexpr std::uint32_t kInvalidBodyId = hand_collider_semantics::kInvalidBodyId;

    struct SemanticContactRecord
    {
        bool valid = false;
        bool isLeft = false;
        hand_collider_semantics::HandColliderRole role = hand_collider_semantics::HandColliderRole::PalmAnchor;
        hand_collider_semantics::HandFinger finger = hand_collider_semantics::HandFinger::None;
        hand_collider_semantics::HandFingerSegment segment = hand_collider_semantics::HandFingerSegment::None;
        std::uint32_t handBodyId = kInvalidBodyId;
        std::uint32_t otherBodyId = kInvalidBodyId;
        std::uint32_t sequence = 0;
        std::uint32_t framesSinceContact = 0xFFFF'FFFFu;
    };

    inline constexpr std::size_t kMaxSemanticContactRecords = hand_collider_semantics::kHandColliderBodyCountPerHand;

    struct SemanticContactCollection
    {
        std::array<SemanticContactRecord, kMaxSemanticContactRecords> records{};
        std::size_t count = 0;

        bool add(const SemanticContactRecord& record)
        {
            if (count >= records.size()) {
                return false;
            }
            records[count++] = record;
            return true;
        }
    };

    struct ThumbOppositionContactPair
    {
        bool valid = false;
        SemanticContactRecord thumb{};
        SemanticContactRecord opposing{};
        const char* reason = "noOpposition";
    };

    inline constexpr std::size_t semanticContactSlotForRole(hand_collider_semantics::HandColliderRole role)
    {
        return static_cast<std::size_t>(role) < kMaxSemanticContactRecords ? static_cast<std::size_t>(role) : 0;
    }

    inline constexpr bool semanticContactSequenceSnapshotStable(std::uint32_t before, std::uint32_t after)
    {
        return before == after && (before & 1u) == 0;
    }

    inline constexpr std::uint32_t semanticFramesSinceContact(std::uint32_t currentFrame, std::uint32_t contactFrame)
    {
        if (contactFrame == 0xFFFF'FFFFu) {
            return 0xFFFF'FFFFu;
        }
        return currentFrame >= contactFrame ? currentFrame - contactFrame : 0;
    }

    inline constexpr bool isThumbContactRole(hand_collider_semantics::HandColliderRole role)
    {
        return hand_collider_semantics::fingerForRole(role) == hand_collider_semantics::HandFinger::Thumb;
    }

    inline constexpr bool isOpposingFingerContactRole(hand_collider_semantics::HandColliderRole role)
    {
        return role == hand_collider_semantics::HandColliderRole::IndexTip ||
               role == hand_collider_semantics::HandColliderRole::MiddleTip ||
               role == hand_collider_semantics::HandColliderRole::IndexMiddle ||
               role == hand_collider_semantics::HandColliderRole::MiddleMiddle;
    }

    inline constexpr int oppositionRolePriority(hand_collider_semantics::HandColliderRole role)
    {
        switch (role) {
        case hand_collider_semantics::HandColliderRole::ThumbTip:
        case hand_collider_semantics::HandColliderRole::IndexTip:
            return 0;
        case hand_collider_semantics::HandColliderRole::ThumbPad:
        case hand_collider_semantics::HandColliderRole::MiddleTip:
            return 1;
        case hand_collider_semantics::HandColliderRole::ThumbMiddle:
        case hand_collider_semantics::HandColliderRole::IndexMiddle:
            return 2;
        case hand_collider_semantics::HandColliderRole::ThumbBase:
        case hand_collider_semantics::HandColliderRole::MiddleMiddle:
            return 3;
        default:
            return 10;
        }
    }

    inline bool isBetterOppositionCandidate(const SemanticContactRecord& candidate, const SemanticContactRecord* current)
    {
        if (!candidate.valid) {
            return false;
        }
        if (!current || !current->valid) {
            return true;
        }
        const int candidatePriority = oppositionRolePriority(candidate.role);
        const int currentPriority = oppositionRolePriority(current->role);
        if (candidatePriority != currentPriority) {
            return candidatePriority < currentPriority;
        }
        return candidate.framesSinceContact < current->framesSinceContact;
    }

    inline ThumbOppositionContactPair selectThumbOppositionContacts(const SemanticContactCollection& contacts)
    {
        ThumbOppositionContactPair result{};
        const SemanticContactRecord* thumb = nullptr;
        const SemanticContactRecord* opposing = nullptr;
        for (std::size_t i = 0; i < contacts.count && i < contacts.records.size(); ++i) {
            const auto& contact = contacts.records[i];
            if (!contact.valid) {
                continue;
            }
            if (isThumbContactRole(contact.role) && isBetterOppositionCandidate(contact, thumb)) {
                thumb = &contact;
            }
            if (isOpposingFingerContactRole(contact.role) && isBetterOppositionCandidate(contact, opposing)) {
                opposing = &contact;
            }
        }

        if (!thumb) {
            result.reason = "missingThumb";
            return result;
        }
        if (!opposing) {
            result.reason = "missingOpposingFinger";
            return result;
        }

        result.valid = true;
        result.thumb = *thumb;
        result.opposing = *opposing;
        result.reason = "thumbOpposition";
        return result;
    }

    class SemanticContactSet
    {
    public:
        void record(const SemanticContactRecord& record)
        {
            if (!record.valid) {
                return;
            }
            auto stored = record;
            stored.framesSinceContact = 0;
            _records[semanticContactSlotForRole(stored.role)] = stored;
        }

        void advanceFrames()
        {
            for (auto& record : _records) {
                if (record.valid && record.framesSinceContact < 0xFFFF'FFFFu) {
                    ++record.framesSinceContact;
                }
            }
        }

        SemanticContactCollection collectFreshForBody(std::uint32_t targetBodyId, std::uint32_t maxFramesSinceContact) const
        {
            SemanticContactCollection result{};
            for (const auto& record : _records) {
                if (!record.valid || record.handBodyId == kInvalidBodyId || record.otherBodyId != targetBodyId) {
                    continue;
                }
                if (record.framesSinceContact > maxFramesSinceContact) {
                    continue;
                }
                result.add(record);
            }
            return result;
        }

        SemanticContactRecord getFreshForRole(hand_collider_semantics::HandColliderRole role, std::uint32_t maxFramesSinceContact) const
        {
            const auto& record = _records[semanticContactSlotForRole(role)];
            if (!record.valid || record.handBodyId == kInvalidBodyId || record.otherBodyId == kInvalidBodyId) {
                return {};
            }
            if (record.framesSinceContact > maxFramesSinceContact) {
                return {};
            }
            return record;
        }

    private:
        std::array<SemanticContactRecord, kMaxSemanticContactRecords> _records{};
    };

    struct SemanticPivotCandidateDecision
    {
        bool accept = false;
        const char* reason = "disabled";
    };

    inline SemanticPivotCandidateDecision evaluateSemanticPivotCandidate(
        bool enabled,
        const SemanticContactRecord& record,
        std::uint32_t targetBodyId,
        std::uint32_t maxFramesSinceContact)
    {
        if (!enabled) {
            return { false, "disabled" };
        }
        if (!record.valid || record.handBodyId == kInvalidBodyId || record.otherBodyId == kInvalidBodyId) {
            return { false, "noSemanticContact" };
        }
        if (targetBodyId == kInvalidBodyId || record.otherBodyId != targetBodyId) {
            return { false, "targetMismatch" };
        }
        if (record.framesSinceContact > maxFramesSinceContact) {
            return { false, "staleContact" };
        }
        if (record.role == hand_collider_semantics::HandColliderRole::PalmAnchor) {
            return { false, "anchorOnly" };
        }
        return { true, "semanticContact" };
    }
}

// ---- HandCollisionSuppressionMath.h ----

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
