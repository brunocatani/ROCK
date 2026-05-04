#pragma once

#include "HandColliderSemanticTypes.h"

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
