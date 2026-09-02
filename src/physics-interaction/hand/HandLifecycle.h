#pragma once

#include "physics-interaction/VectorMath.h"

/*
 * Hand lifecycle helpers are grouped here so lifecycle state, semantic contact state, and collision suppression share one hand-state policy surface.
 */


// ---- HandLifecyclePolicy.h ----

#include <cstddef>

namespace rock::hand_lifecycle_policy
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

namespace rock
{
    enum class HandState : std::uint8_t
    {
        Idle,
        SelectedClose,
        SelectedFar,
        SelectionLocked,
        PreGrabItem,
        PrePullItem,
        HeldInit,
        HeldBody,
        Pulled,
        GrabFromOtherHand,
        GrabExternal,
        LootOtherHand,
        SelectedTwoHand,
        HeldTwoHanded,
        StashCandidate,
        ConsumeCandidate,
    };
}

// ---- HandSemanticContactState.h ----

#include "physics-interaction/hand/HandColliderTypes.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>

namespace rock::hand_semantic_contact_state
{
    /*
     * Semantic hand contacts are stored separately from generic touch state so
     * ROCK can preserve dynamic body ownership while still knowing which
     * generated hand part actually touched the object. Runtime code uses this as
     * evidence for source-body routing and as a safe pivot candidate only when
     * the contact is fresh and belongs to the same target body.
     */

    inline constexpr std::uint32_t kInvalidBodyId = hand_collider_semantics::kInvalidBodyId;

    struct SemanticContactVector
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    // Sentinel age for a record that has never seen a contact on the seconds
    // clock; any finite freshness window classifies it as stale.
    inline constexpr float kSemanticContactNeverSeconds = 1.0e9f;

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
        /*
         * Frame fields are publication/sequence identity and stay counters:
         * the provider V1 contract and frame-count configuration gates consume
         * them. secondsSinceContact is the rate-independent elapsed-freshness
         * value internal gameplay windows consume.
         */
        std::uint32_t contactFrame = 0xFFFF'FFFFu;
        std::uint32_t contactRunStartFrame = 0xFFFF'FFFFu;
        std::uint32_t framesSinceContact = 0xFFFF'FFFFu;
        float secondsSinceContact = kSemanticContactNeverSeconds;
        bool hasContactPointGame = false;
        bool hasContactNormalGame = false;
        SemanticContactVector contactPointGame{};
        SemanticContactVector contactNormalGame{};
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
        return currentFrame - contactFrame;
    }

    inline constexpr bool semanticContactContinuesRun(
        const bool previousValid,
        const std::uint32_t previousOtherBodyId,
        const std::uint32_t currentOtherBodyId,
        const std::uint32_t currentFrame,
        const std::uint32_t previousContactFrame)
    {
        return previousValid &&
               previousOtherBodyId == currentOtherBodyId &&
               currentFrame - previousContactFrame <= 1;
    }

    inline bool isFiniteVector(const SemanticContactVector& value)
    {
        return vector_math::hasFiniteComponents(value);
    }

    inline bool hasUsableContactPoint(const SemanticContactRecord& record)
    {
        return record.valid && record.hasContactPointGame && isFiniteVector(record.contactPointGame);
    }

    inline bool hasUsableContactNormal(const SemanticContactRecord& record)
    {
        if (!record.valid || !record.hasContactNormalGame || !isFiniteVector(record.contactNormalGame)) {
            return false;
        }
        const float lengthSquared = vector_math::lengthSquared(record.contactNormalGame);
        return std::isfinite(lengthSquared) && lengthSquared > 1.0e-6f;
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
            stored.secondsSinceContact = 0.0f;
            _records[semanticContactSlotForRole(stored.role)] = stored;
        }

        /*
         * Advance both clocks once per game frame: the frame counter always
         * counts publications; the seconds clock accumulates only measured
         * game time (pass zero for an invalid or unmeasurable frame so
         * freshness never advances by fabricated time).
         */
        void advance(float validDeltaSeconds)
        {
            const float delta =
                std::isfinite(validDeltaSeconds) && validDeltaSeconds > 0.0f ? validDeltaSeconds : 0.0f;
            for (auto& record : _records) {
                if (!record.valid) {
                    continue;
                }
                if (record.framesSinceContact < 0xFFFF'FFFFu) {
                    ++record.framesSinceContact;
                }
                if (record.secondsSinceContact < kSemanticContactNeverSeconds) {
                    record.secondsSinceContact += delta;
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

        SemanticContactCollection collectFreshForBodyWithinSeconds(std::uint32_t targetBodyId, float maxAgeSeconds) const
        {
            SemanticContactCollection result{};
            for (const auto& record : _records) {
                if (!record.valid || record.handBodyId == kInvalidBodyId || record.otherBodyId != targetBodyId) {
                    continue;
                }
                if (!(record.secondsSinceContact <= maxAgeSeconds)) {
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
        float maxAgeSeconds)
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
        if (!(record.secondsSinceContact <= maxAgeSeconds)) {
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

namespace rock::hand_collision_suppression_math
{
    // Configuration parsing still uses this pure sanitizer. Runtime lease and
    // delayed-restore ownership now lives in CollisionSuppressionRegistry.h.
    inline float sanitizeDelaySeconds(float seconds)
    {
        return std::isfinite(seconds) && seconds > 0.0f ? seconds : 0.0f;
    }
}
