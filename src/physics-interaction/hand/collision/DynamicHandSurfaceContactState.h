#pragma once

#include "physics-interaction/hand/HandLifecycle.h"

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rock::dynamic_hand_surface_contact_state
{
    inline constexpr std::size_t kContactRolesPerHand = 6;
    inline constexpr std::size_t kHandCount = 2;

    struct ContactSource
    {
        bool valid = false;
        bool isLeft = false;
        std::size_t slot = 0;
        hand_collider_semantics::HandColliderRole role =
            hand_collider_semantics::HandColliderRole::PalmAnchor;
        hand_collider_semantics::HandFinger finger =
            hand_collider_semantics::HandFinger::None;
        hand_collider_semantics::HandFingerSegment segment =
            hand_collider_semantics::HandFingerSegment::None;
        std::uint32_t bodyId = hand_semantic_contact_state::kInvalidBodyId;
    };

    /*
     * Dynamic hand compound children intentionally remain outside Hand's
     * ordinary semantic contact set. This bounded publication is the only bridge from their
     * physics callbacks to provider-scoped fixed-surface grabbing.
     *
     * Physics callbacks are the producers and the main game frame is the
     * consumer. Each role owns one seqlock slot; an atomic writer claim drops a
     * duplicate concurrent callback instead of blocking a physics thread.
     * Contacts repeat every solve, so dropping one duplicate is harmless.
     */
    class State
    {
    public:
        void advanceFrame() noexcept
        {
            _frame.fetch_add(1, std::memory_order_acq_rel);
        }

        void clear() noexcept
        {
            _epoch.fetch_add(1, std::memory_order_acq_rel);
        }

        [[nodiscard]] bool record(
            const ContactSource& source,
            const std::uint32_t otherBodyId,
            const hand_semantic_contact_state::SemanticContactVector* contactPointGame,
            const hand_semantic_contact_state::SemanticContactVector* contactNormalGame) noexcept
        {
            if (!source.valid || source.slot >= kContactRolesPerHand ||
                source.bodyId == hand_semantic_contact_state::kInvalidBodyId ||
                otherBodyId == hand_semantic_contact_state::kInvalidBodyId ||
                source.bodyId == otherBodyId) {
                return false;
            }

            auto& slot = _slots[source.isLeft ? 1u : 0u][source.slot];
            if (slot.writer.test_and_set(std::memory_order_acquire)) {
                return false;
            }

            const bool hasPoint =
                contactPointGame && finite(*contactPointGame);
            const bool hasNormal =
                contactNormalGame && finite(*contactNormalGame) &&
                lengthSquared(*contactNormalGame) > 1.0e-6f;
            const hand_semantic_contact_state::SemanticContactVector empty{};
            const auto& point = hasPoint ? *contactPointGame : empty;
            const auto& normal = hasNormal ? *contactNormalGame : empty;

            std::uint32_t sequence =
                slot.sequence.load(std::memory_order_relaxed);
            if ((sequence & 1u) != 0) {
                ++sequence;
            }
            slot.sequence.store(sequence + 1, std::memory_order_release);
            slot.valid.store(0, std::memory_order_release);
            slot.epoch.store(_epoch.load(std::memory_order_acquire), std::memory_order_relaxed);
            slot.frame.store(_frame.load(std::memory_order_acquire), std::memory_order_relaxed);
            slot.role.store(static_cast<std::uint32_t>(source.role), std::memory_order_relaxed);
            slot.finger.store(static_cast<std::uint32_t>(source.finger), std::memory_order_relaxed);
            slot.segment.store(static_cast<std::uint32_t>(source.segment), std::memory_order_relaxed);
            slot.handBodyId.store(source.bodyId, std::memory_order_relaxed);
            slot.otherBodyId.store(otherBodyId, std::memory_order_relaxed);
            slot.pointX.store(point.x, std::memory_order_relaxed);
            slot.pointY.store(point.y, std::memory_order_relaxed);
            slot.pointZ.store(point.z, std::memory_order_relaxed);
            slot.normalX.store(normal.x, std::memory_order_relaxed);
            slot.normalY.store(normal.y, std::memory_order_relaxed);
            slot.normalZ.store(normal.z, std::memory_order_relaxed);
            slot.hasPoint.store(hasPoint ? 1u : 0u, std::memory_order_relaxed);
            slot.hasNormal.store(hasNormal ? 1u : 0u, std::memory_order_relaxed);
            slot.valid.store(1, std::memory_order_release);
            slot.sequence.store(sequence + 2, std::memory_order_release);
            slot.writer.clear(std::memory_order_release);
            return true;
        }

        [[nodiscard]] hand_semantic_contact_state::SemanticContactCollection collectFresh(
            const bool isLeft,
            const std::uint32_t maximumAgeFrames) const noexcept
        {
            hand_semantic_contact_state::SemanticContactCollection result{};
            const std::uint64_t currentEpoch =
                _epoch.load(std::memory_order_acquire);
            const std::uint32_t currentFrame =
                _frame.load(std::memory_order_acquire);

            for (const auto& slot : _slots[isLeft ? 1u : 0u]) {
                for (int attempt = 0; attempt < 3; ++attempt) {
                    const std::uint32_t sequenceBefore =
                        slot.sequence.load(std::memory_order_acquire);
                    if ((sequenceBefore & 1u) != 0) {
                        continue;
                    }
                    if (slot.valid.load(std::memory_order_acquire) == 0) {
                        break;
                    }

                    const std::uint64_t contactEpoch =
                        slot.epoch.load(std::memory_order_relaxed);
                    const std::uint32_t contactFrame =
                        slot.frame.load(std::memory_order_relaxed);
                    hand_semantic_contact_state::SemanticContactRecord contact{};
                    contact.valid = true;
                    contact.isLeft = isLeft;
                    contact.role = static_cast<hand_collider_semantics::HandColliderRole>(
                        slot.role.load(std::memory_order_relaxed));
                    contact.finger = static_cast<hand_collider_semantics::HandFinger>(
                        slot.finger.load(std::memory_order_relaxed));
                    contact.segment = static_cast<hand_collider_semantics::HandFingerSegment>(
                        slot.segment.load(std::memory_order_relaxed));
                    contact.handBodyId =
                        slot.handBodyId.load(std::memory_order_relaxed);
                    contact.otherBodyId =
                        slot.otherBodyId.load(std::memory_order_relaxed);
                    contact.sequence = sequenceBefore;
                    contact.contactFrame = contactFrame;
                    contact.contactRunStartFrame = contactFrame;
                    contact.framesSinceContact = currentFrame - contactFrame;
                    contact.hasContactPointGame =
                        slot.hasPoint.load(std::memory_order_relaxed) != 0;
                    contact.hasContactNormalGame =
                        slot.hasNormal.load(std::memory_order_relaxed) != 0;
                    contact.contactPointGame = {
                        slot.pointX.load(std::memory_order_relaxed),
                        slot.pointY.load(std::memory_order_relaxed),
                        slot.pointZ.load(std::memory_order_relaxed)
                    };
                    contact.contactNormalGame = {
                        slot.normalX.load(std::memory_order_relaxed),
                        slot.normalY.load(std::memory_order_relaxed),
                        slot.normalZ.load(std::memory_order_relaxed)
                    };

                    const std::uint32_t sequenceAfter =
                        slot.sequence.load(std::memory_order_acquire);
                    if (!hand_semantic_contact_state::semanticContactSequenceSnapshotStable(
                            sequenceBefore,
                            sequenceAfter)) {
                        continue;
                    }
                    if (contactEpoch != currentEpoch ||
                        contact.handBodyId == hand_semantic_contact_state::kInvalidBodyId ||
                        contact.otherBodyId == hand_semantic_contact_state::kInvalidBodyId ||
                        contact.framesSinceContact > maximumAgeFrames) {
                        break;
                    }
                    if (contact.hasContactPointGame &&
                        !hand_semantic_contact_state::isFiniteVector(contact.contactPointGame)) {
                        contact.hasContactPointGame = false;
                    }
                    if (contact.hasContactNormalGame &&
                        !hand_semantic_contact_state::hasUsableContactNormal(contact)) {
                        contact.hasContactNormalGame = false;
                    }
                    result.add(contact);
                    break;
                }
            }
            return result;
        }

    private:
        struct AtomicSlot
        {
            std::atomic_flag writer = ATOMIC_FLAG_INIT;
            std::atomic<std::uint32_t> sequence{ 0 };
            std::atomic<std::uint32_t> valid{ 0 };
            std::atomic<std::uint64_t> epoch{ 0 };
            std::atomic<std::uint32_t> frame{ 0 };
            std::atomic<std::uint32_t> role{ 0 };
            std::atomic<std::uint32_t> finger{ 0 };
            std::atomic<std::uint32_t> segment{ 0 };
            std::atomic<std::uint32_t> handBodyId{
                hand_semantic_contact_state::kInvalidBodyId
            };
            std::atomic<std::uint32_t> otherBodyId{
                hand_semantic_contact_state::kInvalidBodyId
            };
            std::atomic<std::uint32_t> hasPoint{ 0 };
            std::atomic<std::uint32_t> hasNormal{ 0 };
            std::atomic<float> pointX{ 0.0f };
            std::atomic<float> pointY{ 0.0f };
            std::atomic<float> pointZ{ 0.0f };
            std::atomic<float> normalX{ 0.0f };
            std::atomic<float> normalY{ 0.0f };
            std::atomic<float> normalZ{ 0.0f };
        };

        static bool finite(
            const hand_semantic_contact_state::SemanticContactVector& value) noexcept
        {
            return std::isfinite(value.x) &&
                   std::isfinite(value.y) &&
                   std::isfinite(value.z);
        }

        static float lengthSquared(
            const hand_semantic_contact_state::SemanticContactVector& value) noexcept
        {
            return value.x * value.x +
                   value.y * value.y +
                   value.z * value.z;
        }

        std::array<std::array<AtomicSlot, kContactRolesPerHand>, kHandCount> _slots{};
        std::atomic<std::uint64_t> _epoch{ 1 };
        std::atomic<std::uint32_t> _frame{ 0 };
    };
}
