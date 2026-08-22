#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "physics-interaction/TransformMath.h"

#include "RE/NetImmerse/NiTransform.h"

namespace rock::test_support
{
    /*
     * A pure model of the deferred hFRIK external-hand contract.
     *
     * The real contract has four properties that make the presentation group
     * hard to reason about, and this model reproduces exactly those:
     *
     *  1. Publication is data-only. A true return means "the target was
     *     accepted into the registry", never "the hand reached it".
     *  2. The target is consumed by the NEXT skeleton pass, not this one.
     *  3. A claim persists until its tag is cleared. It is not per-frame.
     *  4. An unreachable target silently falls back to the tracked hand, and
     *     nothing in the publish API reports that it happened.
     *
     * Property 1 combined with property 4 is why a successful publish cannot
     * be treated as a committed hand pose, and is why the presentation
     * coordinator needs a post-solve readback.
     *
     * Arbitration is priority first, then publication sequence: at equal
     * priority the later publication wins.
     */
    class FakeFrikExternalAuthority
    {
    public:
        enum class Hand : std::size_t
        {
            Left = 0,
            Right = 1,
        };

        struct Claim
        {
            std::string tag;
            RE::NiTransform target{};
            Hand hand = Hand::Left;
            int priority = 0;
            std::uint64_t sequence = 0;
            bool valid = false;
        };

        void setAvailable(const bool available) { _available = available; }

        void setPublicationReady(const Hand hand, const bool ready)
        {
            _publicationReady[index(hand)] = ready;
        }

        // Simulates hFRIK refusing one specific tag, which is how a partial
        // group failure appears to the publisher.
        void setRejectedTag(std::string tag) { _rejectedTag = std::move(tag); }

        void setTrackedHandWorld(const Hand hand, const RE::NiTransform& world)
        {
            _trackedHandWorld[index(hand)] = world;
        }

        // A target farther than this from the tracked hand is unreachable and
        // silently falls back. A non-positive value disables the limit.
        void setReachLimitGameUnits(const float limit)
        {
            _reachLimitGameUnits = limit;
        }

        // Composed into the winning target before the next solve, once.
        void setPendingRecoilDelta(
            const Hand hand,
            const RE::NiTransform& worldDelta)
        {
            _pendingRecoilDelta[index(hand)] = worldDelta;
        }

        [[nodiscard]] bool publish(
            const std::string& tag,
            const Hand hand,
            const RE::NiTransform& target,
            const int priority)
        {
            if (!_available || !_publicationReady[index(hand)] ||
                tag.empty() || tag == _rejectedTag) {
                return false;
            }
            ++_publicationSequence;
            if (auto* existing = find(tag, hand)) {
                existing->target = target;
                existing->priority = priority;
                existing->sequence = _publicationSequence;
                existing->valid = true;
                return true;
            }
            _claims.push_back(Claim{
                .tag = tag,
                .target = target,
                .hand = hand,
                .priority = priority,
                .sequence = _publicationSequence,
                .valid = true,
            });
            return true;
        }

        [[nodiscard]] bool clear(const std::string& tag, const Hand hand)
        {
            auto* existing = find(tag, hand);
            if (!existing) {
                // Clearing an absent tag is not an error in the real API.
                return true;
            }
            existing->valid = false;
            return true;
        }

        [[nodiscard]] bool tryGetWinner(const Hand hand, Claim& outClaim) const
        {
            const Claim* winner = nullptr;
            for (const auto& claim : _claims) {
                if (!claim.valid || claim.hand != hand) {
                    continue;
                }
                if (!winner ||
                    claim.priority > winner->priority ||
                    (claim.priority == winner->priority &&
                        claim.sequence > winner->sequence)) {
                    winner = &claim;
                }
            }
            if (!winner) {
                return false;
            }
            outClaim = *winner;
            return true;
        }

        // One hFRIK skeleton pass. Claims survive it.
        void solveSkeletonFrame()
        {
            for (std::size_t handIndex = 0;
                 handIndex < _presentedWrist.size();
                 ++handIndex) {
                const auto hand = static_cast<Hand>(handIndex);
                _usedFallback[handIndex] = false;
                Claim winner{};
                if (!_available || !tryGetWinner(hand, winner)) {
                    _presentedWrist[handIndex] = _trackedHandWorld[handIndex];
                    continue;
                }

                RE::NiTransform target = winner.target;
                if (_pendingRecoilDelta[handIndex]) {
                    // Recoil is composed INTO the target before the solve, so
                    // the solved wrist already carries the kick.
                    target = transform_math::composeTransforms(
                        *_pendingRecoilDelta[handIndex],
                        target);
                    _pendingRecoilDelta[handIndex].reset();
                }

                if (isReachable(hand, target)) {
                    _presentedWrist[handIndex] = target;
                    continue;
                }
                _usedFallback[handIndex] = true;
                _presentedWrist[handIndex] = _trackedHandWorld[handIndex];
            }
            ++_solveSequence;
        }

        [[nodiscard]] const RE::NiTransform& presentedWrist(
            const Hand hand) const
        {
            return _presentedWrist[index(hand)];
        }

        [[nodiscard]] bool lastSolveUsedFallback(const Hand hand) const
        {
            return _usedFallback[index(hand)];
        }

        [[nodiscard]] std::uint64_t solveSequence() const
        {
            return _solveSequence;
        }

        [[nodiscard]] std::size_t liveClaimCount() const
        {
            std::size_t count = 0;
            for (const auto& claim : _claims) {
                count += claim.valid ? 1u : 0u;
            }
            return count;
        }

        // The skeleton went away. Every retained claim goes with it.
        void dropSkeleton()
        {
            _claims.clear();
            _presentedWrist = {};
            _usedFallback = {};
            _pendingRecoilDelta = {};
        }

    private:
        [[nodiscard]] static constexpr std::size_t index(
            const Hand hand) noexcept
        {
            return static_cast<std::size_t>(hand);
        }

        [[nodiscard]] bool isReachable(
            const Hand hand,
            const RE::NiTransform& target) const
        {
            if (!(_reachLimitGameUnits > 0.0f)) {
                return true;
            }
            const auto& tracked = _trackedHandWorld[index(hand)];
            const float dx = target.translate.x - tracked.translate.x;
            const float dy = target.translate.y - tracked.translate.y;
            const float dz = target.translate.z - tracked.translate.z;
            return (dx * dx + dy * dy + dz * dz) <=
                (_reachLimitGameUnits * _reachLimitGameUnits);
        }

        [[nodiscard]] Claim* find(const std::string& tag, const Hand hand)
        {
            for (auto& claim : _claims) {
                if (claim.tag == tag && claim.hand == hand) {
                    return &claim;
                }
            }
            return nullptr;
        }

        std::vector<Claim> _claims;
        std::array<RE::NiTransform, 2> _trackedHandWorld{};
        std::array<RE::NiTransform, 2> _presentedWrist{};
        std::array<std::optional<RE::NiTransform>, 2> _pendingRecoilDelta{};
        std::array<bool, 2> _publicationReady{ true, true };
        std::array<bool, 2> _usedFallback{};
        std::string _rejectedTag;
        std::uint64_t _publicationSequence = 0;
        std::uint64_t _solveSequence = 0;
        float _reachLimitGameUnits = 0.0f;
        bool _available = true;
    };
}
