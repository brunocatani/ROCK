#pragma once

#include "api/ProviderLeasePolicy.h"
#include "api/ROCKProviderApi.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace rock::provider
{
    enum class TouchGrabMotionClassV1 : std::uint32_t
    {
        Static = 0,
        Keyframed = 1,
        Dynamic = 2,
        Other = 3,
    };

    struct TouchGrabTargetMatchV1
    {
        bool matched{ false };
        bool wildcard{ false };
        bool yieldRequested{ false };
        std::uint64_t ownerToken{ 0 };
        std::uint64_t scopeToken{ 0 };
        RockProviderTouchGrabTargetV1 target{};
    };

    class TouchGrabRegistry
    {
    public:
        enum class RegistrationResult
        {
            Ok,
            InvalidArgument,
            CapacityFull,
            OwnerConflict,
        };

        static constexpr std::size_t kMaximumTargets =
            ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGETS_V1;
        static constexpr std::size_t kMaximumScopes =
            ROCK_PROVIDER_MAX_TOUCH_GRAB_SCOPES_V1;
        static constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;

        RegistrationResult setScope(
            const std::uint64_t ownerToken,
            const std::uint64_t scopeToken,
            const RockProviderTouchGrabTargetV1* targets,
            const std::uint32_t targetCount,
            const std::uint64_t frameIndex)
        {
            if (ownerToken == 0 || scopeToken == 0 ||
                targetCount > kMaximumTargets ||
                (targetCount != 0 && !targets)) {
                return RegistrationResult::InvalidArgument;
            }

            pruneExpired(frameIndex);
            if (!validateInput(targets, targetCount)) {
                return RegistrationResult::InvalidArgument;
            }

            const bool replacingExistingScope =
                hasScope(ownerToken, scopeToken);
            if (!replacingExistingScope &&
                targetCount != 0 &&
                scopeCount() >= kMaximumScopes) {
                return RegistrationResult::CapacityFull;
            }

            const std::size_t retainedCount =
                activeCountOutsideScope(ownerToken, scopeToken);
            if (retainedCount + targetCount > kMaximumTargets) {
                return RegistrationResult::CapacityFull;
            }

            for (std::uint32_t index = 0; index < targetCount; ++index) {
                for (const auto& slot : _slots) {
                    if (!slot.active ||
                        (slot.ownerToken == ownerToken &&
                            slot.scopeToken == scopeToken)) {
                        continue;
                    }
                    if (slot.ownerToken == ownerToken &&
                        slot.target.targetId == targets[index].targetId) {
                        return RegistrationResult::OwnerConflict;
                    }
                    if (targetsConflict(slot.target, targets[index])) {
                        return RegistrationResult::OwnerConflict;
                    }
                }
            }

            std::array<PreservedState, kMaximumTargets> preserved{};
            for (std::uint32_t index = 0; index < targetCount; ++index) {
                for (const auto& slot : _slots) {
                    if (!slot.active ||
                        slot.ownerToken != ownerToken ||
                        slot.scopeToken != scopeToken ||
                        !sameTargetIdentity(slot.target, targets[index])) {
                        continue;
                    }
                    preserved[index].valid = true;
                    preserved[index].state = slot.state;
                    preserved[index].yieldRequested =
                        slot.yieldRequested;
                    break;
                }
            }

            clearScopeUnchecked(ownerToken, scopeToken);
            for (std::uint32_t index = 0; index < targetCount; ++index) {
                auto* slot = firstFreeSlot();
                if (!slot) {
                    return RegistrationResult::CapacityFull;
                }

                slot->active = true;
                slot->ownerToken = ownerToken;
                slot->scopeToken = scopeToken;
                slot->target = targets[index];
                slot->target.leaseFrames =
                    provider_lease_policy::clampLeaseFrames(
                        targets[index].leaseFrames,
                        ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGET_LEASE_FRAMES_V1);
                slot->expiresAfterFrame =
                    provider_lease_policy::exclusiveExpiryFrame(
                        frameIndex,
                        slot->target.leaseFrames);
                slot->yieldRequested =
                    preserved[index].valid &&
                    preserved[index].yieldRequested;
                if (preserved[index].valid) {
                    slot->state = preserved[index].state;
                } else {
                    slot->state = makeArmedState(slot->target, frameIndex);
                    slot->state.sequence = nextSequence();
                }
            }
            return RegistrationResult::Ok;
        }

        bool clearScope(
            const std::uint64_t ownerToken,
            const std::uint64_t scopeToken)
        {
            if (ownerToken == 0 || scopeToken == 0) {
                return false;
            }
            return clearScopeUnchecked(ownerToken, scopeToken);
        }

        void clearOwner(const std::uint64_t ownerToken)
        {
            if (ownerToken == 0) {
                return;
            }
            for (auto& slot : _slots) {
                if (slot.active && slot.ownerToken == ownerToken) {
                    slot = {};
                }
            }
        }

        void clearAll()
        {
            _slots = {};
        }

        [[nodiscard]] TouchGrabTargetMatchV1 resolve(
            const std::uint32_t bodyId,
            const std::uint32_t collisionLayer,
            const TouchGrabMotionClassV1 motionClass,
            const RockProviderHand hand,
            const std::uint32_t worldGeneration,
            const std::uint32_t skeletonGeneration,
            const std::uint32_t providerGeneration,
            const std::uint64_t frameIndex)
        {
            pruneExpired(frameIndex);
            TouchGrabTargetMatchV1 wildcardMatch{};
            for (const auto& slot : _slots) {
                if (!slot.active ||
                    slot.yieldRequested ||
                    slot.state.phase ==
                        RockProviderTouchGrabPhaseV1::Yielded ||
                    slot.state.phase ==
                        RockProviderTouchGrabPhaseV1::Invalidated ||
                    !generationsMatch(
                        slot.target,
                        worldGeneration,
                        skeletonGeneration,
                        providerGeneration) ||
                    !handMatches(slot.target, hand) ||
                    !motionMatches(slot.target, motionClass)) {
                    continue;
                }

                const bool wildcard = hasTouchGrabTargetFlagV1(
                    slot.target.flags,
                    RockProviderTouchGrabTargetFlagV1::MatchAnyBody);
                if (!wildcard && slot.target.bodyId == bodyId) {
                    return makeMatch(slot, false);
                }
                if (wildcard &&
                    collisionLayer < 64 &&
                    (slot.target.allowedLayerMask &
                        (std::uint64_t{ 1 } << collisionLayer)) != 0 &&
                    !wildcardMatch.matched) {
                    wildcardMatch = makeMatch(slot, true);
                }
            }
            return wildcardMatch;
        }

        [[nodiscard]] bool currentTarget(
            const std::uint64_t ownerToken,
            const std::uint64_t scopeToken,
            const std::uint64_t targetId,
            const std::uint32_t targetGeneration,
            const std::uint32_t worldGeneration,
            const std::uint32_t skeletonGeneration,
            const std::uint32_t providerGeneration,
            const std::uint64_t frameIndex,
            TouchGrabTargetMatchV1& outMatch)
        {
            outMatch = {};
            pruneExpired(frameIndex);
            for (const auto& slot : _slots) {
                if (!slot.active ||
                    slot.ownerToken != ownerToken ||
                    slot.scopeToken != scopeToken ||
                    slot.target.targetId != targetId ||
                    slot.target.targetGeneration != targetGeneration ||
                    !generationsMatch(
                        slot.target,
                        worldGeneration,
                        skeletonGeneration,
                        providerGeneration)) {
                    continue;
                }
                outMatch = makeMatch(
                    slot,
                    hasTouchGrabTargetFlagV1(
                        slot.target.flags,
                        RockProviderTouchGrabTargetFlagV1::MatchAnyBody));
                return true;
            }
            return false;
        }

        bool publishState(
            const std::uint64_t ownerToken,
            const std::uint64_t scopeToken,
            const RockProviderTouchGrabStateV1& state,
            const std::uint64_t frameIndex)
        {
            for (auto& slot : _slots) {
                if (!slot.active ||
                    slot.ownerToken != ownerToken ||
                    slot.scopeToken != scopeToken ||
                    slot.target.targetId != state.targetId ||
                    slot.target.targetGeneration != state.targetGeneration) {
                    continue;
                }

                slot.state = state;
                slot.state.size = sizeof(RockProviderTouchGrabStateV1);
                slot.state.version = ROCK_PROVIDER_API_VERSION;
                slot.state.frameIndex = frameIndex;
                slot.state.sequence = nextSequence();
                return true;
            }
            return false;
        }

        bool requestYield(
            const std::uint64_t ownerToken,
            const std::uint64_t scopeToken,
            const std::uint64_t targetId,
            const std::uint32_t targetGeneration,
            const std::uint64_t frameIndex)
        {
            pruneExpired(frameIndex);
            for (auto& slot : _slots) {
                if (slot.active &&
                    slot.ownerToken == ownerToken &&
                    slot.scopeToken == scopeToken &&
                    slot.target.targetId == targetId &&
                    slot.target.targetGeneration == targetGeneration) {
                    slot.yieldRequested = true;
                    return true;
                }
            }
            return false;
        }

        bool acknowledgeYield(
            const std::uint64_t ownerToken,
            const std::uint64_t scopeToken,
            const std::uint64_t targetId,
            const std::uint32_t targetGeneration)
        {
            for (auto& slot : _slots) {
                if (slot.active &&
                    slot.ownerToken == ownerToken &&
                    slot.scopeToken == scopeToken &&
                    slot.target.targetId == targetId &&
                    slot.target.targetGeneration == targetGeneration) {
                    slot.yieldRequested = false;
                    return true;
                }
            }
            return false;
        }

        std::uint32_t copyStates(
            const std::uint64_t ownerToken,
            const std::uint64_t scopeToken,
            RockProviderTouchGrabStateV1* outStates,
            const std::uint32_t maxStates,
            const std::uint64_t frameIndex)
        {
            pruneExpired(frameIndex);
            std::uint32_t copied = 0;
            for (const auto& slot : _slots) {
                if (!slot.active ||
                    slot.ownerToken != ownerToken ||
                    slot.scopeToken != scopeToken) {
                    continue;
                }
                if (outStates && copied < maxStates) {
                    outStates[copied] = slot.state;
                    ++copied;
                } else if (outStates) {
                    break;
                } else {
                    ++copied;
                }
            }
            return copied;
        }

        [[nodiscard]] std::size_t targetCount() const
        {
            return static_cast<std::size_t>(std::count_if(
                _slots.begin(),
                _slots.end(),
                [](const Slot& slot) { return slot.active; }));
        }

    private:
        struct Slot
        {
            bool active{ false };
            bool yieldRequested{ false };
            std::uint64_t ownerToken{ 0 };
            std::uint64_t scopeToken{ 0 };
            std::uint64_t expiresAfterFrame{ 0 };
            RockProviderTouchGrabTargetV1 target{};
            RockProviderTouchGrabStateV1 state{};
        };

        struct PreservedState
        {
            bool valid{ false };
            bool yieldRequested{ false };
            RockProviderTouchGrabStateV1 state{};
        };

        static constexpr std::uint32_t kHandFlags =
            static_cast<std::uint32_t>(
                RockProviderTouchGrabTargetFlagV1::AllowRightHand) |
            static_cast<std::uint32_t>(
                RockProviderTouchGrabTargetFlagV1::AllowLeftHand);
        static constexpr std::uint32_t kMotionFlags =
            static_cast<std::uint32_t>(
                RockProviderTouchGrabTargetFlagV1::MatchStaticMotion) |
            static_cast<std::uint32_t>(
                RockProviderTouchGrabTargetFlagV1::MatchKeyframedMotion) |
            static_cast<std::uint32_t>(
                RockProviderTouchGrabTargetFlagV1::MatchDynamicMotion);
        static constexpr std::uint32_t kKnownFlags =
            kHandFlags |
            kMotionFlags |
            static_cast<std::uint32_t>(
                RockProviderTouchGrabTargetFlagV1::AllowTwoHands) |
            static_cast<std::uint32_t>(
                RockProviderTouchGrabTargetFlagV1::LatchOnRelease) |
            static_cast<std::uint32_t>(
                RockProviderTouchGrabTargetFlagV1::MatchAnyBody);

        static bool finitePoint(const RockProviderPoint3& point)
        {
            return std::isfinite(point.x) &&
                   std::isfinite(point.y) &&
                   std::isfinite(point.z);
        }

        static bool validateTarget(
            const RockProviderTouchGrabTargetV1& target)
        {
            if (target.size != sizeof(RockProviderTouchGrabTargetV1) ||
                target.version != ROCK_PROVIDER_API_VERSION ||
                target.targetId == 0 ||
                target.targetGeneration == 0 ||
                target.leaseFrames == 0 ||
                target.worldGeneration == 0 ||
                target.skeletonGeneration == 0 ||
                target.providerGeneration == 0 ||
                (target.flags & ~kKnownFlags) != 0 ||
                (target.flags & kHandFlags) == 0 ||
                (target.flags & kMotionFlags) == 0) {
                return false;
            }

            const bool wildcard = hasTouchGrabTargetFlagV1(
                target.flags,
                RockProviderTouchGrabTargetFlagV1::MatchAnyBody);
            if (target.kind == RockProviderTouchGrabKindV1::FixedAnchor) {
                if (wildcard) {
                    return target.bodyId == kInvalidBodyId &&
                           target.allowedLayerMask != 0;
                }
                return target.bodyId != kInvalidBodyId;
            }
            if (target.kind != RockProviderTouchGrabKindV1::LimitedHinge &&
                target.kind !=
                    RockProviderTouchGrabKindV1::LimitedPrismatic) {
                return false;
            }
            if (wildcard || target.bodyId == kInvalidBodyId ||
                (target.flags &
                    static_cast<std::uint32_t>(
                        RockProviderTouchGrabTargetFlagV1::
                            MatchStaticMotion)) != 0 ||
                !finitePoint(target.pivotWorldGame) ||
                !finitePoint(target.axisWorldGame) ||
                !std::isfinite(target.minimumCoordinate) ||
                !std::isfinite(target.maximumCoordinate) ||
                !std::isfinite(target.currentCoordinate) ||
                target.maximumCoordinate <= target.minimumCoordinate ||
                target.currentCoordinate < target.minimumCoordinate ||
                target.currentCoordinate > target.maximumCoordinate) {
                return false;
            }

            const float axisLengthSquared =
                target.axisWorldGame.x * target.axisWorldGame.x +
                target.axisWorldGame.y * target.axisWorldGame.y +
                target.axisWorldGame.z * target.axisWorldGame.z;
            return std::isfinite(axisLengthSquared) &&
                   std::abs(axisLengthSquared - 1.0f) <= 0.01f;
        }

        static bool validateInput(
            const RockProviderTouchGrabTargetV1* targets,
            const std::uint32_t targetCount)
        {
            for (std::uint32_t index = 0; index < targetCount; ++index) {
                if (!validateTarget(targets[index])) {
                    return false;
                }
                for (std::uint32_t prior = 0; prior < index; ++prior) {
                    if (targets[prior].targetId ==
                            targets[index].targetId ||
                        targetsConflict(targets[prior], targets[index])) {
                        return false;
                    }
                }
            }
            return true;
        }

        static bool sameTargetIdentity(
            const RockProviderTouchGrabTargetV1& left,
            const RockProviderTouchGrabTargetV1& right)
        {
            return left.targetId == right.targetId &&
                   left.targetGeneration == right.targetGeneration &&
                   left.kind == right.kind &&
                   left.bodyId == right.bodyId;
        }

        static bool targetsConflict(
            const RockProviderTouchGrabTargetV1& left,
            const RockProviderTouchGrabTargetV1& right)
        {
            const bool leftWildcard = hasTouchGrabTargetFlagV1(
                left.flags,
                RockProviderTouchGrabTargetFlagV1::MatchAnyBody);
            const bool rightWildcard = hasTouchGrabTargetFlagV1(
                right.flags,
                RockProviderTouchGrabTargetFlagV1::MatchAnyBody);
            if (!leftWildcard && !rightWildcard) {
                return left.bodyId == right.bodyId;
            }
            if (leftWildcard != rightWildcard) {
                return false;
            }
            return (left.allowedLayerMask & right.allowedLayerMask) != 0 &&
                   (left.flags & right.flags & kHandFlags) != 0 &&
                   (left.flags & right.flags & kMotionFlags) != 0;
        }

        static bool generationsMatch(
            const RockProviderTouchGrabTargetV1& target,
            const std::uint32_t worldGeneration,
            const std::uint32_t skeletonGeneration,
            const std::uint32_t providerGeneration)
        {
            return (target.worldGeneration == 0 ||
                       target.worldGeneration == worldGeneration) &&
                   (target.skeletonGeneration == 0 ||
                       target.skeletonGeneration == skeletonGeneration) &&
                   (target.providerGeneration == 0 ||
                       target.providerGeneration == providerGeneration);
        }

        static bool handMatches(
            const RockProviderTouchGrabTargetV1& target,
            const RockProviderHand hand)
        {
            if (hand == RockProviderHand::Right) {
                return hasTouchGrabTargetFlagV1(
                    target.flags,
                    RockProviderTouchGrabTargetFlagV1::AllowRightHand);
            }
            if (hand == RockProviderHand::Left) {
                return hasTouchGrabTargetFlagV1(
                    target.flags,
                    RockProviderTouchGrabTargetFlagV1::AllowLeftHand);
            }
            return false;
        }

        static bool motionMatches(
            const RockProviderTouchGrabTargetV1& target,
            const TouchGrabMotionClassV1 motionClass)
        {
            switch (motionClass) {
            case TouchGrabMotionClassV1::Static:
                return hasTouchGrabTargetFlagV1(
                    target.flags,
                    RockProviderTouchGrabTargetFlagV1::MatchStaticMotion);
            case TouchGrabMotionClassV1::Keyframed:
                return hasTouchGrabTargetFlagV1(
                    target.flags,
                    RockProviderTouchGrabTargetFlagV1::MatchKeyframedMotion);
            case TouchGrabMotionClassV1::Dynamic:
                return hasTouchGrabTargetFlagV1(
                    target.flags,
                    RockProviderTouchGrabTargetFlagV1::MatchDynamicMotion);
            case TouchGrabMotionClassV1::Other:
            default:
                return false;
            }
        }

        static TouchGrabTargetMatchV1 makeMatch(
            const Slot& slot,
            const bool wildcard)
        {
            TouchGrabTargetMatchV1 match{};
            match.matched = true;
            match.wildcard = wildcard;
            match.yieldRequested = slot.yieldRequested;
            match.ownerToken = slot.ownerToken;
            match.scopeToken = slot.scopeToken;
            match.target = slot.target;
            return match;
        }

        static RockProviderTouchGrabStateV1 makeArmedState(
            const RockProviderTouchGrabTargetV1& target,
            const std::uint64_t frameIndex)
        {
            RockProviderTouchGrabStateV1 state{};
            state.targetId = target.targetId;
            state.targetGeneration = target.targetGeneration;
            state.kind = target.kind;
            state.phase = RockProviderTouchGrabPhaseV1::Armed;
            state.bodyId = target.bodyId;
            state.referenceFormId = target.referenceFormId;
            state.referenceNativeHandle =
                target.referenceNativeHandle;
            state.currentCoordinate = target.currentCoordinate;
            state.worldGeneration = target.worldGeneration;
            state.skeletonGeneration = target.skeletonGeneration;
            state.providerGeneration = target.providerGeneration;
            state.frameIndex = frameIndex;
            if (target.kind ==
                RockProviderTouchGrabKindV1::FixedAnchor) {
                state.currentCoordinate = 0.0f;
                state.flags |= static_cast<std::uint32_t>(
                    RockProviderTouchGrabStateFlagV1::FixedAnchor);
            } else {
                state.flags |= static_cast<std::uint32_t>(
                    RockProviderTouchGrabStateFlagV1::CoordinateValid);
            }
            return state;
        }

        void pruneExpired(const std::uint64_t frameIndex)
        {
            for (auto& slot : _slots) {
                if (slot.active &&
                    !provider_lease_policy::isActive(
                        frameIndex,
                        slot.expiresAfterFrame)) {
                    slot = {};
                }
            }
        }

        bool hasScope(
            const std::uint64_t ownerToken,
            const std::uint64_t scopeToken) const
        {
            return std::any_of(
                _slots.begin(),
                _slots.end(),
                [&](const Slot& slot) {
                    return slot.active &&
                           slot.ownerToken == ownerToken &&
                           slot.scopeToken == scopeToken;
                });
        }

        std::size_t scopeCount() const
        {
            std::array<std::pair<std::uint64_t, std::uint64_t>,
                kMaximumScopes>
                scopes{};
            std::size_t count = 0;
            for (const auto& slot : _slots) {
                if (!slot.active) {
                    continue;
                }
                const auto identity =
                    std::pair{ slot.ownerToken, slot.scopeToken };
                bool found = false;
                for (std::size_t index = 0; index < count; ++index) {
                    if (scopes[index] == identity) {
                        found = true;
                        break;
                    }
                }
                if (!found && count < scopes.size()) {
                    scopes[count++] = identity;
                }
            }
            return count;
        }

        std::size_t activeCountOutsideScope(
            const std::uint64_t ownerToken,
            const std::uint64_t scopeToken) const
        {
            return static_cast<std::size_t>(std::count_if(
                _slots.begin(),
                _slots.end(),
                [&](const Slot& slot) {
                    return slot.active &&
                           (slot.ownerToken != ownerToken ||
                               slot.scopeToken != scopeToken);
                }));
        }

        bool clearScopeUnchecked(
            const std::uint64_t ownerToken,
            const std::uint64_t scopeToken)
        {
            bool cleared = false;
            for (auto& slot : _slots) {
                if (slot.active &&
                    slot.ownerToken == ownerToken &&
                    slot.scopeToken == scopeToken) {
                    slot = {};
                    cleared = true;
                }
            }
            return cleared;
        }

        Slot* firstFreeSlot()
        {
            for (auto& slot : _slots) {
                if (!slot.active) {
                    return &slot;
                }
            }
            return nullptr;
        }

        std::uint64_t nextSequence()
        {
            const auto sequence = _nextStateSequence++;
            if (_nextStateSequence == 0) {
                _nextStateSequence = 1;
            }
            return sequence == 0 ? nextSequence() : sequence;
        }

        std::array<Slot, kMaximumTargets> _slots{};
        std::uint64_t _nextStateSequence{ 1 };
    };
}
