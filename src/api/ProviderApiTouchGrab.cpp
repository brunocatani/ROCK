#define ROCK_API_EXPORTS
/*
 * Build the DLL side of the public provider ABI.
 *
 * TOUCH GRAB: the scope lifecycle for consumer-registered touch grab targets.
 *
 * The registry itself is TouchGrabRegistry; this file owns only the API surface and
 * the internal bridges over it. Scope entry points take the consumer lock and the
 * touch lock together in one scoped_lock.
 */
#include "api/detail/ProviderApiEntryPoints.h"
#include "api/detail/ProviderApiCore.h"
#include "api/detail/ProviderApiState.h"

#include <mutex>

namespace rock::provider::detail
{
    RockProviderResultV1 ROCK_PROVIDER_CALL
    apiSetTouchGrabTargetsForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const RockProviderTouchGrabTargetV1* targets,
        const std::uint32_t targetCount)
    {
        if (targetCount > ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGETS_V1 ||
            (targetCount != 0 && !targets)) {
            return RockProviderResultV1::InvalidArgument;
        }
        for (std::uint32_t index = 0; index < targetCount; ++index) {
            const auto entryResult = validateEntry(&targets[index]);
            if (entryResult != RockProviderResultV1::Ok) {
                return entryResult;
            }
            const auto generationResult = validateGenerationGuards(
                targets[index].worldGeneration,
                targets[index].skeletonGeneration,
                targets[index].providerGeneration);
            if (generationResult != RockProviderResultV1::Ok) {
                return generationResult;
            }
        }
        if (ownerToken == 0 || scopeToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::TouchGrabTargets);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        switch (s_touchGrabTargets.setScope(
            ownerToken,
            scopeToken,
            targets,
            targetCount,
            currentProviderFrameIndex())) {
        case TouchGrabRegistry::RegistrationResult::Ok:
            return RockProviderResultV1::Ok;
        case TouchGrabRegistry::RegistrationResult::CapacityFull:
            return RockProviderResultV1::CapacityFull;
        case TouchGrabRegistry::RegistrationResult::OwnerConflict:
            return RockProviderResultV1::OwnerConflict;
        case TouchGrabRegistry::RegistrationResult::InvalidArgument:
        default:
            return RockProviderResultV1::InvalidArgument;
        }
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
    apiClearTouchGrabTargetsForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken)
    {
        if (ownerToken == 0 || scopeToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::TouchGrabTargets);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        return s_touchGrabTargets.clearScope(ownerToken, scopeToken) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
    apiCopyTouchGrabStatesForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        RockProviderTouchGrabStateV1* outStates,
        const std::uint32_t maxStates,
        std::uint32_t* outStateCount)
    {
        const auto entryResult = validateArrayEntry(
            ownerToken,
            outStates,
            maxStates,
            outStateCount,
            EntryThreadPolicy::AnyThread,
            scopeToken != 0);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::TouchGrabTargets);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        *outStateCount = s_touchGrabTargets.copyStates(
            ownerToken,
            scopeToken,
            outStates,
            maxStates,
            currentProviderFrameIndex());
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
    apiRequestTouchGrabYieldV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const std::uint64_t targetId,
        const std::uint32_t targetGeneration)
    {
        if (ownerToken == 0 || scopeToken == 0 || targetId == 0 ||
            targetGeneration == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::TouchGrabTargets);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        return s_touchGrabTargets.requestYield(
                   ownerToken,
                   scopeToken,
                   targetId,
                   targetGeneration,
                   currentProviderFrameIndex()) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
    }

}
namespace rock::provider
{
    using namespace detail;

    bool resolveTouchGrabTargetV1(
        const std::uint32_t bodyId,
        const std::uint32_t collisionLayer,
        const TouchGrabMotionClassV1 motionClass,
        const RockProviderHand hand,
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration,
        TouchGrabTargetMatchV1& outMatch)
    {
        std::scoped_lock lock(s_touchGrabMutex);
        outMatch = s_touchGrabTargets.resolve(
            bodyId,
            collisionLayer,
            motionClass,
            hand,
            worldGeneration,
            skeletonGeneration,
            providerGeneration,
            currentProviderFrameIndex());
        return outMatch.matched;
    }

    bool currentTouchGrabTargetV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const std::uint64_t targetId,
        const std::uint32_t targetGeneration,
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration,
        TouchGrabTargetMatchV1& outMatch)
    {
        std::scoped_lock lock(s_touchGrabMutex);
        return s_touchGrabTargets.currentTarget(
            ownerToken,
            scopeToken,
            targetId,
            targetGeneration,
            worldGeneration,
            skeletonGeneration,
            providerGeneration,
            currentProviderFrameIndex(),
            outMatch);
    }

    bool publishTouchGrabStateV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const RockProviderTouchGrabStateV1& state)
    {
        std::scoped_lock lock(s_touchGrabMutex);
        return s_touchGrabTargets.publishState(
            ownerToken,
            scopeToken,
            state,
            currentProviderFrameIndex());
    }

    void acknowledgeTouchGrabYieldV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const std::uint64_t targetId,
        const std::uint32_t targetGeneration)
    {
        std::scoped_lock lock(s_touchGrabMutex);
        s_touchGrabTargets.acknowledgeYield(
            ownerToken,
            scopeToken,
            targetId,
            targetGeneration);
    }
}
