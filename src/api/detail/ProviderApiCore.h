#pragma once

#include "api/detail/ProviderApiState.h"

namespace rock::provider::detail
{
    struct FrameCallbackInvocationResult
    {
        bool healthy{ true };
        std::uint32_t exceptionCode{ 0 };
        std::uintptr_t exceptionAddress{ 0 };
    };

    void clearCallbackSlot(std::uint64_t callbackToken);
    [[nodiscard]] FrameCallbackInvocationResult invokeFrameCallbackSafely(
        RockProviderFrameCallback callback,
        const RockProviderFrameSnapshot* snapshot,
        void* userData);
    [[nodiscard]] bool claimOrValidateAnimationOwnerThread();
    [[nodiscard]] bool onAnimationOwnerThread();
    [[nodiscard]] bool consumerHasCapabilityLocked(
        std::uint64_t ownerToken,
        RockProviderConsumerCapabilityV1 capability);
}
