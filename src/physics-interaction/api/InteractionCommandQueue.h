#pragma once

#include "api/ROCKProviderApi.h"

namespace rock::provider
{
    struct QueuedInteractionCommandV1
    {
        std::uint64_t ownerToken{ 0 };
        std::uint64_t commandId{ 0 };
        RockProviderInteractionCommandKindV1 kind{ RockProviderInteractionCommandKindV1::Unknown };
        RockProviderForceGrabRequestV1 forceGrab{};
        RockProviderForceReleaseRequestV1 forceRelease{};
        RockProviderThrownDropRequestV1 thrownDrop{};
    };

    [[nodiscard]] bool dequeueInteractionCommandV1(QueuedInteractionCommandV1& outCommand);
    [[nodiscard]] bool isInteractionCommandActiveV1(std::uint64_t ownerToken, std::uint64_t commandId);
    bool completeInteractionCommandV1(const RockProviderInteractionCommandResultV1& result);
    void clearInteractionCommandsForProviderLossV1(RockProviderInteractionFailureV1 failure);
}
