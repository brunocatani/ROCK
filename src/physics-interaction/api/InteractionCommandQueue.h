#pragma once

#include "api/ROCKProviderApi.h"

#include "RE/Bethesda/TESObjectREFRs.h"

namespace rock::provider
{
    struct QueuedInteractionCommandV1
    {
        std::uint64_t ownerToken{ 0 };
        std::uint64_t commandId{ 0 };
        RockProviderInteractionCommandKindV1 kind{ RockProviderInteractionCommandKindV1::Unknown };
        RockProviderForceGrabRequestV1 forceGrab{};
        RE::ObjectRefHandle targetHandle{};
    };

    [[nodiscard]] bool dequeueInteractionCommandV1(QueuedInteractionCommandV1& outCommand);
    void completeInteractionCommandV1(const RockProviderInteractionCommandResultV1& result);
    void clearInteractionCommandsForProviderLossV1(RockProviderInteractionFailureV1 failure);
}
