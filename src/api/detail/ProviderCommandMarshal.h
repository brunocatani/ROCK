#pragma once

#include "api/detail/ProviderApiState.h"

namespace rock::provider::detail
{
    struct CommandFields
    {
        RockProviderHand hand{ RockProviderHand::None };
        std::uint32_t targetFormId{ 0 };
        std::uint32_t targetBodyId{ kProviderInvalidBodyId };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    [[nodiscard]] CommandFields commandFields(
        const QueuedInteractionCommandV1& command);
    [[nodiscard]] RockProviderInteractionCommandResultV1 makeCommandResult(
        const QueuedInteractionCommandV1& command,
        RockProviderInteractionCommandStateV1 state,
        RockProviderInteractionFailureV1 failure);
    void storeInteractionResultLocked(
        const RockProviderInteractionCommandResultV1& result);
}
