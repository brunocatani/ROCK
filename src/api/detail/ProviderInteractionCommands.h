#pragma once

#include "api/detail/ProviderApiState.h"

namespace rock::provider::detail
{
    void clearInteractionCommandsForOwnerLocked(
        std::uint64_t ownerToken,
        RockProviderInteractionFailureV1 failure);
}
