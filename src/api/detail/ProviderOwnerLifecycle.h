#pragma once

#include <cstdint>

namespace rock::provider::detail
{
    void clearOwnerStateAfterCallbackFault(std::uint64_t ownerToken);
}
