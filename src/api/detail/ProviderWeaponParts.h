#pragma once

#include "api/detail/ProviderApiState.h"

namespace rock::provider::detail
{
    void clearWeaponPartTargetsForOwnerLocked(std::uint64_t ownerToken);
    void clearWeaponPartDrivesForOwnerLocked(std::uint64_t ownerToken);
    void pruneExpiredWeaponPartDrivesLocked(std::uint64_t frameIndex);
}
