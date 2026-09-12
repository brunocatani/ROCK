#pragma once

#include "physics-interaction/weapon/AuthoredWeaponGripCacheFormat.h"

#include <cstdint>

namespace rock::authored_weapon_grip_cache
{
    // Startup-only disk read. Every gameplay lookup after this call is memory-only.
    void preload();

    [[nodiscard]] bool makeCacheKey(
        std::uint32_t runtimeWeaponFormId,
        std::uint64_t pGripVariantKey,
        std::uint64_t instanceContentKey,
        std::uint64_t graphProfileKey,
        bool inPowerArmor,
        CacheKey& out);

    [[nodiscard]] bool find(const CacheKey& key, CacheRecord& out);

    // Updates the bounded in-memory store synchronously and queues one atomic file replacement.
    void save(CacheRecord record);
}
