#pragma once

#include <ROCK/WeaponParts.h>
#include <cstdint>

namespace rock::provider
{
    // A part target or drive as its owner sent it, plus the live source node
    // its source key or name resolved to on the weapon generation. The node is
    // ROCK-private and never crosses the ABI; TargetFlag::MatchSourceKey
    // matches against it.
    struct PartTarget : api::weaponparts::Target
    {
        std::uintptr_t sourceRoot{ 0 };
    };

    struct PartDrive : api::weaponparts::DriveTarget
    {
        std::uintptr_t sourceRoot{ 0 };
    };

    // A part as ROCK's grip code sees it, resolved against the active targets.
    // Owns a copy of the source name so it can outlive the evidence snapshot.
    struct PartQuery
    {
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint32_t bodyId{ api::weaponparts::kNoBody };
        std::uint32_t partKind{ 0 };
        std::uint32_t reloadRole{ 0 };
        std::uint32_t supportRole{ 0 };
        std::uint32_t socketRole{ 0 };
        std::uint32_t actionRole{ 0 };
        std::uintptr_t sourceRoot{ 0 };
        char sourceName[64]{};
    };
}
