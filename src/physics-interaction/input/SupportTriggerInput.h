#pragma once
#include <ROCK/Input.h>

namespace rock::support_trigger_input
{
    api::Status set(api::OwnerToken, const api::input::SupportTriggerRequest&);
    api::Status clear(api::OwnerToken);
    api::Status read(api::input::SupportTriggerState&);
    void clearOwner(api::OwnerToken);
    void reset();
    void beginFrame(std::uint64_t);
    // The colliders arriving change the weapon's interaction key, not the weapon:
    // the lease follows it, so the trigger never drops or needs a fresh release.
    void adoptWeaponKey(std::uint64_t from, std::uint64_t to);
    // OpenVR/native hooks use only atomics; no provider calls or locks.
    std::uint64_t gameMask(bool left, std::uint64_t physicalPressed) noexcept;
    bool blocked(bool left) noexcept;
}
