#pragma once
#include <ROCK/Input.h>

namespace rock::support_trigger_input {
    api::Status set(api::OwnerToken, const api::input::SupportTriggerRequest&);
    api::Status clear(api::OwnerToken);
    api::Status read(api::input::SupportTriggerState&);
    void clearOwner(api::OwnerToken);
    void reset();
    void beginFrame(std::uint64_t);
    // OpenVR/native hooks use only atomics; no provider calls or locks.
    std::uint64_t gameMask(bool left, std::uint64_t physicalPressed) noexcept;
    bool blocked(bool left) noexcept;
}
