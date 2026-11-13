#pragma once

// A consumer built for another ROCK API version is refused by discovery and
// keeps running without ROCK features. ROCK logs it once, naming the DLL to
// update; the game is never stopped.

#include <ROCK/Abi.h>

#include <cstdint>

namespace rock::api::discovery
{
    // Any thread. Called with every discovery result; acts only on version
    // refusals. callerAddress is the discovery call's return address, which
    // lies inside the consumer's DLL.
    void noteRefusal(Status status, InterfaceId id, std::uint32_t requestedMajor, const void* callerAddress) noexcept;

    // Any thread. RPS_UI_Framework.dll is loaded but older than the API ROCK was
    // built for, so ROCK's world drawing, ammo counters and input cooperation
    // stay off.
    void noteRpsUiWithoutDiscovery() noexcept;
}
