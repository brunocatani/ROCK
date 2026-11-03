#pragma once

// A consumer built for another ROCK API version is never left running with
// its ROCK features silently off: discovery reports it and the game stops at
// the main menu with an error box naming the DLL to update
// (RPS/Runtime/RequiredUpdate.h).

#include <ROCK/Abi.h>

#include <cstdint>

namespace rock::api::discovery
{
    // Any thread. Called with every discovery result; acts only on version
    // refusals. callerAddress is the discovery call's return address, which
    // lies inside the consumer's DLL.
    void noteRefusal(Status status, InterfaceId id, std::uint32_t requestedMajor, const void* callerAddress) noexcept;

    // Once, from the F4SE GameLoaded message.
    void watchForOutdatedConsumers() noexcept;
}
