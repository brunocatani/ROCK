#pragma once
#include "Boundary.h"
#include "api/EventStreams.h"

namespace rock::api::boundary {
    template<class Event> Status copyEventStream(OwnerToken owner, InterfaceId family,
        std::uint64_t after, Event* output, std::uint32_t capacity,
        std::uint32_t maximum, StreamV1* state) noexcept {
        if (!state) return Status::InvalidArgument;
        *state = {};
        if (capacity && !output) return Status::InvalidArgument;
        if (capacity > maximum) return Status::CapacityFull;
        for (std::uint32_t i = 0; i < capacity; ++i)
            if (const auto status = checkOutput(output + i); status != Status::Ok) return status;
        return invoke(owner, family, 1, false, [&]() {
            return provider::events::copy(owner, after, output, capacity, *state);
        });
    }
}
