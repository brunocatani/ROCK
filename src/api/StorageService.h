#pragma once
#include <cstdint>
namespace RE { class hknpWorld; }
namespace rock { class PhysicsInteraction; struct PhysicsFrameContext; }
namespace rock::storage_service
{
    void update(PhysicsInteraction&, const PhysicsFrameContext&);
    void shutdown(RE::hknpWorld* validWorld);
    bool allowsGrab(std::uint32_t reference, std::uint64_t owner = 0);
    bool ownsStored(std::uint64_t owner,std::uint32_t reference);
}
