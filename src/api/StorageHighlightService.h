#pragma once
#include <ROCK/Storage.h>

namespace rock
{
    class PhysicsInteraction;
    struct PhysicsFrameContext;
}

namespace rock::storage_highlight
{
    /*
     * Storage highlights: the VATS glow on a loaded reference, or on a
     * visual-only copy of its model (a ghost) placed where the caller says.
     * Thread: the ROCK frame owner, like every Storage call. Ghosts hang under
     * the reference's scene parent and are detached on expiry, owner loss and shutdown.
     */
    api::Status setHighlight(api::OwnerToken owner, const api::storage::Highlight* request) noexcept;
    api::Status clearHighlight(api::OwnerToken owner, std::uint64_t highlightId) noexcept;
    void update(PhysicsInteraction&, const PhysicsFrameContext&);
    void shutdown();
}
