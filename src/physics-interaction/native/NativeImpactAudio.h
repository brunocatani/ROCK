#pragma once

#include <cstdint>

namespace RE { class hknpWorld; }

namespace rock::native_impact_audio
{
    // Process-lifetime hooks installed during Load, before physics callbacks.
    [[nodiscard]] bool install() noexcept;

    // Physics-callback-only observation. No engine pointers escape the call.
    // Diagnostics follow the existing grab-frame trace switch and async writer.
    void observeManifold(RE::hknpWorld* world, std::uint32_t bodyA,
        std::uint32_t bodyB, std::uint32_t shapeKeyA, std::uint32_t shapeKeyB) noexcept;
}
