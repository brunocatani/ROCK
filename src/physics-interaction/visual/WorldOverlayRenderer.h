#pragma once

namespace rock::world_overlay_renderer
{
    // Shared D3D/OpenVR transport. Gameplay publications do not start the
    // diagnostic shape worker or depend on diagnostic visibility/settings.
    bool EnsureInstalled();
    void NotifyPublication() noexcept;
}
