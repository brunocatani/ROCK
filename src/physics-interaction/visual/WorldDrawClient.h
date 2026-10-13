#pragma once

#include <cstdint>

namespace rpsui::sdk
{
    struct WorldDrawPublicationV1;
}

/*
 * RPS UI Framework draws ROCK's world indicators and diagnostics. It is
 * optional: without it ROCK runs normally and draws nothing. ROCK keeps one
 * RPS UI owner whose frame callback composes and submits everything ROCK shows,
 * at render time, so collider poses captured after the physics step are the
 * ones drawn.
 */
namespace rock::world_draw_client
{
    using FrameHandler = void (*)() noexcept;

    // Game thread. Binds RPS UI once its DLL is loaded and installs handler as
    // ROCK's frame callback. Cheap once bound; attempts are spaced while RPS UI
    // is absent or refuses.
    bool ensureBound(FrameHandler handler) noexcept;

    // Frame-handler thread.
    bool submit(const rpsui::sdk::WorldDrawPublicationV1& publication) noexcept;
    bool registerMesh(std::uint64_t meshId, const float* positions, std::uint32_t vertexCount,
        const std::uint32_t* indices, std::uint32_t indexCount) noexcept;
    // Any thread.
    void releaseMesh(std::uint64_t meshId) noexcept;
}
