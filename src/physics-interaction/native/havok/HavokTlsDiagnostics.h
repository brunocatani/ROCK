#pragma once

#include <cstdint>

namespace rock::havok_tls_diagnostics
{
    /*
     * Future custom grab authority needs to know whether FO4VR's body setter
     * wrappers direct-write or enter command-queue mode from the real physics
     * listener callback. This diagnostic reads only the Ghidra-mapped TLS
     * state bytes; it does not alter TLS or drive gameplay behavior.
     */
    inline constexpr std::uintptr_t kCommandModeByteOffset = 0x1528;
    inline constexpr std::uintptr_t kPhysicsContextByteOffset = 0x1529;
    inline constexpr std::uintptr_t kThreadCommandIndexOffset = 0x152C;

    struct CommandQueueState
    {
        bool readable = false;
        std::uint32_t tlsIndex = 0xFFFF'FFFFu;
        const void* tlsBlock = nullptr;
        std::uint8_t commandMode = 0;
        std::uint8_t physicsContext = 0;
        std::uint32_t threadCommandIndex = 0xFFFF'FFFFu;

        [[nodiscard]] bool wrapperQueueModeActive() const noexcept { return readable && commandMode != 0; }
    };

    CommandQueueState readCurrentCommandQueueState();
}
