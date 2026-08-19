#pragma once

#include <cstddef>
#include <cstdint>

namespace rock::entry_trampoline_hook
{
    /*
     * Installs a byte-validated entry trampoline on a native function: the
     * first `stolenBytes` (>= 14, must be position-independent instructions)
     * are compared against `expectedPrefix` and, only on an exact match,
     * relocated into a fresh executable trampoline followed by an absolute
     * jump back; the function entry is patched with an absolute jump to
     * `hook`. On success `original` receives the trampoline (callable as the
     * original function). Fails closed: any mismatch or OS failure leaves the
     * target untouched and returns false.
     *
     * `targetModuleOffset` is a module-relative offset into the running
     * FO4VR binary (resolved via REL::Offset). Install once at startup;
     * there is no uninstall.
     */
    [[nodiscard]] bool install(
        const char* label,
        std::uintptr_t targetModuleOffset,
        const std::uint8_t* expectedPrefix,
        std::size_t stolenBytes,
        void* hook,
        void*& original);
}
