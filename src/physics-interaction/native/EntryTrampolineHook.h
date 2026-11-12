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
     * original function). When another plugin's jump already replaces the
     * prologue, the hook chains instead: a 5-byte jump through an F4SE stub
     * replaces that jump and `original` is the other plugin's detour (see
     * RPS/Runtime/HookChain.h). Fails closed otherwise: any other mismatch or
     * OS failure leaves the target untouched and returns false.
     *
     * `targetModuleOffset` is a module-relative offset into the running
     * FO4VR binary (resolved via REL::Offset). Install once at startup;
     * there is no uninstall.
     */
    [[nodiscard]] bool install(const char* label, std::uintptr_t targetModuleOffset, const std::uint8_t* expectedPrefix,
        std::size_t stolenBytes, void* hook, void*& original);
}
