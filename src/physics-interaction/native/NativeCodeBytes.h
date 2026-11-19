// The live-byte check a hook makes at its site in the game's code before it
// patches, calls or reads anything there.
#pragma once

#include <REL/Relocation.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <source_location>

#include "physics-interaction/native/NativeMemory.h"

namespace rock::native_code
{
    // True when the game's bytes at `moduleOffset` are readable and equal
    // `expected`. An accepted game version does not prove an address: every
    // hard-coded site is compared with the bytes it was written for, and the
    // hook fails closed on any difference.
    template <std::size_t N>
    [[nodiscard]] inline bool bytesMatch(const std::uintptr_t moduleOffset, const std::array<std::uint8_t, N>& expected,
        const std::source_location& site = std::source_location::current()) noexcept
    {
        std::array<std::uint8_t, N> actual{};
        return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(moduleOffset).address()),
                   actual.data(), actual.size(), site) &&
            actual == expected;
    }
}
