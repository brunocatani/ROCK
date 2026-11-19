// The checks a hook makes on the game's code before it patches, calls or
// reads anything there.
#pragma once

#include <REL/Relocation.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <source_location>
#include <span>

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

    // True when the function at `moduleOffset` still starts with
    // `nativePrologue`, or another plugin's detour sits at its entry. Not the
    // same rule as bytesMatch: this one accepts an entry another plugin has
    // hooked, so it is for functions ROCK calls or chains behind, never for
    // instructions ROCK overwrites. Load time only (RPS HookChain::resolveEntry).
    [[nodiscard]] bool entryIsNativeOrDetoured(std::uintptr_t moduleOffset,
        std::span<const std::uint8_t> nativePrologue) noexcept;

    // True when `address` lies inside the game's code section.
    [[nodiscard]] inline bool isInGameText(const std::uintptr_t address) noexcept
    {
        const auto text = REL::Module::get().segment(REL::Segment::text);
        return address >= text.address() && address < text.address() + text.size();
    }
}
