// What the native fault boundaries share when they report a caught fault.
#pragma once

#include <REL/Relocation.h>

#include <bit>
#include <cstdint>

namespace rock::native_fault_report
{
    // The first four occurrences, then every power of two: a fault that
    // repeats every frame stays visible without filling the log.
    [[nodiscard]] inline bool shouldLogOccurrence(const std::uint64_t occurrence) noexcept
    {
        return occurrence <= 4 || std::has_single_bit(occurrence);
    }

    // The module-relative address of an instruction inside the game's code, or
    // 0 when `address` is outside it.
    [[nodiscard]] inline std::uintptr_t gameTextRelativeAddress(const std::uintptr_t address) noexcept
    {
        const auto text = REL::Module::get().segment(REL::Segment::text);
        if (text.size() == 0 || address < text.address() || address >= text.address() + text.size()) {
            return 0;
        }
        return address - REL::Module::get().base();
    }
}
