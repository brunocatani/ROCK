#pragma once
#include <cstdint>

namespace RE { class Actor; class TESForm; }

namespace rock::native_equipped_model_slot
{
    // Actual native biped storage, not an extra scene/world reference. The
    // lease survives skeleton/physics replacement and ends after native
    // unequip has cleared both current and buffered biped records.
    bool install() noexcept;
    bool reserve(RE::TESForm* weapon) noexcept;
    bool releaseIfUnused() noexcept;
    void resetForGameLoad() noexcept;
    std::uint32_t resolve(RE::Actor* actor, RE::TESForm* weapon, std::uint32_t index) noexcept;
}
