#pragma once

#include <cstdint>

namespace rock::weapon_workbench_graph_refresh
{
    [[nodiscard]] bool installHook();

    void publishObservedEquippedWeaponSignature(std::uint64_t signatureKey) noexcept;

    [[nodiscard]] std::uint64_t currentRefreshEpoch() noexcept;
}
