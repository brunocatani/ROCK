#pragma once

#include <cstdint>

namespace rock::native_scope_data
{
    using ManualScopeQuery = bool (*)(const void* weaponIdentity, const void* instanceIdentity) noexcept;
    // Install only after F4SE's trampoline exists. All native callsites are
    // checked before any patch; callback data never outlives its native call.
    [[nodiscard]] bool install(ManualScopeQuery query);
    // Establish the only thread allowed to query ROCK's live scope owner.
    void beginGameFrame();
    // Game-thread publication; native UI/render/equip callbacks read only this atomic mode.
    void setEnabled(bool enabled);
    // Game-thread cleanup after releasing a manual scope transition.
    void restoreNativeHousing();
    // Caller verifies the current WSScope identity. Preserve its open/closed
    // lens visibility when a manual hold needs to configure the housing.
    [[nodiscard]] bool configureManual(void* worldScope, std::uint32_t overlay);
    void reportDiagnostics();
}
