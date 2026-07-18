#pragma once

#include <string_view>

#include "physics-interaction/weapon/WeaponEffectGeometryPolicy.h"

namespace rock::manual_scope_target_policy
{
    /*
     * Some weapon authors ship a magnified optic model but omit Fallout's
     * native scope-overlay OMOD property. The native property remains the
     * authority when present; this narrow fallback only recognizes an active
     * model whose record or path explicitly calls itself a scope. Generic
     * sights/optics are intentionally excluded so red dots do not acquire the
     * native scope menu merely because they occupy an optical slot.
     */
    [[nodiscard]] inline constexpr bool hasExplicitScopeIdentity(
        std::string_view recordName,
        std::string_view modelPath) noexcept
    {
        if (modelPath.empty()) {
            return false;
        }

        using weapon_effect_geometry_policy::containsAsciiInsensitive;
        return containsAsciiInsensitive(recordName, "scope") ||
               containsAsciiInsensitive(modelPath, "scope");
    }

    [[nodiscard]] inline constexpr bool requiresDirectNativeTransition(
        bool nativeScopeOverlayAuthored,
        bool explicitScopeModelInstalled) noexcept
    {
        return explicitScopeModelInstalled && !nativeScopeOverlayAuthored;
    }
}
