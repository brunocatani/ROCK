#include "physics-interaction/weapon/SeeThroughScopesPolicy.h"

#include <cstdio>
#include <cstring>

namespace
{
    bool expectRoute(
        const char* label,
        rock::see_through_scopes_policy::EquippedScopeRoute actual,
        rock::see_through_scopes_policy::EquippedScopeRoute expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %s got %s\n",
            label,
            rock::see_through_scopes_policy::equippedScopeRouteName(expected),
            rock::see_through_scopes_policy::equippedScopeRouteName(actual));
        return false;
    }

    bool expectString(const char* label, const char* actual, const char* expected)
    {
        if (std::strcmp(actual, expected) == 0) {
            return true;
        }

        std::printf("%s expected %s got %s\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace rock::see_through_scopes_policy;

    bool ok = true;

    ok &= expectRoute("STS wins over native fallback",
        chooseEquippedScopeRoute(EquippedScopeRouteInput{
            .activeStsScopeMods = 1,
            .activeNativeScopeMods = 1,
            .stsScopeMeshRenderable = true,
        }),
        EquippedScopeRoute::StsPreferred);

    ok &= expectRoute("STS degrades to native fallback when the STS scope mesh is unavailable",
        chooseEquippedScopeRoute(EquippedScopeRouteInput{
            .activeStsScopeMods = 1,
            .activeNativeScopeMods = 0,
            .stsScopeMeshRenderable = false,
        }),
        EquippedScopeRoute::NativeFallback);

    ok &= expectRoute("native fallback still wins when STS metadata exists without renderable STS mesh",
        chooseEquippedScopeRoute(EquippedScopeRouteInput{
            .activeStsScopeMods = 1,
            .activeNativeScopeMods = 1,
            .stsScopeMeshRenderable = false,
        }),
        EquippedScopeRoute::NativeFallback);

    ok &= expectRoute("native is used when no STS scope is active",
        chooseEquippedScopeRoute(EquippedScopeRouteInput{
            .activeStsScopeMods = 0,
            .activeNativeScopeMods = 1,
        }),
        EquippedScopeRoute::NativeFallback);

    ok &= expectRoute("loaded STS plugin without active STS scope does not force STS",
        chooseEquippedScopeRoute(EquippedScopeRouteInput{
            .activeStsScopeMods = 0,
            .activeNativeScopeMods = 0,
        }),
        EquippedScopeRoute::None);

    ok &= expectString("route name exposes STS preference",
        equippedScopeRouteName(EquippedScopeRoute::StsPreferred),
        "sts-preferred");
    ok &= expectString("route name exposes native fallback",
        equippedScopeRouteName(EquippedScopeRoute::NativeFallback),
        "native-fallback");

    return ok ? 0 : 1;
}
