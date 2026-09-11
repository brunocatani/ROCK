#include "physics-interaction/object/GrabTargetKind.h"

#include <cstdio>

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }
}

int main()
{
    using namespace rock::grab_target;

    bool ok = true;

    ok &= expectFalse("loose object does not prefer mechanical scope", prefersMechanicalScope(Kind::LooseObject));
    ok &= expectFalse("loose object does not relax pocket authority", relaxesStrictPocketAuthorityForMechanicalGrab(Kind::LooseObject));

    ok &= expectTrue("dead actor remains hand-pocket-only", requiresHandPocketGrab(Kind::DeadActorBody));
    ok &= expectFalse("dead actor still cannot use far selection", canUseFarSelection(Kind::DeadActorBody));
    ok &= expectFalse("dead actor still cannot use pull", canUseRockDynamicPull(Kind::DeadActorBody));
    ok &= expectTrue("dead actor prefers mechanical scope", prefersMechanicalScope(Kind::DeadActorBody));
    ok &= expectTrue("dead actor relaxes strict pocket authority", relaxesStrictPocketAuthorityForMechanicalGrab(Kind::DeadActorBody));

    ok &= expectTrue("movable static prefers mechanical scope", prefersMechanicalScope(Kind::DynamicMovableStatic));
    ok &= expectFalse("movable static keeps strict pocket authority", relaxesStrictPocketAuthorityForMechanicalGrab(Kind::DynamicMovableStatic));

    ok &= expectFalse("detached gore does not force mechanical scope", prefersMechanicalScope(Kind::DetachedGore));
    ok &= expectFalse("detached gore keeps strict pocket authority", relaxesStrictPocketAuthorityForMechanicalGrab(Kind::DetachedGore));


    return ok ? 0 : 1;
}
