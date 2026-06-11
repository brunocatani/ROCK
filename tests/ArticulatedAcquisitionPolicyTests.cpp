#include "physics-interaction/object/GrabTargetKind.h"

#include <cstdio>

namespace
{
    bool expectProfile(
        const char* label,
        rock::grab_target::HandlingProfile actual,
        rock::grab_target::HandlingProfile expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected profile=%u got profile=%u\n", label, static_cast<unsigned>(expected), static_cast<unsigned>(actual));
        return false;
    }

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

    ok &= expectProfile("loose object profile", handlingProfile(Kind::LooseObject), HandlingProfile::OrdinaryLooseObject);
    ok &= expectFalse("loose object does not prefer mechanical scope", prefersMechanicalScope(Kind::LooseObject));
    ok &= expectFalse("loose object does not relax pocket authority", relaxesStrictPocketAuthorityForMechanicalGrab(Kind::LooseObject));

    ok &= expectProfile("dead actor profile", handlingProfile(Kind::DeadActorBody), HandlingProfile::CloseMechanicalPreferred);
    ok &= expectTrue("dead actor remains hand-pocket-only", requiresHandPocketGrab(Kind::DeadActorBody));
    ok &= expectFalse("dead actor still cannot use far selection", canUseFarSelection(Kind::DeadActorBody));
    ok &= expectFalse("dead actor still cannot use pull", canUseRockDynamicPull(Kind::DeadActorBody));
    ok &= expectTrue("dead actor prefers mechanical scope", prefersMechanicalScope(Kind::DeadActorBody));
    ok &= expectTrue("dead actor relaxes strict pocket authority", relaxesStrictPocketAuthorityForMechanicalGrab(Kind::DeadActorBody));

    ok &= expectProfile("movable static profile", handlingProfile(Kind::DynamicMovableStatic), HandlingProfile::CloseMechanicalPreferred);
    ok &= expectTrue("movable static prefers mechanical scope", prefersMechanicalScope(Kind::DynamicMovableStatic));
    ok &= expectFalse("movable static keeps strict pocket authority", relaxesStrictPocketAuthorityForMechanicalGrab(Kind::DynamicMovableStatic));

    ok &= expectProfile("detached gore profile", handlingProfile(Kind::DetachedGore), HandlingProfile::CloseSimplePhysical);
    ok &= expectFalse("detached gore does not force mechanical scope", prefersMechanicalScope(Kind::DetachedGore));
    ok &= expectFalse("detached gore keeps strict pocket authority", relaxesStrictPocketAuthorityForMechanicalGrab(Kind::DetachedGore));

    ok &= expectProfile("actor equipment profile", handlingProfile(Kind::ActorEquipment), HandlingProfile::ActorDriven);
    ok &= expectProfile("blocked body profile", handlingProfile(Kind::BlockedWholeActorBody), HandlingProfile::Blocked);

    return ok ? 0 : 1;
}
