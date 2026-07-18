#include "physics-interaction/weapon/AuthoredWeaponGripAuthorityPolicy.h"

int main()
{
    using namespace rock::authored_weapon_grip_authority_policy;

    static_assert(completeFiringFingerPose(0x7FFF));
    static_assert(!completeFiringFingerPose(0x3FFF));
    static_assert(!completeFiringFingerPose(0));

    static_assert(shouldAcceptPublication(false, false, false));
    static_assert(shouldAcceptPublication(false, true, false));
    static_assert(shouldAcceptPublication(true, false, false));
    static_assert(shouldAcceptPublication(true, false, true));
    static_assert(shouldAcceptPublication(true, true, true));
    static_assert(!shouldAcceptPublication(true, true, false));

    return 0;
}
