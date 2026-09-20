#include "physics-interaction/weapon/EquippedWeaponDropMomentum.h"

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
    using namespace rock::equipped_weapon_drop_momentum;

    bool ok = true;

    ok &= expectFalse("discovery sequence cannot settle itself", completedSettleStep(42, 42));
    ok &= expectTrue("later solve sequence completes settling", completedSettleStep(42, 43));
    ok &= expectFalse("publication budget does not expire before its solve bound", publicationProgressStalled(10, 189, 180));
    ok &= expectTrue("publication budget expires at its solve bound", publicationProgressStalled(10, 190, 180));
    ok &= expectFalse("paused or reset solve sequence cannot expire publication", publicationProgressStalled(10, 9, 180));
    ok &= expectFalse("zero publication budget disables expiry", publicationProgressStalled(10, 1000, 0));

    const BodyIdentityKey bodyIdentity{
        .bodyId = 11,
        .motionId = 7,
        .motionFirstBodyId = 11,
        .shapeIdentity = 0x1000,
        .owningNodeIdentity = 0x2000,
        .collisionObjectIdentity = 0x3000,
        .physicsSystemInstanceIdentity = 0x4000,
    };
    ok &= expectTrue("identical native body generation is stable", sameBodyIdentity(bodyIdentity, bodyIdentity));
    auto rebuiltIdentity = bodyIdentity;
    rebuiltIdentity.physicsSystemInstanceIdentity = 0x5000;
    ok &= expectFalse("reused body and motion IDs do not hide a native rebuild", sameBodyIdentity(bodyIdentity, rebuiltIdentity));
    rebuiltIdentity = bodyIdentity;
    rebuiltIdentity.motionId = 8;
    ok &= expectFalse("motion changes invalidate native identity", sameBodyIdentity(bodyIdentity, rebuiltIdentity));
    rebuiltIdentity = bodyIdentity;
    rebuiltIdentity.shapeIdentity = 0x1100;
    ok &= expectFalse("shape changes invalidate native identity", sameBodyIdentity(bodyIdentity, rebuiltIdentity));
    rebuiltIdentity = bodyIdentity;
    rebuiltIdentity.owningNodeIdentity = 0x2100;
    ok &= expectFalse("owner-node changes invalidate native identity", sameBodyIdentity(bodyIdentity, rebuiltIdentity));
    rebuiltIdentity = bodyIdentity;
    rebuiltIdentity.collisionObjectIdentity = 0x3100;
    ok &= expectFalse("collision-object changes invalidate native identity", sameBodyIdentity(bodyIdentity, rebuiltIdentity));

    if (!ok) {
        std::printf("EquippedWeaponDropMomentumPolicyTests failed\n");
        return 1;
    }

    std::printf("EquippedWeaponDropMomentumPolicyTests passed\n");
    return 0;
}
