#include "physics-interaction/weapon/grip/WeaponTransformArbiter.h"

#include "physics-interaction/weapon/TwoHandedGrip.h"

namespace rock
{
    bool WeaponTransformArbiter::requestLeftCarry(
        const CarrySource,
        RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey)
    {
        return _grip.beginPersistentEquippedCarry(
            weaponNode,
            weaponGenerationKey,
            weaponOwnershipKey);
    }

    void WeaponTransformArbiter::restoreNativeRight(
        const CarrySource,
        const char* reason)
    {
        _grip.restoreNativeRightEquippedCarry(reason);
    }

    void WeaponTransformArbiter::clearPersistentCarry(
        const CarrySource,
        const char* reason)
    {
        _grip.clearPersistentEquippedCarry(reason);
    }
}
