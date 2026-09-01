#include "physics-interaction/weapon/grip/WeaponTransformArbiter.h"

#include "physics-interaction/weapon/TwoHandedGrip.h"

namespace rock
{
    bool WeaponTransformArbiter::fixedHandMayClaim() const
    {
        if (_context.shoulderSheathActive ||
            _context.pendingPrimaryOnlyGripStart) {
            return false;
        }
        // An addon-owned Pip-Boy selection is an explicit dynamic side
        // choice. Likewise, a live manual handoff is preserved regardless
        // of whether its effective ambidextrous policy comes from ROCK or
        // the addon. The fixed hand remains the fallback/default rather
        // than fighting the player's deliberate switch.
        if (_context.handAssignmentEngaged) {
            return false;
        }
        if (_context.ambidextrousHandoffEnabled &&
            _grip.isManualOwnershipActive()) {
            return false;
        }
        return true;
    }

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
