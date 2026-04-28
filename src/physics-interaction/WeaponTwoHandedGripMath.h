#pragma once

#include "WeaponTwoHandedSolver.h"

namespace frik::rock::weapon_two_handed_grip_math
{
    /*
     * Equipped weapon two-hand support has two independent ownership rules:
     * the support hand must be attached to the mesh point it actually touched,
     * and it must not also own a normal dynamic-object grab. Keeping these
     * rules as pure math/policy avoids mixing HIGGS-style weapon authority with
     * ROCK's separate dynamic object grab path.
     */

    template <class Transform, class Vector>
    inline Transform alignHandFrameToGripPoint(const Transform& handWorldTransform, const Vector& currentGripPivotWorld, const Vector& targetGripPointWorld)
    {
        Transform result = handWorldTransform;
        const Vector correction = weaponSolverSub(targetGripPointWorld, currentGripPivotWorld);
        result.translate = weaponSolverAdd(result.translate, correction);
        return result;
    }

    inline bool canStartSupportGrip(bool touchingSupportPart, bool gripPressed, bool supportHandHoldingObject)
    {
        return touchingSupportPart && gripPressed && !supportHandHoldingObject;
    }

    inline bool shouldContinueSupportGrip(bool gripPressed, bool supportHandHoldingObject)
    {
        return gripPressed && !supportHandHoldingObject;
    }

    inline bool canProcessNormalGrabInput(bool isLeft, bool equippedWeaponSupportGripActive, bool rightHandWeaponEquipped)
    {
        if (isLeft) {
            return !equippedWeaponSupportGripActive;
        }

        return !rightHandWeaponEquipped;
    }
}
