#pragma once

#include "physics-interaction/hand/HandFrame.h"

namespace rock::weapon_grip_calibration
{
    // Change the hand-in-weapon relation, not the world carry target. Move the
    // solver's grip point by the SAME weapon-local delta, so its position in
    // the physical hand remains fixed. The weapon can then move relative to
    // the anchored hand without dragging that hand along with it.
    inline RE::NiPoint3 offsetInWeapon(const RE::NiTransform& handWeaponLocal,
        const RE::NiPoint3& handOffset, bool isLeft)
    {
        // Only mirrored left firing and right support captures use this path.
        // User offsets are additive to the calibrated hand-local baseline.
        const RE::NiPoint3 calibratedOffset = isLeft ?
            RE::NiPoint3{ handOffset.x, handOffset.y + 0.6f, handOffset.z } :
            RE::NiPoint3{ handOffset.x - 0.9f, handOffset.y, handOffset.z };
        return transformHandspaceLocalToWorld(handWeaponLocal,
            authoredHandspaceToRawHandspaceForHand(calibratedOffset, isLeft) * handWeaponLocal.scale);
    }

    inline RE::NiTransform shiftedHand(const RE::NiTransform& handWeaponLocal,
        const RE::NiPoint3& handOffset, bool isLeft)
    {
        auto result = handWeaponLocal;
        result.translate += offsetInWeapon(handWeaponLocal, handOffset, isLeft);
        return result;
    }
}
