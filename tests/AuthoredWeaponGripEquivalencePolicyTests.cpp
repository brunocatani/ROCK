#include "physics-interaction/weapon/AuthoredWeaponGripEquivalencePolicy.h"

int main()
{
    using namespace rock::authored_weapon_grip_equivalence_policy;

    static_assert(kStableSamplesRequired > 1);
    static_assert(advanceStableSampleCount(false, 0, 0.0f, 0.0f, 0.0f) == 1);
    static_assert(advanceStableSampleCount(true, 1,
                      kStableTranslationToleranceGameUnits,
                      kStableRotationToleranceDegrees,
                      kStableScaleTolerance) == 2);
    static_assert(advanceStableSampleCount(true, 4, 0.0f, 0.0f, 0.0f) == 5);
    static_assert(advanceStableSampleCount(true, kStableSamplesRequired, 0.0f, 0.0f, 0.0f) == kStableSamplesRequired);

    static_assert(advanceStableSampleCount(true, 5, kStableTranslationToleranceGameUnits + 0.001f, 0.0f, 0.0f) == 1);
    static_assert(advanceStableSampleCount(true, 5, 0.0f, kStableRotationToleranceDegrees + 0.001f, 0.0f) == 1);
    static_assert(advanceStableSampleCount(true, 5, 0.0f, 0.0f, kStableScaleTolerance + 0.0001f) == 1);
    static_assert(!readyForStableReport(kStableSamplesRequired - 1));
    static_assert(readyForStableReport(kStableSamplesRequired));
    return 0;
}
