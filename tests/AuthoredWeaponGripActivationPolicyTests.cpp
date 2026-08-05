#include "physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h"

#include <cassert>
#include <cmath>

using namespace rock::authored_weapon_grip_activation_policy;

namespace
{
    bool near(const float lhs, const float rhs, const float tolerance = 0.0001f)
    {
        return std::abs(lhs - rhs) <= tolerance;
    }
}

int main()
{
    static_assert(resolveWeaponFamily(WeaponFamilyInput{
        .effectiveEquipSlotFormID = kRightHandEquipSlotFormID,
        .equippedWeaponPresent = true,
    }) == WeaponFamily::OneHandGun);
    static_assert(resolveWeaponFamily(WeaponFamilyInput{
        .effectiveEquipSlotFormID = kBothHandsEquipSlotFormID,
        .equippedWeaponPresent = true,
    }) == WeaponFamily::TwoHandGun);
    static_assert(resolveWeaponFamily(WeaponFamilyInput{
        .effectiveEquipSlotFormID = kRightHandEquipSlotFormID,
        .equippedWeaponPresent = true,
        .meleeOrUnarmed = true,
    }) == WeaponFamily::Unsupported);
    static_assert(resolveWeaponFamily(WeaponFamilyInput{
        .effectiveEquipSlotFormID = kBothHandsEquipSlotFormID,
        .equippedWeaponPresent = true,
        .heavyGun = true,
    }) == WeaponFamily::Unsupported);
    static_assert(resolveWeaponFamily(WeaponFamilyInput{
        .effectiveEquipSlotFormID = 0xDEADBEEFu,
        .equippedWeaponPresent = true,
    }) == WeaponFamily::Unknown);

    const Vec3 origin{};
    const Vec3 left{ -1.0f, 0.0f, 0.0f };
    const Vec3 down{ 0.0f, 0.0f, -1.0f };

    auto result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::OneHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ -5.0f, 0.0f, 0.0f },
        .leftAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.spatialPass);
    assert(result.selectedCone == AllowedCone::Left);
    assert(near(result.leftDot, 1.0f));

    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::OneHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ 5.0f, 0.0f, 0.0f },
        .leftAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(!result.directionPass);
    assert(!result.spatialPass);

    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ 0.0f, 0.0f, -8.0f },
        .leftAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.spatialPass);
    assert(result.selectedCone == AllowedCone::Down);
    assert(near(result.downDot, 1.0f));

    constexpr float diagonal = 0.70710678118654752440f;
    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::OneHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ -diagonal * 10.0f, diagonal * 10.0f, 0.0f },
        .leftAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.directionPass);
    assert(near(result.leftDot, kActivationConeMinimumDot));

    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ -20.0f, 0.0f, 0.0f },
        .leftAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.directionPass);
    assert(!result.radialPass);
    assert(!result.spatialPass);

    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::OneHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ 0.01f, 0.0f, 0.0f },
        .leftAxisWorld = left,
        .downAxisWorld = down,
        .lastStableDirectionWorld = left,
        .radialCapGameUnits = 12.0f,
        .lastStableDirectionValid = true,
    });
    assert(result.usedLastStableDirection);
    assert(result.directionPass);
    assert(result.spatialPass);

    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ 0.0f, 0.0f, -5.0f },
        .leftAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
        .semanticTargetEligible = false,
    });
    assert(result.radialPass);
    assert(result.directionPass);
    assert(!result.semanticPass);
    assert(!result.spatialPass);

    return 0;
}
