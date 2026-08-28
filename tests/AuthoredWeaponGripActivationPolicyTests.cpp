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
        .effectiveEquipSlotFormID =
            kBothHandsLeftOptionalEquipSlotFormID,
        .equippedWeaponPresent = true,
    }) == WeaponFamily::OneHandGun);
    static_assert(resolveWeaponFamily(WeaponFamilyInput{
        .effectiveEquipSlotFormID = kRightHandEquipSlotFormID,
        .equippedWeaponPresent = true,
        .meleeOrUnarmed = true,
    }) == WeaponFamily::Unsupported);
    static_assert(resolveWeaponFamily(WeaponFamilyInput{
        .effectiveEquipSlotFormID =
            kBothHandsLeftOptionalEquipSlotFormID,
        .equippedWeaponPresent = true,
        .meleeOrUnarmed = true,
    }) == WeaponFamily::Unsupported);
    static_assert(resolveWeaponFamily(WeaponFamilyInput{
        .effectiveEquipSlotFormID = 0xDEADBEEFu,
        .equippedWeaponPresent = true,
    }) == WeaponFamily::Unknown);

    const Vec3 origin{};
    const Vec3 left{ -1.0f, 0.0f, 0.0f };
    const Vec3 down{ 0.0f, 0.0f, -1.0f };
    constexpr float diagonal = 0.70710678118654752440f;

    const auto activationBoundary =
        resolveActivationBoundaryDimensions(12.0f);
    assert(activationBoundary.valid);
    assert(near(
        activationBoundary.axialGameUnits,
        8.485281f));
    assert(near(
        activationBoundary.rimRadiusGameUnits,
        8.485281f));
    assert(near(
        std::sqrt(
            activationBoundary.axialGameUnits *
                activationBoundary.axialGameUnits +
            activationBoundary.rimRadiusGameUnits *
                activationBoundary.rimRadiusGameUnits),
        12.0f));
    assert(!resolveActivationBoundaryDimensions(0.0f).valid);

    auto result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::OneHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ -5.0f, 0.0f, 0.0f },
        .leftAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.spatialPass);
    assert(result.selectedRegion == ActivationRegion::Left);
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
        .weaponFamily = resolveWeaponFamily(WeaponFamilyInput{
            .effectiveEquipSlotFormID =
                kBothHandsLeftOptionalEquipSlotFormID,
            .equippedWeaponPresent = true,
        }),
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ 0.0f, 0.0f, -5.0f },
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
    assert(result.selectedRegion == ActivationRegion::Down);
    assert(near(result.downDot, 1.0f));
    assert(near(result.sweptArcDot, 1.0f));

    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld =
            Vec3{ -diagonal * 10.0f, 0.0f, -diagonal * 10.0f },
        .leftAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.spatialPass);
    assert(result.selectedRegion == ActivationRegion::Arc);
    assert(near(result.sweptArcDot, 1.0f));

    // Regression: the former endpoint-cone union rejected every off-plane
    // approach at the LEFT/DOWN seam because both endpoint dots fell below
    // cos(45 degrees). The 90-degree sweep retains a 45-degree half-width.
    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ -7.0f, 1.41421356f, -7.0f },
        .leftAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.leftDot < kActivationConeMinimumDot);
    assert(result.downDot < kActivationConeMinimumDot);
    assert(result.directionPass);
    assert(result.spatialPass);
    assert(result.selectedRegion == ActivationRegion::Arc);
    assert(near(result.sweptArcDot, 0.989949f));

    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld =
            Vec3{ -5.0f, diagonal * 10.0f, -5.0f },
        .leftAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.directionPass);
    assert(result.spatialPass);
    assert(result.selectedRegion == ActivationRegion::Arc);
    assert(near(result.sweptArcDot, kActivationConeMinimumDot));

    constexpr float outsideArcPlaneDot = 0.49f;
    const float outsideArcReferenceDot = std::sqrt(
        1.0f -
        2.0f * outsideArcPlaneDot * outsideArcPlaneDot);
    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{
            -outsideArcPlaneDot * 10.0f,
            outsideArcReferenceDot * 10.0f,
            -outsideArcPlaneDot * 10.0f,
        },
        .leftAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.sweptArcDot < kActivationConeMinimumDot);
    assert(!result.directionPass);
    assert(!result.spatialPass);

    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ -8.0f, 5.9160798f, 1.0f },
        .leftAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.directionPass);
    assert(result.spatialPass);
    assert(result.selectedRegion == ActivationRegion::Left);
    assert(near(result.sweptArcDot, 0.8f));

    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ -5.0f, 0.0f, 0.0f },
        .leftAxisWorld = left,
        .downAxisWorld = left,
        .radialCapGameUnits = 12.0f,
    });
    assert(!result.directionPass);
    assert(!result.spatialPass);

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
        .rightFiringLeftSupportScope = false,
    });
    assert(result.radialPass);
    assert(result.directionPass);
    assert(!result.scopePass);
    assert(!result.spatialPass);

    IndicatorInput indicatorInput{
        .weaponFamily = WeaponFamily::OneHandGun,
        .authoredSeatWorld = Vec3{ 1.0f, 2.0f, 3.0f },
        .leftAxisWorld = Vec3{ -2.0f, 0.0f, 0.0f },
        .downAxisWorld = Vec3{ 0.0f, 0.0f, -4.0f },
        .activationStateValid = true,
        .activationSpatialPass = true,
        .interactionCandidateValid = true,
        .supportGripAllowed = true,
    };
    auto indicator = evaluateIndicator(indicatorInput);
    bool indicatorChecksPassed =
        indicator.visible &&
        near(indicator.markerWorld.x, -4.0f) &&
        near(indicator.markerWorld.y, 2.0f) &&
        near(indicator.markerWorld.z, 3.0f);
    assert(indicator.visible);
    assert(near(indicator.markerWorld.x, -4.0f));
    assert(near(indicator.markerWorld.y, 2.0f));
    assert(near(indicator.markerWorld.z, 3.0f));

    indicatorInput.weaponFamily = WeaponFamily::TwoHandGun;
    indicator = evaluateIndicator(indicatorInput);
    indicatorChecksPassed =
        indicatorChecksPassed &&
        indicator.visible &&
        near(indicator.markerWorld.x, 1.0f) &&
        near(indicator.markerWorld.y, 2.0f) &&
        near(indicator.markerWorld.z, -2.0f);
    assert(indicator.visible);
    assert(near(indicator.markerWorld.x, 1.0f));
    assert(near(indicator.markerWorld.y, 2.0f));
    assert(near(indicator.markerWorld.z, -2.0f));

    const auto indicatorHidden = [](const IndicatorInput& input) {
        return !evaluateIndicator(input).visible;
    };
    {
        auto input = indicatorInput;
        input.activationStateValid = false;
        indicatorChecksPassed = indicatorChecksPassed && indicatorHidden(input);
    }
    {
        auto input = indicatorInput;
        input.activationSpatialPass = false;
        indicatorChecksPassed = indicatorChecksPassed && indicatorHidden(input);
    }
    {
        auto input = indicatorInput;
        input.interactionCandidateValid = false;
        indicatorChecksPassed = indicatorChecksPassed && indicatorHidden(input);
    }
    {
        auto input = indicatorInput;
        input.supportGripAllowed = false;
        indicatorChecksPassed = indicatorChecksPassed && indicatorHidden(input);
    }
    {
        auto input = indicatorInput;
        input.providerPartAuthorityActive = true;
        indicatorChecksPassed = indicatorChecksPassed && indicatorHidden(input);
    }
    {
        auto input = indicatorInput;
        input.supportHandHoldingObject = true;
        indicatorChecksPassed = indicatorChecksPassed && indicatorHidden(input);
    }
    {
        auto input = indicatorInput;
        input.supportHandWeaponEngaged = true;
        indicatorChecksPassed = indicatorChecksPassed && indicatorHidden(input);
    }
    {
        auto input = indicatorInput;
        input.weaponFamily = WeaponFamily::Unsupported;
        indicatorChecksPassed = indicatorChecksPassed && indicatorHidden(input);
    }
    {
        auto input = indicatorInput;
        input.downAxisWorld = {};
        indicatorChecksPassed = indicatorChecksPassed && indicatorHidden(input);
    }
    assert(indicatorChecksPassed);
    if (!indicatorChecksPassed) {
        return 1;
    }

    return 0;
}
