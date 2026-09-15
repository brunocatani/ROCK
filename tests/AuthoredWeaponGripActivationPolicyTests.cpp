#include "physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h"

#include "physics-interaction/weapon/LooseWeaponAuthoredGrabPolicy.h"
#include <limits>
#include <initializer_list>
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
    const Vec3 right{ 1.0f, 0.0f, 0.0f };
    const Vec3 down{ 0.0f, 0.0f, -1.0f };
    constexpr float diagonal = 0.70710678118654752440f;

    static_assert(resolveHandTopology(false, true) ==
                  HandTopology::RightFiringLeftSupport);
    static_assert(resolveHandTopology(true, false) ==
                  HandTopology::LeftFiringRightSupport);
    static_assert(resolveHandTopology(false, false) ==
                  HandTopology::Invalid);
    static_assert(resolveHandTopology(true, true) ==
                  HandTopology::Invalid);
    static_assert(supportSideRegion(
                      HandTopology::RightFiringLeftSupport) ==
                  ActivationRegion::Left);
    static_assert(supportSideRegion(
                      HandTopology::LeftFiringRightSupport) ==
                  ActivationRegion::Right);
    static_assert([] {
        constexpr Vec3 source{ -2.0f, 3.0f, -4.0f };
        constexpr Vec3 mirrored = orientRightFiringAxisForTopology(
            source,
            HandTopology::LeftFiringRightSupport);
        return mirrored.x == 2.0f &&
               mirrored.y == source.y &&
               mirrored.z == source.z;
    }());

    const auto evaluateRightTopologyGate = [](DirectionGateInput input) {
        input.handTopology = HandTopology::RightFiringLeftSupport;
        return evaluateDirectionGate(input);
    };

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

    auto result = evaluateRightTopologyGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::OneHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ -5.0f, 0.0f, 0.0f },
        .supportSideAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.spatialPass);
    assert(result.selectedRegion == ActivationRegion::Left);
    assert(near(result.supportSideDot, 1.0f));

    result = evaluateRightTopologyGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::OneHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ 5.0f, 0.0f, 0.0f },
        .supportSideAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(!result.directionPass);
    assert(!result.spatialPass);

    result = evaluateRightTopologyGate(DirectionGateInput{
        .weaponFamily = resolveWeaponFamily(WeaponFamilyInput{
            .effectiveEquipSlotFormID =
                kBothHandsLeftOptionalEquipSlotFormID,
            .equippedWeaponPresent = true,
        }),
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ 0.0f, 0.0f, -5.0f },
        .supportSideAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(!result.directionPass);
    assert(!result.spatialPass);

    result = evaluateRightTopologyGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ 0.0f, 0.0f, -8.0f },
        .supportSideAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.spatialPass);
    assert(result.selectedRegion == ActivationRegion::Down);
    assert(near(result.downDot, 1.0f));
    assert(near(result.sweptArcDot, 1.0f));

    result = evaluateRightTopologyGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld =
            Vec3{ -diagonal * 10.0f, 0.0f, -diagonal * 10.0f },
        .supportSideAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.spatialPass);
    assert(result.selectedRegion == ActivationRegion::Arc);
    assert(near(result.sweptArcDot, 1.0f));

    // Regression: the former endpoint-cone union rejected every off-plane
    // approach at the LEFT/DOWN seam because both endpoint dots fell below
    // cos(45 degrees). The 90-degree sweep retains a 45-degree half-width.
    result = evaluateRightTopologyGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ -7.0f, 1.41421356f, -7.0f },
        .supportSideAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.supportSideDot < kActivationConeMinimumDot);
    assert(result.downDot < kActivationConeMinimumDot);
    assert(result.directionPass);
    assert(result.spatialPass);
    assert(result.selectedRegion == ActivationRegion::Arc);
    assert(near(result.sweptArcDot, 0.989949f));

    result = evaluateRightTopologyGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld =
            Vec3{ -5.0f, diagonal * 10.0f, -5.0f },
        .supportSideAxisWorld = left,
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
    result = evaluateRightTopologyGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{
            -outsideArcPlaneDot * 10.0f,
            outsideArcReferenceDot * 10.0f,
            -outsideArcPlaneDot * 10.0f,
        },
        .supportSideAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.sweptArcDot < kActivationConeMinimumDot);
    assert(!result.directionPass);
    assert(!result.spatialPass);

    result = evaluateRightTopologyGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ -8.0f, 5.9160798f, 1.0f },
        .supportSideAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.directionPass);
    assert(result.spatialPass);
    assert(result.selectedRegion == ActivationRegion::Left);
    assert(near(result.sweptArcDot, 0.8f));

    result = evaluateRightTopologyGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ -5.0f, 0.0f, 0.0f },
        .supportSideAxisWorld = left,
        .downAxisWorld = left,
        .radialCapGameUnits = 12.0f,
    });
    assert(!result.directionPass);
    assert(!result.spatialPass);

    result = evaluateRightTopologyGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::OneHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ -diagonal * 10.0f, diagonal * 10.0f, 0.0f },
        .supportSideAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.directionPass);
    assert(near(result.supportSideDot, kActivationConeMinimumDot));

    result = evaluateRightTopologyGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ -20.0f, 0.0f, 0.0f },
        .supportSideAxisWorld = left,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.directionPass);
    assert(!result.radialPass);
    assert(!result.spatialPass);

    result = evaluateRightTopologyGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::OneHandGun,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ 0.01f, 0.0f, 0.0f },
        .supportSideAxisWorld = left,
        .downAxisWorld = down,
        .lastStableDirectionWorld = left,
        .radialCapGameUnits = 12.0f,
        .lastStableDirectionValid = true,
    });
    assert(result.usedLastStableDirection);
    assert(result.directionPass);
    assert(result.spatialPass);

    // Ambidextrous regression: a mirrored authored pose uses an independent
    // RIGHT-facing activation cone. It must never reuse the native LEFT axis.
    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::OneHandGun,
        .handTopology = HandTopology::LeftFiringRightSupport,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ 5.0f, 0.0f, 0.0f },
        .supportSideAxisWorld = right,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.topologyPass);
    assert(result.directionPass);
    assert(result.spatialPass);
    assert(result.selectedRegion == ActivationRegion::Right);
    assert(near(result.supportSideDot, 1.0f));

    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::OneHandGun,
        .handTopology = HandTopology::LeftFiringRightSupport,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ -5.0f, 0.0f, 0.0f },
        .supportSideAxisWorld = right,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.topologyPass);
    assert(!result.directionPass);
    assert(!result.spatialPass);

    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .handTopology = HandTopology::LeftFiringRightSupport,
        .authoredSeatWorld = origin,
        .liveProbeWorld =
            Vec3{ diagonal * 10.0f, 0.0f, -diagonal * 10.0f },
        .supportSideAxisWorld = right,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.topologyPass);
    assert(result.directionPass);
    assert(result.spatialPass);
    assert(result.selectedRegion == ActivationRegion::Arc);
    assert(near(result.sweptArcDot, 1.0f));

    result = evaluateDirectionGate(DirectionGateInput{
        .weaponFamily = WeaponFamily::TwoHandGun,
        .handTopology = HandTopology::Invalid,
        .authoredSeatWorld = origin,
        .liveProbeWorld = Vec3{ 0.0f, 0.0f, -5.0f },
        .supportSideAxisWorld = right,
        .downAxisWorld = down,
        .radialCapGameUnits = 12.0f,
    });
    assert(result.radialPass);
    assert(!result.directionPass);
    assert(!result.topologyPass);
    assert(!result.spatialPass);

    IndicatorInput indicatorInput{
        .weaponFamily = WeaponFamily::OneHandGun,
        .authoredSeatWorld = Vec3{ 1.0f, 2.0f, 3.0f },
        .supportSideAxisWorld = Vec3{ -2.0f, 0.0f, 0.0f },
        .downAxisWorld = Vec3{ 0.0f, 0.0f, -4.0f },
        .activationStateValid = true,
        .activationSpatialPass = true,
        .supportGripAllowed = true,
    };
    auto indicator = evaluateIndicator(indicatorInput);
    bool indicatorChecksPassed =
        indicator.visible &&
        near(indicator.markerWorld.x, -2.0f) &&
        near(indicator.markerWorld.y, 2.0f) &&
        near(indicator.markerWorld.z, 3.0f);
    assert(indicator.visible);
    assert(near(indicator.markerWorld.x, -2.0f));
    assert(near(indicator.markerWorld.y, 2.0f));
    assert(near(indicator.markerWorld.z, 3.0f));

    indicatorInput.supportSideAxisWorld = Vec3{ 2.0f, 0.0f, 0.0f };
    indicator = evaluateIndicator(indicatorInput);
    indicatorChecksPassed =
        indicatorChecksPassed &&
        indicator.visible &&
        near(indicator.markerWorld.x, 4.0f) &&
        near(indicator.markerWorld.y, 2.0f) &&
        near(indicator.markerWorld.z, 3.0f);
    assert(indicator.visible);
    assert(near(indicator.markerWorld.x, 4.0f));
    assert(near(indicator.markerWorld.y, 2.0f));
    assert(near(indicator.markerWorld.z, 3.0f));

    constexpr float kThreeUnitDiagonalComponent = 2.12132034f;
    indicatorInput.weaponFamily = WeaponFamily::TwoHandGun;
    indicator = evaluateIndicator(indicatorInput);
    indicatorChecksPassed =
        indicatorChecksPassed &&
        indicator.visible &&
        near(
            indicator.markerWorld.x,
            1.0f + kThreeUnitDiagonalComponent) &&
        near(indicator.markerWorld.y, 2.0f) &&
        near(
            indicator.markerWorld.z,
            3.0f - kThreeUnitDiagonalComponent);
    assert(indicator.visible);
    assert(near(
        indicator.markerWorld.x,
        1.0f + kThreeUnitDiagonalComponent));
    assert(near(indicator.markerWorld.y, 2.0f));
    assert(near(
        indicator.markerWorld.z,
        3.0f - kThreeUnitDiagonalComponent));

    indicatorInput.supportSideAxisWorld = Vec3{ -2.0f, 0.0f, 0.0f };
    indicator = evaluateIndicator(indicatorInput);
    indicatorChecksPassed =
        indicatorChecksPassed &&
        indicator.visible &&
        near(
            indicator.markerWorld.x,
            1.0f - kThreeUnitDiagonalComponent) &&
        near(indicator.markerWorld.y, 2.0f) &&
        near(
            indicator.markerWorld.z,
            3.0f - kThreeUnitDiagonalComponent);
    assert(indicator.visible);
    assert(near(
        indicator.markerWorld.x,
        1.0f - kThreeUnitDiagonalComponent));
    assert(near(indicator.markerWorld.y, 2.0f));
    assert(near(
        indicator.markerWorld.z,
        3.0f - kThreeUnitDiagonalComponent));

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

    {
        namespace loose = rock::loose_weapon_authored_grab_policy;
        using Role = loose::Role;
        // Independent hand topologies on a never-equipped loose rifle:
        // each physical hand accepts its side and rejects the opposite side.
        bool looseGripsPass = true;
        for (bool supportIsLeft : {false, true}) {
            const auto topology = resolveHandTopology(!supportIsLeft, supportIsLeft);
            const auto side = orientRightFiringAxisForTopology({-1.0f, 0.0f, 0.0f}, topology);
            const auto looseDown = orientRightFiringAxisForTopology({0.0f, 0.0f, -1.0f}, topology);
            DirectionGateInput input{
                .weaponFamily = WeaponFamily::TwoHandGun,
                .handTopology = topology,
                .liveProbeWorld = side,
                .supportSideAxisWorld = side,
                .downAxisWorld = looseDown,
                .radialCapGameUnits = 3.0f,
            };
            looseGripsPass &= loose::select(false, 10.0f, evaluateDirectionGate(input).spatialPass, 1.0f) == Role::Support;
            input.liveProbeWorld = {-side.x, -side.y, -side.z};
            looseGripsPass &= loose::select(false, 10.0f, evaluateDirectionGate(input).spatialPass, 1.0f) == Role::None;
        }
        looseGripsPass &= loose::select(true, 0.5f, true, 1.0f) == Role::Firing;
        looseGripsPass &= loose::select(true, 1.5f, true, 1.0f) == Role::Support;
        looseGripsPass &= loose::select(true, 1.0f, true, 1.0f) == Role::Firing;
        looseGripsPass &= loose::select(false, 0.0f, false, 0.0f) == Role::None;
        looseGripsPass &= loose::select(true, std::numeric_limits<float>::quiet_NaN(), true, 1.0f) == Role::Support;
        looseGripsPass &= loose::select(false, 0.0f, true, std::numeric_limits<float>::infinity()) == Role::None;
        looseGripsPass &= loose::select(true, -1.0f, false, 0.0f) == Role::None;
        if (!looseGripsPass) {
            return 1;
        }
    }

    return 0;
}
