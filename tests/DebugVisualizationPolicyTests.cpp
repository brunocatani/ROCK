#include "physics-interaction/debug/DebugVisualizationPolicy.h"
#include "physics-interaction/weapon/GripZoneIndicatorPolicy.h"

using namespace rock::debug_visualization_policy;

namespace
{
    constexpr bool colliderChildrenDisabled(const State& state)
    {
        return !state.targetColliders &&
               !state.colliderPhaseDiagnostics &&
               !state.handColliders &&
               !state.handBoneColliders &&
               !state.bodyBoneColliders &&
               !state.dynamicHandColliders &&
               !state.weaponColliders &&
               !state.grabbedWeaponPartCollider &&
               !state.dynamicWeaponColliders &&
               !state.grabAuthorityProxyCollider &&
               !state.grabPivotSourceCollider;
    }

    constexpr auto disabledColliderMaster = resolve({
        .targetColliders = true,
        .colliderPhaseDiagnostics = true,
        .handColliders = true,
        .handBoneColliders = true,
        .bodyBoneColliders = true,
        .dynamicHandColliders = true,
        .weaponColliders = true,
        .grabbedWeaponPartCollider = true,
        .dynamicWeaponColliders = true,
        .grabForceTorque = true,
        .grabPivotSourceCollider = true,
        .grabAuthorityProxy = true,
    });
    static_assert(colliderChildrenDisabled(disabledColliderMaster));
    static_assert(disabledColliderMaster.grabForceTorque);
    static_assert(disabledColliderMaster.grabAuthorityProxy);

    constexpr auto emptyEnabledColliderMaster = resolve({
        .colliderMaster = true,
    });
    static_assert(emptyEnabledColliderMaster.colliderMaster);
    static_assert(colliderChildrenDisabled(emptyEnabledColliderMaster));

    constexpr auto selectedColliderChildren = resolve({
        .colliderMaster = true,
        .targetColliders = true,
        .colliderPhaseDiagnostics = true,
        .handColliders = true,
        .handBoneColliders = true,
        .bodyBoneColliders = true,
        .dynamicHandColliders = true,
        .weaponColliders = true,
        .grabbedWeaponPartCollider = true,
        .dynamicWeaponColliders = true,
        .grabForceTorque = true,
        .grabPivotSourceCollider = true,
        .grabAuthorityProxy = true,
    });
    static_assert(selectedColliderChildren.targetColliders);
    static_assert(selectedColliderChildren.colliderPhaseDiagnostics);
    static_assert(selectedColliderChildren.handColliders);
    static_assert(selectedColliderChildren.handBoneColliders);
    static_assert(selectedColliderChildren.bodyBoneColliders);
    static_assert(selectedColliderChildren.dynamicHandColliders);
    static_assert(selectedColliderChildren.weaponColliders);
    static_assert(selectedColliderChildren.grabbedWeaponPartCollider);
    static_assert(selectedColliderChildren.dynamicWeaponColliders);
    static_assert(selectedColliderChildren.grabAuthorityProxyCollider);
    static_assert(selectedColliderChildren.grabPivotSourceCollider);

    constexpr auto disabledSuiteMasters = resolve({
        .fingerSweptArcText = true,
        .fingerSweptArcLiveSkeleton = true,
        .skeletonBoneAxes = true,
        .skeletonBoneLogging = true,
        .skeletonBoneTruncationLogging = true,
        .grabForceTorqueText = true,
        .grabPivotSourceEvidence = true,
        .grabTransformTelemetryAxes = true,
        .grabTransformTelemetryText = true,
    });
    static_assert(!disabledSuiteMasters.fingerSweptArcText);
    static_assert(!disabledSuiteMasters.fingerSweptArcLiveSkeleton);
    static_assert(!disabledSuiteMasters.skeletonBoneAxes);
    static_assert(!disabledSuiteMasters.skeletonBoneLogging);
    static_assert(!disabledSuiteMasters.skeletonBoneTruncationLogging);
    static_assert(!disabledSuiteMasters.grabForceTorqueText);
    static_assert(!disabledSuiteMasters.grabPivotSourceEvidence);
    static_assert(!disabledSuiteMasters.grabTransformTelemetryAxes);
    static_assert(!disabledSuiteMasters.grabTransformTelemetryText);

    constexpr auto unrelatedGenericVisuals = resolve({
        .handAxes = true,
        .grabPivots = true,
        .grabContactPatch = true,
    });
    static_assert(unrelatedGenericVisuals.handAxes);
    static_assert(!resolve({}).weaponAimRays);
    static_assert(resolve({ .weaponAimRays = true }).weaponAimRays);
    static_assert(colliderChildrenDisabled(resolve({ .weaponAimRays = true })));
    static_assert(unrelatedGenericVisuals.grabPivots);
    static_assert(unrelatedGenericVisuals.grabContactPatch);
    static_assert(!unrelatedGenericVisuals.weaponAuthority);
    static_assert(!unrelatedGenericVisuals.looseWeaponGripZones);
    static_assert(!unrelatedGenericVisuals.authoredGripActivationZones);
    static_assert(!unrelatedGenericVisuals.grabPocketNormal);

    static_assert(
        rock::grip_zone_indicator_policy::isCurrentRenderFrame(42, 42));
    static_assert(
        !rock::grip_zone_indicator_policy::isCurrentRenderFrame(41, 42));
    static_assert(
        !rock::grip_zone_indicator_policy::isCurrentRenderFrame(0, 0));
    static_assert(
        rock::grip_zone_indicator_policy::kDefaultDiameterGameUnits >=
        rock::grip_zone_indicator_policy::kMinimumDiameterGameUnits);
    static_assert(
        rock::grip_zone_indicator_policy::kDefaultDiameterGameUnits <=
        rock::grip_zone_indicator_policy::kMaximumDiameterGameUnits);
}

int main()
{
    return 0;
}
