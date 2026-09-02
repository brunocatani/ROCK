#include "physics-interaction/debug/DebugVisualizationPolicy.h"

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
               !state.grabAuthorityProxyCollider;
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
        .grabAuthorityProxy = true,
    });
    static_assert(colliderChildrenDisabled(disabledColliderMaster));
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

    constexpr auto disabledSuiteMasters = resolve({
        .fingerSweptArcText = true,
        .fingerSweptArcLiveSkeleton = true,
        .skeletonBoneAxes = true,
        .skeletonBoneLogging = true,
        .skeletonBoneTruncationLogging = true,
    });
    static_assert(!disabledSuiteMasters.fingerSweptArcText);
    static_assert(!disabledSuiteMasters.fingerSweptArcLiveSkeleton);
    static_assert(!disabledSuiteMasters.skeletonBoneAxes);
    static_assert(!disabledSuiteMasters.skeletonBoneLogging);
    static_assert(!disabledSuiteMasters.skeletonBoneTruncationLogging);

    constexpr auto unrelatedGenericVisuals = resolve({
        .handAxes = true,
        .grabPivots = true,
    });
    static_assert(unrelatedGenericVisuals.handAxes);
    static_assert(unrelatedGenericVisuals.grabPivots);
    static_assert(!unrelatedGenericVisuals.weaponAuthority);
    static_assert(!unrelatedGenericVisuals.looseWeaponGripZones);
    static_assert(!unrelatedGenericVisuals.authoredGripActivationZones);
}

int main()
{
    return 0;
}
