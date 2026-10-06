#pragma once

namespace rock::debug_visualization_policy
{
    struct Input
    {
        bool colliderMaster{ false };
        bool targetColliders{ false };
        bool handColliders{ false };
        bool handBoneColliders{ false };
        bool bodyBoneColliders{ false };
        bool dynamicHandColliders{ false };
        bool weaponColliders{ false };
        bool grabbedWeaponPartCollider{ false };
        bool dynamicWeaponColliders{ false };

        bool handAxes{ false };
        bool grabPivots{ false };
        bool fingerProbes{ false };
        bool palmVectors{ false };
        bool grabPockets{ false };
        bool rootFlattenedFingerSkeleton{ false };
        bool skeletonBones{ false };
        bool skeletonBoneAxes{ false };
        bool skeletonBoneLogging{ false };
        bool skeletonBoneTruncationLogging{ false };
        bool grabPocketNormal{ false };
        bool grabForceTorque{ false };
        bool grabForceTorqueText{ false };
        bool grabPivotSourceCollider{ false };
        bool grabPivotSourceEvidence{ false };
        bool grabSupportFrame{ false };
        bool handBoneContacts{ false };
        bool grabAuthorityProxy{ false };
        bool weaponAuthority{ false };
        bool looseWeaponGripZones{ false };
        bool authoredGripActivationZones{ false };
        bool nativeScopeActivation{ false };
    };

    struct State
    {
        bool colliderMaster{ false };
        bool targetColliders{ false };
        bool handColliders{ false };
        bool handBoneColliders{ false };
        bool bodyBoneColliders{ false };
        bool dynamicHandColliders{ false };
        bool weaponColliders{ false };
        bool grabbedWeaponPartCollider{ false };
        bool dynamicWeaponColliders{ false };
        bool grabAuthorityProxyCollider{ false };
        bool grabPivotSourceCollider{ false };

        bool handAxes{ false };
        bool grabPivots{ false };
        bool fingerProbes{ false };
        bool palmVectors{ false };
        bool grabPockets{ false };
        bool rootFlattenedFingerSkeleton{ false };
        bool skeletonBones{ false };
        bool skeletonBoneAxes{ false };
        bool skeletonBoneLogging{ false };
        bool skeletonBoneTruncationLogging{ false };
        bool grabPocketNormal{ false };
        bool grabForceTorque{ false };
        bool grabForceTorqueText{ false };
        bool grabPivotSourceEvidence{ false };
        bool grabSupportFrame{ false };
        bool handBoneContacts{ false };
        bool grabAuthorityProxy{ false };
        bool weaponAuthority{ false };
        bool looseWeaponGripZones{ false };
        bool authoredGripActivationZones{ false };
        bool nativeScopeActivation{ false };
    };

    [[nodiscard]] constexpr State resolve(const Input& input)
    {
        const bool colliders = input.colliderMaster;
        const bool forceTorque = input.grabForceTorque;
        const bool skeleton = input.skeletonBones;

        return {
            .colliderMaster = colliders,
            .targetColliders = colliders && input.targetColliders,
            .handColliders = colliders && input.handColliders,
            .handBoneColliders = colliders && input.handBoneColliders,
            .bodyBoneColliders = colliders && input.bodyBoneColliders,
            .dynamicHandColliders = colliders && input.dynamicHandColliders,
            .weaponColliders = colliders && input.weaponColliders,
            .grabbedWeaponPartCollider =
                colliders && input.grabbedWeaponPartCollider,
            .dynamicWeaponColliders =
                colliders && input.dynamicWeaponColliders,
            .grabAuthorityProxyCollider =
                colliders && input.grabAuthorityProxy,
            .grabPivotSourceCollider =
                colliders && forceTorque && input.grabPivotSourceCollider,

            .handAxes = input.handAxes,
            .grabPivots = input.grabPivots,
            .fingerProbes = input.fingerProbes,
            .palmVectors = input.palmVectors,
            .grabPockets = input.grabPockets,
            .rootFlattenedFingerSkeleton =
                input.rootFlattenedFingerSkeleton,
            .skeletonBones = skeleton,
            .skeletonBoneAxes = skeleton && input.skeletonBoneAxes,
            .skeletonBoneLogging = skeleton && input.skeletonBoneLogging,
            .skeletonBoneTruncationLogging =
                skeleton && input.skeletonBoneTruncationLogging,
            .grabPocketNormal = input.grabPocketNormal,
            .grabForceTorque = forceTorque,
            .grabForceTorqueText =
                forceTorque && input.grabForceTorqueText,
            .grabPivotSourceEvidence =
                forceTorque && input.grabPivotSourceEvidence,
            .grabSupportFrame = input.grabSupportFrame,
            .handBoneContacts = input.handBoneContacts,
            .grabAuthorityProxy = input.grabAuthorityProxy,
            .weaponAuthority = input.weaponAuthority,
            .looseWeaponGripZones = input.looseWeaponGripZones,
            .authoredGripActivationZones =
                input.authoredGripActivationZones,
            .nativeScopeActivation = input.nativeScopeActivation,
        };
    }
}
