#pragma once

namespace rock::debug_visualization_policy
{
    struct Input
    {
        bool colliderMaster{ false };
        bool targetColliders{ false };
        bool colliderPhaseDiagnostics{ false };
        bool handColliders{ false };
        bool handBoneColliders{ false };
        bool bodyBoneColliders{ false };
        bool dynamicHandColliders{ false };
        bool weaponColliders{ false };
        bool grabbedWeaponPartCollider{ false };
        bool dynamicWeaponColliders{ false };

        bool handAxes{ false };
        bool grabPivots{ false };
        bool fingerSweptArc{ false };
        bool fingerSweptArcText{ false };
        bool fingerSweptArcLiveSkeleton{ false };
        bool palmVectors{ false };
        bool grabPockets{ false };
        bool rootFlattenedFingerSkeleton{ false };
        bool skeletonBones{ false };
        bool skeletonBoneAxes{ false };
        bool skeletonBoneLogging{ false };
        bool skeletonBoneTruncationLogging{ false };
        bool handBoneContacts{ false };
        bool grabAuthorityProxy{ false };
        bool videoSyncMarker{ false };
        bool weaponAuthority{ false };
        bool looseWeaponGripZones{ false };
        bool authoredGripActivationZones{ false };
        bool nativeScopeActivation{ false };
        bool worldOriginDiagnostics{ false };
    };

    struct State
    {
        bool colliderMaster{ false };
        bool targetColliders{ false };
        bool colliderPhaseDiagnostics{ false };
        bool handColliders{ false };
        bool handBoneColliders{ false };
        bool bodyBoneColliders{ false };
        bool dynamicHandColliders{ false };
        bool weaponColliders{ false };
        bool grabbedWeaponPartCollider{ false };
        bool dynamicWeaponColliders{ false };
        bool grabAuthorityProxyCollider{ false };

        bool handAxes{ false };
        bool grabPivots{ false };
        bool fingerSweptArc{ false };
        bool fingerSweptArcText{ false };
        bool fingerSweptArcLiveSkeleton{ false };
        bool palmVectors{ false };
        bool grabPockets{ false };
        bool rootFlattenedFingerSkeleton{ false };
        bool skeletonBones{ false };
        bool skeletonBoneAxes{ false };
        bool skeletonBoneLogging{ false };
        bool skeletonBoneTruncationLogging{ false };
        bool handBoneContacts{ false };
        bool grabAuthorityProxy{ false };
        bool videoSyncMarker{ false };
        bool weaponAuthority{ false };
        bool looseWeaponGripZones{ false };
        bool authoredGripActivationZones{ false };
        bool nativeScopeActivation{ false };
        bool worldOriginDiagnostics{ false };
    };

    [[nodiscard]] constexpr State resolve(const Input& input)
    {
        const bool colliders = input.colliderMaster;
        const bool sweptArc = input.fingerSweptArc;
        const bool skeleton = input.skeletonBones;

        return {
            .colliderMaster = colliders,
            .targetColliders = colliders && input.targetColliders,
            .colliderPhaseDiagnostics =
                colliders && input.colliderPhaseDiagnostics,
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

            .handAxes = input.handAxes,
            .grabPivots = input.grabPivots,
            .fingerSweptArc = sweptArc,
            .fingerSweptArcText = sweptArc && input.fingerSweptArcText,
            .fingerSweptArcLiveSkeleton =
                sweptArc && input.fingerSweptArcLiveSkeleton,
            .palmVectors = input.palmVectors,
            .grabPockets = input.grabPockets,
            .rootFlattenedFingerSkeleton =
                input.rootFlattenedFingerSkeleton,
            .skeletonBones = skeleton,
            .skeletonBoneAxes = skeleton && input.skeletonBoneAxes,
            .skeletonBoneLogging = skeleton && input.skeletonBoneLogging,
            .skeletonBoneTruncationLogging =
                skeleton && input.skeletonBoneTruncationLogging,
            .handBoneContacts = input.handBoneContacts,
            .grabAuthorityProxy = input.grabAuthorityProxy,
            .videoSyncMarker = input.videoSyncMarker,
            .weaponAuthority = input.weaponAuthority,
            .looseWeaponGripZones = input.looseWeaponGripZones,
            .authoredGripActivationZones =
                input.authoredGripActivationZones,
            .nativeScopeActivation = input.nativeScopeActivation,
            .worldOriginDiagnostics = input.worldOriginDiagnostics,
        };
    }
}
