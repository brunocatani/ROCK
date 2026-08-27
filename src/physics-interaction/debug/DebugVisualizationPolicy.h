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
        bool dynamicWeaponColliders{ false };

        bool handAxes{ false };
        bool grabPivots{ false };
        bool fingerProbes{ false };
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
        bool grabPocketNormal{ false };
        bool grabContactPatch{ false };
        bool grabForceTorque{ false };
        bool grabForceTorqueText{ false };
        bool grabPivotSourceCollider{ false };
        bool grabPivotSourceEvidence{ false };
        bool grabSupportFrame{ false };
        bool handBoneContacts{ false };
        bool grabAuthorityProxy{ false };
        bool grabTransformTelemetry{ false };
        bool grabTransformTelemetryAxes{ false };
        bool grabTransformTelemetryText{ false };
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
        bool dynamicWeaponColliders{ false };
        bool grabAuthorityProxyCollider{ false };
        bool grabPivotSourceCollider{ false };

        bool handAxes{ false };
        bool grabPivots{ false };
        bool fingerProbes{ false };
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
        bool grabPocketNormal{ false };
        bool grabContactPatch{ false };
        bool grabForceTorque{ false };
        bool grabForceTorqueText{ false };
        bool grabPivotSourceEvidence{ false };
        bool grabSupportFrame{ false };
        bool handBoneContacts{ false };
        bool grabAuthorityProxy{ false };
        bool grabTransformTelemetry{ false };
        bool grabTransformTelemetryAxes{ false };
        bool grabTransformTelemetryText{ false };
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
        const bool forceTorque = input.grabForceTorque;
        const bool sweptArc = input.fingerSweptArc;
        const bool skeleton = input.skeletonBones;
        const bool telemetry = input.grabTransformTelemetry;

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
            .dynamicWeaponColliders =
                colliders && input.dynamicWeaponColliders,
            .grabAuthorityProxyCollider =
                colliders && input.grabAuthorityProxy,
            .grabPivotSourceCollider =
                colliders && forceTorque && input.grabPivotSourceCollider,

            .handAxes = input.handAxes,
            .grabPivots = input.grabPivots,
            .fingerProbes = input.fingerProbes,
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
            .grabPocketNormal = input.grabPocketNormal,
            .grabContactPatch = input.grabContactPatch,
            .grabForceTorque = forceTorque,
            .grabForceTorqueText =
                forceTorque && input.grabForceTorqueText,
            .grabPivotSourceEvidence =
                forceTorque && input.grabPivotSourceEvidence,
            .grabSupportFrame = input.grabSupportFrame,
            .handBoneContacts = input.handBoneContacts,
            .grabAuthorityProxy = input.grabAuthorityProxy,
            .grabTransformTelemetry = telemetry,
            .grabTransformTelemetryAxes =
                telemetry && input.grabTransformTelemetryAxes,
            .grabTransformTelemetryText =
                telemetry && input.grabTransformTelemetryText,
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
