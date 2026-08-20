#define ROCK_API_EXPORTS
// Build the DLL side of the public provider ABI.
#include "api/detail/ProviderApiEntryPoints.h"
#include "api/detail/ProviderFeatureBits.h"

namespace rock::provider::detail
{
    static_assert(sizeof(RockProviderApi) == 720);

    constexpr RockProviderApi ROCK_PROVIDER_API_FUNCTION_TABLE{
        .getVersion = &apiGetVersion,
        .getModVersion = &apiGetModVersion,
        .isProviderReady = &apiIsProviderReady,
        .registerFrameCallback = &apiRegisterFrameCallback,
        .unregisterFrameCallback = &apiUnregisterFrameCallback,
        .getFrameSnapshot = &apiGetFrameSnapshot,
        .queryWeaponContactAtPoint = &apiQueryWeaponContactAtPoint,
        .clearExternalBodies = &apiClearExternalBodies,
        .setOffhandInteractionReservation = &apiSetOffhandInteractionReservation,
        .registerExternalBodiesV1 = &apiRegisterExternalBodiesV1,
        .getWeaponEvidenceDetailCountV1 = &apiGetWeaponEvidenceDetailCountV1,
        .copyWeaponEvidenceDetailsV1 = &apiCopyWeaponEvidenceDetailsV1,
        .getWeaponEvidenceDetailPointCountV1 = &apiGetWeaponEvidenceDetailPointCountV1,
        .copyWeaponEvidenceDetailPointsV1 = &apiCopyWeaponEvidenceDetailPointsV1,
        .getBodyContactSnapshotV1 = &apiGetBodyContactSnapshotV1,
        .getPrimaryHandV1 = &apiGetPrimaryHandV1,
        .getOffhandHandV1 = &apiGetOffhandHandV1,
        .getHandFrameV1 = &apiGetHandFrameV1,
        .registerConsumerV1 = &apiRegisterConsumerV1,
        .unregisterConsumerV1 = &apiUnregisterConsumerV1,
        .getGrantedCapabilitiesV1 = &apiGetGrantedCapabilitiesV1,
        .getProviderLimitsV1 = &apiGetProviderLimitsV1,
        .getExternalContactSnapshotForOwnerV1 = &apiGetExternalContactSnapshotForOwnerV1,
        .requestForceGrabV1 = &apiRequestForceGrabV1,
        .getInteractionCommandResultV1 = &apiGetInteractionCommandResultV1,
        .requestForceReleaseV1 = &apiRequestForceReleaseV1,
        .requestThrownDropV1 = &apiRequestThrownDropV1,
        .setHandInputSuppressionV1 = &apiSetHandInputSuppressionV1,
        .clearHandInputSuppressionV1 = &apiClearHandInputSuppressionV1,
        .setWeaponPartTargetsV1 = &apiSetWeaponPartTargetsV1,
        .clearWeaponPartTargetsV1 = &apiClearWeaponPartTargetsV1,
        .setWeaponPartDriveTargetsV1 = &apiSetWeaponPartDriveTargetsV1,
        .clearWeaponPartDriveTargetsV1 = &apiClearWeaponPartDriveTargetsV1,
        .queryEquippedWeaponClassificationV1 = &apiQueryEquippedWeaponClassificationV1,
        .getWeaponPartGripStateV1 = &apiGetWeaponPartGripStateV1,
        .getRawWandButtonStateV1 = &apiGetRawWandButtonStateV1,
        .isNativePipboyInputSuppressedV1 = &apiIsNativePipboyInputSuppressedV1,
        .getWeaponEmitterCountV1 = &apiGetWeaponEmitterCountV1,
        .copyWeaponEmittersV1 = &apiCopyWeaponEmittersV1,
        .setNativeAnimationAuthorityV1 = &apiSetNativeAnimationAuthorityV1,
        .clearNativeAnimationAuthorityV1 = &apiClearNativeAnimationAuthorityV1,
        .getNativeAnimationAuthorityStateV1 = &apiGetNativeAnimationAuthorityStateV1,
        .registerAnimationPhaseCallbackV1 = &apiRegisterAnimationPhaseCallbackV1,
        .unregisterAnimationPhaseCallbackV1 = &apiUnregisterAnimationPhaseCallbackV1,
        .getEquippedWeaponGripStateV1 = &apiGetEquippedWeaponGripStateV1,
        .setHandVisualAuthorityV1 = &apiSetHandVisualAuthorityV1,
        .clearHandVisualAuthorityV1 = &apiClearHandVisualAuthorityV1,
        .publishNativeAnimationRuntimeV1 = &apiPublishNativeAnimationRuntimeV1,
        .setEquippedWeaponHandlingAuthorityV1 = &apiSetEquippedWeaponHandlingAuthorityV1,
        .clearEquippedWeaponHandlingAuthorityV1 = &apiClearEquippedWeaponHandlingAuthorityV1,
        .getEquippedWeaponHandlingStateV1 = &apiGetEquippedWeaponHandlingStateV1,
        .publishDebugOverlayV1 = &apiPublishDebugOverlayV1,
        .clearDebugOverlayV1 = &apiClearDebugOverlayV1,
        .getPresentedHandFrameV1 = &apiGetPresentedHandFrameV1,
        .getProviderLimitsExtV1 = &apiGetProviderLimitsExtV1,
        .getPublicStructureSizeV1 = &apiGetPublicStructureSizeV1,
        .registerFrameCallbackForOwnerV1 =
            &apiRegisterFrameCallbackForOwnerV1,
        .unregisterFrameCallbackForOwnerV1 =
            &apiUnregisterFrameCallbackForOwnerV1,
        .getHandInteractionStateV1 = &apiGetHandInteractionStateV1,
        .copyProviderEventsSinceV1 = &apiCopyProviderEventsSinceV1,
        .getEquippedWeaponStateV1 = &apiGetEquippedWeaponStateV1,
        .registerExternalBodiesForScopeV1 =
            &apiRegisterExternalBodiesForScopeV1,
        .clearExternalBodiesForScopeV1 =
            &apiClearExternalBodiesForScopeV1,
        .copyExternalContactsSinceV1 = &apiCopyExternalContactsSinceV1,
        .queryWeaponPartTargetResolutionV1 =
            &apiQueryWeaponPartTargetResolutionV1,
        .copyWeaponPartPoseSnapshotV1 =
            &apiCopyWeaponPartPoseSnapshotV1,
        .copyWeaponPartDriveApplicationResultsV1 =
            &apiCopyWeaponPartDriveApplicationResultsV1,
        .getScopeSightStateV1 = &apiGetScopeSightStateV1,
        .getWeaponCompositionStateV1 = &apiGetWeaponCompositionStateV1,
        .copyWeaponCompositionEntriesV1 =
            &apiCopyWeaponCompositionEntriesV1,
        .getSelectedAuthoredGripPoseV1 = &apiGetSelectedAuthoredGripPoseV1,
        .getPresentedHandPoseV1 = &apiGetPresentedHandPoseV1,
        .copySemanticHandContactsV1 = &apiCopySemanticHandContactsV1,
        .copyPlayerColliderDescriptorsV1 =
            &apiCopyPlayerColliderDescriptorsV1,
        .getHandCollisionAvailabilityV1 =
            &apiGetHandCollisionAvailabilityV1,
        .cancelInteractionCommandV1 = &apiCancelInteractionCommandV1,
        .getHandInputSuppressionStateV1 =
            &apiGetHandInputSuppressionStateV1,
        .acquireOffhandReservationV1 = &apiAcquireOffhandReservationV1,
        .renewOffhandReservationV1 = &apiRenewOffhandReservationV1,
        .releaseOffhandReservationV1 = &apiReleaseOffhandReservationV1,
        .getOffhandReservationStateV1 = &apiGetOffhandReservationStateV1,
        .clearNativeAnimationRuntimeV1 =
            &apiClearNativeAnimationRuntimeV1,
        .setTouchGrabTargetsForScopeV1 =
            &apiSetTouchGrabTargetsForScopeV1,
        .clearTouchGrabTargetsForScopeV1 =
            &apiClearTouchGrabTargetsForScopeV1,
        .copyTouchGrabStatesForScopeV1 =
            &apiCopyTouchGrabStatesForScopeV1,
        .requestTouchGrabYieldV1 =
            &apiRequestTouchGrabYieldV1,
        .requestEquippedWeaponHandV1 =
            &apiRequestEquippedWeaponHandV1,
        .queryWorldRaycastV1 =
            &apiQueryWorldRaycastV1,
        .setColliderVisualizationOverrideV1 =
            &apiSetColliderVisualizationOverrideV1,
        .clearColliderVisualizationOverrideV1 =
            &apiClearColliderVisualizationOverrideV1,
    };

    constexpr RockProviderApiDescriptorV1 ROCK_PROVIDER_API_DESCRIPTOR{
        .size = sizeof(RockProviderApiDescriptorV1),
        .apiVersion = ROCK_PROVIDER_API_VERSION,
        .tableByteSize = sizeof(RockProviderApi),
        .featureBits = kProviderFeatureBitsV1,
        .featureBits2 = kProviderFeatureBits2V1,
        .table = &ROCK_PROVIDER_API_FUNCTION_TABLE,
    };
}
namespace rock::provider
{
    using namespace detail;

    ROCK_PROVIDER_API const RockProviderApi* ROCK_PROVIDER_CALL ROCKAPI_GetProviderApi()
    {
        return &ROCK_PROVIDER_API_FUNCTION_TABLE;
    }

    ROCK_PROVIDER_API const RockProviderApiDescriptorV1* ROCK_PROVIDER_CALL
    ROCKAPI_GetDescriptorV1()
    {
        return &ROCK_PROVIDER_API_DESCRIPTOR;
    }
}
