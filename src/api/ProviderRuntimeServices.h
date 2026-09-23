#pragma once
#include <ROCK/WeaponParts.h>
#include <ROCK/WeaponV1_1.h>
#include "WeaponSourceCatalog.h"
#include "ProviderRuntimeTypes.h"
#include "OwnerBindingPolicy.h"

namespace rock::provider::runtime {
    api::Status captureInventoryWeapon(std::uint64_t owner, std::uint32_t form, std::uint32_t stack,
        api::weapon::v1_1::InventoryWeapon& output);
    api::Status requestInventoryEquip(std::uint64_t owner, const api::weapon::v1_1::EquipRequest& request, std::uint64_t& command);
    api::Status getInventoryEquipResult(std::uint64_t owner, std::uint64_t command, api::weapon::v1_1::EquipResult& output);
    api::Status cancelInventoryEquip(std::uint64_t owner, std::uint64_t command);
    std::uint64_t currentGameFrameIndex(); // Atomic game clock, including pre-publication lifecycle events.
    const char* ROCK_PROVIDER_CALL apiGetModVersion();
    bool ROCK_PROVIDER_CALL apiIsProviderReady();
    bool ROCK_PROVIDER_CALL apiGetFrameSnapshot(RockProviderFrameSnapshot* outSnapshot);
    bool ROCK_PROVIDER_CALL apiQueryWeaponContactAtPoint(const RockProviderWeaponContactQuery* query, RockProviderWeaponContactResult* outResult);
    void ROCK_PROVIDER_CALL apiClearExternalBodies(std::uint64_t ownerToken);
    std::uint32_t ROCK_PROVIDER_CALL apiGetWeaponEvidenceDetailCountV1();
    std::uint32_t ROCK_PROVIDER_CALL apiCopyWeaponEvidenceDetailsV1(RockProviderWeaponEvidenceDetailV1* outDetails, std::uint32_t maxDetails);
    std::uint32_t ROCK_PROVIDER_CALL apiGetWeaponEvidenceDetailPointCountV1(std::uint32_t bodyId);
    std::uint32_t ROCK_PROVIDER_CALL apiCopyWeaponEvidenceDetailPointsV1(std::uint32_t bodyId, RockProviderPoint3* outPoints, std::uint32_t maxPoints);
    std::uint32_t ROCK_PROVIDER_CALL apiGetBodyContactSnapshotV1(RockProviderBodyContactV1* outContacts, std::uint32_t maxContacts);
    RockProviderHand ROCK_PROVIDER_CALL apiGetPrimaryHandV1();
    RockProviderHand ROCK_PROVIDER_CALL apiGetOffhandHandV1();
    bool ROCK_PROVIDER_CALL apiGetHandFrameV1(RockProviderHand hand, RockProviderHandFrameV1* outFrame);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterConsumerV1(const RockProviderConsumerRegistrationV1* registration, RockProviderConsumerHandleV1* outHandle);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiUnregisterConsumerV1(std::uint64_t ownerToken);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestForceGrabV1(std::uint64_t ownerToken, const RockProviderForceGrabRequestV1* request, std::uint64_t* outCommandId);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetInteractionCommandResultV1(std::uint64_t ownerToken, std::uint64_t commandId, RockProviderInteractionCommandResultV1* outResult);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestForceReleaseV1(std::uint64_t ownerToken, const RockProviderForceReleaseRequestV1* request, std::uint64_t* outCommandId);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestThrownDropV1(std::uint64_t ownerToken, const RockProviderThrownDropRequestV1* request, std::uint64_t* outCommandId);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetHandInputSuppressionV1(std::uint64_t ownerToken, const RockProviderHandInputSuppressionRequestV1* request);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearHandInputSuppressionV1(std::uint64_t ownerToken, RockProviderHand hand);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetWeaponPartTargetsV1(std::uint64_t ownerToken, const RockProviderWeaponPartTargetV1* targets, std::uint32_t targetCount);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearWeaponPartTargetsV1(std::uint64_t ownerToken);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetWeaponPartDriveTargetsV1(std::uint64_t ownerToken, const RockProviderWeaponPartDriveTargetV1* targets, std::uint32_t targetCount);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearWeaponPartDriveTargetsV1(std::uint64_t ownerToken);
    bool ROCK_PROVIDER_CALL apiQueryEquippedWeaponClassificationV1(RockProviderWeaponClassificationV1* outResult);
    bool ROCK_PROVIDER_CALL apiGetWeaponPartGripStateV1(RockProviderHand hand, api::weaponparts::WeaponPartGripStateV1* outState);
    bool ROCK_PROVIDER_CALL apiGetRawWandButtonStateV1(RockProviderHand hand, std::uint32_t buttonId, RockProviderRawWandButtonStateV1* outState);
    bool ROCK_PROVIDER_CALL apiIsNativePipboyInputSuppressedV1();
    std::uint32_t ROCK_PROVIDER_CALL apiGetWeaponEmitterCountV1();
    std::uint32_t ROCK_PROVIDER_CALL apiCopyWeaponEmittersV1(RockProviderWeaponEmitterV1* outEmitters, std::uint32_t maxEmitters);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetNativeAnimationAuthorityV1(std::uint64_t ownerToken, const RockProviderNativeAnimationAuthorityRequestV1* request);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearNativeAnimationAuthorityV1(std::uint64_t ownerToken);
    bool ROCK_PROVIDER_CALL apiGetNativeAnimationAuthorityStateV1(RockProviderNativeAnimationAuthorityStateV1* outState);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterAnimationPhaseCallbackV1(std::uint64_t ownerToken, RockProviderAnimationPhaseCallbackV1 callback, void* userData, std::uint64_t* outCallbackToken);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiUnregisterAnimationPhaseCallbackV1(std::uint64_t ownerToken, std::uint64_t callbackToken);
    bool ROCK_PROVIDER_CALL apiGetEquippedWeaponGripStateV1(std::uint64_t ownerToken, RockProviderEquippedWeaponGripStateV1* outState);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetHandVisualAuthorityV1(std::uint64_t ownerToken, const RockProviderHandVisualAuthorityRequestV1* request);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearHandVisualAuthorityV1(std::uint64_t ownerToken, RockProviderHand hand);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiPublishNativeAnimationRuntimeV1(std::uint64_t ownerToken, const RockProviderNativeAnimationRuntimePublicationV1* publication);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetEquippedWeaponHandlingAuthorityV1(std::uint64_t ownerToken, const RockProviderEquippedWeaponHandlingRequestV1* request);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearEquippedWeaponHandlingAuthorityV1(std::uint64_t ownerToken);
    bool ROCK_PROVIDER_CALL apiGetEquippedWeaponHandlingStateV1(RockProviderEquippedWeaponHandlingStateV1* outState);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiPublishDebugOverlayV1(std::uint64_t ownerToken, const RockProviderDebugOverlayPublicationV1* publication);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearDebugOverlayV1(std::uint64_t ownerToken);
    bool ROCK_PROVIDER_CALL apiGetPresentedHandFrameV1(RockProviderHand hand, RockProviderHandFrameV1* outFrame);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterFrameCallbackForOwnerV1(std::uint64_t ownerToken, RockProviderFrameCallback callback, void* userData, std::uint64_t* outCallbackToken);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiUnregisterFrameCallbackForOwnerV1(std::uint64_t ownerToken, std::uint64_t callbackToken);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetHandInteractionStateV1(std::uint64_t ownerToken, RockProviderHand hand, RockProviderHandInteractionStateV1* outState);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetEquippedWeaponStateV1(std::uint64_t ownerToken, RockProviderEquippedWeaponStateV1* outState);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterExternalBodiesForScopeV1(std::uint64_t ownerToken, std::uint64_t scopeToken, const RockProviderExternalBodyRegistration* bodies, std::uint32_t bodyCount);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearExternalBodiesForScopeV1(std::uint64_t ownerToken, std::uint64_t scopeToken);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyExternalContactsSinceV1(std::uint64_t ownerToken, std::uint64_t scopeToken, std::uint64_t afterSequence, RockProviderExternalContactRecordV1* outContacts, std::uint32_t maxContacts, RockProviderExternalContactStreamStateV1* outStreamState);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiQueryWeaponPartTargetResolutionV1(std::uint64_t ownerToken, const RockProviderWeaponPartResolutionQueryV1* query, RockProviderWeaponPartResolutionResultV1* outResolution);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyWeaponPartPoseSnapshotV1(std::uint64_t ownerToken, RockProviderWeaponPartPoseV1* outParts, std::uint32_t maxParts, std::uint32_t* outPartCount);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyWeaponPartDriveApplicationResultsV1(std::uint64_t ownerToken, RockProviderWeaponPartDriveApplicationResultV1* outResults, std::uint32_t maxResults, std::uint32_t* outResultCount);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetScopeSightStateV1(std::uint64_t ownerToken, RockProviderScopeSightStateV1* outState);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetWeaponCompositionStateV1(std::uint64_t ownerToken, RockProviderWeaponCompositionStateV1* outState);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyWeaponCompositionEntriesV1(std::uint64_t ownerToken, RockProviderWeaponCompositionEntryV1* outEntries, std::uint32_t maxEntries, std::uint32_t* outEntryCount);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetSelectedAuthoredGripPoseV1(std::uint64_t ownerToken, RockProviderAuthoredGripPoseV1* outPose);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetPresentedHandPoseV1(std::uint64_t ownerToken, RockProviderHand hand, RockProviderPresentedHandPoseV1* outPose);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopySemanticHandContactsV1(std::uint64_t ownerToken, RockProviderHand hand, std::uint32_t maxFramesSinceContact, RockProviderSemanticHandContactV1* outContacts, std::uint32_t maxContacts, std::uint32_t* outContactCount);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyPlayerColliderDescriptorsV1(std::uint64_t ownerToken, RockProviderPlayerColliderDescriptorV1* outDescriptors, std::uint32_t maxDescriptors, std::uint32_t* outDescriptorCount);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetHandCollisionAvailabilityV1(std::uint64_t ownerToken, RockProviderHand hand, RockProviderHandCollisionAvailabilityV1* outState);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiCancelInteractionCommandV1(std::uint64_t ownerToken, std::uint64_t commandId);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetHandInputSuppressionStateV1(std::uint64_t ownerToken, RockProviderHand hand, RockProviderHandInputSuppressionStateV1* outState);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiAcquireOffhandReservationV1(std::uint64_t ownerToken, const RockProviderOffhandReservationRequestV1* request);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiRenewOffhandReservationV1(std::uint64_t ownerToken, const RockProviderOffhandReservationRequestV1* request);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiReleaseOffhandReservationV1(std::uint64_t ownerToken);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetOffhandReservationStateV1(std::uint64_t ownerToken, RockProviderOffhandReservationStateV1* outState);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearNativeAnimationRuntimeV1(std::uint64_t ownerToken);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetTouchGrabTargetsForScopeV1(std::uint64_t ownerToken, std::uint64_t scopeToken, const RockProviderTouchGrabTargetV1* targets, std::uint32_t targetCount);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearTouchGrabTargetsForScopeV1(std::uint64_t ownerToken, std::uint64_t scopeToken);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyTouchGrabStatesForScopeV1(std::uint64_t ownerToken, std::uint64_t scopeToken, RockProviderTouchGrabStateV1* outStates, std::uint32_t maxStates, std::uint32_t* outStateCount);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestTouchGrabYieldV1(std::uint64_t ownerToken, std::uint64_t scopeToken, std::uint64_t targetId, std::uint32_t targetGeneration);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiQueryWorldRaycastV1(std::uint64_t ownerToken, const RockProviderWorldRaycastRequestV1* request, RockProviderWorldRaycastResultV1* outResult);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetColliderVisualizationOverrideV1(std::uint64_t ownerToken, const RockProviderColliderVisualizationRequestV1* request);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearColliderVisualizationOverrideV1(std::uint64_t ownerToken);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetLogicalInputActionStateV1(std::uint64_t ownerToken, RockProviderLogicalInputActionV1 action, RockProviderLogicalInputActionStateV1* outState);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetPlayerControllerStateV1(std::uint64_t ownerToken, std::uint32_t queryFlags, RockProviderPlayerControllerStateV1* outState);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestPlayerControllerJumpV1(std::uint64_t ownerToken, const RockProviderPlayerControllerJumpRequestV1* request);
    bool ROCK_PROVIDER_CALL apiGetRawWandThumbstickV1(RockProviderHand hand, float* outX, float* outY);
    std::uint32_t ROCK_PROVIDER_CALL apiGetNativeInputContextV1();
    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetHandTargetDetailsV1(std::uint64_t ownerToken, RockProviderHand hand, RockProviderHandTargetDetailsV1* outDetails);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiQueryReferenceInteractionV1(std::uint64_t ownerToken, const RockProviderReferenceQueryV1* query, RockProviderReferenceInteractionV1* outState);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiQueryPowerArmorTargetV1(std::uint64_t ownerToken, const RockProviderReferenceQueryV1* query, RockProviderPowerArmorTargetV1* outTarget);
    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestPowerArmorGrabV1(std::uint64_t ownerToken, const RockProviderPowerArmorGrabRequestV1* request, std::uint64_t* outCommandId);
    rock::api::Status authorize(std::uint64_t owner, rock::api::InterfaceId family, std::uint32_t permission, bool requireThread = true,
        OwnerAccess access = OwnerAccess::Existing);
    rock::api::Status bind(std::uint64_t owner, rock::api::InterfaceId family, std::uint32_t major, std::uint32_t permissions);
    rock::api::SampleV1 sample();
    void revoke(std::uint64_t owner);
    void refreshSources(); // Once per WeaponParts batch before key conversion.
    std::uintptr_t resolveSourceKey(std::uint64_t generation, std::uint64_t key);
    std::uint64_t sourceKey(std::uint64_t generation, std::uintptr_t node);
    std::uint64_t sourceKeyForBody(std::uint64_t generation, std::uint32_t body);
    std::uintptr_t resolveSourceName(std::uint64_t generation,const char* name);
    rock::api::Status copySources(std::uint64_t generation,std::uint32_t offset,rock::provider::WeaponSourceRecord*,std::uint32_t,std::uint32_t&,std::uint32_t&);
    void deferRevoke(std::uint64_t owner);
    rock::api::Status querySourcePose(std::uint64_t generation,std::uint64_t key,WeaponSourcePose&);
    void reportBoundaryFailure(std::uint64_t owner,rock::api::InterfaceId) noexcept;
    rock::api::Status querySourcePath(std::uint64_t generation,std::uint64_t key,std::uint64_t&,std::uint32_t&);
}
