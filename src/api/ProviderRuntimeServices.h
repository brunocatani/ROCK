#pragma once
#include <ROCK/WeaponParts.h>
#include <ROCK/Weapon.h>
#include <ROCK/Input.h>
#include <ROCK/Grab.h>
#include <ROCK/Grab.h>
#include <ROCK/Animation.h>
#include <ROCK/Collision.h>
#include <ROCK/Core.h>
#include <ROCK/Hands.h>
#include <ROCK/PlayerController.h>
#include "WeaponSourceCatalog.h"
#include "WeaponPartGrabPoints.h"
#include "ProviderRuntimeTypes.h"
#include "OwnerBindingPolicy.h"

namespace rock::weapon_part_runtime {struct Target;}
namespace rock::provider::runtime {
    // The public Core snapshot of one internal publication; getSnapshot and
    // frame callbacks deliver the same values.
    inline api::core::Snapshot toCoreSnapshot(const RockProviderFrameSnapshot& snapshot) noexcept
    {
        api::core::Snapshot result{};
        result.sample = { snapshot.frameIndex, snapshot.stateSequence, snapshot.worldGeneration,
            snapshot.skeletonGeneration, snapshot.providerGeneration, snapshot.collisionGeneration };
        result.lifecycleFlags = snapshot.lifecycleFlags;
        result.lastLifecycleReason = static_cast<api::core::LifecycleReason>(snapshot.lastLifecycleReason);
        result.frikSkeletonReady = snapshot.frikSkeletonReady;
        result.menuBlocking = snapshot.menuBlocking;
        result.configBlocking = snapshot.configBlocking;
        result.providerReady = snapshot.providerReady;
        result.stableFrameCount = snapshot.stableFrameCount;
        result.deltaSeconds = snapshot.deltaSeconds;
        return result;
    }
    bool onOwnerThread() noexcept;
    api::Status setWeaponPartGrabPoints(api::OwnerToken,std::span<const api::weaponparts::GrabPoint>);
    api::Status clearWeaponPartGrabPoints(api::OwnerToken);
    bool hasGrabPoints(std::uint64_t generation);
    std::size_t copyWeaponPartPointSelection(std::span<weapon_part_runtime::Target>,std::span<weapon_part_points::OwnedPoint>,std::size_t& pointCount);
    api::Status setHeldObjectTarget(api::OwnerToken,const api::grab::HeldObjectTarget&);
    api::Status clearHeldObjectTarget(api::OwnerToken,api::Hand);
    api::Status getHeldObjectPose(api::OwnerToken,api::Hand,api::grab::HeldObjectPose&);
    api::Status getHeldPlacementState(api::grab::HeldPlacementState& output);
    api::Status submitHeldPlacementIntent(api::OwnerToken owner, const api::grab::HeldPlacementIntent& request);
    api::Status clearHeldPlacementIntent(api::OwnerToken owner);
    // Collision ejection clearance and object rest (EjectionContactsService.cpp,
    // ObjectRestService.cpp); game thread, caller already authorized.
    api::Status requestEjectionClearance(api::OwnerToken owner, const api::collision::EjectionClearanceRequest& request);
    api::Status cancelEjectionClearance(api::OwnerToken owner, std::uint32_t referenceFormId);
    api::Status watchObjectRest(api::OwnerToken owner, const api::collision::ObjectRestRequest& request, api::collision::ObjectRestState& state);
    api::Status cancelObjectRest(api::OwnerToken owner, std::uint32_t referenceFormId);
    api::Status getPlacementClickState(api::input::PlacementClickState& output);
    api::Status captureInventoryWeapon(std::uint64_t owner, std::uint32_t form, std::uint32_t stack,
        api::weapon::InventoryWeapon& output);
    api::Status getHandlingAuthority(api::weapon::HandlingAuthority& output);
    api::Status requestInventoryEquip(std::uint64_t owner, const api::weapon::EquipRequest& request, std::uint64_t& command);
    api::Status getInventoryEquipResult(std::uint64_t owner, std::uint64_t command, api::weapon::EquipResult& output);
    api::Status cancelInventoryEquip(std::uint64_t owner, std::uint64_t command);
    std::uint64_t currentGameFrameIndex(); // Atomic game clock, including pre-publication lifecycle events.
    const char* apiGetModVersion();
    bool apiIsProviderReady();
    bool apiGetFrameSnapshot(RockProviderFrameSnapshot* outSnapshot);
    bool apiQueryWeaponContactAtPoint(const RockProviderWeaponContactQuery* query, RockProviderWeaponContactResult* outResult);
    std::uint32_t apiCopyWeaponEvidenceDetailsV1(RockProviderWeaponEvidenceDetailV1* outDetails, std::uint32_t maxDetails);
    std::uint32_t apiCopyWeaponEvidenceDetailPointsV1(std::uint32_t bodyId, api::Point3* outPoints, std::uint32_t maxPoints);
    std::uint32_t apiGetBodyContactSnapshotV1(RockProviderBodyContactV1* outContacts, std::uint32_t maxContacts);
    api::Hand apiGetPrimaryHandV1();
    api::Hand apiGetOffhandHandV1();
    api::Status apiRegisterConsumerV1(const RockProviderConsumerRegistrationV1* registration, RockProviderConsumerHandleV1* outHandle);
    api::Status apiUnregisterConsumerV1(std::uint64_t ownerToken);
    api::Status apiRequestForceGrabV1(std::uint64_t ownerToken, const RockProviderForceGrabRequestV1* request, std::uint64_t* outCommandId);
    api::Status apiGetInteractionCommandResultV1(std::uint64_t ownerToken, std::uint64_t commandId, RockProviderInteractionCommandResultV1* outResult);
    api::Status apiRequestForceReleaseV1(std::uint64_t ownerToken, const RockProviderForceReleaseRequestV1* request, std::uint64_t* outCommandId);
    api::Status apiRequestThrownDropV1(std::uint64_t ownerToken, const RockProviderThrownDropRequestV1* request, std::uint64_t* outCommandId);
    api::Status apiSetHandInputSuppressionV1(std::uint64_t ownerToken, const ::rock::api::input::HandInputSuppressionRequest* request);
    api::Status apiClearHandInputSuppressionV1(std::uint64_t ownerToken, api::Hand hand);
    api::Status apiSetWeaponPartTargetsV1(std::uint64_t ownerToken, const RockProviderWeaponPartTargetV1* targets, std::uint32_t targetCount);
    api::Status apiClearWeaponPartTargetsV1(std::uint64_t ownerToken);
    api::Status apiSetWeaponPartDriveTargetsV1(std::uint64_t ownerToken, const RockProviderWeaponPartDriveTargetV1* targets, std::uint32_t targetCount);
    api::Status apiClearWeaponPartDriveTargetsV1(std::uint64_t ownerToken);
    bool apiQueryEquippedWeaponClassificationV1(RockProviderWeaponClassificationV1* outResult);
    bool apiGetWeaponPartGripStateV1(api::Hand hand, api::weaponparts::GripState* outState);
    api::weaponparts::GripState publicGripState(const RockProviderWeaponPartGripStateV1& grip);
    bool apiGetRawWandButtonStateV1(api::Hand hand, std::uint32_t buttonId, ::rock::api::input::RawWandButtonState* outState);
    bool apiIsNativePipboyInputSuppressedV1();
    std::uint32_t apiGetWeaponEmitterCountV1();
    std::uint32_t apiCopyWeaponEmittersV1(RockProviderWeaponEmitterV1* outEmitters, std::uint32_t maxEmitters);
    api::Status apiSetNativeAnimationAuthorityV1(std::uint64_t ownerToken, const ::rock::api::animation::NativeAnimationAuthorityRequest* request);
    api::Status apiClearNativeAnimationAuthorityV1(std::uint64_t ownerToken);
    bool apiGetNativeAnimationAuthorityStateV1(::rock::api::animation::NativeAnimationAuthorityState* outState);
    api::Status apiRegisterAnimationPhaseCallbackV1(std::uint64_t ownerToken, RockProviderAnimationPhaseCallbackV1 callback, void* userData, std::uint64_t* outCallbackToken);
    api::Status apiUnregisterAnimationPhaseCallbackV1(std::uint64_t ownerToken, std::uint64_t callbackToken);
    bool apiGetEquippedWeaponGripStateV1(std::uint64_t ownerToken, RockProviderEquippedWeaponGripStateV1* outState);
    api::Status apiSetHandVisualAuthorityV1(std::uint64_t ownerToken, const ::rock::api::animation::HandVisualAuthorityRequest* request);
    api::Status apiClearHandVisualAuthorityV1(std::uint64_t ownerToken, api::Hand hand);
    api::Status apiPublishNativeAnimationRuntimeV1(std::uint64_t ownerToken, const ::rock::api::animation::NativeAnimationRuntimePublication* publication);
    api::Status apiSetEquippedWeaponHandlingAuthorityV1(std::uint64_t ownerToken, const RockProviderEquippedWeaponHandlingRequestV1* request);
    api::Status apiClearEquippedWeaponHandlingAuthorityV1(std::uint64_t ownerToken);
    bool apiGetEquippedWeaponHandlingStateV1(RockProviderEquippedWeaponHandlingStateV1* outState);
    api::Status apiGetHandFrame(api::Hand hand, api::hands::HandFrame& outFrame);
    api::Status apiGetPresentedHandFrame(api::Hand hand, api::hands::HandFrame& outFrame);
    api::Status apiGetPresentedHandPose(std::uint64_t ownerToken, api::Hand hand, api::hands::PresentedHandPose& outPose);
    api::Status apiRegisterFrameCallbackForOwnerV1(std::uint64_t ownerToken, RockProviderFrameCallback callback, void* userData, std::uint64_t* outCallbackToken);
    api::Status apiUnregisterFrameCallbackForOwnerV1(std::uint64_t ownerToken, std::uint64_t callbackToken);
    api::Status apiGetHandInteractionStateV1(std::uint64_t ownerToken, api::Hand hand, RockProviderHandInteractionStateV1* outState);
    api::Status apiGetEquippedWeaponStateV1(std::uint64_t ownerToken, RockProviderEquippedWeaponStateV1* outState);
    api::Status apiQueryWeaponPartTargetResolutionV1(std::uint64_t ownerToken, const RockProviderWeaponPartResolutionQueryV1* query, RockProviderWeaponPartResolutionResultV1* outResolution);
    api::Status apiCopyWeaponPartPoseSnapshotV1(std::uint64_t ownerToken, RockProviderWeaponPartPoseV1* outParts, std::uint32_t maxParts, std::uint32_t* outPartCount);
    api::Status apiCopyWeaponPartDriveApplicationResultsV1(std::uint64_t ownerToken, RockProviderWeaponPartDriveApplicationResultV1* outResults, std::uint32_t maxResults, std::uint32_t* outResultCount);
    api::Status apiGetScopeSightStateV1(std::uint64_t ownerToken, RockProviderScopeSightStateV1* outState);
    api::Status apiGetWeaponCompositionStateV1(std::uint64_t ownerToken, RockProviderWeaponCompositionStateV1* outState);
    api::Status apiCopyWeaponCompositionEntriesV1(std::uint64_t ownerToken, RockProviderWeaponCompositionEntryV1* outEntries, std::uint32_t maxEntries, std::uint32_t* outEntryCount);
    api::Status apiGetSelectedAuthoredGripPoseV1(std::uint64_t ownerToken, RockProviderAuthoredGripPoseV1* outPose);
    api::Status apiCopySemanticHandContactsV1(std::uint64_t ownerToken, api::Hand hand, std::uint32_t maxFramesSinceContact, RockProviderSemanticHandContactV1* outContacts, std::uint32_t maxContacts, std::uint32_t* outContactCount);
    api::Status apiCopyPlayerColliderDescriptorsV1(std::uint64_t ownerToken, RockProviderPlayerColliderDescriptorV1* outDescriptors, std::uint32_t maxDescriptors, std::uint32_t* outDescriptorCount);
    api::Status apiGetHandCollisionAvailabilityV1(std::uint64_t ownerToken, api::Hand hand, RockProviderHandCollisionAvailabilityV1* outState);
    api::Status apiCancelInteractionCommandV1(std::uint64_t ownerToken, std::uint64_t commandId);
    api::Status apiGetHandInputSuppressionStateV1(std::uint64_t ownerToken, api::Hand hand, ::rock::api::input::HandInputSuppressionState* outState);
    api::Status apiAcquireOffhandReservationV1(std::uint64_t ownerToken, const RockProviderOffhandReservationRequestV1* request);
    api::Status apiRenewOffhandReservationV1(std::uint64_t ownerToken, const RockProviderOffhandReservationRequestV1* request);
    api::Status apiReleaseOffhandReservationV1(std::uint64_t ownerToken);
    api::Status apiGetOffhandReservationStateV1(std::uint64_t ownerToken, RockProviderOffhandReservationStateV1* outState);
    api::Status apiClearNativeAnimationRuntimeV1(std::uint64_t ownerToken);
    api::Status apiSetTouchGrabTargetsForScopeV1(std::uint64_t ownerToken, std::uint64_t scopeToken, const ::rock::api::touch::TouchGrabTarget* targets, std::uint32_t targetCount);
    api::Status apiClearTouchGrabTargetsForScopeV1(std::uint64_t ownerToken, std::uint64_t scopeToken);
    api::Status apiCopyTouchGrabStatesForScopeV1(std::uint64_t ownerToken, std::uint64_t scopeToken, ::rock::api::touch::TouchGrabState* outStates, std::uint32_t maxStates, std::uint32_t* outStateCount);
    api::Status apiRequestTouchGrabYieldV1(std::uint64_t ownerToken, std::uint64_t scopeToken, std::uint64_t targetId, std::uint32_t targetGeneration);
    api::Status apiQueryWorldRaycastV1(std::uint64_t ownerToken, const RockProviderWorldRaycastRequestV1* request, RockProviderWorldRaycastResultV1* outResult);
    api::Status apiGetLogicalInputActionStateV1(std::uint64_t ownerToken, ::rock::api::input::LogicalInputAction action, ::rock::api::input::LogicalInputActionState* outState);
    api::Status apiGetPlayerControllerState(std::uint64_t ownerToken, api::playercontroller::PlayerControllerState& outState);
    api::Status apiRequestPlayerControllerJump(std::uint64_t ownerToken, const api::playercontroller::PlayerControllerJumpRequest& request);
    bool apiGetRawWandThumbstickV1(api::Hand hand, float* outX, float* outY);
    std::uint32_t apiGetNativeInputContextV1();
    api::Status apiGetHandTargetDetailsV1(std::uint64_t ownerToken, api::Hand hand, RockProviderHandTargetDetailsV1* outDetails);
    api::Status apiQueryReferenceInteractionV1(std::uint64_t ownerToken, const RockProviderReferenceQueryV1* query, RockProviderReferenceInteractionV1* outState);
    api::Status apiQueryPowerArmorTargetV1(std::uint64_t ownerToken, const RockProviderReferenceQueryV1* query, RockProviderPowerArmorTargetV1* outTarget);
    api::Status apiRequestPowerArmorGrabV1(std::uint64_t ownerToken, const RockProviderPowerArmorGrabRequestV1* request, std::uint64_t* outCommandId);
    rock::api::Status authorize(std::uint64_t owner, rock::api::InterfaceId family, std::uint32_t permission, bool requireThread = true,
        OwnerAccess access = OwnerAccess::Existing);
    rock::api::Status bind(std::uint64_t owner, rock::api::InterfaceId family, std::uint32_t major, std::uint32_t permissions);
    rock::api::Sample sample();
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
