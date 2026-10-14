#pragma once
#include <ROCK/WeaponParts.h>
#include <ROCK/Weapon.h>
#include <ROCK/Input.h>
#include <ROCK/Grab.h>
#include <ROCK/Animation.h>
#include <ROCK/Collision.h>
#include <ROCK/Core.h>
#include <ROCK/Hands.h>
#include <ROCK/PlayerController.h>
#include "WeaponSourceCatalog.h"
#include "WeaponPartGrabPoints.h"
#include "ProviderPublication.h"
#include <ROCK/Touch.h>
#include "ProviderStatePolicy.h"
#include "WeaponPartRequests.h"
#include "OwnerBindingPolicy.h"

namespace rock::weapon_part_runtime {struct Target;}
namespace rock::provider::runtime {
    // The public Core snapshot of one internal publication; getSnapshot and
    // frame callbacks deliver the same values.
    inline api::core::Snapshot toCoreSnapshot(const Publication& snapshot) noexcept
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
    const char* getModVersion();
    bool isProviderReady();
    // Copies the latest publication; false before the first one.
    bool copyPublication(Publication& outPublication);
    std::uint32_t copyBodyContacts(api::collision::BodyContact* outContacts, std::uint32_t capacity);
    // The hand holding the weapon's firing role, and the other hand.
    api::Hand primaryHand();
    api::Hand offhandHand();
    // Weapon slot 0. The publication's state, then the live reads (game thread).
    api::Status copyPublishedWeaponState(api::weapon::WeaponState& outState);
    bool queryWeaponHandling(provider_state_policy::WeaponHandling& outHandling);
    api::Status getWeaponGrip(api::weapon::Grip& outGrip);
    api::Status getWeaponClassification(api::weapon::Classification& outClassification);
    std::uint32_t weaponEmitterCount();
    std::uint32_t copyWeaponEmitters(api::weapon::Emitter* outEmitters, std::uint32_t capacity);
    api::Status getWeaponCompositionState(api::weapon::CompositionState& outState);
    api::Status copyWeaponCompositionEntries(api::weapon::CompositionEntry* outEntries, std::uint32_t capacity, std::uint32_t& outCount);
    api::Status getAuthoredGripPose(api::weapon::AuthoredGripPose& outPose);
    api::Status getScopeSightState(api::weapon::ScopeSightState& outState);
    api::Status setHandlingAuthority(std::uint64_t ownerToken, const api::weapon::HandlingRequest& request);
    api::Status clearHandlingAuthority(std::uint64_t ownerToken);
    api::Status registerOwner(const api::core::Registration& registration, api::core::Owner& outOwner);
    api::Status unregisterOwner(std::uint64_t ownerToken);
    api::Status setHandInputSuppression(std::uint64_t ownerToken, const api::input::HandInputSuppressionRequest& request);
    api::Status clearHandInputSuppression(std::uint64_t ownerToken, api::Hand hand);
    bool getRawWandButtonState(api::Hand hand, std::uint32_t buttonId, api::input::RawWandButtonState& outState);
    bool isNativePipboyInputSuppressed();
    api::Status setNativeAnimationAuthority(std::uint64_t ownerToken, const api::animation::NativeAnimationAuthorityRequest& request);
    api::Status clearNativeAnimationAuthority(std::uint64_t ownerToken);
    bool getNativeAnimationAuthorityState(api::animation::NativeAnimationAuthorityState& outState);
    api::Status registerAnimationPhaseCallback(std::uint64_t ownerToken, api::animation::PhaseCallback callback, void* userData, std::uint64_t* outCallbackToken);
    api::Status unregisterAnimationPhaseCallback(std::uint64_t ownerToken, std::uint64_t callbackToken);
    api::Status setHandVisualAuthority(std::uint64_t ownerToken, const api::animation::HandVisualAuthorityRequest& request);
    api::Status clearHandVisualAuthority(std::uint64_t ownerToken, api::Hand hand);
    api::Status publishNativeAnimationRuntime(std::uint64_t ownerToken, const api::animation::NativeAnimationRuntimePublication& publication);
    api::Status getHandFrame(api::Hand hand, api::hands::HandFrame& outFrame);
    api::Status getPresentedHandFrame(api::Hand hand, api::hands::HandFrame& outFrame);
    api::Status getPresentedHandPose(std::uint64_t ownerToken, api::Hand hand, api::hands::PresentedHandPose& outPose);
    api::Status registerFrameCallback(std::uint64_t ownerToken, api::core::FrameCallback callback, void* userData, std::uint64_t* outCallbackToken);
    api::Status unregisterFrameCallback(std::uint64_t ownerToken, std::uint64_t callbackToken);
    api::Status copySemanticHandContacts(api::Hand hand, std::uint32_t maxFramesSinceContact, api::collision::SemanticHandContact* outContacts, std::uint32_t capacity, std::uint32_t& outCount);
    api::Status copyPlayerColliderDescriptors(api::collision::PlayerColliderDescriptor* outDescriptors, std::uint32_t capacity, std::uint32_t& outCount);
    api::Status getHandCollisionAvailability(api::Hand hand, api::collision::HandCollisionAvailability& outState);
    api::Status getHandInputSuppressionState(std::uint64_t ownerToken, api::Hand hand, api::input::HandInputSuppressionState& outState);
    api::Status clearNativeAnimationRuntime(std::uint64_t ownerToken);
    api::Status setTouchGrabTargetsForScope(std::uint64_t ownerToken, std::uint64_t scopeToken, const api::touch::TouchGrabTarget* targets, std::uint32_t targetCount);
    api::Status clearTouchGrabTargetsForScope(std::uint64_t ownerToken, std::uint64_t scopeToken);
    api::Status copyTouchGrabStatesForScope(std::uint64_t ownerToken, std::uint64_t scopeToken, api::touch::TouchGrabState* outStates, std::uint32_t capacity, std::uint32_t& outCount);
    api::Status requestTouchGrabYield(std::uint64_t ownerToken, std::uint64_t scopeToken, std::uint64_t targetId, std::uint32_t targetGeneration);
    api::Status queryWorldRaycast(std::uint64_t ownerToken, const api::collision::WorldRaycastRequest& request, api::collision::WorldRaycastResult& outResult);
    api::Status getLogicalInputActionState(std::uint64_t ownerToken, api::input::LogicalInputAction action, api::input::LogicalInputActionState& outState);
    api::Status getPlayerControllerState(std::uint64_t ownerToken, api::playercontroller::PlayerControllerState& outState);
    api::Status requestPlayerControllerJump(std::uint64_t ownerToken, const api::playercontroller::PlayerControllerJumpRequest& request);
    bool getRawWandThumbstick(api::Hand hand, float& outX, float& outY);
    std::uint32_t getNativeInputContext();
    rock::api::Status authorize(std::uint64_t owner, rock::api::InterfaceId family, std::uint32_t permission, bool requireThread = true,
        OwnerAccess access = OwnerAccess::Existing);
    rock::api::Status bind(std::uint64_t owner, rock::api::InterfaceId family, std::uint32_t major, std::uint32_t permissions);
    rock::api::Sample sample();
    void revoke(std::uint64_t owner);
    // Grab: interaction commands, hand state and reservations.
    api::Status requestForceGrab(std::uint64_t ownerToken, const api::grab::ForceGrabRequest& request, std::uint64_t& outCommandId);
    api::Status requestForceRelease(std::uint64_t ownerToken, const api::grab::ForceReleaseRequest& request, std::uint64_t& outCommandId);
    api::Status requestThrownDrop(std::uint64_t ownerToken, const api::grab::ThrownDropRequest& request, std::uint64_t& outCommandId);
    api::Status requestPowerArmorGrab(std::uint64_t ownerToken, const api::grab::PowerArmorGrabRequest& request, std::uint64_t& outCommandId);
    api::Status getInteractionCommandResult(std::uint64_t ownerToken, std::uint64_t commandId, api::grab::InteractionCommandResult& outResult);
    api::Status cancelInteractionCommand(std::uint64_t ownerToken, std::uint64_t commandId);
    // The hand's state at the last publication.
    api::Status getHandInteractionState(std::uint64_t ownerToken, api::Hand hand, api::grab::HandInteractionState& outState);
    api::Status getHandTargetDetails(std::uint64_t ownerToken, api::Hand hand, api::grab::HandTargetDetails& outDetails);
    api::Status acquireHandReservation(std::uint64_t ownerToken, const api::grab::HandReservationRequest& request);
    api::Status renewHandReservation(std::uint64_t ownerToken, const api::grab::HandReservationRequest& request);
    api::Status releaseHandReservation(std::uint64_t ownerToken);
    api::Status getHandReservationState(std::uint64_t ownerToken, api::grab::HandReservationState& outState);
    api::Status queryReferenceInteraction(std::uint64_t ownerToken, const api::grab::ReferenceQuery& query, api::grab::ReferenceInteraction& outInteraction);
    api::Status queryPowerArmorTarget(std::uint64_t ownerToken, const api::grab::ReferenceQuery& query, api::grab::PowerArmorTarget& outTarget);
    void refreshSources(); // Once per WeaponParts batch before key conversion.
    // Weapon slot 0's parts. Records carry opaque source keys; targets and
    // drives arrive with their keys already resolved to live nodes.
    bool queryWeaponContact(const api::weaponparts::ContactQuery& query, api::weaponparts::ContactResult& outResult);
    std::uint32_t copyWeaponEvidenceDetails(api::weaponparts::EvidenceDetail* outDetails, std::uint32_t capacity);
    std::uint32_t copyWeaponEvidencePoints(std::uint32_t bodyId, api::Point3* outPoints, std::uint32_t capacity);
    api::Status setWeaponPartTargets(std::uint64_t ownerToken, std::span<const PartTarget> targets);
    api::Status clearWeaponPartTargets(std::uint64_t ownerToken);
    api::Status setWeaponPartDrives(std::uint64_t ownerToken, std::span<const PartDrive> drives);
    api::Status clearWeaponPartDrives(std::uint64_t ownerToken);
    api::Status queryWeaponPartTargetResolution(const provider::PartQuery& query, api::weaponparts::ResolutionResult& outResult);
    api::Status copyWeaponPartPoses(api::weaponparts::PartPose* outPoses, std::uint32_t capacity, std::uint32_t& outCount);
    api::Status copyWeaponPartDriveResults(std::uint64_t ownerToken, api::weaponparts::DriveResult* outResults, std::uint32_t capacity, std::uint32_t& outCount);
    // The grip each hand held at the last publication.
    bool copyPartGripState(api::Hand hand, api::weaponparts::GripState& outState);
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
