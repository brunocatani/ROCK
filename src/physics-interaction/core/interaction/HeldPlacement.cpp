#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/native/NativeObjectPlacement.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "api/ProviderRuntimeServices.h"
#include "RE/Havok/hknpBody.h"

namespace rock {
api::Status PhysicsInteraction::queryHeldPlacementState(api::grab::v1_1::HeldPlacementState& out) const
{
    out={};
    const auto& frame=runtime_state::currentFrame();
    auto* bhk=getPlayerBhkWorld();
    auto* world=bhk ? getHknpWorld(bhk) : nullptr;
    if (!isProviderReady() || !frame.visualAuthorityAvailable || !frame.localSkeletonReady ||
        frame.localMenuBlocking || frame.compatibilityConfigBlocking || !world ||
        bhk!=_lifecycle.cachedBhkWorld || world!=_lifecycle.cachedHknpWorld ||
        !physicsWritesAllowedForWorld(world) || !native_object_placement::available()) return api::Status::NotReady;
    out.frameToken=_placementLease.token();
    out.sample=provider::runtime::sample();
    out.lastRequestToken=_placementLastRequestToken;
    out.lastResult=_placementLastResult;
    out.lastRequestFormId=_placementLastRequestForm;
    using Flag=api::grab::v1_1::HeldPlacementFlag;
    {
        havok_world_lock::ScopedWorldReadLock lock(world);
        for (const bool left : {false,true}) {
            const auto& hand=left ? _leftHand : _rightHand;
            auto& state=left ? out.left : out.right;
            if (!hand.isHolding() || !hand.getHeldRef()) continue;
            state.formId=hand.getHeldRef()->GetFormID();
            const auto& saved=hand.getSavedObjectState();
            if (!saved.isValid() || saved.refr->IsDeleted() || saved.refr->IsDisabled() ||
                saved.targetKind!=grab_target::Kind::LooseObject || hand.getHeldBodyIds().empty() ||
                hand.getHeldBodyIds().size()>held_placement_policy::kMaxBodies) continue;
            state.flags=static_cast<std::uint32_t>(Flag::LooseObject);
            const auto contact=hand.readHeldSurfaceContact();
            if (!contact.recent || !hand.isHeldBodyId(contact.heldBodyId) ||
                _rightHand.isHeldBodyId(contact.otherBodyId) || _leftHand.isHeldBodyId(contact.otherBodyId)) continue;
            const auto other=havok_runtime::snapshotBodyIdentity(world,RE::hknpBodyId{contact.otherBodyId});
            if (!other.valid || !other.body || !other.body->shape) continue;
            const auto motion=physics_body_classifier::motionTypeFromBodyFlags(other.body->flags);
            if (motion==physics_body_classifier::BodyMotionType::Static || motion==physics_body_classifier::BodyMotionType::Keyframed)
                state.flags|=static_cast<std::uint32_t>(Flag::FixedSurfaceContact);
        }
    }
    const auto raw=input_remap_runtime::peekRawButtonState(false,32);
    out.buttonAvailable=raw.available && !input_remap_runtime::isProviderOpenVrGameInputSuppressedForHand(false);
    out.buttonHeld=raw.held; out.buttonPressed=raw.pressed; out.buttonAgeMilliseconds=raw.sampleAgeMilliseconds;
    return api::Status::Ok;
}

std::uint32_t PhysicsInteraction::heldPlacementCandidate(RE::hknpWorld* world) const
{
    if (world!=_lifecycle.cachedHknpWorld) return 0;
    api::grab::v1_1::HeldPlacementState state;
    if (queryHeldPlacementState(state)!=api::Status::Ok) return 0;
    const auto id=held_placement_policy::singleObject(state.right.formId,state.left.formId);
    if (!id) return 0;
    using Flag=api::grab::v1_1::HeldPlacementFlag;
    for (const auto& hand : {state.right,state.left})
        if (hand.formId && !(hand.flags&static_cast<std::uint32_t>(Flag::LooseObject))) return 0;
    return ((state.right.flags|state.left.flags)&static_cast<std::uint32_t>(Flag::FixedSurfaceContact)) ? id : 0;
}

api::Status PhysicsInteraction::submitHeldPlacementIntent(api::OwnerToken owner,const api::grab::v1_1::HeldPlacementIntent& intent)
{
    using Flag=api::grab::v1_1::PlacementIntentFlag;
    const auto sample=provider::runtime::sample();
    if (intent.reserved || (intent.flags&~3u) ||
        ((intent.flags&static_cast<std::uint32_t>(Flag::Anchor)) &&
         (!(intent.flags&static_cast<std::uint32_t>(Flag::ReserveClick)) || !intent.formId))) return api::Status::InvalidArgument;
    if (intent.sample.frameIndex!=sample.frameIndex ||
        intent.sample.worldGeneration!=sample.worldGeneration || intent.sample.skeletonGeneration!=sample.skeletonGeneration ||
        intent.sample.providerGeneration!=sample.providerGeneration ||
        intent.sample.collisionGeneration!=sample.collisionGeneration) return api::Status::GenerationMismatch;
    using Admission=held_placement_policy::FrameLease::Admission;
    switch (_placementLease.submit(owner,intent.frameToken)) {
    case Admission::InvalidOwner: return api::Status::InvalidArgument;
    case Admission::Stale: return api::Status::GenerationMismatch;
    case Admission::Busy: return api::Status::Busy;
    case Admission::Accepted: break;
    }
    _placementIntent=intent;
    return api::Status::RequestQueued;
}

void PhysicsInteraction::clearHeldPlacementIntent(api::OwnerToken owner)
{
    if (_placementLease.owner()==owner) { _placementLease.clear(owner); _placementIntent={}; }
    if (_placementActiveOwner==owner) {
        _placementActiveOwner=0; _placementClickReserved=false;
        _placementCandidate=0; _placementRequestedForm=0;
    }
    if (_placementPendingForm && _placementPendingOwner==owner) {
        _placementPendingOwner=0; _placementPendingForm=0;
        _placementLastResult=api::grab::v1_1::PlacementResult::Rejected;
    }
}

void PhysicsInteraction::resetHeldPlacement()
{
    _placementIntent={}; _placementLease.invalidate();
    _placementActiveOwner=0; _placementPendingOwner=0;
    _placementPendingForm=0; _placementRequestedForm=0;
    _placementCandidate=0; _placementClickReserved=false;
    _placementLastRequestToken=0; _placementLastRequestForm=0;
    _placementScriptWait=0.0f; _placementLastResult=api::grab::v1_1::PlacementResult::None;
}

void PhysicsInteraction::beginHeldPlacementFrame()
{
    const auto intent=std::exchange(_placementIntent,{});
    const auto owner=_placementLease.consume();
    const auto sample=provider::runtime::sample();
    _placementActiveOwner=0; _placementClickReserved=false; _placementCandidate=0; _placementRequestedForm=0;
    const auto& frame=runtime_state::currentFrame();
    const bool usable=isInitialized() && frame.visualAuthorityAvailable && frame.localSkeletonReady &&
        !frame.localMenuBlocking && !frame.compatibilityConfigBlocking &&
        !input_remap_runtime::isProviderOpenVrGameInputSuppressedForHand(false);
    if (!usable || !owner || intent.sample.worldGeneration!=sample.worldGeneration ||
        intent.sample.skeletonGeneration!=sample.skeletonGeneration ||
        intent.sample.providerGeneration!=sample.providerGeneration ||
        intent.sample.collisionGeneration!=sample.collisionGeneration ||
        provider::runtime::authorize(owner,api::grab::kInterfaceId,2,true)!=api::Status::Ok) {
        if (_placementPendingForm) _placementLastResult=api::grab::v1_1::PlacementResult::Rejected;
        _placementPendingForm=0; _placementPendingOwner=0;
        return;
    }
    using Flag=api::grab::v1_1::PlacementIntentFlag;
    if (_placementPendingForm && _placementPendingOwner!=owner) {
        _placementLastResult=api::grab::v1_1::PlacementResult::Rejected;
        _placementPendingForm=0; _placementPendingOwner=0;
    }
    _placementActiveOwner=owner;
    _placementCandidate=intent.formId;
    _placementClickReserved=(intent.flags&static_cast<std::uint32_t>(Flag::ReserveClick))!=0;
    if (intent.flags&static_cast<std::uint32_t>(Flag::Anchor)) {
        _placementRequestedForm=intent.formId;
        if (!_placementPendingForm) {
            _placementLastRequestToken=intent.frameToken;
            _placementLastRequestForm=intent.formId;
            // If this update exits before commit, the caller still receives a
            // terminal result instead of waiting forever for a consumed token.
            _placementLastResult=api::grab::v1_1::PlacementResult::Rejected;
        }
    }
    // A submitted lease observes this click before bipod. An unreserved click
    // remains available to bipod; otherwise drain its edges exactly once.
    if (_placementClickReserved || !g_rockConfig.rockBipodMode)
        (void)input_remap_runtime::consumeRawButtonState(false,32);
}

void PhysicsInteraction::queryPlacementClickState(api::input::v1_1::PlacementClickState& out) const
{
    out={}; out.sample=provider::runtime::sample();
    if (!_placementActiveOwner ||
        provider::runtime::authorize(_placementActiveOwner,api::grab::kInterfaceId,2,true)!=api::Status::Ok) return;
    if (_placementClickReserved) out.flags=static_cast<std::uint32_t>(api::input::v1_1::PlacementClickFlag::InputReserved);
    out.objectFormId=_placementCandidate;
}

void PhysicsInteraction::commitHeldPlacement(const PhysicsFrameContext& frame)
{
    const auto click=std::exchange(_placementRequestedForm,0u);
    const bool starting=click!=0 && _placementPendingForm==0;
    if (starting) {
        _placementPendingForm=click; _placementScriptWait=0.0f; _placementPendingOwner=_placementActiveOwner;
    }
    const auto requested=_placementPendingForm;
    if (!requested) return;
    _placementLastResult=api::grab::v1_1::PlacementResult::Rejected;
    if (!_placementActiveOwner || _placementPendingOwner!=_placementActiveOwner ||
        provider::runtime::authorize(_placementActiveOwner,api::grab::kInterfaceId,2,true)!=api::Status::Ok ||
        requested!=_placementCandidate || frame.menuBlocked || !frame.worldReady || !physicsWritesAllowedForWorld(frame.hknpWorld) ||
        requested!=heldPlacementCandidate(frame.hknpWorld)) {
        ROCK_LOG_INFO(Hand,"Held placement rejected ref={:08X} stage=commit-eligibility menu={} worldReady={}",requested,frame.menuBlocked,frame.worldReady);
        _placementPendingForm=0;
        return;
    }
    auto mutation=_generatedBodyStepDrive.callbackGate().pauseForMutation();
    auto& owner=_rightHand.isHolding() ? _rightHand : _leftHand;
    const auto retained=owner.getSavedObjectState().retainedRef;
    auto* ref=retained.get();
    auto* root=ref ? ref->Get3D() : nullptr;
    auto* cell=ref ? ref->GetParentCell() : nullptr;
    if (!root || !cell || cell->GetbhkWorld()!=frame.bhkWorld) {
        ROCK_LOG_WARN(Hand,"Held placement rejected ref={:08X} stage=reference-world",requested);
        _placementPendingForm=0;
        return;
    }
    const auto prepared=native_object_placement::prepareLoadScript(ref,starting);
    if (prepared==native_object_placement::ScriptPreparation::Rejected) { _placementPendingForm=0; return; }
    if (prepared==native_object_placement::ScriptPreparation::Pending) {
        _placementLastResult=api::grab::v1_1::PlacementResult::Pending;
        _placementScriptWait+=std::clamp(frame.deltaSeconds,0.0f,0.1f);
        if (_placementScriptWait>=2.0f) {
            _placementLastResult=api::grab::v1_1::PlacementResult::Rejected;
            ROCK_LOG_WARN(Hand,"Held placement rejected ref={:08X} stage=load-script-timeout; grab retained",requested);
            _placementPendingForm=0;
        }
        return;
    }
    _placementPendingForm=0;
    const auto pose=root->world;
    std::array<std::uint32_t,held_placement_policy::kMaxBodies> bodies{};
    std::size_t count=0;
    {
        // Cached grab prep deliberately leaves its lifecycle incomplete after
        // native activation. Re-enumerate the current tree once on placement,
        // without seeding a held body that could conceal missing discovery.
        using namespace object_physics_body_set;
        havok_world_lock::ScopedWorldReadLock lock(frame.hknpWorld);
        BodySetScanOptions options{};
        options.requireSameResolvedRef=true;
        ObjectPhysicsBodyScanCursor cursor;
        ObjectPhysicsBodyScanCache cache;
        if (!beginObjectPhysicsBodyScanCache(ref,options,cursor,cache)) {
            ROCK_LOG_WARN(Hand,"Held placement rejected ref={:08X} stage=body-scan-start",requested);
            return;
        }
        const auto step=advanceObjectPhysicsBodyScanCache(frame.hknpWorld,options,
            {256,64,static_cast<std::uint32_t>(bodies.size())},cursor,cache);
        const auto scanned=buildObjectPhysicsBodySetFromScanCache(frame.bhkWorld,frame.hknpWorld,ref,options,cache);
        const auto isPreservedStatic=[&](const ObjectPhysicsBodyRecord& record) {
            return held_placement_policy::preserveStaticBody(record.rejectReason,
                physics_body_classifier::motionTypeFromBodyFlags(record.bodyFlags),record.motionId,
                record.refResolutionKnown && record.resolvedRef==ref);
        };
        const auto staticBodies=static_cast<std::size_t>(std::count_if(scanned.records.begin(),scanned.records.end(),isPreservedStatic));
        const auto& diagnostics=scanned.diagnostics;
        const auto issues=diagnostics.scanFailures+diagnostics.invalidPhysicsSystems+
            diagnostics.benignScanSkips+diagnostics.foreignRefBodySkips+diagnostics.unresolvedRefBodySkips+
            diagnostics.weaponExpansionSkips+diagnostics.depthLimitSkips+diagnostics.staleCacheEntrySkips;
        const bool complete=held_placement_policy::completeBodyScan(step.finished && !step.invalidated,
            scanned.records.size(),scanned.acceptedCount()+staticBodies,issues);
        ROCK_LOG_INFO(Hand,"Held placement body scan ref={:08X} complete={} finished={} bodies={} accepted={} preservedStatic={} issues={} nodes={} collisionObjects={} budgetExhausted={}",
            requested,complete,step.finished,scanned.records.size(),scanned.acceptedCount(),staticBodies,issues,
            diagnostics.visitedNodes,diagnostics.collisionObjects,step.budgetExhausted);
        if (!complete) {
            ROCK_LOG_WARN(Hand,
                "Held placement scan rejected ref={:08X} failures={} invalidSystems={} benignSkips={} foreignSkips={} unresolvedSkips={} depthSkips={} staleSkips={}",
                requested,diagnostics.scanFailures,diagnostics.invalidPhysicsSystems,diagnostics.benignScanSkips,
                diagnostics.foreignRefBodySkips,diagnostics.unresolvedRefBodySkips,
                diagnostics.depthLimitSkips,diagnostics.staleCacheEntrySkips);
            for (std::size_t i=0;i<(std::min)(scanned.records.size(),bodies.size());++i) {
                const auto& record=scanned.records[i];
                if (record.accepted || isPreservedStatic(record)) continue;
                ROCK_LOG_WARN(Hand,
                    "Held placement body rejected ref={:08X} body={} reason={} layer={} filter=0x{:08X} motion={} motionType={} motionProps={} flags=0x{:08X} refKnown={} resolvedRef={:08X} node='{}'",
                    requested,record.bodyId,physics_body_classifier::rejectReasonName(record.rejectReason),
                    record.collisionLayer,record.filterInfo,record.motionId,static_cast<unsigned>(record.motionType),
                    record.motionPropertiesId,record.bodyFlags,record.refResolutionKnown,
                    record.resolvedRef ? record.resolvedRef->GetFormID() : 0,
                    record.owningNode && record.owningNode->name.c_str() ? record.owningNode->name.c_str() : "(none)");
            }
            return;
        }
        for (const auto* hand : { &_rightHand, &_leftHand }) {
            if (!hand->isHolding()) continue;
            for (const auto id:hand->getHeldBodyIds()) {
                if (!scanned.containsAcceptedBody(id)) {
                    ROCK_LOG_WARN(Hand,"Held placement rejected ref={:08X} stage=held-body-missing body={}",requested,id);
                    return;
                }
            }
        }
        for (const auto& record:scanned.records) bodies[count++]=record.bodyId;
    }
    // Release both motors and their inertia/filter leases before freezing. A
    // native save never retains ROCK's hand constraint or a transient body ID.
    std::array<bool,2> released{};
    bool allReleased=true;
    for (const bool left : { false,true }) {
        auto& hand=left ? _leftHand : _rightHand;
        if (!hand.isHolding()) continue;
        auto context=makeGrabReleaseContext(hand,left);
        context.applyCapturedReleaseVelocity=false;
        context.reason="provider-held-placement";
        released[left ? 1u : 0u]=hand.releaseGrabbedObject(frame.hknpWorld,
            GrabReleaseCollisionRestoreMode::Delayed,context).released;
        if (hand.isHolding()) {
            ROCK_LOG_WARN(Hand,"Held placement rejected ref={:08X} stage=release hand={}",requested,left ? "left" : "right");
            allReleased=false;
            break;
        }
        releaseObject(ref,claimOwnerForHand(left));
        clearGameplayCandidatesForHand(hand,left);
        grab_input_intent_policy::reset(_grabInput.intentStates[left ? 1u : 0u]);
        publishHandInputOwnership(hand,left);
    }
    const bool anchored=allReleased && native_object_placement::anchor(ref,frame.hknpWorld,pose,
        std::span<const std::uint32_t>{bodies.data(),count});
    _placementCandidate=0;
    _placementLastResult=anchored ? api::grab::v1_1::PlacementResult::Anchored : api::grab::v1_1::PlacementResult::Rejected;
    ROCK_LOG_INFO(Hand,"Held placement ref={:08X} anchored={} bodies={} position=({:.3f},{:.3f},{:.3f}) save=native",
        requested,anchored,count,pose.translate.x,pose.translate.y,pose.translate.z);
    for (const bool left : { false,true }) {
        if (!released[left ? 1u : 0u]) continue;
        dispatchPhysicsMessage(kPhysMsg_OnRelease,left,ref,requested,0);
        dispatchSimpleGrabEvent(GrabEventType::Released,left,ref);
    }
}
}
