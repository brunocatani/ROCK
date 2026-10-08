#include "physics-interaction/core/PhysicsInteraction.h"
#include "api/ProviderRuntimeServices.h"
#include "RE/Bethesda/TESBoundObjects.h"

namespace rock {
void PhysicsInteraction::updateObjectRest(const PhysicsFrameContext& frame) {
    // Menu blocking is a pause, not a failed native watch. watchNative rejects
    // admission during menus, so do not turn existing watches into failures.
    if(frame.menuBlocked)return;
    const auto sample=provider::runtime::sample();
    auto& gate=_generatedBodyStepDrive.callbackGate();
    auto* right=_rightHand.isHolding()?_rightHand.getHeldRef():nullptr;
    auto* left=_leftHand.isHolding()?_leftHand.getHeldRef():nullptr;
    // Revoke cancelled/stale authority before any watch may freeze a body.
    for(auto& entry:_objectRestRequests.entries)if(entry.owner) {
        const auto id=entry.request.reference.referenceFormId;
        auto* ref=RE::TESForm::GetFormByID<RE::TESObjectREFR>(id);
        const auto& key=entry.request.reference;
        if(entry.cancelled || provider::runtime::authorize(entry.owner,api::InterfaceId::Grab,3,true)!=api::Status::Ok ||
           key.worldGeneration!=sample.worldGeneration || key.skeletonGeneration!=sample.skeletonGeneration || key.providerGeneration!=sample.providerGeneration) {
            entry.cancelled=true;
            if(ref && !_objectRestCollision.releaseReference(ref,frame.hknpWorld,gate))continue;
            entry={};continue;
        }
        auto* base=ref?ref->GetObjectReference():nullptr;
        if(!ref || ref->GetFormType()!=RE::ENUM_FORM_ID::kREFR || ref->IsDeleted() || ref->IsDisabled() || !ref->Get3D() ||
           !base || base->GetFormType()!=RE::ENUM_FORM_ID::kMISC || base->formID!=entry.request.expectedBase ||
           (key.referenceNativeHandle && ref->GetHandle().native_handle()!=key.referenceNativeHandle)) {
            entry.cancelled=true;
            if(ref && !_objectRestCollision.releaseReference(ref,frame.hknpWorld,gate))continue;
            entry={};continue;
        }
        if(ref==right || ref==left) {
            _objectRestCollision.releaseReference(ref,frame.hknpWorld,gate);
            entry.state=api::object_rest::State::Waiting;entry.status=api::Status::RequestQueued;continue;
        }
    }
    // Reuse precisely the weapon support witness, anchored rest interval,
    // native saved placement and lease cleanup, in a separate bounded pool.
    _objectRestCollision.update(frame,right,left,gate,false);
    // Drain every completion before any waiting request can reuse a freed body
    // slot. Otherwise admission can overwrite that slot's _rested receipt.
    for(auto& entry:_objectRestRequests.entries)if(entry.owner&&!entry.cancelled&&
        _objectRestCollision.takeNativeRestCompletion(entry.request.reference.referenceFormId)) {
        entry.state=api::object_rest::State::Resting;entry.status=api::Status::Ok;
        ROCK_LOG_INFO(Weapon,"Object rest completed ref={:08X}; physics ownership released, native placement retained",entry.request.reference.referenceFormId);
    }
    for(auto& entry:_objectRestRequests.entries)if(entry.owner && !entry.cancelled) {
        auto* ref=RE::TESForm::GetFormByID<RE::TESObjectREFR>(entry.request.reference.referenceFormId);
        if(!ref || ref==right || ref==left)continue;
        if(entry.state==api::object_rest::State::Resting || (entry.status!=api::Status::Ok && entry.status!=api::Status::RequestQueued))continue;
        using Result=LooseWeaponCollision::NativeRest;
        Result result;
        try {result=_objectRestCollision.watchNative(ref,frame,gate);}
        catch(...) {
            _objectRestCollision.releaseReference(ref,frame.hknpWorld,gate);entry.status=api::Status::InternalError;
            ROCK_LOG_SAMPLE_WARN(Weapon,2000,"Object rest preparation failed ref={:08X}; native collision retained",ref->formID);continue;
        }
        if(result==Result::CapacityFull)continue;
        if(result==Result::Unavailable){entry.status=api::Status::TargetUnavailable;continue;}
        entry.state=result==Result::Resting?api::object_rest::State::Resting:api::object_rest::State::Watching;
        entry.status=api::Status::Ok;
    }
}
}
