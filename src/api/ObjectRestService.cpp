#include <ROCK/ObjectRest.h>
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "RE/Bethesda/TESBoundObjects.h"

namespace {
    using namespace rock;
    api::Status ROCK_CALL watch(api::OwnerToken owner,const api::object_rest::RequestV1* request,api::object_rest::State* state) noexcept {
        if(state)*state=api::object_rest::State::Waiting;
        if(!request || !state)return api::Status::InvalidArgument;
        if(request->size!=sizeof(*request))return api::Status::InvalidSize;
        if(request->version!=1)return api::Status::UnsupportedVersion;
        if(!request->reference.referenceFormId || !request->expectedBase)return api::Status::InvalidArgument;
        try {
            const auto access=provider::runtime::authorize(owner,api::InterfaceId::Grab,3,true);
            if(access!=api::Status::Ok)return access;
            provider::RockProviderFrameSnapshot frame;
            if(!provider::runtime::apiGetFrameSnapshot(&frame) || !frame.providerReady || frame.menuBlocking)return api::Status::NotReady;
            if(frame.worldGeneration!=request->reference.worldGeneration || frame.skeletonGeneration!=request->reference.skeletonGeneration ||
               frame.providerGeneration!=request->reference.providerGeneration)return api::Status::GenerationMismatch;
            auto* ref=RE::TESForm::GetFormByID<RE::TESObjectREFR>(request->reference.referenceFormId);
            if(!ref || ref->GetFormType()!=RE::ENUM_FORM_ID::kREFR || ref->IsDeleted() || ref->IsDisabled() || !ref->Get3D())return api::Status::TargetUnavailable;
            auto* base=ref->GetObjectReference();
            if(!base || base->GetFormType()!=RE::ENUM_FORM_ID::kMISC || base->formID!=request->expectedBase ||
               (request->reference.referenceNativeHandle && ref->GetHandle().native_handle()!=request->reference.referenceNativeHandle))return api::Status::TargetInvalid;
            auto* physics=PhysicsInteraction::s_instance.load(std::memory_order_acquire);
            return physics?physics->watchObjectRest(owner,*request,*state):api::Status::NotReady;
        }catch(...){return api::Status::InternalError;}
    }
    api::Status ROCK_CALL cancel(api::OwnerToken owner,std::uint32_t reference) noexcept {
        try {
            const auto access=provider::runtime::authorize(owner,api::InterfaceId::Grab,0,true);
            if(access!=api::Status::Ok)return access;
            auto* physics=PhysicsInteraction::s_instance.load(std::memory_order_acquire);
            if(!physics)return api::Status::NotReady;
            physics->cancelObjectRest(owner,reference);return api::Status::Ok;
        }catch(...){return api::Status::InternalError;}
    }
}
extern "C" __declspec(dllexport) const rock::api::object_rest::ApiV1* ROCK_CALL ROCK_ObjectRest_QueryV1(std::uint32_t major,std::uint32_t bytes) noexcept {
    static const rock::api::object_rest::ApiV1 table{sizeof(table),1,&watch,&cancel};
    return major==1 && bytes==sizeof(table)?&table:nullptr;
}
