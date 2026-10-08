#include <ROCK/EjectionContacts.h>
#include "api/ProviderRuntimeServices.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "physics-interaction/core/PhysicsInteraction.h"
namespace {
using namespace rock;
api::Status ROCK_CALL request(api::OwnerToken owner,const api::ejection_contacts::RequestV1* req) noexcept {
    if(!req)return api::Status::InvalidArgument;
    if(req->size!=sizeof(*req))return api::Status::InvalidSize;
    if(req->version!=1)return api::Status::UnsupportedVersion;
    if(!req->reference.referenceFormId||!req->expectedBase||!req->weaponForm||!req->weaponGenerationKey)return api::Status::InvalidArgument;
    try {
        const auto access=provider::runtime::authorize(owner,api::InterfaceId::Grab,3,true);if(access!=api::Status::Ok)return access;
        provider::RockProviderFrameSnapshot frame;
        if(!provider::runtime::apiGetFrameSnapshot(&frame)||!frame.providerReady||frame.menuBlocking)return api::Status::NotReady;
        if(frame.worldGeneration!=req->reference.worldGeneration||frame.skeletonGeneration!=req->reference.skeletonGeneration||frame.providerGeneration!=req->reference.providerGeneration||frame.weaponGenerationKey!=req->weaponGenerationKey||frame.weaponFormId!=req->weaponForm)return api::Status::GenerationMismatch;
        auto* ref=RE::TESForm::GetFormByID<RE::TESObjectREFR>(req->reference.referenceFormId);
        if(!ref||ref->GetFormType()!=RE::ENUM_FORM_ID::kREFR||ref->IsDeleted()||ref->IsDisabled())return api::Status::TargetUnavailable;
        auto* base=ref->GetObjectReference();
        if(!base||base->formID!=req->expectedBase||(req->reference.referenceNativeHandle&&ref->GetHandle().native_handle()!=req->reference.referenceNativeHandle))return api::Status::TargetInvalid;
        auto* physics=PhysicsInteraction::s_instance.load(std::memory_order_acquire);
        return physics?physics->requestEjectionContacts(owner,*req):api::Status::NotReady;
    }catch(...){return api::Status::InternalError;}
}
api::Status ROCK_CALL cancel(api::OwnerToken owner,std::uint32_t reference) noexcept {
    try {
        const auto access=provider::runtime::authorize(owner,api::InterfaceId::Grab,0,true);if(access!=api::Status::Ok)return access;
        auto* physics=PhysicsInteraction::s_instance.load(std::memory_order_acquire);if(!physics)return api::Status::NotReady;
        physics->cancelEjectionContacts(owner,reference);return api::Status::Ok;
    }catch(...){return api::Status::InternalError;}
}
}
extern "C" __declspec(dllexport) const rock::api::ejection_contacts::ApiV1* ROCK_CALL ROCK_EjectionContacts_QueryV1(std::uint32_t major,std::uint32_t bytes) noexcept {
    static const rock::api::ejection_contacts::ApiV1 table{sizeof(table),1,&request,&cancel};
    return major==1&&bytes==sizeof(table)?&table:nullptr;
}
