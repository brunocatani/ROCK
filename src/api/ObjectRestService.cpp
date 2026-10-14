#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "RE/Bethesda/TESBoundObjects.h"

// Collision object rest: validates the MISC reference, then hands the watch to
// PhysicsInteraction. Authorization is the caller's.
namespace rock::provider::runtime {
api::Status watchObjectRest(api::OwnerToken owner, const api::collision::ObjectRestRequest& request, api::collision::ObjectRestState& state) {
    state = api::collision::ObjectRestState::Waiting;
    if (!request.reference.referenceFormId || !request.expectedBase) return api::Status::InvalidArgument;
    Publication frame;
    if (!copyPublication(frame) || !frame.providerReady || frame.menuBlocking) return api::Status::NotReady;
    if (frame.worldGeneration != request.reference.worldGeneration || frame.skeletonGeneration != request.reference.skeletonGeneration ||
        frame.providerGeneration != request.reference.providerGeneration) return api::Status::GenerationMismatch;
    auto* ref = RE::TESForm::GetFormByID<RE::TESObjectREFR>(request.reference.referenceFormId);
    if (!ref || ref->GetFormType() != RE::ENUM_FORM_ID::kREFR || ref->IsDeleted() || ref->IsDisabled() || !ref->Get3D())
        return api::Status::TargetUnavailable;
    auto* base = ref->GetObjectReference();
    if (!base || base->GetFormType() != RE::ENUM_FORM_ID::kMISC || base->formID != request.expectedBase ||
        (request.reference.referenceNativeHandle && ref->GetHandle().native_handle() != request.reference.referenceNativeHandle))
        return api::Status::TargetInvalid;
    auto* physics = PhysicsInteraction::s_instance.load(std::memory_order_acquire);
    return physics ? physics->watchObjectRest(owner, request, state) : api::Status::NotReady;
}
api::Status cancelObjectRest(api::OwnerToken owner, std::uint32_t referenceFormId) {
    auto* physics = PhysicsInteraction::s_instance.load(std::memory_order_acquire);
    if (!physics) return api::Status::NotReady;
    physics->cancelObjectRest(owner, referenceFormId);
    return api::Status::Ok;
}
}
