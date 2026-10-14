#include "api/ProviderRuntimeServices.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "physics-interaction/core/PhysicsInteraction.h"

// Collision ejection clearance: validates the reference and primary weapon,
// then hands the lease to PhysicsInteraction. Authorization is the caller's.
namespace rock::provider::runtime {
api::Status requestEjectionClearance(api::OwnerToken owner, const api::collision::EjectionClearanceRequest& request) {
    constexpr auto knownFlags = static_cast<std::uint32_t>(api::collision::EjectionClearanceFlag::IncludeHands);
    if (!request.reference.referenceFormId || !request.expectedBase || !request.weaponFormId || !request.weaponGenerationKey ||
        (request.flags & ~knownFlags)) return api::Status::InvalidArgument;
    Publication frame;
    if (!copyPublication(frame) || !frame.providerReady || frame.menuBlocking) return api::Status::NotReady;
    if (frame.worldGeneration != request.reference.worldGeneration || frame.skeletonGeneration != request.reference.skeletonGeneration ||
        frame.providerGeneration != request.reference.providerGeneration || frame.weapon.weaponGenerationKey != request.weaponGenerationKey ||
        frame.weaponFormId != request.weaponFormId) return api::Status::GenerationMismatch;
    auto* ref = RE::TESForm::GetFormByID<RE::TESObjectREFR>(request.reference.referenceFormId);
    if (!ref || ref->GetFormType() != RE::ENUM_FORM_ID::kREFR || ref->IsDeleted() || ref->IsDisabled()) return api::Status::TargetUnavailable;
    auto* base = ref->GetObjectReference();
    if (!base || base->formID != request.expectedBase ||
        (request.reference.referenceNativeHandle && ref->GetHandle().native_handle() != request.reference.referenceNativeHandle))
        return api::Status::TargetInvalid;
    auto* physics = PhysicsInteraction::s_instance.load(std::memory_order_acquire);
    return physics ? physics->requestEjectionContacts(owner, request) : api::Status::NotReady;
}
api::Status cancelEjectionClearance(api::OwnerToken owner, std::uint32_t referenceFormId) {
    auto* physics = PhysicsInteraction::s_instance.load(std::memory_order_acquire);
    if (!physics) return api::Status::NotReady;
    physics->cancelEjectionContacts(owner, referenceFormId);
    return api::Status::Ok;
}
}
