#include "api/StorageService.h"
#include <ROCK/Storage.h>
#include <ROCK/StorageFeedback.h>
#include "physics-interaction/hand/HandSelection.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include <algorithm>
#include <array>

namespace rock::storage_service
{
    namespace
    {
        namespace contract = api::storage;
        namespace collision = collision_suppression_registry;
        struct Claim {
            std::uint64_t owner{}, token{}, deadline{};
            std::uint32_t reference{}, weapon{};
            RE::ObjectRefHandle handle{};
            bool stored{}, pinned{};
        };
        struct Contact {
            std::uint64_t owner{}, grab{}, deadline{};
            api::ReferenceKey reference{};
        };
        // Access is confined to the established ROCK frame owner. No plugin
        // pointer or consumer callback is retained by these services.
        std::array<Claim, contract::kMaximumClaims> claims;
        std::array<Contact, 2> contacts;
        struct Feedback {
            api::OwnerToken owner{};
            api::storage_feedback::CandidateV1 request;
            std::uint64_t deadline{}, retry{};
            RE::NiPointer<RE::TESObjectREFR> reference;
            VatsSelectionHighlight highlight;
        };
        std::array<Feedback,2> feedback;
        float pulseClock{};
        float feedbackIntensity() { return .25f+.75f*(.5f+.5f*std::sin(pulseClock*31.41592654f)); }
        void stopFeedback(Feedback& value) {
            value.highlight.stop(); value.reference.reset(); value.owner=0; value.deadline=value.retry=0;
        }
        std::uint64_t nextToken{1};
        collision::SuppressionLeaseSet<128> bodies{collision::CollisionSuppressionOwner::ConsumerStorage};

        bool pin(std::uint32_t reference, std::uint32_t weapon, bool enabled) noexcept
        { (void)reference; (void)weapon; (void)enabled; return true; }

        api::Status authorize(api::OwnerToken owner, unsigned permission = 2)
        { return provider::runtime::authorize(owner, api::InterfaceId::Grab, permission, true); }
        PhysicsInteraction* interaction()
        { return PhysicsInteraction::s_instance.load(std::memory_order_acquire); }
        bool held(PhysicsInteraction& physics, std::uint32_t reference)
        {
            for (const auto* hand : {&physics.getRightHand(), &physics.getLeftHand()})
                if (hand->isHolding() && hand->getHeldRef() && hand->getHeldRef()->formID == reference) return true;
            return false;
        }
        api::Status validate(const contract::TargetV1* request, RE::NiPointer<RE::TESObjectREFR>& ref,
            provider::RockProviderFrameSnapshot& frame)
        {
            if (!request) return api::Status::InvalidArgument;
            if (request->size != sizeof(*request)) return api::Status::InvalidSize;
            if (request->version != 1) return api::Status::UnsupportedVersion;
            if (!request->reference.referenceFormId || unsigned(request->hand) > unsigned(api::Hand::Left)) return api::Status::InvalidArgument;
            if (!provider::runtime::apiGetFrameSnapshot(&frame) || !frame.providerReady || frame.menuBlocking) return api::Status::NotReady;
            if (frame.worldGeneration != request->reference.worldGeneration || frame.skeletonGeneration != request->reference.skeletonGeneration ||
                frame.providerGeneration != request->reference.providerGeneration) return api::Status::GenerationMismatch;
            auto* value = RE::TESForm::GetFormByID<RE::TESObjectREFR>(request->reference.referenceFormId);
            if (value && value->GetFormType() != RE::ENUM_FORM_ID::kREFR) return api::Status::TargetInvalid;
            if (!value || value->IsDeleted() || !value->GetObjectReference() ||
                (request->reference.referenceNativeHandle && value->GetHandle().native_handle() != request->reference.referenceNativeHandle))
                return api::Status::TargetUnavailable;
            ref.reset(value); return api::Status::Ok;
        }
        api::Status claim(api::OwnerToken owner, const contract::TargetV1* request, std::uint64_t* token) noexcept
        {
            if (token) *token = 0;
            if (!token) return api::Status::InvalidArgument;
            try {
                auto status = authorize(owner); if (status != api::Status::Ok) return status;
                auto* physics = interaction(); if (!physics) return api::Status::NotReady;
                RE::NiPointer<RE::TESObjectREFR> ref; provider::RockProviderFrameSnapshot frame;
                status = validate(request, ref, frame); if (status != api::Status::Ok) return status;
                if (request->hand != api::Hand::None) {
                    auto& hand = request->hand == api::Hand::Left ? physics->getLeftHand() : physics->getRightHand();
                    provider::RockProviderHandInteractionStateV1 published{};
                    status = static_cast<api::Status>(provider::runtime::apiGetHandInteractionStateV1(owner,
                        static_cast<provider::RockProviderHand>(request->hand), &published));
                    if (status != api::Status::Ok || published.gripSequence != request->gripSequence ||
                        !hand.isHolding() || hand.getHeldRef() != ref.get()) return api::Status::TargetUnavailable;
                } else if (held(*physics, ref->formID)) return api::Status::ObjectAlreadyOwned;
                for (auto& entry : claims) if (entry.reference == ref->formID) {
                    if (!entry.handle.get() || (!entry.stored && entry.deadline < frame.frameIndex) || authorize(entry.owner) != api::Status::Ok) entry = {};
                    else {
                        if (entry.owner != owner) return api::Status::OwnerConflict;
                        entry.deadline = frame.frameIndex + 120; *token = entry.token; return api::Status::Ok;
                    }
                }
                if (!nextToken || nextToken == UINT64_MAX) return api::Status::CapacityFull;
                for (auto& entry : claims) if (!entry.token) {
                    entry = {owner, nextToken++, frame.frameIndex + 120, ref->formID, 0, ref->GetHandle()};
                    *token = entry.token; return api::Status::Ok;
                }
                return api::Status::CapacityFull;
            } catch (...) { return api::Status::InternalError; }
        }
        api::Status commit(api::OwnerToken owner, std::uint64_t token) noexcept
        {
            try {
                const auto status = authorize(owner); if (status != api::Status::Ok) return status;
                auto* physics = interaction(); if (!physics || !physics->isProviderReady()) return api::Status::NotReady;
                for (auto& entry : claims) if (entry.token == token && entry.owner == owner && token) {
                    if (entry.stored) return api::Status::Ok;
                    if (provider::runtime::currentGameFrameIndex() > entry.deadline) return api::Status::RequestNotFound;
                    const auto ref = entry.handle.get();
                    if (!ref || ref->IsDeleted()) return api::Status::TargetUnavailable;
                    if (held(*physics, entry.reference)) return api::Status::ObjectAlreadyOwned;
                    if (physics->storageHandoffPending(entry.reference)) return api::Status::Busy;
                    auto* form = ref->GetObjectReference(); if (!form) return api::Status::TargetUnavailable;
                    entry.weapon = form->As<RE::TESObjectWEAP>() ? form->formID : 0;
                    if (entry.weapon && !pin(entry.reference, entry.weapon, true)) return api::Status::CapacityFull;
                    entry.pinned = entry.weapon != 0; entry.stored = true;
                    return api::Status::Ok;
                }
                return api::Status::RequestNotFound;
            } catch (...) { return api::Status::InternalError; }
        }
        api::Status release(api::OwnerToken owner, std::uint64_t token) noexcept
        {
            try {
                const auto status = authorize(owner, 0); if (status != api::Status::Ok) return status;
                for (auto& entry : claims) if (token && entry.token == token && entry.owner == owner) {
                    if (entry.pinned && !pin(entry.reference, entry.weapon, false)) return api::Status::Busy;
                    entry = {}; return api::Status::Ok;
                }
                return api::Status::RequestNotFound;
            } catch (...) { return api::Status::InternalError; }
        }
        api::Status claimant(api::OwnerToken owner, std::uint32_t reference, api::OwnerToken* out) noexcept
        {
            if (out) *out = 0;
            if (!out || !reference) return api::Status::InvalidArgument;
            try {
                const auto status = authorize(owner, 1); if (status != api::Status::Ok) return status;
                for (const auto& entry : claims) if (entry.reference == reference && entry.handle.get() &&
                    (entry.stored || entry.deadline >= provider::runtime::currentGameFrameIndex()) &&
                    authorize(entry.owner) == api::Status::Ok) { *out = entry.owner; break; }
                return api::Status::Ok;
            } catch (...) { return api::Status::InternalError; }
        }
        api::Status suppressContacts(api::OwnerToken owner, const contract::TargetV1* request) noexcept
        {
            try {
                auto status = authorize(owner); if (status != api::Status::Ok) return status;
                auto* physics = interaction(); if (!physics) return api::Status::NotReady;
                RE::NiPointer<RE::TESObjectREFR> ref; provider::RockProviderFrameSnapshot frame;
                status = validate(request, ref, frame); if (status != api::Status::Ok) return status;
                if (request->hand == api::Hand::None) return api::Status::InvalidArgument;
                const unsigned index = request->hand == api::Hand::Left ? 1 : 0;
                auto& hand = index ? physics->getLeftHand() : physics->getRightHand();
                provider::RockProviderHandInteractionStateV1 published{};
                status = static_cast<api::Status>(provider::runtime::apiGetHandInteractionStateV1(owner,
                    static_cast<provider::RockProviderHand>(request->hand), &published));
                if (status != api::Status::Ok || published.gripSequence != request->gripSequence ||
                    !hand.isHolding() || hand.getHeldRef() != ref.get()) return api::Status::TargetUnavailable;
                if (hand.getHeldBodyIds().empty() || hand.getHeldBodyIds().size() > 64) return api::Status::CapacityFull;
                auto& contact = contacts[index];
                if (contact.owner && contact.owner != owner && contact.deadline >= frame.frameIndex) return api::Status::OwnerConflict;
                contact = {owner, hand.heldGrabIdentity(), frame.frameIndex + 3, request->reference};
                return api::Status::Ok;
            } catch (...) { return api::Status::InternalError; }
        }
        api::Status extractEquipped(api::OwnerToken owner, const contract::EquippedExtractionV1* request, std::uint32_t* reference) noexcept
        {
            if (reference) *reference = 0;
            if (!request || !reference) return api::Status::InvalidArgument;
            if (request->size != sizeof(*request)) return api::Status::InvalidSize;
            if (request->version != 1) return api::Status::UnsupportedVersion;
            if (request->reserved || !request->weapon || (request->hand != api::Hand::Left && request->hand != api::Hand::Right)) return api::Status::InvalidArgument;
            try {
                const auto status = authorize(owner); if (status != api::Status::Ok) return status;
                provider::RockProviderFrameSnapshot frame;
                if (!provider::runtime::apiGetFrameSnapshot(&frame) || !frame.providerReady || frame.menuBlocking) return api::Status::NotReady;
                if (frame.worldGeneration != request->worldGeneration || frame.skeletonGeneration != request->skeletonGeneration ||
                    frame.providerGeneration != request->providerGeneration) return api::Status::GenerationMismatch;
                auto* physics = interaction();
                return physics ? physics->extractEquippedForStorage(request->hand == api::Hand::Left, request->weapon, *reference) : api::Status::NotReady;
            } catch (...) { return api::Status::InternalError; }
        }
        api::Status draw(api::OwnerToken owner, const api::grab::ForceGrabRequestV1* request, std::uint64_t* command) noexcept
        {
            if (command) *command = 0;
            if (!request || !command) return api::Status::InvalidArgument;
            if (request->size != sizeof(*request)) return api::Status::InvalidSize;
            if (request->version != 1) return api::Status::UnsupportedVersion;
            try {
                const auto access = authorize(owner); if (access != api::Status::Ok) return access;
                
                provider::RockProviderForceGrabRequestV1 native{};
                native.hand = static_cast<provider::RockProviderHand>(request->hand);
                native.flags = request->flags; native.targetFormId = request->targetFormId; native.targetBodyId = request->targetBodyId;
                native.worldGeneration = request->worldGeneration; native.skeletonGeneration = request->skeletonGeneration;
                native.providerGeneration = request->providerGeneration; native.maxDistanceGame = request->maxDistanceGame;
                std::copy_n(request->preferredGrabPointGame, 3, native.preferredGrabPointGame);
                std::copy_n(request->reserved, 5, native.reserved);
                return static_cast<api::Status>(provider::runtime::apiRequestForceGrabV1(owner, &native, command));
            } catch (...) { return api::Status::InternalError; }
        }
        RE::NiAVObject* feedbackNode(Feedback& value, PhysicsInteraction& physics) {
            const auto& request=value.request;
            if (!value.reference || value.reference->IsDeleted()) return nullptr;
            if (!request.reference) return physics.storageEquippedVisual(request.hand==api::Hand::Left,request.expectedBase);
            auto& hand=request.hand==api::Hand::Left?physics.getLeftHand():physics.getRightHand();
            return hand.isHolding() && hand.getHeldRef()==value.reference.get() ? value.reference->Get3D() : nullptr;
        }
        api::Status setCandidate(api::OwnerToken owner,const api::storage_feedback::CandidateV1* request) noexcept {
            if (!request) return api::Status::InvalidArgument;
            if (request->size!=sizeof(*request)) return api::Status::InvalidSize;
            if (request->version!=1) return api::Status::UnsupportedVersion;
            if ((request->hand!=api::Hand::Left && request->hand!=api::Hand::Right) || !request->expectedBase) return api::Status::InvalidArgument;
            try {
                const auto access=authorize(owner); if(access!=api::Status::Ok) return access;
                auto* physics=interaction(); provider::RockProviderFrameSnapshot frame;
                if(!physics || !provider::runtime::apiGetFrameSnapshot(&frame) || !frame.providerReady || frame.menuBlocking) return api::Status::NotReady;
                if(request->worldGeneration!=frame.worldGeneration || request->skeletonGeneration!=frame.skeletonGeneration || request->providerGeneration!=frame.providerGeneration) return api::Status::GenerationMismatch;
                provider::RockProviderHandInteractionStateV1 hand{};
                if(provider::runtime::apiGetHandInteractionStateV1(owner,static_cast<provider::RockProviderHand>(request->hand),&hand)!=provider::RockProviderResultV1::Ok ||
                    hand.gripSequence!=request->gripSequence) return api::Status::TargetUnavailable;
                auto& value=feedback[request->hand==api::Hand::Left?1:0];
                if(value.owner && value.owner!=owner && value.deadline>=frame.frameIndex) return api::Status::OwnerConflict;
                if(value.owner!=owner || value.request.reference!=request->reference || value.request.expectedBase!=request->expectedBase ||
                    value.request.gripSequence!=request->gripSequence) stopFeedback(value);
                value.owner=owner; value.request=*request; value.deadline=frame.frameIndex+3;
                auto* ref=request->reference ? RE::TESForm::GetFormByID<RE::TESObjectREFR>(request->reference) : RE::PlayerCharacter::GetSingleton();
                if(request->reference && (!ref || !ref->GetObjectReference() || ref->GetObjectReference()->formID!=request->expectedBase)) { stopFeedback(value); return api::Status::TargetInvalid; }
                value.reference.reset(ref);
                auto* node=feedbackNode(value,*physics);
                if(!node) { stopFeedback(value); return api::Status::TargetUnavailable; }
                if(!VatsSelectionHighlight::stashAvailable()) { stopFeedback(value); return api::Status::Busy; }
                if(frame.frameIndex<value.retry) return api::Status::NotReady;
                if(!value.highlight.refreshStash(ref,node,feedbackIntensity())) {
                    value.retry=frame.frameIndex+90;
                    ROCK_LOG_SAMPLE_WARN(Hand,1000,"Held stash feedback has no usable VATS target: ref={:08X} base={:08X}",request->reference,request->expectedBase);
                    return api::Status::NotReady;
                }
                return api::Status::Ok;
            } catch(...) { return api::Status::InternalError; }
        }
        api::Status clearCandidate(api::OwnerToken owner,api::Hand hand) noexcept {
            if(hand!=api::Hand::Left && hand!=api::Hand::Right) return api::Status::InvalidArgument;
            try {
                const auto access=authorize(owner,0); if(access!=api::Status::Ok) return access;
                auto& value=feedback[hand==api::Hand::Left?1:0];
                if(value.owner && value.owner!=owner) return api::Status::OwnerConflict;
                stopFeedback(value); return api::Status::Ok;
            } catch(...) { return api::Status::InternalError; }
        }
        api::Status clearContacts(api::OwnerToken owner) noexcept
        {
            try {
                const auto status = authorize(owner, 0); if (status != api::Status::Ok) return status;
                for (auto& contact : contacts) if (contact.owner == owner) contact = {};
                return api::Status::Ok;
            } catch (...) { return api::Status::InternalError; }
        }
    }
    bool allowsGrab(std::uint32_t reference, std::uint64_t owner)
    {
        for (const auto& entry : claims) if (entry.reference == reference && entry.owner != owner &&
            entry.handle.get() && (entry.stored || entry.deadline >= provider::runtime::currentGameFrameIndex()) &&
            authorize(entry.owner) == api::Status::Ok) return false;
        return true;
    }
    void update(PhysicsInteraction& physics, const PhysicsFrameContext& frame)
    {
        const auto sample = provider::runtime::sample();
        pulseClock=std::fmod(pulseClock+std::clamp(frame.deltaSeconds,0.f,.1f),1.f);
        const float intensity=feedbackIntensity();
        for(auto& value:feedback) if(value.owner) {
            if(!frame.worldReady || frame.menuBlocked || value.deadline<sample.frameIndex || authorize(value.owner)!=api::Status::Ok ||
                value.request.worldGeneration!=sample.worldGeneration || value.request.skeletonGeneration!=sample.skeletonGeneration ||
                value.request.providerGeneration!=sample.providerGeneration) { stopFeedback(value); continue; }
            auto* node=feedbackNode(value,physics);
            if(!node || (value.highlight.isActive() && !value.highlight.refreshStash(value.reference.get(),node,intensity))) stopFeedback(value);
        }
        std::array<std::uint32_t,128> wanted{}; unsigned count{};
        for (unsigned i = 0; i < contacts.size(); ++i) {
            auto& contact = contacts[i]; if (!contact.owner) continue;
            auto& hand = i ? physics.getLeftHand() : physics.getRightHand();
            if (!frame.worldReady || frame.menuBlocked || contact.deadline < sample.frameIndex || authorize(contact.owner) != api::Status::Ok ||
                contact.reference.worldGeneration != sample.worldGeneration || contact.reference.skeletonGeneration != sample.skeletonGeneration ||
                contact.reference.providerGeneration != sample.providerGeneration || !hand.isHolding() || hand.heldGrabIdentity() != contact.grab ||
                !hand.getHeldRef() || hand.getHeldRef()->formID != contact.reference.referenceFormId) { contact = {}; continue; }
            for (const auto body : hand.getHeldBodyIds()) if (count < wanted.size() && std::find(wanted.begin(), wanted.begin()+count, body) == wanted.begin()+count) wanted[count++] = body;
        }
        if (frame.worldReady) {
            bodies.releaseWhere(frame.hknpWorld, "consumer-storage-expired",
                [&](std::uint32_t body) { return std::find(wanted.begin(), wanted.begin()+count, body) == wanted.begin()+count; },
                [](auto, const auto&) {});
            for (unsigned i = 0; i < count; ++i) {
                const auto result = bodies.acquire(frame.hknpWorld, wanted[i], "consumer-storage");
                if (!result.valid) ROCK_LOG_SAMPLE_WARN(Hand, 1000, "Storage contact suppression unavailable for body {}", wanted[i]);
            }
        }
        for (auto& entry : claims) if (entry.token && (!entry.handle.get() || authorize(entry.owner) != api::Status::Ok ||
            (!entry.stored && entry.deadline < sample.frameIndex))) entry = {};
    }
    void shutdown(RE::hknpWorld* validWorld)
    {
        for(auto& value:feedback) stopFeedback(value);
        contacts = {};
        if (validWorld) (void)bodies.releaseAll(validWorld, "consumer-storage-shutdown", [](auto, const auto&) {});
        else bodies.clearTracking();
        // Committed item pins belong to the save, independently of transient
        // scene/owner leases. The restoring consumer reclaims exact references.
        for (auto& entry : claims) if (!entry.stored) entry = {};
    }
}
extern "C" __declspec(dllexport) const rock::api::storage_feedback::ApiV1* ROCK_CALL ROCK_StorageFeedback_QueryV1(
    std::uint32_t major,std::uint32_t bytes) noexcept
{
    static const rock::api::storage_feedback::ApiV1 table{sizeof(table),1,rock::storage_service::setCandidate,rock::storage_service::clearCandidate};
    return major==1 && bytes==sizeof(table)?&table:nullptr;
}
extern "C" __declspec(dllexport) const rock::api::storage::ApiV1* ROCK_CALL ROCK_Storage_QueryV1(
    std::uint32_t major, std::uint32_t bytes) noexcept
{
    using namespace rock;
    static const api::storage::ApiV1 api{sizeof(api::storage::ApiV1),1,
        storage_service::claim, storage_service::commit, storage_service::release,
        storage_service::claimant, storage_service::suppressContacts, storage_service::clearContacts, storage_service::extractEquipped, storage_service::draw};
    return major == 1 && bytes == sizeof(api) ? &api : nullptr;
}
