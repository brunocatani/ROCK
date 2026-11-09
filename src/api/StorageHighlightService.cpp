#include "api/StorageHighlightService.h"

#include "api/NativeInteractionState.h"
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/hand/HandSelection.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/scene/NativeNiNodeFactory.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/BSGeometry.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiCloningProcess.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace rock::storage_highlight
{
    namespace
    {
        namespace contract = api::storage;
        using native_interaction_service::detail::finite;
        using native_interaction_service::detail::fromApi;

        // A ghost copies at most this many render shapes of the source model.
        constexpr std::size_t kMaxGhostShapes = 32;
        constexpr std::size_t kMaxGhostWalk = 128;

        // FO4VR 1.2.72 NiObject clone entry (CreateClone + ProcessClone), the
        // call LooseMolotovVisual and the secondary bone bindings verify.
        constexpr std::uintptr_t kCloneRva = 0x1C13FF0;
        constexpr auto kClonePrologue = std::to_array<std::uint8_t>(
            { 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57, 0x48, 0x83, 0xEC, 0x20 });
        using Clone = RE::NiObject* (*)(RE::NiObject*, RE::NiCloningProcess*);

        struct Slot
        {
            api::OwnerToken owner{};
            contract::Highlight request{};
            std::uint64_t deadline{}, retry{};
            RE::NiPointer<RE::TESObjectREFR> reference;
            RE::NiPointer<RE::NiNode> ghost;
            RE::NiPointer<RE::NiNode> ghostParent;
            VatsSelectionHighlight highlight;
        };

        std::array<Slot, contract::kMaxHighlights> slots;
        float pulseClock{};
        int cloneVerified{};  // 0 unknown, 1 verified, -1 refused

        bool pulsing(const Slot& slot)
        {
            return (slot.request.flags & static_cast<std::uint32_t>(contract::HighlightFlag::Pulse)) != 0;
        }

        bool ghosted(const contract::Highlight& request)
        {
            return (request.flags & static_cast<std::uint32_t>(contract::HighlightFlag::Ghost)) != 0;
        }

        // One shared glow: it blinks (about 1.2 Hz) while any highlight pulses.
        float intensity()
        {
            const bool blink = std::any_of(slots.begin(), slots.end(), [](const Slot& s) { return s.owner && pulsing(s); });
            return blink ? .2f + .8f * (.5f + .5f * std::sin(pulseClock * 7.5398f)) : 1.f;
        }

        Clone cloneEntry()
        {
            if (!cloneVerified) {
                std::array<std::uint8_t, kClonePrologue.size()> live{};
                cloneVerified = native_memory::guardedCopyFromMemory(
                                    reinterpret_cast<const void*>(REL::Module::get().base() + kCloneRva), live.data(),
                                    live.size()) &&
                        live == kClonePrologue
                    ? 1
                    : -1;
                if (cloneVerified < 0)
                    ROCK_LOG_ERROR(Hand, "Storage highlight ghosts unavailable: clone entry does not match 1.2.72");
            }
            return cloneVerified > 0 ? reinterpret_cast<Clone>(REL::Module::get().base() + kCloneRva) : nullptr;
        }

        void detachGhost(Slot& slot)
        {
            if (slot.ghost && slot.ghostParent && slot.ghost->parent == slot.ghostParent.get()) {
                RE::NiPointer<RE::NiAVObject> detached;
                slot.ghostParent->DetachChild(slot.ghost.get(), detached);
            }
            slot.ghost.reset();
            slot.ghostParent.reset();
        }

        void stop(Slot& slot)
        {
            slot.highlight.stop();
            detachGhost(slot);
            slot.reference.reset();
            slot.owner = 0;
            slot.request = {};
            slot.deadline = slot.retry = 0;
        }

        /*
         * A visual-only copy of the reference's model: a fresh node holding a
         * clone of every visible, unskinned render shape, each placed at its
         * pose relative to the model root. No node of the source (and no
         * collision object) is cloned, so the copy never enters a physics
         * world and shares no Havok system with the original.
         */
        RE::NiPointer<RE::NiNode> buildGhost(RE::NiAVObject* source, std::uint32_t referenceId)
        {
            auto clone = cloneEntry();
            if (!clone || !source)
                return {};
            struct Pending
            {
                RE::NiAVObject* object;
                RE::NiTransform relative;
            };
            std::array<Pending, kMaxGhostWalk> pending{};
            std::array<RE::NiPointer<RE::BSTriShape>, kMaxGhostShapes> shapes{};
            std::size_t count = 1, shapeCount = 0;
            pending[0] = { source, RE::NiTransform{} };
            RE::NiCloningProcess cloning{};
            cloning.copyType = RE::NiCloningProcess::CopyType::kCopyExact;
            cloning.scale = { 1.0f, 1.0f, 1.0f };
            for (std::size_t index = 0; index < count; ++index) {
                auto [object, relative] = pending[index];
                if (!object || object->GetAppCulled())
                    continue;
                if (auto* shape = object->IsTriShape()) {
                    if (shape->skinInstance || shape->collisionObject || shapeCount == shapes.size())
                        continue;
                    RE::NiPointer<RE::NiObject> copy{ clone(shape, &cloning) };
                    auto* copied = copy ? copy->IsTriShape() : nullptr;
                    if (!copied || copied == shape || copied->parent)
                        continue;
                    copied->local = relative;
                    shapes[shapeCount++].reset(copied);
                    continue;
                }
                if (auto* node = object->IsNode()) {
                    for (const auto& child : node->children) {
                        if (!child || count == pending.size())
                            continue;
                        pending[count++] = { child.get(),
                            transform_math::composeTransforms(relative, child->local) };
                    }
                }
            }
            if (!shapeCount) {
                ROCK_LOG_SAMPLE_WARN(Hand, 2000, "Storage highlight ghost skipped: ref={:08X} has no copyable shape",
                    referenceId);
                return {};
            }
            auto ghost = native_scene::createEngineNiNode(static_cast<std::uint16_t>(shapeCount));
            if (!ghost)
                return {};
            ghost->name = RE::BSFixedString("ROCK_HighlightGhost");
            for (std::size_t index = 0; index < shapeCount; ++index)
                ghost->AttachChild(shapes[index].get(), true);
            ROCK_LOG_INFO(Hand, "Storage highlight ghost built: ref={:08X} shapes={}", referenceId, shapeCount);
            return ghost;
        }

        // Keeps the ghost attached beneath the world root at ghostWorld.
        bool placeGhost(Slot& slot)
        {
            auto* parent = fo4vr::getWorldRootNode();
            if (!parent)
                return false;
            if (!slot.ghost) {
                slot.ghost = buildGhost(slot.reference->Get3D(), slot.request.reference.referenceFormId);
                if (!slot.ghost)
                    return false;
            }
            if (slot.ghost->parent != parent) {
                if (auto* old = slot.ghost->parent) {
                    RE::NiPointer<RE::NiAVObject> detached;
                    old->DetachChild(slot.ghost.get(), detached);
                }
                parent->AttachChild(slot.ghost.get(), true);
                slot.ghostParent.reset(parent);
            }
            slot.ghost->local = transform_math::composeTransforms(transform_math::invertTransform(parent->world),
                fromApi(slot.request.ghostWorld));
            fo4vr::updateDown(slot.ghost.get(), true);
            return true;
        }

        RE::NiAVObject* target(Slot& slot)
        {
            if (!slot.reference || slot.reference->IsDeleted() || slot.reference->IsDisabled() ||
                !slot.reference->Get3D())
                return nullptr;
            if (!ghosted(slot.request))
                return slot.reference->Get3D();
            return placeGhost(slot) ? slot.ghost.get() : nullptr;
        }

        api::Status validate(const contract::Highlight* request)
        {
            if (!request)
                return api::Status::InvalidArgument;
            if (request->size != sizeof(*request))
                return api::Status::InvalidSize;
            if (request->version != 1)
                return api::Status::UnsupportedVersion;
            if (!request->highlightId || !request->reference.referenceFormId || !request->expectedBase ||
                (request->flags & ~contract::kSupportedHighlightFlags) || request->leaseFrames < 1 ||
                request->leaseFrames > contract::kMaxLeaseFrames || (ghosted(*request) && !finite(request->ghostWorld)))
                return api::Status::InvalidArgument;
            return api::Status::Ok;
        }
    }

    api::Status setHighlight(api::OwnerToken owner, const contract::Highlight* request) noexcept
    {
        const auto shape = validate(request);
        if (shape != api::Status::Ok)
            return shape;
        try {
            const auto access = provider::runtime::authorize(owner, api::InterfaceId::Storage, 2, true);
            if (access != api::Status::Ok)
                return access;
            provider::Publication frame;
            if (!provider::runtime::copyPublication(frame) || !frame.providerReady || frame.menuBlocking)
                return api::Status::NotReady;
            if (frame.worldGeneration != request->reference.worldGeneration ||
                frame.skeletonGeneration != request->reference.skeletonGeneration ||
                frame.providerGeneration != request->reference.providerGeneration)
                return api::Status::GenerationMismatch;
            auto* ref = RE::TESForm::GetFormByID<RE::TESObjectREFR>(request->reference.referenceFormId);
            if (!ref || ref->GetFormType() != RE::ENUM_FORM_ID::kREFR || ref->IsDeleted() || ref->IsDisabled() ||
                !ref->Get3D())
                return api::Status::TargetUnavailable;
            auto* base = ref->GetObjectReference();
            if (!base || base->formID != request->expectedBase ||
                (request->reference.referenceNativeHandle &&
                    ref->GetHandle().native_handle() != request->reference.referenceNativeHandle))
                return api::Status::TargetInvalid;

            auto slot = std::find_if(slots.begin(), slots.end(),
                [&](const Slot& s) { return s.owner == owner && s.request.highlightId == request->highlightId; });
            if (slot == slots.end()) {
                slot = std::find_if(slots.begin(), slots.end(),
                    [&](const Slot& s) { return !s.owner || s.deadline < frame.frameIndex; });
                if (slot == slots.end())
                    return api::Status::CapacityFull;
                stop(*slot);
            } else if (slot->request.reference.referenceFormId != request->reference.referenceFormId ||
                       slot->request.expectedBase != request->expectedBase || slot->request.flags != request->flags) {
                stop(*slot);
            }
            slot->owner = owner;
            slot->request = *request;
            slot->deadline = frame.frameIndex + request->leaseFrames;
            slot->reference.reset(ref);
            if (frame.frameIndex < slot->retry)
                return api::Status::NotReady;
            auto* node = target(*slot);
            if (!node || !slot->highlight.refreshWorld(ref, node, intensity())) {
                slot->retry = frame.frameIndex + 90;
                ROCK_LOG_SAMPLE_WARN(Hand, 2000, "Storage highlight has no usable VATS target: ref={:08X} ghost={}",
                    request->reference.referenceFormId, ghosted(*request));
                return api::Status::NotReady;
            }
            return api::Status::Ok;
        } catch (...) {
            return api::Status::InternalError;
        }
    }

    api::Status clearHighlight(api::OwnerToken owner, std::uint64_t highlightId) noexcept
    {
        if (!highlightId)
            return api::Status::InvalidArgument;
        try {
            const auto access = provider::runtime::authorize(owner, api::InterfaceId::Storage, 0, true);
            if (access != api::Status::Ok)
                return access;
            for (auto& slot : slots)
                if (slot.owner == owner && slot.request.highlightId == highlightId)
                    stop(slot);
            return api::Status::Ok;
        } catch (...) {
            return api::Status::InternalError;
        }
    }

    void update(PhysicsInteraction&, const PhysicsFrameContext& frame)
    {
        if (std::none_of(slots.begin(), slots.end(), [](const Slot& s) { return s.owner != 0; }))
            return;
        const auto sample = provider::runtime::sample();
        pulseClock = std::fmod(pulseClock + std::clamp(frame.deltaSeconds, 0.f, .1f), 100.f);
        const float glow = intensity();
        for (auto& slot : slots) {
            if (!slot.owner)
                continue;
            if (!frame.worldReady || frame.menuBlocked || slot.deadline < sample.frameIndex ||
                provider::runtime::authorize(slot.owner, api::InterfaceId::Storage, 2, true) != api::Status::Ok ||
                slot.request.reference.worldGeneration != sample.worldGeneration ||
                slot.request.reference.skeletonGeneration != sample.skeletonGeneration ||
                slot.request.reference.providerGeneration != sample.providerGeneration) {
                stop(slot);
                continue;
            }
            if (!slot.highlight.isActive())
                continue;
            auto* node = target(slot);
            if (!node || !slot.highlight.refreshWorld(slot.reference.get(), node, glow))
                stop(slot);
        }
    }

    void shutdown()
    {
        for (auto& slot : slots)
            stop(slot);
    }
}
