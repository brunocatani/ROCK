#include "physics-interaction/native/ReferenceInteraction.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/PhysicsLog.h"
#include <array>
#include <cstring>
#include <cmath>
#include <atomic>

namespace rock::reference_interaction
{
    namespace
    {
        using Flag = provider::RockProviderTargetDetailFlagV1;
        constexpr std::uint32_t bit(Flag f) { return static_cast<std::uint32_t>(f); }

        template <class Fn, std::size_t N>
        Fn entry(std::uintptr_t rva, const std::array<std::uint8_t, N>& bytes)
        {
            const auto text = REL::Module::get().segment(REL::Segment::text);
            const auto address = REL::Offset(rva).address();
            std::array<std::uint8_t, N> actual{};
            if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72 ||
                address < text.address() || address + N > text.address() + text.size() ||
                !native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(address), actual.data(), N) || actual != bytes) {
                ROCK_LOG_ERROR(Init, "Reference interaction native entry rejected: rva={:X}", rva);
                return nullptr;
            }
            return reinterpret_cast<Fn>(address);
        }

        // VR disassembly witnesses and entry bytes: 2026-09-11 surface/PA audit.
        // Each function validates once; missing evidence disables only its field.
        bool hasExtraList(RE::TESObjectREFR* ref)
        {
            void* extra = nullptr;
            // +100 independently witnessed in 1409BF5D0 and 141411530.
            const bool readable = ref && native_memory::tryReadField(ref, 0x100, extra) &&
                native_memory::pointerRangeLooksReadable(extra, 0x28);
            static std::atomic<bool> reported{false};
            if (ref && !readable && !reported.exchange(true)) {
                ROCK_LOG_ERROR(NativeScene, "Reference interaction unavailable: form={:08X} stage=extra-list reason=unreadable", ref->GetFormID());
            }
            return readable;
        }

        bool finite(const RE::NiTransform& t)
        {
            if (!std::isfinite(t.scale) || std::abs(t.scale) < 0.0001f ||
                !std::isfinite(t.translate.x) || !std::isfinite(t.translate.y) || !std::isfinite(t.translate.z)) return false;
            for (std::size_t r = 0; r < 3; ++r)
                for (std::size_t c = 0; c < 3; ++c)
                    if (!std::isfinite(t.rotate.entry[r][c])) return false;
            return true;
        }

        void copyTransform(const RE::NiTransform& t, provider::RockProviderTransform& out)
        {
            for (std::size_t r = 0; r < 3; ++r)
                for (std::size_t c = 0; c < 3; ++c) out.rotate[r * 3 + c] = t.rotate.entry[r][c];
            out.translate[0] = t.translate.x; out.translate[1] = t.translate.y; out.translate[2] = t.translate.z;
            out.scale = t.scale;
        }
    }

    RE::TESObjectREFR* resolveNode(RE::NiAVObject* node)
    {
        static const auto find = entry<RE::TESObjectREFR* (*)(RE::NiAVObject*)>(0x3F0890,
            std::array<std::uint8_t, 10>{0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10});
        return find && node && native_memory::pointerLooksReadable(node) ? find(node) : nullptr;
    }

    RE::TESObjectREFR* resolveBody(RE::hknpWorld* world, std::uint32_t bodyId)
    {
        if (!world) return nullptr;
        const auto body = havok_runtime::snapshotBody(world, RE::hknpBodyId{bodyId});
        if (!body.valid) return nullptr;
        auto* node = havok_runtime::getOwnerNodeFromBody(body.body);
        return resolveNode(node);
    }

    RE::TESObjectREFR* resolveQuery(const provider::RockProviderReferenceQueryV1& query)
    {
        auto* ref = RE::TESForm::GetFormByID<RE::TESObjectREFR>(query.referenceFormId);
        if (!ref || (query.referenceNativeHandle != 0 && ref->GetHandle().native_handle() != query.referenceNativeHandle)) return nullptr;
        return ref;
    }

    bool describe(RE::TESObjectREFR* ref, provider::RockProviderReferenceInteractionV1& out, const std::int32_t markerIndex)
    {
        out = {};
        if (!ref || !ref->GetObjectReference()) return false;
        out.referenceFormId = ref->GetFormID();
        out.referenceNativeHandle = ref->GetHandle().native_handle();
        out.baseFormId = ref->GetObjectReference()->GetFormID();
        out.baseFormType = static_cast<std::uint32_t>(ref->GetObjectReference()->GetFormType());
        out.flags = bit(Flag::Reference);
        static const auto open = entry<std::uint32_t (*)(RE::TESObjectREFR*)>(0x14A5E0,
            std::array<std::uint8_t, 9>{0x40,0x57,0x48,0x83,0xEC,0x20,0x48,0x8B,0xF9});
        static const auto blocked = entry<bool (*)(RE::TESObjectREFR*)>(0x3F49E0,
            std::array<std::uint8_t, 10>{0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20});
        static const auto used = entry<bool (*)(RE::TESObjectREFR*, std::int32_t, bool)>(0x464D90,
            std::array<std::uint8_t, 10>{0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10});
        if (!hasExtraList(ref)) return true;
        if (open) {
            const auto state = open(ref);
            if (state <= 4) { out.openState = static_cast<provider::RockProviderNativeOpenStateV1>(state); out.flags |= bit(Flag::OpenState); }
        }
        if (blocked) { out.activationBlocked = blocked(ref); out.flags |= bit(Flag::ActivationBlocked); }
        if (used && (out.baseFormType == 0x2A || out.baseFormType == 0x37)) {
            out.furnitureMarkerIndex = markerIndex;
            out.furnitureInUse = used(ref, markerIndex, true);
            out.furnitureInUseIncludingReservations = used(ref, markerIndex, false);
            out.flags |= bit(Flag::FurnitureUse);
        }
        return true;
    }

    bool isPowerArmorFurniture(RE::TESObjectREFR* ref, bool* outAvailable)
    {
        static const auto check = entry<bool (*)(RE::TESBoundObject*)>(0x2FC900,
            std::array<std::uint8_t, 10>{0x48,0x83,0xEC,0x28,0x8B,0x81,0x7C,0x01,0x00,0x00});
        auto* base = ref ? ref->GetObjectReference() : nullptr;
        if (outAvailable) *outAvailable = base &&
            (static_cast<std::uint32_t>(base->GetFormType()) != 0x2A || check != nullptr);
        return check && base && static_cast<std::uint32_t>(base->GetFormType()) == 0x2A && check(base);
    }

    const char* pointName(provider::RockProviderPowerArmorPointV1 point) noexcept
    {
        switch (point) {
        case provider::RockProviderPowerArmorPointV1::LeftArmorHand: return "LArm_Hand";
        case provider::RockProviderPowerArmorPointV1::RightArmorHand: return "RArm_Hand";
        default: return nullptr;
        }
    }

    RE::NiAVObject* findNamedNode(RE::NiAVObject* root, const char* name)
    {
        if (!root || !name) return nullptr;
        std::array<RE::NiAVObject*, 512> pending{};
        std::size_t count = 1, visited = 0;
        pending[0] = root;
        while (count && visited++ < pending.size()) {
            auto* object = pending[--count];
            if (!object || !native_memory::pointerLooksReadable(object)) continue;
            if (object->name.c_str() && std::strcmp(object->name.c_str(), name) == 0) return object;
            auto* node = object->IsNode();
            if (!node || node->children.size() > pending.size()) continue;
            for (auto& child : node->children) {
                if (count == pending.size()) return nullptr;
                if (child) pending[count++] = child.get();
            }
        }
        return nullptr;
    }

    bool pointTransform(RE::TESObjectREFR* frame, provider::RockProviderPowerArmorPointV1 point,
        RE::NiTransform& outWorld, RE::NiTransform* outLocal)
    {
        if (!frame || frame->IsDisabled() || frame->IsDeleted()) return false;
        auto* root = frame ? frame->Get3D() : nullptr;
        if (!root || !finite(root->world)) return false;
        auto* node = findNamedNode(root, pointName(point));
        if (!node || !finite(node->world)) return false;
        outWorld = node->world;
        if (outLocal) *outLocal = transform_math::composeTransforms(transform_math::invertTransform(root->world), node->world);
        return true;
    }

    bool describePowerArmor(RE::TESObjectREFR* ref, provider::RockProviderPowerArmorTargetV1& out, const std::int32_t markerIndex)
    {
        out = {};
        if (!describe(ref, out.touchedReference, markerIndex)) return false;
        RE::NiPointer<RE::TESObjectREFR> frameHold;
        RE::TESObjectREFR* frame = nullptr;
        bool furnitureClassificationAvailable = false;
        if (isPowerArmorFurniture(ref, &furnitureClassificationAvailable)) frame = ref;
        else if (ref->As<RE::Actor>() && hasExtraList(ref)) {
            static const auto inPA = entry<bool (*)(RE::TESObjectREFR*)>(0x9BF5D0,
                std::array<std::uint8_t, 10>{0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10});
            static const auto getFrame = entry<bool (*)(void*, std::uint32_t*)>(0x9A7A0,
                std::array<std::uint8_t, 11>{0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xDA,0xB2,0xBB});
            static const auto resolve = entry<bool (*)(std::uint32_t*, RE::NiPointer<RE::TESObjectREFR>&)>(0xAB60,
                std::array<std::uint8_t, 10>{0x48,0x89,0x5C,0x24,0x10,0x48,0x89,0x6C,0x24,0x18});
            if (!inPA) return true;
            out.flags |= bit(Flag::PowerArmorClassification);
            if (inPA(ref)) {
                out.flags |= bit(Flag::PowerArmorActor);
                out.actorFormId = ref->GetFormID();
                void* extra = nullptr; std::uint32_t handle = 0;
                if (getFrame && resolve && native_memory::tryReadField(ref, 0x100, extra) &&
                    getFrame(extra, &handle) && resolve(&handle, frameHold) && isPowerArmorFurniture(frameHold.get())) frame = frameHold.get();
            }
        }
        if (!ref->As<RE::Actor>() && furnitureClassificationAvailable) out.flags |= bit(Flag::PowerArmorClassification);
        if (!frame) return true;
        out.flags |= bit(Flag::PowerArmorClassification) | bit(Flag::PowerArmorFrame);
        describe(frame, out.frameReference, markerIndex);
        for (std::uint32_t i = 0; i < 2; ++i) {
            auto& pose = out.points[i];
            pose.point = static_cast<provider::RockProviderPowerArmorPointV1>(i + 1);
            RE::NiTransform world{}, local{};
            if (pointTransform(frame, pose.point, world, &local)) {
                pose.valid = 1; copyTransform(world, pose.world); copyTransform(local, pose.frameLocal);
            }
        }
        return true;
    }
}
