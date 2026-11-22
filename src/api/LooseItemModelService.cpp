#include "LooseItemModelService.h"
#include <REL/Relocation.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include "RE/Bethesda/BGSMod.h"
#include "RE/Bethesda/BSGeometry.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiCloningProcess.h"
#include "RE/NetImmerse/NiNode.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/NativeCodeBytes.h"
#include "physics-interaction/weapon/equip/EquippedWeaponVisualState.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock::loose_item_model
{
    namespace
    {
        // The presented model's root, so a second attach of the same tree and the
        // frame thread's checks recognize it.
        constexpr const char* kModelNodeName = "ROCK_LooseItemModel";
        constexpr std::size_t kMaxTreeNodes = 1024;
        constexpr std::size_t kMaxPresented = 256;

        struct Entry
        {
            api::OwnerToken owner{};
            std::uint32_t modelFormId{};
            std::string path;
        };

        struct Presented
        {
            std::uint32_t reference{};
            std::uint32_t base{};
            std::uintptr_t root{}; // Comparison only; never dereferenced.
        };

        // Never destroyed: the process may exit with entries registered, and
        // nothing here owns engine objects. present() runs on loading threads.
        std::mutex& mutex = *new std::mutex;
        std::unordered_map<std::uint32_t, Entry>& entries = *new std::unordered_map<std::uint32_t, Entry>;
        std::array<Presented, kMaxPresented>& presented = *new std::array<Presented, kMaxPresented>{};
        std::size_t nextPresented{};

        using Clone = RE::NiObject* (*)(RE::NiObject*, RE::NiCloningProcess*);

        // FO4VR 1.2.72: 141D0DEE0 is the BSModelDB demand (byte-checked in
        // LooseMolotovVisual.cpp and used by weapon OMOD templates), 141C13FF0
        // CreateClone + ProcessClone (raw witnesses in LooseMolotovVisual.cpp).
        bool nativeCallsAvailable()
        {
            static const bool valid =
                native_code::bytesMatch(0x1D0DEE0,
                    std::to_array<std::uint8_t>({ 0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x30, 0x48,
                        0x8B, 0xDA, 0x48, 0x8D, 0x54, 0x24, 0x20, 0x48, 0xC7, 0x44, 0x24, 0x20, 0, 0, 0, 0 })) &&
                native_code::bytesMatch(0x1C13FF0,
                    std::to_array<std::uint8_t>({ 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57, 0x48,
                        0x83, 0xEC, 0x20, 0x48, 0x8B, 0x01, 0x48, 0x8B, 0xFA, 0x48, 0x8B, 0xF1, 0xFF, 0x90, 0xE0, 0, 0,
                        0 }));
            return valid;
        }

        std::string resourcePath(const char* model)
        {
            const std::string_view value{ model };
            const auto starts = [&](std::string_view prefix) {
                return value.size() >= prefix.size() &&
                    std::equal(prefix.begin(), prefix.end(), value.begin(),
                        [](char a, char b) { return std::tolower(static_cast<unsigned char>(a)) ==
                                                    std::tolower(static_cast<unsigned char>(b)); });
            };
            if (starts("Data\\") || starts("Data/"))
                return std::string(value);
            if (starts("Meshes\\") || starts("Meshes/"))
                return "Data/" + std::string(value);
            return "Data/Meshes/" + std::string(value);
        }

        // The object mod's own model path, or empty.
        std::string modelPathOf(const RE::TESForm* form)
        {
            const auto* mod = form ? form->As<RE::BGSMod::Attachment::Mod>() : nullptr;
            if (!mod)
                return {};
            const auto* model = static_cast<const RE::TESModel*>(mod)->model.c_str();
            return model && *model ? std::string(model) : std::string{};
        }

        // Every node of a detached tree, bounded; empty when the tree is larger.
        std::vector<RE::NiAVObject*> collect(RE::NiAVObject* root)
        {
            std::vector<RE::NiAVObject*> nodes{ root };
            for (std::size_t index = 0; index < nodes.size(); ++index)
                if (auto* node = nodes[index]->IsNode())
                    for (const auto& child : node->children)
                        if (child) {
                            if (nodes.size() == kMaxTreeNodes)
                                return {};
                            nodes.push_back(child.get());
                        }
            return nodes;
        }

        // Centre of the model's geometry in its root's frame, from the shapes'
        // bounds stored in the NIF; the tree is not in a scene yet, so world
        // bounds do not exist.
        bool geometryCentre(RE::NiAVObject* root, RE::NiPoint3& centre)
        {
            RE::NiPoint3 lo{ FLT_MAX, FLT_MAX, FLT_MAX }, hi{ -FLT_MAX, -FLT_MAX, -FLT_MAX };
            bool any = false;
            for (auto* object : collect(root)) {
                auto* geometry = object->IsGeometry();
                if (!geometry || (geometry->flags.flags & 1))
                    continue;
                // The shape's frame relative to the model root.
                RE::NiTransform toRoot = geometry->local;
                for (auto* parent = geometry->parent; parent && parent != root; parent = parent->parent)
                    toRoot = transform_math::composeTransforms(parent->local, toRoot);
                const auto& bound = geometry->modelBound;
                const auto c = transform_math::localPointToWorld(toRoot, bound.center);
                const float r = bound.fRadius * toRoot.scale;
                if (!std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.z) || !std::isfinite(r) || r < 0)
                    continue;
                lo = { (std::min)(lo.x, c.x - r), (std::min)(lo.y, c.y - r), (std::min)(lo.z, c.z - r) };
                hi = { (std::max)(hi.x, c.x + r), (std::max)(hi.y, c.y + r), (std::max)(hi.z, c.z + r) };
                any = true;
            }
            if (!any)
                return false;
            centre = { (lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f, (lo.z + hi.z) * 0.5f };
            return true;
        }

        RE::NiPointer<RE::NiNode> instantiate(const std::string& path)
        {
            RE::NiPointer<RE::NiNode> source;
            const auto resource = resourcePath(path.c_str());
            std::uint64_t flags[2]{ 0, 0x2D };
            std::uint64_t loaded = 0;
            if (f4vr::loadNif(reinterpret_cast<std::uint64_t>(resource.c_str()), reinterpret_cast<std::uint64_t>(&loaded),
                    reinterpret_cast<std::uint64_t>(&flags)) != 0 ||
                !loaded)
                return {};
            // The demand's output already holds a reference for the caller.
            source.reset(reinterpret_cast<RE::NiNode*>(loaded));
            RE::NiCloningProcess cloning{};
            cloning.copyType = RE::NiCloningProcess::CopyType::kCopyExact;
            cloning.scale = { 1.0f, 1.0f, 1.0f };
            const auto clone = reinterpret_cast<Clone>(REL::Offset(0x1C13FF0).address());
            RE::NiPointer<RE::NiObject> copy{ clone(source.get(), &cloning) };
            auto* node = copy ? copy->IsNode() : nullptr;
            if (!node || node == source.get() || node->parent)
                return {};
            return RE::NiPointer<RE::NiNode>{ node };
        }

        void remember(const Presented& value)
        {
            std::scoped_lock lock(mutex);
            for (auto& slot : presented)
                if (slot.reference == value.reference) {
                    slot = value;
                    return;
                }
            for (auto& slot : presented)
                if (!slot.reference) {
                    slot = value;
                    return;
                }
            // Full: the oldest entry gives way; it stays presented but loses its
            // generated collision until its next model load.
            presented[nextPresented++ % presented.size()] = value;
        }

        // Frame thread, under the mutex: the entry still describes the live 3D.
        bool current(const Presented& slot)
        {
            if (!slot.reference || !entries.contains(slot.base))
                return false;
            auto* reference = RE::TESForm::GetFormByID<RE::TESObjectREFR>(slot.reference);
            auto* base = reference ? reference->GetObjectReference() : nullptr;
            return base && base->GetFormID() == slot.base && !reference->IsDeleted() &&
                reinterpret_cast<std::uintptr_t>(reference->Get3D()) == slot.root;
        }
    }

    api::Status set(api::OwnerToken owner, std::span<const contract::LooseItemModel> models)
    {
        if (!owner || models.size() > contract::kMaxLooseItemModels)
            return models.size() > contract::kMaxLooseItemModels ? api::Status::CapacityFull
                                                                 : api::Status::InvalidArgument;
        std::unordered_map<std::uint32_t, Entry> next;
        next.reserve(models.size());
        for (const auto& model : models) {
            if (model.size != sizeof(model) || model.version != 1 || !model.baseFormId || !model.modelFormId)
                return api::Status::InvalidArgument;
            const auto* base = RE::TESForm::GetFormByID(model.baseFormId);
            auto path = modelPathOf(RE::TESForm::GetFormByID(model.modelFormId));
            if (!base || !base->Is(RE::ENUM_FORM_ID::kMISC) || path.empty())
                return api::Status::TargetInvalid;
            if (!next.try_emplace(model.baseFormId, Entry{ owner, model.modelFormId, std::move(path) }).second)
                return api::Status::InvalidArgument;
        }
        std::scoped_lock lock(mutex);
        for (const auto& [base, entry] : next)
            if (const auto found = entries.find(base); found != entries.end() && found->second.owner != owner)
                return api::Status::OwnerConflict;
        std::erase_if(entries, [&](const auto& item) { return item.second.owner == owner; });
        entries.merge(next);
        ROCK_LOG_INFO(Weapon, "Loose item models set: owner={:#x} models={} total={}", owner, models.size(),
            entries.size());
        return api::Status::Ok;
    }

    api::Status clear(api::OwnerToken owner)
    {
        std::scoped_lock lock(mutex);
        std::erase_if(entries, [&](const auto& item) { return item.second.owner == owner; });
        return api::Status::Ok;
    }

    void present(RE::TESObjectREFR* reference, RE::NiAVObject* model) noexcept
    {
        try {
            auto* base = reference ? reference->GetObjectReference() : nullptr;
            if (!base || !model)
                return;
            std::uint32_t modelFormId{};
            std::string path;
            {
                std::scoped_lock lock(mutex);
                const auto found = entries.find(base->GetFormID());
                if (found == entries.end())
                    return;
                modelFormId = found->second.modelFormId;
                path = found->second.path;
            }
            auto* root = model->IsNode();
            if (!root || model->parent)
                return;
            for (const auto& child : root->children)
                if (child && child->name == kModelNodeName)
                    return;
            if (!nativeCallsAvailable()) {
                ROCK_LOG_SAMPLE_WARN(Weapon, 60000,
                    "Loose item model unavailable: native model load or clone signature mismatch");
                return;
            }
            auto part = instantiate(path);
            const auto nodes = part ? collect(part.get()) : std::vector<RE::NiAVObject*>{};
            RE::NiPoint3 centre{};
            if (nodes.empty() || !geometryCentre(part.get(), centre)) {
                ROCK_LOG_SAMPLE_WARN(Weapon, 2000,
                    "Loose item model not shown ref={:08X} base={:08X} omod={:08X} model='{}': {}",
                    reference->GetFormID(), base->GetFormID(), modelFormId, path,
                    !part ? "model did not load" : "no geometry");
                return;
            }
            // The reference keeps its own collision bodies; the part's own
            // collision would add bodies that move apart from them.
            for (auto* node : nodes)
                node->collisionObject.reset();
            // The reference's own geometry stays in the tree, hidden: native
            // code that looks it up by name still finds it.
            for (const auto& child : root->children)
                if (child)
                    equipped_weapon_visual_state::setLocallyVisible(child.get(), false);
            part->name = RE::BSFixedString(kModelNodeName);
            part->local.MakeIdentity();
            part->local.translate = { -centre.x, -centre.y, -centre.z };
            root->AttachChild(part.get(), true);
            remember({ reference->GetFormID(), base->GetFormID(), reinterpret_cast<std::uintptr_t>(model) });
            ROCK_LOG_INFO(Weapon, "Loose item model shown ref={:08X} base={:08X} omod={:08X} model='{}' nodes={}",
                reference->GetFormID(), base->GetFormID(), modelFormId, path, nodes.size());
        } catch (...) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000, "Loose item model presentation failed");
        }
    }

    std::size_t references(std::span<std::uint32_t> output)
    {
        std::scoped_lock lock(mutex);
        std::size_t count{};
        for (auto& slot : presented) {
            if (!slot.reference)
                continue;
            if (!current(slot)) {
                slot = {};
                continue;
            }
            if (count < output.size())
                output[count++] = slot.reference;
        }
        return count;
    }

    bool presents(std::uint32_t reference)
    {
        if (!reference)
            return false;
        std::scoped_lock lock(mutex);
        return std::any_of(presented.begin(), presented.end(),
            [&](const Presented& slot) { return slot.reference == reference && current(slot); });
    }
}
