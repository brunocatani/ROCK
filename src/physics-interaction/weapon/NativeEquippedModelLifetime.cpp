#include "physics-interaction/weapon/NativeEquippedModelLifetime.h"
#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/weapon/NativeWeaponModelGeometry.h"
#include "physics-interaction/weapon/EquippedWeaponVisualState.h"
#include "physics-interaction/weapon/NativeModelAncestry.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include <REL/Relocation.h>
#include <Windows.h>
#include <intrin.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <utility>
#include <vector>

namespace rock::native_equipped_model_lifetime
{
    namespace
    {
        using Retire = void (*)(void*, void*, bool);
        using FindConnection = std::uint32_t (*)(RE::NiAVObject*, void*);
        using QueueDetach = void (*)(void*, const std::uint32_t*, RE::NiAVObject*, bool);
        using Detach = void (*)(const std::uint32_t*, RE::NiAVObject*, bool);
        using Cull = bool (*)(void*, RE::Actor*, bool, std::uint32_t);
        Retire s_retire = nullptr;
        FindConnection s_findConnection = nullptr;
        QueueDetach s_queueDetach = nullptr;
        Detach s_detach = nullptr;
        Cull s_cull = nullptr;
        bool s_installed = false; // Load only. Partially installed hooks forward unchanged.
        std::atomic<std::uint64_t> s_revision{0}; // UI/native owner -> FRIK frame owner.
        std::atomic<std::uint32_t> s_cullLogs{0}; // Native callbacks may occur on different threads.

        using native_model_ancestry::Relation;
        using native_model_ancestry::within;

        struct ProtectedModel
        {
            RE::NiPointer<RE::NiAVObject> root;
            RE::NiPointer<RE::NiAVObject> boundary;
            std::uint32_t form = 0, shapesBefore = 0;
        };
        struct Retirement
        {
            // Strong references exist only for one synchronous native retirement.
            // Nothing from this context survives a queued job or a frame.
            std::array<ProtectedModel, 44> models{};
            std::size_t count = 0;
            std::uint32_t pruned = 0, rejected = 0, unreadable = 0;
            // Ownership sets, built once before the native call. The native
            // connection search visits every node of the biped tree; walking
            // each node's parents with guarded (VirtualQuery) reads per
            // protected model cost ~190 ms per replacement equip (runtime
            // 12 Oct 22:34:33 and 22:35:43). Sorted for binary search.
            // inside:    every node in a protected boundary's subtree.
            // ancestors: every protected root and each of its ancestors.
            std::vector<const void*> inside, ancestors;
            bool insideComplete = true, ancestorsReadable = true;
        };

        void collectSubtree(RE::NiAVObject* object, std::vector<const void*>& out, bool& complete,
            std::uint32_t depth = 0)
        {
            if (!object || !complete) return;
            if (depth >= 128 || out.size() >= 16384) {
                complete = false;
                return;
            }
            out.push_back(object);
            if (auto* node = object->IsNode()) {
                for (const auto& child : node->GetRuntimeData().children)
                    if (child) collectSubtree(child.get(), out, complete, depth + 1);
            }
        }

        // Same relation as native_model_ancestry::within(root, node) == Inside
        // for every node at once: the root and the parents above it.
        void collectAncestors(const void* root, std::vector<const void*>& out, bool& readable)
        {
            for (std::uint32_t depth = 0; root && depth < 128; ++depth) {
                out.push_back(root);
                const void* parent = nullptr;
                if (!native_memory::tryReadField(root, 0x28, parent)) {
                    readable = false;
                    return;
                }
                root = parent;
            }
            if (root) readable = false; // Cycle or over-deep chain.
        }

        void buildOwnership(Retirement& context) noexcept
        {
            try {
                for (std::size_t i = 0; i < context.count; ++i) {
                    collectSubtree(context.models[i].boundary.get(), context.inside, context.insideComplete);
                    collectAncestors(context.models[i].root.get(), context.ancestors, context.ancestorsReadable);
                }
                for (auto* set : { &context.inside, &context.ancestors }) {
                    std::sort(set->begin(), set->end());
                    set->erase(std::unique(set->begin(), set->end()), set->end());
                }
            } catch (...) {
                // Allocation failure: no set can prove a node is outside.
                context.insideComplete = context.ancestorsReadable = false;
            }
        }

        bool contains(const std::vector<const void*>& set, const void* node) noexcept
        {
            return std::binary_search(set.begin(), set.end(), node);
        }
        thread_local Retirement* s_retirement = nullptr;
        class Scope
        {
        public:
            explicit Scope(Retirement* current) noexcept : _previous(std::exchange(s_retirement, current)) {}
            ~Scope() { s_retirement = _previous; }
        private:
            Retirement* _previous;
        };

        // A node crosses ownership when it lies inside a protected boundary,
        // or, for a delete, when it is a protected root or one of its
        // ancestors. A set that could not be built completely cannot prove
        // a node is outside, so it fails closed exactly as an unreadable
        // parent walk did.
        bool crossesOwnership(RE::NiAVObject* node, bool deleting) noexcept
        {
            auto* context = s_retirement;
            if (!context || !node) return false;
            if (!context->insideComplete || (deleting && !context->ancestorsReadable)) {
                ++context->unreadable;
                return true;
            }
            return contains(context->inside, node) || (deleting && contains(context->ancestors, node));
        }

        std::uint32_t findConnection(RE::NiAVObject* root, void* query) noexcept
        {
            // 2DB2B0 recursively searches by BSConnectPoint parent name. A
            // surviving gun is a separate namespace, even with identical names.
            if (crossesOwnership(root, false)) {
                ++s_retirement->pruned;
                return 0; // no match here; native traversal continues elsewhere
            }
            return s_findConnection(root, query);
        }
        void queueDetach(void* queue, const std::uint32_t* owner, RE::NiAVObject* node, bool retarget) noexcept
        {
            if (crossesOwnership(node, true)) { ++s_retirement->rejected; return; }
            s_queueDetach(queue, owner, node, retarget);
        }
        void detach(const std::uint32_t* owner, RE::NiAVObject* node, bool retarget) noexcept
        {
            if (crossesOwnership(node, true)) { ++s_retirement->rejected; return; }
            s_detach(owner, node, retarget);
        }

        bool gunRecord(const void* record, RE::TESObjectWEAP*& weapon, RE::NiAVObject*& model) noexcept
        {
            RE::TESBoundObject* object = nullptr;
            RE::TBO_InstanceData* instance = nullptr;
            if (!native_memory::tryReadField(record, 0, object) || !object || !object->IsWeapon() ||
                !native_memory::tryReadField(record, 8, instance) ||
                !native_memory::tryReadField(record, 0x30, model) || !model) return false;
            weapon = static_cast<RE::TESObjectWEAP*>(object);
            const auto* data = instance ? static_cast<const RE::TESObjectWEAP::InstanceData*>(instance) : &weapon->weaponData;
            return data->type == RE::WEAPON_TYPE::kGun;
        }

        void retire(void* biped, void* record, bool deferred) noexcept
        {
            Retirement context{};
            RE::TESObjectWEAP* outgoing = nullptr;
            RE::NiAVObject* retiringRoot = nullptr;
            const bool playerBiped = s_installed && biped &&
                (biped == native_hand_equipment::playerBiped(true) || biped == native_hand_equipment::playerBiped(false));
            if (playerBiped && gunRecord(record, outgoing, retiringRoot)) {
                // Current native records, not the changed actor equipment array,
                // own the surviving models during a retirement/rebuild callback.
                for (std::uint32_t slot = 0; slot < 44; ++slot) {
                    const auto* candidate = static_cast<const std::byte*>(biped) + 0x10 + slot * 0x58;
                    RE::TESObjectWEAP* weapon = nullptr;
                    RE::NiAVObject* root = nullptr;
                    if (candidate == record || !gunRecord(candidate, weapon, root) || root == retiringRoot) continue;
                    auto& protectedModel = context.models[context.count++];
                    protectedModel.root.reset(root);
                    protectedModel.boundary.reset(root);
                    // Protect a distinct hand attachment from name matching as
                    // well. A shared parent cannot be pruned wholesale because
                    // it may also contain the actual outgoing model.
                    if (root->parent && within(retiringRoot, root->parent) == Relation::Outside)
                        protectedModel.boundary.reset(root->parent);
                    protectedModel.form = weapon->formID;
                    protectedModel.shapesBefore = native_weapon_model_geometry::inspect(root).shapes;
                }
                if (context.count) buildOwnership(context);
            }
            const Scope scope(context.count ? &context : nullptr);
            s_retire(biped, record, deferred);
            if (context.count) {
                try {
                    for (std::size_t i = 0; i < context.count; ++i) {
                        const auto& model = context.models[i];
                        const auto after = native_weapon_model_geometry::inspect(model.root.get());
                        ROCK_LOG_INFO(Weapon,
                            "AKIMBO model retirement outgoing={:08X} survivor={:08X} root={:#x} boundary={:#x} deferred={} prunedSearches={} rejectedDeletes={} unreadable={} shapesBefore={} shapesAfter={} complete={}",
                            outgoing->formID, model.form, reinterpret_cast<std::uintptr_t>(model.root.get()),
                            reinterpret_cast<std::uintptr_t>(model.boundary.get()), deferred, context.pruned, context.rejected,
                            context.unreadable, model.shapesBefore, after.shapes, after.complete);
                    }
                } catch (...) { OutputDebugStringA("ROCK: equipped model retirement diagnostic failed\n"); }
            }
        }

        bool readCull(void* process, std::uint8_t& flags) noexcept
        {
            const void* middle = nullptr;
            return native_memory::tryReadField(process, 8, middle) && middle &&
                native_memory::tryReadField(middle, 0x4A0, flags);
        }
        // 140DCF5E0 (called by the equip executor 140E72DA0 at 140E73280 for
        // a hand slot) hides the actor's weapons with reason 2 until the
        // native draw cleanup clears it (~190 ms, runtime 12 Oct 22:16:13).
        // For ROCK's equip into index 1 that hide only blanks the unchanged
        // primary for a frame and delays the second gun, which ROCK presents
        // itself once its model is complete; it is skipped for that equip.
        constexpr std::uintptr_t kEquipHideReturn = 0xDCF669;
        constexpr std::uint32_t kEquipHideReason = 2;
        std::atomic<std::uint32_t> s_skippedHides{0};

        bool cull(void* process, RE::Actor* actor, bool hidden, std::uint32_t reason) noexcept
        {
            const auto caller = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
            if (s_installed && hidden && reason == kEquipHideReason && actor && actor == f4vr::getPlayer() &&
                caller == REL::Offset(kEquipHideReturn).address() &&
                native_hand_equipment::secondaryEquipInProgress(actor)) {
                if (s_skippedHides.fetch_add(1, std::memory_order_relaxed) < 8)
                    ROCK_LOG_INFO(Weapon, "AKIMBO native equip hide skipped for the second-weapon equip; both guns stay visible");
                return false;
            }
            std::uint8_t before = 0xFF, after = 0xFF;
            const bool trace = s_installed && actor && actor == f4vr::getPlayer() &&
                s_cullLogs.fetch_add(1, std::memory_order_relaxed) < 128;
            if (trace) readCull(process, before);
            const bool result = s_cull(process, actor, hidden, reason);
            if (trace) {
                readCull(process, after);
                try {
                    ROCK_LOG_INFO(Weapon, "AKIMBO native visibility caller={:#x} hidden={} reason={:#x} flagsBefore={:#x} flagsAfter={:#x} changed={} nativeState={} revision={}",
                        caller - REL::Module::get().base(), hidden, reason, before, after, result,
                        f4vr::getNativeWeaponState(static_cast<RE::PlayerCharacter*>(actor)), s_revision.load(std::memory_order_acquire));
                } catch (...) { OutputDebugStringA("ROCK: native visibility diagnostic failed\n"); }
            }
            return result;
        }

        std::uint64_t s_completedRevision = 0, s_reportedRevision = 0; // FRIK frame only
        std::uint8_t s_reportedFlags = 0xFF;
        std::array<std::uintptr_t, 2> s_reportedRoots{};
    }

    void noteEquipmentChange() noexcept
    {
        s_revision.fetch_add(1, std::memory_order_release);
    }

    void updateVisibility() noexcept
    {
        if (!s_installed) return;
        const auto revision = s_revision.load(std::memory_order_acquire);
        if (!revision || revision == s_completedRevision) return;
        const auto& frame = runtime_state::currentFrame();
        if (!frame.localSkeletonReady || !frame.visualAuthorityAvailable || frame.localMenuBlocking ||
            frame.compatibilityConfigBlocking || !frame.weaponDrawn) return;
        auto* player = f4vr::getPlayer();
        std::uint8_t flags = 0xFF;
        if (!player || !readCull(player->currentProcess, flags)) return;
        const auto views = native_hand_equipment::equippedViews();
        std::array<std::uintptr_t, 2> roots{
            reinterpret_cast<std::uintptr_t>(views[0].root), reinterpret_cast<std::uintptr_t>(views[1].root)};
        try {
            if (revision != s_reportedRevision || flags != s_reportedFlags || roots != s_reportedRoots) {
                s_reportedRevision = revision; s_reportedFlags = flags; s_reportedRoots = roots;
                for (std::size_t i = 0; i < views.size(); ++i) {
                    auto* attachment = native_hand_equipment::resolveAttachment(static_cast<std::uint32_t>(i));
                    const auto geometry = native_weapon_model_geometry::inspect(views[i].root);
                    ROCK_LOG_INFO(Weapon,
                        "AKIMBO visibility handoff revision={} index={} weapon={:08X} root={:#x} attachment={:#x} rootCulled={} attachmentCulled={} shapes={} nativeCullFlags={:#x}",
                        revision, i, views[i].weapon ? views[i].weapon->formID : 0, roots[i],
                        reinterpret_cast<std::uintptr_t>(attachment), views[i].root ? views[i].root->GetAppCulled() : true,
                        attachment ? attachment->GetAppCulled() : true, geometry.shapes, flags);
                }
            }
            // Zero means the native controller has released ALL hide owners.
            // Do not erase reload/script/animation hide bits to make a gun show.
            // Complete this inventory handoff once; no permanent visibility force.
            if (flags != 0) return;
            if (views[0].root && views[1].root &&
                (within(views[0].root, views[1].root) != Relation::Outside ||
                 within(views[1].root, views[0].root) != Relation::Outside)) return;
            std::array<std::array<RE::NiAVObject*, 48>, 2> paths{};
            std::array<std::size_t, 2> counts{};
            for (std::size_t i = 0; i < views.size(); ++i) {
                if (!views[i].weapon) continue;
                if (!views[i].root) return;
                const auto geometry = native_weapon_model_geometry::inspect(views[i].root);
                if (!geometry.complete || !geometry.shapes) return;
                auto* attachment = native_hand_equipment::resolveAttachment(static_cast<std::uint32_t>(i));
                auto* node = views[i].root;
                while (node && counts[i] < paths[i].size()) {
                    paths[i][counts[i]++] = node;
                    if (node == attachment) break;
                    node = node->parent;
                }
                if (!attachment || node != attachment) return;
            }
            // Both paths have been validated before changing either hand.
            std::uint32_t restored = 0;
            for (std::size_t i = 0; i < paths.size(); ++i) for (std::size_t j = 0; j < counts[i]; ++j) {
                if (!equipped_weapon_visual_state::isLocallyVisible(paths[i][j])) {
                    equipped_weapon_visual_state::setLocallyVisible(paths[i][j], true);
                    ++restored;
                }
            }
            s_completedRevision = revision;
            ROCK_LOG_INFO(Weapon, "AKIMBO inventory visibility completed revision={} restoredPathNodes={} nativeCullFlags=0", revision, restored);
        } catch (...) { OutputDebugStringA("ROCK: equipped visibility handoff failed\n"); }
    }

    bool install() noexcept
    {
        if (s_installed) return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;
        struct Entry { const char* name; std::uintptr_t rva; std::array<std::uint8_t, 17> bytes; std::size_t size; void* hook; void** original; };
        // Queue/detach guards enter immediately AFTER the unchanged native null
        // test. The relocated prefixes contain no relative branches or RIP data.
        const std::array<Entry, 5> entries{{
            {"Equipped model retire",0x1C5FB0,{0x48,0x8B,0xC4,0x44,0x88,0x40,0x18,0x48,0x89,0x48,0x08,0x55,0x53,0x41,0x54},15,reinterpret_cast<void*>(&retire),reinterpret_cast<void**>(&s_retire)},
            {"Equipped connection search",0x2DB2B0,{0x48,0x89,0x54,0x24,0x10,0x53,0x48,0x83,0xEC,0x20,0x33,0xDB,0x48,0x85,0xC9},15,reinterpret_cast<void*>(&findConnection),reinterpret_cast<void**>(&s_findConnection)},
            {"Equipped queued detach",0xDA94A9,{0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10,0x48,0x89,0x74,0x24,0x18},15,reinterpret_cast<void*>(&queueDetach),reinterpret_cast<void**>(&s_queueDetach)},
            {"Equipped immediate detach",0x1C5E99,{0x53,0x57,0x41,0x56,0x48,0x83,0xEC,0x30,0x48,0x8B,0xFA,0x45,0x0F,0xB6,0xF0},15,reinterpret_cast<void*>(&detach),reinterpret_cast<void**>(&s_detach)},
            {"Equipped visibility owner trace",0xECEB90,{0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20,0x48,0x8B,0xFA,0x41,0x0F,0xB6,0xD8},17,reinterpret_cast<void*>(&cull),reinterpret_cast<void**>(&s_cull)}
        }};
        for (const auto& entry : entries) {
            std::array<std::uint8_t, 17> bytes{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(entry.rva).address()), bytes.data(), entry.size) ||
                std::memcmp(bytes.data(), entry.bytes.data(), entry.size) != 0) {
                ROCK_LOG_ERROR(Init, "{} validation failed; equipped model lifetime hooks unchanged", entry.name);
                return false;
            }
        }
        // Independent readers/writers establish middleHigh+4A0, including the
        // mask operations below their validated prefixes (ED11B0/ED1240).
        constexpr std::array<std::uint8_t, 8> cullField{0x41,0x80,0xB9,0xA0,0x04,0,0,0};
        std::array<std::uint8_t, 8> actual{};
        if (!native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(0xED11BC).address()), actual.data(), actual.size()) || actual != cullField)
            return false;
        const auto matches = [](std::uintptr_t rva, const auto& expected) {
            std::array<std::uint8_t, 9> bytes{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(rva).address()), bytes.data(), expected.size()) &&
                std::memcmp(bytes.data(), expected.data(), expected.size()) == 0;
        };
        if (!matches(0xDA94A0, std::array<std::uint8_t,9>{0x4D,0x85,0xC0,0x0F,0x84,0xD8,0,0,0}) ||
            !matches(0x1C5E90, std::array<std::uint8_t,9>{0x48,0x85,0xD2,0x0F,0x84,0,0x01,0,0}) ||
            !matches(0xED124C, std::array<std::uint8_t,7>{0x80,0xB8,0xA0,0x04,0,0,0})) return false;
        try {
            for (const auto& entry : entries) {
                if (!entry_trampoline_hook::install(entry.name, entry.rva, entry.bytes.data(), entry.size, entry.hook, *entry.original)) {
                    // Disabled wrappers preserve ordinary native behavior if a
                    // later installation fails; no protection scope is entered.
                    ROCK_LOG_ERROR(Init, "Equipped model lifetime installation incomplete; hand-directed equipment unavailable");
                    return false;
                }
            }
            s_installed = true;
            ROCK_LOG_INFO(Init, "Equipped model retirement ownership and inventory visibility handoff installed");
            return true;
        } catch (...) {
            ROCK_LOG_ERROR(Init, "Equipped model lifetime installation failed; hand-directed equipment unavailable");
            return false;
        }
    }
}
