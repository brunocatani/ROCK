#include "physics-interaction/weapon/NativeSecondaryModelConstruction.h"
#include "physics-interaction/weapon/NativeSecondaryModelBranch.h"
#include "physics-interaction/weapon/NativeWeaponModelGeometry.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/BGSInventoryItem.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include <REL/Relocation.h>
#include <Windows.h>
#include <array>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace rock::native_secondary_model
{
    namespace
    {
        using Build = RE::NiAVObject* (*)(void*, std::uint32_t, RE::Actor*, const void* const*, RE::NiNode*);
        using Assemble = void* (*)(std::uint32_t, RE::Actor*, RE::NiAVObject*, void*, bool, float, const RE::BGSObjectInstance*, void*);
        using ModelIndex = std::uint32_t (*)(RE::Actor*, RE::TESBoundObject*, std::uint32_t);
        using Instance = RE::TBO_InstanceData* (*)(RE::ExtraDataList*);
        struct WornExtra
        {
            void* extra = nullptr;
            std::uint32_t stackIndex = 0;
            std::uint8_t type = 0;
            std::array<std::byte, 3> padding{};
        };
        static_assert(sizeof(WornExtra) == 0x10 && offsetof(WornExtra, type) == 0xC);
        using SelectExtra = std::uint32_t (*)(const RE::BGSInventoryItem*, WornExtra*);
        Build s_build = nullptr;
        Assemble s_assemble = nullptr;
        SelectExtra s_selectExtra = nullptr;
        ModelIndex s_modelIndex = nullptr;
        Instance s_instance = nullptr;
        bool s_installed = false; // Plugin load owner; immutable afterwards.

        struct Construction
        {
            RE::Actor* actor = nullptr;
            RE::TESObjectWEAP* weapon = nullptr;
            RE::TBO_InstanceData* instance = nullptr;
            const void* biped = nullptr;
            const void* record = nullptr;
            std::uint32_t slot = ~0u, handIndex = ~0u;
            bool firstPerson = false, assembled = false;
            std::uint32_t stackIndex = ~0u;
            std::string_view selection = "not-reached";
        };
        // Native resource builds can run on different threads. These scopes
        // borrow only the current synchronous native call's retained objects.
        // No inventory/model pointer is published to another thread or frame.
        thread_local Construction* s_construction = nullptr;
        thread_local Construction* s_selection = nullptr;
        template<class T> class Scope
        {
        public:
            Scope(T& slot, T value) noexcept : _slot(slot), _old(std::exchange(slot, value)) {}
            ~Scope() { _slot = _old; }
            Scope(const Scope&) = delete;
            Scope& operator=(const Scope&) = delete;
        private:
            T& _slot;
            T _old;
        };

        bool gunRole(const void* const* biped, std::uint32_t slot, const void* record, bool firstPerson) noexcept
        {
            const auto* context = s_construction;
            const void* current = nullptr;
            return firstPerson && context && context->handIndex == 1 && context->firstPerson && context->slot == slot &&
                context->record == record && native_memory::tryReadValue(biped, current) && current == context->biped;
        }

        std::uint32_t selectExtra(const RE::BGSInventoryItem* inventory, WornExtra* output) noexcept
        {
            // Consume once before entering any native extra-list lookup.
            auto* context = std::exchange(s_selection, nullptr);
            if (!context) return s_selectExtra(inventory, output);
            if (!inventory || inventory->object != context->weapon || !output) {
                context->selection = "inventory-form-mismatch";
                if (output) output->extra = nullptr;
                return 1;
            }

            RE::BSTSmartPointer<RE::BGSInventoryItem::Stack> selected;
            std::uint32_t index = 0, selectedIndex = 0, matches = 0;
            bool hasLeftStack = false;
            const auto required = context->handIndex == 1 ? RE::BGSInventoryItem::Stack::Flag::kSlotIndex2 :
                RE::BGSInventoryItem::Stack::Flag::kSlotIndex1;
            for (auto stack = inventory->stackData; stack;
                stack = RE::BSTSmartPointer<RE::BGSInventoryItem::Stack>(stack->nextStack), ++index) {
                if (index == 4096) {
                    context->selection = "stack-chain-limit";
                    output->extra = nullptr;
                    return 1;
                }
                hasLeftStack |= stack->flags.any(RE::BGSInventoryItem::Stack::Flag::kSlotIndex2);
                if (!stack->flags.any(required)) continue;
                if (!stack->extra || s_instance(stack->extra.get()) != context->instance) continue;
                ++matches;
                selected = stack;
                selectedIndex = index;
            }
            // Primary-only construction retains its original contract. When
            // the same base item also has a left worn stack, protect the right
            // model too: its original first-worn search could pick the left.
            if (context->handIndex == 0 && !hasLeftStack) {
                context->selection = "ordinary-primary";
                return s_selectExtra(inventory, output);
            }
            if (matches != 1) {
                context->selection = matches ? "ambiguous-hand-stack" : "missing-hand-stack";
                output->extra = nullptr;
                return 1;
            }
            // A local view retains the selected real stack; the live chain and
            // worn bits are untouched. The original selector still owns native
            // ExtraDataList locking and extra-type lookup. It stops on this
            // already-worn first entry; its returned ordinal is rebased below.
            const RE::BGSInventoryItem view{inventory->object, selected};
            output->stackIndex = 0;
            const auto result = s_selectExtra(&view, output);
            output->stackIndex += selectedIndex;
            context->stackIndex = output->stackIndex;
            context->selection = output->extra ? "exact-hand-stack" : "hand-stack-without-mod-extra";
            return result;
        }

        void* assemble(std::uint32_t slot, RE::Actor* actor, RE::NiAVObject* root, void* request,
            bool firstPerson, float scale, const RE::BGSObjectInstance* item, void* handles) noexcept
        {
            auto* context = s_construction;
            const bool matches = context && context->slot == slot && context->actor == actor &&
                context->firstPerson == firstPerson && item && item->object == context->weapon &&
                item->instanceData.get() == context->instance;
            const Scope scope{s_selection, matches ? context : nullptr};
            if (matches) context->assembled = true;
            return s_assemble(slot, actor, root, request, firstPerson, scale, item, handles);
        }

        RE::NiAVObject* build(void* resource, std::uint32_t slot, RE::Actor* actor,
            const void* const* biped, RE::NiNode* scene) noexcept
        {
            Construction context{};
            const void* current = nullptr;
            const void* firstPerson = nullptr;
            RE::TESBoundObject* object = nullptr;
            auto* player = f4vr::getPlayer();
            if (actor && actor == player && slot < 44 &&
                native_memory::tryReadValue(biped, current) && current &&
                native_memory::tryReadField(current, slot * 0x58 + 0x10, object) && object && object->IsWeapon()) {
                auto* weapon = static_cast<RE::TESObjectWEAP*>(object);
                RE::TBO_InstanceData* instance = nullptr;
                const auto leftSlot = s_modelIndex(actor, weapon, 1);
                const auto rightSlot = s_modelIndex(actor, weapon, 0);
                if (leftSlot != rightSlot && (slot == leftSlot || slot == rightSlot) &&
                    native_memory::tryReadField(current, slot * 0x58 + 0x18, instance)) {
                    const auto* data = instance ? static_cast<const RE::TESObjectWEAP::InstanceData*>(instance) : &weapon->weaponData;
                    if (data->type == RE::WEAPON_TYPE::kGun &&
                        native_memory::tryReadField(player, 0xFE0, firstPerson)) {
                        context.actor = actor;
                        context.weapon = weapon;
                        context.instance = instance;
                        context.biped = current;
                        context.record = static_cast<const std::byte*>(current) + slot * 0x58 + 0x10;
                        context.slot = slot;
                        context.handIndex = slot == leftSlot ? 1u : 0u;
                        context.firstPerson = current == firstPerson;
                    }
                }
            }
            // Even an unrelated nested build masks the outer scope.
            const Scope scope{s_construction, context.weapon ? &context : nullptr};
            auto* result = s_build(resource, slot, actor, biped, scene);
            if (context.weapon && (context.handIndex == 1 ||
                    (context.selection != "ordinary-primary" && context.selection != "not-reached"))) {
                const auto geometry = native_weapon_model_geometry::inspect(result);
                try {
                    ROCK_LOG_INFO(Weapon,
                        "AKIMBO native model built weapon={:08X} instance={:#x} handIndex={} view={} biped={:#x} slot={} root={:#x} parent={:#x} objects={} shapes={} complete={} assembled={} selection={} stack={} actualSlotPreserved=true",
                        context.weapon->formID, reinterpret_cast<std::uintptr_t>(context.instance),
                        context.handIndex,
                        context.firstPerson ? "first-person" : "body", reinterpret_cast<std::uintptr_t>(current), slot,
                        reinterpret_cast<std::uintptr_t>(result), result ? reinterpret_cast<std::uintptr_t>(result->parent) : 0,
                        geometry.objects, geometry.shapes, geometry.complete, context.assembled, context.selection, context.stackIndex);
                } catch (...) { OutputDebugStringA("ROCK: secondary model construction diagnostic failed\n"); }
            }
            return result;
        }

        struct Patch
        {
            std::uintptr_t rva;
            std::array<std::uint8_t, 8> bytes;
            std::size_t length;
        };
        constexpr std::array<Patch, 6> kPatches{{
            {0x1CA19D, {0x84,0xC0,0x74,0x1E,0x41,0x83,0xFC,0x20}, 8},
            {0x1CA277, {0x84,0xC0,0x74,0x0A,0x41,0x83,0xFC,0x20}, 8},
            {0x1CA2A7, {0x84,0xC0,0x74,0x16,0x41,0x83,0xFC,0x20}, 8},
            {0x1CCFB8, {0xE8,0xE3,0x18,0,0}, 5},
            {0x1CA323, {0xE8,0x98,0x2B,0,0}, 5},
            {0x1C831A, {0xE8,0x51,0x1C,0,0}, 5}
        }};
        template<std::size_t N> bool bytesMatch(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(rva).address()), actual.data(), N) && actual == expected;
        }
        bool redirectMatches(const Patch& patch, std::uintptr_t target, bool branch) noexcept
        {
            const auto address = REL::Offset(patch.rva).address();
            std::array<std::uint8_t, 8> actual{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), actual.data(), patch.length) ||
                actual[0] != (branch ? 0xE9 : 0xE8)) return false;
            for (std::size_t i = 5; i < patch.length; ++i) if (actual[i] != 0x90) return false;
            std::int32_t displacement = 0;
            std::memcpy(&displacement, actual.data() + 1, 4);
            const auto relay = address + 5 + displacement;
            std::array<std::uint8_t, 14> code{};
            std::uintptr_t destination = 0;
            constexpr std::array<std::uint8_t, 6> indirectJump{0xFF,0x25,0,0,0,0};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(relay), code.data(), code.size())) return false;
            std::memcpy(&destination, code.data() + 6, 8);
            return std::memcmp(code.data(), indirectJump.data(), 6) == 0 && destination == target;
        }
    }

    bool install() noexcept
    {
        if (s_installed) return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;
        for (const auto& patch : kPatches) {
            std::array<std::uint8_t, 8> actual{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(patch.rva).address()), actual.data(), patch.length) ||
                std::memcmp(actual.data(), patch.bytes.data(), patch.length) != 0) {
                ROCK_LOG_ERROR(Init, "Secondary native model validation failed at {:#x}; builder unchanged", patch.rva);
                return false;
            }
        }
        if (!bytesMatch(0x1C9F70, std::array<std::uint8_t,15>{0x4C,0x89,0x4C,0x24,0x20,0x4C,0x89,0x44,0x24,0x18,0x48,0x89,0x4C,0x24,0x08}) ||
            !bytesMatch(0x1CE8A0, std::array<std::uint8_t,15>{0x48,0x89,0x7C,0x24,0x20,0x41,0x56,0x48,0x83,0xEC,0x20,0x48,0x8B,0x79,0x08}) ||
            !bytesMatch(0x08B040, std::array<std::uint8_t,15>{0x48,0x89,0x5C,0x24,0x10,0x48,0x89,0x74,0x24,0x18,0x57,0x48,0x83,0xEC,0x20}) ||
            !bytesMatch(0xF0A0A0, std::array<std::uint8_t,23>{0x48,0x8B,0x81,0xE0,0x0F,0,0,0x48,0x85,0xC0,0x74,0x08,0x48,0x3B,0x02,0x75,0x03,0xB0,0x01,0xC3,0x32,0xC0,0xC3})) {
            ROCK_LOG_ERROR(Init, "Secondary native model dependency validation failed; builder unchanged");
            return false;
        }
        auto& trampoline = F4SE::GetTrampoline();
        if (trampoline.free_size() < 2048) return false;
        s_build = reinterpret_cast<Build>(REL::Offset(0x1C9F70).address());
        s_assemble = reinterpret_cast<Assemble>(REL::Offset(0x1CCEC0).address());
        s_selectExtra = reinterpret_cast<SelectExtra>(REL::Offset(0x1CE8A0).address());
        s_modelIndex = reinterpret_cast<ModelIndex>(REL::Offset(0x1CAFC0).address());
        s_instance = reinterpret_cast<Instance>(REL::Offset(0x08B040).address());
        try {
            constexpr std::array<std::array<std::uintptr_t, 3>, 3> destinations{{
                {0x1CA1B7,0x1CA1BF,0x1CA1A5}, {0x1CA285,0x1CA285,0x1CA27F}, {0x1CA2C9,0x1CA2C1,0x1CA2AF}
            }};
            std::array<std::uintptr_t, 6> targets{};
            for (std::size_t i = 0; i < 3; ++i) {
                const auto& destination = destinations[i];
                ModelBranch stub(reinterpret_cast<std::uintptr_t>(&gunRole), REL::Offset(destination[0]).address(),
                    REL::Offset(destination[1]).address(), REL::Offset(destination[2]).address());
                auto* code = trampoline.allocate(stub.getSize());
                std::memcpy(code, stub.getCode(), stub.getSize());
                if (!FlushInstructionCache(GetCurrentProcess(), code, stub.getSize())) throw std::runtime_error("model branch flush failed");
                targets[i] = reinterpret_cast<std::uintptr_t>(code);
            }
            targets[3] = reinterpret_cast<std::uintptr_t>(&selectExtra);
            targets[4] = reinterpret_cast<std::uintptr_t>(&assemble);
            targets[5] = reinterpret_cast<std::uintptr_t>(&build);
            for (std::size_t i = 0; i < kPatches.size(); ++i) {
                const auto& patch = kPatches[i];
                const auto address = REL::Offset(patch.rva).address();
                if (i < 3) {
                    constexpr std::array<std::uint8_t, 8> nops{0x90,0x90,0x90,0x90,0x90,0x90,0x90,0x90};
                    REL::safe_write(address, nops.data(), nops.size());
                    (void)trampoline.write_branch<5>(address, targets[i]);
                } else (void)trampoline.write_call<5>(address, targets[i]);
                if (!redirectMatches(patch, targets[i], i < 3)) throw std::runtime_error("model redirect readback failed");
            }
            if (!FlushInstructionCache(GetCurrentProcess(), nullptr, 0)) throw std::runtime_error("model instruction flush failed");
            s_installed = true;
            ROCK_LOG_INFO(Init, "Secondary native weapon model construction installed: first-person gun role, exact left stack, native biped/graph ownership");
            return true;
        } catch (...) {
            for (const auto& patch : kPatches)
                REL::safe_write(REL::Offset(patch.rva).address(), patch.bytes.data(), patch.length);
            FlushInstructionCache(GetCurrentProcess(), nullptr, 0);
            ROCK_LOG_ERROR(Init, "Secondary native model installation failed; original instructions restored");
            return false;
        }
    }
}
