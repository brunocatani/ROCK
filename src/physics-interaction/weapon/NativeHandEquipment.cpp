#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/weapon/NativeExecutionSlotStub.h"
#include "physics-interaction/weapon/NativeSecondaryModelConstruction.h"
#include "physics-interaction/weapon/NativeEquippedModelLifetime.h"
#include "physics-interaction/weapon/NativePrimaryWeaponGraph.h"
#include "physics-interaction/weapon/NativeSecondaryActions.h"
#include "physics-interaction/weapon/NativeInventoryHandSlots.h"

#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/input/NativePipboyHandInput.h"
#include "physics-interaction/PhysicsLog.h"
#include "RockConfig.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESForms.h"
#include <REL/Relocation.h>
#include <Windows.h>
#include <array>
#include <atomic>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace rock::native_hand_equipment
{
    namespace
    {
        using queued_hand_input::Hand;
        using UseItem = bool (*)(RE::ActorEquipManager*, RE::Actor*, std::uint32_t*,
            std::int32_t, const RE::BGSEquipSlot*, bool, bool);
        using GetSlot = RE::BGSEquipSlot* (*)(RE::TESBoundObject*, RE::TBO_InstanceData*);
        using ResolveForm = RE::TESBoundObject* (*)(void*, std::uint32_t*);
        using ResolveIndex = std::uint32_t* (*)(RE::Actor*, std::uint32_t*, const RE::BGSEquipSlot*);
        using ModelIndex = std::uint32_t (*)(RE::Actor*, RE::TESBoundObject*, std::uint32_t);
        using GetAttachment = RE::NiAVObject* (*)(void*, RE::Actor*, const void* const*, std::uint32_t);
        struct InventorySelection
        {
            RE::TESForm* object = nullptr;
            RE::BSTSmartPointer<RE::TBO_InstanceData> instance{};
            std::uint32_t count = 0, wornMask = 0;
            std::array<std::byte, 8> flags{};
        };
        static_assert(sizeof(InventorySelection) == 0x20);
        static_assert(offsetof(InventorySelection, count) == 0x10);
        static_assert(offsetof(InventorySelection, flags) == 0x18);
        using ReadSelection = void (*)(void*, std::uint32_t*, std::int32_t, InventorySelection*);

        UseItem s_useItem = nullptr;
        GetSlot s_getSlot = nullptr;
        ResolveForm s_resolveForm = nullptr;
        ResolveIndex s_resolveIndex = nullptr;
        ModelIndex s_modelIndex = nullptr;
        GetAttachment s_getAttachment = nullptr;
        ReadSelection s_readSelection = nullptr;
        bool s_installed = false; // Load owner; immutable after hook installation.
        std::atomic<std::uint64_t> s_transaction{0}; // Input/UI -> frame diagnostic publication.

        // NativePipboyHandInput carries physical provenance through BOTH
        // native UI and GFx input queues. Untagged commands remain native.
        struct ToggleContext
        {
            RE::TESBoundObject* weapon = nullptr; // Borrowed for this call only.
            RE::BGSEquipSlot* either = nullptr;
        };
        thread_local ToggleContext s_toggle{};
        struct ExecutionContext
        {
            RE::Actor* actor = nullptr;
            RE::TESBoundObject* weapon = nullptr;
            const RE::BGSEquipSlot* slot = nullptr;
            RE::TBO_InstanceData* instance = nullptr;
            bool eligible = false;
            bool consumed = false;
        };
        // Borrowed only for one synchronous native inventory transaction.
        // Nested/native unrelated equipment cannot reuse the consumed choice.
        thread_local ExecutionContext* s_execution = nullptr;

        template <class T>
        class ScopedValue
        {
        public:
            ScopedValue(T& destination, T value) noexcept :
                _destination(destination), _previous(std::exchange(destination, value)) {}
            ~ScopedValue() { _destination = _previous; }
            ScopedValue(const ScopedValue&) = delete;
            ScopedValue& operator=(const ScopedValue&) = delete;
        private:
            T& _destination;
            T _previous;
        };

        const char* handName(Hand hand) noexcept
        {
            return hand == Hand::Left ? "left" : hand == Hand::Right ? "right" : "none";
        }

        struct Slots
        {
            RE::BGSEquipSlot* right = nullptr;
            RE::BGSEquipSlot* left = nullptr;
            RE::BGSEquipSlot* either = nullptr;
        };

        bool hasHandChildren(const RE::BGSEquipSlot* slot, const Slots& slots) noexcept
        {
            std::uint32_t count = 0;
            const void* storage = nullptr;
            std::array<RE::BGSEquipSlot*, 2> children{};
            return native_memory::tryReadField(slot, 0x30, count) && count == children.size() &&
                native_memory::tryReadField(slot, 0x20, storage) && storage &&
                native_memory::guardedCopyFromMemory(storage, children.data(), sizeof(children)) &&
                children[0] == slots.right && children[1] == slots.left;
        }

        bool resolveSlots(RE::Actor* actor, Slots& slots) noexcept
        {
            slots = {RE::TESForm::GetFormByID<RE::BGSEquipSlot>(0x13F42),
                RE::TESForm::GetFormByID<RE::BGSEquipSlot>(0x13F43),
                RE::TESForm::GetFormByID<RE::BGSEquipSlot>(0x13F44)};
            std::uint32_t rightIndex = ~0u, leftIndex = ~0u, flags = 0;
            if (!slots.right || !slots.left || !slots.either ||
                !hasHandChildren(slots.either, slots) ||
                !native_memory::tryReadField(slots.either, 0x38, flags) || (flags & 1u)) return false;
            s_resolveIndex(actor, &rightIndex, slots.right);
            s_resolveIndex(actor, &leftIndex, slots.left);
            // These indices also select the native primary/shield model
            // paths. An unfamiliar race layout is not safe to reinterpret.
            return rightIndex == 0 && leftIndex == 1;
        }

        RE::BGSEquipSlot* toggleSlot(RE::TESBoundObject* object, RE::TBO_InstanceData* instance) noexcept
        {
            auto* original = s_getSlot(object, instance);
            // Consume before native permission checks or equipment callbacks
            // can make a nested transaction. There is one resolver invocation
            // per E70920 transaction.
            const auto context = std::exchange(s_toggle, ToggleContext{});
            if (object != context.weapon || !context.either) return original;
            Slots slots{};
            auto* player = f4vr::getPlayer();
            const bool eligible = player && resolveSlots(player, slots);
            try {
                ROCK_LOG_INFO(Weapon, "AKIMBO toggle slot inputToken={} weapon={:08X} instance={:#x} original={:08X} either={:08X} eligible={}",
                    native_pipboy_hand_input::current().sequence, object->formID,
                    reinterpret_cast<std::uintptr_t>(instance), original ? original->formID : 0,
                    context.either->formID, eligible);
            } catch (...) {
                OutputDebugStringA("ROCK: native toggle slot diagnostic failed\n");
            }
            if (!eligible) return original;
            if (s_execution && s_execution->weapon == object) {
                s_execution->instance = instance;
                s_execution->eligible = true;
            }
            // Only E70920's local worn-bit/toggle decision sees EitherHand.
            // The base form/instance equip slot is never written. All native
            // conflict resolution, stack splitting, permission checks and
            // indexed EquippedWeaponData construction still execute normally.
            return context.either;
        }

        const RE::BGSEquipSlot* executionSlot(void* component, RE::TBO_InstanceData* instance,
            RE::Actor* actor, const RE::BGSObjectInstance* item, const RE::BGSEquipSlot* requested) noexcept
        {
            using Getter = const RE::BGSEquipSlot* (*)(void*, RE::TBO_InstanceData*);
            const void* vtable = nullptr;
            Getter getter = nullptr;
            if (!native_memory::tryReadField(component, 0, vtable) ||
                !native_memory::tryReadField(vtable, 0x38, getter) || !getter) return nullptr;
            const auto* original = getter(component, instance);
            auto* context = s_execution;
            const bool matches = context && context->eligible && !context->consumed && context->actor == actor &&
                item && item->object == context->weapon && requested == context->slot &&
                (!context->instance || item->instanceData.get() == context->instance);
            if (!matches) {
                if (context && !context->consumed) {
                    try {
                        ROCK_LOG_WARN(Weapon,
                            "AKIMBO execution context mismatch inputToken={} eligible={} actorMatch={} itemMatch={} instanceMatch={} requested={:08X} expected={:08X}",
                            native_pipboy_hand_input::current().sequence, context->eligible, context->actor == actor,
                            item && item->object == context->weapon,
                            item && (!context->instance || item->instanceData.get() == context->instance),
                            requested ? requested->formID : 0, context->slot ? context->slot->formID : 0);
                    } catch (...) {
                        OutputDebugStringA("ROCK: native execution context diagnostic failed\n");
                    }
                }
                return original;
            }
            context->consumed = true;
            // E72E4F skips the explicit request when race+1D8 bit 19 is set.
            // Return the selected leaf at this transaction's native getter so
            // conflict selection, EquippedWeaponData and attachment agree.
            // No race flags, base equip slots or equipment arrays are written.
            try {
                const void* race = nullptr;
                std::uint32_t flags = 0;
                native_memory::tryReadField(actor, 0x418, race);
                native_memory::tryReadField(race, 0x1D8, flags);
                const auto origin = native_pipboy_hand_input::current();
                ROCK_LOG_INFO(Weapon,
                    "AKIMBO execution slot inputToken={} weapon={:08X} instance={:#x} requested={:08X} original={:08X} selected={:08X} raceFlags2={:08X}",
                    origin.sequence, item->object->formID, reinterpret_cast<std::uintptr_t>(item->instanceData.get()),
                    requested ? requested->formID : 0, original ? original->formID : 0,
                    context->slot ? context->slot->formID : 0, flags);
            } catch (...) {
                OutputDebugStringA("ROCK: native execution slot diagnostic failed\n");
            }
            return context->slot;
        }

        bool leftBipedAvailable(RE::PlayerCharacter* player, RE::TESObjectWEAP* weapon) noexcept
        {
            // Native secondary weapons use the race's shield biped object.
            // Preserve existing non-weapon equipment (including wrist gear)
            // rather than silently replacing an occupied model slot.
            const auto index = s_modelIndex(player, weapon, 1);
            const void* biped = nullptr;
            RE::TESForm* occupant = nullptr;
            if (index >= 44 || !native_memory::tryReadField(player, 0xFE0, biped) || !biped ||
                !native_memory::tryReadField(biped, index * 0x58 + 0x10, occupant)) {
                ROCK_LOG_WARN(Weapon, "AKIMBO left equip refused: native biped slot unavailable index={}", index);
                return false;
            }
            if (occupant && !occupant->IsWeapon()) {
                ROCK_LOG_WARN(Weapon, "AKIMBO left equip refused: native biped slot {} occupied by {:08X}", index, occupant->formID);
                return false;
            }
            return true;
        }

        bool useItem(RE::ActorEquipManager* manager, RE::Actor* actor, std::uint32_t* handle,
            std::int32_t stack, const RE::BGSEquipSlot* requestedSlot, bool toggle, bool force) noexcept
        {
            const native_inventory_hand_slots::Scope inventoryOwner(actor);
            auto* player = f4vr::getPlayer();
            const auto origin = native_pipboy_hand_input::current();
            // Akimbo Mode off keeps ordinary single-weapon equipping: no
            // physical-hand routing reaches the native transaction.
            const auto hand = g_rockConfig.rockAkimboMode ? origin.hand : Hand::None;
            if (hand == Hand::None || !actor || actor != player || !handle || requestedSlot) {
                try {
                    ROCK_LOG_SAMPLE_INFO(Weapon, 250,
                        "AKIMBO Pip-Boy ordinary request hand={} handle={:08X} stack={} explicitSlot={}",
                        handName(hand), handle ? *handle : 0, stack, requestedSlot != nullptr);
                } catch (...) {
                    OutputDebugStringA("ROCK: Pip-Boy ordinary request diagnostic failed\n");
                }
                const auto accepted = s_useItem(manager, actor, handle, stack, requestedSlot, toggle, force);
                // Record actual equipment even when provenance was absent;
                // otherwise a failed input route hides all native state.
                s_transaction.fetch_add(1, std::memory_order_release);
                return accepted;
            }

            bool accepted = false;
            try {
                void* inventoryHandles = nullptr;
                if (!native_memory::tryReadValue(reinterpret_cast<void**>(REL::Offset(0x5936A60).address()),
                        inventoryHandles) || !inventoryHandles) return false;
                auto* object = s_resolveForm(inventoryHandles, handle);
                if (!object || object->formType != RE::ENUM_FORM_ID::kWEAP ||
                    static_cast<RE::TESObjectWEAP*>(object)->weaponData.type != RE::WEAPON_TYPE::kGun)
                    return s_useItem(manager, actor, handle, stack, requestedSlot, toggle, force);

                Slots slots{};
                if (!resolveSlots(actor, slots)) {
                    ROCK_LOG_WARN(Weapon, "AKIMBO equip refused: native player hand-slot layout unavailable");
                    return false;
                }
                if (hand == Hand::Left && !leftBipedAvailable(player, static_cast<RE::TESObjectWEAP*>(object))) return false;
                const auto formID = object->formID;
                const auto inventoryHandle = *handle;
                const auto* selected = hand == Hand::Left ? slots.left : slots.right;
                InventorySelection selection{};
                s_readSelection(inventoryHandles, handle, stack, &selection);
                if (selection.object != object || selection.count == 0) {
                    ROCK_LOG_WARN(Weapon, "AKIMBO equip refused: selected native inventory stack unavailable weapon={:08X} stack={}", formID, stack);
                    return false;
                }
                ExecutionContext execution{actor, object, selected};
                const ScopedValue executionScope{s_execution, &execution};
                const ScopedValue context{s_toggle, ToggleContext{object, slots.either}};
                const bool affectsPair = hand == Hand::Left || hasSecondaryGun();
                accepted = s_useItem(manager, actor, handle, stack, selected, toggle, force);
                if (accepted && affectsPair) native_equipped_model_lifetime::noteEquipmentChange();
                // Native return means request accepted, not model/graph ready.
                const auto sequence = s_transaction.fetch_add(1, std::memory_order_release) + 1;
                ROCK_LOG_INFO(Weapon,
                    "AKIMBO equip request={} inputToken={} hand={} weapon={:08X} handle={:08X} stack={} requestedSlot={:08X} accepted={} executionSelected={} nativeInventory=true",
                    sequence, origin.sequence, handName(hand), formID, inventoryHandle, stack,
                    selected->formID, accepted, execution.consumed);
                return accepted;
            } catch (...) {
                // Never retry an inventory mutation after an exception: it
                // could already have succeeded. No exception crosses native.
                OutputDebugStringA("ROCK: hand-directed Pip-Boy transaction diagnostic failed\n");
                return accepted;
            }
        }

        template <std::size_t N>
        bool bytesMatch(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(rva).address()),
                actual.data(), actual.size()) && actual == expected;
        }

        bool callRedirectMatches(std::uintptr_t rva, std::uintptr_t destination, std::uint8_t opcode = 0xE8) noexcept
        {
            const auto address = REL::Offset(rva).address();
            std::array<std::uint8_t, 5> call{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), call.data(), call.size()) ||
                call[0] != opcode) return false;
            std::int32_t displacement = 0;
            std::memcpy(&displacement, call.data() + 1, sizeof(displacement));
            const auto stubAddress = address + 5 + displacement;
            std::array<std::uint8_t, 14> stub{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(stubAddress), stub.data(), stub.size())) return false;
            constexpr std::array<std::uint8_t, 6> jump{0xFF,0x25,0,0,0,0};
            std::uintptr_t target = 0;
            std::memcpy(&target, stub.data() + jump.size(), sizeof(target));
            return std::memcmp(stub.data(), jump.data(), jump.size()) == 0 && target == destination &&
                FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(stubAddress), stub.size());
        }

        struct Identity
        {
            std::uint32_t form = 0, slot = 0, bipedIndex = ~0u;
            std::uintptr_t instance = 0, data = 0, root = 0, holder = 0, manager = 0;
            bool operator==(const Identity&) const = default;
        };
        // Frame-owned numeric observations only. Never dereference a previous
        // observation, even when the actor/model has not apparently changed.
        std::array<Identity, 2> s_previous{};
        std::uint64_t s_observedTransaction = 0;
    }

    const void* playerBiped(bool firstPerson) noexcept
    {
        auto* player = f4vr::getPlayer();
        if (!player) return nullptr;
        const void* biped = nullptr;
        if (firstPerson) {
            native_memory::tryReadField(player, 0xFE0, biped);
            return biped;
        }
        // Native 1C80CA/1C80E1 use the bool biped getter at vslot A1.
        // Call that contract directly; the CommonLib overload declaration
        // order does not match the VR vtable's A1/A2 ordering.
        using GetBiped = const void* const* (*)(RE::Actor*, bool);
        const void* vtable = nullptr;
        GetBiped getBiped = nullptr;
        if (native_memory::tryReadField(player, 0, vtable) &&
            native_memory::tryReadField(vtable, 0x508, getBiped) && getBiped)
            native_memory::tryReadValue(getBiped(player, false), biped);
        return biped;
    }

    std::array<EquippedView, 2> equippedViews() noexcept
    {
        std::array<EquippedView, 2> views{};
        if (!s_installed) return views;
        auto* player = f4vr::getPlayer();
        auto* middle = player && player->currentProcess ? player->currentProcess->middleHigh : nullptr;
        if (!middle || middle->equippedItems.size() > 32) return views;
        const auto* biped = playerBiped(true);
        const auto* bodyBiped = playerBiped(false);
        for (const auto& entry : middle->equippedItems) {
            const auto index = entry.equipIndex.index;
            if (index >= views.size() || !entry.item.object || !entry.item.object->IsWeapon()) continue;
            auto* weapon = static_cast<RE::TESObjectWEAP*>(entry.item.object);
            auto* instance = entry.item.instanceData.get();
            const auto* data = instance ? static_cast<const RE::TESObjectWEAP::InstanceData*>(instance) : &weapon->weaponData;
            if (data->type != RE::WEAPON_TYPE::kGun) continue;
            auto& view = views[index];
            // Duplicate native indices are ambiguous, never choose an arbitrary root.
            if (view.weapon) return {};
            view.weapon = weapon;
            view.instance = instance;
            view.data = reinterpret_cast<std::uintptr_t>(entry.data.get());
            view.slot = entry.equipSlot ? entry.equipSlot->formID : 0;
            const auto modelIndex = s_modelIndex(player, weapon, index);
            if (modelIndex >= 44) continue;
            const auto readModel = [&](const void* source, RE::NiAVObject*& root) {
                if (!source) return;
                // One guarded copy of the record's form (+10), instance (+18)
                // and root (+40) span: each guarded read costs a page query,
                // and this view is rebuilt many times per frame.
                std::array<std::byte, 0x38> fields{};
                const auto* record = static_cast<const std::byte*>(source) + modelIndex * 0x58;
                if (!native_memory::guardedCopyFromMemory(record + 0x10, fields.data(), fields.size())) return;
                RE::TESBoundObject* modelForm = nullptr;
                RE::TBO_InstanceData* modelInstance = nullptr;
                std::memcpy(&modelForm, fields.data(), sizeof(modelForm));
                std::memcpy(&modelInstance, fields.data() + 0x08, sizeof(modelInstance));
                if (modelForm == weapon && modelInstance == instance)
                    std::memcpy(&root, fields.data() + 0x30, sizeof(root));
            };
            readModel(biped, view.root);
            if (bodyBiped != biped) readModel(bodyBiped, view.bodyRoot);
        }
        return views;
    }

    bool hasSecondaryGun() noexcept
    {
        return equippedViews()[1].weapon != nullptr;
    }

    RE::NiAVObject* resolveAttachment(std::uint32_t index) noexcept
    {
        if (!s_installed || !s_getAttachment || index > 1) return nullptr;
        auto* player = f4vr::getPlayer();
        const void* biped = nullptr;
        if (!player || !player->currentProcess ||
            !native_memory::tryReadField(player, 0xFE0, biped) || !biped) return nullptr;
        // EC5230 selects the race's equipment-node name and searches the
        // supplied biped root. EF5273 / EF5C1A use this same helper for the
        // player's first-person Weapon frame before native arm placement.
        return s_getAttachment(player->currentProcess, player, &biped, index);
    }

    void observe() noexcept
    {
        const auto sequence = s_transaction.load(std::memory_order_acquire);
        if (!sequence) return;
        try {
            std::array<Identity, 2> current{};
            std::array<std::uint32_t, 2> ammo{};
            auto* player = f4vr::getPlayer();
            auto* middle = player && player->currentProcess ? player->currentProcess->middleHigh : nullptr;
            if (middle && middle->equippedItems.size() <= 32) {
                for (const auto& entry : middle->equippedItems) {
                    const auto index = entry.equipIndex.index;
                    if (index >= current.size() || !entry.item.object || !entry.item.object->IsWeapon()) continue;
                    auto& identity = current[index];
                    identity.form = entry.item.object->formID;
                    identity.slot = entry.equipSlot ? entry.equipSlot->formID : 0;
                    identity.instance = reinterpret_cast<std::uintptr_t>(entry.item.instanceData.get());
                    identity.data = reinterpret_cast<std::uintptr_t>(entry.data.get());
                    native_memory::tryReadField(entry.data.get(), 0x18, ammo[index]);
                    // Native 1CAFC0 maps index 0 by weapon type and index 1
                    // through the actor's shield slot. This also disambiguates
                    // identical copies with the same instance-data pointer.
                    const void* biped = nullptr;
                    native_memory::tryReadField(player, 0xFE0, biped);
                    identity.bipedIndex = s_modelIndex(player, static_cast<RE::TESObjectWEAP*>(entry.item.object), index);
                    if (biped && identity.bipedIndex < 44) {
                        const auto* record = static_cast<const std::byte*>(biped) + identity.bipedIndex * 0x58;
                        RE::TESBoundObject* form = nullptr;
                        RE::TBO_InstanceData* instance = nullptr;
                        if (native_memory::tryReadField(record, 0x10, form) && form == entry.item.object &&
                            native_memory::tryReadField(record, 0x18, instance) && instance == entry.item.instanceData.get()) {
                            native_memory::tryReadField(record, 0x40, identity.root);
                            native_memory::tryReadField(record, 0x50, identity.holder);
                            native_memory::tryReadField(reinterpret_cast<const void*>(identity.holder), 0x18, identity.manager);
                        }
                    }
                }
            }
            if (sequence == s_observedTransaction && current == s_previous) return;
            s_observedTransaction = sequence;
            s_previous = current;
            for (std::size_t index = 0; index < current.size(); ++index) {
                const auto& item = current[index];
                ROCK_LOG_INFO(Weapon,
                    "AKIMBO native state request={} index={} weapon={:08X} slot={:08X} instance={:#x} data={:#x} loaded={} bipedSlot={} root={:#x} holder={:#x} graphManager={:#x}",
                    sequence, index, item.form, item.slot, item.instance, item.data, ammo[index], item.bipedIndex, item.root, item.holder, item.manager);
            }
        } catch (...) {
            OutputDebugStringA("ROCK: native hand equipment observation failed\n");
        }
    }

    bool install() noexcept
    {
        if (s_installed) return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;
        constexpr std::array<std::uint8_t, 5> useBytes{0xE8,0x17,0x50,0x2D,0};
        constexpr std::array<std::uint8_t, 5> slotBytes{0xE8,0x24,0x58,0x2D,0xFF};
        constexpr std::array<std::uint8_t, 12> executionBytes{0x49,0x8B,0x06,0x48,0x8B,0xD3,0x49,0x8B,0xCE,0xFF,0x50,0x38};
        constexpr std::array<std::uint8_t, 44> executionFrame{
            0x4C,0x89,0x4C,0x24,0x20,0x4C,0x89,0x44,0x24,0x18,
            0x48,0x89,0x54,0x24,0x10,0x48,0x89,0x4C,0x24,0x08,
            0x55,0x53,0x56,0x57,0x41,0x54,0x41,0x55,0x41,0x56,0x41,0x57,
            0x48,0x8D,0x6C,0x24,0xE1,0x48,0x81,0xEC,0xE8,0,0,0};
        if (!bytesMatch(0xB9B904, useBytes) || !bytesMatch(0xE70A37, slotBytes) || !bytesMatch(0xE72E19, executionBytes) ||
            !bytesMatch(0xEC5230, std::array<std::uint8_t,19>{0x44,0x89,0x4C,0x24,0x20,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0x02,0x48,0x8B,0xCA,0x49,0x8B,0xD8}) ||
            !bytesMatch(0x1ABBD0, std::array<std::uint8_t,15>{0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10,0x57,0x48,0x83,0xEC,0x30}) ||
            !bytesMatch(0xE72DA0, executionFrame) ||
            !bytesMatch(0x1AB7A0, std::array<std::uint8_t,9>{0x48,0x83,0xEC,0x28,0xE8,0x07,0xFF,0xFF,0xFF}) ||
            !bytesMatch(0x146260, std::array<std::uint8_t,14>{0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xDA,0xE8,0x82,0xFE,0xFF,0xFF}) ||
            !bytesMatch(0x3E6840, std::array<std::uint8_t,10>{0x48,0x89,0x5C,0x24,0x18,0x56,0x48,0x83,0xEC,0x20}) ||
            !bytesMatch(0x1CAFC0, std::array<std::uint8_t,13>{0x48,0x83,0xEC,0x28,0x48,0x85,0xD2,0x74,0x2C,0x80,0x7A,0x1A,0x2B}) ||
            !bytesMatch(0xE70920, std::array<std::uint8_t,16>{0x48,0x8B,0xC4,0x48,0x89,0x50,0x10,0x55,0x41,0x54,0x41,0x56,0x48,0x8D,0x68,0xB9})) {
            ROCK_LOG_ERROR(Init, "Native hand equipment hook validation failed; Pip-Boy unchanged");
            return false;
        }
        auto& trampoline = F4SE::GetTrampoline();
        if (trampoline.free_size() < 256) return false;
        s_useItem = reinterpret_cast<UseItem>(REL::Offset(0xE70920).address());
        s_getSlot = reinterpret_cast<GetSlot>(REL::Offset(0x146260).address());
        s_resolveForm = reinterpret_cast<ResolveForm>(REL::Offset(0x1AB7A0).address());
        s_resolveIndex = reinterpret_cast<ResolveIndex>(REL::Offset(0x3E6840).address());
        s_modelIndex = reinterpret_cast<ModelIndex>(REL::Offset(0x1CAFC0).address());
        s_getAttachment = reinterpret_cast<GetAttachment>(REL::Offset(0xEC5230).address());
        s_readSelection = reinterpret_cast<ReadSelection>(REL::Offset(0x1ABBD0).address());
        // A left inventory slot without its complete native first-person gun
        // model is not a usable hand-directed equipment capability.
        if (!native_inventory_hand_slots::install() || !native_secondary_model::install() || !native_equipped_model_lifetime::install() ||
            !native_primary_weapon_graph::install() || !native_secondary_actions::install()) return false;
        try {
            native_execution_slot::Stub stub(reinterpret_cast<std::uintptr_t>(&executionSlot), REL::Offset(0xE72E25).address());
            auto* code = trampoline.allocate(stub.getSize());
            std::memcpy(code, stub.getCode(), stub.getSize());
            if (!FlushInstructionCache(GetCurrentProcess(), code, stub.getSize()))
                throw std::runtime_error("execution slot stub instruction flush failed");
            std::array<std::uint8_t, executionBytes.size()> nops{};
            nops.fill(0x90);
            REL::safe_write(REL::Offset(0xE72E19).address(), nops.data(), nops.size());
            // JMP, not CALL: the stub replaces the original call and resumes
            // in the existing native frame without adding a return address.
            (void)trampoline.write_branch<5>(REL::Offset(0xE72E19).address(), reinterpret_cast<std::uintptr_t>(code));
            if (!callRedirectMatches(0xE72E19, reinterpret_cast<std::uintptr_t>(code), 0xE9) ||
                !bytesMatch(0xE72E1E, std::array<std::uint8_t,7>{0x90,0x90,0x90,0x90,0x90,0x90,0x90}))
                throw std::runtime_error("execution slot branch readback failed");
            (void)trampoline.write_call<5>(REL::Offset(0xE70A37).address(), &toggleSlot);
            (void)trampoline.write_call<5>(REL::Offset(0xB9B904).address(), &useItem);
            if (!callRedirectMatches(0xE70A37, reinterpret_cast<std::uintptr_t>(&toggleSlot)) ||
                !callRedirectMatches(0xB9B904, reinterpret_cast<std::uintptr_t>(&useItem)) ||
                !FlushInstructionCache(GetCurrentProcess(), nullptr, 0))
                throw std::runtime_error("hand equipment redirect verification failed");
            if (!native_pipboy_hand_input::install())
                throw std::runtime_error("queued Pip-Boy provenance installation failed");
            s_installed = true;
            ROCK_LOG_INFO(Init, "Native hand-directed Pip-Boy equipment installed; scoped execution-slot resolver and independent equipped action hooks active");
            return true;
        } catch (...) {
            REL::safe_write(REL::Offset(0xE72E19).address(), executionBytes.data(), executionBytes.size());
            REL::safe_write(REL::Offset(0xE70A37).address(), slotBytes.data(), slotBytes.size());
            REL::safe_write(REL::Offset(0xB9B904).address(), useBytes.data(), useBytes.size());
            FlushInstructionCache(GetCurrentProcess(), nullptr, 0);
            ROCK_LOG_ERROR(Init, "Native hand equipment installation failed; original hooks restored");
            return false;
        }
    }
}
