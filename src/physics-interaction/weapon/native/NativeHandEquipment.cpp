// Hand-directed equipment: which hand each equipped weapon belongs to, equips
// into the second slot, support-hand requests and restoring the hands after a
// load. Installs the native hooks of the second weapon slot.
#include "physics-interaction/weapon/native/NativeHandEquipment.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/weapon/classification/WeaponTypePolicy.h"
#include "physics-interaction/weapon/equip/EquipSlotAllocationPolicy.h"
#include "physics-interaction/weapon/equip/WeaponEquipTransfer.h"
#include "physics-interaction/weapon/native/NativeEquippedModelLifetime.h"
#include "physics-interaction/weapon/native/NativeExecutionSlotStub.h"
#include "physics-interaction/weapon/native/NativeInventoryHandSlots.h"
#include "physics-interaction/weapon/native/NativePipboyWornRows.h"
#include "physics-interaction/weapon/native/NativePrimaryWeaponGraph.h"
#include "physics-interaction/weapon/secondary/NativeSecondaryActions.h"
#include "physics-interaction/weapon/secondary/NativeSecondaryMelee.h"
#include "physics-interaction/weapon/secondary/NativeSecondaryModelConstruction.h"

#include <REL/Relocation.h>
#include <Windows.h>
#include <array>
#include <atomic>
#include <cstring>
#include <stdexcept>
#include <utility>
#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESForms.h"
#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/input/NativePipboyHandInput.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/native/NativeMeleeWand.h"
#include "rock_support/Fo4VrRuntime.h"
#include <RPS/Runtime/HookChain.h>

namespace rock::native_hand_equipment
{
    namespace
    {
        using queued_hand_input::Hand;
        using UseItem = bool (*)(RE::ActorEquipManager*, RE::Actor*, std::uint32_t*, std::int32_t,
            const RE::BGSEquipSlot*, bool, bool);
        using GetSlot = RE::BGSEquipSlot* (*)(RE::TESBoundObject*, RE::TBO_InstanceData*);
        using ResolveForm = RE::TESBoundObject* (*)(void*, std::uint32_t*);
        using ResolveIndex = std::uint32_t* (*)(RE::Actor*, std::uint32_t*, const RE::BGSEquipSlot*);
        using ModelIndex = std::uint32_t (*)(RE::Actor*, RE::TESBoundObject*, std::uint32_t);
        using GetAttachment = RE::NiAVObject* (*)(void*, RE::Actor*, const void* const*, std::uint32_t);
        using QueuesEquip = bool (*)(RE::Actor*, RE::TESForm*);

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
        QueuesEquip s_queuesEquip = nullptr;
        bool s_installed = false; // Load owner; immutable after hook installation.
        std::atomic<std::uint64_t> s_transaction{ 0 }; // Input/UI -> frame diagnostic publication.
        // Input/UI writer, frame reader: fields first, then the sequence with
        // release; the reader acquires the sequence before the fields.
        std::atomic<std::uint32_t> s_directedForm{ 0 };
        std::atomic<std::uintptr_t> s_directedInstance{ 0 };
        std::atomic<bool> s_directedLeft{ false };
        std::atomic<std::uint64_t> s_directedSequence{ 0 };
        std::atomic<std::uint64_t> s_directedApplied{ 0 }; // Frame writer, UI reader.

        // Pip-Boy support toggle: same publication discipline as the directed
        // primary (UI writer, frame reader).
        std::atomic<std::uint32_t> s_supportForm{ 0 };
        std::atomic<std::uintptr_t> s_supportInstance{ 0 };
        std::atomic<bool> s_supportAttach{ false };
        std::atomic<std::uint64_t> s_supportSequence{ 0 };
        std::atomic<std::uint64_t> s_supportApplied{ 0 }; // Frame writer, UI reader.

        void publishSupportRequest(std::uint32_t form, std::uintptr_t instance, bool attach) noexcept
        {
            s_supportForm.store(form, std::memory_order_relaxed);
            s_supportInstance.store(instance, std::memory_order_relaxed);
            s_supportAttach.store(attach, std::memory_order_relaxed);
            s_supportSequence.fetch_add(1, std::memory_order_release);
        }

        void publishDirectedPrimary(std::uint32_t form, std::uintptr_t instance, bool left) noexcept
        {
            s_directedForm.store(form, std::memory_order_relaxed);
            s_directedInstance.store(instance, std::memory_order_relaxed);
            s_directedLeft.store(left, std::memory_order_relaxed);
            s_directedSequence.fetch_add(1, std::memory_order_release);
        }

        // Hand of the index-1 weapon. Written by the Pip-Boy transaction
        // (UI thread) and by the frame owner when it derives a hand for an
        // unrecorded item; read by both. The SRW lock guards only these
        // fields; no native call is made while it is held.
        struct SecondaryHandRecord
        {
            std::uint32_t form = 0;
            std::uintptr_t instance = 0;
            bool left = true;
        };

        SRWLOCK s_secondaryHandLock = SRWLOCK_INIT;
        SecondaryHandRecord s_secondaryHand{};

        void recordSecondaryHand(std::uint32_t form, std::uintptr_t instance, bool left) noexcept
        {
            AcquireSRWLockExclusive(&s_secondaryHandLock);
            s_secondaryHand = { form, instance, left };
            ReleaseSRWLockExclusive(&s_secondaryHandLock);
        }

        // The first weapon's saved hand, waiting for the loaded item. Written
        // by the save-load callbacks, consumed by the frame owner; the lock
        // guards only these fields.
        struct PendingPrimaryRestore
        {
            std::uint32_t form = 0;
            bool left = false;
            bool supported = false;
            std::uint64_t transaction = 0; // s_transaction when restored.
        };

        SRWLOCK s_restoreLock = SRWLOCK_INIT;
        PendingPrimaryRestore s_pendingPrimary{};

        /*
         * Hand holding the index-0 weapon, written by the frame owner from
         * its settled grip outside menus (recordPrimaryHand) and when a
         * directed hand is applied. Menus release the live firing grip and
         * its input routing, so Pip-Boy readers must use this record: the
         * live routing reported every left-held weapon as right (runtime
         * 14 Oct 21:25:03). Same lock discipline as the secondary record.
         */
        SRWLOCK s_primaryHandLock = SRWLOCK_INIT;
        PrimaryHand s_primaryHand{};

        // An item without instance data is identified by its form alone.
        bool sameItem(std::uint32_t form, std::uintptr_t instance, std::uint32_t otherForm,
            std::uintptr_t otherInstance) noexcept
        {
            return form != 0 && form == otherForm && (!instance || !otherInstance || instance == otherInstance);
        }

        struct WornWeapon
        {
            std::uint32_t form = 0;
            std::uintptr_t instance = 0;
        };

        std::array<WornWeapon, 2> wornWeapons(RE::Actor* actor) noexcept;

        // The hand of this exact index-0 item: a directed Pip-Boy hand not
        // yet applied, else its recorded hand, else the hand the frame owner
        // will give it (opposite a recorded second weapon, else the default
        // hand).
        PrimaryHand resolveRecordedPrimaryHand(std::uint32_t form, std::uintptr_t instance) noexcept
        {
            const auto directed = lastDirectedPrimaryEquip();
            if (directed.sequence != s_directedApplied.load(std::memory_order_acquire) &&
                sameItem(directed.form, directed.instance, form, instance))
                return { form, instance, directed.left, true };
            AcquireSRWLockShared(&s_primaryHandLock);
            const auto record = s_primaryHand;
            ReleaseSRWLockShared(&s_primaryHandLock);
            if (sameItem(record.form, record.instance, form, instance))
                return record;
            const auto worn = wornWeapons(f4vr::getPlayer());
            AcquireSRWLockShared(&s_secondaryHandLock);
            const auto secondary = s_secondaryHand;
            ReleaseSRWLockShared(&s_secondaryHandLock);
            const bool left = sameItem(secondary.form, secondary.instance, worn[1].form, worn[1].instance)
                ? !secondary.left
                : g_rockConfig.rockDefaultHandLeft;
            return { form, instance, left, true };
        }

        // A support toggle not yet applied to this item: 1 attach, 0 detach, -1 none.
        int pendingSupport(std::uint32_t form, std::uintptr_t instance) noexcept
        {
            const auto sequence = s_supportSequence.load(std::memory_order_acquire);
            if (!sequence || sequence == s_supportApplied.load(std::memory_order_acquire) ||
                !sameItem(s_supportForm.load(std::memory_order_relaxed),
                    s_supportInstance.load(std::memory_order_relaxed), form, instance))
                return -1;
            return s_supportAttach.load(std::memory_order_relaxed) ? 1 : 0;
        }

        // As above, with a support toggle not yet applied taken as done.
        PrimaryHand resolvePrimaryHand(std::uint32_t form, std::uintptr_t instance) noexcept
        {
            auto hand = resolveRecordedPrimaryHand(form, instance);
            if (const int pending = pendingSupport(form, instance); pending >= 0)
                hand.supported = pending == 1;
            return hand;
        }

        // The recorded hand for this exact index-1 item, else the hand that
        // does not hold index 0 (recorded so it stays put afterwards).
        bool resolveSecondaryHand(std::uint32_t form, std::uintptr_t instance) noexcept
        {
            AcquireSRWLockShared(&s_secondaryHandLock);
            const auto record = s_secondaryHand;
            ReleaseSRWLockShared(&s_secondaryHandLock);
            if (sameItem(record.form, record.instance, form, instance))
                return record.left;
            const auto worn = wornWeapons(f4vr::getPlayer());
            const bool left = !(worn[0].form && resolvePrimaryHand(worn[0].form, worn[0].instance).left);
            if (form)
                recordSecondaryHand(form, instance, left);
            return left;
        }

        std::array<WornWeapon, 2> wornWeapons(RE::Actor* actor) noexcept
        {
            std::array<WornWeapon, 2> worn{};
            auto* middle = actor && actor->currentProcess ? actor->currentProcess->middleHigh : nullptr;
            if (!middle)
                return worn;
            RE::BSAutoLock<RE::BSSpinLock> lock{ middle->equippedItemsLock };
            if (middle->equippedItems.size() > 32)
                return worn;
            for (const auto& entry : middle->equippedItems) {
                const auto index = entry.equipIndex.index;
                if (index >= worn.size() || !entry.item.object || !entry.item.object->IsWeapon())
                    continue;
                worn[index] = { entry.item.object->formID,
                    reinterpret_cast<std::uintptr_t>(entry.item.instanceData.get()) };
            }
            return worn;
        }

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
            bool nativeQueued = false; // UseItem would have queued it for the equip animation.
        };

        // Borrowed only for one synchronous native inventory transaction.
        // Nested/native unrelated equipment cannot reuse the consumed choice.
        thread_local ExecutionContext* s_execution = nullptr;

        template <class T>
        class ScopedValue
        {
        public:
            ScopedValue(T& destination, T value) noexcept :
                _destination(destination), _previous(std::exchange(destination, value))
            {}

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
            slots = { RE::TESForm::GetFormByID<RE::BGSEquipSlot>(0x13F42),
                RE::TESForm::GetFormByID<RE::BGSEquipSlot>(0x13F43),
                RE::TESForm::GetFormByID<RE::BGSEquipSlot>(0x13F44) };
            std::uint32_t rightIndex = ~0u, leftIndex = ~0u, flags = 0;
            if (!slots.right || !slots.left || !slots.either || !hasHandChildren(slots.either, slots) ||
                !native_memory::tryReadField(slots.either, 0x38, flags) || (flags & 1u))
                return false;
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
            if (object != context.weapon || !context.either)
                return original;
            Slots slots{};
            auto* player = f4vr::getPlayer();
            const bool eligible = player && resolveSlots(player, slots);
            try {
                ROCK_LOG_INFO(Weapon,
                    "AKIMBO toggle slot inputToken={} weapon={:08X} instance={:#x} original={:08X} either={:08X} "
                    "eligible={}",
                    native_pipboy_hand_input::current().sequence, object->formID,
                    reinterpret_cast<std::uintptr_t>(instance), original ? original->formID : 0, context.either->formID,
                    eligible);
            } catch (...) {
                OutputDebugStringA("ROCK: native toggle slot diagnostic failed\n");
            }
            if (!eligible)
                return original;
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

        const RE::BGSEquipSlot* executionSlot(void* component, RE::TBO_InstanceData* instance, RE::Actor* actor,
            const RE::BGSObjectInstance* item, const RE::BGSEquipSlot* requested) noexcept
        {
            using Getter = const RE::BGSEquipSlot* (*)(void*, RE::TBO_InstanceData*);
            const void* vtable = nullptr;
            Getter getter = nullptr;
            if (!native_memory::tryReadField(component, 0, vtable) ||
                !native_memory::tryReadField(vtable, 0x38, getter) || !getter)
                return nullptr;
            const auto* original = getter(component, instance);
            auto* context = s_execution;
            const bool matches = context && context->eligible && !context->consumed && context->actor == actor &&
                item && item->object == context->weapon && requested == context->slot &&
                (!context->instance || item->instanceData.get() == context->instance);
            if (!matches) {
                if (context && !context->consumed) {
                    try {
                        ROCK_LOG_WARN(Weapon,
                            "AKIMBO execution context mismatch inputToken={} eligible={} actorMatch={} itemMatch={} "
                            "instanceMatch={} requested={:08X} expected={:08X}",
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
                    "AKIMBO execution slot inputToken={} weapon={:08X} instance={:#x} requested={:08X} original={:08X} "
                    "selected={:08X} raceFlags2={:08X}",
                    origin.sequence, item->object->formID, reinterpret_cast<std::uintptr_t>(item->instanceData.get()),
                    requested ? requested->formID : 0, original ? original->formID : 0,
                    context->slot ? context->slot->formID : 0, flags);
            } catch (...) {
                OutputDebugStringA("ROCK: native execution slot diagnostic failed\n");
            }
            return context->slot;
        }

        /*
         * UseItem (E70920) asks 140DF5050 once, at 140E70A79, whether its
         * equip waits for the actor's equip animation, and passes that answer
         * to every EquipObject and UnequipObject of the transaction. It is
         * yes only while no game-pausing menu is open (UI menuMode 0): the
         * wrist Pip-Boy, never the projected one. The process drain
         * (EC4BE0/EC4DA0) builds a queued equip later, outside this
         * transaction, where E72DA0 ignores the queued hand leaf (race bit 19)
         * and builds the authored slot: a left equip replaced the right gun
         * at index 0 after the native swap had sheathed it. A hand-directed
         * equip completes inside its transaction, as with the projected
         * Pip-Boy; every other UseItem keeps the native answer.
         */
        bool useItemQueuesEquip(RE::Actor* actor, RE::TESForm* object) noexcept
        {
            const bool queues = s_queuesEquip(actor, object);
            auto* execution = s_execution;
            if (!execution || execution->actor != actor || execution->weapon != object)
                return queues;
            execution->nativeQueued = queues;
            return false;
        }

        void canonicalizeRestoredPrimary(RE::Actor* actor, const Slots& slots) noexcept
        {
            // Every equip made while installed is constructed on its hand leaf
            // (native_inventory_hand_slots). A saved game instead restores the
            // primary's EquippedItem with its authored multi-hand slot
            // (heldOnRightLeaf); the reconstruction path rebuilds data only when
            // the item changed. That slot contains the left leaf, so a left
            // equip's native conflict pass would unequip the primary. Re-seat
            // it on the right leaf, the slot its construction would have used.
            auto* middle = actor && actor->currentProcess ? actor->currentProcess->middleHigh : nullptr;
            if (!middle)
                return;
            RE::BSAutoLock<RE::BSSpinLock> lock{ middle->equippedItemsLock };
            for (auto& entry : middle->equippedItems) {
                if (entry.equipIndex.index != 0 || !entry.equipSlot || !entry.item.object ||
                    !entry.item.object->IsWeapon())
                    continue;
                const auto slot = entry.equipSlot->formID;
                if (!native_inventory_hand_slots::heldOnRightLeaf(entry.equipSlot))
                    continue;
                const auto* data = entry.item.instanceData
                    ? static_cast<const RE::TESObjectWEAP::InstanceData*>(entry.item.instanceData.get())
                    : &static_cast<const RE::TESObjectWEAP*>(entry.item.object)->weaponData;
                if (!weapon_type_policy::occupiesHandSlot(data->type.get()))
                    continue;
                entry.equipSlot = slots.right;
                try {
                    ROCK_LOG_INFO(Weapon,
                        "AKIMBO restored primary re-seated weapon={:08X} instance={:#x} authoredSlot={:08X} "
                        "actualSlot={:08X}",
                        entry.item.object->formID, reinterpret_cast<std::uintptr_t>(entry.item.instanceData.get()),
                        slot, slots.right->formID);
                } catch (...) {
                    OutputDebugStringA("ROCK: restored primary diagnostic failed\n");
                }
            }
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
                ROCK_LOG_WARN(Weapon, "AKIMBO left equip refused: native biped slot {} occupied by {:08X}", index,
                    occupant->formID);
                return false;
            }
            return true;
        }

        bool useItem(RE::ActorEquipManager* manager, RE::Actor* actor, std::uint32_t* handle, std::int32_t stack,
            const RE::BGSEquipSlot* requestedSlot, bool toggle, bool force) noexcept
        {
            const native_inventory_hand_slots::Scope inventoryOwner(actor);
            auto* player = f4vr::getPlayer();
            const auto origin = native_pipboy_hand_input::current();
            // Several weapons may always be equipped: the physical hand that
            // pressed decides. Equips without provenance stay native.
            const auto hand = origin.hand;
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
                        inventoryHandles) ||
                    !inventoryHandles)
                    return false;
                auto* object = s_resolveForm(inventoryHandles, handle);
                // Guns and melee weapons take a hand; throwables stay native.
                if (!object || object->formType != RE::ENUM_FORM_ID::kWEAP ||
                    !weapon_type_policy::occupiesHandSlot(
                        static_cast<RE::TESObjectWEAP*>(object)->weaponData.type.get()))
                    return s_useItem(manager, actor, handle, stack, requestedSlot, toggle, force);

                Slots slots{};
                if (!resolveSlots(actor, slots)) {
                    ROCK_LOG_WARN(Weapon, "AKIMBO equip refused: native player hand-slot layout unavailable");
                    return false;
                }
                const auto formID = object->formID;
                const auto inventoryHandle = *handle;
                InventorySelection selection{};
                s_readSelection(inventoryHandles, handle, stack, &selection);
                if (selection.object != object || selection.count == 0) {
                    ROCK_LOG_WARN(Weapon,
                        "AKIMBO equip refused: selected native inventory stack unavailable weapon={:08X} stack={}",
                        formID, stack);
                    return false;
                }
                const auto instance = reinterpret_cast<std::uintptr_t>(selection.instance.get());

                // The equip slot follows the weapon, not the hand: index 0 is
                // the player's own graph and index 1 ROCK's second graph, in
                // either hand. The acting hand chooses which weapon it holds.
                const bool actingLeft = hand == Hand::Left;
                const auto worn = wornWeapons(actor);
                const bool secondaryLeft = worn[1].form && resolveSecondaryHand(worn[1].form, worn[1].instance);
                // Includes a hand chosen earlier in this Pip-Boy session,
                // applied only once the menu closes.
                const auto primary = resolvePrimaryHand(worn[0].form, worn[0].instance);
                namespace allocation = equip_slot_allocation;
                const auto handOf = [](bool left) { return left ? allocation::Hand::Left : allocation::Hand::Right; };
                /*
                 * The pressed row is the item a slot wears when that slot's
                 * worn stack is on the row. A worn copy has its own row
                 * (native_pipboy_worn_rows), and identical copies share one
                 * TBO_InstanceData, so form and instance would name the worn
                 * copy for a press on the spare copies' row too.
                 */
                auto* weapon = static_cast<RE::TESObjectWEAP*>(object);
                const auto row = weapon_equip_transfer::pipboyRowCopies(weapon, static_cast<std::uint32_t>(stack));
                const auto wears = [&](std::uint32_t index) {
                    if (!worn[index].form)
                        return false;
                    if (row.valid)
                        return row.wornStack[index] >= 0;
                    return sameItem(worn[index].form, worn[index].instance, formID, instance);
                };
                const auto decision = allocation::decide(handOf(actingLeft),
                    { {
                        // A support-held (detached) slot 0 has no firing hand,
                        // only the hand carrying it.
                        { .worn = worn[0].form != 0,
                            .firingHand = primary.firing ? handOf(primary.left) : allocation::Hand::None,
                            .holdingHand = primary.firing ? allocation::Hand::None : handOf(primary.left),
                            .requestedItem = wears(0) },
                        { .worn = worn[1].form != 0, .firingHand = handOf(secondaryLeft), .requestedItem = wears(1) },
                    } });
                if (decision.action == allocation::Action::ToggleSupport) {
                    const int target = static_cast<int>(decision.slot);
                    const int displacedIndex = decision.displacedSlot;
                    // Only the first weapon takes a support grip: the second is
                    // carried by its firing grip alone, unless the weapon this
                    // hand holds leaves index 0 and the second is promoted.
                    if (target != 0 && displacedIndex != 0) {
                        ROCK_LOG_INFO(Weapon, "AKIMBO Pip-Boy support refused hand={} weapon={:08X}: second weapon",
                            handName(hand), formID);
                        return false;
                    }
                    if (displacedIndex >= 0) {
                        if (!weapon_equip_transfer::unequipEquippedIndex(static_cast<std::uint32_t>(displacedIndex),
                                worn[displacedIndex].form, worn[displacedIndex].instance)) {
                            ROCK_LOG_WARN(Weapon,
                                "AKIMBO Pip-Boy support refused hand={} weapon={:08X}: its weapon {:08X} at index {} "
                                "did not unequip",
                                handName(hand), formID, worn[displacedIndex].form, displacedIndex);
                            return false;
                        }
                        native_equipped_model_lifetime::noteEquipmentChange();
                    }
                    // A promotion re-equips the weapon, so it is then named by form.
                    const bool attach = displacedIndex >= 0 || !primary.supported;
                    publishSupportRequest(worn[target].form, target == 0 ? worn[target].instance : 0u, attach);
                    ROCK_LOG_INFO(Weapon,
                        "AKIMBO Pip-Boy support request inputToken={} hand={} weapon={:08X} index={} displacedIndex={} "
                        "attach={} rowCopies={}",
                        origin.sequence, handName(hand), formID, target, displacedIndex, attach, row.copies);
                    return true;
                }
                if (decision.action == allocation::Action::MoveFiringGrip) {
                    const int wornIndex = static_cast<int>(decision.slot);
                    const int actingIndex = decision.displacedSlot;
                    // Cross-hand press: the item moves to the pressing hand
                    // without a re-equip. A weapon whose firing grip that hand
                    // held returns to inventory.
                    const bool displaced = actingIndex < 0 ||
                        weapon_equip_transfer::unequipEquippedIndex(static_cast<std::uint32_t>(actingIndex),
                            worn[actingIndex].form, worn[actingIndex].instance);
                    if (displaced) {
                        if (wornIndex == 0)
                            publishDirectedPrimary(worn[0].form, worn[0].instance, actingLeft);
                        else
                            recordSecondaryHand(worn[1].form, worn[1].instance, actingLeft);
                        if (actingIndex >= 0)
                            native_equipped_model_lifetime::noteEquipmentChange();
                    }
                    const auto sequence = s_transaction.fetch_add(1, std::memory_order_release) + 1;
                    ROCK_LOG_INFO(Weapon,
                        "AKIMBO cross-hand move request={} inputToken={} hand={} weapon={:08X} index={} "
                        "displacedIndex={} accepted={}",
                        sequence, origin.sequence, handName(hand), formID, wornIndex, actingIndex, displaced);
                    return displaced;
                }
                const std::uint32_t index = decision.slot;
                const bool toggles = wears(index);
                if (index == 1 && !leftBipedAvailable(player, static_cast<RE::TESObjectWEAP*>(object)))
                    return false;
                if (index == 1)
                    canonicalizeRestoredPrimary(actor, slots);
                const auto* selected = index == 0 ? slots.right : slots.left;
                ExecutionContext execution{ actor, object, selected };
                const ScopedValue executionScope{ s_execution, &execution };
                const ScopedValue context{ s_toggle, ToggleContext{ object, slots.either } };
                const bool affectsPair = index == 1 || worn[1].form != 0;
                /*
                 * The native toggle mistakes a copy of the same base form in
                 * the other hand for this one (bench: the left toggle-off
                 * re-equipped the left copy, or emptied the right hand), so
                 * that toggle-off unequips the exact copy on its own leaf.
                 */
                const bool sameFormOther = toggles && worn[1 - index].form == formID;
                if (sameFormOther)
                    accepted = weapon_equip_transfer::unequipEquippedIndex(index, worn[index].form, worn[index].instance);
                else
                    accepted = s_useItem(manager, actor, handle, stack, selected, toggle, force);
                if (accepted && affectsPair)
                    native_equipped_model_lifetime::noteEquipmentChange();
                // The equip completed inside this transaction (unqueued) and
                // the merge pass put the other copies on the worn stack: give
                // the equipped copy its own stack and Pip-Boy row.
                if (accepted && !toggles)
                    weapon_equip_transfer::separateSpareCopiesFromWornStacks();
                if (accepted && !toggles) {
                    if (index == 0)
                        publishDirectedPrimary(formID, instance, actingLeft);
                    else
                        recordSecondaryHand(formID, instance, actingLeft);
                    // A hand that takes a second weapon leaves the first one's support grip.
                    if (index == 1 && primary.firing && primary.supported && actingLeft != primary.left)
                        publishSupportRequest(worn[0].form, worn[0].instance, false);
                }
                // Native return means request accepted, not model/graph ready.
                const auto sequence = s_transaction.fetch_add(1, std::memory_order_release) + 1;
                ROCK_LOG_INFO(Weapon,
                    "AKIMBO equip request={} inputToken={} hand={} weapon={:08X} handle={:08X} stack={} rowCopies={} "
                    "index={} toggleOff={} exactUnequip={} requestedSlot={:08X} accepted={} executionSelected={} "
                    "nativeQueued={} nativeInventory=true",
                    sequence, origin.sequence, handName(hand), formID, inventoryHandle, stack, row.copies, index,
                    toggles, sameFormOther, selected->formID, accepted, execution.consumed, execution.nativeQueued);
                return accepted;
            } catch (...) {
                // Never retry an inventory mutation after an exception: it
                // could already have succeeded. No exception crosses native.
                OutputDebugStringA("ROCK: hand-directed Pip-Boy transaction diagnostic failed\n");
                return accepted;
            }
        }

        /*
         * AttackBlockHandler (FC8340) skips the right attack of a non-automatic
         * gun when 141133BB0 reports an equipped melee weapon. That scan visits
         * every equipped WEAP (14113F710, type < 7), so a melee second weapon at
         * index 1 cancelled the primary gun's trigger. For the player only the
         * primary decides at this call site; the VR melee callers of 141133BB0
         * keep the full scan. 141133AE0 is its unchanged empty-slot fallback.
         */
        using ActorTest = bool (*)(RE::Actor*);
        ActorTest s_meleeEquipped = nullptr;
        ActorTest s_defaultSlotEmpty = nullptr;

        bool triggerMeleeEquipped(RE::Actor* actor) noexcept
        {
            if (!actor || actor != f4vr::getPlayer())
                return s_meleeEquipped(actor);
            auto* primary = f4vr::getEquippedWeaponItem(0);
            auto* weapon = primary && primary->item.object ? primary->item.object->As<RE::TESObjectWEAP>() : nullptr;
            return (weapon && weapon_type_policy::isEquippedMelee(weapon->weaponData.type.get())) ||
                s_defaultSlotEmpty(actor);
        }

        template <std::size_t N>
        bool bytesMatch(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(rva).address()),
                       actual.data(), actual.size()) &&
                actual == expected;
        }

        // A function ROCK calls or resumes inside: its prologue, or another plugin's
        // detour at the entry.
        template <std::size_t N>
        bool callable(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected) noexcept
        {
            return static_cast<bool>(RPS::Runtime::HookChain::resolveEntry(REL::Offset(rva).address(), expected));
        }

        bool callRedirectMatches(std::uintptr_t rva, std::uintptr_t destination, std::uint8_t opcode = 0xE8) noexcept
        {
            const auto address = REL::Offset(rva).address();
            std::array<std::uint8_t, 5> call{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), call.data(),
                    call.size()) ||
                call[0] != opcode)
                return false;
            std::int32_t displacement = 0;
            std::memcpy(&displacement, call.data() + 1, sizeof(displacement));
            const auto stubAddress = address + 5 + displacement;
            std::array<std::uint8_t, 14> stub{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(stubAddress), stub.data(),
                    stub.size()))
                return false;
            constexpr std::array<std::uint8_t, 6> jump{ 0xFF, 0x25, 0, 0, 0, 0 };
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
        if (!player)
            return nullptr;
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
        if (native_memory::tryReadField(player, 0, vtable) && native_memory::tryReadField(vtable, 0x508, getBiped) &&
            getBiped)
            native_memory::tryReadValue(getBiped(player, false), biped);
        return biped;
    }

    std::array<EquippedView, 2> equippedViews() noexcept
    {
        std::array<EquippedView, 2> views{};
        if (!s_installed)
            return views;
        auto* player = f4vr::getPlayer();
        auto* middle = player && player->currentProcess ? player->currentProcess->middleHigh : nullptr;
        if (!middle || middle->equippedItems.size() > 32)
            return views;
        const auto* biped = playerBiped(true);
        const auto* bodyBiped = playerBiped(false);
        for (const auto& entry : middle->equippedItems) {
            const auto index = entry.equipIndex.index;
            if (index >= views.size() || !entry.item.object || !entry.item.object->IsWeapon())
                continue;
            auto* weapon = static_cast<RE::TESObjectWEAP*>(entry.item.object);
            auto* instance = entry.item.instanceData.get();
            const auto* data =
                instance ? static_cast<const RE::TESObjectWEAP::InstanceData*>(instance) : &weapon->weaponData;
            if (!weapon_type_policy::occupiesHandSlot(data->type.get()))
                continue;
            auto& view = views[index];
            // Duplicate native indices are ambiguous, never choose an arbitrary root.
            if (view.weapon)
                return {};
            view.weapon = weapon;
            view.firearm = data->type == RE::WEAPON_TYPE::kGun;
            view.instance = instance;
            view.data = reinterpret_cast<std::uintptr_t>(entry.data.get());
            view.slot = entry.equipSlot ? entry.equipSlot->formID : 0;
            const auto modelIndex = s_modelIndex(player, weapon, index);
            if (modelIndex >= 44)
                continue;
            const auto readModel = [&](const void* source, RE::NiAVObject*& root) {
                if (!source)
                    return;
                // One guarded copy of the record's form (+10), instance (+18)
                // and root (+40) span: each guarded read costs a page query,
                // and this view is rebuilt many times per frame.
                std::array<std::byte, 0x38> fields{};
                const auto* record = static_cast<const std::byte*>(source) + modelIndex * 0x58;
                if (!native_memory::guardedCopyFromMemory(record + 0x10, fields.data(), fields.size()))
                    return;
                RE::TESBoundObject* modelForm = nullptr;
                RE::TBO_InstanceData* modelInstance = nullptr;
                std::memcpy(&modelForm, fields.data(), sizeof(modelForm));
                std::memcpy(&modelInstance, fields.data() + 0x08, sizeof(modelInstance));
                if (modelForm == weapon && modelInstance == instance)
                    std::memcpy(&root, fields.data() + 0x30, sizeof(root));
            };
            readModel(biped, view.root);
            if (bodyBiped != biped)
                readModel(bodyBiped, view.bodyRoot);
        }
        return views;
    }

    DirectedEquip lastDirectedPrimaryEquip() noexcept
    {
        DirectedEquip result{};
        result.sequence = s_directedSequence.load(std::memory_order_acquire);
        if (!result.sequence)
            return {};
        result.form = s_directedForm.load(std::memory_order_relaxed);
        result.instance = s_directedInstance.load(std::memory_order_relaxed);
        result.left = s_directedLeft.load(std::memory_order_relaxed);
        return result;
    }

    bool secondaryEquipInProgress(const RE::Actor* actor) noexcept
    {
        const auto* execution = s_execution;
        return execution && actor && execution->actor == actor && execution->slot && execution->slot->formID == 0x13F43;
    }

    bool equipIntoSecondary(RE::Actor* actor, RE::TESObjectWEAP* weapon, RE::TBO_InstanceData* instance, bool left,
        SecondaryEquipCall equip, void* context) noexcept
    {
        auto* player = f4vr::getPlayer();
        if (!s_installed || !actor || actor != player || !weapon || !equip)
            return false;
        bool accepted = false;
        try {
            Slots slots{};
            if (!resolveSlots(actor, slots)) {
                ROCK_LOG_WARN(Weapon, "AKIMBO second weapon equip refused: native player hand-slot layout unavailable");
                return false;
            }
            if (!leftBipedAvailable(player, weapon))
                return false;
            canonicalizeRestoredPrimary(actor, slots);
            const native_inventory_hand_slots::Scope inventoryOwner(actor);
            ExecutionContext execution{ actor, weapon, slots.left, instance };
            execution.eligible = true;
            const ScopedValue executionScope{ s_execution, &execution };
            accepted = equip(slots.left, context);
            if (accepted) {
                native_equipped_model_lifetime::noteEquipmentChange();
                recordSecondaryHand(weapon->formID, reinterpret_cast<std::uintptr_t>(instance), left);
            }
            const auto sequence = s_transaction.fetch_add(1, std::memory_order_release) + 1;
            ROCK_LOG_INFO(Weapon,
                "AKIMBO second weapon equip request={} hand={} weapon={:08X} instance={:#x} accepted={} "
                "executionSelected={}",
                sequence, left ? "left" : "right", weapon->formID, reinterpret_cast<std::uintptr_t>(instance), accepted,
                execution.consumed);
        } catch (...) {
            OutputDebugStringA("ROCK: second weapon equip diagnostic failed\n");
        }
        return accepted;
    }

    WornHand wornRowHand(std::uint32_t form, int slot, bool& outFiring, bool& outSupported) noexcept
    {
        outSupported = false;
        outFiring = true;
        if (!s_installed || !form || (slot != 0 && slot != 1))
            return WornHand::None;
        const auto worn = wornWeapons(f4vr::getPlayer());
        const auto hand = [](bool left) { return left ? WornHand::Left : WornHand::Right; };
        const int index = worn[slot].form == form ? slot : worn[1 - slot].form == form ? 1 - slot : -1;
        if (index < 0)
            return WornHand::None;
        // The second weapon is carried only by its firing grip, unless a
        // support attach that promotes it is pending.
        if (index == 1) {
            outSupported = pendingSupport(worn[1].form, worn[1].instance) == 1;
            return hand(resolveSecondaryHand(worn[1].form, worn[1].instance));
        }
        // Same hand rule as Pip-Boy allocation; a support-held primary shows
        // the hand that holds it.
        const auto primary = resolvePrimaryHand(worn[0].form, worn[0].instance);
        outFiring = primary.firing;
        outSupported = primary.firing && primary.supported;
        return hand(primary.left);
    }

    void recordPrimaryHand(std::uint32_t form, std::uintptr_t instance, bool left, bool firing, bool supported) noexcept
    {
        if (!form)
            return;
        AcquireSRWLockExclusive(&s_primaryHandLock);
        const bool changed = !sameItem(s_primaryHand.form, s_primaryHand.instance, form, instance) ||
            s_primaryHand.left != left || s_primaryHand.firing != firing || s_primaryHand.supported != supported;
        s_primaryHand = { form, instance, left, firing, supported };
        ReleaseSRWLockExclusive(&s_primaryHandLock);
        if (changed)
            ROCK_LOG_INFO(Weapon,
                "Primary weapon hand recorded weapon={:08X} instance={:#x} hand={} role={} supported={}", form,
                instance, left ? "left" : "right", firing ? "firing" : "support", supported);
    }

    SupportRequest pendingSupportRequest() noexcept
    {
        const auto sequence = s_supportSequence.load(std::memory_order_acquire);
        if (!sequence || sequence == s_supportApplied.load(std::memory_order_acquire))
            return {};
        return { s_supportForm.load(std::memory_order_relaxed), s_supportInstance.load(std::memory_order_relaxed),
            s_supportAttach.load(std::memory_order_relaxed), sequence };
    }

    void noteSupportRequestApplied(std::uint64_t sequence, bool supported) noexcept
    {
        // The item may have been equipped after the last record (inside the
        // menu): record its hand with the result, or the display loses it.
        const auto form = s_supportForm.load(std::memory_order_relaxed);
        const auto instance = s_supportInstance.load(std::memory_order_relaxed);
        const auto worn = wornWeapons(f4vr::getPlayer());
        if (sameItem(worn[0].form, worn[0].instance, form, instance)) {
            auto hand = resolveRecordedPrimaryHand(worn[0].form, worn[0].instance);
            AcquireSRWLockExclusive(&s_primaryHandLock);
            s_primaryHand = { worn[0].form, worn[0].instance, hand.left, hand.firing, hand.firing && supported };
            ReleaseSRWLockExclusive(&s_primaryHandLock);
        }
        s_supportApplied.store(sequence, std::memory_order_release);
    }

    SavedHands savedHands() noexcept
    {
        SavedHands saved{};
        AcquireSRWLockShared(&s_primaryHandLock);
        const auto primary = s_primaryHand;
        ReleaseSRWLockShared(&s_primaryHandLock);
        AcquireSRWLockShared(&s_secondaryHandLock);
        const auto secondary = s_secondaryHand;
        ReleaseSRWLockShared(&s_secondaryHandLock);
        saved.primaryForm = primary.form;
        saved.primaryLeft = primary.left;
        saved.primaryFiring = primary.firing;
        saved.primarySupported = primary.supported;
        saved.secondaryForm = secondary.form;
        saved.secondaryLeft = secondary.left;
        return saved;
    }

    void restoreHands(const SavedHands& hands) noexcept
    {
        // Instance 0 matches any instance of the form (sameItem).
        recordSecondaryHand(hands.secondaryForm, 0, hands.secondaryLeft);
        AcquireSRWLockExclusive(&s_primaryHandLock);
        s_primaryHand = {};
        ReleaseSRWLockExclusive(&s_primaryHandLock);
        // A part-grip carry has no firing hand to direct.
        const bool primary = hands.primaryForm != 0 && hands.primaryFiring;
        AcquireSRWLockExclusive(&s_restoreLock);
        s_pendingPrimary = primary ? PendingPrimaryRestore{ hands.primaryForm, hands.primaryLeft,
            hands.primarySupported, s_transaction.load(std::memory_order_acquire) }
                                   : PendingPrimaryRestore{};
        ReleaseSRWLockExclusive(&s_restoreLock);
        if (hands.primaryForm || hands.secondaryForm) {
            ROCK_LOG_INFO(Weapon,
                "Saved weapon hands loaded first={:08X} hand={} supported={} firing={} second={:08X} hand={}",
                hands.primaryForm, hands.primaryLeft ? "left" : "right", hands.primarySupported, hands.primaryFiring,
                hands.secondaryForm, hands.secondaryLeft ? "left" : "right");
        }
    }

    void applyRestoredHands() noexcept
    {
        AcquireSRWLockShared(&s_restoreLock);
        const auto pending = s_pendingPrimary;
        ReleaseSRWLockShared(&s_restoreLock);
        if (!pending.form)
            return;
        const auto clear = [] {
            AcquireSRWLockExclusive(&s_restoreLock);
            s_pendingPrimary = {};
            ReleaseSRWLockExclusive(&s_restoreLock);
        };
        // An equip or cross-hand move chose a hand since the load.
        if (s_transaction.load(std::memory_order_acquire) != pending.transaction) {
            ROCK_LOG_INFO(Weapon, "Saved first weapon hand dropped weapon={:08X}: an equip chose a hand first",
                pending.form);
            clear();
            return;
        }
        const auto worn = wornWeapons(f4vr::getPlayer());
        if (!worn[0].form)
            return;
        clear();
        if (worn[0].form != pending.form) {
            ROCK_LOG_INFO(Weapon, "Saved first weapon hand dropped weapon={:08X}: {:08X} is worn", pending.form,
                worn[0].form);
            return;
        }
        publishDirectedPrimary(worn[0].form, worn[0].instance, pending.left);
        if (pending.supported)
            publishSupportRequest(worn[0].form, worn[0].instance, true);
        ROCK_LOG_INFO(Weapon, "Saved first weapon hand restored weapon={:08X} instance={:#x} hand={} supported={}",
            worn[0].form, worn[0].instance, pending.left ? "left" : "right", pending.supported);
    }

    void directPrimaryHand(std::uint32_t form, std::uintptr_t instance, bool left) noexcept
    {
        publishDirectedPrimary(form, instance, left);
    }

    void noteDirectedPrimaryApplied(std::uint64_t sequence) noexcept
    {
        const auto directed = lastDirectedPrimaryEquip();
        if (directed.sequence == sequence)
            recordPrimaryHand(directed.form, directed.instance, directed.left, true, false);
        s_directedApplied.store(sequence, std::memory_order_release);
    }

    bool hasSecondaryGun() noexcept { return equippedViews()[1].firearm; }

    bool secondaryHandIsLeft() noexcept
    {
        const auto view = equippedViews()[1];
        return resolveSecondaryHand(view.weapon ? view.weapon->formID : 0,
            reinterpret_cast<std::uintptr_t>(view.instance));
    }

    RE::NiAVObject* resolveAttachment(std::uint32_t index) noexcept
    {
        if (!s_installed || !s_getAttachment || index > 1)
            return nullptr;
        auto* player = f4vr::getPlayer();
        const void* biped = nullptr;
        if (!player || !player->currentProcess || !native_memory::tryReadField(player, 0xFE0, biped) || !biped)
            return nullptr;
        // EC5230 selects the race's equipment-node name and searches the
        // supplied biped root. EF5273 / EF5C1A use this same helper for the
        // player's first-person Weapon frame before native arm placement.
        return s_getAttachment(player->currentProcess, player, &biped, index);
    }

    void observe() noexcept
    {
        const auto sequence = s_transaction.load(std::memory_order_acquire);
        if (!sequence)
            return;
        try {
            std::array<Identity, 2> current{};
            std::array<std::uint32_t, 2> ammo{};
            auto* player = f4vr::getPlayer();
            auto* middle = player && player->currentProcess ? player->currentProcess->middleHigh : nullptr;
            if (middle && middle->equippedItems.size() <= 32) {
                for (const auto& entry : middle->equippedItems) {
                    const auto index = entry.equipIndex.index;
                    if (index >= current.size() || !entry.item.object || !entry.item.object->IsWeapon())
                        continue;
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
                    identity.bipedIndex =
                        s_modelIndex(player, static_cast<RE::TESObjectWEAP*>(entry.item.object), index);
                    if (biped && identity.bipedIndex < 44) {
                        const auto* record = static_cast<const std::byte*>(biped) + identity.bipedIndex * 0x58;
                        RE::TESBoundObject* form = nullptr;
                        RE::TBO_InstanceData* instance = nullptr;
                        if (native_memory::tryReadField(record, 0x10, form) && form == entry.item.object &&
                            native_memory::tryReadField(record, 0x18, instance) &&
                            instance == entry.item.instanceData.get()) {
                            native_memory::tryReadField(record, 0x40, identity.root);
                            native_memory::tryReadField(record, 0x50, identity.holder);
                            native_memory::tryReadField(reinterpret_cast<const void*>(identity.holder), 0x18,
                                identity.manager);
                        }
                    }
                }
            }
            if (sequence == s_observedTransaction && current == s_previous)
                return;
            s_observedTransaction = sequence;
            s_previous = current;
            for (std::size_t index = 0; index < current.size(); ++index) {
                const auto& item = current[index];
                ROCK_LOG_INFO(Weapon,
                    "AKIMBO native state request={} index={} weapon={:08X} slot={:08X} instance={:#x} data={:#x} "
                    "loaded={} bipedSlot={} root={:#x} holder={:#x} graphManager={:#x}",
                    sequence, index, item.form, item.slot, item.instance, item.data, ammo[index], item.bipedIndex,
                    item.root, item.holder, item.manager);
            }
        } catch (...) {
            OutputDebugStringA("ROCK: native hand equipment observation failed\n");
        }
    }

    bool install() noexcept
    {
        if (s_installed)
            return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72)
            return false;
        // The four patched calls reach the native function or another plugin's
        // detour; the hooks call through whichever is there.
        const auto callAt = [](std::uintptr_t site, std::uintptr_t target) {
            const auto resolved =
                RPS::Runtime::HookChain::resolveCall(REL::Offset(site).address(), REL::Offset(target).address());
            if (resolved.foreign())
                ROCK_LOG_INFO(Init, "Hand equipment call at {:#x} already hooked by {}; ROCK calls through it", site,
                    resolved.owner);
            return resolved.callee;
        };
        const auto useCallee = callAt(0xB9B904, 0xE70920);
        const auto slotCallee = callAt(0xE70A37, 0x146260);
        const auto triggerMeleeCallee = callAt(0xFC833B, 0x1133BB0);
        const auto queueCallee = callAt(0xE70A79, 0xDF5050);
        // Each call's bytes before ROCK patched it, for rollback.
        constexpr std::array<std::uintptr_t, 4> kCallSites{ 0xE70A37, 0xB9B904, 0xFC833B, 0xE70A79 };
        std::array<std::array<std::uint8_t, 5>, kCallSites.size()> callsBefore{};
        for (std::size_t i = 0; i < kCallSites.size(); ++i)
            std::memcpy(callsBefore[i].data(), reinterpret_cast<const void*>(REL::Offset(kCallSites[i]).address()), 5);
        constexpr std::array<std::uint8_t, 12> executionBytes{ 0x49, 0x8B, 0x06, 0x48, 0x8B, 0xD3, 0x49, 0x8B, 0xCE,
            0xFF, 0x50, 0x38 };
        constexpr std::array<std::uint8_t, 44> executionFrame{ 0x4C, 0x89, 0x4C, 0x24, 0x20, 0x4C, 0x89, 0x44, 0x24,
            0x18, 0x48, 0x89, 0x54, 0x24, 0x10, 0x48, 0x89, 0x4C, 0x24, 0x08, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41,
            0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0x6C, 0x24, 0xE1, 0x48, 0x81, 0xEC, 0xE8, 0, 0, 0 };
        if (!useCallee || !slotCallee || !bytesMatch(0xE72E19, executionBytes) || !triggerMeleeCallee ||
            !queueCallee ||
            !callable(0xDF5050,
                std::array<std::uint8_t, 14>{ 0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x20, 0x0F, 0xB6,
                    0x42, 0x1A }) ||
            !callable(0x1133BB0, std::array<std::uint8_t, 8>{ 0x40, 0x53, 0x48, 0x83, 0xEC, 0x20, 0x48, 0x8B }) ||
            !callable(0x1133AE0, std::array<std::uint8_t, 8>{ 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74 }) ||
            !callable(0xEC5230,
                std::array<std::uint8_t, 19>{ 0x44, 0x89, 0x4C, 0x24, 0x20, 0x53, 0x48, 0x83, 0xEC, 0x20, 0x48, 0x8B,
                    0x02, 0x48, 0x8B, 0xCA, 0x49, 0x8B, 0xD8 }) ||
            !callable(0x1ABBD0,
                std::array<std::uint8_t, 15>{ 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57, 0x48,
                    0x83, 0xEC, 0x30 }) ||
            !callable(0xE72DA0, executionFrame) ||
            !callable(0x1AB7A0,
                std::array<std::uint8_t, 9>{ 0x48, 0x83, 0xEC, 0x28, 0xE8, 0x07, 0xFF, 0xFF, 0xFF }) ||
            !callable(0x146260,
                std::array<std::uint8_t, 14>{ 0x40, 0x53, 0x48, 0x83, 0xEC, 0x20, 0x48, 0x8B, 0xDA, 0xE8, 0x82, 0xFE,
                    0xFF, 0xFF }) ||
            !callable(0x3E6840,
                std::array<std::uint8_t, 10>{ 0x48, 0x89, 0x5C, 0x24, 0x18, 0x56, 0x48, 0x83, 0xEC, 0x20 }) ||
            !callable(0x1CAFC0,
                std::array<std::uint8_t, 13>{ 0x48, 0x83, 0xEC, 0x28, 0x48, 0x85, 0xD2, 0x74, 0x2C, 0x80, 0x7A, 0x1A,
                    0x2B }) ||
            !callable(0xE70920,
                std::array<std::uint8_t, 16>{ 0x48, 0x8B, 0xC4, 0x48, 0x89, 0x50, 0x10, 0x55, 0x41, 0x54, 0x41, 0x56,
                    0x48, 0x8D, 0x68, 0xB9 })) {
            ROCK_LOG_ERROR(Init, "Native hand equipment hook validation failed; Pip-Boy unchanged");
            return false;
        }
        auto& trampoline = F4SE::GetTrampoline();
        if (trampoline.free_size() < 256)
            return false;
        s_useItem = reinterpret_cast<UseItem>(useCallee);
        s_getSlot = reinterpret_cast<GetSlot>(slotCallee);
        s_resolveForm = reinterpret_cast<ResolveForm>(REL::Offset(0x1AB7A0).address());
        s_resolveIndex = reinterpret_cast<ResolveIndex>(REL::Offset(0x3E6840).address());
        s_modelIndex = reinterpret_cast<ModelIndex>(REL::Offset(0x1CAFC0).address());
        s_getAttachment = reinterpret_cast<GetAttachment>(REL::Offset(0xEC5230).address());
        s_readSelection = reinterpret_cast<ReadSelection>(REL::Offset(0x1ABBD0).address());
        s_meleeEquipped = reinterpret_cast<ActorTest>(triggerMeleeCallee);
        s_defaultSlotEmpty = reinterpret_cast<ActorTest>(REL::Offset(0x1133AE0).address());
        s_queuesEquip = reinterpret_cast<QueuesEquip>(queueCallee);
        // A left inventory slot without its complete native first-person gun
        // model is not a usable hand-directed equipment capability.
        if (!native_inventory_hand_slots::install() || !native_secondary_model::install() ||
            !native_equipped_model_lifetime::install() || !native_primary_weapon_graph::install() ||
            !native_secondary_actions::install() || !native_secondary_melee::install() || !native_melee_wand::install())
            return false;
        try {
            native_execution_slot::Stub stub(reinterpret_cast<std::uintptr_t>(&executionSlot),
                REL::Offset(0xE72E25).address());
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
                !bytesMatch(0xE72E1E, std::array<std::uint8_t, 7>{ 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90 }))
                throw std::runtime_error("execution slot branch readback failed");
            (void)trampoline.write_call<5>(REL::Offset(0xE70A37).address(), &toggleSlot);
            (void)trampoline.write_call<5>(REL::Offset(0xB9B904).address(), &useItem);
            (void)trampoline.write_call<5>(REL::Offset(0xFC833B).address(), &triggerMeleeEquipped);
            (void)trampoline.write_call<5>(REL::Offset(0xE70A79).address(), &useItemQueuesEquip);
            if (!callRedirectMatches(0xE70A37, reinterpret_cast<std::uintptr_t>(&toggleSlot)) ||
                !callRedirectMatches(0xB9B904, reinterpret_cast<std::uintptr_t>(&useItem)) ||
                !callRedirectMatches(0xFC833B, reinterpret_cast<std::uintptr_t>(&triggerMeleeEquipped)) ||
                !callRedirectMatches(0xE70A79, reinterpret_cast<std::uintptr_t>(&useItemQueuesEquip)) ||
                !FlushInstructionCache(GetCurrentProcess(), nullptr, 0))
                throw std::runtime_error("hand equipment redirect verification failed");
            if (!native_pipboy_hand_input::install())
                throw std::runtime_error("queued Pip-Boy provenance installation failed");
            // Without it identical copies share a Pip-Boy row as in vanilla;
            // hand-directed equips still work.
            if (!native_pipboy_worn_rows::install())
                ROCK_LOG_WARN(Init, "Pip-Boy worn weapon rows unavailable: identical copies share one row");
            s_installed = true;
            ROCK_LOG_INFO(Init,
                "Native hand-directed Pip-Boy equipment installed; scoped execution-slot resolver, unqueued directed "
                "equips and independent equipped action hooks active");
            return true;
        } catch (...) {
            REL::safe_write(REL::Offset(0xE72E19).address(), executionBytes.data(), executionBytes.size());
            for (std::size_t i = 0; i < kCallSites.size(); ++i)
                REL::safe_write(REL::Offset(kCallSites[i]).address(), callsBefore[i].data(), callsBefore[i].size());
            FlushInstructionCache(GetCurrentProcess(), nullptr, 0);
            ROCK_LOG_ERROR(Init, "Native hand equipment installation failed; original hooks restored");
            return false;
        }
    }
}
