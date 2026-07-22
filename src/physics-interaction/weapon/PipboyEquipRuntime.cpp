#include "physics-interaction/weapon/PipboyEquipRuntime.h"

#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/PhysicsLog.h"

#include "RE/Bethesda/BGSInventoryInterface.h"
#include "RE/Bethesda/BGSInventoryItem.h"
#include "RE/Bethesda/IMenu.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Scaleform/GFx/GFx_Player.h"

#include <F4SE/F4SE.h>
#include <REL/Relocation.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>

namespace rock::pipboy_equip_runtime
{
    namespace
    {
        constexpr std::size_t kPipboyInventoryUpdateDataVtableIndex = 2;
        constexpr std::uint32_t kMaximumInventoryStackIndex = 4096;
        constexpr std::size_t kMaximumGfxTraversalDepth = 12;
        constexpr std::size_t kMaximumGfxTraversalNodes = 1024;

        using UseItem_t = void (*)(std::uint32_t, std::uint32_t, bool&, bool&);
        using UpdateData_t = void (*)(RE::PipboyInventoryMenu*);

        UseItem_t s_originalUseItem = nullptr;
        UpdateData_t s_originalUpdateData = nullptr;
        std::atomic<bool> s_hooksInstalled{ false };
        std::atomic<bool> s_leftHandEquipAvailable{ false };
        std::atomic<std::uint8_t> s_equipMode{
            static_cast<std::uint8_t>(pipboy_equip_policy::EquipMode::NativeRight)
        };
        thread_local bool s_insideUpdateDataHook = false;

        struct AtomicSelectionMailbox
        {
            std::atomic_flag writer = ATOMIC_FLAG_INIT;
            std::atomic<std::uint64_t> version{ 0 };
            std::atomic<std::uint64_t> sequence{ 0 };
            std::atomic<std::uint32_t> handleId{ 0 };
            std::atomic<std::uint32_t> stackId{ 0 };
            std::atomic<std::uint32_t> formId{ 0 };
            std::atomic<std::uint8_t> hand{ 0 };
            std::atomic<std::uint8_t> source{ 0 };
            std::atomic<bool> equipped{ false };
        };

        struct AtomicAssignment
        {
            std::atomic_flag writer = ATOMIC_FLAG_INIT;
            std::atomic<std::uint64_t> version{ 0 };
            std::atomic<std::uint64_t> sequence{ 0 };
            std::atomic<std::uint32_t> handleId{ 0 };
            std::atomic<std::uint32_t> stackId{ 0 };
            std::atomic<std::uint32_t> formId{ 0 };
            std::atomic<std::uint8_t> hand{ 0 };
            std::atomic<bool> active{ false };
        };

        AtomicSelectionMailbox s_selectionMailbox;
        AtomicAssignment s_assignment;
        std::atomic<std::uint64_t> s_selectionSequence{ 0 };
        std::atomic<std::uint64_t> s_assignmentSequence{ 0 };

        [[nodiscard]] pipboy_equip_policy::EquipMode configuredEquipMode() noexcept
        {
            return static_cast<pipboy_equip_policy::EquipMode>(
                s_equipMode.load(std::memory_order_acquire));
        }

        [[nodiscard]] bool tryBeginWrite(std::atomic_flag& writer)
        {
            return !writer.test_and_set(std::memory_order_acquire);
        }

        void endWrite(std::atomic_flag& writer)
        {
            writer.clear(std::memory_order_release);
        }

        void publishSelection(const SelectionEvent& event)
        {
            if (!tryBeginWrite(s_selectionMailbox.writer)) {
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Pip-Boy equip selection mailbox busy; dropping overlapping selection event");
                return;
            }

            s_selectionMailbox.version.fetch_add(1, std::memory_order_acq_rel);
            s_selectionMailbox.handleId.store(event.handleId, std::memory_order_relaxed);
            s_selectionMailbox.stackId.store(event.stackId, std::memory_order_relaxed);
            s_selectionMailbox.formId.store(event.formId, std::memory_order_relaxed);
            s_selectionMailbox.hand.store(static_cast<std::uint8_t>(event.requestedHand), std::memory_order_relaxed);
            s_selectionMailbox.source.store(static_cast<std::uint8_t>(event.triggerSource), std::memory_order_relaxed);
            s_selectionMailbox.equipped.store(event.equipped, std::memory_order_relaxed);
            s_selectionMailbox.sequence.store(event.sequence, std::memory_order_relaxed);
            s_selectionMailbox.version.fetch_add(1, std::memory_order_release);
            endWrite(s_selectionMailbox.writer);
        }

        [[nodiscard]] bool readAssignment(AssignmentSnapshot& outAssignment)
        {
            for (int attempt = 0; attempt < 4; ++attempt) {
                const std::uint64_t before = s_assignment.version.load(std::memory_order_acquire);
                if ((before & 1u) != 0) {
                    continue;
                }
                AssignmentSnapshot candidate{
                    .sequence = s_assignment.sequence.load(std::memory_order_relaxed),
                    .handleId = s_assignment.handleId.load(std::memory_order_relaxed),
                    .stackId = s_assignment.stackId.load(std::memory_order_relaxed),
                    .formId = s_assignment.formId.load(std::memory_order_relaxed),
                    .hand = static_cast<pipboy_equip_policy::Hand>(s_assignment.hand.load(std::memory_order_relaxed)),
                    .active = s_assignment.active.load(std::memory_order_relaxed),
                };
                const std::uint64_t after = s_assignment.version.load(std::memory_order_acquire);
                if (before == after && (after & 1u) == 0) {
                    outAssignment = candidate;
                    return true;
                }
            }
            outAssignment = {};
            return false;
        }

        void writeAssignment(const AssignmentSnapshot& assignment)
        {
            if (!tryBeginWrite(s_assignment.writer)) {
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Pip-Boy equip assignment mailbox busy; retaining prior assignment");
                return;
            }
            s_assignment.version.fetch_add(1, std::memory_order_acq_rel);
            s_assignment.handleId.store(assignment.handleId, std::memory_order_relaxed);
            s_assignment.stackId.store(assignment.stackId, std::memory_order_relaxed);
            s_assignment.formId.store(assignment.formId, std::memory_order_relaxed);
            s_assignment.hand.store(static_cast<std::uint8_t>(assignment.hand), std::memory_order_relaxed);
            s_assignment.active.store(assignment.active, std::memory_order_relaxed);
            s_assignment.sequence.store(assignment.sequence, std::memory_order_relaxed);
            s_assignment.version.fetch_add(1, std::memory_order_release);
            endWrite(s_assignment.writer);
        }

        [[nodiscard]] bool valueToUInt32(const RE::Scaleform::GFx::Value& value, std::uint32_t& outValue)
        {
            if (value.IsUInt()) {
                outValue = value.GetUInt();
                return true;
            }
            if (value.IsInt() && value.GetInt() >= 0) {
                outValue = static_cast<std::uint32_t>(value.GetInt());
                return true;
            }
            if (value.IsNumber()) {
                const double number = value.GetNumber();
                if (std::isfinite(number) && number >= 0.0 && number <= static_cast<double>((std::numeric_limits<std::uint32_t>::max)()) && std::floor(number) == number) {
                    outValue = static_cast<std::uint32_t>(number);
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool stackValueContainsId(const RE::Scaleform::GFx::Value& value, const std::uint32_t targetStackId)
        {
            std::uint32_t direct = 0;
            if (valueToUInt32(value, direct)) {
                return direct == targetStackId;
            }
            if (!value.IsObject()) {
                return false;
            }

            bool matched = false;
            value.VisitMembers([&](const char*, const RE::Scaleform::GFx::Value& child) {
                std::uint32_t stackId = 0;
                if (!matched && valueToUInt32(child, stackId) && stackId == targetStackId) {
                    matched = true;
                }
            });
            return matched;
        }

        struct TextMutation
        {
            RE::Scaleform::GFx::Value row{};
            RE::Scaleform::GFx::Value originalText{};
            bool applied{ false };
        };

        struct UpdateDataHookScope
        {
            UpdateDataHookScope()
            {
                s_insideUpdateDataHook = true;
            }

            ~UpdateDataHookScope()
            {
                s_insideUpdateDataHook = false;
            }

            UpdateDataHookScope(const UpdateDataHookScope&) = delete;
            UpdateDataHookScope& operator=(const UpdateDataHookScope&) = delete;
        };

        [[nodiscard]] bool tryMutateMatchingRow(
            const RE::Scaleform::GFx::Value& candidate,
            const AssignmentSnapshot& assignment,
            TextMutation& mutation)
        {
            if (!candidate.IsObject()) {
                return false;
            }

            RE::Scaleform::GFx::Value handleValue{};
            RE::Scaleform::GFx::Value stackValue{};
            RE::Scaleform::GFx::Value textValue{};
            std::uint32_t handleId = 0;
            if (!candidate.GetMember("HandleID", &handleValue) ||
                !valueToUInt32(handleValue, handleId) ||
                handleId != assignment.handleId ||
                !candidate.GetMember("StackID", &stackValue) ||
                !stackValueContainsId(stackValue, assignment.stackId) ||
                !candidate.GetMember("text", &textValue) ||
                !textValue.IsString()) {
                return false;
            }

            const char* original = textValue.GetString();
            if (!original) {
                return false;
            }
            std::string tagged{ original };
            tagged.append(pipboy_equip_policy::handTag(assignment.hand));
            RE::Scaleform::GFx::Value row = candidate;
            RE::Scaleform::GFx::Value taggedValue{ tagged.c_str() };
            if (!row.SetMember("text", taggedValue)) {
                return false;
            }

            mutation.row = std::move(row);
            mutation.originalText = std::move(textValue);
            mutation.applied = true;
            return true;
        }

        void findAndMutateMatchingRow(
            const RE::Scaleform::GFx::Value& value,
            const AssignmentSnapshot& assignment,
            const std::size_t depth,
            std::size_t& visitedNodes,
            TextMutation& mutation)
        {
            if (mutation.applied || !value.IsObject() || depth > kMaximumGfxTraversalDepth || visitedNodes >= kMaximumGfxTraversalNodes) {
                return;
            }
            ++visitedNodes;
            if (tryMutateMatchingRow(value, assignment, mutation)) {
                return;
            }
            value.VisitMembers([&](const char*, const RE::Scaleform::GFx::Value& child) {
                if (!mutation.applied) {
                    findAndMutateMatchingRow(child, assignment, depth + 1, visitedNodes, mutation);
                }
            });
        }

        void hookedUpdateData(RE::PipboyInventoryMenu* menu)
        {
            if (!s_originalUpdateData || !menu) {
                return;
            }
            if (s_insideUpdateDataHook) {
                s_originalUpdateData(menu);
                return;
            }
            if (!pipboy_equip_policy::managesHandAssignment(configuredEquipMode())) {
                s_originalUpdateData(menu);
                return;
            }

            UpdateDataHookScope hookScope;
            AssignmentSnapshot assignment{};
            TextMutation mutation{};
            if (readAssignment(assignment) && assignment.active && menu->dataObj.IsObject()) {
                std::size_t visitedNodes = 0;
                findAndMutateMatchingRow(menu->dataObj, assignment, 0, visitedNodes, mutation);
            }

            s_originalUpdateData(menu);
            if (mutation.applied) {
                (void)mutation.row.SetMember("text", mutation.originalText);
            }
        }

        void hookedUseItem(
            const std::uint32_t handleId,
            const std::uint32_t stackId,
            bool& actionSucceeded,
            bool& secondaryResult)
        {
            const auto trigger = input_remap_runtime::consumePipboyEquipTriggerResolution();
            const auto equipMode = configuredEquipMode();
            s_originalUseItem(handleId, stackId, actionSucceeded, secondaryResult);
            if (!pipboy_equip_policy::managesHandAssignment(equipMode) || !actionSucceeded) {
                return;
            }

            StackSnapshot stack{};
            if (!inspectStack(handleId, stackId, stack) || !stack.resolved || !stack.weapon) {
                return;
            }

            const pipboy_equip_policy::TriggerResolution triggerResolution{
                .hand = trigger.hand,
                .source = trigger.source,
            };
            const auto requestedHand = pipboy_equip_policy::resolveRequestedHand(equipMode, triggerResolution);
            SelectionEvent event{
                .sequence = s_selectionSequence.fetch_add(1, std::memory_order_acq_rel) + 1,
                .handleId = handleId,
                .stackId = stackId,
                .formId = stack.formId,
                .requestedHand = requestedHand.hand,
                .triggerSource = requestedHand.source,
                .equipped = stack.equipped,
            };
            publishSelection(event);

            if (stack.equipped) {
                const auto provisionalHand = requestedHand.hand == pipboy_equip_policy::Hand::Left &&
                                                     s_leftHandEquipAvailable.load(std::memory_order_acquire) ?
                    pipboy_equip_policy::Hand::Left :
                    pipboy_equip_policy::Hand::Right;
                publishWeaponAssignment(handleId, stackId, stack.formId, provisionalHand);
            } else {
                clearWeaponAssignment(handleId, stackId);
            }

            ROCK_LOG_INFO(Weapon,
                "Pip-Boy weapon selection handle={} stack={} form={:08X} action={} requestedHand={} triggerSource={}",
                handleId,
                stackId,
                stack.formId,
                stack.equipped ? "equip" : "unequip",
                requestedHand.hand == pipboy_equip_policy::Hand::Left ? "left" : "right",
                static_cast<unsigned>(requestedHand.source));
        }
    }

    bool installHooks()
    {
        if (s_hooksInstalled.load(std::memory_order_acquire)) {
            return true;
        }

        REL::Relocation<std::uintptr_t> useItemCallSite{ REL::Offset(offsets::kHookSite_PipboyInventoryUseItem) };
        const std::uintptr_t callAddress = useItemCallSite.address();
        const auto* callBytes = reinterpret_cast<const std::uint8_t*>(callAddress);
        if (!callBytes || callBytes[0] != 0xE8) {
            logger::critical("ROCK: Pip-Boy UseItem hook validation failed at 0x{:X}: expected CALL rel32.", callAddress);
            return false;
        }
        const auto relativeTarget = *reinterpret_cast<const std::int32_t*>(callBytes + 1);
        const std::uintptr_t decodedTarget = callAddress + 5u + relativeTarget;
        const std::uintptr_t expectedUseItem = REL::Offset(offsets::kFunc_PipboyInventoryUseItem).address();
        if (decodedTarget != expectedUseItem) {
            logger::critical("ROCK: Pip-Boy UseItem hook validation failed at 0x{:X}: target 0x{:X}, expected 0x{:X}.", callAddress, decodedTarget, expectedUseItem);
            return false;
        }

        REL::Relocation<std::uintptr_t> inventoryVtable{ RE::VTABLE::PipboyInventoryMenu[0] };
        const auto* slots = reinterpret_cast<const std::uintptr_t*>(inventoryVtable.address());
        const std::uintptr_t currentUpdateData = slots ? slots[kPipboyInventoryUpdateDataVtableIndex] : 0;
        const std::uintptr_t expectedUpdateData = REL::Offset(offsets::kFunc_PipboyInventoryUpdateData).address();
        if (currentUpdateData != expectedUpdateData) {
            logger::critical("ROCK: Pip-Boy inventory UpdateData hook validation failed: slot target 0x{:X}, expected 0x{:X}.", currentUpdateData, expectedUpdateData);
            return false;
        }

        auto& trampoline = F4SE::GetTrampoline();
        const auto originalUseItem = trampoline.write_call<5>(callAddress, &hookedUseItem);
        s_originalUseItem = reinterpret_cast<UseItem_t>(originalUseItem);
        const auto originalUpdateData = inventoryVtable.write_vfunc(kPipboyInventoryUpdateDataVtableIndex, &hookedUpdateData);
        s_originalUpdateData = reinterpret_cast<UpdateData_t>(originalUpdateData);
        if (!s_originalUseItem || !s_originalUpdateData) {
            logger::critical("ROCK: Pip-Boy equip hook installation returned a null original target.");
            return false;
        }

        s_hooksInstalled.store(true, std::memory_order_release);
        logger::info("ROCK: Pip-Boy trigger-hand equip and inventory side-tag hooks installed.");
        return true;
    }

    bool consumeSelectionEvent(std::uint64_t& lastSequence, SelectionEvent& outEvent)
    {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const std::uint64_t before = s_selectionMailbox.version.load(std::memory_order_acquire);
            if ((before & 1u) != 0) {
                continue;
            }
            SelectionEvent candidate{
                .sequence = s_selectionMailbox.sequence.load(std::memory_order_relaxed),
                .handleId = s_selectionMailbox.handleId.load(std::memory_order_relaxed),
                .stackId = s_selectionMailbox.stackId.load(std::memory_order_relaxed),
                .formId = s_selectionMailbox.formId.load(std::memory_order_relaxed),
                .requestedHand = static_cast<pipboy_equip_policy::Hand>(s_selectionMailbox.hand.load(std::memory_order_relaxed)),
                .triggerSource = static_cast<pipboy_equip_policy::TriggerSource>(s_selectionMailbox.source.load(std::memory_order_relaxed)),
                .equipped = s_selectionMailbox.equipped.load(std::memory_order_relaxed),
            };
            const std::uint64_t after = s_selectionMailbox.version.load(std::memory_order_acquire);
            if (before == after && (after & 1u) == 0) {
                if (candidate.sequence == 0 || candidate.sequence == lastSequence) {
                    return false;
                }
                lastSequence = candidate.sequence;
                outEvent = candidate;
                return true;
            }
        }
        return false;
    }

    bool getAssignment(AssignmentSnapshot& outAssignment)
    {
        return readAssignment(outAssignment);
    }

    bool inspectStack(const std::uint32_t handleId, const std::uint32_t stackId, StackSnapshot& outSnapshot)
    {
        outSnapshot = {};
        if (stackId > kMaximumInventoryStackIndex) {
            return false;
        }
        const auto* inventory = RE::BGSInventoryInterface::GetSingleton();
        const auto* item = inventory ? inventory->RequestInventoryItem(handleId) : nullptr;
        if (!item || !item->object) {
            return false;
        }

        auto* stack = item->stackData.get();
        for (std::uint32_t index = 0; stack && index < stackId; ++index) {
            stack = stack->nextStack.get();
        }
        if (!stack) {
            return false;
        }

        outSnapshot = StackSnapshot{
            .formId = item->object->formID,
            .resolved = true,
            .weapon = item->object->Is(RE::ENUM_FORM_ID::kWEAP),
            .equipped = stack->IsEquipped(),
        };
        return true;
    }

    void setLeftHandEquipAvailable(const bool available)
    {
        s_leftHandEquipAvailable.store(available, std::memory_order_release);
    }

    void setEquipMode(const pipboy_equip_policy::EquipMode mode)
    {
        s_equipMode.store(static_cast<std::uint8_t>(mode), std::memory_order_release);
    }

    void publishWeaponAssignment(
        const std::uint32_t handleId,
        const std::uint32_t stackId,
        const std::uint32_t formId,
        const pipboy_equip_policy::Hand hand)
    {
        writeAssignment(AssignmentSnapshot{
            .sequence = s_assignmentSequence.fetch_add(1, std::memory_order_acq_rel) + 1,
            .handleId = handleId,
            .stackId = stackId,
            .formId = formId,
            .hand = hand,
            .active = true,
        });
    }

    void clearWeaponAssignment(const std::uint32_t handleId, const std::uint32_t stackId)
    {
        AssignmentSnapshot current{};
        if (readAssignment(current) && current.active &&
            (handleId != 0 || stackId != 0) &&
            (current.handleId != handleId || current.stackId != stackId)) {
            return;
        }
        writeAssignment(AssignmentSnapshot{
            .sequence = s_assignmentSequence.fetch_add(1, std::memory_order_acq_rel) + 1,
        });
    }

    void resetRuntimeState()
    {
        setEquipMode(pipboy_equip_policy::EquipMode::NativeRight);
        setLeftHandEquipAvailable(false);
        clearWeaponAssignment();
        publishSelection(SelectionEvent{
            .sequence = s_selectionSequence.fetch_add(1, std::memory_order_acq_rel) + 1,
        });
    }
}
