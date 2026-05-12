#include "physics-interaction/weapon/WeaponInstanceWitnessRuntime.h"

#include "physics-interaction/PhysicsLog.h"

#include "RE/Bethesda/BGSInventoryItem.h"
#include "RE/Bethesda/BGSMod.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/BSLock.h"
#include "RE/Bethesda/BSTEvent.h"
#include "RE/Bethesda/Events.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"

#include "REL/Relocation.h"

#include <Windows.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>

namespace rock::weapon_instance_witness_runtime
{
    namespace
    {
        constexpr std::uint32_t kMaxPendingWitnessScans = 180;
        constexpr std::chrono::milliseconds kPendingWitnessScanRetryInterval{ 16 };
        constexpr std::chrono::milliseconds kActiveWitnessRevalidateInterval{ 33 };
        constexpr std::chrono::seconds kPendingWitnessCaptureWindow{ 6 };
        constexpr std::chrono::seconds kActiveWitnessLifetime{ 8 };
        using WitnessClock = std::chrono::steady_clock;

        struct PendingSignal
        {
            WeaponInstanceWitnessSource source{ WeaponInstanceWitnessSource::None };
            std::uint64_t sequence{ 0 };
            std::uint32_t weaponFormID{ 0 };
            std::uint32_t modFormID{ 0 };
            std::uint16_t uniqueID{ 0 };
            std::uint8_t attachIndex{ 0 };
            std::uint8_t rank{ 0 };
            std::uint32_t scansRemaining{ 0 };
            WitnessClock::time_point expiresAt{};
            WitnessClock::time_point nextScanAfter{};
            const char* reason{ "" };
        };

        std::mutex s_stateLock;
        std::atomic<std::uint64_t> s_nextSignalSequence{ 1 };
        std::atomic<bool> s_runtimeInstalled{ false };
        std::atomic<bool> s_attachModHookInstalled{ false };
        std::atomic<bool> s_equipEventSinkInstalled{ false };

        PendingSignal s_pendingSignal{};
        AuthoritativeEquippedWeaponWitness s_activeWitness{};
        AuthoritativeEquippedWeaponWitness s_gameLoadBaselineWitness{};
        WitnessClock::time_point s_activeWitnessExpiresAt{};
        WitnessClock::time_point s_activeWitnessNextValidationAfter{};

        using AttachModToReference_t = bool (*)(
            RE::TESObjectREFR&,
            RE::BGSMod::Attachment::Mod&,
            std::uint8_t,
            std::uint8_t);

        AttachModToReference_t s_originalAttachModToReference = nullptr;
        void* s_attachModOriginalTrampoline = nullptr;

        void writeAbsoluteJump(std::uint8_t* target, std::uintptr_t destination)
        {
            target[0] = 0xFF;
            target[1] = 0x25;
            target[2] = 0x00;
            target[3] = 0x00;
            target[4] = 0x00;
            target[5] = 0x00;
            *reinterpret_cast<std::uintptr_t*>(target + 6) = destination;
        }

        bool installEntryHook(
            const char* label,
            std::uintptr_t targetAddress,
            const std::uint8_t* expectedPrefix,
            std::size_t stolenBytes,
            void* hook,
            void*& original)
        {
            if (!targetAddress || !expectedPrefix || !hook) {
                ROCK_LOG_ERROR(Init, "{} hook install failed: invalid target, prefix, or hook", label);
                return false;
            }

            constexpr std::size_t kAbsoluteJumpBytes = 14;
            if (stolenBytes < kAbsoluteJumpBytes) {
                ROCK_LOG_ERROR(Init, "{} hook install failed: stolen byte count {} is too small", label, stolenBytes);
                return false;
            }

            auto* target = reinterpret_cast<std::uint8_t*>(targetAddress);
            if (std::memcmp(target, expectedPrefix, stolenBytes) != 0) {
                ROCK_LOG_ERROR(Init, "{} hook validation failed at 0x{:X}; native bytes changed, hook not installed", label, targetAddress);
                return false;
            }

            const std::size_t trampolineBytes = stolenBytes + kAbsoluteJumpBytes;
            auto* trampoline = reinterpret_cast<std::uint8_t*>(
                VirtualAlloc(nullptr, trampolineBytes, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE));
            if (!trampoline) {
                ROCK_LOG_ERROR(Init, "{} hook install failed: trampoline allocation failed", label);
                return false;
            }

            std::memcpy(trampoline, target, stolenBytes);
            writeAbsoluteJump(trampoline + stolenBytes, targetAddress + stolenBytes);

            DWORD oldTrampolineProtect = 0;
            if (!VirtualProtect(trampoline, trampolineBytes, PAGE_EXECUTE_READ, &oldTrampolineProtect)) {
                ROCK_LOG_ERROR(Init, "{} hook install failed: trampoline protection failed", label);
                VirtualFree(trampoline, 0, MEM_RELEASE);
                return false;
            }

            DWORD oldProtect = 0;
            if (!VirtualProtect(target, stolenBytes, PAGE_EXECUTE_READWRITE, &oldProtect)) {
                ROCK_LOG_ERROR(Init, "{} hook install failed at 0x{:X}: target protection failed", label, targetAddress);
                VirtualFree(trampoline, 0, MEM_RELEASE);
                return false;
            }

            writeAbsoluteJump(target, reinterpret_cast<std::uintptr_t>(hook));
            for (std::size_t i = kAbsoluteJumpBytes; i < stolenBytes; ++i) {
                target[i] = 0x90;
            }

            FlushInstructionCache(GetCurrentProcess(), target, stolenBytes);
            VirtualProtect(target, stolenBytes, oldProtect, &oldProtect);

            original = trampoline;
            ROCK_LOG_INFO(Init, "Installed {} hook at 0x{:X}, original trampoline=0x{:X}", label, targetAddress, reinterpret_cast<std::uintptr_t>(trampoline));
            return true;
        }

        bool isWeaponForm(const RE::TESForm* form)
        {
            return form && form->GetFormType() == RE::ENUM_FORM_ID::kWEAP;
        }

        bool isPlayerReference(const RE::TESObjectREFR* ref)
        {
            if (!ref) {
                return false;
            }

            const auto* player = RE::PlayerCharacter::GetSingleton();
            return ref == player || ref->GetFormID() == 0x14;
        }

        bool objectInstanceExtraHasActiveMod(
            const RE::BGSObjectInstanceExtra* extra,
            std::uint32_t modFormID)
        {
            if (modFormID == 0) {
                return true;
            }
            if (!extra || !extra->values) {
                return false;
            }

            const auto indexData = extra->GetIndexData();
            for (const auto& modIndex : indexData) {
                if (!modIndex.disabled && modIndex.objectID == modFormID) {
                    return true;
                }
            }

            return false;
        }

        void mixObjectInstanceExtraWitness(
            std::uint64_t& key,
            const RE::BGSObjectInstanceExtra* extra,
            std::uint64_t& outSignature,
            std::uint32_t& outCount,
            std::uint32_t& outActiveCount,
            std::uint32_t& outDisabledCount)
        {
            outSignature = 0;
            outCount = 0;
            outActiveCount = 0;
            outDisabledCount = 0;

            if (!extra || !extra->values) {
                weapon_visual_composition_policy::mixValue(key, 0u);
                return;
            }

            const auto indexData = extra->GetIndexData();
            std::uint64_t witnessKey = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
            weapon_visual_composition_policy::mixString(witnessKey, "ROCKInventoryObjectInstanceExtraIndexWitnessV1");
            weapon_visual_composition_policy::mixValue(witnessKey, indexData.size());

            weapon_visual_composition_policy::mixValue(key, 1u);
            weapon_visual_composition_policy::mixValue(key, indexData.size());
            outCount = static_cast<std::uint32_t>(
                (std::min)(indexData.size(), static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)())));

            for (const auto& modIndex : indexData) {
                weapon_visual_composition_policy::mixValue(witnessKey, modIndex.objectID);
                weapon_visual_composition_policy::mixValue(witnessKey, modIndex.index);
                weapon_visual_composition_policy::mixValue(witnessKey, modIndex.rank);
                weapon_visual_composition_policy::mixValue(witnessKey, modIndex.disabled);

                weapon_visual_composition_policy::mixValue(key, modIndex.objectID);
                weapon_visual_composition_policy::mixValue(key, modIndex.index);
                weapon_visual_composition_policy::mixValue(key, modIndex.rank);
                weapon_visual_composition_policy::mixValue(key, modIndex.disabled);

                if (modIndex.disabled) {
                    ++outDisabledCount;
                } else {
                    ++outActiveCount;
                }
            }

            outSignature = witnessKey;
        }

        weapon_generation_identity_policy::EquippedWeaponGenerationIdentity makeIdentityFromWitness(
            const AuthoritativeEquippedWeaponWitness& witness)
        {
            weapon_generation_identity_policy::EquippedWeaponGenerationIdentity identity{};
            if (witness.weaponFormID == 0 || witness.weaponFormAddress == 0) {
                return identity;
            }

            identity.hasEquippedWeapon = true;
            identity.formID = witness.weaponFormID;
            identity.formAddress = witness.weaponFormAddress;
            identity.instanceDataAddress = witness.instanceDataAddress;
            identity.instanceKeywordDataAddress = witness.instanceKeywordDataAddress;
            identity.instanceContentKey = witness.instanceContentKey;
            identity.objectInstanceExtraAddress = witness.objectInstanceExtraAddress;
            identity.objectIndexDataSignature = witness.objectIndexDataSignature;
            identity.objectIndexDataCount = witness.objectIndexDataCount;
            identity.activeModCount = witness.activeModCount;
            identity.disabledModCount = witness.disabledModCount;
            identity.equippedDataAddress = witness.equippedDataAddress;
            identity.equippedObjectAddress = witness.equippedObjectAddress;
            identity.displayName = witness.displayName;
            return identity;
        }

        AuthoritativeEquippedWeaponWitness makeWitnessFromInventoryStack(
            WeaponInstanceWitnessSource source,
            std::uint64_t sequence,
            const RE::TESBoundObject& object,
            const RE::BGSInventoryItem::Stack& stack,
            std::uint32_t stackIndex)
        {
            AuthoritativeEquippedWeaponWitness witness{};
            const auto* weapon = const_cast<RE::TESBoundObject&>(object).As<RE::TESObjectWEAP>();
            const auto* extraList = stack.extra.get();
            const auto* instanceExtra = extraList ? extraList->GetByType<RE::ExtraInstanceData>() : nullptr;
            const auto* uniqueExtra = extraList ? extraList->GetByType<RE::ExtraUniqueID>() : nullptr;
            const auto* objectInstanceExtra = extraList ? extraList->GetByType<RE::BGSObjectInstanceExtra>() : nullptr;
            const auto* instanceData = instanceExtra ? instanceExtra->data.get() : nullptr;
            const auto displayName = weapon ? RE::TESFullName::GetFullName(*weapon) : std::string_view{};

            std::uint64_t contentKey = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
            weapon_visual_composition_policy::mixString(contentKey, "ROCKInventoryEquippedInstanceContentV1");
            weapon_visual_composition_policy::mixValue(contentKey, object.GetFormID());
            weapon_visual_composition_policy::mixValue(contentKey, reinterpret_cast<std::uintptr_t>(instanceData ? instanceData->GetKeywordData() : nullptr));

            std::uint64_t objectIndexDataSignature = 0;
            std::uint32_t objectIndexDataCount = 0;
            std::uint32_t activeModCount = 0;
            std::uint32_t disabledModCount = 0;
            mixObjectInstanceExtraWitness(
                contentKey,
                objectInstanceExtra,
                objectIndexDataSignature,
                objectIndexDataCount,
                activeModCount,
                disabledModCount);
            weapon_visual_composition_policy::mixString(contentKey, displayName);

            weapon_generation_identity_policy::EquippedWeaponGenerationIdentity identity{};
            identity.hasEquippedWeapon = true;
            identity.formID = object.GetFormID();
            identity.formAddress = reinterpret_cast<std::uintptr_t>(&object);
            identity.instanceDataAddress = reinterpret_cast<std::uintptr_t>(instanceData);
            identity.instanceKeywordDataAddress = reinterpret_cast<std::uintptr_t>(instanceData ? instanceData->GetKeywordData() : nullptr);
            identity.instanceContentKey = contentKey;
            identity.objectInstanceExtraAddress = reinterpret_cast<std::uintptr_t>(objectInstanceExtra);
            identity.objectIndexDataSignature = objectIndexDataSignature;
            identity.objectIndexDataCount = objectIndexDataCount;
            identity.activeModCount = activeModCount;
            identity.disabledModCount = disabledModCount;
            identity.equippedDataAddress = reinterpret_cast<std::uintptr_t>(extraList);
            identity.equippedObjectAddress = reinterpret_cast<std::uintptr_t>(&object);
            identity.displayName = displayName;
            const auto instanceWitness = weapon_generation_identity_policy::makeEquippedWeaponInstanceWitness(identity);

            witness.source = source;
            witness.sequence = sequence;
            witness.weaponFormID = identity.formID;
            witness.weaponFormAddress = identity.formAddress;
            witness.instanceDataAddress = identity.instanceDataAddress;
            witness.instanceKeywordDataAddress = identity.instanceKeywordDataAddress;
            witness.objectInstanceExtraAddress = identity.objectInstanceExtraAddress;
            witness.equippedDataAddress = identity.equippedDataAddress;
            witness.equippedObjectAddress = identity.equippedObjectAddress;
            witness.instanceContentKey = identity.instanceContentKey;
            witness.objectIndexDataSignature = identity.objectIndexDataSignature;
            witness.objectIndexDataCount = identity.objectIndexDataCount;
            witness.activeModCount = identity.activeModCount;
            witness.disabledModCount = identity.disabledModCount;
            witness.stackIndex = stackIndex;
            witness.uniqueID = uniqueExtra ? uniqueExtra->uniqueID : 0;
            witness.displayName = displayName;
            witness.signature = instanceWitness.signature;
            return witness;
        }

        bool scanEquippedWeaponInventoryStack(
            const PendingSignal& signal,
            AuthoritativeEquippedWeaponWitness& outWitness)
        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            auto* inventoryList = player ? player->inventoryList : nullptr;
            if (!player || !inventoryList) {
                return false;
            }

            const RE::BSAutoReadLock lock{ inventoryList->rwLock };
            std::uint32_t stackIndex = 0;
            for (auto& item : inventoryList->data) {
                auto* object = item.object;
                if (!object || !isWeaponForm(object)) {
                    continue;
                }

                if (signal.weaponFormID != 0 && object->GetFormID() != signal.weaponFormID) {
                    continue;
                }

                for (auto* stack = item.stackData.get(); stack; stack = stack->nextStack.get()) {
                    const auto currentStackIndex = stackIndex++;
                    if (!stack->IsEquipped()) {
                        continue;
                    }

                    const auto* extraList = stack->extra.get();
                    const auto* uniqueExtra = extraList ? extraList->GetByType<RE::ExtraUniqueID>() : nullptr;
                    if (signal.uniqueID != 0 && (!uniqueExtra || uniqueExtra->uniqueID != signal.uniqueID)) {
                        continue;
                    }
                    const auto* objectInstanceExtra = extraList ? extraList->GetByType<RE::BGSObjectInstanceExtra>() : nullptr;
                    if (!objectInstanceExtraHasActiveMod(objectInstanceExtra, signal.modFormID)) {
                        continue;
                    }

                    outWitness = makeWitnessFromInventoryStack(signal.source, signal.sequence, *object, *stack, currentStackIndex);
                    outWitness.mutatingModFormID = signal.modFormID;
                    outWitness.attachIndex = signal.attachIndex;
                    outWitness.rank = signal.rank;
                    return outWitness.weaponFormID != 0 && outWitness.weaponFormAddress != 0;
                }
            }

            return false;
        }

        bool scanActiveWitnessStack(
            const AuthoritativeEquippedWeaponWitness& active,
            AuthoritativeEquippedWeaponWitness& outWitness)
        {
            if (active.source != WeaponInstanceWitnessSource::GameLoadBaseline &&
                active.source != WeaponInstanceWitnessSource::InventoryEquip &&
                active.source != WeaponInstanceWitnessSource::AttachMod) {
                return false;
            }

            return scanEquippedWeaponInventoryStack(
                PendingSignal{
                    .source = active.source,
                    .sequence = active.sequence,
                    .weaponFormID = active.weaponFormID,
                    .modFormID = active.mutatingModFormID,
                    .uniqueID = active.uniqueID,
                    .attachIndex = active.attachIndex,
                    .rank = active.rank,
                    .scansRemaining = 1,
                    .reason = "activeWitnessRevalidate",
                },
                outWitness);
        }

        void recordPendingSignal(
            WeaponInstanceWitnessSource source,
            std::uint32_t weaponFormID,
            std::uint16_t uniqueID,
            std::uint32_t modFormID,
            std::uint8_t attachIndex,
            std::uint8_t rank,
            const char* reason)
        {
            const auto sequence = s_nextSignalSequence.fetch_add(1, std::memory_order_acq_rel);
            const auto now = WitnessClock::now();
            {
                std::scoped_lock lock{ s_stateLock };
                s_pendingSignal = PendingSignal{
                    .source = source,
                    .sequence = sequence,
                    .weaponFormID = weaponFormID,
                    .modFormID = modFormID,
                    .uniqueID = uniqueID,
                    .attachIndex = attachIndex,
                    .rank = rank,
                    .scansRemaining = kMaxPendingWitnessScans,
                    .expiresAt = now + kPendingWitnessCaptureWindow,
                    .nextScanAfter = now,
                    .reason = reason,
                };
                if (source != WeaponInstanceWitnessSource::GameLoadBaseline) {
                    s_activeWitness = {};
                    s_activeWitnessExpiresAt = {};
                    s_activeWitnessNextValidationAfter = {};
                }
            }

            ROCK_LOG_INFO(Weapon,
                "Weapon instance witness signal source={} sequence={} form={:08X} uniqueID={} mod={:08X} attachIndex={} rank={} reason={}",
                sourceName(source),
                sequence,
                weaponFormID,
                uniqueID,
                modFormID,
                attachIndex,
                rank,
                reason ? reason : "");
        }

        void refreshPendingSignal()
        {
            const auto now = WitnessClock::now();
            PendingSignal signal{};
            {
                std::scoped_lock lock{ s_stateLock };
                signal = s_pendingSignal;
            }

            if (signal.source == WeaponInstanceWitnessSource::None || signal.scansRemaining == 0) {
                return;
            }
            if (signal.expiresAt != WitnessClock::time_point{} && now >= signal.expiresAt) {
                {
                    std::scoped_lock lock{ s_stateLock };
                    if (s_pendingSignal.sequence == signal.sequence) {
                        s_pendingSignal = {};
                    }
                }
                ROCK_LOG_WARN(Weapon,
                    "Expired weapon instance witness signal source={} sequence={} form={:08X} uniqueID={} mod={:08X} reason={} elapsed=deadline",
                    sourceName(signal.source),
                    signal.sequence,
                    signal.weaponFormID,
                    signal.uniqueID,
                    signal.modFormID,
                    signal.reason ? signal.reason : "");
                return;
            }
            if (signal.nextScanAfter != WitnessClock::time_point{} && now < signal.nextScanAfter) {
                return;
            }

            AuthoritativeEquippedWeaponWitness witness{};
            if (scanEquippedWeaponInventoryStack(signal, witness)) {
                {
                    std::scoped_lock lock{ s_stateLock };
                    if (s_pendingSignal.sequence != signal.sequence) {
                        return;
                    }

                    if (signal.source == WeaponInstanceWitnessSource::GameLoadBaseline) {
                        s_gameLoadBaselineWitness = witness;
                    }
                    s_activeWitness = witness;
                    s_activeWitnessExpiresAt = now + kActiveWitnessLifetime;
                    s_activeWitnessNextValidationAfter = now + kActiveWitnessRevalidateInterval;
                    s_pendingSignal = {};
                }

                ROCK_LOG_INFO(Weapon,
                    "Captured equipped weapon instance witness source={} sequence={} signature={:016X} form={:08X}/{:x} instance={:x} keyword={:x} extra={:x} equippedData={:x} mods={}/{} stack={} uniqueID={} mod={:08X} attachIndex={} rank={} name={}",
                    sourceName(witness.source),
                    witness.sequence,
                    witness.signature,
                    witness.weaponFormID,
                    witness.weaponFormAddress,
                    witness.instanceDataAddress,
                    witness.instanceKeywordDataAddress,
                    witness.objectInstanceExtraAddress,
                    witness.equippedDataAddress,
                    witness.activeModCount,
                    witness.objectIndexDataCount,
                    witness.stackIndex,
                    witness.uniqueID,
                    witness.mutatingModFormID,
                    witness.attachIndex,
                    witness.rank,
                    witness.displayName);
                return;
            }

            bool expired = false;
            {
                std::scoped_lock lock{ s_stateLock };
                if (s_pendingSignal.sequence != signal.sequence || s_pendingSignal.source == WeaponInstanceWitnessSource::None) {
                    return;
                }

                if (s_pendingSignal.scansRemaining > 0) {
                    --s_pendingSignal.scansRemaining;
                }
                expired = s_pendingSignal.scansRemaining == 0;
                if (expired) {
                    s_pendingSignal = {};
                } else {
                    s_pendingSignal.nextScanAfter = now + kPendingWitnessScanRetryInterval;
                }
            }

            if (expired) {
                ROCK_LOG_WARN(Weapon,
                    "Expired weapon instance witness signal source={} sequence={} form={:08X} uniqueID={} mod={:08X} reason={} elapsed=scans",
                    sourceName(signal.source),
                    signal.sequence,
                    signal.weaponFormID,
                    signal.uniqueID,
                    signal.modFormID,
                    signal.reason ? signal.reason : "");
            }
        }

        bool hookedAttachModToReference(
            RE::TESObjectREFR& ref,
            RE::BGSMod::Attachment::Mod& mod,
            std::uint8_t attachIndex,
            std::uint8_t rank)
        {
            auto* original = s_originalAttachModToReference;
            if (!original) {
                return false;
            }

            const bool result = original(ref, mod, attachIndex, rank);
            if (result && isPlayerReference(&ref)) {
                recordPendingSignal(
                    WeaponInstanceWitnessSource::AttachMod,
                    0,
                    0,
                    mod.GetFormID(),
                    attachIndex,
                    rank,
                    "AttachModToReferencePostCall");
            }
            return result;
        }

        bool installAttachModHook()
        {
            if (s_attachModHookInstalled.load(std::memory_order_acquire)) {
                return true;
            }

            REL::Relocation<std::uintptr_t> attachModTarget{ REL::RelocationID(3303, 2189033) };
            constexpr std::uint8_t kExpectedAttachModPrefix[] = {
                0x48, 0x89, 0x5C, 0x24, 0x20,
                0x44, 0x89, 0x44, 0x24, 0x18,
                0x55,
                0x56,
                0x57,
                0x41, 0x54,
            };

            void* original = nullptr;
            if (!installEntryHook(
                    "WeaponAttachModToReferenceWitness",
                    attachModTarget.address(),
                    kExpectedAttachModPrefix,
                    sizeof(kExpectedAttachModPrefix),
                    reinterpret_cast<void*>(&hookedAttachModToReference),
                    original)) {
                return false;
            }

            s_attachModOriginalTrampoline = original;
            s_originalAttachModToReference = reinterpret_cast<AttachModToReference_t>(original);
            s_attachModHookInstalled.store(true, std::memory_order_release);
            return true;
        }

        class EquipEventSink final :
            public RE::BSTEventSink<RE::TESEquipEvent>
        {
        public:
            RE::BSEventNotifyControl ProcessEvent(
                const RE::TESEquipEvent& event,
                RE::BSTEventSource<RE::TESEquipEvent>*) override
            {
                if (!event.equipped || !isPlayerReference(event.actor.get())) {
                    return RE::BSEventNotifyControl::kContinue;
                }

                const auto* form = RE::TESForm::GetFormByID(event.baseObject);
                if (!isWeaponForm(form)) {
                    return RE::BSEventNotifyControl::kContinue;
                }

                recordPendingSignal(
                    WeaponInstanceWitnessSource::InventoryEquip,
                    event.baseObject,
                    event.uniqueID,
                    0,
                    0,
                    0,
                    "TESEquipEvent");
                return RE::BSEventNotifyControl::kContinue;
            }
        };

        EquipEventSink s_equipEventSink;

        bool installEquipEventSink()
        {
            if (s_equipEventSinkInstalled.load(std::memory_order_acquire)) {
                return true;
            }

            auto* source = RE::TESEquipEvent::GetEventSource();
            if (!source) {
                ROCK_LOG_ERROR(Init, "Weapon instance witness runtime could not resolve TESEquipEvent source");
                return false;
            }

            source->RegisterSink(&s_equipEventSink);
            s_equipEventSinkInstalled.store(true, std::memory_order_release);
            ROCK_LOG_INFO(Init, "Registered weapon instance witness TESEquipEvent sink");
            return true;
        }
    }

    const char* sourceName(WeaponInstanceWitnessSource source) noexcept
    {
        switch (source) {
        case WeaponInstanceWitnessSource::GameLoadBaseline:
            return "gameLoadBaseline";
        case WeaponInstanceWitnessSource::InventoryEquip:
            return "inventoryEquip";
        case WeaponInstanceWitnessSource::AttachMod:
            return "attachMod";
        case WeaponInstanceWitnessSource::None:
        default:
            return "none";
        }
    }

    bool installWeaponInstanceWitnessRuntime()
    {
        if (s_runtimeInstalled.load(std::memory_order_acquire)) {
            return true;
        }

        const bool equipSinkInstalled = installEquipEventSink();
        const bool attachHookInstalled = installAttachModHook();
        const bool installed = equipSinkInstalled && attachHookInstalled;
        if (installed) {
            s_runtimeInstalled.store(true, std::memory_order_release);
            ROCK_LOG_INFO(Init, "Weapon instance witness runtime installed");
        }
        return installed;
    }

    void noteGameSessionReset(const char* reason)
    {
        {
            std::scoped_lock lock{ s_stateLock };
            s_pendingSignal = {};
            s_activeWitness = {};
            s_gameLoadBaselineWitness = {};
        }

        ROCK_LOG_INFO(Weapon, "Weapon instance witness runtime reset reason={}", reason ? reason : "");
    }

    void noteGameLoadWeaponWitnessBaseline()
    {
        recordPendingSignal(WeaponInstanceWitnessSource::GameLoadBaseline, 0, 0, 0, 0, 0, "gameLoadBaseline");
        refreshPendingSignal();
    }

    bool tryGetAuthoritativeEquippedWeaponIdentity(
        weapon_generation_identity_policy::EquippedWeaponGenerationIdentity& outIdentity,
        AuthoritativeEquippedWeaponWitness* outWitness)
    {
        refreshPendingSignal();
        const auto now = WitnessClock::now();

        AuthoritativeEquippedWeaponWitness active{};
        WitnessClock::time_point activeExpiresAt{};
        WitnessClock::time_point nextValidationAfter{};
        {
            std::scoped_lock lock{ s_stateLock };
            active = s_activeWitness;
            activeExpiresAt = s_activeWitnessExpiresAt;
            nextValidationAfter = s_activeWitnessNextValidationAfter;
        }

        if (active.source == WeaponInstanceWitnessSource::None || active.signature == 0) {
            return false;
        }
        if (activeExpiresAt != WitnessClock::time_point{} && now >= activeExpiresAt) {
            {
                std::scoped_lock lock{ s_stateLock };
                if (s_activeWitness.sequence == active.sequence) {
                    s_activeWitness = {};
                    s_activeWitnessExpiresAt = {};
                    s_activeWitnessNextValidationAfter = {};
                }
            }
            ROCK_LOG_INFO(Weapon,
                "Expired authoritative weapon instance witness source={} sequence={} signature={:016X}; bounded remap window elapsed",
                sourceName(active.source),
                active.sequence,
                active.signature);
            return false;
        }
        if (nextValidationAfter != WitnessClock::time_point{} && now < nextValidationAfter) {
            outIdentity = makeIdentityFromWitness(active);
            if (outWitness) {
                *outWitness = active;
            }
            return outIdentity.hasEquippedWeapon;
        }

        AuthoritativeEquippedWeaponWitness refreshed{};
        if (!scanActiveWitnessStack(active, refreshed)) {
            {
                std::scoped_lock lock{ s_stateLock };
                if (s_activeWitness.sequence == active.sequence) {
                    s_activeWitness = {};
                    s_activeWitnessExpiresAt = {};
                    s_activeWitnessNextValidationAfter = {};
                }
            }
            ROCK_LOG_INFO(Weapon,
                "Cleared authoritative weapon instance witness source={} sequence={} signature={:016X}; equipped inventory stack no longer matches",
                sourceName(active.source),
                active.sequence,
                active.signature);
            return false;
        }

        {
            std::scoped_lock lock{ s_stateLock };
            if (s_activeWitness.sequence == active.sequence) {
                refreshed.mutatingModFormID = active.mutatingModFormID;
                refreshed.attachIndex = active.attachIndex;
                refreshed.rank = active.rank;
                s_activeWitness = refreshed;
                s_activeWitnessNextValidationAfter = now + kActiveWitnessRevalidateInterval;
            }
        }

        outIdentity = makeIdentityFromWitness(refreshed);
        if (outWitness) {
            *outWitness = refreshed;
        }
        return outIdentity.hasEquippedWeapon;
    }

    bool tryGetAuthoritativeEquippedWeaponWitness(
        std::uint64_t expectedSignature,
        AuthoritativeEquippedWeaponWitness& outWitness)
    {
        weapon_generation_identity_policy::EquippedWeaponGenerationIdentity identity{};
        AuthoritativeEquippedWeaponWitness witness{};
        if (!tryGetAuthoritativeEquippedWeaponIdentity(identity, &witness)) {
            return false;
        }

        if (expectedSignature != 0 && witness.signature != expectedSignature) {
            return false;
        }

        outWitness = witness;
        return true;
    }

    bool nativeVisualRemapAllowedForWitness(
        std::uint64_t expectedSignature,
        const char*& outReason)
    {
        outReason = "nativeRemapBlockedNoAttachWitness";
        refreshPendingSignal();

        AuthoritativeEquippedWeaponWitness active{};
        {
            std::scoped_lock lock{ s_stateLock };
            active = s_activeWitness;
        }

        if (active.source == WeaponInstanceWitnessSource::None || active.signature == 0) {
            return false;
        }

        if (active.source != WeaponInstanceWitnessSource::AttachMod) {
            outReason = active.source == WeaponInstanceWitnessSource::InventoryEquip ?
                "nativeRemapBlockedInventoryEquip" :
                "nativeRemapBlockedNonAttachWitness";
            return false;
        }

        if (expectedSignature != 0 && active.signature != expectedSignature) {
            outReason = "nativeRemapBlockedAttachWitnessMismatch";
            return false;
        }

        AuthoritativeEquippedWeaponWitness revalidated{};
        if (!tryGetAuthoritativeEquippedWeaponWitness(expectedSignature, revalidated)) {
            outReason = "nativeRemapBlockedAttachWitnessStale";
            return false;
        }

        outReason = "nativeRemapAllowedAttachModWitness";
        return true;
    }

    void clearAuthoritativeEquippedWeaponWitness(
        std::uint64_t settledSignature,
        const char* reason)
    {
        AuthoritativeEquippedWeaponWitness cleared{};
        {
            std::scoped_lock lock{ s_stateLock };
            if (s_activeWitness.source == WeaponInstanceWitnessSource::None) {
                return;
            }
            if (settledSignature != 0 && s_activeWitness.signature != settledSignature) {
                return;
            }
            cleared = s_activeWitness;
            s_activeWitness = {};
            s_activeWitnessExpiresAt = {};
            s_activeWitnessNextValidationAfter = {};
        }

        ROCK_LOG_INFO(Weapon,
            "Cleared authoritative weapon instance witness source={} sequence={} signature={:016X} reason={}",
            sourceName(cleared.source),
            cleared.sequence,
            cleared.signature,
            reason ? reason : "");
    }
}
