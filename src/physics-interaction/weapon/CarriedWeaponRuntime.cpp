#include "physics-interaction/weapon/CarriedWeaponRuntime.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/NativeWeaponQualification.h"
#include "physics-interaction/weapon/WeaponGripTransfer.h"
#include "physics-interaction/weapon/telemetry/NativeScopeShotDiagnostics.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/BSLock.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESRace.h"
#include <Windows.h>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>

namespace rock
{
    namespace
    {
        using ReadItem = bool (*)(RE::AIProcess*, std::uint32_t, RE::EquippedItem*);
        using SetItem = void (*)(RE::AIProcess*, RE::Actor*, const RE::BGSObjectInstance*, const RE::BGSEquipSlot*);
        using Fire = void (*)(const RE::BGSObjectInstance*, RE::TESObjectREFR*, std::uint32_t, RE::TESAmmo*, void*);
        using Reload = bool (*)(RE::Actor*, const RE::BGSObjectInstance*, std::uint32_t);
        using Rate = float (*)(RE::TESObjectWEAP*, RE::TBO_InstanceData*);
        using SetCount = void (*)(RE::AIProcess*, std::uint32_t, std::uint32_t);
        using ResolveIndex = std::uint32_t* (*)(RE::Actor*, std::uint32_t*, const RE::BGSEquipSlot*);
        using ItemCount = bool (*)(RE::TESObjectREFR*, std::uint32_t*, RE::TESForm*, bool);
        std::atomic<std::uintptr_t> borrowedData{};
        std::atomic<std::uint32_t> borrowedIndex{UINT32_MAX};
        std::atomic<std::uint64_t> saveEpoch{};
        std::atomic<bool> saving{};
        std::atomic<std::uint32_t> interactionThread{};
        bool ammoHookInstalled{};

        struct Archive
        {
            std::uint32_t reference{}, weapon{}, ammo{}, loaded{};
            float cooldown{}, reloadRemaining{};
            std::uint32_t flags{};
        };
        static_assert(sizeof(Archive) == 28);
        // Frame thread publishes values. F4SE serialization callbacks copy
        // values under the same nonblocking guard; no game pointers cross it.
        struct ArchiveChannel
        {
            std::atomic_flag busy = ATOMIC_FLAG_INIT;
            Archive value{};
            bool read(Archive& output) noexcept {
                if (busy.test_and_set(std::memory_order_acquire)) return false;
                output = value;
                busy.clear(std::memory_order_release);
                return true;
            }
            bool write(const Archive& input) noexcept {
                if (busy.test_and_set(std::memory_order_acquire)) return false;
                value = input;
                busy.clear(std::memory_order_release);
                return true;
            }
        } liveArchive, loadedArchive;
        constexpr std::uint32_t archiveType = 0x424D4B41; // AKMB

        void saveArchive(const F4SE::SerializationInterface* serialization) noexcept
        {
            try {
                Archive record{};
                if (!liveArchive.read(record)) {
                    ROCK_LOG_ERROR(Weapon, "Akimbo save snapshot unavailable");
                } else if (record.reference && !serialization->WriteRecord(archiveType, 1, &record, sizeof(record))) {
                    ROCK_LOG_ERROR(Weapon, "Akimbo save record could not be written");
                }
            } catch (...) { try { ROCK_LOG_ERROR(Weapon, "Akimbo save callback failed"); } catch (...) {} }
        }

        void loadArchive(const F4SE::SerializationInterface* serialization) noexcept
        {
            try {
                (void)loadedArchive.write({});
                std::uint32_t type{}, version{}, length{};
                for (unsigned recordIndex = 0; recordIndex < 64 && serialization->GetNextRecordInfo(type, version, length); ++recordIndex) {
                    if (type != archiveType || version != 1 || length != sizeof(Archive)) continue;
                    Archive record{};
                    if (serialization->ReadRecordData(&record, sizeof(record)) != sizeof(record) || !record.reference || !record.weapon ||
                        (record.flags & ~1u) || !std::isfinite(record.cooldown) || record.cooldown < 0.0f ||
                        !std::isfinite(record.reloadRemaining) || record.reloadRemaining < 0.0f) continue;
                    const auto reference = serialization->ResolveFormID(record.reference);
                    const auto weapon = serialization->ResolveFormID(record.weapon);
                    const auto ammo = record.ammo ? serialization->ResolveFormID(record.ammo) : std::optional<std::uint32_t>{0};
                    if (!reference || !weapon || !ammo) continue;
                    record.reference = *reference; record.weapon = *weapon; record.ammo = *ammo;
                    if (!loadedArchive.write(record)) ROCK_LOG_ERROR(Weapon, "Akimbo restored snapshot publication failed");
                }
            } catch (...) { try { ROCK_LOG_ERROR(Weapon, "Akimbo save data could not be restored"); } catch (...) {} }
        }

        void revertArchive(const F4SE::SerializationInterface*) noexcept
        {
            (void)liveArchive.write({});
            (void)loadedArchive.write({});
            saving.store(false, std::memory_order_release);
        }

        RE::EquippedItem emptyItem() { return {RE::BGSObjectInstance(nullptr, nullptr)}; }

        // FO4VR raw witnesses and caller ABIs are recorded in the akimbo
        // implementation ledger. No flat relocation IDs enter this backend.
        bool nativeReady()
        {
            struct Guard { std::uintptr_t rva; std::array<std::uint8_t, 12> bytes; };
            static const bool ready = [] {
                constexpr std::array guards{
                    Guard{0xE803D0,{0x48,0x89,0x5C,0x24,0x18,0x48,0x89,0x6C,0x24,0x20,0x89,0x54}},
                    Guard{0xE806B0,{0x4C,0x8B,0xDC,0x49,0x89,0x6B,0x20,0x56,0x41,0x54,0x41,0x55}},
                    Guard{0xE88A10,{0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10,0x48,0x89}},
                    Guard{0xEC39C0,{0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20,0x8B,0x41}},
                    Guard{0xECDE70,{0x48,0x8B,0x49,0x08,0x48,0x85,0xC9,0x0F,0x85,0x93,0x57,0x01}},
                    Guard{0x7D6230,{0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xD9,0xFF,0x15,0x39}},
                    Guard{0x3E8380,{0x48,0x89,0x5C,0x24,0x10,0x44,0x88,0x4C,0x24,0x20,0x56,0x57}},
                    Guard{0x2F30E0,{0x48,0x85,0xC9,0x74,0x15,0x48,0x3B,0x0D,0x7C,0x1C,0x65,0x05}},
                    Guard{0x333740,{0x44,0x89,0x44,0x24,0x18,0x48,0x89,0x54,0x24,0x10,0x55,0x41}},
                    Guard{0x333360,{0x0F,0x57,0xC0,0x48,0x85,0xD2,0x74,0x08,0x8B,0x82,0x10,0x01}},
                    Guard{0xEC4B90,{0x40,0x53,0x48,0x83,0xEC,0x30,0x41,0x8B,0xD8,0x4C,0x8D,0x44}},
                    Guard{0xE4E3B0,{0x44,0x89,0x44,0x24,0x18,0x55,0x41,0x56,0x41,0x57,0x48,0x8D}},
                    Guard{0x3E6840,{0x48,0x89,0x5C,0x24,0x18,0x56,0x48,0x83,0xEC,0x20,0x8B,0x05}}
                };
                for (const auto& guard : guards) {
                    std::array<std::uint8_t, 12> bytes{};
                    if (!native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(guard.rva).address()),
                            bytes.data(), bytes.size()) || bytes != guard.bytes) {
                        ROCK_LOG_ERROR(Weapon, "Akimbo native contract unavailable rva={:X}", guard.rva);
                        return false;
                    }
                }
                return true;
            }();
            return ready;
        }

        bool readItem(std::uint32_t index, RE::EquippedItem& item) noexcept
        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (!player || !player->currentProcess || !player->currentProcess->middleHigh || !nativeReady() ||
                player->currentProcess->middleHigh->equippedItems.size() > native_weapon_qualification::kMaximumEquipped) return false;
            return reinterpret_cast<ReadItem>(REL::Offset(0xE803D0).address())(player->currentProcess, index, &item);
        }

        RE::EquippedWeaponData* weaponData(const RE::EquippedItem& item) noexcept
        {
            std::uintptr_t table{};
            if (!item.item.object || item.item.object->formType != RE::ENUM_FORM_ID::kWEAP || !item.data ||
                !native_memory::tryReadField(item.data.get(), 0, table) || table != REL::Offset(0x2D7FCF8).address()) return nullptr;
            return static_cast<RE::EquippedWeaponData*>(item.data.get());
        }

        struct ShotOrigin
        {
            RE::TESForm* form{};
            RE::TBO_InstanceData* instance{};
            RE::NiTransform world{};
            std::uint32_t index{};
            bool active{}, applied{};
            std::uint32_t launches{}, lastHandle{};
        };
        thread_local ShotOrigin currentShot;

        std::uint32_t reservedReloadCount(RE::AIProcess* process, std::uint32_t index, std::uint32_t requested) noexcept
        {
            const auto carriedIndex = borrowedIndex.load(std::memory_order_acquire);
            const auto expectedData = borrowedData.load(std::memory_order_acquire);
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (!expectedData || !player || player->currentProcess != process || (index != 0 && index != carriedIndex)) return requested;
            auto carried = emptyItem(), primary = emptyItem();
            if (!readItem(carriedIndex, carried) || reinterpret_cast<std::uintptr_t>(carried.data.get()) != expectedData ||
                !readItem(0, primary)) return requested;
            const auto* target = weaponData(index == carriedIndex ? carried : primary);
            const auto* other = weaponData(index == carriedIndex ? primary : carried);
            if (!target || !other || !target->ammo || target->ammo != other->ammo) return requested;
            std::uint32_t total{};
            if (!reinterpret_cast<ItemCount>(REL::Offset(0x3E8380).address())(player, &total, target->ammo, false)) return 0;
            return akimbo::clampReload(requested, total, other->ammoCount);
        }

        void onReloadCount(RE::AIProcess* process, std::uint32_t index, std::uint32_t requested) noexcept
        {
            reinterpret_cast<SetCount>(REL::Offset(0xEC4B90).address())(process, index,
                reservedReloadCount(process, index, requested));
        }

        bool eraseContext(std::uint32_t index, std::uintptr_t expectedData, bool quiescentBoundary) noexcept
        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            auto* process = player ? player->currentProcess : nullptr;
            auto* middle = process ? process->middleHigh : nullptr;
            if (!middle) return true;
            auto* lock = reinterpret_cast<RE::BSSpinLock*>(reinterpret_cast<char*>(middle) + 0x288);
            // Save and lifecycle teardown use the engine's existing equipment
            // lock after gameplay dispatch stops. Live frame cleanup only tries.
            if (quiescentBoundary) lock->lock();
            else {
                using TryLock = bool (*)(RE::BSSpinLock*);
                if (!reinterpret_cast<TryLock>(REL::Offset(0x7D6230).address())(lock)) return false;
            }
            struct ReleaseLock { RE::BSSpinLock* value; ~ReleaseLock() { value->unlock(); } } release{lock};
            auto& items = middle->equippedItems;
            if (items.size() > 4096) return false;
            for (std::uint32_t i = 0; i < items.size(); ++i) {
                auto& item = items[i];
                if (item.equipIndex.index != index || reinterpret_cast<std::uintptr_t>(item.data.get()) != expectedData) continue;
                // E80900 teardown, selected by the exact owned data identity,
                // not its broad base-form/overlapping-slot filter.
                const auto lease = item.data;
                using StopIdle = void (*)(RE::EquippedItemData*, bool);
                reinterpret_cast<StopIdle>(REL::Offset(0xEC39C0).address())(lease.get(), false);
                const std::array<std::uint32_t, 2> event{0, index};
                using Notify = void (*)(RE::AIProcess*, const void*);
                reinterpret_cast<Notify>(REL::Offset(0xECDE70).address())(process, event.data());
                // A synchronous native listener may have changed the array.
                // Re-resolve the exact record instead of erasing an old offset.
                if (process->middleHigh != middle || items.size() > 4096) return false;
                for (std::uint32_t j = 0; j < items.size(); ++j) {
                    if (items[j].equipIndex.index != index || items[j].data.get() != lease.get()) continue;
                    using Erase = void (*)(void*, std::uint32_t, std::uint32_t);
                    reinterpret_cast<Erase>(REL::Offset(0xE88A10).address())(&items, j, 1);
                    break;
                }
                break;
            }
            return true;
        }
    }

    void CarriedWeaponRuntime::beforeSave() noexcept
    {
        saving.store(true, std::memory_order_release);
        const auto expectedData = borrowedData.load(std::memory_order_acquire);
        const auto index = borrowedIndex.load(std::memory_order_acquire);
        if (!expectedData || index == UINT32_MAX) return;
        // Values-only archive and atomic native identity publication are safe
        // for F4SE's save callback; no frame-owned hand/session object is read.
        auto item = emptyItem();
        Archive record{};
        if (readItem(index, item) && reinterpret_cast<std::uintptr_t>(item.data.get()) == expectedData && liveArchive.read(record)) {
            if (const auto* data = weaponData(item)) {
                record.loaded = data->ammoCount;
                (void)liveArchive.write(record);
            }
        }
        if (!eraseContext(index, expectedData, true)) {
            try { ROCK_LOG_ERROR(Weapon, "Akimbo native context could not be removed at the save boundary"); } catch (...) {}
            return;
        }
        borrowedData.store(0, std::memory_order_release);
        borrowedIndex.store(UINT32_MAX, std::memory_order_release);
        saveEpoch.fetch_add(1, std::memory_order_acq_rel);
    }

    void CarriedWeaponRuntime::afterSave() noexcept
    {
        saving.store(false, std::memory_order_release);
    }

    bool CarriedWeaponRuntime::ready() noexcept
    {
        return ammoHookInstalled && native_scope_shot_diagnostics::hooksReady() && !saving.load(std::memory_order_acquire);
    }

    void CarriedWeaponRuntime::noteInteractionThread() noexcept
    {
        interactionThread.store(GetCurrentThreadId(), std::memory_order_release);
    }

    bool CarriedWeaponRuntime::isInteractionThread() noexcept
    {
        return interactionThread.load(std::memory_order_acquire) == GetCurrentThreadId();
    }

    bool CarriedWeaponRuntime::install() noexcept try
    {
        if (ammoHookInstalled) return true;
        if (!nativeReady()) return false;
        auto* serialization = F4SE::GetSerializationInterface();
        if (!serialization) return false;
        // Actor::ReloadWeapon, after native capacity/perk/inventory resolution.
        // Raw setup: MOV RCX,RDI; CMOVC ESI,EBX; MOV R8D,ESI; CALL SetCount.
        constexpr std::array<std::uint8_t, 9> prefix{0x48,0x8B,0xCF,0x0F,0x42,0xF3,0x44,0x8B,0xC6};
        std::array<std::uint8_t, 14> actual{};
        const auto site = REL::Offset(0xE4E709).address();
        if (!native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(site - prefix.size()), actual.data(), actual.size()) ||
            !std::equal(prefix.begin(), prefix.end(), actual.begin()) || actual[9] != 0xE8 ||
            F4SE::GetTrampoline().free_size() < 16) return false;
        std::int32_t displacement{};
        std::memcpy(&displacement, actual.data() + 10, sizeof(displacement));
        if (site + 5 + displacement != REL::Offset(0xEC4B90).address()) return false;
        F4SE::GetTrampoline().write_call<5>(site, &onReloadCount);
        serialization->SetUniqueID(0x4B434F52); // ROCK, private co-save records
        serialization->SetSaveCallback(&saveArchive);
        serialization->SetLoadCallback(&loadArchive);
        serialization->SetRevertCallback(&revertArchive);
        ammoHookInstalled = true;
        return true;
    }
    catch (...) {
        try { ROCK_LOG_ERROR(Weapon, "Akimbo native hook installation failed"); } catch (...) {}
        return false;
    }

    bool CarriedWeaponRuntime::owns(const RE::TESObjectREFR* reference) const noexcept
    {
        return reference && _reference.get() == reference;
    }

    bool CarriedWeaponRuntime::retains(const RE::TESObjectREFR* reference) const noexcept
    {
        if (!reference) return false;
        if (_transfer.valid && _transfer.reference.get().get() == reference && reference->GetObjectReference() == _transfer.form) return true;
        Archive restored{};
        const auto* form = reference->GetObjectReference();
        return loadedArchive.read(restored) && form && restored.reference == reference->formID && restored.weapon == form->formID;
    }

    bool CarriedWeaponRuntime::ownsNativeContext(const RE::EquippedItem& item) noexcept
    {
        return item.data && item.equipIndex.index == borrowedIndex.load(std::memory_order_acquire) &&
            reinterpret_cast<std::uintptr_t>(item.data.get()) == borrowedData.load(std::memory_order_acquire);
    }

    bool CarriedWeaponRuntime::contextCurrent(RE::EquippedItem& item) const noexcept
    {
        return _reference && readItem(_index, item) && item.item.object == _weapon.object &&
            item.item.instanceData.get() == _weapon.instanceData.get() && item.data.get() == _data.get();
    }

    bool CarriedWeaponRuntime::sourceCurrent() const noexcept
    {
        if (!_reference || _reference->GetHandle() != _referenceHandle || _reference->GetObjectReference() != _weapon.object) return false;
        const auto* extra = _reference->extraList ? _reference->extraList->GetByType<RE::ExtraInstanceData>() : nullptr;
        return (extra ? extra->data.get() : nullptr) == _weapon.instanceData.get();
    }

    void CarriedWeaponRuntime::observeAmmo() noexcept
    {
        auto item = emptyItem();
        auto* data = contextCurrent(item) ? weaponData(item) : nullptr;
        _ammoKnown = data && native_memory::tryReadValue(&data->ammoCount, _loaded);
        RE::TESAmmo* ammo{};
        _ammoForm = _ammoKnown && native_memory::tryReadValue(&data->ammo, ammo) && ammo ? ammo->formID : 0;
        if (_reference && _weapon.object && _ammoKnown) {
            (void)liveArchive.write({_reference->formID, _weapon.object->formID, _ammoForm, _loaded,
                _operation.cooldown(), _operation.reloadRemaining(), _operation.reloading() ? 1u : 0u});
        }
    }

    bool CarriedWeaponRuntime::admit(const Input& input)
    {
        if (!input.reference || !ready() || !nativeReady()) return false;
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto primary = emptyItem();
        if (!player || ((!readItem(0, primary) || !weaponData(primary)) && !retains(input.reference))) return false;
        auto* form = input.reference->GetObjectReference();
        auto* weapon = form ? form->As<RE::TESObjectWEAP>() : nullptr;
        if (!weapon || weapon->weaponData.type != RE::WEAPON_TYPE::kGun) return false;
        RE::BSTSmartPointer<RE::TBO_InstanceData> instance{};
        if (input.reference->extraList) {
            if (const auto* extra = input.reference->extraList->GetByType<RE::ExtraInstanceData>()) instance = extra->data;
        }
        auto* slot = RE::TESForm::GetFormByID<RE::BGSEquipSlot>(0x13F43);
        std::uint32_t index = UINT32_MAX;
        if (!slot) return false;
        reinterpret_cast<ResolveIndex>(REL::Offset(0x3E6840).address())(player, &index, slot);
        auto occupied = emptyItem();
        if (index == 0 || index == UINT32_MAX || readItem(index, occupied)) return false;

        const auto* effective = instance ? static_cast<const RE::TESObjectWEAP::InstanceData*>(instance.get()) : &weapon->weaponData;
        // F04DB0 routes these two native ammo identities through player-owned
        // reload selection, before the indexed actor path used here.
        using NeedsPlayerReload = bool (*)(const RE::TESAmmo*);
        if (!effective->ammo || reinterpret_cast<NeedsPlayerReload>(REL::Offset(0x2F30E0).address())(effective->ammo)) return false;
        const float rate = reinterpret_cast<Rate>(REL::Offset(0x333360).address())(weapon, instance.get());
        const auto* ranged = effective->rangedData ? effective->rangedData : weapon->weaponData.rangedData;
        if (!ranged || !std::isfinite(rate) || rate <= 0.0f || !std::isfinite(effective->reloadSpeed) ||
            effective->reloadSpeed <= 0.0f || !std::isfinite(ranged->reloadSeconds) || ranged->reloadSeconds <= 0.0f) return false;
        // Charged operations need their own native admission/completion path;
        // never turn a charge weapon into an ordinary trigger-driven firearm.
        if (effective->flags.any(RE::WEAPON_FLAGS::kChargingAttack, RE::WEAPON_FLAGS::kChargingReload,
                RE::WEAPON_FLAGS::kHoldInputToPower)) return false;

        _reference.reset(input.reference);
        _referenceHandle = input.reference->GetHandle();
        _weapon = RE::BGSObjectInstance(weapon, instance.get());
        _slot = slot;
        _index = index;
        _thread = GetCurrentThreadId();
        _saveEpoch = saveEpoch.load(std::memory_order_acquire);
        _secondsPerShot = 1.0f / rate;
        _reloadSeconds = ranged->reloadSeconds / effective->reloadSpeed;
        _automatic = effective->flags.any(RE::WEAPON_FLAGS::kAutomatic);
        _operation.begin(_nextSession++);
        _operation.bind(input.hand, input.grip);
        if (!publishContext()) { _faulted = true; clear(true); return false; }
        auto admitted = emptyItem();
        auto* admittedData = contextCurrent(admitted) ? weaponData(admitted) : nullptr;
        Archive restored{};
        const bool restoreSaved = loadedArchive.read(restored) && restored.reference == input.reference->formID &&
            restored.weapon == form->formID && admittedData && admittedData->ammo && admittedData->ammo->formID == restored.ammo;
        const bool restoreTransfer = _transfer.valid && _transfer.reference == _referenceHandle && _transfer.form == form &&
            admittedData && admittedData->ammo && admittedData->ammo->formID == _transfer.ammo;
        if (restoreTransfer || restoreSaved) {
            const auto count = restoreTransfer ? _transfer.loaded : restored.loaded;
            reinterpret_cast<SetCount>(REL::Offset(0xEC4B90).address())(player->currentProcess, _index, count);
            if (restoreSaved) {
                _operation.restore(restored.cooldown, restored.reloadRemaining, (restored.flags & 1) != 0);
                (void)loadedArchive.write({});
            }
            auto currentPrimary = emptyItem();
            if (readItem(0, currentPrimary)) {
                if (const auto* primaryData = weaponData(currentPrimary))
                    onReloadCount(player->currentProcess, 0, primaryData->ammoCount);
            }
            _transfer = {};
        } else {
            reload();
        }
        observeAmmo();
        ROCK_LOG_INFO(Weapon, "Akimbo carried session={} ref={:08X} form={:08X} index={} instance=0x{:X} ammo={:08X} loaded={} valid={}",
            _operation.session(), input.reference->formID, form->formID, _index,
            reinterpret_cast<std::uintptr_t>(_weapon.instanceData.get()), _ammoForm, _loaded, _ammoKnown);
        return true;
    }

    bool CarriedWeaponRuntime::publishContext()
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto primary = emptyItem(), before = emptyItem();
        if (!ready() || !player || !player->currentProcess || !player->currentProcess->middleHigh ||
            player->currentProcess->middleHigh->equippedItems.size() >= native_weapon_qualification::kMaximumEquipped ||
            !sourceCurrent() || readItem(_index, before)) return false;
        const bool hadPrimary = readItem(0, primary) && weaponData(primary);
        const auto primaryCount = hadPrimary ? weaponData(primary)->ammoCount : 0;
        reinterpret_cast<SetItem>(REL::Offset(0xE806B0).address())(player->currentProcess, player, &_weapon, _slot);
        auto item = emptyItem(), after = emptyItem();
        if (!readItem(_index, item) || item.item.object != _weapon.object ||
            item.item.instanceData.get() != _weapon.instanceData.get() || !weaponData(item)) return false;
        _data = item.data;
        _registered = true;
        borrowedData.store(reinterpret_cast<std::uintptr_t>(_data.get()), std::memory_order_release);
        borrowedIndex.store(_index, std::memory_order_release);
        if (hadPrimary && (!readItem(0, after) || after.item.object != primary.item.object ||
            after.item.instanceData.get() != primary.item.instanceData.get() || after.data.get() != primary.data.get() ||
            !weaponData(after) || weaponData(after)->ammoCount != primaryCount)) {
            // The player initializer can touch its shared default clip. Undo
            // that write only while the exact original primary record survives.
            if (after.data.get() == primary.data.get() && weaponData(after))
                reinterpret_cast<SetCount>(REL::Offset(0xEC4B90).address())(player->currentProcess, 0, primaryCount);
            ROCK_LOG_ERROR(Weapon, "Akimbo admission failed primary-preservation check session={}", _operation.session());
            return false;
        }
        return true;
    }

    void CarriedWeaponRuntime::removeContext() noexcept
    {
        if (!_registered || GetCurrentThreadId() != _thread) return;
        if (!eraseContext(_index, reinterpret_cast<std::uintptr_t>(_data.get()), false)) return;
        borrowedData.store(0, std::memory_order_release);
        borrowedIndex.store(UINT32_MAX, std::memory_order_release);
        _registered = false;
    }
    void CarriedWeaponRuntime::clear(bool nativeWorldAvailable) noexcept
    {
        if (nativeWorldAvailable) removeContext();
        if (nativeWorldAvailable && _registered) return;
        borrowedData.store(0, std::memory_order_release);
        borrowedIndex.store(UINT32_MAX, std::memory_order_release);
        _registered = false;
        if (_data) static_cast<RE::EquippedWeaponData*>(_data.get())->fireNode = nullptr;
        _muzzle.reset();
        _reference.reset();
        _weapon = RE::BGSObjectInstance(nullptr, nullptr);
        _data.reset();
        _referenceHandle = {};
        _declinedReference = {};
        _slot = nullptr;
        _ammoKnown = false;
        _operation.begin(0);
        _suspended = false;
        _faulted = false;
        (void)liveArchive.write({});
    }

    void CarriedWeaponRuntime::shutdown(bool nativeWorldAvailable) noexcept
    {
        interactionThread.store(0, std::memory_order_release);
        // PhysicsInteraction has stopped its callbacks before destroying the
        // hands. A lifecycle callback need not use the last frame's thread ID.
        if (nativeWorldAvailable && _registered &&
            !eraseContext(_index, reinterpret_cast<std::uintptr_t>(_data.get()), true)) {
            try { ROCK_LOG_ERROR(Weapon, "Akimbo lifecycle cleanup could not resolve its native context index={}", _index); } catch (...) {}
        }
        clear(false);
        cancelTransfer();
    }

    bool CarriedWeaponRuntime::suspend() noexcept
    {
        if (!_reference) return true;
        if (_suspended) return true;
        if (_faulted) return false;
        observeAmmo();
        removeContext();
        _suspended = !_registered;
        _operation.cancelInput();
        ROCK_LOG_DEBUG(Weapon, "Akimbo suspend session={} thread={} owner={} removed={}",
            _operation.session(), GetCurrentThreadId(), _thread, !_registered);
        return _suspended;
    }

    void CarriedWeaponRuntime::captureTransfer() noexcept
    {
        _transfer = {};
        auto item = emptyItem();
        auto* data = readItem(0, item) ? weaponData(item) : nullptr;
        if (!data || !data->ammo) return;
        _transfer = {item.item.object, {}, data->ammoCount, data->ammo->formID, true};
    }

    void CarriedWeaponRuntime::commitTransfer(RE::ObjectRefHandle reference) noexcept
    {
        if (!_transfer.valid || !reference) { _transfer = {}; return; }
        _transfer.reference = reference;
        if (const auto retained = reference.get()) {
            (void)liveArchive.write({retained->formID, _transfer.form->formID, _transfer.ammo, _transfer.loaded});
        }
    }

    void CarriedWeaponRuntime::cancelTransfer() noexcept { _transfer = {}; }

    bool CarriedWeaponRuntime::preparePrimaryEquip(RE::TESObjectREFR* reference) noexcept
    {
        if (!owns(reference)) return true;
        observeAmmo();
        // Complete weapon-owned operations before returning to the native
        // primary animation path; promotion must not bypass either timer.
        if (!_ammoKnown || _faulted || _operation.reloading() || _operation.cooldown() > 0.0f) return false;
        _transfer = {_weapon.object, _referenceHandle, _loaded, _ammoForm, true, true};
        clear(true); // Pickup must not retain ROCK's old world-reference lease.
        return !hasSession();
    }

    void CarriedWeaponRuntime::finishPrimaryEquip(RE::ObjectRefHandle source, RE::TESObjectWEAP* weapon,
        std::uintptr_t instance, bool committed) noexcept
    {
        if (!_transfer.valid || !_transfer.promoting || !committed || _transfer.form != weapon || source != _transfer.reference) return;
        // The source lease can already be consumed; the exact native transfer
        // result, not a fresh lookup by base form, supplies this instance.
        auto item = emptyItem();
        auto* data = readItem(0, item) ? weaponData(item) : nullptr;
        if (!data || item.item.object != weapon || reinterpret_cast<std::uintptr_t>(item.item.instanceData.get()) != instance ||
            !data->ammo || data->ammo->formID != _transfer.ammo) return;
        auto* player = RE::PlayerCharacter::GetSingleton();
        reinterpret_cast<SetCount>(REL::Offset(0xEC4B90).address())(player->currentProcess, 0, _transfer.loaded);
        _transfer = {};
    }

    void CarriedWeaponRuntime::reload()
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto item = emptyItem();
        if (!player || !contextCurrent(item)) return;
        const bool completed = reinterpret_cast<Reload>(REL::Offset(0xE4E3B0).address())(player, &_weapon, _index);
        observeAmmo();
        _operation.completeReload();
        ROCK_LOG_DEBUG(Weapon, "Akimbo reload session={} index={} completed={} loaded={} known={}",
            _operation.session(), _index, completed, _loaded, _ammoKnown);
    }

    bool CarriedWeaponRuntime::applyShotOrigin(void* launchData) noexcept
    {
        // True means an unrelated native shot needs no override. False during
        // our dispatch makes the origin hook decline the shot, never substitute
        // the primary weapon's muzzle after an identity/read failure.
        if (!currentShot.active) return true;
        if (!launchData) return false;
        std::uintptr_t form{}, instance{};
        std::uint32_t index{};
        if (!native_memory::tryReadField(launchData, 0x30, form) ||
            !native_memory::tryReadField(launchData, 0x38, instance) ||
            !native_memory::tryReadField(launchData, 0x48, index) || index != currentShot.index ||
            form != reinterpret_cast<std::uintptr_t>(currentShot.form) ||
            instance != reinterpret_cast<std::uintptr_t>(currentShot.instance)) return false;
        const auto& rotation = currentShot.world.rotate;
        const auto x = rotation.entry[0][1], y = rotation.entry[1][1], z = rotation.entry[2][1];
        const float yaw = std::atan2(x, y);
        const float pitch = -std::atan2(z, std::sqrt(x*x + y*y));
        const auto& position = currentShot.world.translate;
        if (!std::isfinite(yaw) || !std::isfinite(pitch) || !std::isfinite(position.x) ||
            !std::isfinite(position.y) || !std::isfinite(position.z) || x*x + y*y + z*z < 0.0001f) return false;
        currentShot.applied = native_memory::tryWriteValue(static_cast<RE::NiPoint3*>(launchData), currentShot.world.translate) &&
            native_memory::guardedCopyToMemory(static_cast<char*>(launchData) + 0x4C, &yaw, sizeof(yaw)) &&
            native_memory::guardedCopyToMemory(static_cast<char*>(launchData) + 0x50, &pitch, sizeof(pitch));
        return currentShot.applied;
    }

    void CarriedWeaponRuntime::fire()
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto item = emptyItem();
        if (!ready() || !player || !_muzzle || currentShot.active || !sourceCurrent() || !contextCurrent(item)) return;
        auto* data = weaponData(item);
        if (!data || !data->ammo || !data->ammoCount || !weapon_grip_transfer::validFrame(_muzzle->world)) return;
        data->fireNode = _muzzle.get();
        const auto weapon = _weapon;
        const auto dataLease = _data;
        const auto muzzleLease = _muzzle;
        const auto session = _operation.session();
        currentShot = {weapon.object, weapon.instanceData.get(), muzzleLease->world, _index, true, false};
        struct ClearShot { ~ClearShot() { currentShot = {}; } } clearShot;
        const auto before = data->ammoCount;
        auto primaryBefore = emptyItem(), primaryAfter = emptyItem();
        const auto* primaryData = readItem(0, primaryBefore) ? weaponData(primaryBefore) : nullptr;
        const auto primaryCountBefore = primaryData ? primaryData->ammoCount : 0;
        reinterpret_cast<Fire>(REL::Offset(0x333740).address())(&weapon, player, _index, data->ammo, nullptr);
        const auto result = currentShot;
        currentShot = {};
        observeAmmo();
        primaryData = readItem(0, primaryAfter) ? weaponData(primaryAfter) : nullptr;
        ROCK_LOG_DEBUG(Weapon, "Akimbo shot session={} form={:08X} index={} before={} after={} known={} primaryBefore={} primaryAfter={} primaryKnown={} originApplied={} projectiles={} lastHandle={:08X} origin=({:.3f},{:.3f},{:.3f})",
            session, weapon.object->formID, _index, before, _loaded, _ammoKnown, primaryCountBefore,
            primaryData ? primaryData->ammoCount : 0, primaryData != nullptr, result.applied, result.launches, result.lastHandle,
            result.world.translate.x, result.world.translate.y, result.world.translate.z);
    }

    void CarriedWeaponRuntime::observeShotLaunch(const void* launchData, std::uint32_t handle) noexcept
    {
        if (!currentShot.active || !launchData || !handle) return;
        std::uintptr_t weapon{}, instance{};
        std::uint32_t index{};
        if (native_memory::tryReadField(launchData, 0x30, weapon) &&
            native_memory::tryReadField(launchData, 0x38, instance) &&
            native_memory::tryReadField(launchData, 0x48, index) && index == currentShot.index &&
            weapon == reinterpret_cast<std::uintptr_t>(currentShot.form) &&
            instance == reinterpret_cast<std::uintptr_t>(currentShot.instance)) {
            ++currentShot.launches;
            currentShot.lastHandle = handle;
        }
    }

    void CarriedWeaponRuntime::prepare(const Input& input)
    {
        if (saving.load(std::memory_order_acquire)) return;
        if (_reference && _reference.get() != input.reference) clear(true);
        if (!input.reference) { _declinedReference = {}; return; }
        if (!_reference && _declinedReference != input.reference->GetHandle() && !admit(input)) {
            _declinedReference = input.reference->GetHandle();
            ROCK_LOG_WARN(Weapon, "Akimbo admission declined ref={:08X} form={:08X}; native contract, firearm data or second index unavailable",
                input.reference->formID, input.reference->GetObjectReference() ? input.reference->GetObjectReference()->formID : 0);
        }
    }

    void CarriedWeaponRuntime::update(const Input& input)
    {
        prepare(input);
        if (!ready() || GetCurrentThreadId() != _thread || !owns(input.reference)) return;
        if (_faulted) { removeContext(); return; }
        const auto saved = saveEpoch.load(std::memory_order_acquire);
        if (saved != _saveEpoch) {
            _saveEpoch = saved;
            _registered = false;
            _suspended = true;
            _operation.cancelInput();
            Archive snapshot{};
            if (liveArchive.read(snapshot) && snapshot.reference == _reference->formID && snapshot.weapon == _weapon.object->formID) {
                _loaded = snapshot.loaded;
            }
        }
        if (_suspended) {
            const auto savedCount = _loaded;
            const bool known = _ammoKnown;
            if (!publishContext()) {
                removeContext();
                _faulted = true;
                ROCK_LOG_ERROR(Weapon, "Akimbo resume rejected session={} index={}; held item retained, firing disabled",
                    _operation.session(), _index);
                return;
            }
            if (known) {
                auto* player = RE::PlayerCharacter::GetSingleton();
                reinterpret_cast<SetCount>(REL::Offset(0xEC4B90).address())(player->currentProcess, _index, savedCount);
                auto primary = emptyItem();
                if (readItem(0, primary)) {
                    if (const auto* data = weaponData(primary)) onReloadCount(player->currentProcess, 0, data->ammoCount);
                }
            }
            _suspended = false;
        }
        auto item = emptyItem();
        if (!sourceCurrent() || !contextCurrent(item)) {
            removeContext();
            _faulted = true;
            _operation.cancelInput();
            ROCK_LOG_ERROR(Weapon, "Akimbo identity/context changed session={} index={}; held item retained, firing disabled",
                _operation.session(), _index);
            return;
        }
        _operation.bind(input.hand, input.grip);
        _operation.advance(input.deltaSeconds);
        // Publish the replacement cache while both nodes are still pinned.
        // Native readers must never see a retired node after its lease drops.
        RE::NiPointer<RE::NiAVObject> nextMuzzle(input.muzzle);
        weaponData(item)->fireNode = nextMuzzle.get();
        _muzzle = std::move(nextMuzzle);
        observeAmmo();
        if (!input.inputAllowed) { _operation.cancelInput(); return; }
        if (!_muzzle && input.grip == akimbo::Grip::Firing && input.triggerHeld) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Akimbo shot blocked session={} ref={:08X}: held model has no bounded ProjectileNode",
                _operation.session(), _reference->formID);
        }
        if (input.reloadPressed) (void)_operation.beginReload(_reloadSeconds);
        if (_operation.reloadDue()) reload();
        const auto ticket = _operation.requestFire(_muzzle && input.inputAllowed, input.triggerHeld,
            _automatic, _ammoKnown, _loaded);
        if (_operation.current(ticket)) {
            fire();
            _operation.completeFire(ticket, _secondsPerShot);
        }
        observeAmmo(); // Publish operation timing together with the resulting native count.
    }
}
