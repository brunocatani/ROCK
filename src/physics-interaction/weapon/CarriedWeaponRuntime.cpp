#include "physics-interaction/weapon/CarriedWeaponRuntime.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/NativeWeaponQualification.h"
#include "physics-interaction/weapon/NativeCarriedWeaponContext.h"
#include "physics-interaction/weapon/CarriedWeaponProjectile.h"
#include "physics-interaction/weapon/WeaponGripTransfer.h"
#include "physics-interaction/weapon/telemetry/NativeScopeShotDiagnostics.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/PlayerCharacter.h"
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
        using Fire = void (*)(const RE::BGSObjectInstance*, RE::TESObjectREFR*, std::uint32_t, RE::TESAmmo*, void*);
        using Reload = bool (*)(RE::Actor*, const RE::BGSObjectInstance*, std::uint32_t);
        using Rate = float (*)(RE::TESObjectWEAP*, RE::TBO_InstanceData*);
        using SetCount = void (*)(RE::AIProcess*, std::uint32_t, std::uint32_t);
        using ResolveIndex = std::uint32_t* (*)(RE::Actor*, std::uint32_t*, const RE::BGSEquipSlot*);
        using ItemCount = bool (*)(RE::TESObjectREFR*, std::uint32_t*, RE::TESForm*, bool);
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
        };
        std::array<ArchiveChannel, 2> liveArchives{}, loadedArchives{};
        constexpr std::uint32_t archiveType = 0x424D4B41; // AKMB

        void saveArchive(const F4SE::SerializationInterface* serialization) noexcept
        {
            try {
                for (auto& channel : liveArchives) {
                    Archive record{};
                    if (!channel.read(record)) ROCK_LOG_ERROR(Weapon, "Physical weapon save snapshot unavailable");
                    else if (record.reference && !serialization->WriteRecord(archiveType, 1, &record, sizeof(record)))
                        ROCK_LOG_ERROR(Weapon, "Physical weapon save record could not be written");
                }
            } catch (...) { try { ROCK_LOG_ERROR(Weapon, "Akimbo save callback failed"); } catch (...) {} }
        }

        void loadArchive(const F4SE::SerializationInterface* serialization) noexcept
        {
            try {
                for (auto& channel : loadedArchives) (void)channel.write({});
                unsigned restoredCount = 0;
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
                    if (restoredCount < loadedArchives.size() && !loadedArchives[restoredCount++].write(record))
                        ROCK_LOG_ERROR(Weapon, "Physical weapon restored snapshot publication failed");
                }
            } catch (...) { try { ROCK_LOG_ERROR(Weapon, "Akimbo save data could not be restored"); } catch (...) {} }
        }

        void revertArchive(const F4SE::SerializationInterface*) noexcept
        {
            for (auto& channel : liveArchives) (void)channel.write({});
            for (auto& channel : loadedArchives) (void)channel.write({});
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
                    Guard{0xEC39C0,{0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20,0x8B,0x41}},
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

        struct FirearmTiming { float shot{}, reload{}; bool automatic{}; };
        bool readFirearmTiming(RE::TESObjectWEAP* weapon, RE::TBO_InstanceData* instance, FirearmTiming& timing)
        {
            if (!weapon || weapon->weaponData.type != RE::WEAPON_TYPE::kGun) return false;
            const auto* effective = instance ? static_cast<const RE::TESObjectWEAP::InstanceData*>(instance) : &weapon->weaponData;
            // Player-owned reload selection and charging need separate native
            // operations. Reject them before converting an existing weapon.
            using NeedsPlayerReload = bool (*)(const RE::TESAmmo*);
            if (!effective->ammo || reinterpret_cast<NeedsPlayerReload>(REL::Offset(0x2F30E0).address())(effective->ammo) ||
                effective->flags.any(RE::WEAPON_FLAGS::kChargingAttack, RE::WEAPON_FLAGS::kChargingReload,
                    RE::WEAPON_FLAGS::kHoldInputToPower)) return false;
            const float rate = reinterpret_cast<Rate>(REL::Offset(0x333360).address())(weapon, instance);
            const auto* ranged = effective->rangedData ? effective->rangedData : weapon->weaponData.rangedData;
            if (!ranged || !std::isfinite(rate) || rate <= 0.0f || !std::isfinite(effective->reloadSpeed) ||
                effective->reloadSpeed <= 0.0f || !std::isfinite(ranged->reloadSeconds) || ranged->reloadSeconds <= 0.0f) return false;
            timing = {1.0f / rate, ranged->reloadSeconds / effective->reloadSpeed, effective->flags.any(RE::WEAPON_FLAGS::kAutomatic)};
            return std::isfinite(timing.shot) && std::isfinite(timing.reload);
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
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (!player || player->currentProcess != process) return requested;
            const auto* scoped = native_carried_weapon_context::current(process, index);
            auto primary = emptyItem();
            const auto* target = scoped ? weaponData(*scoped) : readItem(index, primary) ? weaponData(primary) : nullptr;
            if (!target || !target->ammo) return requested;
            std::uint64_t reserved = 0;
            for (unsigned slot = 0; slot < liveArchives.size(); ++slot) {
                if (scoped && slot == index) continue;
                Archive other{};
                if (!liveArchives[slot].read(other)) return (std::min)(requested, target->ammoCount);
                if (other.reference && other.ammo == target->ammo->formID) reserved += other.loaded;
            }
            if (scoped) {
                // During dual entry one original native gun can remain while
                // the second item is prepared. Reserve that magazine too.
                auto native = emptyItem();
                if (native_carried_weapon_context::readNative(process, 0, native)) {
                    const auto* nativeData = weaponData(native);
                    if (nativeData && nativeData->ammo == target->ammo) reserved += nativeData->ammoCount;
                }
            }
            std::uint32_t total{};
            if (!reinterpret_cast<ItemCount>(REL::Offset(0x3E8380).address())(player, &total, target->ammo, false)) return 0;
            return akimbo::clampReload(requested, total, static_cast<std::uint32_t>((std::min)(reserved, std::uint64_t{UINT32_MAX})));
        }

        void onReloadCount(RE::AIProcess* process, std::uint32_t index, std::uint32_t requested) noexcept
        {
            reinterpret_cast<SetCount>(REL::Offset(0xEC4B90).address())(process, index,
                reservedReloadCount(process, index, requested));
        }

    }

    void PhysicalWeaponSession::beforeSave() noexcept
    {
        // The carried data is private, so native save/load never serializes
        // an extra equipped item. Its value snapshot belongs to the co-save.
        saving.store(true, std::memory_order_release);
    }

    void PhysicalWeaponSession::afterSave() noexcept
    {
        saving.store(false, std::memory_order_release);
    }

    bool PhysicalWeaponSession::ready() noexcept
    {
        return ammoHookInstalled && native_scope_shot_diagnostics::hooksReady() && !saving.load(std::memory_order_acquire);
    }

    void PhysicalWeaponSession::noteInteractionThread() noexcept
    {
        interactionThread.store(GetCurrentThreadId(), std::memory_order_release);
    }

    bool PhysicalWeaponSession::isInteractionThread() noexcept
    {
        return interactionThread.load(std::memory_order_acquire) == GetCurrentThreadId();
    }

    bool PhysicalWeaponSession::install() noexcept try
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
        if (!native_carried_weapon_context::install() || !carried_weapon_projectile::install()) return false;
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

    bool PhysicalWeaponSession::owns(const RE::TESObjectREFR* reference) const noexcept
    {
        return reference && _reference.get() == reference;
    }

    bool PhysicalWeaponSession::retains(const RE::TESObjectREFR* reference) const noexcept
    {
        if (!reference) return false;
        if (_transfer.valid && _transfer.reference.get().get() == reference && reference->GetObjectReference() == _transfer.form) return true;
        Archive restored{};
        const auto* form = reference->GetObjectReference();
        return loadedArchives[_slotNumber].read(restored) && form && restored.reference == reference->formID && restored.weapon == form->formID;
    }

    bool PhysicalWeaponSession::contextCurrent(RE::EquippedItem& item) const noexcept
    {
        if (!_registered || !_reference || !_data) return false;
        item.item = _weapon;
        item.equipSlot = _slot;
        item.equipIndex.index = _index;
        item.data = _data;
        return weaponData(item) != nullptr;
    }

    bool PhysicalWeaponSession::sourceCurrent() const noexcept
    {
        if (!_reference || _reference->GetHandle() != _referenceHandle || _reference->GetObjectReference() != _weapon.object) return false;
        const auto* extra = _reference->extraList ? _reference->extraList->GetByType<RE::ExtraInstanceData>() : nullptr;
        return (extra ? extra->data.get() : nullptr) == _weapon.instanceData.get();
    }

    void PhysicalWeaponSession::observeAmmo() noexcept
    {
        auto item = emptyItem();
        auto* data = contextCurrent(item) ? weaponData(item) : nullptr;
        _ammoKnown = data && native_memory::tryReadValue(&data->ammoCount, _loaded);
        RE::TESAmmo* ammo{};
        _ammoForm = _ammoKnown && native_memory::tryReadValue(&data->ammo, ammo) && ammo ? ammo->formID : 0;
        if (_reference && _weapon.object && _ammoKnown) {
            (void)liveArchives[_slotNumber].write({_reference->formID, _weapon.object->formID, _ammoForm, _loaded,
                _operation.cooldown(), _operation.reloadRemaining(), _operation.reloading() ? 1u : 0u});
        }
    }

    bool PhysicalWeaponSession::admit(const Input& input)
    {
        if (!input.reference || !ready() || !nativeReady()) return false;
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player || !player->currentProcess || !player->currentProcess->middleHigh || _slotNumber >= 2) return false;
        auto* form = input.reference->GetObjectReference();
        auto* weapon = form ? form->As<RE::TESObjectWEAP>() : nullptr;
        if (!weapon || weapon->weaponData.type != RE::WEAPON_TYPE::kGun) return false;
        RE::BSTSmartPointer<RE::TBO_InstanceData> instance{};
        if (input.reference->extraList) {
            if (const auto* extra = input.reference->extraList->GetByType<RE::ExtraInstanceData>()) instance = extra->data;
        }
        auto* slot = RE::TESForm::GetFormByID<RE::BGSEquipSlot>(_slotNumber ? 0x13F43 : 0x13F42);
        std::uint32_t index = UINT32_MAX;
        if (!slot) return false;
        reinterpret_cast<ResolveIndex>(REL::Offset(0x3E6840).address())(player, &index, slot);
        if (index != _slotNumber) return false;

        FirearmTiming timing{};
        if (!readFirearmTiming(weapon, instance.get(), timing)) return false;

        _reference.reset(input.reference);
        _referenceHandle = input.reference->GetHandle();
        _weapon = RE::BGSObjectInstance(weapon, instance.get());
        _slot = slot;
        _index = index;
        _thread = GetCurrentThreadId();
        _secondsPerShot = timing.shot;
        _reloadSeconds = timing.reload;
        _automatic = timing.automatic;
        _operation.begin((_nextSession++ << 1) | _slotNumber);
        _operation.bind(input.hand, input.grip);
        if (!publishContext()) { _faulted = true; clear(true); return false; }
        auto admitted = emptyItem();
        auto* admittedData = contextCurrent(admitted) ? weaponData(admitted) : nullptr;
        Archive restored{};
        const bool restoreSaved = loadedArchives[_slotNumber].read(restored) && restored.reference == input.reference->formID &&
            restored.weapon == form->formID && admittedData && admittedData->ammo && admittedData->ammo->formID == restored.ammo;
        const bool restoreTransfer = _transfer.valid && _transfer.reference == _referenceHandle && _transfer.form == form &&
            admittedData && admittedData->ammo && admittedData->ammo->formID == _transfer.ammo;
        if (restoreTransfer || restoreSaved) {
            const auto count = restoreTransfer ? _transfer.loaded : restored.loaded;
            admittedData->ammoCount = count;
            observeAmmo();
            if (restoreSaved) {
                _operation.restore(restored.cooldown, restored.reloadRemaining, (restored.flags & 1) != 0);
                (void)loadedArchives[_slotNumber].write({});
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
        carried_weapon_projectile::publish(input.reference->formID, player->GetHandle().native_handle(),
            reinterpret_cast<std::uintptr_t>(_weapon.object),
            reinterpret_cast<std::uintptr_t>(_weapon.instanceData.get()), _index);
        ROCK_LOG_INFO(Weapon, "Akimbo private carried session={} ref={:08X} form={:08X} index={} instance=0x{:X} ammo={:08X} loaded={} valid={}",
            _operation.session(), input.reference->formID, form->formID, _index,
            reinterpret_cast<std::uintptr_t>(_weapon.instanceData.get()), _ammoForm, _loaded, _ammoKnown);
        return true;
    }

    bool PhysicalWeaponSession::publishContext()
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!ready() || !player || !player->currentProcess || !sourceCurrent()) return false;
        _data = native_carried_weapon_context::create(player, _weapon, _index);
        if (!_data) return false;
        const auto* weapon = static_cast<const RE::TESObjectWEAP*>(_weapon.object);
        const auto* effective = _weapon.instanceData ?
            static_cast<const RE::TESObjectWEAP::InstanceData*>(_weapon.instanceData.get()) : &weapon->weaponData;
        auto* data = static_cast<RE::EquippedWeaponData*>(_data.get());
        data->ammo = effective->ammo;
        _registered = true;
        return true;
    }

    void PhysicalWeaponSession::removeContext() noexcept
    {
        carried_weapon_projectile::clear(_slotNumber);
        if (!_registered) return;
        using StopIdle = void (*)(RE::EquippedItemData*, bool);
        if (_data) reinterpret_cast<StopIdle>(REL::Offset(0xEC39C0).address())(_data.get(), false);
        _registered = false;
    }
    void PhysicalWeaponSession::clear(bool nativeWorldAvailable) noexcept
    {
        if (!physics.clear(nativeWorldAvailable)) return;
        carried_weapon_projectile::clear(_slotNumber);
        _cycle.clear();
        if (nativeWorldAvailable) removeContext();
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
        _faulted = false;
        (void)liveArchives[_slotNumber].write({});
    }

    void PhysicalWeaponSession::shutdown(bool nativeWorldAvailable) noexcept
    {
        interactionThread.store(0, std::memory_order_release);
        clear(nativeWorldAvailable);
        cancelTransfer();
    }

    bool PhysicalWeaponSession::suspend() noexcept
    {
        // Equip notifications cannot evict a private context. Only clear
        // pending physical input so a native transition cannot replay it.
        _operation.cancelInput();
        return true;
    }

    bool PhysicalWeaponSession::captureTransfer() noexcept
    {
        _transfer = {};
        auto item = emptyItem();
        auto* data = readItem(0, item) ? weaponData(item) : nullptr;
        if (!data || !data->ammo || !item.item.object) return false;
        FirearmTiming timing{};
        if (!readFirearmTiming(item.item.object->As<RE::TESObjectWEAP>(), item.item.instanceData.get(), timing)) return false;
        _transfer = {item.item.object, {}, data->ammoCount, data->ammo->formID, true};
        return true;
    }

    void PhysicalWeaponSession::commitTransfer(RE::ObjectRefHandle reference) noexcept
    {
        if (!_transfer.valid || !reference) { _transfer = {}; return; }
        _transfer.reference = reference;
        if (const auto retained = reference.get()) {
            (void)liveArchives[_slotNumber].write({retained->formID, _transfer.form->formID, _transfer.ammo, _transfer.loaded});
        }
    }

    void PhysicalWeaponSession::cancelTransfer() noexcept
    {
        _transfer = {};
        if (!_reference) (void)liveArchives[_slotNumber].write({});
    }

    bool PhysicalWeaponSession::preparePrimaryEquip(RE::TESObjectREFR* reference) noexcept
    {
        if (!owns(reference)) {
            if (_transfer.valid && reference && _transfer.reference == reference->GetHandle()) _transfer.promoting = true;
            return true;
        }
        observeAmmo();
        // Complete weapon-owned operations before returning to the native
        // primary animation path; promotion must not bypass either timer.
        if (!canReturnToNative()) return false;
        _transfer = {_weapon.object, _referenceHandle, _loaded, _ammoForm, true, true};
        clear(true); // Pickup must not retain ROCK's old world-reference lease.
        return !hasSession();
    }

    void PhysicalWeaponSession::finishPrimaryEquip(RE::ObjectRefHandle source, RE::TESObjectWEAP* weapon,
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

    void PhysicalWeaponSession::reload()
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto item = emptyItem();
        if (!player || !player->currentProcess || !contextCurrent(item)) return;
        native_carried_weapon_context::Scope context(player->currentProcess, item);
        const bool completed = reinterpret_cast<Reload>(REL::Offset(0xE4E3B0).address())(player, &_weapon, _index);
        observeAmmo();
        _operation.completeReload();
        ROCK_LOG_DEBUG(Weapon, "Akimbo reload session={} index={} completed={} loaded={} known={}",
            _operation.session(), _index, completed, _loaded, _ammoKnown);
    }

    bool PhysicalWeaponSession::applyShotOrigin(void* launchData) noexcept
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

    void PhysicalWeaponSession::fire()
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto item = emptyItem();
        if (!ready() || !player || !player->currentProcess || !_muzzle || currentShot.active || !sourceCurrent() || !contextCurrent(item)) return;
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
        Archive peerBefore{}, peerAfter{};
        const bool peerKnownBefore = liveArchives[_slotNumber ^ 1u].read(peerBefore) && peerBefore.reference;
        {
            native_carried_weapon_context::Scope context(player->currentProcess, item);
            reinterpret_cast<Fire>(REL::Offset(0x333740).address())(&weapon, player, _index, data->ammo, nullptr);
        }
        const auto result = currentShot;
        currentShot = {};
        observeAmmo();
        if (result.applied && result.launches && _ammoKnown && _loaded < before) _cycle.fire();
        const bool peerKnownAfter = liveArchives[_slotNumber ^ 1u].read(peerAfter) && peerAfter.reference;
        ROCK_LOG_DEBUG(Weapon, "Physical weapon shot session={} slot={} form={:08X} before={} after={} known={} peerForm={:08X} peerBefore={} peerAfter={} peerKnown={} originApplied={} projectiles={} lastHandle={:08X} origin=({:.3f},{:.3f},{:.3f})",
            session, _slotNumber, weapon.object->formID, before, _loaded, _ammoKnown,
            peerBefore.weapon, peerBefore.loaded, peerAfter.loaded,
            peerKnownBefore && peerKnownAfter && peerBefore.reference == peerAfter.reference,
            result.applied, result.launches, result.lastHandle,
            result.world.translate.x, result.world.translate.y, result.world.translate.z);
    }

    void PhysicalWeaponSession::observeShotLaunch(const void* launchData, std::uint32_t handle) noexcept
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

    void PhysicalWeaponSession::prepare(const Input& input)
    {
        _cycle.reap();
        if (saving.load(std::memory_order_acquire)) return;
        if (_reference && _reference.get() != input.reference) clear(true);
        if (!input.reference) { _declinedReference = {}; return; }
        if (!_reference && _declinedReference != input.reference->GetHandle() && !admit(input)) {
            _declinedReference = input.reference->GetHandle();
            ROCK_LOG_WARN(Weapon, "Akimbo admission declined ref={:08X} form={:08X}; native contract, firearm data or second index unavailable",
                input.reference->formID, input.reference->GetObjectReference() ? input.reference->GetObjectReference()->formID : 0);
        }
        if (owns(input.reference) && !_cycle.ready()) _cycle.update(input.reference, 0.0f);
    }

    void PhysicalWeaponSession::update(const Input& input)
    {
        prepare(input);
        if (!ready() || GetCurrentThreadId() != _thread || !owns(input.reference)) return;
        if (_faulted) { removeContext(); return; }
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
        _cycle.update(input.reference, input.deltaSeconds);
        // Publish the replacement cache while both nodes are still pinned.
        // Native readers must never see a retired node after its lease drops.
        RE::NiPointer<RE::NiAVObject> nextMuzzle(input.muzzle);
        weaponData(item)->fireNode = nextMuzzle.get();
        _muzzle = std::move(nextMuzzle);
        observeAmmo();
        // An accepted reload belongs to this item and must finish before
        // restoration to native single-weapon handling can proceed.
        if (_operation.reloadDue()) reload();
        if (!input.inputAllowed) { _operation.cancelInput(); return; }
        if (!_muzzle && input.grip == akimbo::Grip::Firing && input.triggerHeld) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Akimbo shot blocked session={} ref={:08X}: held model has no bounded ProjectileNode",
                _operation.session(), _reference->formID);
        }
        if (input.reloadPressed) (void)_operation.beginReload(_reloadSeconds);
        const auto ticket = _operation.requestFire(_muzzle && input.inputAllowed, input.triggerHeld,
            _automatic, _ammoKnown, _loaded);
        if (_operation.current(ticket)) {
            fire();
            _operation.completeFire(ticket, _secondsPerShot);
        }
        observeAmmo(); // Publish operation timing together with the resulting native count.
    }
}
