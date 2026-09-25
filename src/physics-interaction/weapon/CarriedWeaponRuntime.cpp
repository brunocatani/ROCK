#include "physics-interaction/weapon/CarriedWeaponRuntime.h"
#include "experimental/LooseReloadBridge.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/NativeWeaponQualification.h"
#include "physics-interaction/weapon/NativeCarriedWeaponContext.h"
#include "physics-interaction/weapon/CarriedWeaponProjectile.h"
#include "physics-interaction/weapon/LooseWeaponExperimentPolicy.h"
#include "physics-interaction/weapon/WeaponGripTransfer.h"
#include "physics-interaction/weapon/telemetry/NativeScopeShotDiagnostics.h"
#include "physics-interaction/weapon/telemetry/NativeScopeShotPolicy.h"
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
        } liveArchive, loadedArchive;
        constexpr std::uint32_t archiveType = 0x424D4B41; // AKMB

        void saveArchive(const F4SE::SerializationInterface* serialization) noexcept
        {
            try {
                Archive record{};
                if (!liveArchive.read(record)) {
                    ROCK_LOG_ERROR(Weapon, "Akimbo save snapshot unavailable");
                    return;
                }
                if (!record.reference && !loadedArchive.read(record)) return;
                if (record.reference && !serialization->WriteRecord(archiveType, 1, &record, sizeof(record))) {
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
                    Guard{0xEC39C0,{0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20,0x8B,0x41}},
                    Guard{0xEC3910,{0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20,0x48,0x8B}},
                    Guard{0x104BB50,{0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x30,0x48,0x83}},
                    Guard{0x834DF0,{0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x40,0x8B,0x41}},
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
            native_scope_shot_policy::Point lastLaunchDirection{};
            float maxLaunchDeltaDegrees{};
            std::uint32_t angleSamples{};
        };
        thread_local ShotOrigin currentShot;

        std::uint32_t reservedReloadCount(RE::AIProcess* process, std::uint32_t index, std::uint32_t requested) noexcept
        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (!player || player->currentProcess != process) return requested;
            auto primary = emptyItem();
            if (!readItem(0, primary)) return requested;
            const auto* primaryData = weaponData(primary);
            if (!primaryData || !primaryData->ammo) return requested;
            RE::TESAmmo* targetAmmo{};
            std::uint32_t otherLoaded{};
            if (index == 0) {
                Archive carried{};
                if (!liveArchive.read(carried)) return (std::min)(requested, primaryData->ammoCount);
                if (!carried.reference || carried.ammo != primaryData->ammo->formID) return requested;
                targetAmmo = primaryData->ammo;
                otherLoaded = carried.loaded;
            } else {
                const auto* scoped = native_carried_weapon_context::current(process, index);
                const auto* carried = scoped ? weaponData(*scoped) : nullptr;
                if (!carried || carried->ammo != primaryData->ammo) return requested;
                targetAmmo = carried->ammo;
                otherLoaded = primaryData->ammoCount;
            }
            std::uint32_t total{};
            if (!reinterpret_cast<ItemCount>(REL::Offset(0x3E8380).address())(player, &total, targetAmmo, false)) return 0;
            return akimbo::clampReload(requested, total, otherLoaded);
        }

        void onReloadCount(RE::AIProcess* process, std::uint32_t index, std::uint32_t requested) noexcept
        {
            reinterpret_cast<SetCount>(REL::Offset(0xEC4B90).address())(process, index,
                reservedReloadCount(process, index, requested));
        }

    }

    void CarriedWeaponRuntime::beforeSave() noexcept
    {
        loose_reload_bridge::clear();
        // The carried data is private, so native save/load never serializes
        // an extra equipped item. Its value snapshot belongs to the co-save.
        saving.store(true, std::memory_order_release);
        carried_weapon_projectile::clearEffects();
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

    bool CarriedWeaponRuntime::owns(const RE::TESObjectREFR* reference) const noexcept
    {
        return reference && _reference.get() == reference;
    }

    bool CarriedWeaponRuntime::isLooseFirearm(const RE::TESObjectREFR* reference) noexcept
    {
        auto* form = reference ? reference->GetObjectReference() : nullptr;
        const auto* weapon = form ? form->As<RE::TESObjectWEAP>() : nullptr;
        return weapon && weapon->weaponData.type == RE::WEAPON_TYPE::kGun;
    }

    bool CarriedWeaponRuntime::nativeWeaponAbsent() noexcept
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!ready() || !player || !player->currentProcess || !player->currentProcess->middleHigh ||
            player->currentProcess->middleHigh->equippedItems.size() > native_weapon_qualification::kMaximumEquipped) return false;
        auto item = emptyItem();
        return !readItem(0, item) || !item.item.object || item.item.object->formType != RE::ENUM_FORM_ID::kWEAP;
    }

    bool CarriedWeaponRuntime::retains(const RE::TESObjectREFR* reference) const noexcept
    {
        if (!reference) return false;
        if (_transfer.valid && _transfer.reference.get().get() == reference && reference->GetObjectReference() == _transfer.form) return true;
        Archive restored{};
        const auto* form = reference->GetObjectReference();
        return loadedArchive.read(restored) && form && restored.reference == reference->formID && restored.weapon == form->formID;
    }

    bool CarriedWeaponRuntime::contextCurrent(RE::EquippedItem& item) const noexcept
    {
        if (!_registered || !_reference || !_data) return false;
        item.item = _weapon;
        item.equipSlot = _slot;
        item.equipIndex.index = _index;
        item.data = _data;
        return weaponData(item) != nullptr;
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
        if (_ammoKnown && !_ammoForm && _weapon.object) {
            const auto* weapon = static_cast<const RE::TESObjectWEAP*>(_weapon.object);
            const auto* effective = _weapon.instanceData ? static_cast<const RE::TESObjectWEAP::InstanceData*>(_weapon.instanceData.get()) : &weapon->weaponData;
            _ammoForm = effective->ammo ? effective->ammo->formID : 0;
        }
        if (_reference && _weapon.object && _ammoKnown) {
            (void)liveArchive.write({_reference->formID, _weapon.object->formID, _ammoForm, _loaded,
                _operation.cooldown(), _operation.reloadRemaining(), _operation.reloading() ? 1u : 0u});
        }
    }

    bool CarriedWeaponRuntime::admit(const Input& input)
    {
        if (!input.reference || !ready() || !nativeReady()) return false;
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player || !loose_weapon_experiment::canAdmit(ready(), !nativeWeaponAbsent(), input.reference->formID)) return false;
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
        _secondsPerShot = 1.0f / rate;
        _reloadSeconds = ranged->reloadSeconds / effective->reloadSpeed;
        _reloadSpeed = effective->reloadSpeed;
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
            admittedData->ammoCount = count;
            observeAmmo();
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
        carried_weapon_projectile::publish(input.reference->formID, player->GetHandle().native_handle(),
            reinterpret_cast<std::uintptr_t>(_weapon.object),
            reinterpret_cast<std::uintptr_t>(_weapon.instanceData.get()), _index);
        ROCK_LOG_INFO(Weapon, "Loose firearm admitted nativePrimary=absent session={} ref={:08X} form={:08X} index={} instance=0x{:X} ammo={:08X} loaded={} valid={}",
            _operation.session(), input.reference->formID, form->formID, _index,
            reinterpret_cast<std::uintptr_t>(_weapon.instanceData.get()), _ammoForm, _loaded, _ammoKnown);
        return true;
    }

    bool CarriedWeaponRuntime::publishContext()
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

    void CarriedWeaponRuntime::stopAttackSound(const char* reason) noexcept
    {
        if (!_data || !nativeReady()) return;
        const auto lease = _data;
        auto* data = static_cast<RE::EquippedWeaponData*>(lease.get());
        const auto attack = data->attackSound.soundID;
        if (attack == UINT32_MAX) return;
        // EC3700 starts this handle; E51910 ends it via EC3910 when leaving
        // attack state 16. EC2FD0 uses the same stop during destruction.
        // Unlike EC39C0 (idle only), this preserves native reverb-tail rules.
        using StopAttack = void (*)(RE::EquippedWeaponData*);
        reinterpret_cast<StopAttack>(REL::Offset(0xEC3910).address())(data);
        // A still-loading handle can survive the native call. Keep checking
        // it on inactive frames rather than forgetting a pending sound.
        try {
            ROCK_LOG_SAMPLE_INFO(Weapon, 250,
                "Loose firearm attack sound stop session={} ref={:08X} reason={} attack={:08X}->{:08X} reverb={:08X} tail={:08X}",
                _operation.session(), _reference ? _reference->formID : 0, reason,
                attack, data->attackSound.soundID, data->reverbSound.soundID, data->prevReverb.soundID);
        } catch (...) {}
    }

    void CarriedWeaponRuntime::removeContext() noexcept
    {
        loose_reload_bridge::clear();
        carried_weapon_projectile::clear();
        if (!_registered) return;
        stopAttackSound("context-retired");
        using StopIdle = void (*)(RE::EquippedItemData*, bool);
        if (_data) reinterpret_cast<StopIdle>(REL::Offset(0xEC39C0).address())(_data.get(), false);
        _registered = false;
    }
    void CarriedWeaponRuntime::clear(bool nativeWorldAvailable) noexcept
    {
        loose_reload_bridge::clear();
        if (nativeWorldAvailable && isInteractionThread() && _reference && _ammoKnown && !_faulted && !saving.load(std::memory_order_acquire)) {
            observeAmmo();
            Archive last{};
            if (liveArchive.read(last) && last.reference) (void)loadedArchive.write(last);
        }
        carried_weapon_projectile::clear();
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
        _kick = {};
        _operation.begin(0);
        _faulted = false;
        (void)liveArchive.write({});
    }

    void CarriedWeaponRuntime::shutdown(bool nativeWorldAvailable) noexcept
    {
        interactionThread.store(0, std::memory_order_release);
        clear(nativeWorldAvailable);
        cancelTransfer();
    }

    bool CarriedWeaponRuntime::suspend() noexcept
    {
        loose_reload_bridge::clear();
        // Equip notifications cannot evict a private context. Only clear
        // pending physical input so a native transition cannot replay it.
        _operation.cancelInput();
        stopAttackSound("suspended");
        carried_weapon_projectile::clearEffects();
        _kick = {};
        return true;
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

    void CarriedWeaponRuntime::cancelTransfer() noexcept
    {
        _transfer = {};
        if (!_reference) (void)liveArchive.write({});
    }

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
        if (!player || !player->currentProcess || !contextCurrent(item)) return;
        // An unsuccessful native reload may clear its selected ammo pointer.
        // Re-select only this exact instance's ammo before another attempt.
        const auto* weapon = static_cast<const RE::TESObjectWEAP*>(_weapon.object);
        const auto* effective = _weapon.instanceData ? static_cast<const RE::TESObjectWEAP::InstanceData*>(_weapon.instanceData.get()) : &weapon->weaponData;
        weaponData(item)->ammo = effective->ammo;
        native_carried_weapon_context::Scope context(player->currentProcess, item);
        const bool completed = reinterpret_cast<Reload>(REL::Offset(0xE4E3B0).address())(player, &_weapon, _index);
        observeAmmo();
        _operation.completeReload();
        _cycle.finishReload();
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
        const auto& position = currentShot.world.translate;
        const auto direction = native_scope_shot_policy::nodeAxisRay(
            {position.x, position.y, position.z}, currentShot.world.rotate, 1);
        const auto angles = native_scope_shot_policy::launchAngles(direction);
        if (!angles.valid) return false;
        currentShot.applied = native_memory::tryWriteValue(static_cast<RE::NiPoint3*>(launchData), currentShot.world.translate) &&
            native_memory::guardedCopyToMemory(static_cast<char*>(launchData) + 0x4C, &angles.yaw, sizeof(angles.yaw)) &&
            native_memory::guardedCopyToMemory(static_cast<char*>(launchData) + 0x50, &angles.pitch, sizeof(angles.pitch));
        return currentShot.applied;
    }

    void CarriedWeaponRuntime::fire()
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto item = emptyItem();
        if (!ready() || !nativeWeaponAbsent() || !player || !player->currentProcess || !_muzzle || currentShot.active || !sourceCurrent() || !contextCurrent(item)) {
            stopAttackSound("dispatch-unavailable");
            return;
        }
        auto* data = weaponData(item);
        if (!data || !data->ammo || !data->ammoCount || !weapon_grip_transfer::validFrame(_muzzle->world)) {
            stopAttackSound("dispatch-invalid");
            return;
        }
        data->fireNode = _muzzle.get();
        const auto weapon = _weapon;
        const auto dataLease = _data;
        const auto muzzleLease = _muzzle;
        const RE::NiPointer<RE::NiAVObject> rootLease(_reference->Get3D());
        const auto session = _operation.session();
        const auto shotHand = _operation.hand();
        const auto referenceId = _reference->formID;
        currentShot = {weapon.object, weapon.instanceData.get(), muzzleLease->world, _index, true, false};
        struct ClearShot { ~ClearShot() { currentShot = {}; } } clearShot;
        const auto before = data->ammoCount;
        auto primaryBefore = emptyItem(), primaryAfter = emptyItem();
        const auto* primaryData = readItem(0, primaryBefore) ? weaponData(primaryBefore) : nullptr;
        const auto primaryCountBefore = primaryData ? primaryData->ammoCount : 0;
        {
            native_carried_weapon_context::Scope context(player->currentProcess, item, rootLease.get());
            reinterpret_cast<Fire>(REL::Offset(0x333740).address())(&weapon, player, _index, data->ammo, nullptr);
        }
        const auto result = currentShot;
        currentShot = {};
        observeAmmo();
        if (result.applied && result.launches && _ammoKnown && _loaded < before) {
            _cycle.fire(_secondsPerShot);
            const auto* enabled = f4vr::getIniSetting("bUseKickback:VR");
            const auto* minOffset = f4vr::getIniSetting("fKickbackMinOffset:VR");
            const auto* maxOffset = f4vr::getIniSetting("fKickbackMaxOffset:VR");
            const auto* minDuration = f4vr::getIniSetting("fKickbackMinDuration:VR");
            const auto* maxDuration = f4vr::getIniSetting("fKickbackMaxDuration:VR");
            if (data->aimModel && enabled && enabled->GetBinary() && minOffset && maxOffset && minDuration && maxDuration) {
                const auto& target = data->aimModel->targetRecoilHead;
                const auto& current = data->aimModel->currentRecoilHead;
                _kick.fire(std::hypot(target.x, target.y), std::hypot(current.x, current.y),
                    minOffset->GetFloat(), maxOffset->GetFloat(), minDuration->GetFloat(), maxDuration->GetFloat());
            }
        }
        if (!result.applied || !result.launches) stopAttackSound("dispatch-failed");
        primaryData = readItem(0, primaryAfter) ? weaponData(primaryAfter) : nullptr;
        const auto& p = result.world.translate;
        const auto expected = native_scope_shot_policy::nodeAxisRay({p.x, p.y, p.z}, result.world.rotate, 1);
        const auto& r = result.world.rotate.entry;
        const auto transposed = native_scope_shot_policy::ray({p.x, p.y, p.z}, {r[0][1], r[1][1], r[2][1]});
        ROCK_LOG_SAMPLE_INFO(Weapon, 250,
            "Loose firearm aim ref={:08X} form={:08X} hand={} nativePrimary={} muzzle=({:.4f},{:.4f},{:.4f}) transposed=({:.4f},{:.4f},{:.4f}) launch=({:.4f},{:.4f},{:.4f}) maxLaunchDeltaDeg={:.3f} angleSamples={} projectiles={} originApplied={} origin=({:.3f},{:.3f},{:.3f})",
            referenceId, weapon.object->formID, shotHand == akimbo::Hand::Left ? "left" : "right",
            primaryData != nullptr, expected.direction.x, expected.direction.y, expected.direction.z,
            transposed.direction.x, transposed.direction.y, transposed.direction.z,
            result.lastLaunchDirection.x, result.lastLaunchDirection.y, result.lastLaunchDirection.z,
            result.maxLaunchDeltaDegrees, result.angleSamples, result.launches, result.applied, p.x, p.y, p.z);
        ROCK_LOG_DEBUG(Weapon, "Loose firearm shot session={} form={:08X} index={} before={} after={} known={} primaryBefore={} primaryAfter={} primaryKnown={} originApplied={} projectiles={} lastHandle={:08X} origin=({:.3f},{:.3f},{:.3f})",
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
            float yaw{}, pitch{};
            if (native_memory::tryReadField(launchData, 0x4C, yaw) && native_memory::tryReadField(launchData, 0x50, pitch)) {
                const auto launch = native_scope_shot_policy::launchRay({}, yaw, pitch);
                const auto expected = native_scope_shot_policy::nodeAxisRay({}, currentShot.world.rotate, 1);
                if (launch.valid && expected.valid) {
                    currentShot.lastLaunchDirection = launch.direction;
                    currentShot.maxLaunchDeltaDegrees = (std::max)(currentShot.maxLaunchDeltaDegrees,
                        native_scope_shot_policy::angleDegrees(expected, launch));
                    ++currentShot.angleSamples;
                }
            }
        }
    }

    void CarriedWeaponRuntime::prepare(const Input& input)
    {
        _cycle.reap();
        if (saving.load(std::memory_order_acquire)) {
            _operation.cancelInput();
            stopAttackSound("saving");
            return;
        }
        if (_reference && _reference.get() != input.reference) clear(true);
        if (!input.reference) { _declinedReference = {}; return; }
        if (!nativeWeaponAbsent()) {
            _operation.cancelInput();
            stopAttackSound("native-weapon-present");
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000, "Loose firearm waiting: unequip the native weapon; experiment accepts one loose gun only");
            return;
        }
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
        auto item = emptyItem();
        if (!sourceCurrent() || !contextCurrent(item)) {
            removeContext();
            _faulted = true;
            _operation.cancelInput();
            ROCK_LOG_ERROR(Weapon, "Akimbo identity/context changed session={} index={}; held item retained, firing disabled",
                _operation.session(), _index);
            return;
        }
        const auto previousBinding = _operation.binding();
        _operation.bind(input.hand, input.grip);
        if (previousBinding != _operation.binding()) stopAttackSound("grip-changed");
        _cycle.update(input.reference, input.deltaSeconds, input.inputAllowed);
        // Publish the replacement cache while both nodes are still pinned.
        // Native readers must never see a retired node after its lease drops.
        RE::NiPointer<RE::NiAVObject> nextMuzzle(input.muzzle);
        weaponData(item)->fireNode = nextMuzzle.get();
        if (auto* flash = weaponData(item)->muzzleFlash) {
            // 104B720 retains/releases the NiPointer at +18; 104BC90 uses
            // that exact node's world pose. A replaced model must not leave
            // the flash following its previously pinned muzzle.
            auto* anchor = reinterpret_cast<RE::NiPointer<RE::NiAVObject>*>(reinterpret_cast<std::byte*>(flash) + 0x18);
            RE::NiAVObject* priorAnchor{};
            if (!native_memory::tryReadValue(reinterpret_cast<RE::NiAVObject* const*>(anchor), priorAnchor) ||
                (priorAnchor != nextMuzzle.get() && !native_memory::pointerRangeLooksWritable(anchor, sizeof(*anchor)))) {
                removeContext(); _faulted = true;
                ROCK_LOG_ERROR(Weapon, "Loose weapon muzzle-effect anchor unavailable session={}", _operation.session());
                return;
            }
            if (priorAnchor != nextMuzzle.get()) *anchor = nextMuzzle;
        }
        _muzzle = std::move(nextMuzzle);
        observeAmmo();
        auto* liveData = weaponData(item);
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (liveData && liveData->muzzleFlash && player && input.inputAllowed && std::isfinite(input.deltaSeconds) && input.deltaSeconds > 0) {
            using UpdateFlash = void (*)(RE::MuzzleFlash*, float, RE::Actor*);
            reinterpret_cast<UpdateFlash>(REL::Offset(0x104BB50).address())(liveData->muzzleFlash, input.deltaSeconds, player);
        }
        if (!input.inputAllowed || !nativeWeaponAbsent()) {
            _operation.cancelInput();
            stopAttackSound("input-blocked");
            carried_weapon_projectile::clearEffects();
            _kick = {};
            return;
        }
        carried_weapon_projectile::publishEffects(_reference.get(), _reference->Get3D(), _muzzle.get(), item);
        bool resumedReload{};
        if (_operation.reloading() && !_cycle.reloading()) {
            if (const float authored = _cycle.reloadSeconds(_loaded == 0, _reloadSpeed); authored > 0) _reloadSeconds = authored;
            if (!_cycle.reload(_reloadSeconds, _loaded == 0, _reloadSeconds - _operation.reloadRemaining())) {
                stopAttackSound("reload-presentation-pending");
                return;
            }
            resumedReload = true;
            ++_reloadSequence;
        }
        _operation.advance(!resumedReload ? input.deltaSeconds : 0.0f);
        if (input.triggerHeld && (!_cycle.ready() || !_muzzle || input.grip != akimbo::Grip::Firing)) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Loose firearm trigger blocked ref={:08X} firingGrip={} muzzle={} animationReady={} animationFailed={}",
                _reference->formID, input.grip == akimbo::Grip::Firing, _muzzle != nullptr, _cycle.ready(), _cycle.failed());
        }
        if (input.reloadPressed && input.grip == akimbo::Grip::Firing && !_operation.reloading()) {
            const float authored = _cycle.reloadSeconds(_loaded == 0, _reloadSpeed);
            if (authored > 0 && _cycle.reload(authored, _loaded == 0) && _operation.beginReload(authored)) {
                ++_reloadSequence;
                _reloadSeconds = authored;
                stopAttackSound("reload");
            } else {
                ROCK_LOG_SAMPLE_WARN(Animation, 1000, "Loose weapon reload unavailable: exact clip pending or unsupported ref={:08X}", _reference->formID);
            }
        }
        if (_operation.reloadDue()) reload();
        const auto ticket = _operation.requestFire(_muzzle && _cycle.ready() && input.inputAllowed, input.triggerHeld,
            _automatic, _ammoKnown, _loaded);
        if (_operation.current(ticket)) {
            fire();
            _operation.completeFire(ticket, _secondsPerShot);
        }
        observeAmmo(); // Publish operation timing together with the resulting native count.
        if (!_operation.canSustainAttack(_muzzle && _cycle.ready(), input.triggerHeld, _ammoKnown, _loaded)) {
            stopAttackSound(!input.triggerHeld ? "trigger-released" : !_ammoKnown || !_loaded ? "ammunition-unavailable" : "firing-inactive");
        }
    }

    void CarriedWeaponRuntime::present() noexcept
    {
        if (_reference && !_faulted && sourceCurrent() && nativeWeaponAbsent()) _cycle.present();
    }

    bool CarriedWeaponRuntime::copyReloadPose(loose_reload_experiment::Snapshot& out) noexcept
    {
        out = {};
        if (saving.load(std::memory_order_acquire) || !isInteractionThread() || !_reference || _faulted ||
            !_operation.reloading() ||
            !nativeWeaponAbsent() || !sourceCurrent() || !_cycle.copyReloadPose(out)) return false;
        out.session = _operation.session();
        out.binding = _operation.binding();
        out.reload = _reloadSequence;
        out.reference = _reference->formID;
        out.weapon = _weapon.object->formID;
        out.firingHand = _operation.hand() == akimbo::Hand::Left ? 1u : 0u;
        out.active = 1;
        return true;
    }

    RE::NiPoint3 CarriedWeaponRuntime::advanceRecoil(float deltaSeconds) noexcept
    {
        auto item = emptyItem();
        if (!ready() || !_reference || !_muzzle || _faulted || !nativeWeaponAbsent() ||
            !sourceCurrent() || !contextCurrent(item) || !std::isfinite(deltaSeconds) || deltaSeconds <= 0) return {};
        auto* data = weaponData(item);
        if (data->aimModel) {
            // E28120 normally advances this only through the equipment array.
            using UpdateAim = void (*)(RE::AimModel*, float);
            reinterpret_cast<UpdateAim>(REL::Offset(0x834DF0).address())(data->aimModel, deltaSeconds);
        }
        const float distance = _kick.advance(deltaSeconds);
        auto* root = _reference->Get3D();
        auto* node = _muzzle.get();
        unsigned depth{};
        for (; node && node != root && depth < 64; ++depth) node = node->parent;
        if (!root || node != root) return {};
        const auto direction = native_scope_shot_policy::nodeAxisRay({}, _muzzle->world.rotate, 1);
        if (!direction.valid) return {};
        return {-distance * direction.direction.x, -distance * direction.direction.y, -distance * direction.direction.z};
    }
}
