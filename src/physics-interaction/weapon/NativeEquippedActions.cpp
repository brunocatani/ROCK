#include "physics-interaction/weapon/NativeEquippedActions.h"
#include "physics-interaction/weapon/PhysicalWeaponShotPolicy.h"
#include "physics-interaction/weapon/WeaponSceneTraversal.h"
#include "physics-interaction/weapon/WeaponGripTransfer.h"
#include "physics-interaction/weapon/telemetry/NativeScopeShotDiagnostics.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include <array>
#include <atomic>
#include <cstring>

namespace rock::native_equipped_actions
{
    namespace
    {
        using SetAttack = void (*)(RE::Actor*, std::uint32_t, std::uint32_t);
        using FireEvent = bool (*)(void*, RE::Actor*, const RE::BSFixedString*);
        using Reload = bool (*)(RE::Actor*, const RE::BGSObjectInstance*, std::uint32_t);
        using SetCount = void (*)(RE::AIProcess*, std::uint32_t, std::uint32_t);
        using ItemCount = bool (*)(RE::TESObjectREFR*, std::uint32_t*, RE::TESForm*, bool);
        using Rate = float (*)(RE::TESObjectWEAP*, RE::TBO_InstanceData*);
        using Haptic = void (*)(std::uint32_t,float,float);
        using HapticPattern = void (*)(std::uint32_t,float,float,std::uint32_t,std::uint32_t);
        std::atomic<bool> saving{};
        bool installed{};
        std::uint64_t nextSession{1}; // interaction thread only
        struct Shot {
            native_equipped_weapon::Identity identity{};
            RE::TESForm* form{};
            RE::NiTransform muzzle{};
            std::uint32_t launches{};
            bool active{}, originApplied{};
            akimbo::Hand hand{akimbo::Hand::None};
        };
        thread_local Shot shot;

        std::uint32_t shotController(std::uint32_t native) noexcept
        {
            // 1BAA0E0 forwards the queue index through 1BA70C0 to OpenVR:
            // index 0 -> role 1 (left), index 1 -> role 2 (right).
            return !shot.active || shot.hand == akimbo::Hand::None ? native : shot.hand == akimbo::Hand::Left ? 0u : 1u;
        }
        void fireHaptic(std::uint32_t controller,float intensity,float duration)
        {
            reinterpret_cast<Haptic>(REL::Offset(0x1BA9BF0).address())(shotController(controller),intensity,duration);
        }
        void fireHapticPattern(std::uint32_t controller,float intensity,float duration,std::uint32_t pattern,std::uint32_t count)
        {
            reinterpret_cast<HapticPattern>(REL::Offset(0x1BA9D20).address())(shotController(controller),intensity,duration,pattern,count);
        }

        template<std::size_t N>
        bool matches(std::uintptr_t rva, const std::array<std::uint8_t,N>& bytes)
        {
            std::array<std::uint8_t,N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(rva).address()),
                actual.data(), actual.size()) && actual == bytes;
        }

        void reloadCount(RE::AIProcess* process, std::uint32_t index, std::uint32_t requested)
        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            native_equipped_weapon::Snapshot target, peer;
            if (player && process == player->currentProcess && index < 2 &&
                native_equipped_weapon::read(index, target) && native_equipped_weapon::read(index ^ 1u, peer)) {
                const auto* data = static_cast<RE::EquippedWeaponData*>(target.item.data.get());
                const auto* other = static_cast<RE::EquippedWeaponData*>(peer.item.data.get());
                if (data->ammo && data->ammo == other->ammo) {
                    std::uint32_t total{};
                    requested = reinterpret_cast<ItemCount>(REL::Offset(0x3E8380).address())(player, &total, data->ammo, false) ?
                        akimbo::clampReload(requested, total, other->ammoCount) : data->ammoCount;
                }
            }
            reinterpret_cast<SetCount>(REL::Offset(0xEC4B90).address())(process, index, requested);
        }
    }

    bool install() noexcept try
    {
        if (installed) return true;
        if (!native_equipped_weapon::ready() ||
            !matches(0xFF2A40, std::array<std::uint8_t,10>{0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10}) ||
            !matches(0xE51910, std::array<std::uint8_t,10>{0x89,0x54,0x24,0x10,0x53,0x55,0x56,0x57,0x41,0x56}) ||
            !matches(0x333360, std::array<std::uint8_t,9>{0x0F,0x57,0xC0,0x48,0x85,0xD2,0x74,0x08,0x8B}) ||
            !matches(0xE4E3B0, std::array<std::uint8_t,10>{0x44,0x89,0x44,0x24,0x18,0x55,0x41,0x56,0x41,0x57}) ||
            !matches(0x3E8380, std::array<std::uint8_t,10>{0x48,0x89,0x5C,0x24,0x10,0x44,0x88,0x4C,0x24,0x20})) return false;
        constexpr std::array<std::uint8_t,14> siteBytes{0x48,0x8B,0xCF,0x0F,0x42,0xF3,0x44,0x8B,0xC6,0xE8,0x82,0x64,0x07,0x00};
        if (!matches(0xE4E700, siteBytes) ||
            !matches(0x333F38,std::array<std::uint8_t,11>{0x8B,0x88,0xCC,0x08,0x00,0x00,0xE8,0xDD,0x5D,0x87,0x01}) ||
            !matches(0x3340A2,std::array<std::uint8_t,11>{0x8B,0x88,0xCC,0x08,0x00,0x00,0xE8,0x43,0x5B,0x87,0x01}) ||
            F4SE::GetTrampoline().free_size() < 48) return false;
        F4SE::GetTrampoline().write_call<5>(REL::Offset(0xE4E709).address(), &reloadCount);
        F4SE::GetTrampoline().write_call<5>(REL::Offset(0x333F3E).address(), &fireHapticPattern);
        F4SE::GetTrampoline().write_call<5>(REL::Offset(0x3340A8).address(), &fireHaptic);
        installed = true;
        return true;
    }
    catch (...) { return false; }

    bool ready() noexcept { return installed && native_scope_shot_diagnostics::hooksReady() && !saving.load(std::memory_order_acquire); }
    bool supports(RE::TESObjectWEAP* weapon, RE::TBO_InstanceData* instance) noexcept
    {
        if (!ready() || !weapon || weapon->weaponData.type != RE::WEAPON_TYPE::kGun) return false;
        const auto* effective = instance ? static_cast<RE::TESObjectWEAP::InstanceData*>(instance) : &weapon->weaponData;
        if (!effective->ammo || !effective->rangedData || effective->flags.any(
                RE::WEAPON_FLAGS::kChargingAttack,RE::WEAPON_FLAGS::kChargingReload,RE::WEAPON_FLAGS::kHoldInputToPower)) return false;
        const float rate = reinterpret_cast<Rate>(REL::Offset(0x333360).address())(weapon,instance);
        const float reload = effective->rangedData->reloadSeconds/effective->reloadSpeed;
        return std::isfinite(rate) && rate > 0 && std::isfinite(1.0f/rate) &&
            std::isfinite(reload) && reload > 0 && effective->reloadSpeed > 0;
    }
    void beforeSave() noexcept { saving.store(true, std::memory_order_release); }
    void afterSave() noexcept { saving.store(false, std::memory_order_release); }

    bool applyShotOrigin(void* launchData) noexcept
    {
        if (!shot.active) return true;
        std::uintptr_t form{}, instance{};
        std::uint32_t index{};
        if (!launchData || !native_memory::tryReadField(launchData,0x30,form) ||
            !native_memory::tryReadField(launchData,0x38,instance) || !native_memory::tryReadField(launchData,0x48,index) ||
            form != reinterpret_cast<std::uintptr_t>(shot.form) || instance != shot.identity.instance || index != shot.identity.index) return false;
        const auto aim = physical_weapon_shot_policy::muzzleAim(shot.muzzle);
        if (!aim.ray.valid) return false;
        shot.originApplied = native_memory::tryWriteValue(static_cast<RE::NiPoint3*>(launchData), shot.muzzle.translate) &&
            native_memory::guardedCopyToMemory(static_cast<char*>(launchData)+0x4C, &aim.yaw, sizeof(aim.yaw)) &&
            native_memory::guardedCopyToMemory(static_cast<char*>(launchData)+0x50, &aim.pitch, sizeof(aim.pitch));
        return shot.originApplied;
    }

    void observeShotLaunch(const void* launchData, std::uint32_t handle) noexcept
    {
        if (!shot.active || !handle || !launchData) return;
        std::uint32_t index{};
        std::uintptr_t form{}, instance{};
        if (native_memory::tryReadField(launchData,0x48,index) && index == shot.identity.index &&
            native_memory::tryReadField(launchData,0x30,form) && form == reinterpret_cast<std::uintptr_t>(shot.form) &&
            native_memory::tryReadField(launchData,0x38,instance) && instance == shot.identity.instance) ++shot.launches;
    }

    bool Session::prepare(const native_equipped_weapon::Snapshot& item, std::uint64_t content, float seconds)
    {
        if (!ready() || !item.equipped || !item.attached) return false;
        if (_identity != item.identity) {
            clear(true);
            auto* weapon = static_cast<RE::TESObjectWEAP*>(item.item.item.object);
            auto* instance = item.item.item.instanceData.get();
            const auto* effective = instance ? static_cast<RE::TESObjectWEAP::InstanceData*>(instance) : &weapon->weaponData;
            const auto* ranged = effective->rangedData;
            if (!supports(weapon,instance)) return false;
            const auto rate = reinterpret_cast<Rate>(REL::Offset(0x333360).address())(weapon,instance);
            if (!std::isfinite(rate) || rate <= 0 || !std::isfinite(ranged->reloadSeconds) || ranged->reloadSeconds <= 0 ||
                !std::isfinite(effective->reloadSpeed) || effective->reloadSpeed <= 0) return false;
            _identity = item.identity;
            _shotSeconds = 1.0f/rate;
            _reloadSeconds = ranged->reloadSeconds/effective->reloadSpeed;
            _automatic = effective->flags.any(RE::WEAPON_FLAGS::kAutomatic);
            _operation.begin(nextSession++);
        }
        auto* weapon = static_cast<RE::TESObjectWEAP*>(item.item.item.object);
        _cycle.updateEquipped(weapon,item.item.item.instanceData.get(),item.model.get(),content,seconds);
        RE::NiAVObject* muzzle{};
        unsigned candidates{};
        const auto traversal = weapon_scene::visitScene(item.model.get(), [&](RE::NiAVObject* node) {
            if (node->name.c_str() && _stricmp(node->name.c_str(),"ProjectileNode")==0) {muzzle=node; ++candidates;}
            return true;
        });
        if (traversal.truncated || candidates != 1 || !muzzle) return false;
        auto* data = static_cast<RE::EquippedWeaponData*>(item.item.data.get());
        if (_muzzle.get() != muzzle) {
            _previousMuzzle.reset(data->fireNode);
            _muzzle.reset(muzzle);
        }
        data->fireNode = muzzle;
        return _cycle.ready();
    }

    void Session::stopAttack()
    {
        if (_attackActive && native_equipped_weapon::matches(_identity))
            reinterpret_cast<SetAttack>(REL::Offset(0xE51910).address())(RE::PlayerCharacter::GetSingleton(),_identity.index,0);
        _attackActive = false;
    }

    void Session::bind(akimbo::Hand hand, akimbo::Grip grip)
    {
        const auto before = _operation.binding();
        _operation.bind(hand,grip);
        if (_operation.binding() != before) stopAttack();
    }
    void Session::suspend() { stopAttack(); _operation.cancelInput(); }

    void Session::clear(bool nativeWorldAvailable)
    {
        if (nativeWorldAvailable) {
            stopAttack();
            native_equipped_weapon::Snapshot current;
            if (native_equipped_weapon::read(_identity.index,current) && current.identity == _identity) {
                auto* data = static_cast<RE::EquippedWeaponData*>(current.item.data.get());
                if (data->fireNode == _muzzle.get()) data->fireNode = _previousMuzzle.get();
            }
        }
        _attackActive = false;
        _cycle.clear();
        _muzzle.reset(); _previousMuzzle.reset();
        _identity = {}; _operation = {};
    }

    bool Session::fire(const native_equipped_weapon::Snapshot& item)
    {
        if (!ready() || shot.active || item.identity != _identity || !_muzzle ||
            !weapon_grip_transfer::validFrame(_muzzle->world) || !native_equipped_weapon::matches(_identity)) return false;
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* data = static_cast<RE::EquippedWeaponData*>(item.item.data.get());
        const auto before = data->ammoCount;
        if (!before) return false;
        const RE::BSFixedString payload(_identity.index ? "1" : "0");
        shot = {_identity,item.item.item.object,_muzzle->world,0,true,false,_operation.hand()};
        struct ClearShot { ~ClearShot() { shot = {}; } } reset;
        if (!_attackActive) reinterpret_cast<SetAttack>(REL::Offset(0xE51910).address())(player,_identity.index,15);
        _attackActive = true;
        // FF2A40's incoming RCX is unused. Its indexed event resolves the
        // real equipment, performs the normal state transition, then fires.
        reinterpret_cast<FireEvent>(REL::Offset(0xFF2A40).address())(nullptr,player,&payload);
        const bool fired = shot.originApplied && shot.launches && data->ammoCount < before;
        if (fired) _cycle.fire(_shotSeconds);
        else stopAttack();
        ROCK_LOG_DEBUG(Weapon,"Native equipped shot index={} form={:08X} data={:#x} ammo={}->{} origin={} projectiles={} soundMapping={}",
            _identity.index,_identity.form,_identity.data,before,data->ammoCount,shot.originApplied,shot.launches,data->attackSoundData != nullptr);
        return fired;
    }

    Events Session::update(const native_equipped_weapon::Snapshot& item, const Input& input)
    {
        Events events;
        if (item.identity != _identity || !ready()) { suspend(); return events; }
        auto* data = static_cast<RE::EquippedWeaponData*>(item.item.data.get());
        events.before = events.after = data->ammoCount;
        bind(input.hand,input.grip);
        if (!input.allowed || !_cycle.ready()) { suspend(); return events; }
        _operation.advance(input.deltaSeconds);
        if (_operation.reloadDue()) {
            events.reloadFinished = reinterpret_cast<Reload>(REL::Offset(0xE4E3B0).address())(
                RE::PlayerCharacter::GetSingleton(),&item.item.item,_identity.index);
            _operation.completeReload();
        }
        if (input.reloadPressed && _operation.beginReload(_reloadSeconds)) {
            stopAttack();
            events.reloadStarted = true;
        }
        const auto ticket = _operation.requestFire(input.allowed,input.triggerHeld,_automatic,true,data->ammoCount);
        if (_operation.current(ticket)) {
            events.fired = fire(item);
            _operation.completeFire(ticket,_shotSeconds);
        }
        if (!input.triggerHeld || !data->ammoCount || _operation.reloading() || input.grip != akimbo::Grip::Firing ||
            (!_automatic && _operation.cooldown() == 0)) stopAttack();
        events.after = data->ammoCount;
        return events;
    }
}
