#include "WeaponReloadEventBridge.h"

#include "HavokOffsets.h"
#include "PhysicsLog.h"
#include "RockConfig.h"

#include <algorithm>

namespace frik::rock
{
    namespace
    {
        using ReloadEventSource = RE::BSTGlobalEvent::EventSource<RE::PlayerWeaponReloadEvent>;
        using AmmoEventSource = RE::BSTGlobalEvent::EventSource<RE::PlayerAmmoCountEvent>;

        ReloadEventSource* getReloadEventSource()
        {
            static REL::Relocation<ReloadEventSource**> singleton{ REL::Offset(offsets::kData_PlayerWeaponReloadEventSource) };
            return *singleton;
        }

        AmmoEventSource* getAmmoEventSource()
        {
            static REL::Relocation<AmmoEventSource**> singleton{ REL::Offset(offsets::kData_PlayerAmmoCountEventSource) };
            return *singleton;
        }
    }

    void WeaponReloadEventBridge::install()
    {
        if (_installed.load(std::memory_order_acquire)) {
            return;
        }

        auto* reloadSource = getReloadEventSource();
        auto* ammoSource = getAmmoEventSource();
        if (!reloadSource || !ammoSource) {
            ROCK_LOG_ERROR(Weapon, "WeaponReloadEventBridge install failed: reloadSource={:p} ammoSource={:p}", static_cast<void*>(reloadSource), static_cast<void*>(ammoSource));
            return;
        }

        reloadSource->RegisterSink(static_cast<RE::BSTEventSink<RE::PlayerWeaponReloadEvent>*>(this));
        ammoSource->RegisterSink(static_cast<RE::BSTEventSink<RE::PlayerAmmoCountEvent>*>(this));
        _installed.store(true, std::memory_order_release);

        ROCK_LOG_INFO(Weapon, "WeaponReloadEventBridge installed (reloadSource={:p}, ammoSource={:p})", static_cast<void*>(reloadSource), static_cast<void*>(ammoSource));
    }

    void WeaponReloadEventBridge::uninstall()
    {
        bool expected = true;
        if (!_installed.compare_exchange_strong(expected, false, std::memory_order_acq_rel)) {
            return;
        }

        if (auto* source = getReloadEventSource()) {
            source->UnregisterSink(static_cast<RE::BSTEventSink<RE::PlayerWeaponReloadEvent>*>(this));
        }
        if (auto* source = getAmmoEventSource()) {
            source->UnregisterSink(static_cast<RE::BSTEventSink<RE::PlayerAmmoCountEvent>*>(this));
        }

        ROCK_LOG_INFO(Weapon, "WeaponReloadEventBridge uninstalled");
    }

    void WeaponReloadEventBridge::reset()
    {
        _sequence.store(0, std::memory_order_release);
        _reloadSequence.store(0, std::memory_order_release);
        _ammoSequence.store(0, std::memory_order_release);
        _consumedReloadSequence.store(0, std::memory_order_release);
        _consumedAmmoSequence.store(0, std::memory_order_release);
        _lastReloadValue.store(false, std::memory_order_release);
        _lastClipAmmo.store(0, std::memory_order_release);
        _lastReserveAmmo.store(0, std::memory_order_release);
    }

    WeaponReloadNativeSnapshot WeaponReloadEventBridge::consumeSnapshot(bool weaponEquipped)
    {
        WeaponReloadNativeSnapshot snapshot{};
        snapshot.weaponEquipped = weaponEquipped;

        const auto reloadSequence = _reloadSequence.load(std::memory_order_acquire);
        const auto ammoSequence = _ammoSequence.load(std::memory_order_acquire);

        if (reloadSequence != 0 && reloadSequence != _consumedReloadSequence.load(std::memory_order_acquire)) {
            snapshot.reloadEventReceived = true;
            snapshot.reloadEventValue = _lastReloadValue.load(std::memory_order_acquire);
            snapshot.reloadSequence = reloadSequence;
            _consumedReloadSequence.store(reloadSequence, std::memory_order_release);
        }

        if (ammoSequence != 0 && ammoSequence != _consumedAmmoSequence.load(std::memory_order_acquire)) {
            snapshot.ammoEventReceived = true;
            snapshot.clipAmmo = _lastClipAmmo.load(std::memory_order_acquire);
            snapshot.reserveAmmo = _lastReserveAmmo.load(std::memory_order_acquire);
            snapshot.ammoSequence = ammoSequence;
            _consumedAmmoSequence.store(ammoSequence, std::memory_order_release);
        }

        snapshot.sequence = std::max(_sequence.load(std::memory_order_acquire), std::max(reloadSequence, ammoSequence));
        return snapshot;
    }

    RE::BSEventNotifyControl WeaponReloadEventBridge::ProcessEvent(
        const RE::PlayerWeaponReloadEvent& event, [[maybe_unused]] RE::BSTEventSource<RE::PlayerWeaponReloadEvent>* source)
    {
        if (!event.optionalValue.has_value()) {
            return RE::BSEventNotifyControl::kContinue;
        }

        const bool reloadActive = event.optionalValue.value();
        _lastReloadValue.store(reloadActive, std::memory_order_release);
        const auto sequence = _sequence.fetch_add(1, std::memory_order_acq_rel) + 1;
        _reloadSequence.store(sequence, std::memory_order_release);

        if (g_rockConfig.rockReloadDebugStageLogging) {
            ROCK_LOG_DEBUG(Weapon, "WeaponReloadEventBridge reload event active={} seq={}", reloadActive, sequence);
        }

        return RE::BSEventNotifyControl::kContinue;
    }

    RE::BSEventNotifyControl WeaponReloadEventBridge::ProcessEvent(
        const RE::PlayerAmmoCountEvent& event, [[maybe_unused]] RE::BSTEventSource<RE::PlayerAmmoCountEvent>* source)
    {
        if (!event.optionalValue.has_value()) {
            return RE::BSEventNotifyControl::kContinue;
        }

        const auto counts = event.optionalValue.value();
        _lastClipAmmo.store(counts.clipAmmo, std::memory_order_release);
        _lastReserveAmmo.store(counts.reserveAmmo, std::memory_order_release);
        const auto sequence = _sequence.fetch_add(1, std::memory_order_acq_rel) + 1;
        _ammoSequence.store(sequence, std::memory_order_release);

        if (g_rockConfig.rockReloadDebugStageLogging) {
            ROCK_LOG_DEBUG(Weapon, "WeaponReloadEventBridge ammo event clip={} reserve={} seq={}", counts.clipAmmo, counts.reserveAmmo, sequence);
        }

        return RE::BSEventNotifyControl::kContinue;
    }
}
