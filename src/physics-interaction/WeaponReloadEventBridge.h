#pragma once

#include "WeaponReloadStageObserver.h"

#include "RE/Bethesda/BSTEvent.h"
#include "RE/Bethesda/Events.h"

#include <atomic>
#include <cstdint>

namespace frik::rock
{
    /*
     * ROCK observes vanilla reload and ammo events instead of replacing the
     * game's reload input flow. This preserves mod compatibility while giving
     * the physical reload coordinator verified stage signals. Ammo mutation,
     * node clone/hide, and completion blocking remain outside this bridge so
     * those higher-risk native calls can be verified separately before use.
     */
    class WeaponReloadEventBridge final :
        public RE::BSTEventSink<RE::PlayerWeaponReloadEvent>,
        public RE::BSTEventSink<RE::PlayerAmmoCountEvent>
    {
    public:
        void install();
        void uninstall();
        void reset();

        [[nodiscard]] bool isInstalled() const { return _installed.load(std::memory_order_acquire); }

        [[nodiscard]] WeaponReloadNativeSnapshot consumeSnapshot(bool weaponEquipped);

        RE::BSEventNotifyControl ProcessEvent(const RE::PlayerWeaponReloadEvent& event, RE::BSTEventSource<RE::PlayerWeaponReloadEvent>* source) override;
        RE::BSEventNotifyControl ProcessEvent(const RE::PlayerAmmoCountEvent& event, RE::BSTEventSource<RE::PlayerAmmoCountEvent>* source) override;

    private:
        std::atomic<bool> _installed{ false };

        std::atomic<std::uint32_t> _sequence{ 0 };
        std::atomic<std::uint32_t> _reloadSequence{ 0 };
        std::atomic<std::uint32_t> _ammoSequence{ 0 };

        std::atomic<std::uint32_t> _consumedReloadSequence{ 0 };
        std::atomic<std::uint32_t> _consumedAmmoSequence{ 0 };

        std::atomic<bool> _lastReloadValue{ false };
        std::atomic<std::uint32_t> _lastClipAmmo{ 0 };
        std::atomic<std::uint32_t> _lastReserveAmmo{ 0 };
    };
}
