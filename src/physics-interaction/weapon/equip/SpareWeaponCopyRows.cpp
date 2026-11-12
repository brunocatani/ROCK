// Thread model: TESContainerChangedEvent may arrive on any game thread and
// only raises an atomic flag. The frame thread consumes it and writes the
// player's inventory. A save keeps the split copies' unique ID tags, so a
// loaded save needs only the one check the initial flag requests.
#include "physics-interaction/weapon/equip/SpareWeaponCopyRows.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/weapon/equip/WeaponEquipTransfer.h"

#include "RE/Bethesda/BSTEvent.h"
#include "RE/Bethesda/Events.h"
#include "RE/Bethesda/TESForms.h"

#include <atomic>

namespace rock::spare_weapon_copy_rows
{
    namespace
    {
        constexpr std::uint32_t kPlayerFormId = 0x14;

        // Raised at load: the first world frame checks the inventory once.
        std::atomic<bool> s_pending{ true };
        std::atomic<bool> s_registered{ false };
        std::atomic<bool> s_missingSourceLogged{ false };

        class ContainerChangedSink final : public RE::BSTEventSink<RE::TESContainerChangedEvent>
        {
        public:
            RE::BSEventNotifyControl ProcessEvent(const RE::TESContainerChangedEvent& event,
                RE::BSTEventSource<RE::TESContainerChangedEvent>*) override
            {
                if (event.newContainerFormID != kPlayerFormId || event.baseObjectFormID == 0)
                    return RE::BSEventNotifyControl::kContinue;
                const auto* form = RE::TESForm::GetFormByID(event.baseObjectFormID);
                if (form && form->IsWeapon())
                    s_pending.store(true, std::memory_order_release);
                return RE::BSEventNotifyControl::kContinue;
            }
        };

        ContainerChangedSink s_sink;
    }

    void ensureRegistered() noexcept
    {
        bool expected = false;
        if (!s_registered.compare_exchange_strong(expected, true, std::memory_order_acq_rel,
                std::memory_order_acquire))
            return;
        auto* source = RE::TESContainerChangedEvent::GetEventSource();
        if (!source) {
            s_registered.store(false, std::memory_order_release);
            if (!s_missingSourceLogged.exchange(true, std::memory_order_acq_rel))
                ROCK_LOG_WARN(Weapon, "Container-changed event source unavailable; spare weapon rows will retry");
            return;
        }
        source->RegisterSink(&s_sink);
    }

    void service(const bool menuOpen, const bool playerReady) noexcept
    {
        if (menuOpen || !playerReady || !s_pending.exchange(false, std::memory_order_acq_rel))
            return;
        weapon_equip_transfer::separateSpareCopiesFromWornStacks();
    }
}
