// The menu sink that asks PhysicsInteraction to rebuild weapon collision when a
// workbench menu closes: the player may have changed the weapon's parts there.
// One sink and its registration flags exist for the process.
#pragma once

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/core/PhysicsInteraction.h"

#include "RE/Bethesda/Events.h"
#include "RE/Bethesda/UI.h"

#include <array>
#include <atomic>
#include <string_view>

namespace rock
{
    namespace physics_interaction_internal
    {
        inline constexpr std::array<std::string_view, 5> kWeaponCollisionWorkbenchExitMenuNames{
            "ExamineMenu",
            "PowerArmorModMenu",
            "RobotModMenu",
            "Crafting Menu",
            "CraftingMenu",
        };

        inline std::atomic<bool> s_weaponCollisionWorkbenchExitMenuSinkRegistered{ false };
        inline std::atomic<bool> s_weaponCollisionWorkbenchExitMenuSinkMissingUILogged{ false };

        [[nodiscard]] inline bool isWeaponCollisionWorkbenchExitMenu(const RE::BSFixedString& menuName)
        {
            for (const auto name : kWeaponCollisionWorkbenchExitMenuNames) {
                if (menuName == name) {
                    return true;
                }
            }

            return false;
        }

        class WeaponCollisionWorkbenchExitMenuSink final : public RE::BSTEventSink<RE::MenuOpenCloseEvent>
        {
        public:
            RE::BSEventNotifyControl ProcessEvent(const RE::MenuOpenCloseEvent& event,
                RE::BSTEventSource<RE::MenuOpenCloseEvent>*) override
            {
                if (event.opening || !event.menuName.c_str() || !isWeaponCollisionWorkbenchExitMenu(event.menuName)) {
                    return RE::BSEventNotifyControl::kContinue;
                }

                auto* interaction = PhysicsInteraction::s_instance.load(std::memory_order_acquire);
                if (interaction && interaction->isInitialized()) {
                    interaction->requestWeaponCollisionRebuildAfterWorkbenchExit(event.menuName.c_str());
                }

                return RE::BSEventNotifyControl::kContinue;
            }
        };

        inline WeaponCollisionWorkbenchExitMenuSink s_weaponCollisionWorkbenchExitMenuSink;

        inline bool ensureWeaponCollisionWorkbenchExitMenuSinkRegistered()
        {
            bool expected = false;
            if (!s_weaponCollisionWorkbenchExitMenuSinkRegistered.compare_exchange_strong(expected, true,
                    std::memory_order_acq_rel, std::memory_order_acquire)) {
                return true;
            }

            auto* ui = RE::UI::GetSingleton();
            if (!ui) {
                s_weaponCollisionWorkbenchExitMenuSinkRegistered.store(false, std::memory_order_release);
                if (!s_weaponCollisionWorkbenchExitMenuSinkMissingUILogged.exchange(true, std::memory_order_acq_rel)) {
                    ROCK_LOG_WARN(Weapon,
                        "UI singleton unavailable; weapon collision workbench-exit menu sink will retry");
                }
                return false;
            }

            ui->RegisterSink<RE::MenuOpenCloseEvent>(&s_weaponCollisionWorkbenchExitMenuSink);
            s_weaponCollisionWorkbenchExitMenuSinkMissingUILogged.store(false, std::memory_order_release);
            ROCK_LOG_INFO(Weapon, "Registered weapon collision workbench-exit menu sink for {} menu names",
                kWeaponCollisionWorkbenchExitMenuNames.size());
            return true;
        }
    }
}
