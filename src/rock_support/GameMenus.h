#pragma once

#include "rock_support/Logger.h"

#include <array>
#include <string_view>

#include <RE/Fallout.h>

namespace rock::fo4vr
{
    // ROCK only needs three menu facts. Keeping a fixed state table avoids the
    // Framework handler's dynamic map and makes repeated open/close events idempotent.
    class GameMenusHandler final : public RE::BSTEventSink<RE::MenuOpenCloseEvent>
    {
    public:
        ~GameMenusHandler() override
        {
            if (_registered) {
                if (auto* ui = RE::UI::GetSingleton()) {
                    ui->UnregisterSink(this);
                }
            }
        }

        GameMenusHandler(const GameMenusHandler&) = delete;
        GameMenusHandler& operator=(const GameMenusHandler&) = delete;
        GameMenusHandler(GameMenusHandler&&) = delete;
        GameMenusHandler& operator=(GameMenusHandler&&) = delete;

        GameMenusHandler() = default;

        void init()
        {
            if (_registered) {
                return;
            }
            auto* ui = RE::UI::GetSingleton();
            if (!ui) {
                logger::error("Failed to initialize ROCK menu state: UI singleton is unavailable");
                return;
            }

            _stoppingMenuOpen.fill(false);
            _scopeMenuOpen = false;
            _loadingMenuOpen = false;
            ui->RegisterSink(this);
            _registered = true;
        }

        [[nodiscard]] bool isInScopeMenu() const noexcept
        {
            return _scopeMenuOpen;
        }

        [[nodiscard]] bool isLoadingMenuOpen() const noexcept
        {
            return _loadingMenuOpen;
        }

        [[nodiscard]] bool isGameStopped() const noexcept
        {
            for (const bool open : _stoppingMenuOpen) {
                if (open) {
                    return true;
                }
            }
            return false;
        }

    private:
        RE::BSEventNotifyControl ProcessEvent(
            const RE::MenuOpenCloseEvent& event,
            RE::BSTEventSource<RE::MenuOpenCloseEvent>*) override
        {
            const char* rawName = event.menuName.c_str();
            if (!rawName) {
                logger::warn("ROCK menu event contained a null menu name");
                return RE::BSEventNotifyControl::kContinue;
            }

            const std::string_view menuName{ rawName };
            if (menuName == "ScopeMenu") {
                _scopeMenuOpen = event.opening;
            }
            if (menuName == "LoadingMenu") {
                _loadingMenuOpen = event.opening;
            }
            for (std::size_t index = 0; index < kGameStoppingMenus.size(); ++index) {
                if (menuName == kGameStoppingMenus[index]) {
                    _stoppingMenuOpen[index] = event.opening;
                    break;
                }
            }

            logger::debug("Game menu '{}' {}", menuName, event.opening ? "opened" : "closed");
            return RE::BSEventNotifyControl::kContinue;
        }

        inline static constexpr std::array kGameStoppingMenus{
            std::string_view{ "BarterMenu" },
            std::string_view{ "Book Menu" },
            std::string_view{ "Console" },
            std::string_view{ "Native UI Menu" },
            std::string_view{ "ContainerMenu" },
            std::string_view{ "Dialogue Menu" },
            std::string_view{ "Crafting Menu" },
            std::string_view{ "Credits Menu" },
            std::string_view{ "Cursor Menu" },
            std::string_view{ "Debug Text Menu" },
            std::string_view{ "FavoritesMenu" },
            std::string_view{ "GiftMenu" },
            std::string_view{ "InventoryMenu" },
            std::string_view{ "Journal Menu" },
            std::string_view{ "Kinect Menu" },
            std::string_view{ "LoadingMenu" },
            std::string_view{ "Lockpicking Menu" },
            std::string_view{ "MagicMenu" },
            std::string_view{ "PipboyMenu" },
            std::string_view{ "LevelUpMenu" },
            std::string_view{ "MainMenu" },
            std::string_view{ "PauseMenu" },
            std::string_view{ "MapMarkerText3D" },
            std::string_view{ "MapMenu" },
            std::string_view{ "MessageBoxMenu" },
            std::string_view{ "Mist Menu" },
            std::string_view{ "Quantity Menu" },
            std::string_view{ "RaceSex Menu" },
            std::string_view{ "Sleep/Wait Menu" },
            std::string_view{ "StatsMenuSkillRing" },
            std::string_view{ "StatsMenuPerks" },
            std::string_view{ "Training Menu" },
            std::string_view{ "Tutorial Menu" },
            std::string_view{ "TweenMenu" },
        };

        std::array<bool, kGameStoppingMenus.size()> _stoppingMenuOpen{};
        bool _scopeMenuOpen{ false };
        bool _loadingMenuOpen{ false };
        bool _registered{ false };
    };
}
