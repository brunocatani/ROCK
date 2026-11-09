#include "setup-wizard/AutoOpenGate.h"

#include <array>
#include <cstring>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/input/InputRemapRuntime.h"

#include "RE/Bethesda/BSInputEnableManager.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/UI.h"

namespace rock::setup_wizard::auto_open_gate
{
    namespace
    {
        /*
         * BSInputEnableManager's "is this user event enabled" check in Fallout4VR.exe 1.2.72.
         * Raw disassembly: Papyrus Game.IsMovementControlsEnabled (0x1414467C0) calls it with
         * the manager from 0x145932260 (address library ID 781703) and flag 1 (movement), and
         * the native registered as IsLookingControlsEnabled (0x141446780) with flag 2. It takes
         * the manager's lock at +0x110 and returns true when the force-enable flags at +0x120
         * hold any requested bit or the cached enabled flags at +0x118 hold all of them. The
         * prefix below runs through both reads; any other bytes disable the gate.
         */
        constexpr std::uintptr_t kFunc_IsUserEventEnabled = 0x1BACD80;
        constexpr std::array<std::uint8_t, 62> kExpectedPrefix{ 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10,
            0x57, 0x48, 0x83, 0xEC, 0x20, 0x48, 0x8D, 0x99, 0x10, 0x01, 0x00, 0x00, 0x8B, 0xFA, 0x48, 0x8B, 0xF1, 0x48, 0x8B,
            0xCB, 0x33, 0xD2, 0xE8, 0x6B, 0x80, 0x47, 0xFE, 0x85, 0xBE, 0x20, 0x01, 0x00, 0x00, 0x41, 0x0F, 0x95, 0xC0, 0x45,
            0x84, 0xC0, 0x75, 0x0E, 0x8B, 0x86, 0x18, 0x01, 0x00, 0x00, 0x23, 0xC7, 0x3B, 0xC7 };
        constexpr std::uint32_t kMovement = 1u << 0;
        constexpr std::uint32_t kLooking = 1u << 1;

        using IsUserEventEnabled = bool (*)(RE::BSInputEnableManager*, std::uint32_t);

        [[nodiscard]] IsUserEventEnabled resolveCheck() noexcept
        {
            static const IsUserEventEnabled check = []() noexcept -> IsUserEventEnabled {
                if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
                    ROCK_LOG_WARN(Config, "Setup wizard gate unavailable: unsupported runtime");
                    return nullptr;
                }
                const auto address = REL::Offset(kFunc_IsUserEventEnabled).address();
                if (std::memcmp(reinterpret_cast<const void*>(address), kExpectedPrefix.data(), kExpectedPrefix.size()) != 0) {
                    ROCK_LOG_WARN(Config, "Setup wizard gate unavailable: unexpected bytes at the input-enable check; the "
                                          "wizard does not open by itself");
                    return nullptr;
                }
                return reinterpret_cast<IsUserEventEnabled>(address);
            }();
            return check;
        }

        [[nodiscard]] bool menuOpen(RE::UI& ui, const char* name)
        {
            return ui.GetMenuOpen(RE::BSFixedString{ name });
        }
    }

    Reading read() noexcept
    {
        Reading reading{};
        try {
            const auto* player = RE::PlayerCharacter::GetSingleton();
            reading.inWorld = player && player->parentCell;
            reading.menuOpen = input_remap_runtime::isMenuInputActive();
            if (auto* ui = RE::UI::GetSingleton()) {
                reading.fading = menuOpen(*ui, "FaderMenu") || menuOpen(*ui, "LoadingMenu");
                reading.mirrorOpen = menuOpen(*ui, "LooksMenu");
            }
            auto* manager = RE::BSInputEnableManager::GetSingleton();
            if (const auto check = resolveCheck(); check && manager) {
                reading.controlsKnown = true;
                reading.movement = check(manager, kMovement);
                reading.looking = check(manager, kLooking);
            }
        } catch (...) {
            return {};
        }
        return reading;
    }

    std::string describe(const Reading& reading)
    {
        std::string text;
        const auto add = [&](const char* name, bool value) {
            if (!text.empty()) text += ", ";
            text += name;
            text += value ? " yes" : " no";
        };
        add("in world", reading.inWorld);
        add("menu", reading.menuOpen);
        add("fade or loading", reading.fading);
        add("mirror", reading.mirrorOpen);
        add("controls known", reading.controlsKnown);
        add("movement", reading.movement);
        add("looking", reading.looking);
        return text;
    }
}
