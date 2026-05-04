#include "InputRemapRuntime.h"

#include "InputRemapPolicy.h"
#include "PhysicsLog.h"
#include "RockConfig.h"

#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/UI.h"

#include <REL/Relocation.h>
#include <vrcf/VRControllersManager.h>
#include <windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <intrin.h>
#include <optional>
#include <string_view>

namespace frik::rock::input_remap_runtime
{
    namespace
    {
        constexpr std::size_t kGetControllerStateVTableIndex = 34;
        constexpr std::size_t kGetControllerStateWithPoseVTableIndex = 35;
        // Ghidra verified FO4VR's ReadyWeapon handler target at 0x140FCE650 is a normal
        // function prologue and the live dispatch slot is 0x142D8A480. The previous
        // CommonLib write_branch hook treated that prologue as an existing branch/call
        // and produced a bogus callable original, which crashed on right-thumbstick
        // input. A vtable-slot hook keeps the original function pointer intact; an
        // entry trampoline would also work, but this handler is virtual and the slot is
        // the narrower patch surface.
        constexpr std::uintptr_t kReadyWeaponHandlerFunctionOffset = 0xFCE650;
        constexpr std::uintptr_t kReadyWeaponHandlerVTableSlotOffset = 0x2D8A480;

        using GetControllerState_t = bool (*)(vr::IVRSystem*, vr::TrackedDeviceIndex_t, vr::VRControllerState_t*, std::uint32_t);
        using GetControllerStateWithPose_t =
            bool (*)(vr::IVRSystem*, vr::ETrackingUniverseOrigin, vr::TrackedDeviceIndex_t, vr::VRControllerState_t*, std::uint32_t, vr::TrackedDevicePose_t*);
        using ReadyWeaponHandler_t = bool (*)(void*, void*);

        struct ControllerTracker
        {
            std::atomic<std::uint64_t> rawPressed{ 0 };
            std::atomic<std::uint64_t> rawTouched{ 0 };
            std::atomic<std::uint64_t> pressedEdges{ 0 };
            std::atomic<std::uint64_t> releasedEdges{ 0 };
            std::atomic<bool> valid{ false };
            std::atomic<std::uint64_t> gameFacingPressed{ 0 };
            std::atomic<bool> gameFacingValid{ false };
        };

        std::array<ControllerTracker, 2> s_controllers;
        std::atomic<bool> s_gameplayInputAllowed{ false };
        std::atomic<bool> s_weaponDrawn{ false };
        std::atomic<std::uint32_t> s_pendingWeaponToggleRequests{ 0 };
        std::atomic<bool> s_hooksInstalled{ false };
        std::atomic<bool> s_readyWeaponHookInstalled{ false };
        std::atomic<bool> s_menuInputGateRegistered{ false };
        std::atomic<bool> s_menuInputActive{ false };
        std::atomic<bool> s_missingVRSystemLogged{ false };
        std::atomic<bool> s_missingUILogged{ false };

        void** s_vrSystemVTable = nullptr;
        GetControllerState_t s_originalGetControllerState = nullptr;
        GetControllerStateWithPose_t s_originalGetControllerStateWithPose = nullptr;
        ReadyWeaponHandler_t s_originalReadyWeaponHandler = nullptr;

        /*
         * ROCK remaps the right Favorites/thumbstick input only while gameplay owns controller input.
         * FRIK's public menu gate intentionally exposes a small set of FRIK-relevant menus, so relying
         * on that cached frame flag lets ContainerMenu/BarterMenu continue through the OpenVR hook and
         * lose menu actions such as Take All/Store All. HIGGS solves this class of problem with an
         * event-driven game-stopping menu tracker; ROCK keeps the same shape here so the hook can make
         * a cheap, current, menu-aware decision without polling Scaleform on every controller sample.
         */
        constexpr std::array<std::string_view, 48> kGameStoppingMenuNames{
            "BarterMenu",
            "Book Menu",
            "Console",
            "Native UI Menu",
            "ContainerMenu",
            "Crafting Menu",
            "Credits Menu",
            "Cursor Menu",
            "CursorMenu",
            "Debug Text Menu",
            "Dialogue Menu",
            "DialogueMenu",
            "ExamineConfirmMenu",
            "ExamineMenu",
            "FavoritesMenu",
            "GiftMenu",
            "InventoryMenu",
            "Journal Menu",
            "Kinect Menu",
            "LevelUpMenu",
            "Loading Menu",
            "LoadingMenu",
            "Lockpicking Menu",
            "LockpickingMenu",
            "MagicMenu",
            "Main Menu",
            "MainMenu",
            "MapMarkerText3D",
            "MapMenu",
            "MessageBoxMenu",
            "Mist Menu",
            "PauseMenu",
            "PipboyHolotapeMenu",
            "PipboyMenu",
            "PowerArmorModMenu",
            "Quantity Menu",
            "RaceSex Menu",
            "SitWaitMenu",
            "Sleep/Wait Menu",
            "StatsMenuPerks",
            "StatsMenuSkillRing",
            "TerminalHolotapeMenu",
            "TerminalMenu",
            "TerminalMenuButtons",
            "Training Menu",
            "Tutorial Menu",
            "TweenMenu",
            "WorkshopMenu",
        };

        std::array<std::atomic<bool>, kGameStoppingMenuNames.size()> s_gameStoppingMenuOpen{};

        [[nodiscard]] std::optional<std::size_t> findGameStoppingMenuIndex(const RE::BSFixedString& menuName)
        {
            for (std::size_t i = 0; i < kGameStoppingMenuNames.size(); ++i) {
                if (menuName == kGameStoppingMenuNames[i]) {
                    return i;
                }
            }

            return std::nullopt;
        }

        void publishMenuInputActiveFromTrackedMenus()
        {
            for (const auto& menuOpen : s_gameStoppingMenuOpen) {
                if (menuOpen.load(std::memory_order_acquire)) {
                    s_menuInputActive.store(true, std::memory_order_release);
                    return;
                }
            }

            s_menuInputActive.store(false, std::memory_order_release);
        }

        void refreshTrackedMenuState(const RE::UI& ui)
        {
            for (std::size_t i = 0; i < kGameStoppingMenuNames.size(); ++i) {
                const RE::BSFixedString menuName{ kGameStoppingMenuNames[i] };
                s_gameStoppingMenuOpen[i].store(ui.GetMenuOpen(menuName), std::memory_order_release);
            }

            publishMenuInputActiveFromTrackedMenus();
        }

        class MenuInputGate final : public RE::BSTEventSink<RE::MenuOpenCloseEvent>
        {
        public:
            RE::BSEventNotifyControl ProcessEvent(const RE::MenuOpenCloseEvent& event, RE::BSTEventSource<RE::MenuOpenCloseEvent>*) override
            {
                const auto menuIndex = findGameStoppingMenuIndex(event.menuName);
                if (!menuIndex) {
                    return RE::BSEventNotifyControl::kContinue;
                }

                s_gameStoppingMenuOpen[*menuIndex].store(event.opening, std::memory_order_release);
                publishMenuInputActiveFromTrackedMenus();
                ROCK_LOG_DEBUG(Input, "Input remap menu gate: {} {}", event.menuName.c_str(), event.opening ? "opened" : "closed");
                return RE::BSEventNotifyControl::kContinue;
            }
        };

        MenuInputGate s_menuInputGate;

        [[nodiscard]] constexpr std::size_t controllerIndex(input_remap_policy::Hand hand)
        {
            return hand == input_remap_policy::Hand::Left ? 0u : 1u;
        }

        [[nodiscard]] input_remap_policy::Settings makeSettings()
        {
            return input_remap_policy::Settings{
                .enabled = g_rockConfig.rockInputRemapEnabled,
                .grabButtonId = g_rockConfig.rockGrabButtonID,
                .weaponToggleButtonId = g_rockConfig.rockRightWeaponReadyButtonID,
                .suppressRightGrabGameInput = g_rockConfig.rockSuppressRightGrabGameInput,
                .suppressRightFavoritesGameInput = g_rockConfig.rockSuppressRightFavoritesGameInput,
                .suppressRightTriggerGameInput = g_rockConfig.rockSuppressNativeReadyWeaponAutoReady,
            };
        }

        [[nodiscard]] bool resolveControllerHand(vr::TrackedDeviceIndex_t deviceIndex, input_remap_policy::Hand& outHand)
        {
            auto* system = vr::VRSystem();
            if (!system || deviceIndex == vr::k_unTrackedDeviceIndexInvalid) {
                return false;
            }

            const auto rightIndex = system->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand);
            if (deviceIndex == rightIndex) {
                outHand = input_remap_policy::Hand::Right;
                return true;
            }

            const auto leftIndex = system->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_LeftHand);
            if (deviceIndex == leftIndex) {
                outHand = input_remap_policy::Hand::Left;
                return true;
            }

            return false;
        }

        [[nodiscard]] bool shouldFilterCaller(std::uintptr_t callerAddress)
        {
            const auto text = REL::Module::get().segment(REL::Segment::text);
            return input_remap_policy::shouldFilterGameFacingInput(callerAddress, text.address(), text.size());
        }

        bool ensureMenuInputGateRegistered()
        {
            auto* ui = RE::UI::GetSingleton();
            if (!ui) {
                if (!s_missingUILogged.exchange(true, std::memory_order_acq_rel)) {
                    ROCK_LOG_WARN(Input, "UI singleton unavailable; input remap menu gate will retry");
                }
                return false;
            }

            if (s_menuInputGateRegistered.load(std::memory_order_acquire)) {
                return true;
            }

            refreshTrackedMenuState(*ui);
            ui->RegisterSink(&s_menuInputGate);
            s_menuInputGateRegistered.store(true, std::memory_order_release);
            s_missingUILogged.store(false, std::memory_order_release);
            refreshTrackedMenuState(*ui);
            ROCK_LOG_INFO(Input, "Registered input remap menu gate for {} game-stopping menus", kGameStoppingMenuNames.size());
            return true;
        }

        [[nodiscard]] bool isGameStoppingMenuInputActive()
        {
            return s_menuInputActive.load(std::memory_order_acquire);
        }

        [[nodiscard]] bool isAddressInGameText(std::uintptr_t address)
        {
            const auto text = REL::Module::get().segment(REL::Segment::text);
            return text.size() != 0 && address >= text.address() && address < text.address() + text.size();
        }

        void captureAndFilterControllerState(vr::TrackedDeviceIndex_t deviceIndex, vr::VRControllerState_t* state, std::uint32_t stateSize, bool filterGameFacingState)
        {
            if (!state || stateSize < sizeof(vr::VRControllerState_t)) {
                return;
            }

            input_remap_policy::Hand hand{};
            if (!resolveControllerHand(deviceIndex, hand)) {
                return;
            }

            auto& tracker = s_controllers[controllerIndex(hand)];
            const std::uint64_t rawPressed = state->ulButtonPressed;
            const std::uint64_t rawTouched = state->ulButtonTouched;

            const bool hadPrevious = tracker.valid.exchange(true, std::memory_order_acq_rel);
            const std::uint64_t previousRawPressed = tracker.rawPressed.exchange(rawPressed, std::memory_order_acq_rel);
            tracker.rawTouched.store(rawTouched, std::memory_order_release);

            const auto rawTransition = input_remap_policy::evaluateEdgeTransition(hadPrevious, previousRawPressed, rawPressed);
            if (hadPrevious) {
                tracker.pressedEdges.fetch_or(rawTransition.pressedEdges, std::memory_order_acq_rel);
                tracker.releasedEdges.fetch_or(rawTransition.releasedEdges, std::memory_order_acq_rel);
            }

            if (!filterGameFacingState) {
                return;
            }

            const bool hadGameFacingPrevious = tracker.gameFacingValid.exchange(true, std::memory_order_acq_rel);
            const std::uint64_t previousGameFacingPressed = tracker.gameFacingPressed.exchange(rawPressed, std::memory_order_acq_rel);
            const auto gameFacingTransition = input_remap_policy::evaluateEdgeTransition(hadGameFacingPrevious, previousGameFacingPressed, rawPressed);

            const auto decision = input_remap_policy::evaluate(
                input_remap_policy::Input{
                    .hand = hand,
                    .gameplayInputAllowed = s_gameplayInputAllowed.load(std::memory_order_acquire),
                    .menuInputActive = isGameStoppingMenuInputActive(),
                    .weaponDrawn = s_weaponDrawn.load(std::memory_order_acquire),
                    .rawPressed = rawPressed,
                    .rawTouched = rawTouched,
                    .previousRawPressed = gameFacingTransition.previousPressedForEvaluation,
                },
                makeSettings());

            if (decision.weaponToggleRequested) {
                s_pendingWeaponToggleRequests.fetch_add(1, std::memory_order_acq_rel);
            }

            state->ulButtonPressed = decision.filteredPressed;
            state->ulButtonTouched = decision.filteredTouched;
            for (std::uint32_t axisIndex = 0; axisIndex < vr::k_unControllerStateAxisCount; ++axisIndex) {
                if ((decision.filteredAxisMask & (std::uint8_t{ 1 } << axisIndex)) == 0) {
                    continue;
                }

                state->rAxis[axisIndex].x = 0.0f;
                state->rAxis[axisIndex].y = 0.0f;
            }
        }

        bool hookedGetControllerState(
            vr::IVRSystem* system, vr::TrackedDeviceIndex_t controllerDeviceIndex, vr::VRControllerState_t* controllerState, std::uint32_t controllerStateSize)
        {
            const bool result = s_originalGetControllerState ? s_originalGetControllerState(system, controllerDeviceIndex, controllerState, controllerStateSize) : false;
            if (result) {
                captureAndFilterControllerState(
                    controllerDeviceIndex, controllerState, controllerStateSize, shouldFilterCaller(reinterpret_cast<std::uintptr_t>(_ReturnAddress())));
            }
            return result;
        }

        bool hookedGetControllerStateWithPose(vr::IVRSystem* system,
            vr::ETrackingUniverseOrigin origin,
            vr::TrackedDeviceIndex_t controllerDeviceIndex,
            vr::VRControllerState_t* controllerState,
            std::uint32_t controllerStateSize,
            vr::TrackedDevicePose_t* trackedDevicePose)
        {
            const bool result = s_originalGetControllerStateWithPose ?
                                    s_originalGetControllerStateWithPose(system, origin, controllerDeviceIndex, controllerState, controllerStateSize, trackedDevicePose) :
                                    false;
            if (result) {
                captureAndFilterControllerState(
                    controllerDeviceIndex, controllerState, controllerStateSize, shouldFilterCaller(reinterpret_cast<std::uintptr_t>(_ReturnAddress())));
            }
            return result;
        }

        bool patchPointerSlot(void** slot, void* hook, void*& original, const char* label)
        {
            if (!slot) {
                ROCK_LOG_ERROR(Input, "{} hook install failed: slot is null", label);
                return false;
            }

            if (*slot == hook) {
                return original != nullptr;
            }

            DWORD oldProtect = 0;
            if (!VirtualProtect(slot, sizeof(void*), PAGE_EXECUTE_READWRITE, &oldProtect)) {
                ROCK_LOG_ERROR(Input, "{} hook install failed: VirtualProtect failed", label);
                return false;
            }

            original = *slot;
            *slot = hook;
            FlushInstructionCache(GetCurrentProcess(), slot, sizeof(void*));
            VirtualProtect(slot, sizeof(void*), oldProtect, &oldProtect);

            ROCK_LOG_INFO(Input, "Installed {} hook at slot=0x{:X}, original=0x{:X}, hook=0x{:X}", label, reinterpret_cast<std::uintptr_t>(slot),
                reinterpret_cast<std::uintptr_t>(original),
                reinterpret_cast<std::uintptr_t>(hook));
            return original != nullptr;
        }

        bool patchVTableSlot(void** vtable, std::size_t index, void* hook, void*& original, const char* label)
        {
            if (!vtable) {
                ROCK_LOG_ERROR(Input, "{} hook install failed: vtable is null", label);
                return false;
            }

            return patchPointerSlot(&vtable[index], hook, original, label);
        }

        bool shouldSuppressNativeReadyWeaponAction(bool originalReadyActionMatched)
        {
            return input_remap_policy::shouldSuppressNativeReadyWeaponAutoReady(input_remap_policy::NativeReadyWeaponSuppressionInput{
                .remapEnabled = g_rockConfig.rockInputRemapEnabled,
                .suppressionEnabled = g_rockConfig.rockSuppressNativeReadyWeaponAutoReady,
                .gameplayInputAllowed = s_gameplayInputAllowed.load(std::memory_order_acquire),
                .menuInputActive = isGameStoppingMenuInputActive(),
                .weaponDrawn = s_weaponDrawn.load(std::memory_order_acquire),
                .originalReadyActionMatched = originalReadyActionMatched,
            });
        }

        bool hookedReadyWeaponHandler(void* handler, void* inputEvent)
        {
            const bool originalMatched = s_originalReadyWeaponHandler ? s_originalReadyWeaponHandler(handler, inputEvent) : false;
            if (shouldSuppressNativeReadyWeaponAction(originalMatched)) {
                ROCK_LOG_DEBUG(Input, "Suppressed native ReadyWeapon auto-ready while weapon is holstered");
                return false;
            }

            return originalMatched;
        }

        bool installNativeReadyWeaponSuppressionHook()
        {
            if (s_readyWeaponHookInstalled.load(std::memory_order_acquire)) {
                return true;
            }

            REL::Relocation<std::uintptr_t> slotEntry{ REL::Offset(kReadyWeaponHandlerVTableSlotOffset) };
            auto* slot = reinterpret_cast<void**>(slotEntry.address());
            const auto expectedTarget = REL::Offset(kReadyWeaponHandlerFunctionOffset).address();
            const auto currentTarget = slot ? reinterpret_cast<std::uintptr_t>(*slot) : 0;
            if (!slot || currentTarget == 0) {
                ROCK_LOG_ERROR(Input, "Failed to install ReadyWeapon action suppression hook: vtable slot 0x{:X} is invalid", slotEntry.address());
                return false;
            }

            if (currentTarget != expectedTarget) {
                if (isAddressInGameText(currentTarget)) {
                    ROCK_LOG_ERROR(Input,
                        "ReadyWeapon action suppression hook validation failed: slot 0x{:X} points to game text 0x{:X}, expected 0x{:X}",
                        slotEntry.address(),
                        currentTarget,
                        expectedTarget);
                    return false;
                }

                ROCK_LOG_WARN(Input,
                    "ReadyWeapon action suppression vtable slot 0x{:X} is already patched to external target 0x{:X}; ROCK will chain it",
                    slotEntry.address(),
                    currentTarget);
            }

            void* original = reinterpret_cast<void*>(s_originalReadyWeaponHandler);
            const bool installed = patchPointerSlot(slot, reinterpret_cast<void*>(&hookedReadyWeaponHandler), original, "ReadyWeapon action suppression");
            s_originalReadyWeaponHandler = reinterpret_cast<ReadyWeaponHandler_t>(original);
            s_readyWeaponHookInstalled.store(installed, std::memory_order_release);
            if (!installed) {
                ROCK_LOG_ERROR(Input, "Failed to install ReadyWeapon action suppression hook at vtable slot 0x{:X}", slotEntry.address());
            }
            return installed;
        }

        RawButtonState readRawButtonState(bool isLeft, int buttonId, bool consumeEdges)
        {
            RawButtonState result{};
            const auto mask = input_remap_policy::buttonMask(buttonId);
            if (mask == 0) {
                result.available = true;
                return result;
            }

            auto& tracker = s_controllers[isLeft ? 0u : 1u];
            if (!tracker.valid.load(std::memory_order_acquire)) {
                return result;
            }

            result.available = true;
            result.held = (tracker.rawPressed.load(std::memory_order_acquire) & mask) != 0;

            if (consumeEdges) {
                result.pressed = (tracker.pressedEdges.fetch_and(~mask, std::memory_order_acq_rel) & mask) != 0;
                result.released = (tracker.releasedEdges.fetch_and(~mask, std::memory_order_acq_rel) & mask) != 0;
            } else {
                result.pressed = (tracker.pressedEdges.load(std::memory_order_acquire) & mask) != 0;
                result.released = (tracker.releasedEdges.load(std::memory_order_acquire) & mask) != 0;
            }

            return result;
        }
    }

    bool installInputRemapHooks()
    {
        ensureMenuInputGateRegistered();

        const auto settings = makeSettings();
        const bool needsNativeReadyWeaponHook = input_remap_policy::shouldInstallNativeReadyWeaponSuppressionHook(
            g_rockConfig.rockInputRemapEnabled, g_rockConfig.rockSuppressNativeReadyWeaponAutoReady);
        const bool nativeReadyWeaponHookReady = !needsNativeReadyWeaponHook || installNativeReadyWeaponSuppressionHook();

        if (!settings.enabled) {
            return nativeReadyWeaponHookReady;
        }

        if (s_hooksInstalled.load(std::memory_order_acquire)) {
            return nativeReadyWeaponHookReady;
        }

        auto* system = vr::VRSystem();
        if (!system) {
            if (!s_missingVRSystemLogged.exchange(true, std::memory_order_acq_rel)) {
                ROCK_LOG_WARN(Input, "OpenVR IVRSystem unavailable; input remap hook will retry");
            }
            return false;
        }

        auto*** objectVTable = reinterpret_cast<void***>(system);
        s_vrSystemVTable = objectVTable ? *objectVTable : nullptr;

        void* originalState = reinterpret_cast<void*>(s_originalGetControllerState);
        void* originalStateWithPose = reinterpret_cast<void*>(s_originalGetControllerStateWithPose);

        const bool stateHooked = patchVTableSlot(
            s_vrSystemVTable, kGetControllerStateVTableIndex, reinterpret_cast<void*>(&hookedGetControllerState), originalState, "IVRSystem::GetControllerState");
        const bool stateWithPoseHooked = patchVTableSlot(s_vrSystemVTable,
            kGetControllerStateWithPoseVTableIndex,
            reinterpret_cast<void*>(&hookedGetControllerStateWithPose),
            originalStateWithPose,
            "IVRSystem::GetControllerStateWithPose");

        s_originalGetControllerState = reinterpret_cast<GetControllerState_t>(originalState);
        s_originalGetControllerStateWithPose = reinterpret_cast<GetControllerStateWithPose_t>(originalStateWithPose);

        const bool installed = stateHooked && stateWithPoseHooked;
        s_hooksInstalled.store(installed, std::memory_order_release);
        return installed && nativeReadyWeaponHookReady;
    }

    bool isInputRemapHookInstalled()
    {
        return s_hooksInstalled.load(std::memory_order_acquire);
    }

    void setGameplayInputAllowed(bool allowed)
    {
        s_gameplayInputAllowed.store(allowed, std::memory_order_release);
    }

    void setWeaponDrawn(bool weaponDrawn)
    {
        s_weaponDrawn.store(weaponDrawn, std::memory_order_release);
    }

    bool isMenuInputActive()
    {
        return isGameStoppingMenuInputActive();
    }

    void processPendingWeaponToggleRequests()
    {
        const std::uint32_t requestCount = s_pendingWeaponToggleRequests.exchange(0, std::memory_order_acq_rel);
        if (requestCount == 0) {
            return;
        }

        if (!g_rockConfig.rockInputRemapEnabled || !s_gameplayInputAllowed.load(std::memory_order_acquire) || isGameStoppingMenuInputActive()) {
            ROCK_LOG_DEBUG(Input, "Dropped {} pending weapon toggle request(s) because gameplay input is not active", requestCount);
            return;
        }

        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            ROCK_LOG_WARN(Input, "Dropped {} pending weapon toggle request(s): PlayerCharacter unavailable", requestCount);
            return;
        }

        const bool weaponDrawn = player->GetWeaponMagicDrawn();
        const bool targetDrawn = (requestCount % 2u) == 0u ? weaponDrawn : !weaponDrawn;
        if (targetDrawn == weaponDrawn) {
            ROCK_LOG_DEBUG(Input, "Consumed {} weapon toggle requests with no net weapon-state change", requestCount);
            return;
        }

        player->DrawWeaponMagicHands(targetDrawn);
        ROCK_LOG_INFO(Input, "Right stick click requested weapon {}", targetDrawn ? "draw" : "holster");
    }

    RawButtonState peekRawButtonState(bool isLeft, int buttonId)
    {
        return readRawButtonState(isLeft, buttonId, false);
    }

    RawButtonState consumeRawButtonState(bool isLeft, int buttonId)
    {
        return readRawButtonState(isLeft, buttonId, true);
    }
}
