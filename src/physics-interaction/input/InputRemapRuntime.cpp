#include "physics-interaction/input/InputRemapRuntime.h"

#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/PhysicsLog.h"
#include "RockConfig.h"

#include "f4vr/F4VRUtils.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/ControlMap.h"
#include "RE/Bethesda/InputEvent.h"
#include "RE/Bethesda/UI.h"

#include <REL/Relocation.h>
#include <vrcf/VRControllersManager.h>
#include <windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <optional>
#include <string_view>

namespace rock::input_remap_runtime
{
    namespace
    {
        constexpr std::size_t kGetControllerStateVTableIndex = 34;
        constexpr std::size_t kGetControllerStateWithPoseVTableIndex = 35;
        constexpr DWORD kPageExecuteReadWrite = 0x00000040u;
        constexpr std::uintptr_t kReadyWeaponHandlerHandleEventFunctionOffset = 0x0FC9220;
        constexpr std::uintptr_t kReadyWeaponHandlerHandleEventVTableSlotOffset = 0x2D8A4D0;
        constexpr std::uintptr_t kFavoritesManagerHandleEventFunctionOffset = 0x12F19D0;
        constexpr std::uintptr_t kFavoritesManagerHandleEventVTableSlotOffset = 0x2DC8520;
        constexpr std::uintptr_t kMeleeThrowHandlerHandleEventFunctionOffset = 0x0FC8AE0;
        constexpr std::uintptr_t kMeleeThrowHandlerHandleEventVTableSlotOffset = 0x2D8A9F0;
        constexpr std::uintptr_t kMeleeThrowFallbackDrawPressPatchSite = 0x0FC8C88;
        constexpr std::uintptr_t kMeleeThrowFallbackDrawReleasePatchSite = 0x0FC8E7E;
        constexpr std::uint8_t kConditionalShortJumpGreaterEqual = 0x7D;
        constexpr std::uint8_t kUnconditionalShortJump = 0xEB;
        constexpr std::uint8_t kMeleeThrowFallbackBranchDisplacement = 0x0D;
        constexpr std::string_view kNativeEventWandGrip{ "WandGrip" };
        constexpr std::string_view kNativeEventWandTrigger{ "WandTrigger" };
        constexpr std::string_view kNativeEventWandThumbClick{ "WandThumbClick" };

        using GetControllerState_t = bool (*)(vr::IVRSystem*, vr::TrackedDeviceIndex_t, vr::VRControllerState_t*, std::uint32_t);
        using GetControllerStateWithPose_t =
            bool (*)(vr::IVRSystem*, vr::ETrackingUniverseOrigin, vr::TrackedDeviceIndex_t, vr::VRControllerState_t*, std::uint32_t, vr::TrackedDevicePose_t*);
        using NativeInputEventHandler_t = void (*)(void*, RE::InputEvent*, void*, void*);
        using FavoritesInputEventHandler_t = void (*)(void*, RE::InputEvent*);

        struct ControllerTracker
        {
            std::atomic<std::uint64_t> rawPressed{ 0 };
            std::atomic<std::uint64_t> rawTouched{ 0 };
            std::atomic<float> rawAxis0X{ 0.0f };
            std::atomic<float> rawAxis0Y{ 0.0f };
            std::atomic<float> rawAxis1X{ 0.0f };
            std::atomic<std::uint64_t> pressedEdges{ 0 };
            std::atomic<std::uint64_t> releasedEdges{ 0 };
            std::atomic<bool> valid{ false };
            std::atomic<std::uint32_t> rawThumbDirection{ 0 };
        };

        std::array<ControllerTracker, 2> s_controllers;
        std::atomic<bool> s_gameplayInputAllowed{ false };
        std::atomic<bool> s_weaponDrawn{ false };
        std::atomic<std::uint32_t> s_pendingWeaponToggleRequests{ 0 };
        std::atomic<bool> s_hooksInstalled{ false };
        std::atomic<bool> s_readyWeaponEventHookInstalled{ false };
        std::atomic<bool> s_favoritesEventHookInstalled{ false };
        std::atomic<bool> s_meleeThrowEventHookInstalled{ false };
        std::atomic<bool> s_meleeThrowFallbackPatchesApplied{ false };
        std::atomic<bool> s_menuInputGateRegistered{ false };
        std::atomic<bool> s_menuInputActive{ false };
        std::atomic<bool> s_missingVRSystemLogged{ false };
        std::atomic<bool> s_missingUILogged{ false };
        std::atomic<std::uint64_t> s_externalDiagnosticSuppressionOwner{ 0 };
        std::atomic<std::uint32_t> s_externalDiagnosticSuppressionFlags{ 0 };
        std::atomic<std::uint32_t> s_diagnosticInputPressedFlags{ 0 };
        std::atomic<std::uint32_t> s_diagnosticInputReleasedFlags{ 0 };
        std::atomic<std::uint64_t> s_diagnosticInputSequence{ 0 };

        void** s_vrSystemVTable = nullptr;
        GetControllerState_t s_originalGetControllerState = nullptr;
        GetControllerStateWithPose_t s_originalGetControllerStateWithPose = nullptr;
        NativeInputEventHandler_t s_originalReadyWeaponEventHandler = nullptr;
        NativeInputEventHandler_t s_originalMeleeThrowEventHandler = nullptr;
        FavoritesInputEventHandler_t s_originalFavoritesEventHandler = nullptr;

        /*
         * ROCK remaps right-hand grab/trigger/thumbstick only while gameplay owns controller input.
         * Character creation is a menu-mode/input-context surface in FO4VR, and not every step reliably
         * behaves like a normal menu-stack open in the FRIK menu gate. ROCK stops grab remapping while
         * game-stopping menus are active and also watches FO4VR menu-mode/context state so chargen controls
         * receive raw OpenVR buttons.
         */
        constexpr std::array<std::string_view, 52> kGameStoppingMenuNames{
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
            "Looks Menu",
            "LooksMenu",
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
            "SPECIAL Menu",
            "SPECIALMenu",
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

        constexpr std::array<RE::UserEvents::INPUT_CONTEXT_ID, 5> kRemapBlockingInputContexts{
            RE::UserEvents::INPUT_CONTEXT_ID::kLooksMenu,
            RE::UserEvents::INPUT_CONTEXT_ID::kLevelUpMenu,
            RE::UserEvents::INPUT_CONTEXT_ID::kLevelUpMenuPrevNext,
            RE::UserEvents::INPUT_CONTEXT_ID::kPauseMenu,
            RE::UserEvents::INPUT_CONTEXT_ID::kMainMenu,
        };

        std::array<std::atomic<bool>, kGameStoppingMenuNames.size()> s_gameStoppingMenuOpen{};
        std::array<std::atomic<bool>, kGameStoppingMenuNames.size()> s_gameStoppingMenuModeOpen{};
        std::array<std::atomic<std::uint32_t>, kGameStoppingMenuNames.size()> s_gameStoppingMenuModeDepth{};

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
            for (std::size_t i = 0; i < kGameStoppingMenuNames.size(); ++i) {
                if (s_gameStoppingMenuOpen[i].load(std::memory_order_acquire) ||
                    s_gameStoppingMenuModeOpen[i].load(std::memory_order_acquire) ||
                    s_gameStoppingMenuModeDepth[i].load(std::memory_order_acquire) > 0) {
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
            , public RE::BSTEventSink<RE::MenuModeChangeEvent>
            , public RE::BSTEventSink<RE::MenuModeCounterChangedEvent>
        {
        public:
            RE::BSEventNotifyControl ProcessEvent(const RE::MenuOpenCloseEvent& event, RE::BSTEventSource<RE::MenuOpenCloseEvent>*) override
            {
                const auto menuIndex = findGameStoppingMenuIndex(event.menuName);
                if (!menuIndex) {
                    return RE::BSEventNotifyControl::kContinue;
                }

                s_gameStoppingMenuOpen[*menuIndex].store(event.opening, std::memory_order_release);
                if (!event.opening) {
                    s_gameStoppingMenuModeOpen[*menuIndex].store(false, std::memory_order_release);
                    s_gameStoppingMenuModeDepth[*menuIndex].store(0, std::memory_order_release);
                }
                publishMenuInputActiveFromTrackedMenus();
                ROCK_LOG_DEBUG(Input, "Input remap menu gate: {} {}", event.menuName.c_str(), event.opening ? "opened" : "closed");
                return RE::BSEventNotifyControl::kContinue;
            }

            RE::BSEventNotifyControl ProcessEvent(const RE::MenuModeChangeEvent& event, RE::BSTEventSource<RE::MenuModeChangeEvent>*) override
            {
                const auto menuIndex = findGameStoppingMenuIndex(event.menuName);
                if (!menuIndex) {
                    return RE::BSEventNotifyControl::kContinue;
                }

                s_gameStoppingMenuModeOpen[*menuIndex].store(event.enteringMenuMode, std::memory_order_release);
                if (!event.enteringMenuMode) {
                    s_gameStoppingMenuModeDepth[*menuIndex].store(0, std::memory_order_release);
                }
                publishMenuInputActiveFromTrackedMenus();
                ROCK_LOG_DEBUG(Input, "Input remap menu mode gate: {} {}", event.menuName.c_str(), event.enteringMenuMode ? "entered" : "left");
                return RE::BSEventNotifyControl::kContinue;
            }

            RE::BSEventNotifyControl ProcessEvent(const RE::MenuModeCounterChangedEvent& event, RE::BSTEventSource<RE::MenuModeCounterChangedEvent>*) override
            {
                const auto menuIndex = findGameStoppingMenuIndex(event.menuName);
                if (!menuIndex) {
                    return RE::BSEventNotifyControl::kContinue;
                }

                auto& depth = s_gameStoppingMenuModeDepth[*menuIndex];
                if (event.incrementing) {
                    depth.fetch_add(1, std::memory_order_acq_rel);
                } else {
                    std::uint32_t current = depth.load(std::memory_order_acquire);
                    while (current > 0 && !depth.compare_exchange_weak(current, current - 1, std::memory_order_acq_rel)) {
                    }
                }

                publishMenuInputActiveFromTrackedMenus();
                ROCK_LOG_DEBUG(Input, "Input remap menu mode counter gate: {} {}", event.menuName.c_str(), event.incrementing ? "incremented" : "decremented");
                return RE::BSEventNotifyControl::kContinue;
            }
        };

        MenuInputGate s_menuInputGate;

        [[nodiscard]] constexpr std::size_t controllerIndex(input_remap_policy::Hand hand)
        {
            return hand == input_remap_policy::Hand::Left ? 0u : 1u;
        }

        [[nodiscard]] bool isPrimaryHand(input_remap_policy::Hand hand)
        {
            const bool primaryIsLeft = f4vr::isLeftHandedMode();
            return hand == (primaryIsLeft ? input_remap_policy::Hand::Left : input_remap_policy::Hand::Right);
        }

        [[nodiscard]] std::uint32_t thumbDirectionFromAxis(float x, float y)
        {
            constexpr float threshold = 0.72f;
            if (!std::isfinite(x) || !std::isfinite(y)) {
                return 0;
            }

            const float absX = std::fabs(x);
            const float absY = std::fabs(y);
            if (absX < threshold && absY < threshold) {
                return 0;
            }

            if (absX >= absY) {
                return x < 0.0f ? kDiagnosticInputRightThumbstickLeftPressed : kDiagnosticInputRightThumbstickRightPressed;
            }
            return y < 0.0f ? kDiagnosticInputRightThumbstickDownPressed : kDiagnosticInputRightThumbstickUpPressed;
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
                .suppressNativeMeleeThrowGameInput = g_rockConfig.rockSuppressNativeMeleeThrowGameInput,
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
            ui->RegisterSink<RE::MenuOpenCloseEvent>(&s_menuInputGate);
            ui->RegisterSink<RE::MenuModeChangeEvent>(&s_menuInputGate);
            ui->RegisterSink<RE::MenuModeCounterChangedEvent>(&s_menuInputGate);
            s_menuInputGateRegistered.store(true, std::memory_order_release);
            s_missingUILogged.store(false, std::memory_order_release);
            refreshTrackedMenuState(*ui);
            ROCK_LOG_INFO(Input, "Registered input remap menu gate for {} game-stopping menus", kGameStoppingMenuNames.size());
            return true;
        }

        [[nodiscard]] bool isGameStoppingMenuInputActive()
        {
            if (s_menuInputActive.load(std::memory_order_acquire)) {
                return true;
            }

            auto* controlMap = RE::ControlMap::GetSingleton();
            if (!controlMap) {
                return false;
            }

            for (const auto& activeContext : controlMap->contextPriorityStack) {
                for (const auto blockedContext : kRemapBlockingInputContexts) {
                    if (activeContext == blockedContext) {
                        return true;
                    }
                }
            }

            return false;
        }

        [[nodiscard]] bool isAddressInGameText(std::uintptr_t address)
        {
            const auto text = REL::Module::get().segment(REL::Segment::text);
            return text.size() != 0 && address >= text.address() && address < text.address() + text.size();
        }

        void recordDiagnosticInputSample(input_remap_policy::Hand hand, const vr::VRControllerState_t& state, const input_remap_policy::EdgeTransition& rawTransition)
        {
            std::uint32_t pressedFlags = 0;
            std::uint32_t releasedFlags = 0;
            const auto triggerMask = input_remap_policy::buttonMask(input_remap_policy::kOpenVrSteamVrTriggerButtonId);
            if (triggerMask != 0 && isPrimaryHand(hand)) {
                if ((rawTransition.pressedEdges & triggerMask) != 0) {
                    pressedFlags |= kDiagnosticInputPrimaryTriggerPressed;
                }
                if ((rawTransition.releasedEdges & triggerMask) != 0) {
                    releasedFlags |= kDiagnosticInputPrimaryTriggerReleased;
                }
            }

            if (hand == input_remap_policy::Hand::Right) {
                auto& tracker = s_controllers[controllerIndex(hand)];
                const auto direction = thumbDirectionFromAxis(state.rAxis[0].x, state.rAxis[0].y);
                const auto previousDirection = tracker.rawThumbDirection.exchange(direction, std::memory_order_acq_rel);
                if (direction != 0 && direction != previousDirection) {
                    pressedFlags |= direction;
                }
            }

            if (pressedFlags != 0) {
                s_diagnosticInputPressedFlags.fetch_or(pressedFlags, std::memory_order_acq_rel);
            }
            if (releasedFlags != 0) {
                s_diagnosticInputReleasedFlags.fetch_or(releasedFlags, std::memory_order_acq_rel);
            }
            if (pressedFlags != 0 || releasedFlags != 0) {
                s_diagnosticInputSequence.fetch_add(1, std::memory_order_acq_rel);
            }
        }

        void captureControllerState(vr::TrackedDeviceIndex_t deviceIndex, vr::VRControllerState_t* state, std::uint32_t stateSize)
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
            tracker.rawAxis0X.store(state->rAxis[0].x, std::memory_order_release);
            tracker.rawAxis0Y.store(state->rAxis[0].y, std::memory_order_release);
            tracker.rawAxis1X.store(state->rAxis[1].x, std::memory_order_release);

            const auto rawTransition = input_remap_policy::evaluateEdgeTransition(hadPrevious, previousRawPressed, rawPressed);
            if (hadPrevious) {
                tracker.pressedEdges.fetch_or(rawTransition.pressedEdges, std::memory_order_acq_rel);
                tracker.releasedEdges.fetch_or(rawTransition.releasedEdges, std::memory_order_acq_rel);
            }
            recordDiagnosticInputSample(hand, *state, rawTransition);

            const auto decision = input_remap_policy::evaluate(
                input_remap_policy::Input{
                    .hand = hand,
                    .gameplayInputAllowed = s_gameplayInputAllowed.load(std::memory_order_acquire),
                    .menuInputActive = isGameStoppingMenuInputActive(),
                    .weaponDrawn = s_weaponDrawn.load(std::memory_order_acquire),
                    .rawPressed = rawPressed,
                    .rawTouched = rawTouched,
                    .previousRawPressed = rawTransition.previousPressedForEvaluation,
                },
                makeSettings());

            if (decision.weaponToggleRequested) {
                s_pendingWeaponToggleRequests.fetch_add(1, std::memory_order_acq_rel);
            }
        }

        bool hookedGetControllerState(
            vr::IVRSystem* system, vr::TrackedDeviceIndex_t controllerDeviceIndex, vr::VRControllerState_t* controllerState, std::uint32_t controllerStateSize)
        {
            const bool result = s_originalGetControllerState ? s_originalGetControllerState(system, controllerDeviceIndex, controllerState, controllerStateSize) : false;
            if (result) {
                captureControllerState(controllerDeviceIndex, controllerState, controllerStateSize);
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
                captureControllerState(controllerDeviceIndex, controllerState, controllerStateSize);
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
            if (!VirtualProtect(slot, sizeof(void*), kPageExecuteReadWrite, &oldProtect)) {
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

        /*
         * ROCK samples OpenVR controller state for its own intent detection, but
         * suppresses FO4VR's native auto-actions at their verified action handlers.
         * That keeps trigger/grip/thumbstick state readable for other OpenVR users
         * while preventing duplicate vanilla ready/favorites/attack behavior.
         */
        [[nodiscard]] bool eventNameMatches(const RE::InputEvent* event, std::string_view expected)
        {
            if (!event) {
                return false;
            }

            const auto& userEvent = event->QUserEvent();
            const auto* userEventText = userEvent.c_str();
            return std::string_view{ userEventText ? userEventText : "", userEvent.length() } == expected;
        }

        [[nodiscard]] input_remap_policy::NativeActionSuppressionInput makeNativeActionSuppressionInput(bool suppressionEnabled, bool eventMatched)
        {
            return input_remap_policy::NativeActionSuppressionInput{
                .remapEnabled = g_rockConfig.rockInputRemapEnabled,
                .suppressionEnabled = suppressionEnabled,
                .gameplayInputAllowed = s_gameplayInputAllowed.load(std::memory_order_acquire),
                .menuInputActive = isGameStoppingMenuInputActive(),
                .weaponDrawn = s_weaponDrawn.load(std::memory_order_acquire),
                .eventMatched = eventMatched,
            };
        }

        void markInputEventStopped(RE::InputEvent* event)
        {
            if (event) {
                event->handled = RE::InputEvent::HANDLED_RESULT::kStop;
            }
        }

        [[nodiscard]] bool shouldSuppressNativeGripReadyAction(const RE::InputEvent* event)
        {
            return input_remap_policy::shouldSuppressNativeGripReadyAction(
                makeNativeActionSuppressionInput(g_rockConfig.rockSuppressRightGrabGameInput, eventNameMatches(event, kNativeEventWandGrip)));
        }

        [[nodiscard]] bool shouldSuppressNativeFavoritesAction(const RE::InputEvent* event)
        {
            return input_remap_policy::shouldSuppressNativeFavoritesAction(
                makeNativeActionSuppressionInput(g_rockConfig.rockSuppressRightFavoritesGameInput, eventNameMatches(event, kNativeEventWandThumbClick)));
        }

        [[nodiscard]] bool shouldSuppressNativeTriggerActionEvent(const RE::InputEvent* event)
        {
            return input_remap_policy::shouldSuppressNativeTriggerAction(
                makeNativeActionSuppressionInput(g_rockConfig.rockSuppressNativeReadyWeaponAutoReady, eventNameMatches(event, kNativeEventWandTrigger)));
        }

        [[nodiscard]] bool shouldSuppressNativeMeleeThrowAction(const RE::InputEvent* event)
        {
            // FO4VR's verified MeleeThrow handler accepts its grenade/throw action from WandGrip.
            return input_remap_policy::shouldSuppressNativeMeleeThrowAction(
                makeNativeActionSuppressionInput(g_rockConfig.rockSuppressNativeMeleeThrowGameInput, eventNameMatches(event, kNativeEventWandGrip)));
        }

        void hookedReadyWeaponEventHandler(void* handler, RE::InputEvent* inputEvent, void* cursor, void* unk)
        {
            if (shouldSuppressNativeGripReadyAction(inputEvent)) {
                markInputEventStopped(inputEvent);
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed native WandGrip ReadyWeapon event while ROCK owns holstered right-grab input");
                return;
            }

            if (s_originalReadyWeaponEventHandler) {
                s_originalReadyWeaponEventHandler(handler, inputEvent, cursor, unk);
            }
        }

        void hookedFavoritesEventHandler(void* handler, RE::InputEvent* inputEvent)
        {
            if (shouldSuppressNativeFavoritesAction(inputEvent)) {
                markInputEventStopped(inputEvent);
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed native WandThumbClick Favorites event while ROCK owns right-stick weapon toggle");
                return;
            }

            if (s_originalFavoritesEventHandler) {
                s_originalFavoritesEventHandler(handler, inputEvent);
            }
        }

        void hookedMeleeThrowEventHandler(void* handler, RE::InputEvent* inputEvent, void* cursor, void* unk)
        {
            if (shouldSuppressNativeMeleeThrowAction(inputEvent)) {
                markInputEventStopped(inputEvent);
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed native WandGrip MeleeThrow event while ROCK owns gameplay grab input");
                return;
            }

            if (s_originalMeleeThrowEventHandler) {
                s_originalMeleeThrowEventHandler(handler, inputEvent, cursor, unk);
            }
        }

        template <class HandlerT>
        bool installNativeActionVTableHook(
            std::uintptr_t slotOffset, std::uintptr_t expectedFunctionOffset, HandlerT hook, HandlerT& original, std::atomic<bool>& installedFlag, const char* label)
        {
            if (installedFlag.load(std::memory_order_acquire)) {
                return true;
            }

            REL::Relocation<std::uintptr_t> slotEntry{ REL::Offset(slotOffset) };
            auto* slot = reinterpret_cast<void**>(slotEntry.address());
            const auto expectedTarget = REL::Offset(expectedFunctionOffset).address();
            const auto hookTarget = reinterpret_cast<std::uintptr_t>(hook);
            const auto currentTarget = slot ? reinterpret_cast<std::uintptr_t>(*slot) : 0;
            if (!slot || currentTarget == 0) {
                ROCK_LOG_ERROR(Input, "Failed to install {} hook: vtable slot 0x{:X} is invalid", label, slotEntry.address());
                return false;
            }

            if (currentTarget == hookTarget) {
                const bool installed = original != nullptr;
                installedFlag.store(installed, std::memory_order_release);
                return installed;
            }

            if (currentTarget != expectedTarget) {
                if (isAddressInGameText(currentTarget)) {
                    ROCK_LOG_ERROR(Input,
                        "{} hook validation failed: slot 0x{:X} points to game text 0x{:X}, expected 0x{:X}",
                        label,
                        slotEntry.address(),
                        currentTarget,
                        expectedTarget);
                    return false;
                }

                ROCK_LOG_WARN(Input, "{} vtable slot 0x{:X} is already patched to external target 0x{:X}; ROCK will chain it", label, slotEntry.address(), currentTarget);
            }

            void* originalPointer = reinterpret_cast<void*>(original);
            const bool installed = patchPointerSlot(slot, reinterpret_cast<void*>(hook), originalPointer, label);
            original = reinterpret_cast<HandlerT>(originalPointer);
            installedFlag.store(installed, std::memory_order_release);
            if (!installed) {
                ROCK_LOG_ERROR(Input, "Failed to install {} hook at vtable slot 0x{:X}", label, slotEntry.address());
            }
            return installed;
        }

        bool installReadyWeaponEventSuppressionHook()
        {
            return installNativeActionVTableHook(kReadyWeaponHandlerHandleEventVTableSlotOffset,
                kReadyWeaponHandlerHandleEventFunctionOffset,
                &hookedReadyWeaponEventHandler,
                s_originalReadyWeaponEventHandler,
                s_readyWeaponEventHookInstalled,
                "ReadyWeaponHandler::HandleEvent suppression");
        }

        bool installFavoritesEventSuppressionHook()
        {
            return installNativeActionVTableHook(kFavoritesManagerHandleEventVTableSlotOffset,
                kFavoritesManagerHandleEventFunctionOffset,
                &hookedFavoritesEventHandler,
                s_originalFavoritesEventHandler,
                s_favoritesEventHookInstalled,
                "FavoritesManager::HandleEvent suppression");
        }

        bool installMeleeThrowEventSuppressionHook()
        {
            return installNativeActionVTableHook(kMeleeThrowHandlerHandleEventVTableSlotOffset,
                kMeleeThrowHandlerHandleEventFunctionOffset,
                &hookedMeleeThrowEventHandler,
                s_originalMeleeThrowEventHandler,
                s_meleeThrowEventHookInstalled,
                "MeleeThrowHandler::HandleEvent suppression");
        }

        bool writeMeleeThrowFallbackBranch(std::uintptr_t siteOffset, bool suppress, const char* label)
        {
            REL::Relocation<std::uintptr_t> site{ REL::Offset(siteOffset) };
            auto* bytes = reinterpret_cast<std::uint8_t*>(site.address());
            if (!bytes) {
                ROCK_LOG_ERROR(Input, "{} patch failed: site is null", label);
                return false;
            }

            if (bytes[1] != kMeleeThrowFallbackBranchDisplacement) {
                ROCK_LOG_ERROR(Input,
                    "{} patch validation failed at 0x{:X}: expected branch displacement 0x{:02X}, found 0x{:02X}",
                    label,
                    site.address(),
                    kMeleeThrowFallbackBranchDisplacement,
                    bytes[1]);
                return false;
            }

            const auto desiredOpcode = suppress ? kUnconditionalShortJump : kConditionalShortJumpGreaterEqual;
            const auto expectedCurrentOpcode = suppress ? kConditionalShortJumpGreaterEqual : kUnconditionalShortJump;
            if (bytes[0] == desiredOpcode) {
                return true;
            }
            if (bytes[0] != expectedCurrentOpcode) {
                ROCK_LOG_ERROR(Input,
                    "{} patch validation failed at 0x{:X}: expected opcode 0x{:02X} or 0x{:02X}, found 0x{:02X}",
                    label,
                    site.address(),
                    desiredOpcode,
                    expectedCurrentOpcode,
                    bytes[0]);
                return false;
            }

            DWORD oldProtect = 0;
            if (!VirtualProtect(bytes, sizeof(std::uint8_t), kPageExecuteReadWrite, &oldProtect)) {
                ROCK_LOG_ERROR(Input, "{} patch failed at 0x{:X}: VirtualProtect failed", label, site.address());
                return false;
            }

            bytes[0] = desiredOpcode;
            FlushInstructionCache(GetCurrentProcess(), bytes, sizeof(std::uint8_t));
            VirtualProtect(bytes, sizeof(std::uint8_t), oldProtect, &oldProtect);

            ROCK_LOG_INFO(Input, "{} MeleeThrow fallback draw branch at 0x{:X}", suppress ? "Patched" : "Restored", site.address());
            return true;
        }

        bool updateMeleeThrowFallbackPatches(bool suppress)
        {
            const bool firstPatchOk = writeMeleeThrowFallbackBranch(kMeleeThrowFallbackDrawPressPatchSite,
                suppress,
                "MeleeThrowHandler fallback draw press");
            const bool secondPatchOk = writeMeleeThrowFallbackBranch(kMeleeThrowFallbackDrawReleasePatchSite,
                suppress,
                "MeleeThrowHandler fallback draw release");

            if (suppress && !(firstPatchOk && secondPatchOk)) {
                (void)writeMeleeThrowFallbackBranch(kMeleeThrowFallbackDrawPressPatchSite,
                    false,
                    "MeleeThrowHandler fallback draw press rollback");
                (void)writeMeleeThrowFallbackBranch(kMeleeThrowFallbackDrawReleasePatchSite,
                    false,
                    "MeleeThrowHandler fallback draw release rollback");
                s_meleeThrowFallbackPatchesApplied.store(false, std::memory_order_release);
                return false;
            }

            if (firstPatchOk && secondPatchOk) {
                s_meleeThrowFallbackPatchesApplied.store(suppress, std::memory_order_release);
            }
            return firstPatchOk && secondPatchOk;
        }

        bool updateNativeActionSuppressionHooks(const input_remap_policy::Settings& settings)
        {
            bool ready = true;
            if (input_remap_policy::shouldInstallNativeActionSuppressionHook(settings.enabled, settings.suppressRightGrabGameInput)) {
                ready = installReadyWeaponEventSuppressionHook() && ready;
            }
            if (input_remap_policy::shouldInstallNativeActionSuppressionHook(settings.enabled, settings.suppressRightFavoritesGameInput)) {
                ready = installFavoritesEventSuppressionHook() && ready;
            }
            if (input_remap_policy::shouldInstallNativeActionSuppressionHook(settings.enabled, settings.suppressNativeMeleeThrowGameInput)) {
                ready = installMeleeThrowEventSuppressionHook() && ready;
            }

            const bool suppressTriggerFallbacks = input_remap_policy::shouldInstallNativeActionSuppressionHook(settings.enabled, settings.suppressRightTriggerGameInput);
            ready = updateMeleeThrowFallbackPatches(suppressTriggerFallbacks) && ready;
            return ready;
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

        bool refreshControllerTrackerFromOpenVr(vr::ETrackedControllerRole role)
        {
            auto* system = vr::VRSystem();
            if (!system) {
                return false;
            }

            const auto index = system->GetTrackedDeviceIndexForControllerRole(role);
            if (index == vr::k_unTrackedDeviceIndexInvalid) {
                return false;
            }

            vr::VRControllerState_t state{};
            const bool ok = s_originalGetControllerState ?
                                s_originalGetControllerState(system, index, &state, sizeof(state)) :
                                system->GetControllerState(index, &state, sizeof(state));
            if (!ok) {
                return false;
            }

            captureControllerState(index, &state, sizeof(state));
            return true;
        }

        void refreshWeaponToggleControllerTracker()
        {
            (void)refreshControllerTrackerFromOpenVr(vr::TrackedControllerRole_RightHand);
        }

        void refreshDiagnosticControllerTrackers()
        {
            (void)refreshControllerTrackerFromOpenVr(vr::TrackedControllerRole_LeftHand);
            (void)refreshControllerTrackerFromOpenVr(vr::TrackedControllerRole_RightHand);
        }
    }

    bool installInputRemapHooks()
    {
        ensureMenuInputGateRegistered();

        const auto settings = makeSettings();
        const bool nativeActionSuppressionReady = updateNativeActionSuppressionHooks(settings);

        if (!settings.enabled) {
            return nativeActionSuppressionReady;
        }

        if (s_hooksInstalled.load(std::memory_order_acquire)) {
            return nativeActionSuppressionReady;
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
        return installed && nativeActionSuppressionReady;
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

    bool shouldSuppressNativeTriggerAction(const RE::InputEvent* event)
    {
        return shouldSuppressNativeTriggerActionEvent(event);
    }

    void processPendingWeaponToggleRequests()
    {
        refreshWeaponToggleControllerTracker();

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

    DiagnosticInputSnapshot consumeDiagnosticInputSnapshot()
    {
        refreshDiagnosticControllerTrackers();

        const bool primaryIsLeft = f4vr::isLeftHandedMode();
        auto& primary = s_controllers[primaryIsLeft ? 0u : 1u];
        auto& right = s_controllers[1u];

        DiagnosticInputSnapshot snapshot{};
        snapshot.sequence = s_diagnosticInputSequence.load(std::memory_order_acquire);
        snapshot.flags = s_diagnosticInputPressedFlags.exchange(0, std::memory_order_acq_rel) |
                         s_diagnosticInputReleasedFlags.exchange(0, std::memory_order_acq_rel);

        const auto triggerMask = input_remap_policy::buttonMask(input_remap_policy::kOpenVrSteamVrTriggerButtonId);
        if (triggerMask != 0 && (primary.rawPressed.load(std::memory_order_acquire) & triggerMask) != 0) {
            snapshot.flags |= kDiagnosticInputPrimaryTriggerHeld;
        }

        snapshot.rightThumbstickX = right.rawAxis0X.load(std::memory_order_acquire);
        snapshot.rightThumbstickY = right.rawAxis0Y.load(std::memory_order_acquire);
        snapshot.primaryTriggerAxisX = primary.rawAxis1X.load(std::memory_order_acquire);
        return snapshot;
    }

    bool setExternalPrimaryTriggerSuppression(std::uint64_t ownerToken, bool suppress)
    {
        return setExternalDiagnosticInputSuppression(ownerToken, suppress ? kDiagnosticSuppressionPrimaryTrigger : 0u);
    }

    bool setExternalDiagnosticInputSuppression(std::uint64_t ownerToken, std::uint32_t suppressionFlags)
    {
        if (ownerToken == 0) {
            return false;
        }

        if (suppressionFlags != 0) {
            const auto currentOwner = s_externalDiagnosticSuppressionOwner.exchange(ownerToken, std::memory_order_acq_rel);
            const auto previousFlags = s_externalDiagnosticSuppressionFlags.exchange(suppressionFlags, std::memory_order_acq_rel);
            if (currentOwner != ownerToken || previousFlags != suppressionFlags) {
                ROCK_LOG_INFO(Input,
                    "External diagnostic input suppression request noted by owner=0x{:X} flags=0x{:X}; OpenVR state remains read-only under native action suppression",
                    ownerToken,
                    suppressionFlags);
            }
            return true;
        }

        const auto currentOwner = s_externalDiagnosticSuppressionOwner.load(std::memory_order_acquire);
        if (currentOwner == ownerToken || currentOwner == 0) {
            const auto previousFlags = s_externalDiagnosticSuppressionFlags.exchange(0, std::memory_order_acq_rel);
            s_externalDiagnosticSuppressionOwner.store(0, std::memory_order_release);
            if (previousFlags != 0 || currentOwner != 0) {
                ROCK_LOG_INFO(Input, "External diagnostic input suppression released by owner=0x{:X}", ownerToken);
            }
            return true;
        }

        return false;
    }
}
