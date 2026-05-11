#include "physics-interaction/input/InputRemapRuntime.h"

#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/PhysicsLog.h"
#include "RockConfig.h"

#include "f4vr/F4VRUtils.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/ControlMap.h"
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
#include <intrin.h>
#include <optional>
#include <string_view>

namespace rock::input_remap_runtime
{
    namespace
    {
        constexpr std::size_t kGetControllerStateVTableIndex = 34;
        constexpr std::size_t kGetControllerStateWithPoseVTableIndex = 35;
        constexpr DWORD kPageExecuteReadWrite = 0x00000040u;
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
            std::atomic<float> rawAxis0X{ 0.0f };
            std::atomic<float> rawAxis0Y{ 0.0f };
            std::atomic<float> rawAxis1X{ 0.0f };
            std::atomic<std::uint64_t> pressedEdges{ 0 };
            std::atomic<std::uint64_t> releasedEdges{ 0 };
            std::atomic<bool> valid{ false };
            std::atomic<std::uint64_t> gameFacingPressed{ 0 };
            std::atomic<bool> gameFacingValid{ false };
            std::atomic<std::uint32_t> rawThumbDirection{ 0 };
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
        std::atomic<std::uint64_t> s_externalDiagnosticSuppressionOwner{ 0 };
        std::atomic<std::uint32_t> s_externalDiagnosticSuppressionFlags{ 0 };
        std::atomic<std::uint32_t> s_diagnosticInputPressedFlags{ 0 };
        std::atomic<std::uint32_t> s_diagnosticInputReleasedFlags{ 0 };
        std::atomic<std::uint64_t> s_diagnosticInputSequence{ 0 };

        void** s_vrSystemVTable = nullptr;
        GetControllerState_t s_originalGetControllerState = nullptr;
        GetControllerStateWithPose_t s_originalGetControllerStateWithPose = nullptr;
        ReadyWeaponHandler_t s_originalReadyWeaponHandler = nullptr;

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

        [[nodiscard]] std::uint32_t externalDiagnosticSuppressionFlagsFor(input_remap_policy::Hand hand, bool menuInputActive)
        {
            if (menuInputActive || s_externalDiagnosticSuppressionOwner.load(std::memory_order_acquire) == 0) {
                return 0;
            }

            const auto flags = s_externalDiagnosticSuppressionFlags.load(std::memory_order_acquire);
            std::uint32_t activeFlags = 0;
            if ((flags & kDiagnosticSuppressionPrimaryTrigger) != 0 && isPrimaryHand(hand)) {
                activeFlags |= kDiagnosticSuppressionPrimaryTrigger;
            }
            if ((flags & kDiagnosticSuppressionRightThumbstick) != 0 && hand == input_remap_policy::Hand::Right) {
                activeFlags |= kDiagnosticSuppressionRightThumbstick;
            }
            return activeFlags;
        }

        void suppressAxisButtonInGameFacingDecision(input_remap_policy::Decision& decision, int buttonId)
        {
            const auto buttonMask = input_remap_policy::buttonMask(buttonId);
            if (buttonMask == 0) {
                return;
            }

            decision.filteredPressed &= ~buttonMask;
            decision.filteredTouched &= ~buttonMask;
            decision.filteredAxisMask |= input_remap_policy::axisMaskFromOpenVrButtonId(buttonId);
        }

        void applyDiagnosticSuppression(input_remap_policy::Decision& decision, std::uint32_t suppressionFlags)
        {
            if ((suppressionFlags & kDiagnosticSuppressionPrimaryTrigger) != 0) {
                suppressAxisButtonInGameFacingDecision(decision, input_remap_policy::kOpenVrSteamVrTriggerButtonId);
            }
            if ((suppressionFlags & kDiagnosticSuppressionRightThumbstick) != 0) {
                suppressAxisButtonInGameFacingDecision(decision, input_remap_policy::kOpenVrAxisButtonBase);
            }
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
            tracker.rawAxis0X.store(state->rAxis[0].x, std::memory_order_release);
            tracker.rawAxis0Y.store(state->rAxis[0].y, std::memory_order_release);
            tracker.rawAxis1X.store(state->rAxis[1].x, std::memory_order_release);

            const auto rawTransition = input_remap_policy::evaluateEdgeTransition(hadPrevious, previousRawPressed, rawPressed);
            if (hadPrevious) {
                tracker.pressedEdges.fetch_or(rawTransition.pressedEdges, std::memory_order_acq_rel);
                tracker.releasedEdges.fetch_or(rawTransition.releasedEdges, std::memory_order_acq_rel);
            }
            recordDiagnosticInputSample(hand, *state, rawTransition);

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

            auto filteredDecision = decision;
            applyDiagnosticSuppression(filteredDecision, externalDiagnosticSuppressionFlagsFor(hand, isGameStoppingMenuInputActive()));

            if (filteredDecision.weaponToggleRequested) {
                s_pendingWeaponToggleRequests.fetch_add(1, std::memory_order_acq_rel);
            }

            state->ulButtonPressed = filteredDecision.filteredPressed;
            state->ulButtonTouched = filteredDecision.filteredTouched;
            for (std::uint32_t axisIndex = 0; axisIndex < vr::k_unControllerStateAxisCount; ++axisIndex) {
                if ((filteredDecision.filteredAxisMask & (std::uint8_t{ 1 } << axisIndex)) == 0) {
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

            captureAndFilterControllerState(index, &state, sizeof(state), false);
            return true;
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
                ROCK_LOG_INFO(Input, "External diagnostic input suppression acquired by owner=0x{:X} flags=0x{:X}", ownerToken, suppressionFlags);
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
