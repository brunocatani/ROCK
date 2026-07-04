#include "physics-interaction/input/InputRemapRuntime.h"

#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/PhysicsLog.h"
#include "RockConfig.h"

#include "api/ROCKProviderApi.h"
#include "api/FRIKApi.h"

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
        constexpr std::uintptr_t kReadyWeaponHandlerHandleEventFunctionOffset = 0x0FC9220;
        constexpr std::uintptr_t kReadyWeaponHandlerHandleEventVTableSlotOffset = 0x2D8A4D0;
        constexpr std::uintptr_t kActivateHandlerHandleEventFunctionOffset = 0x0FC7F00;
        constexpr std::uintptr_t kActivateHandlerHandleEventVTableSlotOffset = 0x2D8A640;
        constexpr std::uintptr_t kFavoritesManagerHandleEventFunctionOffset = 0x12F19D0;
        constexpr std::uintptr_t kFavoritesManagerHandleEventVTableSlotOffset = 0x2DC8520;
        constexpr std::uintptr_t kMeleeThrowHandlerHandleEventFunctionOffset = 0x0FC8AE0;
        constexpr std::uintptr_t kMeleeThrowHandlerHandleEventVTableSlotOffset = 0x2D8A9F0;
        /*
         * PipboyHandler (BSInputEventUser in the MenuControls chain, vtable
         * 0x2DCC778) processes the pipboy-hand trigger in vtable slot 11
         * (0x2DCC7D0): press starts hold tracking, holding past the game
         * threshold toggles the pipboy light, release opens the Pip-Boy.
         * Verified 2026-07-04 from raw disassembly of the constructor
         * (0x1325090 region) and the slot-11 processor (0x1326D90); the same
         * function independently re-uses the already-verified player global
         * (0x5B043F0), device-to-controller-id converter (0x1BA6ED0), and
         * action dispatcher data (0x5A3B8A0). ShouldHandleEvent (slot 1) also
         * accepts "Pause" and the Quick* tab events, so the hook must match
         * the "Pipboy" user event exactly and never swallow the rest.
         */
        constexpr std::uintptr_t kPipboyHandlerHandleEventFunctionOffset = 0x1326D90;
        constexpr std::uintptr_t kPipboyHandlerHandleEventVTableSlotOffset = 0x2DCC7D0;
        constexpr std::uintptr_t kNativeActionDispatcherFunctionOffset = 0x0FC07E0;
        constexpr std::uintptr_t kNativeInputDeviceToControllerIdFunctionOffset = 0x1BA6ED0;
        constexpr std::uintptr_t kNativePlayerActionDispatcherDataOffset = 0x5A3B8A0;
        constexpr std::uintptr_t kNativePlayerDataOffset = 0x5B043F0;
        constexpr std::ptrdiff_t kNativePrimaryWandDeviceIdOffset = 0x8CC;
        constexpr int kNativeReloadActionId = 0x6C;
        constexpr std::uint32_t kNativeActionPriorityQueue = 2;
        constexpr std::uintptr_t kMeleeThrowFallbackDrawPressPatchSite = 0x0FC8C88;
        constexpr std::uintptr_t kMeleeThrowFallbackDrawReleasePatchSite = 0x0FC8E7E;
        constexpr std::uint8_t kConditionalShortJumpGreaterEqual = 0x7D;
        constexpr std::uint8_t kUnconditionalShortJump = 0xEB;
        constexpr std::uint8_t kMeleeThrowFallbackBranchDisplacement = 0x0D;
        constexpr std::string_view kNativeEventActivate{ "Activate" };
        constexpr std::string_view kNativeEventWandAccept{ "WandAccept" };
        constexpr std::string_view kNativeEventWandGrip{ "WandGrip" };
        constexpr std::string_view kNativeEventWandTrigger{ "WandTrigger" };
        constexpr std::string_view kNativeEventWandThumbClick{ "WandThumbClick" };
        constexpr std::string_view kNativeEventPipboy{ "Pipboy" };

        using GetControllerState_t = bool (*)(vr::IVRSystem*, vr::TrackedDeviceIndex_t, vr::VRControllerState_t*, std::uint32_t);
        using GetControllerStateWithPose_t =
            bool (*)(vr::IVRSystem*, vr::ETrackingUniverseOrigin, vr::TrackedDeviceIndex_t, vr::VRControllerState_t*, std::uint32_t, vr::TrackedDevicePose_t*);
        using NativeInputEventHandler_t = void (*)(void*, RE::InputEvent*, void*, void*);
        using NativeActionDispatcher_t = bool (*)(void*, int, std::uint32_t);
        using NativeInputDeviceToControllerId_t = std::int32_t (*)(std::int32_t);
        using FavoritesInputEventHandler_t = void (*)(void*, RE::InputEvent*);
        // Verified PipboyHandler slot-11 signature: (this, event) only; no cursor/unk tail like the PlayerControls handlers.
        using PipboyInputEventHandler_t = void (*)(void*, RE::InputEvent*);

        struct ControllerTracker
        {
            std::atomic<std::uint64_t> rawPressed{ 0 };
            std::atomic<std::uint64_t> rawTouched{ 0 };
            std::atomic<std::uint64_t> pressedEdges{ 0 };
            std::atomic<std::uint64_t> releasedEdges{ 0 };
            std::atomic<std::uint64_t> rearmPressedMask{ 0 };
            std::atomic<bool> valid{ false };
            std::atomic<bool> weaponToggleTracking{ false };
            std::atomic<bool> weaponToggleEligibleAtPress{ false };
            std::atomic<bool> weaponToggleBlocked{ false };
            std::atomic<std::uint64_t> weaponTogglePressStartMs{ 0 };
        };

        std::array<ControllerTracker, 2> s_controllers;
        std::atomic<bool> s_gameplayInputAllowed{ false };
        std::atomic<bool> s_weaponDrawn{ false };
        std::atomic<bool> s_rightHandHeldWeapon{ false };
        std::array<std::atomic<bool>, 2> s_handInteractionEngaged{};
        std::atomic<bool> s_equippedWeaponPrimaryDetachInputActive{ false };
        std::atomic<bool> s_equippedWeaponPrimaryDetached{ false };
        std::atomic<std::uint32_t> s_pendingWeaponToggleRequests{ 0 };
        std::atomic<bool> s_hooksInstalled{ false };
        std::atomic<bool> s_readyWeaponEventHookInstalled{ false };
        std::atomic<bool> s_activateEventHookInstalled{ false };
        std::atomic<bool> s_favoritesEventHookInstalled{ false };
        std::atomic<bool> s_meleeThrowEventHookInstalled{ false };
        std::atomic<bool> s_pipboyEventHookInstalled{ false };
        std::atomic<bool> s_meleeThrowFallbackPatchesApplied{ false };
        std::atomic<bool> s_menuInputGateRegistered{ false };
        std::atomic<bool> s_menuInputActive{ false };
        std::atomic<bool> s_missingVRSystemLogged{ false };
        std::atomic<bool> s_missingUILogged{ false };
        std::array<std::atomic<bool>, 2> s_providerOpenVrGameInputSuppressed{};
        void** s_vrSystemVTable = nullptr;
        GetControllerState_t s_originalGetControllerState = nullptr;
        GetControllerStateWithPose_t s_originalGetControllerStateWithPose = nullptr;
        NativeInputEventHandler_t s_originalReadyWeaponEventHandler = nullptr;
        NativeInputEventHandler_t s_originalActivateEventHandler = nullptr;
        NativeInputEventHandler_t s_originalMeleeThrowEventHandler = nullptr;
        FavoritesInputEventHandler_t s_originalFavoritesEventHandler = nullptr;
        PipboyInputEventHandler_t s_originalPipboyEventHandler = nullptr;

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

        [[nodiscard]] bool isCompatibilityConfigInputActive()
        {
            const auto* frikApi = frik::api::FRIKApi::inst;
            return frikApi &&
                   ((frikApi->isConfigOpen && frikApi->isConfigOpen()) ||
                       (frikApi->isWristPipboyOpen && frikApi->isWristPipboyOpen()));
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

        [[nodiscard]] bool isProviderOpenVrGameInputSuppressed(input_remap_policy::Hand hand)
        {
            return s_providerOpenVrGameInputSuppressed[controllerIndex(hand)].load(std::memory_order_acquire);
        }

        [[nodiscard]] bool isAnyProviderOpenVrGameInputSuppressed()
        {
            return s_providerOpenVrGameInputSuppressed[0].load(std::memory_order_acquire) ||
                   s_providerOpenVrGameInputSuppressed[1].load(std::memory_order_acquire);
        }

        [[nodiscard]] bool isCallerModule(const void* address, const wchar_t* moduleName)
        {
            if (!address || !moduleName) {
                return false;
            }

            HMODULE callerModule = nullptr;
            if (!GetModuleHandleExW(
                    GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                    reinterpret_cast<LPCWSTR>(address),
                    &callerModule) ||
                !callerModule) {
                return false;
            }

            return callerModule == GetModuleHandleW(moduleName);
        }

        [[nodiscard]] bool isModuleOnCurrentStack(const wchar_t* moduleName)
        {
            auto* targetModule = GetModuleHandleW(moduleName);
            if (!targetModule) {
                return false;
            }

            void* frames[16]{};
            const auto frameCount = CaptureStackBackTrace(0, static_cast<DWORD>(sizeof(frames) / sizeof(frames[0])), frames, nullptr);
            for (USHORT i = 0; i < frameCount; ++i) {
                HMODULE frameModule = nullptr;
                if (GetModuleHandleExW(
                        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                        reinterpret_cast<LPCWSTR>(frames[i]),
                        &frameModule) &&
                    frameModule == targetModule) {
                    return true;
                }
            }

            return false;
        }

        [[nodiscard]] bool shouldBypassProviderOpenVrGameInputSuppression(const void* callerAddress)
        {
            /*
             * The configurator consumes raw controller input through ROCK while its lease masks game-facing state.
             * Some helper paths call through framework/static-library frames before reaching OpenVR, so the immediate
             * return address is not always enough to identify the configurator as the consumer.
             */
            return isCallerModule(callerAddress, L"ROCKConfigurator.dll") ||
                   isModuleOnCurrentStack(L"ROCKConfigurator.dll");
        }

        void clearOpenVrControllerStateForGame(vr::VRControllerState_t* state, std::uint32_t stateSize)
        {
            if (!state || stateSize < sizeof(vr::VRControllerState_t)) {
                return;
            }

            state->ulButtonPressed = 0;
            state->ulButtonTouched = 0;
            for (auto& axis : state->rAxis) {
                axis.x = 0.0f;
                axis.y = 0.0f;
            }
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
                .suppressPipboyGameInputWhileHolding = g_rockConfig.rockSuppressPipboyGameInputWhileHolding,
                .virtualHolstersCompatibilityEnabled = g_rockConfig.rockVirtualHolstersCompatibilityEnabled,
                .virtualHolstersDeferGrabInZone = g_rockConfig.rockVirtualHolstersDeferGrabInZone,
                .virtualHolstersDeferWeaponToggleInZone = g_rockConfig.rockVirtualHolstersDeferWeaponToggleInZone,
                .virtualHolstersDeferOnlyMatchingButton = g_rockConfig.rockVirtualHolstersDeferOnlyMatchingButton,
                .realisticWeaponHandlingEnabled = g_rockConfig.rockRealisticWeaponHandlingEnabled,
            };
        }

        /*
         * Optional ABI bridge into VirtualHolsters. ROCK never loads the DLL and
         * only resolves the exported API if VirtualHolsters is already present.
         * The local prefix preserves VirtualHolstersAPI.h vtable slots through
         * GetHolsterButtonId; ROCK does not take ownership of the object.
         */
        class VirtualHolstersAPI
        {
        public:
            virtual std::uint32_t __cdecl GetVersion() const = 0;
            virtual bool __cdecl IsHandInHolsterZone(bool isLeft) const = 0;
            virtual std::uint32_t __cdecl GetCurrentHolster() const = 0;
            virtual bool __cdecl IsHolsterFree(std::uint32_t holsterIndex) const = 0;
            virtual const char* __cdecl GetHolsteredWeaponName(std::uint32_t holsterIndex) const = 0;
            virtual bool __cdecl IsWeaponAlreadyHolstered(const char* weaponName) const = 0;
            virtual bool __cdecl GetHolsterPosition(std::uint32_t holsterIndex, float& outX, float& outY, float& outZ) const = 0;
            virtual float __cdecl GetHolsterRadius(std::uint32_t holsterIndex) const = 0;
            virtual bool __cdecl IsInitialized() const = 0;
            virtual bool __cdecl IsGripAssignedToHolster() const = 0;
            virtual std::uint32_t __cdecl GetHolsterButtonId() const = 0;
        };

        using GetVirtualHolstersApi_t = VirtualHolstersAPI*(__cdecl*)();

        struct VirtualHolstersState
        {
            bool available{ false };
            bool initialized{ false };
            bool handInZone{ false };
            int holsterButtonId{ -1 };
        };

        std::atomic<VirtualHolstersAPI*> s_virtualHolstersApi{ nullptr };
        std::atomic<std::uint64_t> s_nextVirtualHolstersProbeMs{ 0 };
        std::atomic<bool> s_virtualHolstersResolvedLogged{ false };
        std::atomic<bool> s_virtualHolstersInvalidLogged{ false };

        [[nodiscard]] VirtualHolstersAPI* resolveVirtualHolstersApi()
        {
            if (!g_rockConfig.rockVirtualHolstersCompatibilityEnabled || g_rockConfig.rockRealisticWeaponHandlingEnabled) {
                return nullptr;
            }

            if (auto* cachedApi = s_virtualHolstersApi.load(std::memory_order_acquire)) {
                return cachedApi;
            }

            const auto nowMs = static_cast<std::uint64_t>(GetTickCount64());
            auto nextProbeMs = s_nextVirtualHolstersProbeMs.load(std::memory_order_acquire);
            if (nowMs < nextProbeMs) {
                return nullptr;
            }
            if (!s_nextVirtualHolstersProbeMs.compare_exchange_strong(nextProbeMs, nowMs + 3000u, std::memory_order_acq_rel)) {
                return nullptr;
            }

            auto* module = GetModuleHandleA("VirtualHolsters.dll");
            if (!module) {
                return nullptr;
            }

            auto* getApi = reinterpret_cast<GetVirtualHolstersApi_t>(GetProcAddress(module, "VHAPI_GetApi"));
            if (!getApi) {
                if (!s_virtualHolstersInvalidLogged.exchange(true, std::memory_order_acq_rel)) {
                    ROCK_LOG_WARN(Input, "VirtualHolsters.dll is loaded but VHAPI_GetApi was not exported; compatibility bridge disabled");
                }
                return nullptr;
            }

            auto* api = getApi();
            if (!api) {
                if (!s_virtualHolstersInvalidLogged.exchange(true, std::memory_order_acq_rel)) {
                    ROCK_LOG_WARN(Input, "VirtualHolsters VHAPI_GetApi returned null; compatibility bridge disabled");
                }
                return nullptr;
            }

            const auto version = api->GetVersion();
            if (version < 1) {
                if (!s_virtualHolstersInvalidLogged.exchange(true, std::memory_order_acq_rel)) {
                    ROCK_LOG_WARN(Input, "VirtualHolsters API version {} is unsupported; compatibility bridge disabled", version);
                }
                return nullptr;
            }

            s_virtualHolstersApi.store(api, std::memory_order_release);
            if (!s_virtualHolstersResolvedLogged.exchange(true, std::memory_order_acq_rel)) {
                ROCK_LOG_INFO(Input, "Resolved VirtualHolsters API v{} for optional input compatibility", version);
            }
            return api;
        }

        [[nodiscard]] VirtualHolstersState queryVirtualHolstersState(bool isLeft)
        {
            VirtualHolstersState state{};
            auto* api = resolveVirtualHolstersApi();
            if (!api) {
                return state;
            }

            state.available = true;
            state.initialized = api->IsInitialized();
            if (!state.initialized) {
                return state;
            }

            state.handInZone = api->IsHandInHolsterZone(isLeft);
            if (state.handInZone) {
                state.holsterButtonId = static_cast<int>(api->GetHolsterButtonId());
            }
            return state;
        }

        [[nodiscard]] bool shouldDeferVirtualHolstersInput(bool isLeft, int buttonId, bool deferActionEnabled, std::string_view actionName)
        {
            const auto settings = makeSettings();
            const auto virtualHolsters = queryVirtualHolstersState(isLeft);
            const bool defer = input_remap_policy::shouldDeferVirtualHolstersInput(input_remap_policy::VirtualHolstersCompatibilityInput{
                .compatibilityEnabled = settings.virtualHolstersCompatibilityEnabled,
                .deferActionEnabled = deferActionEnabled,
                .deferOnlyMatchingButton = settings.virtualHolstersDeferOnlyMatchingButton,
                .realisticWeaponHandlingEnabled = settings.realisticWeaponHandlingEnabled,
                .apiAvailable = virtualHolsters.available,
                .initialized = virtualHolsters.initialized,
                .handInZone = virtualHolsters.handInZone,
                .rockButtonId = buttonId,
                .holsterButtonId = virtualHolsters.holsterButtonId,
            });

            if (defer) {
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Deferred ROCK {} input for {} hand in VirtualHolsters zone: ROCK button={} VH button={} matchOnly={}",
                    actionName,
                    isLeft ? "left" : "right",
                    buttonId,
                    virtualHolsters.holsterButtonId,
                    settings.virtualHolstersDeferOnlyMatchingButton ? "yes" : "no");
            }
            return defer;
        }

        [[nodiscard]] bool shouldDeferWeaponToggleForVirtualHolsters()
        {
            return shouldDeferVirtualHolstersInput(false,
                g_rockConfig.rockRightWeaponReadyButtonID,
                g_rockConfig.rockVirtualHolstersDeferWeaponToggleInZone,
                "weapon toggle");
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

        [[nodiscard]] bool isInputBlockingMenuActive()
        {
            return isGameStoppingMenuInputActive() || isCompatibilityConfigInputActive();
        }

        [[nodiscard]] double currentTimeSeconds()
        {
            return static_cast<double>(GetTickCount64()) / 1000.0;
        }

        [[nodiscard]] input_remap_policy::WeaponToggleClickState loadWeaponToggleClickState(const ControllerTracker& tracker)
        {
            return input_remap_policy::WeaponToggleClickState{
                .tracking = tracker.weaponToggleTracking.load(std::memory_order_acquire),
                .eligibleAtPress = tracker.weaponToggleEligibleAtPress.load(std::memory_order_acquire),
                .blocked = tracker.weaponToggleBlocked.load(std::memory_order_acquire),
                .pressStartSeconds = static_cast<double>(tracker.weaponTogglePressStartMs.load(std::memory_order_acquire)) / 1000.0,
            };
        }

        void storeWeaponToggleClickState(ControllerTracker& tracker, const input_remap_policy::WeaponToggleClickState& state)
        {
            tracker.weaponToggleTracking.store(state.tracking, std::memory_order_release);
            tracker.weaponToggleEligibleAtPress.store(state.eligibleAtPress, std::memory_order_release);
            tracker.weaponToggleBlocked.store(state.blocked, std::memory_order_release);
            tracker.weaponTogglePressStartMs.store(
                state.pressStartSeconds > 0.0 ? static_cast<std::uint64_t>(state.pressStartSeconds * 1000.0) : 0u,
                std::memory_order_release);
        }

        void clearButtonEdges(ControllerTracker& tracker, std::uint64_t mask)
        {
            tracker.pressedEdges.fetch_and(~mask, std::memory_order_acq_rel);
            tracker.releasedEdges.fetch_and(~mask, std::memory_order_acq_rel);
        }

        [[nodiscard]] bool isAddressInGameText(std::uintptr_t address)
        {
            const auto text = REL::Module::get().segment(REL::Segment::text);
            return text.size() != 0 && address >= text.address() && address < text.address() + text.size();
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

            const auto rawTransition = input_remap_policy::evaluateEdgeTransition(hadPrevious, previousRawPressed, rawPressed);
            if (hadPrevious) {
                tracker.pressedEdges.fetch_or(rawTransition.pressedEdges, std::memory_order_acq_rel);
                tracker.releasedEdges.fetch_or(rawTransition.releasedEdges, std::memory_order_acq_rel);
            }

            const bool inputBlockingMenuActive = isInputBlockingMenuActive();
            if (inputBlockingMenuActive) {
                tracker.rearmPressedMask.fetch_or(rawPressed, std::memory_order_acq_rel);
            } else {
                const auto rearmMask = tracker.rearmPressedMask.load(std::memory_order_acquire);
                const auto releasedFromRearm = rearmMask & ~rawPressed;
                if (releasedFromRearm != 0) {
                    clearButtonEdges(tracker, releasedFromRearm);
                    tracker.rearmPressedMask.fetch_and(~releasedFromRearm, std::memory_order_acq_rel);
                }
            }

            const auto settings = makeSettings();
            const auto weaponToggleMask = input_remap_policy::buttonMask(settings.weaponToggleButtonId);
            auto weaponToggleClickState = loadWeaponToggleClickState(tracker);
            const auto weaponToggleClick = input_remap_policy::updateWeaponToggleClick(weaponToggleClickState,
                input_remap_policy::WeaponToggleClickInput{
                    .enabled = settings.enabled,
                    .gameplayInputAllowed = s_gameplayInputAllowed.load(std::memory_order_acquire),
                    .menuInputActive = inputBlockingMenuActive,
                    .rightHand = hand == input_remap_policy::Hand::Right,
                    .held = weaponToggleMask != 0 && (rawPressed & weaponToggleMask) != 0,
                    .pressed = hadPrevious && weaponToggleMask != 0 && (rawTransition.pressedEdges & weaponToggleMask) != 0,
                    .released = hadPrevious && weaponToggleMask != 0 && (rawTransition.releasedEdges & weaponToggleMask) != 0,
                    .currentTimeSeconds = currentTimeSeconds(),
                });
            storeWeaponToggleClickState(tracker, weaponToggleClickState);

            if (weaponToggleClick.weaponToggleRequested) {
                s_pendingWeaponToggleRequests.fetch_add(1, std::memory_order_acq_rel);
            }
        }

        bool hookedGetControllerState(
            vr::IVRSystem* system, vr::TrackedDeviceIndex_t controllerDeviceIndex, vr::VRControllerState_t* controllerState, std::uint32_t controllerStateSize)
        {
            const void* callerAddress = _ReturnAddress();
            const bool result = s_originalGetControllerState ? s_originalGetControllerState(system, controllerDeviceIndex, controllerState, controllerStateSize) : false;
            if (result) {
                captureControllerState(controllerDeviceIndex, controllerState, controllerStateSize);
                input_remap_policy::Hand hand{};
                if (resolveControllerHand(controllerDeviceIndex, hand)) {
                    if (isProviderOpenVrGameInputSuppressed(hand) &&
                        !shouldBypassProviderOpenVrGameInputSuppression(callerAddress)) {
                        clearOpenVrControllerStateForGame(controllerState, controllerStateSize);
                        return result;
                    }
                }
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
            const void* callerAddress = _ReturnAddress();
            const bool result = s_originalGetControllerStateWithPose ?
                                    s_originalGetControllerStateWithPose(system, origin, controllerDeviceIndex, controllerState, controllerStateSize, trackedDevicePose) :
                                    false;
            if (result) {
                captureControllerState(controllerDeviceIndex, controllerState, controllerStateSize);
                input_remap_policy::Hand hand{};
                if (resolveControllerHand(controllerDeviceIndex, hand)) {
                    if (isProviderOpenVrGameInputSuppressed(hand) &&
                        !shouldBypassProviderOpenVrGameInputSuppression(callerAddress)) {
                        clearOpenVrControllerStateForGame(controllerState, controllerStateSize);
                        return result;
                    }
                }
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

        [[nodiscard]] bool isPrimaryWandInputEvent(const RE::InputEvent* event)
        {
            if (!event) {
                return false;
            }

            static REL::Relocation<NativeInputDeviceToControllerId_t> nativeDeviceToControllerId{ REL::Offset(kNativeInputDeviceToControllerIdFunctionOffset) };
            static REL::Relocation<void**> nativePlayer{ REL::Offset(kNativePlayerDataOffset) };

            auto* player = *nativePlayer;
            if (!player) {
                return false;
            }

            const auto primaryWandDeviceId = *reinterpret_cast<const std::int32_t*>(reinterpret_cast<std::uintptr_t>(player) + kNativePrimaryWandDeviceIdOffset);
            return nativeDeviceToControllerId(event->deviceID) == primaryWandDeviceId;
        }

        [[nodiscard]] bool isActivateReloadEvent(const RE::InputEvent* event)
        {
            return eventNameMatches(event, kNativeEventActivate) || eventNameMatches(event, kNativeEventWandAccept);
        }

        [[nodiscard]] int nativeEventButtonIdForVirtualHolsters(const RE::InputEvent* event)
        {
            const auto* idEvent = event ? event->As<RE::IDEvent>() : nullptr;
            if (!idEvent) {
                return -1;
            }

            const auto buttonId = static_cast<int>(idEvent->QIDCode());
            return input_remap_policy::isValidButtonId(buttonId) ? buttonId : -1;
        }

        [[nodiscard]] input_remap_policy::NativeActionSuppressionInput makeNativeActionSuppressionInput(bool suppressionEnabled, bool eventMatched)
        {
            return input_remap_policy::NativeActionSuppressionInput{
                .remapEnabled = g_rockConfig.rockInputRemapEnabled,
                .suppressionEnabled = suppressionEnabled,
                .gameplayInputAllowed = s_gameplayInputAllowed.load(std::memory_order_acquire),
                .menuInputActive = isInputBlockingMenuActive(),
                .weaponDrawn = s_weaponDrawn.load(std::memory_order_acquire),
                .rightHandHeldWeapon = s_rightHandHeldWeapon.load(std::memory_order_acquire),
                .primaryHandEvent = false,
                .equippedWeaponPrimaryDetachInputActive = s_equippedWeaponPrimaryDetachInputActive.load(std::memory_order_acquire),
                .equippedWeaponPrimaryDetached = s_equippedWeaponPrimaryDetached.load(std::memory_order_acquire),
                .eventMatched = eventMatched,
            };
        }

        [[nodiscard]] input_remap_policy::NativeActionSuppressionInput makeNativeActionSuppressionInput(
            bool suppressionEnabled, const RE::InputEvent* event, bool eventMatched)
        {
            auto input = makeNativeActionSuppressionInput(suppressionEnabled, eventMatched);
            input.primaryHandEvent = isPrimaryWandInputEvent(event);
            return input;
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

        [[nodiscard]] bool shouldSuppressNativeGripReloadAction(const RE::InputEvent* event)
        {
            return input_remap_policy::shouldSuppressNativeGripReloadAction(
                makeNativeActionSuppressionInput(g_rockConfig.rockSuppressRightGrabGameInput, event, eventNameMatches(event, kNativeEventWandGrip)));
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

        /*
         * The pipboy trigger rides the secondary (non-primary) wand, so the
         * suppression gate is the offhand's engagement state, not a fixed
         * left hand. Verified in the slot-11 processor: its open path only
         * accepts events whose controller id matches the secondary wand slot
         * at player+0x8D0 (primary sits at the already-verified +0x8CC).
         */
        [[nodiscard]] bool isPipboyHandEngaged()
        {
            const bool pipboyHandIsLeft = !f4vr::isLeftHandedMode();
            return s_handInteractionEngaged[pipboyHandIsLeft ? 0u : 1u].load(std::memory_order_acquire);
        }

        [[nodiscard]] bool shouldSuppressNativePipboyActionEvent(const RE::InputEvent* event)
        {
            auto input = makeNativeActionSuppressionInput(g_rockConfig.rockSuppressPipboyGameInputWhileHolding, eventNameMatches(event, kNativeEventPipboy));
            input.pipboyHandEngaged = isPipboyHandEngaged();
            return input_remap_policy::shouldSuppressNativePipboyAction(input);
        }

        [[nodiscard]] bool shouldSuppressNativeMeleeThrowAction(const RE::InputEvent* event)
        {
            // FO4VR's verified MeleeThrow handler accepts its grenade/throw action from WandGrip.
            return input_remap_policy::shouldSuppressNativeMeleeThrowAction(
                makeNativeActionSuppressionInput(g_rockConfig.rockSuppressNativeMeleeThrowGameInput, eventNameMatches(event, kNativeEventWandGrip)));
        }

        [[nodiscard]] bool shouldRoutePrimaryActivateReload(const RE::InputEvent* event)
        {
            const auto* button = event ? event->As<RE::ButtonEvent>() : nullptr;
            const bool eventMatched = isActivateReloadEvent(event);
            const bool primaryHandEvent = eventMatched && isPrimaryWandInputEvent(event);
            const bool virtualHolstersOwnsInput = primaryHandEvent &&
                                                  shouldDeferVirtualHolstersInput(f4vr::isLeftHandedMode(),
                                                      nativeEventButtonIdForVirtualHolsters(event),
                                                      g_rockConfig.rockVirtualHolstersDeferWeaponToggleInZone,
                                                      "primary activate reload");
            return input_remap_policy::shouldRoutePrimaryActivateReload(input_remap_policy::NativeActivateReloadInput{
                .remapEnabled = g_rockConfig.rockInputRemapEnabled,
                .gameplayInputAllowed = s_gameplayInputAllowed.load(std::memory_order_acquire),
                .menuInputActive = isInputBlockingMenuActive(),
                .weaponDrawn = s_weaponDrawn.load(std::memory_order_acquire),
                .primaryHandEvent = primaryHandEvent,
                .buttonJustPressed = button && button->QJustPressed(),
                .virtualHolstersOwnsInput = virtualHolstersOwnsInput,
                .eventMatched = eventMatched,
            });
        }

        [[nodiscard]] bool dispatchNativeReloadAction()
        {
            static REL::Relocation<void**> nativeActionDispatcherObject{ REL::Offset(kNativePlayerActionDispatcherDataOffset) };
            static REL::Relocation<NativeActionDispatcher_t> nativeActionDispatcher{ REL::Offset(kNativeActionDispatcherFunctionOffset) };

            auto* dispatcherObject = *nativeActionDispatcherObject;
            if (!dispatcherObject) {
                ROCK_LOG_SAMPLE_WARN(Input, g_rockConfig.rockLogSampleMilliseconds, "Cannot route primary activate to reload: native action dispatcher unavailable");
                return false;
            }

            return nativeActionDispatcher(dispatcherObject, kNativeReloadActionId, kNativeActionPriorityQueue);
        }

        void hookedReadyWeaponEventHandler(void* handler, RE::InputEvent* inputEvent, void* cursor, void* unk)
        {
            if (isAnyProviderOpenVrGameInputSuppressed()) {
                markInputEventStopped(inputEvent);
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed native ReadyWeapon input while provider OpenVR game-input suppression is active");
                return;
            }

            if (shouldSuppressNativeGripReadyAction(inputEvent)) {
                markInputEventStopped(inputEvent);
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed native WandGrip ReadyWeapon event while ROCK owns holstered right-grab input");
                return;
            }

            if (shouldSuppressNativeGripReloadAction(inputEvent)) {
                markInputEventStopped(inputEvent);
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed native primary WandGrip ReadyWeapon reload while ROCK routes reload to activate/use");
                return;
            }

            if (shouldSuppressNativeTriggerActionEvent(inputEvent)) {
                markInputEventStopped(inputEvent);
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed native WandTrigger ReadyWeapon event while ROCK owns holstered or held-weapon trigger input");
                return;
            }

            if (s_originalReadyWeaponEventHandler) {
                s_originalReadyWeaponEventHandler(handler, inputEvent, cursor, unk);
            }
        }

        void hookedActivateEventHandler(void* handler, RE::InputEvent* inputEvent, void* cursor, void* unk)
        {
            if (isAnyProviderOpenVrGameInputSuppressed()) {
                markInputEventStopped(inputEvent);
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed native Activate input while provider OpenVR game-input suppression is active");
                return;
            }

            if (shouldRoutePrimaryActivateReload(inputEvent)) {
                markInputEventStopped(inputEvent);
                if (dispatchNativeReloadAction()) {
                    ROCK_LOG_SAMPLE_DEBUG(Input,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Routed native primary activate/use input to equipped weapon reload");
                }
                return;
            }

            if (s_originalActivateEventHandler) {
                s_originalActivateEventHandler(handler, inputEvent, cursor, unk);
            }
        }

        void hookedFavoritesEventHandler(void* handler, RE::InputEvent* inputEvent)
        {
            if (isAnyProviderOpenVrGameInputSuppressed()) {
                markInputEventStopped(inputEvent);
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed native Favorites input while provider OpenVR game-input suppression is active");
                return;
            }

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
            if (isAnyProviderOpenVrGameInputSuppressed()) {
                markInputEventStopped(inputEvent);
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed native MeleeThrow input while provider OpenVR game-input suppression is active");
                return;
            }

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

        void hookedPipboyEventHandler(void* handler, RE::InputEvent* inputEvent)
        {
            if (isAnyProviderOpenVrGameInputSuppressed()) {
                markInputEventStopped(inputEvent);
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed native Pipboy input while provider OpenVR game-input suppression is active");
                return;
            }

            if (shouldSuppressNativePipboyActionEvent(inputEvent)) {
                markInputEventStopped(inputEvent);
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed native Pipboy open/light trigger event while the pipboy hand is engaged in a ROCK interaction");
                return;
            }

            if (s_originalPipboyEventHandler) {
                s_originalPipboyEventHandler(handler, inputEvent);
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

        bool installActivateEventReloadHook()
        {
            return installNativeActionVTableHook(kActivateHandlerHandleEventVTableSlotOffset,
                kActivateHandlerHandleEventFunctionOffset,
                &hookedActivateEventHandler,
                s_originalActivateEventHandler,
                s_activateEventHookInstalled,
                "ActivateHandler::HandleEvent reload remap");
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

        bool installPipboyEventSuppressionHook()
        {
            return installNativeActionVTableHook(kPipboyHandlerHandleEventVTableSlotOffset,
                kPipboyHandlerHandleEventFunctionOffset,
                &hookedPipboyEventHandler,
                s_originalPipboyEventHandler,
                s_pipboyEventHookInstalled,
                "PipboyHandler::HandleButtonEvent suppression");
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
            if (settings.enabled) {
                ready = installActivateEventReloadHook() && ready;
            }
            if (input_remap_policy::shouldInstallNativeActionSuppressionHook(settings.enabled, settings.suppressRightFavoritesGameInput)) {
                ready = installFavoritesEventSuppressionHook() && ready;
            }
            if (input_remap_policy::shouldInstallNativeActionSuppressionHook(settings.enabled, settings.suppressNativeMeleeThrowGameInput)) {
                ready = installMeleeThrowEventSuppressionHook() && ready;
            }
            if (input_remap_policy::shouldInstallNativeActionSuppressionHook(settings.enabled, settings.suppressPipboyGameInputWhileHolding)) {
                ready = installPipboyEventSuppressionHook() && ready;
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
            const auto rawPressed = tracker.rawPressed.load(std::memory_order_acquire);
            const bool rawHeld = (rawPressed & mask) != 0;

            if (isInputBlockingMenuActive()) {
                if (rawHeld) {
                    tracker.rearmPressedMask.fetch_or(mask, std::memory_order_acq_rel);
                }
                clearButtonEdges(tracker, mask);
                return result;
            }

            if ((tracker.rearmPressedMask.load(std::memory_order_acquire) & mask) != 0) {
                clearButtonEdges(tracker, mask);
                if (!rawHeld) {
                    tracker.rearmPressedMask.fetch_and(~mask, std::memory_order_acq_rel);
                }
                return result;
            }

            result.held = rawHeld;

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

    void setRightHandHeldWeapon(bool heldWeapon)
    {
        s_rightHandHeldWeapon.store(heldWeapon, std::memory_order_release);
    }

    void setHandInteractionEngaged(bool isLeft, bool engaged)
    {
        s_handInteractionEngaged[isLeft ? 0u : 1u].store(engaged, std::memory_order_release);
    }

    void setEquippedWeaponPrimaryDetachInputActive(bool active)
    {
        s_equippedWeaponPrimaryDetachInputActive.store(active, std::memory_order_release);
    }

    void setEquippedWeaponPrimaryDetached(bool detached)
    {
        s_equippedWeaponPrimaryDetached.store(detached, std::memory_order_release);
    }

    void setProviderOpenVrGameInputSuppressed(bool isLeft, bool suppressed)
    {
        s_providerOpenVrGameInputSuppressed[isLeft ? 0u : 1u].store(suppressed, std::memory_order_release);
    }

    bool isMenuInputActive()
    {
        return isInputBlockingMenuActive();
    }

    bool shouldDeferGrabInputForVirtualHolsters(bool isLeft, int buttonId)
    {
        return shouldDeferVirtualHolstersInput(isLeft,
            buttonId,
            g_rockConfig.rockVirtualHolstersDeferGrabInZone,
            "grab");
    }

    bool shouldSuppressNativeTriggerAction(const RE::InputEvent* event)
    {
        return shouldSuppressNativeTriggerActionEvent(event);
    }

    bool isNativePipboyInputSuppressionActive()
    {
        // Mirrors hookedPipboyEventHandler for a matched "Pipboy" event so API consumers see the live hook decision.
        if (isAnyProviderOpenVrGameInputSuppressed()) {
            return true;
        }

        auto input = makeNativeActionSuppressionInput(g_rockConfig.rockSuppressPipboyGameInputWhileHolding, true);
        input.pipboyHandEngaged = isPipboyHandEngaged();
        return input_remap_policy::shouldSuppressNativePipboyAction(input);
    }

    void processPendingWeaponToggleRequests()
    {
        refreshWeaponToggleControllerTracker();

        const std::uint32_t requestCount = s_pendingWeaponToggleRequests.exchange(0, std::memory_order_acq_rel);
        if (requestCount == 0) {
            return;
        }

        if (!g_rockConfig.rockInputRemapEnabled || !s_gameplayInputAllowed.load(std::memory_order_acquire) || isInputBlockingMenuActive()) {
            ROCK_LOG_DEBUG(Input, "Dropped {} pending weapon toggle request(s) because gameplay input is not active", requestCount);
            return;
        }

        if (shouldDeferWeaponToggleForVirtualHolsters()) {
            ROCK_LOG_DEBUG(Input, "Dropped {} pending weapon toggle request(s) because VirtualHolsters owns the active holster zone", requestCount);
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
