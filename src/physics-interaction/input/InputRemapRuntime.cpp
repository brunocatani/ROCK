#include "physics-interaction/input/InputRemapRuntime.h"

#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/ManualScopeInputPolicy.h"
#include "physics-interaction/input/NativeVatsInputSuppressionPolicy.h"
#include "physics-interaction/input/PipboyPauseGesturePolicy.h"
#include "physics-interaction/object/FarSelectionBlacklistPolicy.h"
#include "physics-interaction/PhysicsLog.h"
#include "RockConfig.h"

#include "api/ROCKProviderApiInternal.h"
#include "api/FRIKApi.h"

#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/BSLock.h"
#include "RE/Bethesda/BSPointerHandle.h"
#include "RE/Bethesda/ControlMap.h"
#include "RE/Bethesda/InputEvent.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Bethesda/UI.h"

#include <F4SE/F4SE.h>
#include <REL/Relocation.h>
#include "rock_support/VRControllers.h"
#include <windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <intrin.h>
#include <optional>
#include <string_view>

namespace rock::input_remap_runtime
{
    namespace
    {
        constexpr std::size_t kGetControllerStateVTableIndex = 34;
        constexpr std::size_t kGetControllerStateWithPoseVTableIndex = 35;
        // TriggerHapticPulse has immediately followed GetControllerStateWithPose
        // in every IVRSystem revision that has the state getters; the two
        // adjacent indices above are proven live in-game.
        constexpr std::size_t kTriggerHapticPulseVTableIndex = 36;
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
         * MenuOpenHandler is the MenuControls handler that owns the semantic
         * "Pause" event. FO4VR maps OpenVR button 1 (Quest 2 Y on the
         * secondary wand) to Pause. Its slot-11 processor opens Pause from a
         * released ButtonEvent after verifying the secondary-wand device and
         * all native menu/player gates. Verified 2026-07-21 from the
         * MenuControls constructor (RVA 0x1323210), handler construction/list
         * order (RVA 0x1323870), and processor (RVA 0x1326760).
         */
        constexpr std::uintptr_t kMenuOpenHandlerHandleEventFunctionOffset = 0x1326760;
        constexpr std::uintptr_t kMenuOpenHandlerHandleEventVTableSlotOffset = 0x2DCC850;
        /*
         * MenuOpenHandler calls this helper only after its semantic Pause,
         * player/menu, and primary-wand VATS eligibility gates. The helper's
         * button-down path starts V.A.N.S. at Bethesda's hold threshold; its
         * release path opens ordinary VATS. There is one executable caller.
         * Verified 2026-07-27 in FO4VR 1.2.72 at callsite RVA 0x1326990 and
         * helper RVA 0xBEB280.
         */
        constexpr std::uintptr_t kNativeVatsVansDecisionFunctionOffset = 0x0BEB280;
        constexpr std::uintptr_t kNativeVatsVansDecisionCallSiteOffset = 0x1326990;
        constexpr std::uintptr_t kMenuControlsSingletonOffset = 0x5A3B888;
        constexpr std::ptrdiff_t kMenuControlsPipboyHandlerOffset = 0x68;
        constexpr std::uintptr_t kPipboyHandlerVTableOffset = 0x2DCC778;
        /*
         * PipboyHandler (BSInputEventUser in the MenuControls chain, vtable
         * 0x2DCC778) processes the pipboy-hand trigger in vtable slot 11
         * (0x2DCC7D0): press starts hold tracking, holding past the game
         * threshold toggles the pipboy light, release opens the Pip-Boy.
         * Verified 2026-07-04 from raw disassembly of the constructor
         * (0x1325090 region) and the slot-11 processor (0x1326D90); the same
         * function independently re-uses the already-verified player global
         * (0x5B043F0), device-to-controller-id converter (0x1BA6ED0), and
         * action dispatcher data (0x5A3B8A0). The VR wand trigger reaches the
         * handler as user event "WandTrigger" from BOTH wands (observed live
         * 2026-07-04 via the hook trace; the handler filters to the secondary
         * wand at player+0x8D0 internally), while "Pipboy" is the flat/
         * gamepad direct binding. ShouldHandleEvent (slot 1) additionally
         * accepts "Pause" and the Quick* tab events, which the hook must
         * never swallow.
         */
        constexpr std::uintptr_t kPipboyHandlerHandleEventFunctionOffset = 0x1326D90;
        constexpr std::uintptr_t kPipboyHandlerHandleEventVTableSlotOffset = 0x2DCC7D0;
        /*
         * PipboyLightHandler (PlayerControls family, vtable 0x2D8A248) owns
         * the VR flashlight: its HandleEvent at slot offset +0x58 fires the
         * light toggle (0xDAF090 on the global at 0x5B279E0) once per hold
         * when heldDownSecs passes the same threshold global (0x3844EA0) the
         * flat path uses, latched by this+0x28 until release. Verified
         * 2026-07-04 from raw disassembly after live traces showed the
         * PipboyHandler hook suppressing opens while the light still fired -
         * the light-on-hold path inside PipboyHandler is flat-game only.
         * The function checks neither event name nor device, so the hook
         * applies the shared pipboy suppression policy (secondary-wand
         * trigger + engaged hand) before chaining.
         */
        constexpr std::uintptr_t kPipboyLightHandlerHandleEventFunctionOffset = 0x0FC9170;
        constexpr std::uintptr_t kPipboyLightHandlerHandleEventVTableSlotOffset = 0x2D8A2A0;
        /*
         * FO4VR's ActivateHandler resolves the wand's current "pick ref" (what it is pointing
         * at/reaching for) from one of these two per-wand handle globals before dispatching
         * Activate/WandAccept - there is no separate "Take" input action; Take vs Talk/Open/
         * Search/Read is decided later, per-FormType, on whichever ref this handle names.
         * Verified 2026-07-05 from raw disassembly of two independent functions
         * (FUN_140FCF360/FUN_140FCB5B0 RVA, both reached from the already-hooked
         * ActivateHandler::HandleEvent at 0xFC7F00) that each independently perform the same
         * primary/secondary wand selection over these globals, plus an independently-named
         * Ghidra xref symbol ("ViewCasterPrimaryWand") on the primary global's writer that
         * corroborates it as the wand raycast/pick target rather than a decompiler artifact.
         * Values below are RVA (Ghidra VA 0x145AC72B0/0x145AC7F10 minus the 0x140000000 image
         * base, matching every other offset in this file). Each global holds an 8-byte pointer
         * to the wand's caster/view object, NOT a bare handle - the real ObjectRefHandle is
         * behind a locked virtual call on that caster (see readWandPickRefHandle()). An earlier
         * build treated the global's raw memory as the handle directly and silently never
         * resolved anything; corrected 2026-07-05 - see
         * docs/docs/reverse-engineering/2026-07-05-take-equip-activate-classification-ghidra-findings.md.
         */
        constexpr std::uintptr_t kActivatePrimaryWandPickRefGlobalOffset = 0x5AC72B0;
        constexpr std::uintptr_t kActivateSecondaryWandPickRefGlobalOffset = 0x5AC7F10;
        /*
         * The wand caster object's handle accessor, verified 2026-07-05 via raw disassembly of
         * FUN_1409D0CE0 (RVA 0x9D0CE0): lock a spinlock at caster+0x70, call the vtable
         * function at byte offset 0x10 (slot index 2, 8-byte slots) with the caster as `this`,
         * read the first 4 bytes of the CALL'S RETURN VALUE (not the caster) as the handle,
         * unlock. Cross-checked against a second, already-verified caller in the native action
         * dispatcher at 0xFC07E0.
         */
        using WandPickRefCasterAccessor_t = void* (*)(void*);
        constexpr std::ptrdiff_t kWandPickRefCasterSpinLockOffset = 0x70;
        constexpr std::size_t kWandPickRefCasterVTableSlot = 2;
        constexpr std::uintptr_t kNativeActionDispatcherFunctionOffset = 0x0FC07E0;
        constexpr std::uintptr_t kNativeInputDeviceToControllerIdFunctionOffset = 0x1BA6ED0;
        constexpr std::uintptr_t kNativePlayerActionDispatcherDataOffset = 0x5A3B8A0;
        constexpr std::uintptr_t kNativePlayerDataOffset = 0x5B043F0;
        constexpr std::ptrdiff_t kNativePrimaryWandDeviceIdOffset = 0x8CC;
        constexpr std::ptrdiff_t kNativeSecondaryWandDeviceIdOffset = 0x8D0;
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
        constexpr std::string_view kNativeEventPause{ "Pause" };

        using GetControllerState_t = bool (*)(vr::IVRSystem*, vr::TrackedDeviceIndex_t, vr::VRControllerState_t*, std::uint32_t);
        using GetControllerStateWithPose_t =
            bool (*)(vr::IVRSystem*, vr::ETrackingUniverseOrigin, vr::TrackedDeviceIndex_t, vr::VRControllerState_t*, std::uint32_t, vr::TrackedDevicePose_t*);
        using TriggerHapticPulse_t = void (*)(vr::IVRSystem*, vr::TrackedDeviceIndex_t, std::uint32_t, unsigned short);
        using NativeInputEventHandler_t = void (*)(void*, RE::InputEvent*, void*, void*);
        using NativeActionDispatcher_t = bool (*)(void*, int, std::uint32_t);
        using NativeInputDeviceToControllerId_t = std::int32_t (*)(std::int32_t);
        using FavoritesInputEventHandler_t = void (*)(void*, RE::InputEvent*);
        // Verified PipboyHandler slot-11 signature: (this, event) only; no cursor/unk tail like the PlayerControls handlers.
        using PipboyInputEventHandler_t = void (*)(void*, RE::InputEvent*);
        using MenuOpenInputEventHandler_t = void (*)(void*, RE::InputEvent*);
        using NativeVatsVansDecision_t = void (*)(RE::ButtonEvent*);

        struct ControllerTracker
        {
            std::atomic<std::uint64_t> rawPressed{ 0 };
            std::atomic<std::uint64_t> rawTouched{ 0 };
            std::atomic<std::uint64_t> pressedEdges{ 0 };
            std::atomic<std::uint64_t> releasedEdges{ 0 };
            std::atomic<std::uint64_t> rearmPressedMask{ 0 };
            // Last raw analog trigger (Axis1) sample; feeds the left-hand
            // fire remap which presents it on the other wand's state.
            std::atomic<float> triggerAxisX{ 0.0f };
            std::atomic<float> triggerAxisY{ 0.0f };
            std::atomic<std::uint64_t> sampleSequence{ 0 };
            std::atomic<std::uint64_t> sampleTickMilliseconds{ 0 };
            std::atomic<bool> valid{ false };
        };

        std::atomic<std::uint64_t> s_nextControllerSampleSequence{ 1 };

        std::array<ControllerTracker, 2> s_controllers;
        std::atomic<bool> s_gameplayInputAllowed{ false };
        std::atomic<bool> s_weaponDrawn{ false };
        std::array<std::atomic<bool>, 2> s_handHeldWeapon{};
        std::array<std::atomic<bool>, 2> s_handInteractionEngaged{};
        std::array<std::atomic<std::uint32_t>, 2> s_heldObjectFormId{};
        std::array<std::atomic<bool>, 2> s_pendingSavedGrabOffsetRequest{};
        std::atomic<bool> s_equippedWeaponFiringGripInputActive{ false };
        std::atomic<bool> s_equippedWeaponPrimaryDetached{ false };
        std::atomic<bool> s_equippedWeaponLeftHandFiringActive{ false };
        manual_scope_input_policy::RuntimeState s_manualScopeInputState{};
        // MenuControls dispatches ButtonEvents serially on the frame/input
        // thread; these gesture states are never read from worker callbacks.
        pipboy_pause_gesture_policy::RuntimeState s_pipboyPauseGestureState{};
        native_vats_input_suppression_policy::RuntimeState s_nativeVatsInputSuppressionState{};
        std::atomic<bool> s_manualScopeActivationRequested{ false };
        std::atomic<bool> s_hooksInstalled{ false };
        std::atomic<bool> s_readyWeaponEventHookInstalled{ false };
        std::atomic<bool> s_activateEventHookInstalled{ false };
        std::atomic<bool> s_favoritesEventHookInstalled{ false };
        std::atomic<bool> s_meleeThrowEventHookInstalled{ false };
        std::atomic<bool> s_pipboyEventHookInstalled{ false };
        std::atomic<bool> s_pipboyLightEventHookInstalled{ false };
        std::atomic<bool> s_menuOpenEventHookInstalled{ false };
        std::atomic<bool> s_nativeVatsVansDecisionHookInstalled{ false };
        std::atomic<bool> s_nativeVatsVansDecisionHookInstallFailed{ false };
        std::atomic<bool> s_meleeThrowFallbackPatchesApplied{ false };
        std::atomic<bool> s_menuInputGateRegistered{ false };
        std::atomic<bool> s_menuInputActive{ false };
        std::atomic<bool> s_pipboyMenuOpen{ false };
        std::atomic<std::uint32_t> s_pipboyMenuGeneration{ 1 };
        // High 32 bits = Pip-Boy menu generation, low 32 bits =
        // GetTickCount64() milliseconds. Zero means no pending evidence.
        std::array<std::atomic<std::uint64_t>, 2> s_pipboyTriggerTransitions{};
        std::atomic<bool> s_missingVRSystemLogged{ false };
        std::atomic<bool> s_missingUILogged{ false };
        std::array<std::atomic<bool>, 2> s_providerOpenVrGameInputSuppressed{};
        void** s_vrSystemVTable = nullptr;
        GetControllerState_t s_originalGetControllerState = nullptr;
        GetControllerStateWithPose_t s_originalGetControllerStateWithPose = nullptr;
        TriggerHapticPulse_t s_originalTriggerHapticPulse = nullptr;
        NativeInputEventHandler_t s_originalReadyWeaponEventHandler = nullptr;
        NativeInputEventHandler_t s_originalActivateEventHandler = nullptr;
        NativeInputEventHandler_t s_originalMeleeThrowEventHandler = nullptr;
        FavoritesInputEventHandler_t s_originalFavoritesEventHandler = nullptr;
        PipboyInputEventHandler_t s_originalPipboyEventHandler = nullptr;
        NativeInputEventHandler_t s_originalPipboyLightEventHandler = nullptr;
        MenuOpenInputEventHandler_t s_originalMenuOpenEventHandler = nullptr;
        NativeVatsVansDecision_t s_originalNativeVatsVansDecision = nullptr;

        void blockManualScopeInputUntilRelease()
        {
            manual_scope_input_policy::blockUntilRelease(s_manualScopeInputState);
            s_manualScopeActivationRequested.store(false, std::memory_order_release);
        }

        void resetManualScopeInput()
        {
            manual_scope_input_policy::reset(s_manualScopeInputState);
            s_manualScopeActivationRequested.store(false, std::memory_order_release);
        }

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

        void publishPipboyMenuOpen(const bool open)
        {
            const bool wasOpen = s_pipboyMenuOpen.exchange(open, std::memory_order_acq_rel);
            if (open && !wasOpen) {
                s_pipboyMenuGeneration.fetch_add(1, std::memory_order_acq_rel);
            }
            if (open != wasOpen) {
                for (auto& token : s_pipboyTriggerTransitions) {
                    token.store(0, std::memory_order_release);
                }
            }
        }

        void publishPipboyTriggerTransition(const input_remap_policy::Hand hand)
        {
            if (!s_pipboyMenuOpen.load(std::memory_order_acquire)) {
                return;
            }
            const std::uint32_t generation = s_pipboyMenuGeneration.load(std::memory_order_acquire);
            const std::uint32_t tick = static_cast<std::uint32_t>(GetTickCount64());
            const std::uint64_t packed = (static_cast<std::uint64_t>(generation) << 32u) | tick;
            const std::size_t index = hand == input_remap_policy::Hand::Left ? 0u : 1u;
            s_pipboyTriggerTransitions[index].store(packed, std::memory_order_release);
        }

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

            publishPipboyMenuOpen(ui.GetMenuOpen(RE::BSFixedString{ "PipboyMenu" }));

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
                if (event.menuName == "PipboyMenu") {
                    publishPipboyMenuOpen(event.opening);
                }
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

        [[nodiscard]] std::uint32_t currentProviderHandInputSuppressionFlagsAtDispatch()
        {
            return provider::currentHandInputSuppressionFlagsV1(
                       provider::RockProviderHand::Right) |
                   provider::currentHandInputSuppressionFlagsV1(
                       provider::RockProviderHand::Left);
        }

        [[nodiscard]] bool isAnyProviderOpenVrGameInputSuppressedAtDispatch()
        {
            if (isAnyProviderOpenVrGameInputSuppressed()) {
                return true;
            }

            return provider::hasHandInputSuppressionFlagV1(
                currentProviderHandInputSuppressionFlagsAtDispatch(),
                provider::RockProviderHandInputSuppressionFlagV1::
                    SuppressOpenVrGameInput);
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
             *
             * hFRIK is deliberately NOT in this list: it must keep seeing the
             * left-wand trigger blanking (its own Pip-Boy/gesture logic acts
             * on the left trigger, which fires the weapon while left-firing).
             * hFRIK only gets the cross-map skipped - see the includeCrossMap
             * flavor of applyLeftHandFireTriggerRemapForGame.
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

        [[nodiscard]] bool isInputBlockingMenuActive();

        /*
         * Left-hand fire: while the LEFT hand occupies the equipped weapon's
         * firing grip, the game must fire from the LEFT physical trigger. The
         * game reads fire from the primary (right) wand's trigger button bit
         * and analog Axis1, so ROCK presents the left trigger there and blanks
         * the trigger on both physical identities: the right physical trigger
         * goes inert (support hand) and the left wand's own trigger reads
         * empty (no Pip-Boy/flashlight side effects). ROCK's internal reads
         * are captured pre-remap and stay physical.
         */
        [[nodiscard]] bool shouldRemapLeftHandFireTriggerForGame()
        {
            return s_equippedWeaponLeftHandFiringActive.load(std::memory_order_acquire) &&
                g_rockConfig.rockInputRemapEnabled &&
                s_gameplayInputAllowed.load(std::memory_order_acquire) &&
                !isInputBlockingMenuActive();
        }

        /*
         * Applied uniformly to every non-configurator caller: a caller-module
         * split (skip the cross-map for FRIK.dll polls) broke firing in-game
         * - the game's effective fire input is fed through polls the
         * return-address heuristic attributed to FRIK, so input flow must
         * never fork by caller. The free right hand's phantom trigger curl is
         * instead fixed at its consumer: hFRIK's dynamic hand pose ignores
         * the trigger-tracked finger bones on the free right hand while the
         * weapon-node ownership block (left-firing) is active.
         */
        void applyLeftHandFireTriggerRemapForGame(input_remap_policy::Hand hand, vr::VRControllerState_t* state, std::uint32_t stateSize)
        {
            if (!state || stateSize < sizeof(vr::VRControllerState_t)) {
                return;
            }

            constexpr std::uint64_t triggerButtonMask = 1ull << input_remap_policy::kOpenVrSteamVrTriggerButtonId;
            constexpr std::size_t triggerAxisIndex =
                static_cast<std::size_t>(input_remap_policy::kOpenVrSteamVrTriggerButtonId - input_remap_policy::kOpenVrAxisButtonBase);

            if (hand == input_remap_policy::Hand::Right) {
                const auto& leftTracker = s_controllers[controllerIndex(input_remap_policy::Hand::Left)];
                const bool leftValid = leftTracker.valid.load(std::memory_order_acquire);
                const std::uint64_t leftPressed = leftValid ? leftTracker.rawPressed.load(std::memory_order_acquire) : 0;
                const std::uint64_t leftTouched = leftValid ? leftTracker.rawTouched.load(std::memory_order_acquire) : 0;
                state->ulButtonPressed = (state->ulButtonPressed & ~triggerButtonMask) | (leftPressed & triggerButtonMask);
                state->ulButtonTouched = (state->ulButtonTouched & ~triggerButtonMask) | (leftTouched & triggerButtonMask);
                state->rAxis[triggerAxisIndex].x = leftValid ? leftTracker.triggerAxisX.load(std::memory_order_acquire) : 0.0f;
                state->rAxis[triggerAxisIndex].y = leftValid ? leftTracker.triggerAxisY.load(std::memory_order_acquire) : 0.0f;
            } else {
                state->ulButtonPressed &= ~triggerButtonMask;
                state->ulButtonTouched &= ~triggerButtonMask;
                state->rAxis[triggerAxisIndex].x = 0.0f;
                state->rAxis[triggerAxisIndex].y = 0.0f;
            }
        }

        [[nodiscard]] input_remap_policy::Settings makeSettings()
        {
            return input_remap_policy::Settings{
                .enabled = g_rockConfig.rockInputRemapEnabled,
                .grabButtonId = g_rockConfig.rockGrabButtonID,
                .suppressRightGrabGameInput = g_rockConfig.rockSuppressRightGrabGameInput,
                .suppressRightFavoritesGameInput = g_rockConfig.rockSuppressRightFavoritesGameInput,
                .suppressRightTriggerGameInput = g_rockConfig.rockSuppressNativeReadyWeaponAutoReady,
                .suppressNativeMeleeThrowGameInput = g_rockConfig.rockSuppressNativeMeleeThrowGameInput,
                .suppressPipboyGameInputWhileHolding = g_rockConfig.rockSuppressPipboyGameInputWhileHolding,
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

        [[nodiscard]] bool isInputBlockingMenuActive()
        {
            return isGameStoppingMenuInputActive() || isCompatibilityConfigInputActive();
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
            constexpr std::size_t triggerAxisIndex =
                static_cast<std::size_t>(input_remap_policy::kOpenVrSteamVrTriggerButtonId - input_remap_policy::kOpenVrAxisButtonBase);
            tracker.triggerAxisX.store(state->rAxis[triggerAxisIndex].x, std::memory_order_release);
            tracker.triggerAxisY.store(state->rAxis[triggerAxisIndex].y, std::memory_order_release);
            tracker.sampleTickMilliseconds.store(
                GetTickCount64(),
                std::memory_order_release);
            tracker.sampleSequence.store(
                s_nextControllerSampleSequence.fetch_add(
                    1,
                    std::memory_order_acq_rel),
                std::memory_order_release);

            const bool hadPrevious = tracker.valid.exchange(true, std::memory_order_acq_rel);
            const std::uint64_t previousRawPressed = tracker.rawPressed.exchange(rawPressed, std::memory_order_acq_rel);
            tracker.rawTouched.store(rawTouched, std::memory_order_release);

            const auto rawTransition = input_remap_policy::evaluateEdgeTransition(hadPrevious, previousRawPressed, rawPressed);
            if (hadPrevious) {
                tracker.pressedEdges.fetch_or(rawTransition.pressedEdges, std::memory_order_acq_rel);
                tracker.releasedEdges.fetch_or(rawTransition.releasedEdges, std::memory_order_acq_rel);
                const auto triggerMask = input_remap_policy::buttonMask(input_remap_policy::kOpenVrSteamVrTriggerButtonId);
                if (((rawTransition.pressedEdges | rawTransition.releasedEdges) & triggerMask) != 0) {
                    publishPipboyTriggerTransition(hand);
                }
            }

            const bool inputBlockingMenuActive = isInputBlockingMenuActive();
            if (inputBlockingMenuActive) {
                tracker.rearmPressedMask.fetch_or(rawPressed, std::memory_order_acq_rel);
                /*
                 * Gameplay edges must not originate inside a blocking menu.
                 * The accumulator kept latching press edges during menus; a
                 * grab squeezed at the workbench and still held on exit kept
                 * its latched press (the rearm clear only fires on release),
                 * so the first post-menu frame read held+pressed and started
                 * a phantom primary-only grip mid weapon reassembly - the
                 * "weapon invisible until swap" report. Rearm semantics stay:
                 * a button held through the menu needs a full release and a
                 * fresh gameplay press to act again.
                 */
                clearButtonEdges(tracker, ~0ull);
            } else {
                const auto rearmMask = tracker.rearmPressedMask.load(std::memory_order_acquire);
                const auto releasedFromRearm = rearmMask & ~rawPressed;
                if (releasedFromRearm != 0) {
                    clearButtonEdges(tracker, releasedFromRearm);
                    tracker.rearmPressedMask.fetch_and(~releasedFromRearm, std::memory_order_acq_rel);
                }
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
                    if (shouldRemapLeftHandFireTriggerForGame() &&
                        !shouldBypassProviderOpenVrGameInputSuppression(callerAddress)) {
                        applyLeftHandFireTriggerRemapForGame(hand, controllerState, controllerStateSize);
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
                    if (shouldRemapLeftHandFireTriggerForGame() &&
                        !shouldBypassProviderOpenVrGameInputSuppression(callerAddress)) {
                        applyLeftHandFireTriggerRemapForGame(hand, controllerState, controllerStateSize);
                    }
                }
            }
            return result;
        }

        /*
         * While the LEFT hand fires, the game believes the RIGHT wand fired
         * and sends its haptic pulses (weapon fire rumble) there. Retarget
         * game-originated pulses aimed at the RIGHT wand to the LEFT wand so
         * the rumble lands in the hand actually holding the weapon. ROCK,
         * FRIK and the configurator address PHYSICAL hands with their own
         * pulses and pass through untouched.
         */
        void hookedTriggerHapticPulse(vr::IVRSystem* system, vr::TrackedDeviceIndex_t controllerDeviceIndex, std::uint32_t axisId, unsigned short durationMicroSec)
        {
            const void* callerAddress = _ReturnAddress();
            vr::TrackedDeviceIndex_t targetDeviceIndex = controllerDeviceIndex;
            if (shouldRemapLeftHandFireTriggerForGame() &&
                !isCallerModule(callerAddress, L"ROCK.dll") &&
                !isCallerModule(callerAddress, L"FRIK.dll") &&
                !shouldBypassProviderOpenVrGameInputSuppression(callerAddress)) {
                input_remap_policy::Hand hand{};
                if (resolveControllerHand(controllerDeviceIndex, hand) && hand == input_remap_policy::Hand::Right) {
                    const auto leftIndex = system ?
                        system->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_LeftHand) :
                        vr::k_unTrackedDeviceIndexInvalid;
                    if (leftIndex != vr::k_unTrackedDeviceIndexInvalid) {
                        targetDeviceIndex = leftIndex;
                    }
                }
            }
            if (s_originalTriggerHapticPulse) {
                s_originalTriggerHapticPulse(system, targetDeviceIndex, axisId, durationMicroSec);
            }
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

            /*
             * BSFixedString pools intern case-insensitively and the native
             * handlers compare interned pointers, so the case stored in the
             * event depends on which spelling entered the pool first (the
             * binary carries both "Pipboy" and "PipBoy" literals). Match with
             * the same case-insensitive semantics the engine uses.
             */
            const auto& userEvent = event->QUserEvent();
            const auto* userEventText = userEvent.c_str();
            const std::string_view name{ userEventText ? userEventText : "", userEvent.length() };
            return name.length() == expected.length() && _strnicmp(name.data(), expected.data(), expected.length()) == 0;
        }

        enum class NativeWandIdentity : std::uint8_t
        {
            Unknown,
            Primary,
            Secondary,
        };

        [[nodiscard]] NativeWandIdentity resolveNativeWandIdentity(const RE::InputEvent* event)
        {
            if (!event) {
                return NativeWandIdentity::Unknown;
            }

            static REL::Relocation<NativeInputDeviceToControllerId_t> nativeDeviceToControllerId{ REL::Offset(kNativeInputDeviceToControllerIdFunctionOffset) };
            static REL::Relocation<void**> nativePlayer{ REL::Offset(kNativePlayerDataOffset) };

            auto* player = *nativePlayer;
            if (!player) {
                return NativeWandIdentity::Unknown;
            }

            const auto controllerId = nativeDeviceToControllerId(event->deviceID);
            const auto primaryWandDeviceId = *reinterpret_cast<const std::int32_t*>(reinterpret_cast<std::uintptr_t>(player) + kNativePrimaryWandDeviceIdOffset);
            if (controllerId == primaryWandDeviceId) {
                return NativeWandIdentity::Primary;
            }

            const auto secondaryWandDeviceId = *reinterpret_cast<const std::int32_t*>(reinterpret_cast<std::uintptr_t>(player) + kNativeSecondaryWandDeviceIdOffset);
            return controllerId == secondaryWandDeviceId ? NativeWandIdentity::Secondary : NativeWandIdentity::Unknown;
        }

        [[nodiscard]] bool isPrimaryWandInputEvent(const RE::InputEvent* event)
        {
            return resolveNativeWandIdentity(event) == NativeWandIdentity::Primary;
        }

        [[nodiscard]] bool isSecondaryWandInputEvent(const RE::InputEvent* event)
        {
            return resolveNativeWandIdentity(event) == NativeWandIdentity::Secondary;
        }

        [[nodiscard]] bool isActivateReloadEvent(const RE::InputEvent* event)
        {
            return eventNameMatches(event, kNativeEventActivate) || eventNameMatches(event, kNativeEventWandAccept);
        }

        [[nodiscard]] input_remap_policy::NativeActionSuppressionInput makeNativeActionSuppressionInput(bool suppressionEnabled, bool eventMatched)
        {
            return input_remap_policy::NativeActionSuppressionInput{
                .remapEnabled = g_rockConfig.rockInputRemapEnabled,
                .suppressionEnabled = suppressionEnabled,
                .gameplayInputAllowed = s_gameplayInputAllowed.load(std::memory_order_acquire),
                .menuInputActive = isInputBlockingMenuActive(),
                .weaponDrawn = s_weaponDrawn.load(std::memory_order_acquire),
                .eventHandHeldWeapon = false,
                .primaryHandEvent = false,
                .equippedWeaponFiringGripInputActive = s_equippedWeaponFiringGripInputActive.load(std::memory_order_acquire),
                .equippedWeaponPrimaryDetached = s_equippedWeaponPrimaryDetached.load(std::memory_order_acquire),
                .eventMatched = eventMatched,
            };
        }

        [[nodiscard]] input_remap_policy::NativeActionSuppressionInput makeNativeActionSuppressionInput(
            bool suppressionEnabled, const RE::InputEvent* event, bool eventMatched)
        {
            auto input = makeNativeActionSuppressionInput(suppressionEnabled, eventMatched);
            input.primaryHandEvent = isPrimaryWandInputEvent(event);
            // ROCK owns controller identity: FO4VR's primary wand is the
            // physical right controller and the secondary wand is the
            // physical left controller. The game's native handedness setting
            // must never remap ROCK input ownership.
            const bool eventHandIsLeft = !input.primaryHandEvent;
            input.eventHandHeldWeapon = s_handHeldWeapon[eventHandIsLeft ? 0u : 1u].load(std::memory_order_acquire);
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
                makeNativeActionSuppressionInput(
                    g_rockConfig.rockSuppressNativeReadyWeaponAutoReady,
                    event,
                    eventNameMatches(event, kNativeEventWandTrigger)));
        }

        /*
         * The Pip-Boy trigger rides ROCK's physical-left secondary wand, so
         * the suppression gate is the left hand's engagement state. Verified
         * in the slot-11 processor: its open path only
         * accepts events whose controller id matches the secondary wand slot
         * at player+0x8D0 (primary sits at the already-verified +0x8CC).
         */
        [[nodiscard]] bool isPipboyHandEngaged()
        {
            constexpr auto index = 0u;
            // All three publications describe the same ownership boundary but
            // are refreshed by different transition paths. Treat any durable
            // witness as engaged so a just-committed left grab cannot expose a
            // one-frame Pip-Boy-open window before the aggregate flag catches
            // up.
            return s_handInteractionEngaged[index].load(std::memory_order_acquire) ||
                   s_handHeldWeapon[index].load(std::memory_order_acquire) ||
                   s_heldObjectFormId[index].load(std::memory_order_acquire) != 0u;
        }

        [[nodiscard]] bool shouldSuppressNativePipboyActionEvent(const RE::InputEvent* event)
        {
            // VR wand triggers arrive as "WandTrigger"; "Pipboy" covers flat/gamepad direct bindings.
            const bool eventMatched = eventNameMatches(event, kNativeEventWandTrigger) || eventNameMatches(event, kNativeEventPipboy);
            auto input = makeNativeActionSuppressionInput(g_rockConfig.rockSuppressPipboyGameInputWhileHolding, event, eventMatched);
            input.pipboyHandEngaged = isPipboyHandEngaged();
            return input_remap_policy::shouldSuppressNativePipboyAction(input);
        }

        [[nodiscard]] bool shouldSuppressLegacyPipboyTriggerOpenEvent(const RE::InputEvent* event)
        {
            return input_remap_policy::shouldSuppressLegacyPipboyTriggerOpen(input_remap_policy::LegacyPipboyTriggerOpenInput{
                .remapEnabled = g_rockConfig.rockInputRemapEnabled,
                .gameplayInputAllowed = s_gameplayInputAllowed.load(std::memory_order_acquire),
                .menuInputActive = isInputBlockingMenuActive(),
                .eventMatched = eventNameMatches(event, kNativeEventWandTrigger),
                .secondaryWandEvent = isSecondaryWandInputEvent(event),
            });
        }

        [[nodiscard]] bool shouldSuppressNativeMeleeThrowAction(const RE::InputEvent* event)
        {
            // FO4VR's verified MeleeThrow handler accepts its grenade/throw action from WandGrip.
            return input_remap_policy::shouldSuppressNativeMeleeThrowAction(
                makeNativeActionSuppressionInput(g_rockConfig.rockSuppressNativeMeleeThrowGameInput, eventNameMatches(event, kNativeEventWandGrip)));
        }

        [[nodiscard]] bool shouldRouteFiringHandActivateReload(const RE::InputEvent* event)
        {
            const auto* button = event ? event->As<RE::ButtonEvent>() : nullptr;
            const bool eventMatched = isActivateReloadEvent(event);
            /*
             * This handler only ever receives PRIMARY-wand Activate/WandAccept
             * (live-verified 2026-07-12; the secondary wand's accept button
             * never produces an event here), so the event's physical hand is
             * the primary-wand hand by identity.
             */
            const bool primaryHandEvent = eventMatched && isPrimaryWandInputEvent(event);
            constexpr bool primaryHandIsLeft = false;
            const bool firingHandIsLeft = s_equippedWeaponLeftHandFiringActive.load(std::memory_order_acquire);
            const bool firingHandIsPrimaryHand = firingHandIsLeft == primaryHandIsLeft;
            const bool route = input_remap_policy::shouldRouteFiringHandActivateReload(input_remap_policy::NativeActivateReloadInput{
                .remapEnabled = g_rockConfig.rockInputRemapEnabled,
                .gameplayInputAllowed = s_gameplayInputAllowed.load(std::memory_order_acquire),
                .menuInputActive = isInputBlockingMenuActive(),
                .weaponDrawn = s_weaponDrawn.load(std::memory_order_acquire),
                .primaryHandEvent = primaryHandEvent,
                .firingHandIsPrimaryHand = firingHandIsPrimaryHand,
                .buttonJustPressed = button && button->QJustPressed(),
                .eventMatched = eventMatched,
            });

            if (eventMatched && button && button->QJustPressed()) {
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Reload hand gate: primaryHandEvent={} primaryHandIsLeft={} firing={} -> {}",
                    primaryHandEvent ? "yes" : "no",
                    primaryHandIsLeft ? "yes" : "no",
                    firingHandIsLeft ? "left" : "right",
                    route ? "route" : "reject");
            }
            return route;
        }

        [[nodiscard]] bool shouldDeferFiringHandActivateForManualScope(const RE::InputEvent* event)
        {
            const bool eventMatched = isActivateReloadEvent(event);
            const bool primaryHandEvent = eventMatched && isPrimaryWandInputEvent(event);
            constexpr bool primaryHandIsLeft = false;
            const bool firingHandIsLeft = s_equippedWeaponLeftHandFiringActive.load(std::memory_order_acquire);
            return input_remap_policy::shouldDeferFiringHandActivateForManualScope(input_remap_policy::ManualScopeActivateInput{
                .manualScopeEnabled = !g_rockConfig.rockAutoActivateScope,
                .rawInputCaptureAvailable = s_hooksInstalled.load(std::memory_order_acquire),
                .gameplayInputAllowed = s_gameplayInputAllowed.load(std::memory_order_acquire),
                .menuInputActive = isInputBlockingMenuActive(),
                .weaponDrawn = s_weaponDrawn.load(std::memory_order_acquire),
                .primaryHandEvent = primaryHandEvent,
                .firingHandIsPrimaryHand = firingHandIsLeft == primaryHandIsLeft,
                .eventMatched = eventMatched,
            });
        }

        [[nodiscard]] bool dispatchNativeReloadAction()
        {
            const bool gameplayAllowed =
                s_gameplayInputAllowed.load(std::memory_order_acquire);
            const bool menuActive = isInputBlockingMenuActive();
            // The cached snapshot is refreshed later in PhysicsInteraction's
            // hand pass. Query the provider lease here as well so a UI lease
            // acquired during this frame closes the reload path immediately.
            const bool providerSuppressed =
                isAnyProviderOpenVrGameInputSuppressedAtDispatch();
            const bool weaponDrawn =
                s_weaponDrawn.load(std::memory_order_acquire);
            if (!gameplayAllowed || menuActive || providerSuppressed ||
                !weaponDrawn) {
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Rejected native reload dispatch at final gate gameplay={} menu={} providerLease={} weaponDrawn={}",
                    gameplayAllowed ? "yes" : "no",
                    menuActive ? "yes" : "no",
                    providerSuppressed ? "yes" : "no",
                    weaponDrawn ? "yes" : "no");
                return false;
            }

            static REL::Relocation<void**> nativeActionDispatcherObject{ REL::Offset(kNativePlayerActionDispatcherDataOffset) };
            static REL::Relocation<NativeActionDispatcher_t> nativeActionDispatcher{ REL::Offset(kNativeActionDispatcherFunctionOffset) };

            auto* dispatcherObject = *nativeActionDispatcherObject;
            if (!dispatcherObject) {
                ROCK_LOG_SAMPLE_WARN(Input, g_rockConfig.rockLogSampleMilliseconds, "Cannot route firing-hand activate to reload: native action dispatcher unavailable");
                return false;
            }

            return nativeActionDispatcher(
                dispatcherObject,
                kNativeReloadActionId,
                kNativeActionPriorityQueue);
        }

        /*
         * Take/Equip suppression must gate on the SAME hand whose wand fired the Activate
         * press, unlike Pipboy (which always rides the off-hand trigger). Both A (Activate)
         * and B (VATS) buttons live on the primary wand in FO4VR's default VR bindings, so in
         * practice this resolves to "is the primary hand holding a ROCK object", but the
         * primary/secondary selection is kept generic to match the native handler's own
         * primary/secondary wand dispatch (see kActivate*WandPickRefGlobalOffset comment).
         */
        [[nodiscard]] std::size_t takeEquipHandIndex(bool primaryHandEvent)
        {
            const bool eventHandIsLeft = !primaryHandEvent;
            return eventHandIsLeft ? 0u : 1u;
        }

        [[nodiscard]] bool isTakeEquipHandEngaged(bool primaryHandEvent)
        {
            return s_handInteractionEngaged[takeEquipHandIndex(primaryHandEvent)].load(std::memory_order_acquire);
        }

        [[nodiscard]] std::uint32_t readWandPickRefHandle(std::uintptr_t globalOffset)
        {
            /*
             * The globals at kActivate*WandPickRefGlobalOffset do NOT store a bare
             * ObjectRefHandle - each stores an 8-byte pointer to the wand's caster/view
             * object (the primary one is FO4VR's ViewCasterPrimaryWand). The real handle is
             * behind a locked virtual call, verified 2026-07-05 via raw disassembly of
             * FUN_1409D0CE0 (two agreeing sources: decompile + byte-for-byte disassembly,
             * cross-checked against a second caller in the already-verified native action
             * dispatcher at 0xFC07E0): lock casterObj+0x70, call the vtable slot at byte
             * offset 0x10 (index 2) with the caster as `this`, read the first 4 bytes of the
             * RETURNED pointer (not the caster itself) as the ObjectRefHandle, unlock. An
             * earlier build read the global's raw memory directly as the handle, which
             * silently produced a plausible-looking-but-wrong value (0xFE3A360 observed live)
             * and made suppression permanently a no-op - see
             * docs/docs/reverse-engineering/2026-07-05-take-equip-activate-classification-ghidra-findings.md
             * for full verification detail.
             */
            REL::Relocation<void**> casterObjectGlobal{ REL::Offset(globalOffset) };
            void* casterObject = *casterObjectGlobal;
            if (!casterObject) {
                return 0;
            }

            auto* lock = reinterpret_cast<RE::BSSpinLock*>(reinterpret_cast<std::uintptr_t>(casterObject) + kWandPickRefCasterSpinLockOffset);
            lock->lock("ROCK-take-equip-pick-ref");

            std::uint32_t handleValue = 0;
            const auto vtable = *reinterpret_cast<void***>(casterObject);
            if (vtable) {
                const auto accessor = reinterpret_cast<WandPickRefCasterAccessor_t>(vtable[kWandPickRefCasterVTableSlot]);
                if (void* handleStorage = accessor(casterObject)) {
                    handleValue = *reinterpret_cast<std::uint32_t*>(handleStorage);
                }
            }

            lock->unlock();
            return handleValue;
        }

        [[nodiscard]] bool isTakeEquipTargetEligible(bool primaryHandEvent)
        {
            const auto globalOffset = primaryHandEvent ? kActivatePrimaryWandPickRefGlobalOffset : kActivateSecondaryWandPickRefGlobalOffset;
            std::uint32_t handleValue = readWandPickRefHandle(globalOffset);
            if (handleValue == 0) {
                /*
                 * The native dispatcher at 0xFC07E0 falls back to the other wand's caster when
                 * the primary one resolves empty (observed for an adjacent action-id dispatch
                 * that shares this same accessor). Mirrored defensively here - it can only
                 * help, never misclassify, since a resolved secondary-hand target still goes
                 * through the identical identity/FormType checks below.
                 */
                const auto fallbackOffset = primaryHandEvent ? kActivateSecondaryWandPickRefGlobalOffset : kActivatePrimaryWandPickRefGlobalOffset;
                handleValue = readWandPickRefHandle(fallbackOffset);
            }
            if (handleValue == 0) {
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Take/Equip classification stopped at stage=no-handle (offset=0x{:X} value=0)",
                    globalOffset);
                return false;
            }

            RE::ObjectRefHandle handle{};
            static_assert(sizeof(handle) == sizeof(handleValue));
            std::memcpy(&handle, &handleValue, sizeof(handle));

            const auto ref = handle.get();
            if (!ref) {
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Take/Equip classification stopped at stage=handle-resolve (offset=0x{:X} handleValue=0x{:X})",
                    globalOffset,
                    handleValue);
                return false;
            }

            /*
             * Priority case: the wand's pick-ref is very often the very object ROCK is already
             * holding in this hand (it is right in front of/touching the hand). Native
             * Activate must never take/equip an object ROCK already owns, regardless of its
             * FormType, so an exact identity match against the hand's own held-object formID
             * (pushed in each frame from PhysicsInteraction via setHeldObjectFormId, sourced
             * from Hand::getHeldRef()) always suppresses - this does not depend on the
             * FormType allowlist below at all.
             */
            const auto refFormId = ref->GetFormID();
            const auto heldFormId = s_heldObjectFormId[takeEquipHandIndex(primaryHandEvent)].load(std::memory_order_acquire);
            if (heldFormId != 0 && heldFormId == refFormId) {
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Take/Equip suppression classification: pick-ref formID=0x{:X} matches ROCK's own held object in this hand",
                    refFormId);
                return true;
            }

            const auto* baseForm = ref->GetObjectReference();
            const char* formTypeChars = baseForm ? baseForm->GetFormTypeString() : nullptr;
            if (!formTypeChars) {
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Take/Equip classification stopped at stage=form-type (handleValue=0x{:X} refFormID=0x{:X} heldFormID=0x{:X})",
                    handleValue,
                    refFormId,
                    heldFormId);
                return false;
            }

            const bool eligible = far_selection_blacklist_policy::listContainsText(g_rockConfig.rockSuppressTakeEquipFormTypes, formTypeChars);
            ROCK_LOG_SAMPLE_DEBUG(Input,
                g_rockConfig.rockLogSampleMilliseconds,
                "Take/Equip suppression classification: handleValue=0x{:X} refFormID=0x{:X} heldFormID=0x{:X} formType='{}' eligible={}",
                handleValue,
                refFormId,
                heldFormId,
                formTypeChars,
                eligible ? "yes" : "no");
            return eligible;
        }

        [[nodiscard]] bool shouldSuppressNativeTakeEquipActionEvent(const RE::InputEvent* event)
        {
            const bool eventMatched = isActivateReloadEvent(event);
            if (!eventMatched) {
                return false;
            }

            const bool primaryHandEvent = isPrimaryWandInputEvent(event);
            const bool handEngaged = isTakeEquipHandEngaged(primaryHandEvent);
            const bool targetEligible = handEngaged && isTakeEquipTargetEligible(primaryHandEvent);

            auto input = makeNativeActionSuppressionInput(g_rockConfig.rockSuppressTakeEquipGameInputWhileHolding, event, eventMatched);
            input.takeEquipHandEngaged = handEngaged;
            input.takeEquipTargetEligible = targetEligible;
            const bool suppress = input_remap_policy::shouldSuppressNativeTakeEquipAction(input);

            // Temporary diagnostic: fires on every matched Activate/WandAccept edge so a single
            // in-game repro shows exactly which gate (hand-engaged vs target-FormType) blocks
            // suppression. Rate-limited like every other native-action trace in this file.
            ROCK_LOG_SAMPLE_DEBUG(Input,
                g_rockConfig.rockLogSampleMilliseconds,
                "Take/Equip gate: primaryHandEvent={} physicalHand={} handEngaged={} targetEligible={} suppressionEnabled={} gameplay={} menuInput={} -> {}",
                primaryHandEvent ? "yes" : "no",
                primaryHandEvent ? "right" : "left",
                handEngaged ? "yes" : "no",
                targetEligible ? "yes" : "no",
                g_rockConfig.rockSuppressTakeEquipGameInputWhileHolding ? "yes" : "no",
                input.gameplayInputAllowed ? "yes" : "no",
                input.menuInputActive ? "yes" : "no",
                suppress ? "suppress" : "native");

            return suppress;
        }

        /*
         * Developer-mode saved-grab-offset recorder: a plain A-button
         * (Activate/WandAccept) press records/overwrites the saved grab
         * offset for whichever hand(s) currently hold a ROCK object.
         *
         * Activate/WandAccept is a single physical button that only ever
         * fires from one wand (the primary, per isPrimaryWandInputEvent),
         * so this deliberately does NOT map the event to a single target
         * hand the way take/equip suppression does. It checks both hands'
         * actual engaged state directly: that is what lets a left-hand-held
         * object be saved even though the button itself lives on the
         * (default) right/primary controller, and lets a single press save
         * both hands as distinct offsets when each hand holds something.
         * Never gates on target FormType and never stops the event - it is
         * a pure side-effect tap.
         */
        [[nodiscard]] bool handleSavedGrabOffsetRequestEvent(const RE::InputEvent* event)
        {
            if (!g_rockConfig.rockDeveloperModeEnabled || !isActivateReloadEvent(event)) {
                return false;
            }

            const auto* button = event->As<RE::ButtonEvent>();
            if (!button || !button->QJustPressed()) {
                return false;
            }

            bool requested = false;
            for (const bool isLeft : { true, false }) {
                const std::size_t handIndex = isLeft ? 0u : 1u;
                if (s_handInteractionEngaged[handIndex].load(std::memory_order_acquire)) {
                    s_pendingSavedGrabOffsetRequest[handIndex].store(true, std::memory_order_release);
                    requested = true;
                }
            }
            return requested;
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

            if (shouldDeferFiringHandActivateForManualScope(inputEvent)) {
                markInputEventStopped(inputEvent);
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Deferred native firing-hand activate/use event to manual scope/reload hold arbitration hand={}",
                    s_equippedWeaponLeftHandFiringActive.load(std::memory_order_acquire) ? "left-X" : "right-A");
                return;
            }

            if (handleSavedGrabOffsetRequestEvent(inputEvent)) {
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Recorded a pending saved-grab-offset request from an Activate/WandAccept press");
            }

            if (shouldRouteFiringHandActivateReload(inputEvent)) {
                markInputEventStopped(inputEvent);
                if (dispatchNativeReloadAction()) {
                    ROCK_LOG_SAMPLE_DEBUG(Input,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Routed native firing-hand activate/use input to equipped weapon reload hand={}",
                        s_equippedWeaponLeftHandFiringActive.load(std::memory_order_acquire) ? "left-X" : "right-A");
                }
                return;
            }

            if (shouldSuppressNativeTakeEquipActionEvent(inputEvent)) {
                markInputEventStopped(inputEvent);
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed native Activate/WandAccept take/equip outcome while ROCK holds an object in the same hand");
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

        [[nodiscard]] bool decideAndTracePipboyOpenSuppression(const RE::InputEvent* inputEvent)
        {
            const bool providerSuppressed = isAnyProviderOpenVrGameInputSuppressed();
            const bool legacyTriggerSuppressed = shouldSuppressLegacyPipboyTriggerOpenEvent(inputEvent);
            const bool interactionSuppressed = shouldSuppressNativePipboyActionEvent(inputEvent);
            const bool suppressed = providerSuppressed || legacyTriggerSuppressed || interactionSuppressed;

            if (inputEvent) {
                const auto& userEvent = inputEvent->QUserEvent();
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Pipboy handler event '{}': movedTrigger={} engaged={} gameplay={} menuInput={} providerLease={} -> {}",
                    userEvent.c_str() ? userEvent.c_str() : "",
                    legacyTriggerSuppressed ? "yes" : "no",
                    isPipboyHandEngaged() ? "yes" : "no",
                    s_gameplayInputAllowed.load(std::memory_order_acquire) ? "yes" : "no",
                    isInputBlockingMenuActive() ? "yes" : "no",
                    providerSuppressed ? "yes" : "no",
                    suppressed ? "suppressed" : "native");
            }

            return suppressed;
        }

        [[nodiscard]] bool decideAndTracePipboyLightSuppression(const RE::InputEvent* inputEvent)
        {
            const bool providerSuppressed = isAnyProviderOpenVrGameInputSuppressed();
            const bool interactionSuppressed = shouldSuppressNativePipboyActionEvent(inputEvent);
            const bool suppressed = providerSuppressed || interactionSuppressed;

            if (inputEvent) {
                const auto& userEvent = inputEvent->QUserEvent();
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "PipboyLight handler event '{}': engaged={} gameplay={} menuInput={} providerLease={} -> {}",
                    userEvent.c_str() ? userEvent.c_str() : "",
                    isPipboyHandEngaged() ? "yes" : "no",
                    s_gameplayInputAllowed.load(std::memory_order_acquire) ? "yes" : "no",
                    isInputBlockingMenuActive() ? "yes" : "no",
                    providerSuppressed ? "yes" : "no",
                    suppressed ? "suppressed" : "native");
            }

            return suppressed;
        }

        [[nodiscard]] void* resolvePipboyHandlerFromMenuControls()
        {
            static REL::Relocation<void**> menuControlsSingleton{ REL::Offset(kMenuControlsSingletonOffset) };
            auto* menuControls = *menuControlsSingleton;
            if (!menuControls) {
                return nullptr;
            }

            auto* pipboyHandler = *reinterpret_cast<void**>(
                reinterpret_cast<std::uintptr_t>(menuControls) + kMenuControlsPipboyHandlerOffset);
            if (!pipboyHandler) {
                return nullptr;
            }

            const auto actualVTable = *reinterpret_cast<const std::uintptr_t*>(pipboyHandler);
            const auto expectedVTable = REL::Offset(kPipboyHandlerVTableOffset).address();
            if (actualVTable != expectedVTable) {
                ROCK_LOG_SAMPLE_WARN(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Cannot route Pause tap to Pip-Boy: handler vtable 0x{:X}, expected 0x{:X}",
                    actualVTable,
                    expectedVTable);
                return nullptr;
            }

            return pipboyHandler;
        }

        [[nodiscard]] bool dispatchNativePipboyTap(RE::ButtonEvent& event)
        {
            auto* pipboyHandler = resolvePipboyHandlerFromMenuControls();
            if (!pipboyHandler || !s_originalPipboyEventHandler) {
                ROCK_LOG_SAMPLE_WARN(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Cannot route Pause tap to Pip-Boy: native PipboyHandler unavailable");
                return false;
            }

            const RE::BSFixedString originalUserEvent = event.strUserEvent;
            const float originalValue = event.value;
            const float originalHeldDownSecs = event.heldDownSecs;
            const auto originalHandled = event.handled;

            // The verified VR path requires the native WandTrigger lifecycle,
            // including its secondary-device check. The physical Pause event
            // already carries that same secondary wand device ID; only the
            // semantic name and button phase are temporarily substituted.
            event.strUserEvent = kNativeEventWandTrigger.data();
            event.value = 1.0f;
            event.heldDownSecs = 0.0f;
            event.handled = RE::InputEvent::HANDLED_RESULT::kUnhandled;
            s_originalPipboyEventHandler(pipboyHandler, &event);

            event.value = 0.0f;
            event.heldDownSecs = (std::max)(originalHeldDownSecs, 0.001f);
            event.handled = RE::InputEvent::HANDLED_RESULT::kUnhandled;
            s_originalPipboyEventHandler(pipboyHandler, &event);

            event.strUserEvent = originalUserEvent;
            event.value = originalValue;
            event.heldDownSecs = originalHeldDownSecs;
            event.handled = originalHandled;
            return true;
        }

        [[nodiscard]] bool dispatchNativePauseHold(void* handler, RE::ButtonEvent& event)
        {
            if (!handler || !s_originalMenuOpenEventHandler) {
                ROCK_LOG_SAMPLE_WARN(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Cannot route held Pause button: native MenuOpenHandler unavailable");
                return false;
            }

            const float originalValue = event.value;
            const float originalHeldDownSecs = event.heldDownSecs;
            const auto originalHandled = event.handled;

            // MenuOpenHandler performs its complete native eligibility and
            // secondary-device checks on a release phase. Feed that verified
            // phase once when ROCK's hold threshold is crossed; the physical
            // release is consumed later by the gesture state machine.
            event.value = 0.0f;
            event.heldDownSecs = (std::max)(
                originalHeldDownSecs,
                pipboy_pause_gesture_policy::sanitizedHoldSeconds(g_rockConfig.rockPipboyPauseHoldSeconds));
            event.handled = RE::InputEvent::HANDLED_RESULT::kUnhandled;
            s_originalMenuOpenEventHandler(handler, &event);

            event.value = originalValue;
            event.heldDownSecs = originalHeldDownSecs;
            event.handled = originalHandled;
            return true;
        }

        void hookedNativeVatsVansDecision(RE::ButtonEvent* button)
        {
            if (!button) {
                if (s_originalNativeVatsVansDecision) {
                    s_originalNativeVatsVansDecision(button);
                }
                return;
            }

            const std::uint32_t flags =
                currentProviderHandInputSuppressionFlagsAtDispatch();
            const bool suppressAll =
                isAnyProviderOpenVrGameInputSuppressed() ||
                provider::hasHandInputSuppressionFlagV1(
                    flags,
                    provider::RockProviderHandInputSuppressionFlagV1::
                        SuppressOpenVrGameInput);
            const auto decision =
                native_vats_input_suppression_policy::update(
                    s_nativeVatsInputSuppressionState,
                    native_vats_input_suppression_policy::Input{
                        .buttonDown = button->QPressed(),
                        .justPressed = button->QJustPressed(),
                        .released =
                            !button->QPressed() &&
                            button->QHeldDownSecs() >= 0.0f,
                        .suppressVats =
                            g_rockConfig.rockSuppressNativeVats ||
                            provider::hasHandInputSuppressionFlagV1(
                                flags,
                                provider::
                                    RockProviderHandInputSuppressionFlagV1::
                                        SuppressNativeVats),
                        .suppressVans =
                            g_rockConfig.rockSuppressNativeVans ||
                            provider::hasHandInputSuppressionFlagV1(
                                flags,
                                provider::
                                    RockProviderHandInputSuppressionFlagV1::
                                        SuppressNativeVans),
                        .suppressAll = suppressAll,
                    });

            if (decision.forwardNative) {
                if (s_originalNativeVatsVansDecision) {
                    s_originalNativeVatsVansDecision(button);
                }
                return;
            }

            ROCK_LOG_SAMPLE_DEBUG(Input,
                g_rockConfig.rockLogSampleMilliseconds,
                "Suppressed native VATS input phase: reason={} heldSeconds={:.3f}",
                decision.reason,
                button->QHeldDownSecs());
        }

        void hookedMenuOpenEventHandler(void* handler, RE::InputEvent* inputEvent)
        {
            auto* button = inputEvent ? inputEvent->As<RE::ButtonEvent>() : nullptr;
            if (!button || !eventNameMatches(inputEvent, kNativeEventPause) || !isSecondaryWandInputEvent(inputEvent)) {
                if (s_originalMenuOpenEventHandler) {
                    s_originalMenuOpenEventHandler(handler, inputEvent);
                }
                return;
            }

            const bool gameplayInputAllowed = s_gameplayInputAllowed.load(std::memory_order_acquire);
            const bool menuInputActive = isInputBlockingMenuActive();
            const bool providerSuppressed = isAnyProviderOpenVrGameInputSuppressedAtDispatch();
            const auto decision = pipboy_pause_gesture_policy::update(s_pipboyPauseGestureState,
                pipboy_pause_gesture_policy::Input{
                    .enabled = g_rockConfig.rockInputRemapEnabled,
                    .eligible = gameplayInputAllowed && !menuInputActive,
                    .pressed = button->QJustPressed(),
                    .held = button->QPressed(),
                    .released = !button->QPressed(),
                    .pipboyDispatchAllowed = !providerSuppressed,
                    .heldSeconds = button->QHeldDownSecs(),
                    .holdSeconds = g_rockConfig.rockPipboyPauseHoldSeconds,
                });

            if (!decision.consume) {
                if (s_originalMenuOpenEventHandler) {
                    s_originalMenuOpenEventHandler(handler, inputEvent);
                }
                return;
            }

            if (decision.dispatchPipboy) {
                const bool routed = dispatchNativePipboyTap(*button);
                ROCK_LOG_DEBUG(Input,
                    "Pause-button short press routed to Pip-Boy result={} heldSeconds={:.3f}",
                    routed ? "dispatched" : "unavailable",
                    button->QHeldDownSecs());
            } else if (decision.dispatchPause) {
                const bool routed = dispatchNativePauseHold(handler, *button);
                ROCK_LOG_DEBUG(Input,
                    "Pause-button hold routed to native Pause result={} heldSeconds={:.3f}",
                    routed ? "dispatched" : "unavailable",
                    button->QHeldDownSecs());
            } else if (std::string_view{ decision.reason } == "short-release-blocked") {
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed Pause-button Pip-Boy tap while provider OpenVR game-input suppression is active");
            }

            markInputEventStopped(inputEvent);
        }

        void hookedPipboyEventHandler(void* handler, RE::InputEvent* inputEvent)
        {
            if (decideAndTracePipboyOpenSuppression(inputEvent)) {
                markInputEventStopped(inputEvent);
                return;
            }

            if (s_originalPipboyEventHandler) {
                s_originalPipboyEventHandler(handler, inputEvent);
            }
        }

        void hookedPipboyLightEventHandler(void* handler, RE::InputEvent* inputEvent, void* cursor, void* unk)
        {
            if (decideAndTracePipboyLightSuppression(inputEvent)) {
                markInputEventStopped(inputEvent);
                return;
            }

            if (s_originalPipboyLightEventHandler) {
                s_originalPipboyLightEventHandler(handler, inputEvent, cursor, unk);
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

        bool installPipboyPauseArbitrationHooks()
        {
            const bool openHookReady = installNativeActionVTableHook(kPipboyHandlerHandleEventVTableSlotOffset,
                kPipboyHandlerHandleEventFunctionOffset,
                &hookedPipboyEventHandler,
                s_originalPipboyEventHandler,
                s_pipboyEventHookInstalled,
                "PipboyHandler::HandleButtonEvent suppression");
            const bool lightHookReady = installNativeActionVTableHook(kPipboyLightHandlerHandleEventVTableSlotOffset,
                kPipboyLightHandlerHandleEventFunctionOffset,
                &hookedPipboyLightEventHandler,
                s_originalPipboyLightEventHandler,
                s_pipboyLightEventHookInstalled,
                "PipboyLightHandler::HandleEvent suppression");
            const bool menuOpenHookReady = installNativeActionVTableHook(kMenuOpenHandlerHandleEventVTableSlotOffset,
                kMenuOpenHandlerHandleEventFunctionOffset,
                &hookedMenuOpenEventHandler,
                s_originalMenuOpenEventHandler,
                s_menuOpenEventHookInstalled,
                "MenuOpenHandler::HandleButtonEvent Pip-Boy/Pause arbitration");
            return openHookReady && lightHookReady && menuOpenHookReady;
        }

        bool installNativeVatsVansInputSuppressionHook()
        {
            if (s_nativeVatsVansDecisionHookInstalled.load(
                    std::memory_order_acquire)) {
                return true;
            }
            if (s_nativeVatsVansDecisionHookInstallFailed.load(
                    std::memory_order_acquire)) {
                return false;
            }

            REL::Relocation<std::uintptr_t> callSite{
                REL::Offset(kNativeVatsVansDecisionCallSiteOffset)
            };
            const auto callSiteAddress = callSite.address();
            const auto* callBytes =
                reinterpret_cast<const std::uint8_t*>(callSiteAddress);
            if (!callBytes || callBytes[0] != 0xE8) {
                ROCK_LOG_ERROR(Input,
                    "Native VATS/V.A.N.S. suppression hook validation failed at 0x{:X}: expected CALL rel32",
                    callSiteAddress);
                s_nativeVatsVansDecisionHookInstallFailed.store(
                    true,
                    std::memory_order_release);
                return false;
            }

            const auto relativeTarget =
                *reinterpret_cast<const std::int32_t*>(callBytes + 1);
            const auto decodedTarget =
                callSiteAddress + 5u + relativeTarget;
            const auto expectedTarget =
                REL::Offset(kNativeVatsVansDecisionFunctionOffset).address();
            if (decodedTarget != expectedTarget) {
                ROCK_LOG_ERROR(Input,
                    "Native VATS/V.A.N.S. suppression hook validation failed at 0x{:X}: target 0x{:X}, expected 0x{:X}",
                    callSiteAddress,
                    decodedTarget,
                    expectedTarget);
                s_nativeVatsVansDecisionHookInstallFailed.store(
                    true,
                    std::memory_order_release);
                return false;
            }

            auto& trampoline = F4SE::GetTrampoline();
            const auto original = trampoline.write_call<5>(
                callSiteAddress,
                &hookedNativeVatsVansDecision);
            s_originalNativeVatsVansDecision =
                reinterpret_cast<NativeVatsVansDecision_t>(original);
            const bool installed =
                s_originalNativeVatsVansDecision != nullptr;
            s_nativeVatsVansDecisionHookInstalled.store(
                installed,
                std::memory_order_release);
            if (!installed) {
                ROCK_LOG_ERROR(Input,
                    "Native VATS/V.A.N.S. suppression hook install returned a null original target");
                s_nativeVatsVansDecisionHookInstallFailed.store(
                    true,
                    std::memory_order_release);
                return false;
            }

            ROCK_LOG_INFO(Input,
                "Installed native VATS/V.A.N.S. suppression hook at 0x{:X}; original=0x{:X}",
                callSiteAddress,
                original);
            return true;
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

        bool updateNativeActionSuppressionHooks(const input_remap_policy::Settings& settings, const bool manualScopeEnabled)
        {
            bool ready = installNativeVatsVansInputSuppressionHook();
            if (input_remap_policy::shouldInstallNativeActionSuppressionHook(settings.enabled, settings.suppressRightGrabGameInput)) {
                ready = installReadyWeaponEventSuppressionHook() && ready;
            }
            if (input_remap_policy::shouldInstallActivateEventHook(settings.enabled, manualScopeEnabled)) {
                ready = installActivateEventReloadHook() && ready;
            }
            if (input_remap_policy::shouldInstallNativeActionSuppressionHook(settings.enabled, settings.suppressRightFavoritesGameInput)) {
                ready = installFavoritesEventSuppressionHook() && ready;
            }
            if (input_remap_policy::shouldInstallNativeActionSuppressionHook(settings.enabled, settings.suppressNativeMeleeThrowGameInput)) {
                ready = installMeleeThrowEventSuppressionHook() && ready;
            }
            if (input_remap_policy::shouldInstallPipboyPauseArbitrationHooks(settings.enabled)) {
                ready = installPipboyPauseArbitrationHooks() && ready;
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
                result.availabilityReason =
                    RawButtonAvailabilityReason::InvalidButton;
                return result;
            }

            auto& tracker = s_controllers[isLeft ? 0u : 1u];
            if (!tracker.valid.load(std::memory_order_acquire)) {
                return result;
            }

            result.sampleSequence =
                tracker.sampleSequence.load(std::memory_order_acquire);
            const auto sampleTick = tracker.sampleTickMilliseconds.load(
                std::memory_order_acquire);
            const auto now = GetTickCount64();
            result.sampleAgeMilliseconds = static_cast<std::uint32_t>(
                (std::min<std::uint64_t>)(
                    now >= sampleTick ? now - sampleTick : 0,
                    UINT32_MAX));
            result.available = true;
            result.availabilityReason = RawButtonAvailabilityReason::Available;
            const auto rawPressed = tracker.rawPressed.load(std::memory_order_acquire);
            const bool rawHeld = (rawPressed & mask) != 0;

            if (isInputBlockingMenuActive()) {
                if (rawHeld) {
                    tracker.rearmPressedMask.fetch_or(mask, std::memory_order_acq_rel);
                }
                clearButtonEdges(tracker, mask);
                result.available = false;
                result.availabilityReason =
                    RawButtonAvailabilityReason::BlockingMenu;
                return result;
            }

            if ((tracker.rearmPressedMask.load(std::memory_order_acquire) & mask) != 0) {
                clearButtonEdges(tracker, mask);
                if (!rawHeld) {
                    tracker.rearmPressedMask.fetch_and(~mask, std::memory_order_acq_rel);
                }
                result.available = false;
                result.availabilityReason =
                    RawButtonAvailabilityReason::ReleaseToRearm;
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

    }

    bool installInputRemapHooks()
    {
        ensureMenuInputGateRegistered();

        const auto settings = makeSettings();
        const bool manualScopeEnabled = !g_rockConfig.rockAutoActivateScope;
        const bool nativeActionSuppressionReady = updateNativeActionSuppressionHooks(settings, manualScopeEnabled);

        if (!input_remap_policy::shouldInstallRawControllerHooks(settings.enabled, manualScopeEnabled)) {
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
        void* originalTriggerHapticPulse = reinterpret_cast<void*>(s_originalTriggerHapticPulse);

        const bool stateHooked = patchVTableSlot(
            s_vrSystemVTable, kGetControllerStateVTableIndex, reinterpret_cast<void*>(&hookedGetControllerState), originalState, "IVRSystem::GetControllerState");
        const bool stateWithPoseHooked = patchVTableSlot(s_vrSystemVTable,
            kGetControllerStateWithPoseVTableIndex,
            reinterpret_cast<void*>(&hookedGetControllerStateWithPose),
            originalStateWithPose,
            "IVRSystem::GetControllerStateWithPose");
        // Haptic reroute degrades gracefully: a failed install only means
        // fire rumble stays on the right wand during left-firing.
        if (!patchVTableSlot(s_vrSystemVTable,
                kTriggerHapticPulseVTableIndex,
                reinterpret_cast<void*>(&hookedTriggerHapticPulse),
                originalTriggerHapticPulse,
                "IVRSystem::TriggerHapticPulse")) {
            ROCK_LOG_WARN(Input, "TriggerHapticPulse hook unavailable; left-firing haptics stay on the right wand");
        }

        s_originalGetControllerState = reinterpret_cast<GetControllerState_t>(originalState);
        s_originalGetControllerStateWithPose = reinterpret_cast<GetControllerStateWithPose_t>(originalStateWithPose);
        s_originalTriggerHapticPulse = reinterpret_cast<TriggerHapticPulse_t>(originalTriggerHapticPulse);

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
        if (!allowed) {
            blockManualScopeInputUntilRelease();
        }
    }

    void setWeaponDrawn(bool weaponDrawn)
    {
        s_weaponDrawn.store(weaponDrawn, std::memory_order_release);
        if (!weaponDrawn) {
            blockManualScopeInputUntilRelease();
        }
    }

    void setHandHeldWeapon(const bool isLeft, const bool heldWeapon)
    {
        s_handHeldWeapon[isLeft ? 0u : 1u].store(heldWeapon, std::memory_order_release);
    }

    void setHandInteractionEngaged(bool isLeft, bool engaged)
    {
        s_handInteractionEngaged[isLeft ? 0u : 1u].store(engaged, std::memory_order_release);
    }

    void setHeldObjectFormId(bool isLeft, std::uint32_t formId)
    {
        s_heldObjectFormId[isLeft ? 0u : 1u].store(formId, std::memory_order_release);
    }

    void setEquippedWeaponFiringGripInputActive(bool active)
    {
        s_equippedWeaponFiringGripInputActive.store(active, std::memory_order_release);
    }

    void setEquippedWeaponPrimaryDetached(bool detached)
    {
        s_equippedWeaponPrimaryDetached.store(detached, std::memory_order_release);
    }

    void setEquippedWeaponLeftHandFiringActive(bool active)
    {
        const bool previous = s_equippedWeaponLeftHandFiringActive.exchange(active, std::memory_order_acq_rel);
        if (previous != active) {
            blockManualScopeInputUntilRelease();
            ROCK_LOG_INFO(Input, "Left-hand fire trigger remap {}", active ? "ENGAGED" : "released");
        }
    }

    void setProviderOpenVrGameInputSuppressed(bool isLeft, bool suppressed)
    {
        const bool wasSuppressed =
            s_providerOpenVrGameInputSuppressed[isLeft ? 0u : 1u].exchange(
                suppressed,
                std::memory_order_acq_rel);
        if (suppressed && !wasSuppressed) {
            // This setter is driven from PhysicsInteraction's frame thread.
            // A provider UI lease must also cancel an in-progress A/X
            // tap/hold transaction so its eventual release cannot reload.
            blockManualScopeInputUntilRelease();
        }
    }

    void updateFiringHandReloadInput(const float deltaSeconds)
    {
        /*
         * Consume both physical accept buttons every frame. Automatic mode
         * preserves the existing primary-event/secondary-raw reload split.
         * Manual mode owns either physical firing-hand gesture end to end:
         * release before the threshold dispatches reload, while crossing it
         * holds native scope activation until release. Draining both edges in
         * every mode prevents a config or firing-hand change from replaying a
         * stale press.
         *
         * The accumulator's menu rearm guarantees a menu-accept press (A/X
         * confirms UI selections) cannot replay as a reload on menu exit.
         * Provider API consumers are unaffected: apiGetRawWandButtonStateV1
         * exposes level state only, by design. Frame-thread only.
         */
        const auto leftAcceptState = consumeRawButtonState(true, input_remap_policy::kOpenVrAcceptButtonId);
        const auto rightAcceptState = consumeRawButtonState(false, input_remap_policy::kOpenVrAcceptButtonId);
        if (isAnyProviderOpenVrGameInputSuppressed()) {
            // The reads above intentionally drain both physical A/X edges.
            // SuppressOpenVrGameInput is a game-facing lease, so ROCK must not
            // turn the Configurator's raw UI press into a native reload.
            blockManualScopeInputUntilRelease();
            return;
        }
        constexpr bool secondaryHandIsLeft = true;
        const bool firingHandIsLeft = s_equippedWeaponLeftHandFiringActive.load(std::memory_order_acquire);

        if (!g_rockConfig.rockAutoActivateScope) {
            const auto toManualButtonState = [](const RawButtonState& state) {
                return manual_scope_input_policy::ButtonState{
                    .available = state.available,
                    .held = state.held,
                    .pressed = state.pressed,
                    .released = state.released,
                };
            };
            const auto previousState = s_manualScopeInputState.state;
            const auto decision = manual_scope_input_policy::update(s_manualScopeInputState,
                manual_scope_input_policy::Input{
                    .manualModeEnabled = true,
                    .gameplayInputAllowed = s_gameplayInputAllowed.load(std::memory_order_acquire),
                    .menuInputActive = isInputBlockingMenuActive(),
                    .weaponDrawn = s_weaponDrawn.load(std::memory_order_acquire),
                    .firingHandIsLeft = firingHandIsLeft,
                    .leftButton = toManualButtonState(leftAcceptState),
                    .rightButton = toManualButtonState(rightAcceptState),
                    .deltaSeconds = deltaSeconds,
                    .holdSeconds = g_rockConfig.rockManualScopeHoldSeconds,
                });
            s_manualScopeActivationRequested.store(decision.scopeRequested, std::memory_order_release);

            if (decision.state != previousState || decision.dispatchReload) {
                ROCK_LOG_DEBUG(Input,
                    "Manual scope input: hand={} state={}->{} scope={} reload={} reason={}",
                    firingHandIsLeft ? "left-X" : "right-A",
                    static_cast<std::uint32_t>(previousState),
                    static_cast<std::uint32_t>(decision.state),
                    decision.scopeRequested ? "held" : "off",
                    decision.dispatchReload ? "dispatch" : "no",
                    decision.reason);
            }

            if (decision.dispatchReload && dispatchNativeReloadAction()) {
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Dispatched manual-scope short release to equipped weapon reload hand={}",
                    firingHandIsLeft ? "left-X" : "right-A");
            }
            return;
        }

        resetManualScopeInput();
        const auto& acceptState = secondaryHandIsLeft ? leftAcceptState : rightAcceptState;
        if (!acceptState.available || !acceptState.pressed) {
            return;
        }

        const bool firingHandIsSecondaryHand = firingHandIsLeft == secondaryHandIsLeft;
        const bool dispatch = input_remap_policy::shouldDispatchSecondaryHandReloadPress(input_remap_policy::SecondaryHandReloadInput{
            .remapEnabled = g_rockConfig.rockInputRemapEnabled,
            .gameplayInputAllowed = s_gameplayInputAllowed.load(std::memory_order_acquire),
            .menuInputActive = isInputBlockingMenuActive(),
            .weaponDrawn = s_weaponDrawn.load(std::memory_order_acquire),
            .firingHandIsSecondaryHand = firingHandIsSecondaryHand,
            .acceptButtonPressedEdge = acceptState.pressed,
        });

        ROCK_LOG_SAMPLE_DEBUG(Input,
            g_rockConfig.rockLogSampleMilliseconds,
            "Reload press gate: hand={} firing={} weaponDrawn={} -> {}",
            secondaryHandIsLeft ? "left-X" : "right-A",
            firingHandIsLeft ? "left" : "right",
            s_weaponDrawn.load(std::memory_order_acquire) ? "yes" : "no",
            dispatch ? "dispatch" : "drop");

        if (dispatch && dispatchNativeReloadAction()) {
            ROCK_LOG_SAMPLE_DEBUG(Input,
                g_rockConfig.rockLogSampleMilliseconds,
                "Dispatched secondary-hand accept press to equipped weapon reload hand={}",
                secondaryHandIsLeft ? "left-X" : "right-A");
        }
    }

    bool isManualScopeActivationRequested()
    {
        return !g_rockConfig.rockAutoActivateScope &&
               s_manualScopeActivationRequested.load(std::memory_order_acquire);
    }

    bool isMenuInputActive()
    {
        return isInputBlockingMenuActive();
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

    bool isPipboyMenuOpen()
    {
        return s_pipboyMenuOpen.load(std::memory_order_acquire);
    }

    PipboyEquipTriggerResolution consumePipboyEquipTriggerResolution()
    {
        constexpr std::uint32_t kTransitionMaximumAgeMilliseconds = 500;
        const auto unpack = [](const std::uint64_t packed) {
            return pipboy_equip_policy::TransitionToken{
                .menuGeneration = static_cast<std::uint32_t>(packed >> 32u),
                .tickMilliseconds = static_cast<std::uint32_t>(packed),
                .present = packed != 0,
            };
        };

        // Consume both tokens even when held-state resolves the hand so no
        // transition can leak into the next Pip-Boy selection.
        const auto leftTransition = unpack(s_pipboyTriggerTransitions[0].exchange(0, std::memory_order_acq_rel));
        const auto rightTransition = unpack(s_pipboyTriggerTransitions[1].exchange(0, std::memory_order_acq_rel));
        const auto resolution = pipboy_equip_policy::resolveTriggerHand(
            isRawButtonPhysicallyHeld(true, input_remap_policy::kOpenVrSteamVrTriggerButtonId),
            isRawButtonPhysicallyHeld(false, input_remap_policy::kOpenVrSteamVrTriggerButtonId),
            leftTransition,
            rightTransition,
            s_pipboyMenuGeneration.load(std::memory_order_acquire),
            static_cast<std::uint32_t>(GetTickCount64()),
            kTransitionMaximumAgeMilliseconds);
        return PipboyEquipTriggerResolution{
            .hand = resolution.hand,
            .source = resolution.source,
        };
    }

    bool consumePendingSavedGrabOffsetRequest(bool isLeft)
    {
        return s_pendingSavedGrabOffsetRequest[isLeft ? 0u : 1u].exchange(false, std::memory_order_acq_rel);
    }

    RawButtonState peekRawButtonState(bool isLeft, int buttonId)
    {
        return readRawButtonState(isLeft, buttonId, false);
    }

    RawButtonState consumeRawButtonState(bool isLeft, int buttonId)
    {
        return readRawButtonState(isLeft, buttonId, true);
    }

    bool isRawButtonPhysicallyHeld(bool isLeft, int buttonId)
    {
        const auto mask = input_remap_policy::buttonMask(buttonId);
        if (mask == 0) {
            return false;
        }

        const auto& tracker = s_controllers[isLeft ? 0u : 1u];
        return tracker.valid.load(std::memory_order_acquire) &&
               (tracker.rawPressed.load(std::memory_order_acquire) & mask) != 0;
    }

}
