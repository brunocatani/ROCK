#include "physics-interaction/core/PhysicsHooks.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/NativeMeleeSuppressionPolicy.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/object/PhysicsBodyClassifier.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/native/HavokRuntime.h"

#include "RockConfig.h"

#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/InputEvent.h"
#include "RE/Bethesda/Settings.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <string_view>

namespace rock
{
    namespace
    {
        using NativeMeleeHandler_t = bool (*)(void*, RE::Actor*, RE::BSFixedString*);
        using NativeMeleeInputGateHandler_t = bool (*)(void*, const RE::InputEvent*);
        using NativePlayerWeaponSwingCallback_t = void (*)(RE::Actor*, std::uint32_t);
        using NativeVrMeleeImpactCallback_t = void (*)(RE::Actor*, void*, void*);

        static NativeMeleeHandler_t g_originalWeaponSwingHandler = nullptr;
        static NativeMeleeHandler_t g_originalHitFrameHandler = nullptr;
        static NativeMeleeInputGateHandler_t g_originalAttackBlockShouldHandleEvent = nullptr;
        static NativePlayerWeaponSwingCallback_t g_originalPlayerWeaponSwingCallback = nullptr;
        static NativeVrMeleeImpactCallback_t g_originalVrMeleeImpactCallback = nullptr;
        /*
         * Native melee suppression can be queried from animation-event hooks, so
         * its physical-swing bridge needs a tiny shared clock. This is advanced
         * by ROCK update frames instead of wall milliseconds: debugger stalls,
         * menus, loading, and frame hitches must not expire a gameplay lease
         * behind the simulation.
         */
        static std::atomic<std::uint64_t> g_nativeMeleeFrameClock{ 1 };
        static std::array<std::atomic<std::uint64_t>, 2> g_nativeMeleePhysicalSwingExpiresAtFrame{};
        static std::atomic<bool> g_nativeMeleeSuppressionHooksInstalled{ false };
        constexpr std::uint64_t kNativeMeleePhysicalSwingLeaseFrames = 24;
        constexpr std::uint64_t kNativeMeleeRuntimeSettingCheckIntervalFrames = 90;
        constexpr char kNativeMeleeVelocityCheckSetting[] = "bMeleeVelocityCheck:VRInput";
        constexpr char kNativeMeleeLinearVelocityThresholdSetting[] = "fMeleeLinearVelocityThreshold:VRInput";
        constexpr char kNativeMeleeAngularVelocityThresholdSetting[] = "fMeleeAngularVelocityThreshold:VRInput";
        constexpr float kNativeMeleeSuppressedVelocityThreshold = 1.0e9f;
        constexpr std::array<std::uint8_t, 14> kVrMeleeImpactExpectedPrefix{
            0x48, 0x8B, 0xC4,
            0x4C, 0x89, 0x40, 0x18,
            0x48, 0x89, 0x50, 0x10,
            0x55,
            0x53,
            0x56
        };
        constexpr DWORD kVirtualMemoryCommitReserve = 0x00001000u | 0x00002000u;
        constexpr DWORD kVirtualMemoryRelease = 0x00008000u;
        constexpr DWORD kPageExecuteRead = 0x00000020u;
        constexpr DWORD kPageExecuteReadWrite = 0x00000040u;

        struct NativeBinaryRuntimeSettingState
        {
            RE::Setting* setting = nullptr;
            bool originalValue = false;
            bool originalCaptured = false;
            bool missingLogged = false;
            bool typeMismatchLogged = false;
            bool confirmedLogged = false;
            std::atomic<std::uint32_t> reapplyCount{ 0 };
        };

        struct NativeFloatRuntimeSettingState
        {
            RE::Setting* setting = nullptr;
            float originalValue = 0.0f;
            bool originalCaptured = false;
            bool missingLogged = false;
            bool typeMismatchLogged = false;
            bool confirmedLogged = false;
            std::atomic<std::uint32_t> reapplyCount{ 0 };
        };

        static std::atomic<std::uint64_t> g_nativeMeleeRuntimeSettingNextCheckFrame{ 0 };
        static NativeBinaryRuntimeSettingState g_nativeMeleeVelocityCheckState;
        static NativeFloatRuntimeSettingState g_nativeMeleeLinearThresholdState;
        static NativeFloatRuntimeSettingState g_nativeMeleeAngularThresholdState;

        bool isAddressInGameText(std::uintptr_t address)
        {
            const auto text = REL::Module::get().segment(REL::Segment::text);
            return address >= text.address() && address < text.address() + text.size();
        }

        bool validateNativeMeleeVtableTarget(std::uintptr_t entryOffset, std::uintptr_t expectedFunctionOffset, const char* label)
        {
            REL::Relocation<std::uintptr_t> entry{ REL::Offset(entryOffset) };
            auto* slot = reinterpret_cast<std::uintptr_t*>(entry.address());
            if (!slot) {
                ROCK_LOG_ERROR(Init, "{} vtable validation failed: slot is null", label);
                return false;
            }

            const auto current = *slot;
            const auto expected = REL::Offset(expectedFunctionOffset).address();
            if (!current) {
                ROCK_LOG_ERROR(Init, "{} vtable validation failed: current target is null", label);
                return false;
            }

            if (current == expected) {
                return true;
            }

            if (isAddressInGameText(current)) {
                ROCK_LOG_ERROR(Init, "{} vtable validation failed: slot 0x{:X} points to game text 0x{:X}, expected 0x{:X}", label, entry.address(), current, expected);
                return false;
            }

            ROCK_LOG_WARN(Init, "{} vtable slot 0x{:X} is already patched to external target 0x{:X}; ROCK will chain it if hook install proceeds", label, entry.address(), current);
            return true;
        }

        bool validateEntryTrampolineTarget(const char* label, std::uintptr_t targetOffset, const std::uint8_t* expectedPrefix, std::size_t stolenBytes)
        {
            if (stolenBytes < 14) {
                ROCK_LOG_ERROR(Init, "{} entry validation failed: stolen byte count {} cannot hold an absolute jump", label, stolenBytes);
                return false;
            }

            REL::Relocation<std::uintptr_t> target{ REL::Offset(targetOffset) };
            auto* targetAddr = reinterpret_cast<const std::uint8_t*>(target.address());
            if (!targetAddr || !expectedPrefix) {
                ROCK_LOG_ERROR(Init, "{} entry validation failed: target or validation bytes are null", label);
                return false;
            }

            if (std::memcmp(targetAddr, expectedPrefix, stolenBytes) != 0) {
                ROCK_LOG_ERROR(Init, "{} entry validation failed at 0x{:X}; native bytes changed", label, target.address());
                return false;
            }

            return true;
        }

        const RE::Actor* resolveNativePlayerActorGlobal()
        {
            static REL::Relocation<RE::Actor**> nativePlayerActor{ REL::Offset(offsets::kData_PlayerActorSingleton) };
            return *nativePlayerActor;
        }

        bool isPlayerActor(const RE::Actor* actor)
        {
            if (!actor) {
                return false;
            }

            const auto* commonLibPlayer = RE::PlayerCharacter::GetSingleton();
            const auto* nativePlayerActor = resolveNativePlayerActorGlobal();
            const auto* actorPtr = reinterpret_cast<const void*>(actor);
            const bool matchesCommonLib = commonLibPlayer && actorPtr == reinterpret_cast<const void*>(commonLibPlayer);
            const bool matchesNative = nativePlayerActor && actorPtr == reinterpret_cast<const void*>(nativePlayerActor);

            if (commonLibPlayer && nativePlayerActor && reinterpret_cast<const void*>(commonLibPlayer) != reinterpret_cast<const void*>(nativePlayerActor)) {
                static std::atomic<bool> loggedDivergence{ false };
                if (!loggedDivergence.exchange(true, std::memory_order_acq_rel)) {
                    ROCK_LOG_WARN(Combat,
                        "Native melee player pointer divergence: CommonLib player={:p}, native actor global={:p}; accepting either for suppression",
                        static_cast<const void*>(commonLibPlayer),
                        static_cast<const void*>(nativePlayerActor));
                }
            }

            return matchesCommonLib || matchesNative;
        }

        bool shouldSuppressNativeVrMeleeVelocity()
        {
            return g_nativeMeleeSuppressionHooksInstalled.load(std::memory_order_acquire) && g_rockConfig.rockEnabled && g_rockConfig.rockNativeMeleeSuppressionEnabled &&
                   g_rockConfig.rockNativeMeleeFullSuppression;
        }

        native_melee_suppression::NativeMeleeInputEvent classifyNativeMeleeInputEvent(const RE::InputEvent* event)
        {
            if (!event) {
                return native_melee_suppression::NativeMeleeInputEvent::Unknown;
            }

            const auto& userEvent = event->QUserEvent();
            const auto* userEventText = userEvent.c_str();
            const std::string_view userEventName{ userEventText ? userEventText : "", userEvent.length() };

            if (userEventName == "RightStick") {
                return native_melee_suppression::NativeMeleeInputEvent::RightStick;
            }
            if (userEventName == "PrimaryAttack") {
                return native_melee_suppression::NativeMeleeInputEvent::PrimaryAttack;
            }
            if (userEventName == "SecondaryAttack") {
                return native_melee_suppression::NativeMeleeInputEvent::SecondaryAttack;
            }

            return native_melee_suppression::NativeMeleeInputEvent::Unknown;
        }

        const char* nativeMeleeInputEventName(native_melee_suppression::NativeMeleeInputEvent event)
        {
            switch (event) {
            case native_melee_suppression::NativeMeleeInputEvent::RightStick:
                return "RightStick";
            case native_melee_suppression::NativeMeleeInputEvent::PrimaryAttack:
                return "PrimaryAttack";
            case native_melee_suppression::NativeMeleeInputEvent::SecondaryAttack:
                return "SecondaryAttack";
            case native_melee_suppression::NativeMeleeInputEvent::Unknown:
            default:
                return "Unknown";
            }
        }

        native_melee_suppression::NativeMeleeInputGatePolicyInput makeNativeMeleeInputGatePolicyInput(
            native_melee_suppression::NativeMeleeInputEvent event)
        {
            return native_melee_suppression::NativeMeleeInputGatePolicyInput{ .rockEnabled = g_rockConfig.rockEnabled,
                .suppressionEnabled = g_rockConfig.rockNativeMeleeSuppressionEnabled,
                .fullSuppression = g_rockConfig.rockNativeMeleeFullSuppression,
                .inputEvent = event };
        }

        native_melee_suppression::NativeMeleeImpactPolicyInput makeNativeMeleeImpactPolicyInput(const RE::Actor* actor)
        {
            return native_melee_suppression::NativeMeleeImpactPolicyInput{ .rockEnabled = g_rockConfig.rockEnabled,
                .suppressionEnabled = g_rockConfig.rockNativeMeleeSuppressionEnabled,
                .fullSuppression = g_rockConfig.rockNativeMeleeFullSuppression,
                .actorIsPlayer = isPlayerActor(actor) };
        }

        RE::Setting* resolveNativeMeleeRuntimeSetting(RE::Setting*& cachedSetting, const char* settingName, bool& missingLogged)
        {
            if (!cachedSetting) {
                cachedSetting = RE::GetINISetting(settingName);
            }

            if (!cachedSetting && !missingLogged) {
                missingLogged = true;
                ROCK_LOG_WARN(Combat, "Native VR melee suppression could not resolve INI setting '{}'", settingName);
            }

            return cachedSetting;
        }

        bool enforceNativeMeleeBinarySetting(NativeBinaryRuntimeSettingState& state, const char* settingName, bool desiredValue, const char* label)
        {
            auto* setting = resolveNativeMeleeRuntimeSetting(state.setting, settingName, state.missingLogged);
            if (!setting) {
                return false;
            }

            if (setting->GetType() != RE::Setting::SETTING_TYPE::kBinary) {
                if (!state.typeMismatchLogged) {
                    state.typeMismatchLogged = true;
                    ROCK_LOG_ERROR(Combat, "Native VR melee suppression found non-binary setting '{}'", settingName);
                }
                return false;
            }

            if (!state.originalCaptured) {
                state.originalValue = setting->GetBinary();
                state.originalCaptured = true;
                ROCK_LOG_INFO(Combat, "Native VR melee {} setting '{}' resolved: original={}", label, settingName, state.originalValue ? "true" : "false");
            }

            const bool currentValue = setting->GetBinary();
            if (currentValue != desiredValue) {
                setting->SetBinary(desiredValue);
                const auto reapplyCount = state.reapplyCount.fetch_add(1, std::memory_order_relaxed) + 1;
                if (reapplyCount == 1 || g_rockConfig.rockNativeMeleeDebugLogging || reapplyCount % 30 == 0) {
                    ROCK_LOG_WARN(Combat,
                        "Set FO4VR native VR melee {} setting '{}' to {} (original={} reapplyCount={})",
                        label,
                        settingName,
                        desiredValue ? "true" : "false",
                        state.originalValue ? "true" : "false",
                        reapplyCount);
                }
                state.confirmedLogged = true;
                return true;
            }

            if (!state.confirmedLogged) {
                ROCK_LOG_INFO(Combat, "FO4VR native VR melee {} setting '{}' is {}", label, settingName, desiredValue ? "true" : "false");
                state.confirmedLogged = true;
            }

            return true;
        }

        bool enforceNativeMeleeFloatMinimumSetting(NativeFloatRuntimeSettingState& state, const char* settingName, float desiredMinimum, const char* label)
        {
            auto* setting = resolveNativeMeleeRuntimeSetting(state.setting, settingName, state.missingLogged);
            if (!setting) {
                return false;
            }

            if (setting->GetType() != RE::Setting::SETTING_TYPE::kFloat) {
                if (!state.typeMismatchLogged) {
                    state.typeMismatchLogged = true;
                    ROCK_LOG_ERROR(Combat, "Native VR melee suppression found non-float setting '{}'", settingName);
                }
                return false;
            }

            if (!state.originalCaptured) {
                state.originalValue = setting->GetFloat();
                state.originalCaptured = true;
                ROCK_LOG_INFO(Combat, "Native VR melee {} setting '{}' resolved: original={:.3f}", label, settingName, state.originalValue);
            }

            const float currentValue = setting->GetFloat();
            if (!std::isfinite(currentValue) || currentValue < desiredMinimum) {
                setting->SetFloat(desiredMinimum);
                const auto reapplyCount = state.reapplyCount.fetch_add(1, std::memory_order_relaxed) + 1;
                if (reapplyCount == 1 || g_rockConfig.rockNativeMeleeDebugLogging || reapplyCount % 30 == 0) {
                    ROCK_LOG_WARN(Combat,
                        "Raised FO4VR native VR melee {} setting '{}' to {:.1f} (original={:.3f} reapplyCount={})",
                        label,
                        settingName,
                        desiredMinimum,
                        state.originalValue,
                        reapplyCount);
                }
                state.confirmedLogged = true;
                return true;
            }

            if (!state.confirmedLogged) {
                ROCK_LOG_INFO(Combat, "FO4VR native VR melee {} setting '{}' is armed at {:.1f}", label, settingName, currentValue);
                state.confirmedLogged = true;
            }

            return true;
        }

        bool isLeftSideString(const RE::BSFixedString* side)
        {
            if (!side) {
                return false;
            }

            const char* text = side->c_str();
            if (!text) {
                return false;
            }

            return std::string_view(text) == "Left";
        }

        bool isAnyNativeMeleePhysicalSwingActive()
        {
            const auto currentFrame = g_nativeMeleeFrameClock.load(std::memory_order_acquire);
            return native_melee_suppression::isPhysicalSwingLeaseActive(currentFrame, g_nativeMeleePhysicalSwingExpiresAtFrame[0].load(std::memory_order_acquire)) ||
                   native_melee_suppression::isPhysicalSwingLeaseActive(currentFrame, g_nativeMeleePhysicalSwingExpiresAtFrame[1].load(std::memory_order_acquire));
        }

        bool isNativeMeleePhysicalSwingActiveForSide(const RE::BSFixedString* side)
        {
            if (!side) {
                return isAnyNativeMeleePhysicalSwingActive();
            }

            const bool isLeft = isLeftSideString(side);
            const auto currentFrame = g_nativeMeleeFrameClock.load(std::memory_order_acquire);
            return native_melee_suppression::isPhysicalSwingLeaseActive(
                currentFrame, g_nativeMeleePhysicalSwingExpiresAtFrame[isLeft ? 1 : 0].load(std::memory_order_acquire));
        }

        native_melee_suppression::NativeMeleePolicyInput makeNativeMeleePolicyInput(
            const native_melee_suppression::NativeMeleeEvent event, const RE::Actor* actor, const RE::BSFixedString* side)
        {
            return native_melee_suppression::NativeMeleePolicyInput{ .rockEnabled = g_rockConfig.rockEnabled,
                .suppressionEnabled = g_rockConfig.rockNativeMeleeSuppressionEnabled,
                .fullSuppression = g_rockConfig.rockNativeMeleeFullSuppression,
                .suppressWeaponSwing = g_rockConfig.rockNativeMeleeSuppressWeaponSwing,
                .suppressHitFrame = g_rockConfig.rockNativeMeleeSuppressHitFrame,
                .actorIsPlayer = isPlayerActor(actor),
                .physicalSwingActive = event == native_melee_suppression::NativeMeleeEvent::HitFrame ? isNativeMeleePhysicalSwingActiveForSide(side)
                                                                                                     : isAnyNativeMeleePhysicalSwingActive() };
        }

        bool applyNativeMeleeDecision(const native_melee_suppression::NativeMeleeEvent event,
            const native_melee_suppression::NativeMeleePolicyInput& input,
            const native_melee_suppression::NativeMeleePolicyDecision& decision)
        {
            using native_melee_suppression::NativeMeleeEvent;
            using native_melee_suppression::NativeMeleeSuppressionAction;

            if (g_rockConfig.rockNativeMeleeDebugLogging || decision.action != NativeMeleeSuppressionAction::CallNative) {
                static std::atomic<std::uint32_t> weaponLogCounter{ 0 };
                static std::atomic<std::uint32_t> hitFrameLogCounter{ 0 };
                auto& counter = event == NativeMeleeEvent::WeaponSwing ? weaponLogCounter : hitFrameLogCounter;
                const auto count = counter.fetch_add(1, std::memory_order_relaxed) + 1;

                if (count == 1 || (g_rockConfig.rockNativeMeleeDebugLogging && count % 45 == 0) || count % 180 == 0) {
                    ROCK_LOG_DEBUG(Combat, "Native melee {} decision={} reason={} player={} physicalSwing={} count={}",
                        event == NativeMeleeEvent::WeaponSwing ? "WeaponSwing" : "HitFrame",
                        decision.action == NativeMeleeSuppressionAction::CallNative      ? "native"
                            : decision.action == NativeMeleeSuppressionAction::ReturnHandled ? "handled"
                                                                                              : "unhandled",
                        decision.reason, input.actorIsPlayer ? "yes" : "no", input.physicalSwingActive ? "yes" : "no", count);
                }
            }

            switch (decision.action) {
            case native_melee_suppression::NativeMeleeSuppressionAction::CallNative:
                return true;
            case native_melee_suppression::NativeMeleeSuppressionAction::ReturnHandled:
                return true;
            case native_melee_suppression::NativeMeleeSuppressionAction::ReturnUnhandled:
                return false;
            }

            return true;
        }

        bool applyNativeMeleeImpactDecision(const native_melee_suppression::NativeMeleeImpactPolicyInput& input,
            const native_melee_suppression::NativeMeleeImpactPolicyDecision& decision)
        {
            using native_melee_suppression::NativeMeleeImpactAction;

            if (g_rockConfig.rockNativeMeleeDebugLogging || decision.action != NativeMeleeImpactAction::CallNative) {
                static std::atomic<std::uint32_t> impactLogCounter{ 0 };
                const auto count = impactLogCounter.fetch_add(1, std::memory_order_relaxed) + 1;
                if (count == 1 || (g_rockConfig.rockNativeMeleeDebugLogging && count % 45 == 0) || count % 180 == 0) {
                    ROCK_LOG_DEBUG(Combat,
                        "Native melee VRMeleeImpact decision={} reason={} player={} full={} count={}",
                        decision.action == NativeMeleeImpactAction::CallNative ? "native" : "suppressed",
                        decision.reason,
                        input.actorIsPlayer ? "yes" : "no",
                        input.fullSuppression ? "yes" : "no",
                        count);
                }
            }

            return decision.action == NativeMeleeImpactAction::Suppress;
        }

        bool hookedWeaponSwingHandler(void* handler, RE::Actor* actor, RE::BSFixedString* side)
        {
            const auto input = makeNativeMeleePolicyInput(native_melee_suppression::NativeMeleeEvent::WeaponSwing, actor, side);
            const auto decision = native_melee_suppression::evaluateNativeMeleeSuppression(native_melee_suppression::NativeMeleeEvent::WeaponSwing, input);

            const bool shouldCallNative = decision.action == native_melee_suppression::NativeMeleeSuppressionAction::CallNative;
            const bool decisionResult = applyNativeMeleeDecision(native_melee_suppression::NativeMeleeEvent::WeaponSwing, input, decision);
            return shouldCallNative ? (g_originalWeaponSwingHandler ? g_originalWeaponSwingHandler(handler, actor, side) : false) : decisionResult;
        }

        bool hookedHitFrameHandler(void* handler, RE::Actor* actor, RE::BSFixedString* side)
        {
            const auto input = makeNativeMeleePolicyInput(native_melee_suppression::NativeMeleeEvent::HitFrame, actor, side);
            const auto decision = native_melee_suppression::evaluateNativeMeleeSuppression(native_melee_suppression::NativeMeleeEvent::HitFrame, input);

            const bool shouldCallNative = decision.action == native_melee_suppression::NativeMeleeSuppressionAction::CallNative;
            const bool decisionResult = applyNativeMeleeDecision(native_melee_suppression::NativeMeleeEvent::HitFrame, input, decision);
            return shouldCallNative ? (g_originalHitFrameHandler ? g_originalHitFrameHandler(handler, actor, side) : false) : decisionResult;
        }

        void hookedPlayerWeaponSwingCallback(RE::Actor* actor, std::uint32_t equipIndex)
        {
            /*
             * FO4VR can reach PlayerCharacter::WeaponSwingCallBack separately
             * from the WeaponSwing animation handler vtable. Ghidra verified the
             * PlayerCharacter vtable slot and showed that the callback dispatches
             * weapon-swing side effects, so full native suppression must stop it
             * at this player-only boundary without changing NPC swing behavior.
             */
            const auto input = makeNativeMeleePolicyInput(native_melee_suppression::NativeMeleeEvent::WeaponSwing, actor, nullptr);
            const auto decision = native_melee_suppression::evaluateNativeMeleeSuppression(native_melee_suppression::NativeMeleeEvent::WeaponSwing, input);
            const bool shouldCallNative = decision.action == native_melee_suppression::NativeMeleeSuppressionAction::CallNative;

            if (!shouldCallNative) {
                applyNativeMeleeDecision(native_melee_suppression::NativeMeleeEvent::WeaponSwing, input, decision);
                return;
            }

            if (g_originalPlayerWeaponSwingCallback) {
                g_originalPlayerWeaponSwingCallback(actor, equipIndex);
            }
        }

        void hookedVrMeleeImpactCallback(RE::Actor* actor, void* contactEvent, void* collisionEvent)
        {
            /*
             * FO4VR registers this callback while attaching native VR melee
             * collision to the first-person weapon nodes. It owns the native
             * contact-to-hit path, including target filtering, action dispatch,
             * impulse direction, and melee cooldown writes. Full ROCK native
             * suppression skips the player callback here so SCISSORS can own the
             * replacement point-collision damage path without a duplicate native
             * impact firing from the same swing.
             */
            const auto input = makeNativeMeleeImpactPolicyInput(actor);
            const auto decision = native_melee_suppression::evaluateNativeMeleeImpactSuppression(input);

            if (applyNativeMeleeImpactDecision(input, decision)) {
                ROCK_LOG_SAMPLE_DEBUG(Combat,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed FO4VR native VRMeleeImpact callback actor={:p} contactEvent={:p} collisionEvent={:p}",
                    static_cast<void*>(actor),
                    contactEvent,
                    collisionEvent);
                return;
            }

            if (g_originalVrMeleeImpactCallback) {
                g_originalVrMeleeImpactCallback(actor, contactEvent, collisionEvent);
            }
        }

        bool hookedAttackBlockShouldHandleEvent(void* handler, const RE::InputEvent* event)
        {
            /*
             * FO4VR has a third native melee path before the animation events:
             * AttackBlockHandler::ShouldHandleEvent accepts the VR RightStick
             * input and can classify fast rotation as melee. The hook is kept
             * at this input gate and only returns false for RightStick during
             * full ROCK suppression; PrimaryAttack and SecondaryAttack remain
             * native so normal weapon inputs are not segregated or broken.
             */
            const auto inputEvent = classifyNativeMeleeInputEvent(event);
            const auto policyInput = makeNativeMeleeInputGatePolicyInput(inputEvent);
            const auto decision = native_melee_suppression::evaluateNativeMeleeInputGate(policyInput);

            if (g_rockConfig.rockNativeMeleeDebugLogging || decision.action != native_melee_suppression::NativeMeleeInputGateAction::CallNative) {
                static std::atomic<std::uint32_t> inputGateLogCounter{ 0 };
                const auto count = inputGateLogCounter.fetch_add(1, std::memory_order_relaxed) + 1;
                if (count == 1 || (g_rockConfig.rockNativeMeleeDebugLogging && count % 45 == 0) || count % 180 == 0) {
                    ROCK_LOG_DEBUG(Combat,
                        "Native melee AttackBlock input gate event={} decision={} reason={} count={}",
                        nativeMeleeInputEventName(inputEvent),
                        decision.action == native_melee_suppression::NativeMeleeInputGateAction::CallNative ? "native" : "false",
                        decision.reason,
                        count);
                }
            }

            if (decision.action == native_melee_suppression::NativeMeleeInputGateAction::ReturnFalse) {
                return false;
            }

            return g_originalAttackBlockShouldHandleEvent ? g_originalAttackBlockShouldHandleEvent(handler, event) : false;
        }

        template <class HandlerT>
        bool installNativeMeleeVtableHook(std::uintptr_t entryOffset, HandlerT hook, HandlerT& original, const char* label)
        {
            REL::Relocation<std::uintptr_t> entry{ REL::Offset(entryOffset) };
            auto* slot = reinterpret_cast<std::uintptr_t*>(entry.address());
            if (!slot) {
                ROCK_LOG_ERROR(Init, "FAILED to install {} hook: vtable slot is null", label);
                return false;
            }

            const auto hookAddress = reinterpret_cast<std::uintptr_t>(hook);
            const auto currentTarget = *slot;
            if (currentTarget == hookAddress) {
                ROCK_LOG_INFO(Init, "{} hook already installed at 0x{:X}", label, entry.address());
                return original != nullptr;
            }

            original = reinterpret_cast<HandlerT>(currentTarget);
            if (!original) {
                ROCK_LOG_ERROR(Init, "FAILED to install {} hook at 0x{:X}: original is null", label, entry.address());
                return false;
            }

            DWORD oldProtect = 0;
            if (!VirtualProtect(slot, sizeof(*slot), kPageExecuteReadWrite, &oldProtect)) {
                ROCK_LOG_ERROR(Init, "FAILED to install {} hook at 0x{:X}: VirtualProtect failed", label, entry.address());
                original = nullptr;
                return false;
            }

            *slot = hookAddress;
            FlushInstructionCache(GetCurrentProcess(), slot, sizeof(*slot));
            VirtualProtect(slot, sizeof(*slot), oldProtect, &oldProtect);

            ROCK_LOG_INFO(Init, "Installed {} vtable hook at 0x{:X}, original=0x{:X}, hook=0x{:X}", label, entry.address(), reinterpret_cast<std::uintptr_t>(original),
                hookAddress);
            return true;
        }

        template <class HandlerT>
        bool restoreNativeMeleeVtableHook(std::uintptr_t entryOffset, HandlerT hook, HandlerT& original, bool& installed, const char* label)
        {
            if (!installed) {
                return true;
            }

            REL::Relocation<std::uintptr_t> entry{ REL::Offset(entryOffset) };
            auto* slot = reinterpret_cast<std::uintptr_t*>(entry.address());
            if (!slot || !original) {
                ROCK_LOG_ERROR(Init, "FAILED to restore {} hook: slot or original is null", label);
                return false;
            }

            const auto hookAddress = reinterpret_cast<std::uintptr_t>(hook);
            const auto originalAddress = reinterpret_cast<std::uintptr_t>(original);
            if (*slot != hookAddress) {
                ROCK_LOG_WARN(Init, "Skipped {} rollback: slot 0x{:X} no longer points to ROCK hook", label, entry.address());
                installed = false;
                original = nullptr;
                return true;
            }

            DWORD oldProtect = 0;
            if (!VirtualProtect(slot, sizeof(*slot), kPageExecuteReadWrite, &oldProtect)) {
                ROCK_LOG_ERROR(Init, "FAILED to restore {} hook at 0x{:X}: VirtualProtect failed", label, entry.address());
                return false;
            }

            *slot = originalAddress;
            FlushInstructionCache(GetCurrentProcess(), slot, sizeof(*slot));
            VirtualProtect(slot, sizeof(*slot), oldProtect, &oldProtect);

            ROCK_LOG_WARN(Init, "Rolled back {} vtable hook at 0x{:X}", label, entry.address());
            installed = false;
            original = nullptr;
            return true;
        }
    }

    bool validateNativeMeleeSuppressionHookTargets()
    {
        const bool swingValid = validateNativeMeleeVtableTarget(
            offsets::kVtableEntry_WeaponSwingHandler_Handle, offsets::kFunc_WeaponSwingHandler_Handle, "WeaponSwingHandler::Handle");
        const bool hitFrameValid =
            validateNativeMeleeVtableTarget(offsets::kVtableEntry_HitFrameHandler_Handle, offsets::kFunc_HitFrameHandler_Handle, "HitFrameHandler::Handle");
        const bool attackBlockValid = validateNativeMeleeVtableTarget(offsets::kVtableEntry_AttackBlockHandler_ShouldHandleEvent,
            offsets::kFunc_AttackBlockHandler_ShouldHandleEvent,
            "AttackBlockHandler::ShouldHandleEvent");
        const bool playerSwingCallbackValid = validateNativeMeleeVtableTarget(offsets::kVtableEntry_PlayerCharacter_WeaponSwingCallBack,
            offsets::kFunc_PlayerCharacter_WeaponSwingCallBack,
            "PlayerCharacter::WeaponSwingCallBack");
        const bool vrMeleeImpactValid = validateEntryTrampolineTarget(
            "VRMeleeImpact", offsets::kFunc_VRMeleeImpactCallback, kVrMeleeImpactExpectedPrefix.data(), kVrMeleeImpactExpectedPrefix.size());
        return swingValid && hitFrameValid && attackBlockValid && playerSwingCallbackValid && vrMeleeImpactValid;
    }

    void setNativeMeleePhysicalSwingActive(bool isLeft, bool active)
    {
        const auto currentFrame = g_nativeMeleeFrameClock.load(std::memory_order_acquire);
        const auto expiresAtFrame = active ? (currentFrame + kNativeMeleePhysicalSwingLeaseFrames) : 0;
        g_nativeMeleePhysicalSwingExpiresAtFrame[isLeft ? 1 : 0].store(expiresAtFrame, std::memory_order_release);
    }

    bool isNativeMeleePhysicalSwingActive(bool isLeft)
    {
        const auto currentFrame = g_nativeMeleeFrameClock.load(std::memory_order_acquire);
        return native_melee_suppression::isPhysicalSwingLeaseActive(
            currentFrame, g_nativeMeleePhysicalSwingExpiresAtFrame[isLeft ? 1 : 0].load(std::memory_order_acquire));
    }

    void advanceNativeMeleeFrameClock()
    {
        g_nativeMeleeFrameClock.fetch_add(1, std::memory_order_acq_rel);
    }

    void clearNativeMeleePhysicalSwingLeases()
    {
        g_nativeMeleePhysicalSwingExpiresAtFrame[0].store(0, std::memory_order_release);
        g_nativeMeleePhysicalSwingExpiresAtFrame[1].store(0, std::memory_order_release);
    }

    void enforceNativeMeleeRuntimeSuppression(bool forceCheck)
    {
        /*
         * FO4VR owns two native player melee paths: animation events
         * (WeaponSwing/HitFrame) and a VRInput velocity gate that can classify
         * controller or HMD motion as melee. Full suppression keeps that gate
         * enabled and pushes its thresholds out of reach; disabling the boolean
         * bypasses the gate on some builds and causes cooldown-paced false melee
         * swings.
         */
        const auto currentFrame = g_nativeMeleeFrameClock.load(std::memory_order_acquire);
        if (!shouldSuppressNativeVrMeleeVelocity()) {
            return;
        }

        const auto nextCheckFrame = g_nativeMeleeRuntimeSettingNextCheckFrame.load(std::memory_order_acquire);
        if (!forceCheck && currentFrame < nextCheckFrame) {
            return;
        }
        g_nativeMeleeRuntimeSettingNextCheckFrame.store(currentFrame + kNativeMeleeRuntimeSettingCheckIntervalFrames, std::memory_order_release);

        enforceNativeMeleeBinarySetting(g_nativeMeleeVelocityCheckState, kNativeMeleeVelocityCheckSetting, true, "velocity gate");
        enforceNativeMeleeFloatMinimumSetting(
            g_nativeMeleeLinearThresholdState, kNativeMeleeLinearVelocityThresholdSetting, kNativeMeleeSuppressedVelocityThreshold, "linear threshold");
        enforceNativeMeleeFloatMinimumSetting(
            g_nativeMeleeAngularThresholdState, kNativeMeleeAngularVelocityThresholdSetting, kNativeMeleeSuppressedVelocityThreshold, "angular threshold");
    }

    using HandleBumpedCharacter_t = void (*)(void*, void*, void*);
    static HandleBumpedCharacter_t g_originalHandleBumped = nullptr;

    static void writeAbsoluteJump(std::uint8_t* target, std::uintptr_t destination)
    {
        target[0] = 0xFF;
        target[1] = 0x25;
        target[2] = 0x00;
        target[3] = 0x00;
        target[4] = 0x00;
        target[5] = 0x00;
        *reinterpret_cast<std::uintptr_t*>(target + 6) = destination;
    }

    static bool installEntryTrampolineHook(const char* label,
        std::uintptr_t targetOffset,
        const std::uint8_t* expectedPrefix,
        std::size_t stolenBytes,
        void* hook,
        void*& original)
    {
        if (stolenBytes < 14) {
            ROCK_LOG_ERROR(Init, "{} hook install failed: stolen byte count {} cannot hold an absolute jump", label, stolenBytes);
            return false;
        }

        REL::Relocation<std::uintptr_t> target{ REL::Offset(targetOffset) };
        auto* targetAddr = reinterpret_cast<std::uint8_t*>(target.address());
        if (!targetAddr || !expectedPrefix) {
            ROCK_LOG_ERROR(Init, "{} hook install failed: target or validation bytes are null", label);
            return false;
        }

        if (std::memcmp(targetAddr, expectedPrefix, stolenBytes) != 0) {
            ROCK_LOG_ERROR(Init, "{} hook validation failed at 0x{:X}; native bytes changed, hook not installed", label, target.address());
            return false;
        }

        constexpr std::size_t kJumpBytes = 14;
        const std::size_t trampolineBytes = stolenBytes + kJumpBytes;
        auto* trampolineMem = reinterpret_cast<std::uint8_t*>(VirtualAlloc(nullptr, trampolineBytes, kVirtualMemoryCommitReserve, kPageExecuteReadWrite));
        if (!trampolineMem) {
            ROCK_LOG_ERROR(Init, "{} hook install failed: trampoline allocation failed", label);
            return false;
        }

        std::memcpy(trampolineMem, targetAddr, stolenBytes);
        writeAbsoluteJump(trampolineMem + stolenBytes, target.address() + stolenBytes);

        DWORD oldTrampolineProtect = 0;
        if (!VirtualProtect(trampolineMem, trampolineBytes, kPageExecuteRead, &oldTrampolineProtect)) {
            ROCK_LOG_ERROR(Init, "{} hook install failed: trampoline protection failed", label);
            VirtualFree(trampolineMem, 0, kVirtualMemoryRelease);
            return false;
        }

        DWORD oldProtect = 0;
        if (!VirtualProtect(targetAddr, stolenBytes, kPageExecuteReadWrite, &oldProtect)) {
            ROCK_LOG_ERROR(Init, "{} hook install failed at 0x{:X}: target protection failed", label, target.address());
            VirtualFree(trampolineMem, 0, kVirtualMemoryRelease);
            return false;
        }

        writeAbsoluteJump(targetAddr, reinterpret_cast<std::uintptr_t>(hook));
        for (std::size_t i = kJumpBytes; i < stolenBytes; ++i) {
            targetAddr[i] = 0x90;
        }

        FlushInstructionCache(GetCurrentProcess(), targetAddr, stolenBytes);
        VirtualProtect(targetAddr, stolenBytes, oldProtect, &oldProtect);

        original = trampolineMem;
        ROCK_LOG_INFO(Init, "Installed {} hook at 0x{:X}, original trampoline=0x{:X}", label, target.address(), reinterpret_cast<std::uintptr_t>(trampolineMem));
        return true;
    }

    static bool restoreEntryTrampolineHook(const char* label,
        std::uintptr_t targetOffset,
        const std::uint8_t* originalPrefix,
        std::size_t stolenBytes,
        void* hook,
        void*& original,
        bool& installed)
    {
        if (!installed) {
            return true;
        }

        REL::Relocation<std::uintptr_t> target{ REL::Offset(targetOffset) };
        auto* targetAddr = reinterpret_cast<std::uint8_t*>(target.address());
        if (!targetAddr || !originalPrefix || !original) {
            ROCK_LOG_ERROR(Init, "{} rollback failed: target, original bytes, or trampoline is null", label);
            return false;
        }

        const auto hookAddress = reinterpret_cast<std::uintptr_t>(hook);
        const bool hasRockJump = targetAddr[0] == 0xFF && targetAddr[1] == 0x25 && targetAddr[2] == 0x00 && targetAddr[3] == 0x00 && targetAddr[4] == 0x00 &&
                                 targetAddr[5] == 0x00 && *reinterpret_cast<std::uintptr_t*>(targetAddr + 6) == hookAddress;
        if (!hasRockJump) {
            ROCK_LOG_WARN(Init, "Skipped {} rollback: target 0x{:X} no longer points to ROCK hook", label, target.address());
            installed = false;
            original = nullptr;
            return true;
        }

        DWORD oldProtect = 0;
        if (!VirtualProtect(targetAddr, stolenBytes, kPageExecuteReadWrite, &oldProtect)) {
            ROCK_LOG_ERROR(Init, "{} rollback failed at 0x{:X}: target protection failed", label, target.address());
            return false;
        }

        std::memcpy(targetAddr, originalPrefix, stolenBytes);
        FlushInstructionCache(GetCurrentProcess(), targetAddr, stolenBytes);
        VirtualProtect(targetAddr, stolenBytes, oldProtect, &oldProtect);

        VirtualFree(original, 0, kVirtualMemoryRelease);
        original = nullptr;
        installed = false;
        ROCK_LOG_WARN(Init, "Rolled back {} entry hook at 0x{:X}", label, target.address());
        return true;
    }

    void* resolvePlayerCharacterController()
    {
        return static_cast<void*>(character_controller_runtime::tryGetPlayerCharacterController());
    }

    bool isPlayerCharacterController(void* controller)
    {
        void* playerController = resolvePlayerCharacterController();
        return controller && playerController && controller == playerController;
    }

    RE::bhkWorld* resolvePlayerBhkWorld()
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* cell = player ? player->GetParentCell() : nullptr;
        return cell ? cell->GetbhkWorld() : nullptr;
    }

    physics_body_classifier::BodyMotionType resolveBodyMotionType(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        auto* body = havok_runtime::getBody(world, bodyId);
        if (!body) {
            return physics_body_classifier::BodyMotionType::Unknown;
        }

        auto motionType = physics_body_classifier::motionTypeFromBodyFlags(body->flags);
        if (motionType == physics_body_classifier::BodyMotionType::Unknown) {
            motionType = physics_body_classifier::motionTypeFromMotionPropertiesId(static_cast<std::uint16_t>(body->motionPropertiesId));
        }
        return motionType;
    }

    bool isDynamicMovableStaticPlayerContactTarget(RE::bhkWorld* bhkWorld, RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t layer)
    {
        if (!bhkWorld || !world || !collision_layer_policy::isPlayerCharacterControllerSupportLayer(layer)) {
            return false;
        }

        /*
         * Most preserved player-controller contacts are static world support.
         * Resolve the expensive scene reference only after runtime body state
         * proves this support-layer body is actually dynamic.
         */
        if (resolveBodyMotionType(world, bodyId) != physics_body_classifier::BodyMotionType::Dynamic) {
            return false;
        }

        auto* ref = resolveBodyToRef(bhkWorld, world, bodyId);
        auto* baseForm = ref ? ref->GetObjectReference() : nullptr;
        return baseForm && baseForm->Is(RE::ENUM_FORM_ID::kMSTT);
    }

    collision_layer_policy::PlayerCharacterControllerContactPolicyDecision evaluatePlayerControllerTargetBody(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* world,
        std::uint32_t rawBodyId)
    {
        if (!world || rawBodyId == body_frame::kInvalidBodyId) {
            return collision_layer_policy::PlayerCharacterControllerContactPolicyDecision{ .suppress = false, .reason = "unknownTargetLayer" };
        }

        const RE::hknpBodyId bodyId{ rawBodyId };
        std::uint32_t filterInfo = 0;
        if (!body_collision::tryReadFilterInfo(world, bodyId, filterInfo)) {
            return collision_layer_policy::PlayerCharacterControllerContactPolicyDecision{ .suppress = false, .reason = "unknownTargetLayer" };
        }

        const std::uint32_t layer = filterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
        return collision_layer_policy::evaluatePlayerCharacterControllerContact(
            collision_layer_policy::PlayerCharacterControllerContactPolicyInput{
                .filterEnabled = true,
                .playerController = true,
                .targetLayerKnown = true,
                .targetLayer = layer,
                .targetIsDynamicMovableStatic = isDynamicMovableStaticPlayerContactTarget(bhkWorld, world, bodyId, layer),
            });
    }

    void hookedHandleBumpedCharacter(void* controller, void* bumpedCC, void* contactInfo)
    {
        bool originalAttempted = false;
        if (!PhysicsInteraction::s_hooksEnabled.load(std::memory_order_acquire)) {
            if (g_originalHandleBumped) {
                originalAttempted = true;
                g_originalHandleBumped(controller, bumpedCC, contactInfo);
            }
            return;
        }

        __try {
            const auto decision = collision_layer_policy::evaluatePlayerCharacterControllerContact(
                collision_layer_policy::PlayerCharacterControllerContactPolicyInput{
                    .filterEnabled = g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled,
                    .playerController = isPlayerCharacterController(controller),
                    .targetLayerKnown = bumpedCC != nullptr,
                    .targetLayer = collision_layer_policy::FO4_LAYER_CHARCONTROLLER,
                });

            if (decision.suppress) {
                ROCK_LOG_SAMPLE_DEBUG(Bump,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed player HandleBumpedCharacter target={:p} reason={}",
                    bumpedCC,
                    decision.reason);
                return;
            }

            if (g_originalHandleBumped) {
                originalAttempted = true;
                g_originalHandleBumped(controller, bumpedCC, contactInfo);
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            static int sehLogCounter = 0;
            if (sehLogCounter++ % 100 == 0) {
                logger::error(
                    "[ROCK::Bump] SEH exception caught on physics thread (count={}) — "
                    "trampoline or stale pointer issue",
                    sehLogCounter);
            }
            if (!originalAttempted && g_originalHandleBumped) {
                __try {
                    originalAttempted = true;
                    g_originalHandleBumped(controller, bumpedCC, contactInfo);
                } __except (EXCEPTION_EXECUTE_HANDLER) {
                }
            }
        }
    }

    void installBumpHook()
    {
        static bool installed = false;
        static bool installAttempted = false;
        if (installed)
            return;
        if (installAttempted)
            return;
        installAttempted = true;

        // Ghidra verified HandleBumpedCharacter at 0x141E24980 starts with ordinary
        // prologue instructions, not an existing branch/call. CommonLib write_branch
        // cannot derive a callable original from those bytes, so this hook uses an
        // explicit relocated-entry trampoline and validates the exact whole
        // instructions before patching.
        constexpr std::array<std::uint8_t, 15> expectedPrefix{
            0x48, 0x89, 0x5C, 0x24, 0x08,
            0x48, 0x89, 0x74, 0x24, 0x18,
            0x57,
            0x48, 0x83, 0xEC, 0x70
        };

        void* original = reinterpret_cast<void*>(g_originalHandleBumped);
        installed = installEntryTrampolineHook(
            "HandleBumpedCharacter", offsets::kFunc_HandleBumpedCharacter, expectedPrefix.data(), expectedPrefix.size(), &hookedHandleBumpedCharacter, original);
        g_originalHandleBumped = reinterpret_cast<HandleBumpedCharacter_t>(original);
    }

    void installNativeGrabHook()
    {
        static bool installed = false;
        if (installed)
            return;
        installed = true;

        static REL::Relocation<std::uintptr_t> target{ REL::Offset(offsets::kFunc_VRGrabInitiate) };
        auto* addr = reinterpret_cast<std::uint8_t*>(target.address());

        DWORD oldProtect;
        if (VirtualProtect(addr, 3, kPageExecuteReadWrite, &oldProtect)) {
            addr[0] = 0x31;
            addr[1] = 0xC0;
            addr[2] = 0xC3;
            VirtualProtect(addr, 3, oldProtect, &oldProtect);
            ROCK_LOG_INFO(Init, "Patched VR Grab Initiate at 0x{:X} — native grab DISABLED (xor eax,eax; ret)", target.address());
        } else {
            ROCK_LOG_ERROR(Init, "FAILED to patch VR Grab Initiate at 0x{:X} — VirtualProtect failed", target.address());
        }
    }

    bool installNativeMeleeSuppressionHooks()
    {
        static bool weaponSwingInstalled = false;
        static bool hitFrameInstalled = false;
        static bool attackBlockInstalled = false;
        static bool playerWeaponSwingCallbackInstalled = false;
        static bool vrMeleeImpactInstalled = false;

        auto rollbackNativeMeleeSuppressionHooks = [&]() {
            bool rollbackOk = true;

            rollbackOk = restoreNativeMeleeVtableHook(offsets::kVtableEntry_PlayerCharacter_WeaponSwingCallBack,
                             &hookedPlayerWeaponSwingCallback,
                             g_originalPlayerWeaponSwingCallback,
                             playerWeaponSwingCallbackInstalled,
                             "PlayerCharacter::WeaponSwingCallBack") &&
                         rollbackOk;
            rollbackOk = restoreNativeMeleeVtableHook(offsets::kVtableEntry_AttackBlockHandler_ShouldHandleEvent,
                             &hookedAttackBlockShouldHandleEvent,
                             g_originalAttackBlockShouldHandleEvent,
                             attackBlockInstalled,
                             "AttackBlockHandler::ShouldHandleEvent") &&
                         rollbackOk;
            rollbackOk = restoreNativeMeleeVtableHook(
                             offsets::kVtableEntry_HitFrameHandler_Handle, &hookedHitFrameHandler, g_originalHitFrameHandler, hitFrameInstalled, "HitFrameHandler::Handle") &&
                         rollbackOk;
            rollbackOk = restoreNativeMeleeVtableHook(offsets::kVtableEntry_WeaponSwingHandler_Handle,
                             &hookedWeaponSwingHandler,
                             g_originalWeaponSwingHandler,
                             weaponSwingInstalled,
                             "WeaponSwingHandler::Handle") &&
                         rollbackOk;

            void* impactOriginal = reinterpret_cast<void*>(g_originalVrMeleeImpactCallback);
            rollbackOk = restoreEntryTrampolineHook("VRMeleeImpact",
                             offsets::kFunc_VRMeleeImpactCallback,
                             kVrMeleeImpactExpectedPrefix.data(),
                             kVrMeleeImpactExpectedPrefix.size(),
                             &hookedVrMeleeImpactCallback,
                             impactOriginal,
                             vrMeleeImpactInstalled) &&
                         rollbackOk;
            g_originalVrMeleeImpactCallback = reinterpret_cast<NativeVrMeleeImpactCallback_t>(impactOriginal);

            if (!rollbackOk) {
                ROCK_LOG_CRITICAL(Init, "Native melee suppression rollback was incomplete; one or more partial hooks may remain installed");
            }

            g_nativeMeleeSuppressionHooksInstalled.store(false, std::memory_order_release);
            return rollbackOk;
        };

        const bool allInstalled = weaponSwingInstalled && hitFrameInstalled && attackBlockInstalled && playerWeaponSwingCallbackInstalled && vrMeleeImpactInstalled;
        if (allInstalled) {
            g_nativeMeleeSuppressionHooksInstalled.store(true, std::memory_order_release);
            return true;
        }

        const bool anyInstalled = weaponSwingInstalled || hitFrameInstalled || attackBlockInstalled || playerWeaponSwingCallbackInstalled || vrMeleeImpactInstalled;
        if (anyInstalled) {
            ROCK_LOG_ERROR(Init, "Native melee suppression hook set is partial before install; rolling back before retry");
            rollbackNativeMeleeSuppressionHooks();
            return false;
        }

        if (!validateNativeMeleeSuppressionHookTargets()) {
            ROCK_LOG_ERROR(Init, "Native melee suppression hook validation failed; install deferred");
            g_nativeMeleeSuppressionHooksInstalled.store(false, std::memory_order_release);
            return false;
        }

        if (!vrMeleeImpactInstalled) {
            void* impactOriginal = reinterpret_cast<void*>(g_originalVrMeleeImpactCallback);
            vrMeleeImpactInstalled = installEntryTrampolineHook("VRMeleeImpact",
                offsets::kFunc_VRMeleeImpactCallback,
                kVrMeleeImpactExpectedPrefix.data(),
                kVrMeleeImpactExpectedPrefix.size(),
                &hookedVrMeleeImpactCallback,
                impactOriginal);
            g_originalVrMeleeImpactCallback = reinterpret_cast<NativeVrMeleeImpactCallback_t>(impactOriginal);
        }
        if (!weaponSwingInstalled) {
            weaponSwingInstalled = installNativeMeleeVtableHook(
                offsets::kVtableEntry_WeaponSwingHandler_Handle, &hookedWeaponSwingHandler, g_originalWeaponSwingHandler, "WeaponSwingHandler::Handle");
        }
        if (!hitFrameInstalled) {
            hitFrameInstalled =
                installNativeMeleeVtableHook(offsets::kVtableEntry_HitFrameHandler_Handle, &hookedHitFrameHandler, g_originalHitFrameHandler, "HitFrameHandler::Handle");
        }
        if (!attackBlockInstalled) {
            attackBlockInstalled = installNativeMeleeVtableHook(offsets::kVtableEntry_AttackBlockHandler_ShouldHandleEvent,
                &hookedAttackBlockShouldHandleEvent,
                g_originalAttackBlockShouldHandleEvent,
                "AttackBlockHandler::ShouldHandleEvent");
        }
        if (!playerWeaponSwingCallbackInstalled) {
            playerWeaponSwingCallbackInstalled = installNativeMeleeVtableHook(offsets::kVtableEntry_PlayerCharacter_WeaponSwingCallBack,
                &hookedPlayerWeaponSwingCallback,
                g_originalPlayerWeaponSwingCallback,
                "PlayerCharacter::WeaponSwingCallBack");
        }

        if (!(weaponSwingInstalled && hitFrameInstalled && attackBlockInstalled && playerWeaponSwingCallbackInstalled && vrMeleeImpactInstalled)) {
            ROCK_LOG_ERROR(Init,
                "Native melee suppression hook installation was incomplete: weaponSwing={} hitFrame={} attackBlock={} playerSwingCallback={} vrMeleeImpact={}; rolling back",
                weaponSwingInstalled ? "yes" : "no",
                hitFrameInstalled ? "yes" : "no",
                attackBlockInstalled ? "yes" : "no",
                playerWeaponSwingCallbackInstalled ? "yes" : "no",
                vrMeleeImpactInstalled ? "yes" : "no");
            rollbackNativeMeleeSuppressionHooks();
            g_nativeMeleeSuppressionHooksInstalled.store(false, std::memory_order_release);
            return false;
        }

        g_nativeMeleeSuppressionHooksInstalled.store(true, std::memory_order_release);
        ROCK_LOG_INFO(Init, "Native melee suppression hooks installed: weaponSwing={} hitFrame={} attackBlock={} playerSwingCallback={} vrMeleeImpact={} enabled={} full={} suppressSwing={} suppressHitFrame={}",
            weaponSwingInstalled ? "yes" : "no", hitFrameInstalled ? "yes" : "no", attackBlockInstalled ? "yes" : "no",
            playerWeaponSwingCallbackInstalled ? "yes" : "no", vrMeleeImpactInstalled ? "yes" : "no", g_rockConfig.rockNativeMeleeSuppressionEnabled ? "yes" : "no",
            g_rockConfig.rockNativeMeleeFullSuppression ? "yes" : "no", g_rockConfig.rockNativeMeleeSuppressWeaponSwing ? "yes" : "no",
            g_rockConfig.rockNativeMeleeSuppressHitFrame ? "yes" : "no");
        return weaponSwingInstalled && hitFrameInstalled && attackBlockInstalled && playerWeaponSwingCallbackInstalled && vrMeleeImpactInstalled;
    }

    using ProcessConstraints_t = void (*)(void*, void*, void*, void*);
    static ProcessConstraints_t g_originalProcessConstraints = nullptr;

    void hookedProcessConstraintsCallback(void* controller, void* charProxy, void* manifold, void* simplexInput)
    {
        bool originalAttempted = false;
        if (!PhysicsInteraction::s_hooksEnabled.load(std::memory_order_acquire)) {
            if (g_originalProcessConstraints) {
                originalAttempted = true;
                g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);
            }
            return;
        }

        __try {
            const bool playerControllerFilterEnabled = g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled;
            const bool playerController = playerControllerFilterEnabled && isPlayerCharacterController(controller);
            RE::bhkWorld* playerBhkWorld = playerController ? resolvePlayerBhkWorld() : nullptr;
            RE::hknpWorld* playerHknpWorld = playerBhkWorld ? havok_runtime::getHknpWorldFromBhk(playerBhkWorld) : nullptr;
            const bool playerControllerFilterActive = playerController && playerHknpWorld;

            auto* pi = PhysicsInteraction::s_instance.load(std::memory_order_acquire);
            const bool piReady = pi && pi->isInitialized();
            const bool rightHolding = piReady && pi->getRightHand().isHoldingAtomic();
            const bool leftHolding = piReady && pi->getLeftHand().isHoldingAtomic();
            const bool diagnosticsEnabled = g_rockConfig.rockDebugGrabFrameLogging || g_rockConfig.rockDebugVerboseLogging;
            const auto contactPolicy = held_grab_cc_policy::evaluateHeldGrabContactPolicy(held_grab_cc_policy::HeldGrabContactPolicyInput{
                .hooksEnabled = piReady,
                .holdingHeldObject = rightHolding || leftHolding,
                .diagnosticsEnabled = diagnosticsEnabled,
            });
            const bool heldFilterActive = piReady && contactPolicy.mayFilterBeforeOriginal;

            if (!heldFilterActive && !playerControllerFilterActive) {
                if (g_originalProcessConstraints) {
                    originalAttempted = true;
                    g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);
                }
                return;
            }

            const auto contactBuffers = held_grab_cc_policy::makeGeneratedContactBufferView(manifold, simplexInput);
            if (!contactBuffers.valid) {
                if (g_rockConfig.rockDebugVerboseLogging) {
                    ROCK_LOG_SAMPLE_DEBUG(CC,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Skipped character-controller pre-filter reason={} manifoldCount={} constraintCount={} heldFilter={} playerObjectFilter={}",
                        contactBuffers.reason,
                        contactBuffers.manifoldCount,
                        contactBuffers.constraintCount,
                        heldFilterActive ? "on" : "off",
                        playerControllerFilterActive ? "on" : "off");
                }
                if (g_originalProcessConstraints) {
                    originalAttempted = true;
                    g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);
                }
                return;
            }

            int removedHeldPairs = 0;
            int removedPlayerObjectPairs = 0;
            int removedPlayerNonSupportPairs = 0;
            int removedPlayerDynamicMovableStaticPairs = 0;
            int preservedPlayerSupportPairs = 0;
            int preservedUnknownTargetPairs = 0;
            const auto filterResult = held_grab_cc_policy::filterGeneratedContactBuffers(contactBuffers, [&](std::uint32_t bodyId) {
                if (heldFilterActive) {
                    bool isHeld = false;
                    if (rightHolding) {
                        isHeld = pi->getRightHand().isHeldBodyId(bodyId);
                    }
                    if (!isHeld && leftHolding) {
                        isHeld = pi->getLeftHand().isHeldBodyId(bodyId);
                    }
                    if (isHeld) {
                        ++removedHeldPairs;
                        return true;
                    }
                }

                if (playerControllerFilterActive) {
                    const auto decision = evaluatePlayerControllerTargetBody(playerBhkWorld, playerHknpWorld, bodyId);
                    if (decision.suppress) {
                        ++removedPlayerObjectPairs;
                        if (std::string_view(decision.reason) == "dynamicMovableStaticSupportLayer") {
                            ++removedPlayerDynamicMovableStaticPairs;
                        } else {
                            ++removedPlayerNonSupportPairs;
                        }
                        return true;
                    }
                    if (std::string_view(decision.reason) == "supportLayer") {
                        ++preservedPlayerSupportPairs;
                    } else if (std::string_view(decision.reason) == "unknownTargetLayer") {
                        ++preservedUnknownTargetPairs;
                    }
                }
                return false;
            });

            if (diagnosticsEnabled && filterResult.valid) {
                if (filterResult.removedPairCount > 0) {
                    ROCK_LOG_SAMPLE_DEBUG(CC,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Filtered {} character-controller contacts before original listener kept={} originalPairs={} heldRemoved={} playerObjectRemoved={} playerNonSupportRemoved={} playerDynamicMovableStaticRemoved={} playerSupportPreserved={} playerUnknownPreserved={}",
                        filterResult.removedPairCount,
                        filterResult.keptPairCount,
                        filterResult.originalPairCount,
                        removedHeldPairs,
                        removedPlayerObjectPairs,
                        removedPlayerNonSupportPairs,
                        removedPlayerDynamicMovableStaticPairs,
                        preservedPlayerSupportPairs,
                        preservedUnknownTargetPairs);
                } else if (g_rockConfig.rockDebugVerboseLogging) {
                    ROCK_LOG_SAMPLE_DEBUG(CC,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Character-controller pre-filter kept native contacts originalPairs={} reason={} playerSupportPreserved={} playerUnknownPreserved={}",
                        filterResult.originalPairCount,
                        filterResult.reason,
                        preservedPlayerSupportPairs,
                        preservedUnknownTargetPairs);
                }
            }

            if (g_originalProcessConstraints) {
                originalAttempted = true;
                g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            static int sehCount = 0;
            if (sehCount++ % 100 == 0) {
                logger::error("[ROCK::CC] SEH exception in hookedProcessConstraintsCallback (count={})", sehCount);
            }
            if (!originalAttempted && g_originalProcessConstraints) {
                __try {
                    originalAttempted = true;
                    g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);
                } __except (EXCEPTION_EXECUTE_HANDLER) {
                }
            }
        }
    }

    void installRefreshManifoldHook()
    {
        static bool installed = false;
        static bool installAttempted = false;
        if (installed)
            return;
        if (installAttempted)
            return;
        installAttempted = true;

        // Ghidra verified bhkCharProxyController::processConstraintsCallback at
        // 0x141E4B7E0 starts with whole prologue instructions through PUSH R12.
        // This callback owns the generated contact rows ROCK compacts, so it
        // uses the same fail-closed relocated-entry trampoline as
        // HandleBumpedCharacter instead of copying unvalidated bytes.
        constexpr std::array<std::uint8_t, 14> expectedPrefix{
            0x48, 0x8B, 0xC4,
            0x4C, 0x89, 0x48, 0x20,
            0x4C, 0x89, 0x40, 0x18,
            0x55,
            0x41, 0x54
        };

        void* original = reinterpret_cast<void*>(g_originalProcessConstraints);
        installed = installEntryTrampolineHook("ProcessConstraintsCallback",
            offsets::kFunc_ProcessConstraintsCallback,
            expectedPrefix.data(),
            expectedPrefix.size(),
            &hookedProcessConstraintsCallback,
            original);
        g_originalProcessConstraints = reinterpret_cast<ProcessConstraints_t>(original);
    }
}
