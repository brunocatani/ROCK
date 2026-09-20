#include "physics-interaction/core/PhysicsHooks.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/NativeMeleeSuppressionPolicy.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokTimingFixPolicy.h"
#include "physics-interaction/native/NativeGrabHapticSuppressionPolicy.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/native/NativePlayerCollisionFilter.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RockConfig.h"

#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/InputEvent.h"
#include "RE/Bethesda/Settings.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string_view>

namespace rock
{
    namespace
    {
        using NativeMeleeHandler_t = bool (*)(void*, RE::Actor*, RE::BSFixedString*);
        using NativeMeleeInputGateHandler_t = bool (*)(void*, const RE::InputEvent*);
        using NativePlayerWeaponSwingCallback_t = void (*)(RE::Actor*, std::uint32_t);
        using NativeVrMeleeImpactCallback_t = void (*)(RE::Actor*, void*, void*);
        using BhkWorldSetDeltaTime_t = void (*)(float);

        static NativeMeleeHandler_t g_originalWeaponSwingHandler = nullptr;
        static NativeMeleeHandler_t g_originalHitFrameHandler = nullptr;
        static NativeMeleeInputGateHandler_t g_originalAttackBlockShouldHandleEvent = nullptr;
        static NativePlayerWeaponSwingCallback_t g_originalPlayerWeaponSwingCallback = nullptr;
        static NativeVrMeleeImpactCallback_t g_originalVrMeleeImpactCallback = nullptr;
        static BhkWorldSetDeltaTime_t g_originalBhkWorldSetDeltaTime = nullptr;
        static std::atomic<bool> g_havokTimingFixMissingOriginalLogged{ false };
        static std::atomic<bool> g_havokTimingFixWriteFailureLogged{ false };

        constexpr std::uintptr_t kFunc_BhkWorldSetDeltaTime = 0x1DF7120;
        constexpr std::uintptr_t kHookSite_BhkWorldSetDeltaTimeMainCall = 0x0D84BD0;
        static std::atomic<std::uint64_t> g_nativeRuntimeSettingFrameClock{ 1 };
        struct ProxyContactTrace
        {
            std::atomic<std::uint64_t> playerCallbacks{ 0 }, identityFailures{ 0 }, invalidBuffers{ 0 };
            std::atomic<std::uint64_t> removed{ 0 }, movableStatics{ 0 }, looseWeapons{ 0 }, held{ 0 };
            std::atomic<std::uint64_t> supportKept{ 0 }, attacksKept{ 0 }, unknownKept{ 0 };
            std::atomic<unsigned> failedProofStage{ 0 }; // 1 listener, 2 controller, 3 reciprocal proxy
        };
        static ProxyContactTrace g_proxyContactTrace;

        // Physics callbacks publish counts only. The existing main-thread clock
        // reports actual filtering and identity failures at most once per 5 s.
        void reportProxyContactTrace()
        {
            static std::uint64_t nextReportMs = 0;
            static bool identityReported = false;
            const auto now = GetTickCount64();
            if (now < nextReportMs) {
                return;
            }
            nextReportMs = now + 5000;
            const auto callbacks = g_proxyContactTrace.playerCallbacks.exchange(0);
            const auto failures = g_proxyContactTrace.identityFailures.exchange(0);
            const auto invalid = g_proxyContactTrace.invalidBuffers.exchange(0);
            const auto removed = g_proxyContactTrace.removed.exchange(0);
            const auto mstt = g_proxyContactTrace.movableStatics.exchange(0);
            const auto looseWeapons = g_proxyContactTrace.looseWeapons.exchange(0);
            const auto held = g_proxyContactTrace.held.exchange(0);
            const auto support = g_proxyContactTrace.supportKept.exchange(0);
            const auto attacks = g_proxyContactTrace.attacksKept.exchange(0);
            const auto unknown = g_proxyContactTrace.unknownKept.exchange(0);
            if (callbacks && !identityReported) {
                identityReported = true;
                ROCK_LOG_INFO(CC, "Player controller contact filter active: listener + 16 identity, vtables and reciprocal proxy verified");
            }
            if (removed || failures || invalid) {
                ROCK_LOG_INFO(CC,
                    "Player controller contact filter: callbacks={} removed={} msttRemoved={} looseWeaponsRemoved={} heldRemoved={} supportKept={} attacksKept={} unknownKept={} invalidBuffers={} identityFailures={} failedProofStage={}",
                    callbacks, removed, mstt, looseWeapons, held, support, attacks, unknown, invalid, failures,
                    g_proxyContactTrace.failedProofStage.load());
            }
            if (failures) {
                ROCK_LOG_WARN(CC, "Player controller listener identity rejected: count={} deepestStage={} (1=listener,2=controller,3=proxy); native contacts preserved",
                    failures, g_proxyContactTrace.failedProofStage.load());
            }
        }
        static std::atomic<bool> g_nativeMeleeSuppressionHooksInstalled{ false };
        static std::atomic<bool> g_nativeMeleeSuppressionActive{ false };
        constexpr std::uint64_t kNativeMeleeRuntimeSettingCheckIntervalFrames = 90;
        constexpr std::uint64_t kNativeGrabHapticRuntimeSettingCheckIntervalFrames = 90;
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
            bool applied = false;
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
            bool applied = false;
            std::atomic<std::uint32_t> reapplyCount{ 0 };
        };

        struct NativeGrabHapticBinarySettingState
        {
            RE::Setting* setting = nullptr;
            bool originalValue = false;
            bool originalCaptured = false;
            bool missingLogged = false;
            bool typeMismatchLogged = false;
            bool confirmedLogged = false;
            bool applied = false;
            std::atomic<std::uint32_t> reapplyCount{ 0 };
        };

        struct NativeGrabHapticFloatSettingState
        {
            RE::Setting* setting = nullptr;
            float originalValue = 0.0f;
            bool originalCaptured = false;
            bool missingLogged = false;
            bool typeMismatchLogged = false;
            bool confirmedLogged = false;
            bool applied = false;
            std::atomic<std::uint32_t> reapplyCount{ 0 };
        };

        static std::atomic<std::uint64_t> g_nativeMeleeRuntimeSettingNextCheckFrame{ 0 };
        // Runtime-setting ownership is mutated only by initialization and the
        // main-frame update. Hook callbacks read config, never this lease state.
        static bool g_nativeMeleeRuntimeSuppressionRequested = false;
        static NativeBinaryRuntimeSettingState g_nativeMeleeVelocityCheckState;
        static NativeFloatRuntimeSettingState g_nativeMeleeLinearThresholdState;
        static NativeFloatRuntimeSettingState g_nativeMeleeAngularThresholdState;
        static std::atomic<std::uint64_t> g_nativeGrabHapticRuntimeSettingNextCheckFrame{ 0 };
        static NativeGrabHapticBinarySettingState g_nativeGrabHapticRolloverState;
        static NativeGrabHapticFloatSettingState g_nativeGrabHapticHoverIntensityState;
        static NativeGrabHapticFloatSettingState g_nativeGrabHapticHoverDurationState;

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

        float readBhkWorldFloatGlobal(std::uintptr_t offset, float fallback)
        {
            REL::Relocation<float*> value{ REL::Offset(offset) };
            return value.address() ? *value : fallback;
        }

        std::uint32_t readBhkWorldUintGlobal(std::uintptr_t offset, std::uint32_t fallback)
        {
            REL::Relocation<std::uint32_t*> value{ REL::Offset(offset) };
            return value.address() ? *value : fallback;
        }

        bool writeBhkWorldFloatGlobal(std::uintptr_t offset, float newValue)
        {
            REL::Relocation<float*> value{ REL::Offset(offset) };
            if (!value.address()) {
                return false;
            }
            *value = newValue;
            return true;
        }

        bool writeBhkWorldUintGlobal(std::uintptr_t offset, std::uint32_t newValue)
        {
            REL::Relocation<std::uint32_t*> value{ REL::Offset(offset) };
            if (!value.address()) {
                return false;
            }
            *value = newValue;
            return true;
        }

        bool validateBhkWorldSetDeltaTimeMainCallSite()
        {
            /*
             * Ghidra verified the main update call at 0x140D84BD0 targets
             * bhkWorld::SetDeltaTime at 0x141DF7120. The SetDeltaTime entry
             * prologue includes RIP-relative instructions, so ROCK patches the
             * call site instead of using the relocated-entry trampoline helper.
             */
            REL::Relocation<std::uintptr_t> callSite{ REL::Offset(kHookSite_BhkWorldSetDeltaTimeMainCall) };
            const auto callSiteAddress = callSite.address();
            auto* callBytes = reinterpret_cast<const std::uint8_t*>(callSiteAddress);
            if (!callBytes) {
                ROCK_LOG_ERROR(Init, "HAVOK_TIMING_FIX call-site validation failed: call site is null");
                return false;
            }

            if (callBytes[0] != 0xE8) {
                ROCK_LOG_ERROR(
                    Init,
                    "HAVOK_TIMING_FIX call-site validation failed at 0x{:X}: expected CALL rel32, found opcode 0x{:02X}",
                    callSiteAddress,
                    callBytes[0]);
                return false;
            }

            const auto relativeTarget = *reinterpret_cast<const std::int32_t*>(callBytes + 1);
            const auto decodedTarget = callSiteAddress + 5u + relativeTarget;
            const auto expectedTarget = REL::Offset(kFunc_BhkWorldSetDeltaTime).address();
            if (decodedTarget != expectedTarget) {
                ROCK_LOG_ERROR(
                    Init,
                    "HAVOK_TIMING_FIX call-site validation failed at 0x{:X}: decoded target 0x{:X}, expected 0x{:X}",
                    callSiteAddress,
                    decodedTarget,
                    expectedTarget);
                return false;
            }

            return true;
        }

        void hookedBhkWorldSetDeltaTime(float rawDeltaSeconds)
        {
            if (!g_originalBhkWorldSetDeltaTime) {
                if (!g_havokTimingFixMissingOriginalLogged.exchange(true, std::memory_order_acq_rel)) {
                    ROCK_LOG_CRITICAL(Init, "HAVOK_TIMING_FIX missing original bhkWorld::SetDeltaTime; timing hook cannot safely continue");
                }
                return;
            }

            g_originalBhkWorldSetDeltaTime(rawDeltaSeconds);

            if (!g_rockConfig.rockHavokTimingFixEnabled && !performance_profiler::enabled()) {
                return;
            }

            const float accumulatedDeltaSeconds =
                readBhkWorldFloatGlobal(offsets::kData_BhkWorldAccumulatedDeltaSeconds, rawDeltaSeconds);
            const float oldSubstepDeltaSeconds =
                readBhkWorldFloatGlobal(offsets::kData_BhkWorldSubstepDeltaSeconds, rawDeltaSeconds);
            const auto oldSubstepCount =
                readBhkWorldUintGlobal(offsets::kData_BhkWorldSubstepCount, 1);
            performance_profiler::observeValue(performance_profiler::ValueMetric::PhysicsOriginalSubsteps, oldSubstepCount);
            if (havok_timing_fix_policy::isUsableDeltaSeconds(rawDeltaSeconds)) {
                performance_profiler::observeValue(performance_profiler::ValueMetric::PhysicsRawDeltaMicroseconds,
                    static_cast<std::uint64_t>(rawDeltaSeconds * 1'000'000.0f));
            }
            if (!g_rockConfig.rockHavokTimingFixEnabled) {
                performance_profiler::observeValue(performance_profiler::ValueMetric::PhysicsRequestedSubsteps, oldSubstepCount);
                return;
            }
            const auto decision = havok_timing_fix_policy::evaluateTimingFix(havok_timing_fix_policy::TimingFixInput{
                .rawDeltaSeconds = rawDeltaSeconds,
                .accumulatedDeltaSeconds = accumulatedDeltaSeconds,
                .minPhysicsFrameRate = g_rockConfig.rockHavokTimingFixMinPhysicsFrameRate,
                .maxSubsteps = g_rockConfig.rockHavokTimingFixMaxSubsteps,
            });

            if (!decision.valid) {
                performance_profiler::observeValue(performance_profiler::ValueMetric::PhysicsRequestedSubsteps, oldSubstepCount);
                if (g_rockConfig.rockDebugVerboseLogging || g_rockConfig.rockDebugGrabFrameLogging) {
                    ROCK_LOG_SAMPLE_DEBUG(Physics,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "HAVOK_TIMING_FIX skipped reason={} rawDt={:.6f} accumDt={:.6f} oldSubDt={:.6f} oldSubsteps={}",
                        decision.reason,
                        rawDeltaSeconds,
                        accumulatedDeltaSeconds,
                        oldSubstepDeltaSeconds,
                        oldSubstepCount);
                }
                return;
            }

            const bool wroteSubstepDelta =
                writeBhkWorldFloatGlobal(offsets::kData_BhkWorldSubstepDeltaSeconds, decision.substepDeltaSeconds);
            const bool wroteSubstepCount =
                writeBhkWorldUintGlobal(offsets::kData_BhkWorldSubstepCount, decision.substepCount);
            if (!wroteSubstepDelta || !wroteSubstepCount) {
                if (!g_havokTimingFixWriteFailureLogged.exchange(true, std::memory_order_acq_rel)) {
                    ROCK_LOG_ERROR(Init,
                        "HAVOK_TIMING_FIX failed to write FO4VR timing globals: wroteSubstepDelta={} wroteSubstepCount={}",
                        wroteSubstepDelta ? "yes" : "no",
                        wroteSubstepCount ? "yes" : "no");
                }
                return;
            }

            performance_profiler::observeValue(performance_profiler::ValueMetric::PhysicsRequestedSubsteps, decision.substepCount);
            if (decision.substepCount > oldSubstepCount) {
                performance_profiler::addCounter(performance_profiler::Counter::PhysicsTimingSubstepsIncreased);
            }
            if (g_rockConfig.rockDebugVerboseLogging || g_rockConfig.rockDebugGrabFrameLogging) {
                ROCK_LOG_SAMPLE_DEBUG(Physics,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "HAVOK_TIMING_FIX rawDt={:.6f} accumDt={:.6f} oldSubDt={:.6f} oldSubsteps={} newSubDt={:.6f} newSubsteps={} minHz={:.2f} maxFrameDt={:.6f} maxSubsteps={}",
                    rawDeltaSeconds,
                    accumulatedDeltaSeconds,
                    oldSubstepDeltaSeconds,
                    oldSubstepCount,
                    decision.substepDeltaSeconds,
                    decision.substepCount,
                    g_rockConfig.rockHavokTimingFixMinPhysicsFrameRate,
                    decision.maxPhysicsFrameSeconds,
                    decision.clampedMaxSubsteps);
            }
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
            return native_melee_suppression::NativeMeleeInputGatePolicyInput{
                .suppressionActive = g_nativeMeleeSuppressionActive.load(std::memory_order_acquire),
                .inputEvent = event };
        }

        native_melee_suppression::NativeMeleeImpactPolicyInput makeNativeMeleeImpactPolicyInput(const RE::Actor* actor)
        {
            const bool player = isPlayerActor(actor);
            return native_melee_suppression::NativeMeleeImpactPolicyInput{
                .suppressionActive = g_nativeMeleeSuppressionActive.load(std::memory_order_acquire) ||
                    (player && input_remap_runtime::isBareFistMeleeSuppressed() && !f4vr::getEquippedWeaponItem()),
                .actorIsPlayer = player };
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
                if (setting->GetBinary() != desiredValue) {
                    ROCK_LOG_ERROR(Combat, "Failed to apply FO4VR native VR melee {} setting '{}'", label, settingName);
                    return false;
                }
                const auto reapplyCount = state.reapplyCount.fetch_add(1, std::memory_order_relaxed) + 1;
                if (reapplyCount == 1 || reapplyCount % 30 == 0) {
                    ROCK_LOG_WARN(Combat,
                        "Set FO4VR native VR melee {} setting '{}' to {} (original={} reapplyCount={})",
                        label,
                        settingName,
                        desiredValue ? "true" : "false",
                        state.originalValue ? "true" : "false",
                        reapplyCount);
                }
                state.confirmedLogged = true;
            }

            if (!state.confirmedLogged) {
                ROCK_LOG_INFO(Combat, "FO4VR native VR melee {} setting '{}' is {}", label, settingName, desiredValue ? "true" : "false");
                state.confirmedLogged = true;
            }

            state.applied = true;
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
                const float appliedValue = setting->GetFloat();
                if (!std::isfinite(appliedValue) || appliedValue < desiredMinimum) {
                    ROCK_LOG_ERROR(Combat, "Failed to apply FO4VR native VR melee {} setting '{}'", label, settingName);
                    return false;
                }
                const auto reapplyCount = state.reapplyCount.fetch_add(1, std::memory_order_relaxed) + 1;
                if (reapplyCount == 1 || reapplyCount % 30 == 0) {
                    ROCK_LOG_WARN(Combat,
                        "Raised FO4VR native VR melee {} setting '{}' to {:.1f} (original={:.3f} reapplyCount={})",
                        label,
                        settingName,
                        desiredMinimum,
                        state.originalValue,
                        reapplyCount);
                }
                state.confirmedLogged = true;
            }

            if (!state.confirmedLogged) {
                ROCK_LOG_INFO(Combat, "FO4VR native VR melee {} setting '{}' is armed at {:.1f}", label, settingName, currentValue);
                state.confirmedLogged = true;
            }

            state.applied = true;
            return true;
        }

        template <class State>
        void releaseNativeMeleeRuntimeSettingOwnership(State& state)
        {
            state.originalCaptured = false;
            state.applied = false;
            state.confirmedLogged = false;
            state.reapplyCount.store(0, std::memory_order_relaxed);
        }

        bool restoreNativeMeleeBinarySetting(
            NativeBinaryRuntimeSettingState& state, const char* settingName, bool appliedValue, const char* label)
        {
            if (!state.originalCaptured || !state.applied) {
                return true;
            }

            auto* setting = resolveNativeMeleeRuntimeSetting(state.setting, settingName, state.missingLogged);
            if (!setting) {
                return false;
            }

            if (setting->GetType() != RE::Setting::SETTING_TYPE::kBinary) {
                if (!state.typeMismatchLogged) {
                    state.typeMismatchLogged = true;
                    ROCK_LOG_ERROR(Combat, "Native VR melee suppression found non-binary setting '{}' while restoring", settingName);
                }
                return false;
            }

            const bool currentValue = setting->GetBinary();
            if (currentValue == appliedValue) {
                if (currentValue != state.originalValue) {
                    setting->SetBinary(state.originalValue);
                    if (setting->GetBinary() != state.originalValue) {
                        ROCK_LOG_ERROR(Combat, "Failed to restore FO4VR native VR melee {} setting '{}'", label, settingName);
                        return false;
                    }
                }
                ROCK_LOG_INFO(Combat,
                    "Restored FO4VR native VR melee {} setting '{}' to {}",
                    label,
                    settingName,
                    state.originalValue ? "true" : "false");
            } else {
                ROCK_LOG_INFO(Combat,
                    "Released FO4VR native VR melee {} setting '{}' without overwrite because its live value changed outside ROCK",
                    label,
                    settingName);
            }

            releaseNativeMeleeRuntimeSettingOwnership(state);
            return true;
        }

        bool restoreNativeMeleeFloatSetting(
            NativeFloatRuntimeSettingState& state, const char* settingName, float appliedValue, const char* label)
        {
            if (!state.originalCaptured || !state.applied) {
                return true;
            }

            auto* setting = resolveNativeMeleeRuntimeSetting(state.setting, settingName, state.missingLogged);
            if (!setting) {
                return false;
            }

            if (setting->GetType() != RE::Setting::SETTING_TYPE::kFloat) {
                if (!state.typeMismatchLogged) {
                    state.typeMismatchLogged = true;
                    ROCK_LOG_ERROR(Combat, "Native VR melee suppression found non-float setting '{}' while restoring", settingName);
                }
                return false;
            }

            const float currentValue = setting->GetFloat();
            if (currentValue == appliedValue) {
                if (!native_melee_suppression::sameRuntimeFloatBits(currentValue, state.originalValue)) {
                    setting->SetFloat(state.originalValue);
                    if (!native_melee_suppression::sameRuntimeFloatBits(setting->GetFloat(), state.originalValue)) {
                        ROCK_LOG_ERROR(Combat, "Failed to restore FO4VR native VR melee {} setting '{}'", label, settingName);
                        return false;
                    }
                }
                ROCK_LOG_INFO(Combat, "Restored FO4VR native VR melee {} setting '{}' to {:.3f}", label, settingName, state.originalValue);
            } else {
                ROCK_LOG_INFO(Combat,
                    "Released FO4VR native VR melee {} setting '{}' without overwrite because its live value changed outside ROCK",
                    label,
                    settingName);
            }

            releaseNativeMeleeRuntimeSettingOwnership(state);
            return true;
        }

        [[nodiscard]] bool nativeMeleeRuntimeSuppressionApplied()
        {
            return g_nativeMeleeVelocityCheckState.applied || g_nativeMeleeLinearThresholdState.applied || g_nativeMeleeAngularThresholdState.applied;
        }

        RE::Setting* resolveNativeGrabHapticRuntimeSetting(RE::Setting*& cachedSetting, const char* settingName, bool& missingLogged)
        {
            if (!cachedSetting) {
                cachedSetting = RE::GetINISetting(settingName);
            }

            if (!cachedSetting && !missingLogged) {
                missingLogged = true;
                ROCK_LOG_WARN(Haptics, "Native grab-hover haptic suppression could not resolve INI setting '{}'", settingName);
            }

            return cachedSetting;
        }

        bool enforceNativeGrabHapticBinarySetting(NativeGrabHapticBinarySettingState& state, const char* settingName, bool desiredValue, const char* label)
        {
            auto* setting = resolveNativeGrabHapticRuntimeSetting(state.setting, settingName, state.missingLogged);
            if (!setting) {
                return false;
            }

            if (setting->GetType() != RE::Setting::SETTING_TYPE::kBinary) {
                if (!state.typeMismatchLogged) {
                    state.typeMismatchLogged = true;
                    ROCK_LOG_ERROR(Haptics, "Native grab-hover haptic suppression found non-binary setting '{}'", settingName);
                }
                return false;
            }

            if (!state.originalCaptured) {
                state.originalValue = setting->GetBinary();
                state.originalCaptured = true;
                ROCK_LOG_INFO(Haptics, "Native grab-hover {} setting '{}' resolved: original={}", label, settingName, state.originalValue ? "true" : "false");
            }

            const bool currentValue = setting->GetBinary();
            if (currentValue != desiredValue) {
                setting->SetBinary(desiredValue);
                state.applied = true;
                const auto reapplyCount = state.reapplyCount.fetch_add(1, std::memory_order_relaxed) + 1;
                if (reapplyCount == 1 || reapplyCount % 30 == 0) {
                    ROCK_LOG_WARN(Haptics,
                        "Set FO4VR native grab-hover {} setting '{}' to {} (original={} reapplyCount={})",
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
                ROCK_LOG_INFO(Haptics, "FO4VR native grab-hover {} setting '{}' is {}", label, settingName, desiredValue ? "true" : "false");
                state.confirmedLogged = true;
            }

            return true;
        }

        bool enforceNativeGrabHapticFloatSetting(NativeGrabHapticFloatSettingState& state, const char* settingName, float desiredValue, const char* label)
        {
            auto* setting = resolveNativeGrabHapticRuntimeSetting(state.setting, settingName, state.missingLogged);
            if (!setting) {
                return false;
            }

            if (setting->GetType() != RE::Setting::SETTING_TYPE::kFloat) {
                if (!state.typeMismatchLogged) {
                    state.typeMismatchLogged = true;
                    ROCK_LOG_ERROR(Haptics, "Native grab-hover haptic suppression found non-float setting '{}'", settingName);
                }
                return false;
            }

            if (!state.originalCaptured) {
                state.originalValue = setting->GetFloat();
                state.originalCaptured = true;
                ROCK_LOG_INFO(Haptics, "Native grab-hover {} setting '{}' resolved: original={:.3f}", label, settingName, state.originalValue);
            }

            const float currentValue = setting->GetFloat();
            if (!std::isfinite(currentValue) || currentValue != desiredValue) {
                setting->SetFloat(desiredValue);
                state.applied = true;
                const auto reapplyCount = state.reapplyCount.fetch_add(1, std::memory_order_relaxed) + 1;
                if (reapplyCount == 1 || reapplyCount % 30 == 0) {
                    ROCK_LOG_WARN(Haptics,
                        "Set FO4VR native grab-hover {} setting '{}' to {:.3f} (original={:.3f} reapplyCount={})",
                        label,
                        settingName,
                        desiredValue,
                        state.originalValue,
                        reapplyCount);
                }
                state.confirmedLogged = true;
                return true;
            }

            if (!state.confirmedLogged) {
                ROCK_LOG_INFO(Haptics, "FO4VR native grab-hover {} setting '{}' is {:.3f}", label, settingName, desiredValue);
                state.confirmedLogged = true;
            }

            return true;
        }

        bool restoreNativeGrabHapticBinarySetting(NativeGrabHapticBinarySettingState& state, const char* settingName, const char* label)
        {
            if (!state.originalCaptured || !state.applied) {
                return true;
            }

            auto* setting = resolveNativeGrabHapticRuntimeSetting(state.setting, settingName, state.missingLogged);
            if (!setting) {
                return false;
            }

            if (setting->GetType() != RE::Setting::SETTING_TYPE::kBinary) {
                if (!state.typeMismatchLogged) {
                    state.typeMismatchLogged = true;
                    ROCK_LOG_ERROR(Haptics, "Native grab-hover haptic suppression found non-binary setting '{}' while restoring", settingName);
                }
                return false;
            }

            if (setting->GetBinary() != state.originalValue) {
                setting->SetBinary(state.originalValue);
                ROCK_LOG_INFO(Haptics,
                    "Restored FO4VR native grab-hover {} setting '{}' to {}",
                    label,
                    settingName,
                    state.originalValue ? "true" : "false");
            }
            state.applied = false;
            state.confirmedLogged = false;
            return true;
        }

        bool restoreNativeGrabHapticFloatSetting(NativeGrabHapticFloatSettingState& state, const char* settingName, const char* label)
        {
            if (!state.originalCaptured || !state.applied) {
                return true;
            }

            auto* setting = resolveNativeGrabHapticRuntimeSetting(state.setting, settingName, state.missingLogged);
            if (!setting) {
                return false;
            }

            if (setting->GetType() != RE::Setting::SETTING_TYPE::kFloat) {
                if (!state.typeMismatchLogged) {
                    state.typeMismatchLogged = true;
                    ROCK_LOG_ERROR(Haptics, "Native grab-hover haptic suppression found non-float setting '{}' while restoring", settingName);
                }
                return false;
            }

            const float currentValue = setting->GetFloat();
            if (!std::isfinite(currentValue) || currentValue != state.originalValue) {
                setting->SetFloat(state.originalValue);
                ROCK_LOG_INFO(Haptics, "Restored FO4VR native grab-hover {} setting '{}' to {:.3f}", label, settingName, state.originalValue);
            }
            state.applied = false;
            state.confirmedLogged = false;
            return true;
        }

        [[nodiscard]] bool nativeGrabHapticSuppressionApplied()
        {
            return g_nativeGrabHapticRolloverState.applied || g_nativeGrabHapticHoverIntensityState.applied || g_nativeGrabHapticHoverDurationState.applied;
        }

        native_melee_suppression::NativeMeleePolicyInput makeNativeMeleePolicyInput(const RE::Actor* actor)
        {
            const bool player = isPlayerActor(actor);
            return native_melee_suppression::NativeMeleePolicyInput{
                .suppressionActive = g_nativeMeleeSuppressionActive.load(std::memory_order_acquire) ||
                    (player && input_remap_runtime::isBareFistMeleeSuppressed() && !f4vr::getEquippedWeaponItem()),
                .actorIsPlayer = player };
        }

        bool applyNativeMeleeDecision(const native_melee_suppression::NativeMeleeEvent event,
            const native_melee_suppression::NativeMeleePolicyInput& input,
            const native_melee_suppression::NativeMeleePolicyDecision& decision)
        {
            using native_melee_suppression::NativeMeleeEvent;
            using native_melee_suppression::NativeMeleeSuppressionAction;

            if (decision.action != NativeMeleeSuppressionAction::CallNative) {
                static std::atomic<std::uint32_t> weaponLogCounter{ 0 };
                static std::atomic<std::uint32_t> hitFrameLogCounter{ 0 };
                auto& counter = event == NativeMeleeEvent::WeaponSwing ? weaponLogCounter : hitFrameLogCounter;
                const auto count = counter.fetch_add(1, std::memory_order_relaxed) + 1;

                if (count == 1 || count % 180 == 0) {
                    ROCK_LOG_DEBUG(Combat, "Native melee {} decision={} reason={} player={} count={}",
                        event == NativeMeleeEvent::WeaponSwing ? "WeaponSwing" : "HitFrame",
                        decision.action == NativeMeleeSuppressionAction::CallNative ? "native" : "handled",
                        decision.reason, input.actorIsPlayer ? "yes" : "no", count);
                }
            }

            switch (decision.action) {
            case native_melee_suppression::NativeMeleeSuppressionAction::CallNative:
                return true;
            case native_melee_suppression::NativeMeleeSuppressionAction::ReturnHandled:
                return true;
            }

            return true;
        }

        bool applyNativeMeleeImpactDecision(const native_melee_suppression::NativeMeleeImpactPolicyInput& input,
            const native_melee_suppression::NativeMeleeImpactPolicyDecision& decision)
        {
            using native_melee_suppression::NativeMeleeImpactAction;

            if (decision.action != NativeMeleeImpactAction::CallNative) {
                static std::atomic<std::uint32_t> impactLogCounter{ 0 };
                const auto count = impactLogCounter.fetch_add(1, std::memory_order_relaxed) + 1;
                if (count == 1 || count % 180 == 0) {
                    ROCK_LOG_DEBUG(Combat,
                        "Native melee VRMeleeImpact decision={} reason={} player={} count={}",
                        decision.action == NativeMeleeImpactAction::CallNative ? "native" : "suppressed",
                        decision.reason,
                        input.actorIsPlayer ? "yes" : "no",
                        count);
                }
            }

            return decision.action == NativeMeleeImpactAction::Suppress;
        }

        /*
         * NATIVE-MELEE-TRACE: native melee damage is reported broken while
         * suppression is disabled and every hook forwards to native. Each
         * pass-through boundary below logs a bounded trace so one in-game
         * session shows the deepest stage the native chain reaches:
         * AttackBlock input gate -> WeaponSwingHandler -> HitFrameHandler ->
         * PlayerCharacter::WeaponSwingCallBack -> VRMeleeImpact contact.
         * Diagnostic instrumentation; remove with the root-cause fix.
         */
        [[nodiscard]] bool shouldEmitNativeMeleeTrace(std::atomic<std::uint32_t>& counter, std::uint32_t& outCount, std::uint32_t burst = 20, std::uint32_t period = 60)
        {
            outCount = counter.fetch_add(1, std::memory_order_relaxed) + 1;
            return outCount <= burst || outCount % period == 0;
        }

        enum class MeleeContactDecodeStage : std::uint32_t
        {
            Complete = 0,
            EventBuffers = 1,
            World = 2,
            PartnerBody = 3
        };

        // hkHandle has a non-trivial destructor; MSVC requires its construction
        // outside the function containing the SEH boundary below.
        bool readMeleeBodyFilter(RE::hknpWorld* world, std::uint32_t bodyId, std::uint32_t& filterInfo)
        {
            return body_collision::tryReadFilterInfo(world, RE::hknpBodyId{ bodyId }, filterInfo);
        }

        // The callback owns these event buffers for this invocation only. Keep
        // the existing FO4VR event layout, but protect the complete read with SEH
        // instead of querying page protection for each scalar. Body lookup still
        // validates the allocation bound, body identity, and live motion slot.
        // Never cache an event, world, or body pointer between callbacks.
        MeleeContactDecodeStage readMeleeContactPartner(
            const void* contactEvent, const void* collisionEvent, std::uint32_t& filterInfo) noexcept
        {
            filterInfo = 0;
            auto stage = MeleeContactDecodeStage::EventBuffers;
            if (!native_memory::pointerLooksReadable(contactEvent) ||
                !native_memory::pointerLooksReadable(collisionEvent)) {
                return stage;
            }

            __try {
                RE::hknpWorld* world = nullptr;
                std::uint32_t ourIndex = 0;
                std::uint32_t otherId = 0;
                std::memcpy(&world, contactEvent, sizeof(world));
                std::memcpy(&ourIndex, static_cast<const char*>(contactEvent) + 0x20, sizeof(ourIndex));
                if (ourIndex > 1) {
                    return stage;
                }
                std::memcpy(&otherId,
                    static_cast<const char*>(collisionEvent) + 0x08 + (1u - ourIndex) * sizeof(otherId),
                    sizeof(otherId));
                stage = MeleeContactDecodeStage::World;
                if (!native_memory::pointerLooksReadable(world)) {
                    return stage;
                }
                stage = MeleeContactDecodeStage::PartnerBody;
                if (!readMeleeBodyFilter(world, otherId, filterInfo)) {
                    return stage;
                }
                return MeleeContactDecodeStage::Complete;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return stage;
            }
        }

        // Counts cross the physics/main-thread boundary without retaining engine
        // pointers. Timing and per-frame volume use the opt-in asynchronous profiler;
        // only malformed input produces a rate-limited main-thread warning.
        std::atomic<std::uint64_t> g_meleeCallbacksThisFrame{ 0 };
        std::atomic<std::uint64_t> g_meleeDecodeFailures{ 0 };
        std::atomic<MeleeContactDecodeStage> g_meleeDecodeFailureStage{ MeleeContactDecodeStage::Complete };

        void reportMeleeContactFrame()
        {
            const auto callbacks = g_meleeCallbacksThisFrame.exchange(0, std::memory_order_relaxed);
            performance_profiler::observeValue(performance_profiler::ValueMetric::NativeMeleeCallbacksPerFrame, callbacks);
            const auto failures = g_meleeDecodeFailures.exchange(0, std::memory_order_relaxed);
            if (failures != 0) {
                ROCK_LOG_SAMPLE_WARN(Combat, 2000,
                    "Native melee contact decode failed: frameFailures={} deepestStage={} (1=events,2=world,3=body); native forwarding preserved",
                    failures, static_cast<std::uint32_t>(g_meleeDecodeFailureStage.load(std::memory_order_relaxed)));
            }
        }

        bool hookedWeaponSwingHandler(void* handler, RE::Actor* actor, RE::BSFixedString* side)
        {
            const auto input = makeNativeMeleePolicyInput(actor);
            const auto decision = native_melee_suppression::evaluateNativeMeleeSuppression(native_melee_suppression::NativeMeleeEvent::WeaponSwing, input);

            const bool shouldCallNative = decision.action == native_melee_suppression::NativeMeleeSuppressionAction::CallNative;
            const bool decisionResult = applyNativeMeleeDecision(native_melee_suppression::NativeMeleeEvent::WeaponSwing, input, decision);
            if (!shouldCallNative) {
                return decisionResult;
            }

            const bool nativeResult = g_originalWeaponSwingHandler ? g_originalWeaponSwingHandler(handler, actor, side) : false;
            if (input.actorIsPlayer && !input.suppressionActive) {
                static std::atomic<std::uint32_t> traceCounter{ 0 };
                std::uint32_t count = 0;
                if (shouldEmitNativeMeleeTrace(traceCounter, count)) {
                    ROCK_LOG_INFO(Combat, "NATIVE-MELEE-TRACE WeaponSwingHandler native result={} count={}", nativeResult ? "true" : "false", count);
                }
            }
            return nativeResult;
        }

        bool hookedHitFrameHandler(void* handler, RE::Actor* actor, RE::BSFixedString* side)
        {
            const auto input = makeNativeMeleePolicyInput(actor);
            const auto decision = native_melee_suppression::evaluateNativeMeleeSuppression(native_melee_suppression::NativeMeleeEvent::HitFrame, input);

            const bool shouldCallNative = decision.action == native_melee_suppression::NativeMeleeSuppressionAction::CallNative;
            const bool decisionResult = applyNativeMeleeDecision(native_melee_suppression::NativeMeleeEvent::HitFrame, input, decision);
            if (!shouldCallNative) {
                return decisionResult;
            }

            const bool nativeResult = g_originalHitFrameHandler ? g_originalHitFrameHandler(handler, actor, side) : false;
            if (input.actorIsPlayer && !input.suppressionActive) {
                static std::atomic<std::uint32_t> traceCounter{ 0 };
                std::uint32_t count = 0;
                if (shouldEmitNativeMeleeTrace(traceCounter, count)) {
                    ROCK_LOG_INFO(Combat, "NATIVE-MELEE-TRACE HitFrameHandler native result={} count={}", nativeResult ? "true" : "false", count);
                }
            }
            return nativeResult;
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
            const auto input = makeNativeMeleePolicyInput(actor);
            const auto decision = native_melee_suppression::evaluateNativeMeleeSuppression(native_melee_suppression::NativeMeleeEvent::WeaponSwing, input);
            const bool shouldCallNative = decision.action == native_melee_suppression::NativeMeleeSuppressionAction::CallNative;

            if (!shouldCallNative) {
                applyNativeMeleeDecision(native_melee_suppression::NativeMeleeEvent::WeaponSwing, input, decision);
                return;
            }

            if (input.actorIsPlayer && !input.suppressionActive) {
                static std::atomic<std::uint32_t> traceCounter{ 0 };
                std::uint32_t count = 0;
                if (shouldEmitNativeMeleeTrace(traceCounter, count)) {
                    ROCK_LOG_INFO(Combat, "NATIVE-MELEE-TRACE PlayerWeaponSwingCallback equipIndex={} count={}", equipIndex, count);
                }
            }

            if (g_originalPlayerWeaponSwingCallback) {
                g_originalPlayerWeaponSwingCallback(actor, equipIndex);
            }
        }

        void hookedVrMeleeImpactCallback(RE::Actor* actor, void* contactEvent, void* collisionEvent)
        {
            performance_profiler::ScopedTimer callbackTimer(performance_profiler::Scope::NativeMeleeCallback);
            if (performance_profiler::enabled()) {
                g_meleeCallbacksThisFrame.fetch_add(1, std::memory_order_relaxed);
            }
            /*
             * FO4VR registers this callback while attaching native VR melee
             * collision to the first-person weapon nodes. It owns the native
             * contact-to-hit path, including target filtering, action dispatch,
             * impulse direction, and melee cooldown writes. When complete ROCK
             * suppression is active, skip the player callback here so no native
             * impact or damage side effect survives the master switch.
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

            /*
             * Drop events whose partner is a ROCK-owned body before native
             * sees them. The native melee event stream ignores the collision
             * filter, so this hook is the only boundary that can keep ROCK's
             * co-located colliders from consuming the melee hit and arming
             * the cooldown. Undecodable events pass through unchanged.
             */
            std::uint32_t partnerFilter = 0;
            const auto decodeStage = readMeleeContactPartner(contactEvent, collisionEvent, partnerFilter);
            if (decodeStage != MeleeContactDecodeStage::Complete) {
                g_meleeDecodeFailureStage.store(decodeStage, std::memory_order_relaxed);
                g_meleeDecodeFailures.fetch_add(1, std::memory_order_relaxed);
                performance_profiler::addCounter(performance_profiler::Counter::NativeMeleeDecodeFailed);
            } else if (collision_layer_policy::isRockOwnedMatrixLayer(
                           partnerFilter & collision_layer_policy::FO4_LAYER_FILTER_MASK)) {
                performance_profiler::addCounter(performance_profiler::Counter::NativeMeleeRockPartnerDropped);
                return;
            }

            if (g_originalVrMeleeImpactCallback) {
                performance_profiler::ScopedTimer nativeTimer(performance_profiler::Scope::NativeMeleeDispatch);
                g_originalVrMeleeImpactCallback(actor, contactEvent, collisionEvent);
            }
        }

        bool hookedAttackBlockShouldHandleEvent(void* handler, const RE::InputEvent* event)
        {
            if (input_remap_runtime::shouldSuppressNativeTriggerAction(event)) {
                ROCK_LOG_SAMPLE_DEBUG(Input,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Suppressed native WandTrigger AttackBlock gate while ROCK owns holstered or held-weapon trigger input");
                return false;
            }

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

            if (decision.action != native_melee_suppression::NativeMeleeInputGateAction::CallNative) {
                static std::atomic<std::uint32_t> inputGateLogCounter{ 0 };
                const auto count = inputGateLogCounter.fetch_add(1, std::memory_order_relaxed) + 1;
                if (count == 1 || count % 180 == 0) {
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

            const bool nativeResult = g_originalAttackBlockShouldHandleEvent ? g_originalAttackBlockShouldHandleEvent(handler, event) : false;
            if (inputEvent == native_melee_suppression::NativeMeleeInputEvent::RightStick && !policyInput.suppressionActive) {
                static std::atomic<std::uint32_t> acceptedCounter{ 0 };
                static std::atomic<std::uint32_t> seenCounter{ 0 };
                const auto seen = seenCounter.fetch_add(1, std::memory_order_relaxed) + 1;
                if (nativeResult) {
                    std::uint32_t accepted = 0;
                    if (shouldEmitNativeMeleeTrace(acceptedCounter, accepted)) {
                        ROCK_LOG_INFO(Combat, "NATIVE-MELEE-TRACE AttackBlock RightStick accepted by native: accepted={} seen={}", accepted, seen);
                    }
                } else if (seen == 1 || seen % 300 == 0) {
                    ROCK_LOG_INFO(Combat,
                        "NATIVE-MELEE-TRACE AttackBlock RightStick pass-through: accepted={} seen={}",
                        acceptedCounter.load(std::memory_order_relaxed),
                        seen);
                }
            }
            return nativeResult;
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

        /*
         * NATIVE-MELEE-TRACE: with suppression disabled ROCK never reads the
         * native VR melee velocity-gate settings, so a hostile runtime value
         * (for example a leftover or third-party 1e9 threshold) is invisible.
         * Dump the live values once per session while suppression is off.
         */
        void logNativeMeleeRuntimeGateSettingsOnce()
        {
            static std::atomic<bool> logged{ false };
            if (logged.load(std::memory_order_acquire)) {
                return;
            }

            auto* velocityCheck = RE::GetINISetting(native_melee_suppression::kVelocityCheckSetting);
            auto* linearThreshold = RE::GetINISetting(native_melee_suppression::kLinearVelocityThresholdSetting);
            auto* angularThreshold = RE::GetINISetting(native_melee_suppression::kAngularVelocityThresholdSetting);
            if (!velocityCheck && !linearThreshold && !angularThreshold) {
                return;
            }
            logged.store(true, std::memory_order_release);

            const char* velocityCheckText = "unresolved";
            if (velocityCheck) {
                velocityCheckText = velocityCheck->GetType() == RE::Setting::SETTING_TYPE::kBinary ?
                    (velocityCheck->GetBinary() ? "true" : "false") :
                    "wrong-type";
            }
            ROCK_LOG_INFO(Combat,
                "NATIVE-MELEE-TRACE runtime gate settings while suppression disabled: {}={} {}={} {}={}",
                native_melee_suppression::kVelocityCheckSetting,
                velocityCheckText,
                native_melee_suppression::kLinearVelocityThresholdSetting,
                linearThreshold && linearThreshold->GetType() == RE::Setting::SETTING_TYPE::kFloat ? linearThreshold->GetFloat() : -1.0f,
                native_melee_suppression::kAngularVelocityThresholdSetting,
                angularThreshold && angularThreshold->GetType() == RE::Setting::SETTING_TYPE::kFloat ? angularThreshold->GetFloat() : -1.0f);
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

    void advanceNativeRuntimeSettingFrameClock()
    {
        g_nativeRuntimeSettingFrameClock.fetch_add(1, std::memory_order_acq_rel);
        reportProxyContactTrace();
        reportMeleeContactFrame();
    }

    bool isNativeMeleeSuppressionActive()
    {
        return g_nativeMeleeSuppressionActive.load(std::memory_order_acquire);
    }

    bool areNativeMeleeHooksInstalled()
    {
        return g_nativeMeleeSuppressionHooksInstalled.load(std::memory_order_acquire);
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
        const native_melee_suppression::NativeMeleeRuntimeSettingPolicyInput input{
            .hooksInstalled = g_nativeMeleeSuppressionHooksInstalled.load(std::memory_order_acquire),
            .suppressionEnabled = !g_rockConfig.rockEnableVanillaMelee,
        };
        const bool shouldSuppress = native_melee_suppression::shouldSuppressNativeMeleeRuntimeSettings(input);
        const bool shouldRestore = native_melee_suppression::shouldRestoreNativeMeleeRuntimeSettings(nativeMeleeRuntimeSuppressionApplied(), input);
        const bool requestChanged = g_nativeMeleeRuntimeSuppressionRequested != shouldSuppress;
        g_nativeMeleeRuntimeSuppressionRequested = shouldSuppress;
        g_nativeMeleeSuppressionActive.store(shouldSuppress, std::memory_order_release);
        if (!shouldSuppress && !shouldRestore) {
            if (input.hooksInstalled) {
                logNativeMeleeRuntimeGateSettingsOnce();
            }
            return;
        }

        const auto currentFrame = g_nativeRuntimeSettingFrameClock.load(std::memory_order_acquire);
        const auto nextCheckFrame = g_nativeMeleeRuntimeSettingNextCheckFrame.load(std::memory_order_acquire);
        if (!forceCheck && !requestChanged && currentFrame < nextCheckFrame) {
            return;
        }
        g_nativeMeleeRuntimeSettingNextCheckFrame.store(currentFrame + kNativeMeleeRuntimeSettingCheckIntervalFrames, std::memory_order_release);

        if (shouldSuppress) {
            enforceNativeMeleeBinarySetting(
                g_nativeMeleeVelocityCheckState, native_melee_suppression::kVelocityCheckSetting, true, "velocity gate");
            enforceNativeMeleeFloatMinimumSetting(g_nativeMeleeLinearThresholdState,
                native_melee_suppression::kLinearVelocityThresholdSetting,
                native_melee_suppression::kSuppressedVelocityThreshold,
                "linear threshold");
            enforceNativeMeleeFloatMinimumSetting(g_nativeMeleeAngularThresholdState,
                native_melee_suppression::kAngularVelocityThresholdSetting,
                native_melee_suppression::kSuppressedVelocityThreshold,
                "angular threshold");
            return;
        }

        restoreNativeMeleeBinarySetting(
            g_nativeMeleeVelocityCheckState, native_melee_suppression::kVelocityCheckSetting, true, "velocity gate");
        restoreNativeMeleeFloatSetting(g_nativeMeleeLinearThresholdState,
            native_melee_suppression::kLinearVelocityThresholdSetting,
            native_melee_suppression::kSuppressedVelocityThreshold,
            "linear threshold");
        restoreNativeMeleeFloatSetting(g_nativeMeleeAngularThresholdState,
            native_melee_suppression::kAngularVelocityThresholdSetting,
            native_melee_suppression::kSuppressedVelocityThreshold,
            "angular threshold");
    }

    void enforceNativeGrabHapticRuntimeSuppression(bool forceCheck)
    {
        /*
         * FO4VR's native grabbable-object affordance is exposed through the VR
         * hover/rollover rumble settings. Suppressing those settings leaves
         * ROCK-owned haptics untouched because ROCK emits haptics directly
         * through its feedback pipeline instead of through native rollover UI.
         */
        const native_grab_haptic_suppression::RuntimeInput input{
            .rockEnabled = true,
            .suppressionEnabled = native_grab_haptic_suppression::kSuppressionEnabled,
        };
        const bool shouldSuppress = native_grab_haptic_suppression::shouldSuppressNativeGrabHoverHaptics(input);
        const bool shouldRestore = native_grab_haptic_suppression::shouldRestoreNativeGrabHoverHaptics(nativeGrabHapticSuppressionApplied(), input);
        if (!shouldSuppress && !shouldRestore) {
            return;
        }

        const auto currentFrame = g_nativeRuntimeSettingFrameClock.load(std::memory_order_acquire);
        const auto nextCheckFrame = g_nativeGrabHapticRuntimeSettingNextCheckFrame.load(std::memory_order_acquire);
        if (!forceCheck && currentFrame < nextCheckFrame) {
            return;
        }
        g_nativeGrabHapticRuntimeSettingNextCheckFrame.store(currentFrame + kNativeGrabHapticRuntimeSettingCheckIntervalFrames, std::memory_order_release);

        if (shouldSuppress) {
            enforceNativeGrabHapticBinarySetting(g_nativeGrabHapticRolloverState,
                native_grab_haptic_suppression::kRolloverRumbleEnabledSetting,
                native_grab_haptic_suppression::kSuppressedRolloverRumbleEnabled,
                "rollover rumble");
            enforceNativeGrabHapticFloatSetting(g_nativeGrabHapticHoverIntensityState,
                native_grab_haptic_suppression::kHoverRumbleIntensitySetting,
                native_grab_haptic_suppression::kSuppressedHoverRumbleFloat,
                "hover intensity");
            enforceNativeGrabHapticFloatSetting(g_nativeGrabHapticHoverDurationState,
                native_grab_haptic_suppression::kHoverRumbleDurationSetting,
                native_grab_haptic_suppression::kSuppressedHoverRumbleFloat,
                "hover duration");
            return;
        }

        restoreNativeGrabHapticBinarySetting(g_nativeGrabHapticRolloverState,
            native_grab_haptic_suppression::kRolloverRumbleEnabledSetting,
            "rollover rumble");
        restoreNativeGrabHapticFloatSetting(g_nativeGrabHapticHoverIntensityState,
            native_grab_haptic_suppression::kHoverRumbleIntensitySetting,
            "hover intensity");
        restoreNativeGrabHapticFloatSetting(g_nativeGrabHapticHoverDurationState,
            native_grab_haptic_suppression::kHoverRumbleDurationSetting,
            "hover duration");
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

    struct PlayerContactTargetIdentity
    {
        bool isMovableStatic = false;
        bool isCar = false;
        bool isLooseWeapon = false;
    };

    PlayerContactTargetIdentity resolvePlayerContactTargetIdentity(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* world,
        RE::hknpBodyId bodyId,
        std::uint32_t layer)
    {
        if (layer == collision_layer_policy::FO4_LAYER_WEAPON) {
            return PlayerContactTargetIdentity{
                .isLooseWeapon = native_player_collision::isLooseWeaponBody(
                    havok_runtime::snapshotBody(world, bodyId)),
            };
        }
        const bool requiresFormIdentity =
            collision_layer_policy::isPlayerCharacterControllerSupportLayer(layer) ||
            collision_layer_policy::isDynamicWorldCarLayer(layer);
        if (!bhkWorld || !world || !requiresFormIdentity) {
            return {};
        }

        /*
         * MSTT bodies commonly sit on support layers that the player controller
         * must preserve for world geometry. For movable statics, form identity is
         * the safer discriminator than hknp motion flags: those flags can report
         * static/keyframed while the native controller still generates push
         * constraints against the movable object.
         */
        auto* ref = resolveBodyToRef(bhkWorld, world, bodyId);
        auto* baseForm = ref ? ref->GetObjectReference() : nullptr;
        if (!baseForm || !baseForm->Is(RE::ENUM_FORM_ID::kMSTT)) {
            return {};
        }
        return PlayerContactTargetIdentity{
            .isMovableStatic = true,
            .isCar = fo4vr::isExplodableCar(baseForm),
        };
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
        const auto targetIdentity = resolvePlayerContactTargetIdentity(bhkWorld, world, bodyId, layer);
        return collision_layer_policy::evaluatePlayerCharacterControllerContact(
            collision_layer_policy::PlayerCharacterControllerContactPolicyInput{
                .filterEnabled = true,
                .playerController = true,
                .targetLayerKnown = true,
                .targetLayer = layer,
                .targetIsMovableStatic = targetIdentity.isMovableStatic,
                .targetIsCar = targetIdentity.isCar,
                .targetIsLooseWeapon = targetIdentity.isLooseWeapon,
            });
    }

    bool isPlayerProxyListener(void* listener, void* proxy, void* playerController)
    {
        if (!native_player_collision::proxyListenerMatchesPlayer(
                reinterpret_cast<std::uintptr_t>(listener), reinterpret_cast<std::uintptr_t>(playerController))) {
            return false;
        }
        // Keep the callback's listener pointer unchanged when chaining native.
        // The adjusted interface is used only for player identity and checked
        // against both vtables and the callback's live hknpCharacterProxy.
        std::uintptr_t listenerVtable = 0, controllerVtable = 0;
        void* ownedProxy = nullptr;
        const bool listenerValid = native_memory::tryReadField(listener, 0, listenerVtable) &&
            listenerVtable == REL::Offset(0x2E892B8).address();
        const bool controllerValid = listenerValid && native_memory::tryReadField(playerController, 0, controllerVtable) &&
            controllerVtable == REL::Offset(0x2E89328).address();
        const bool proxyValid = controllerValid && native_memory::tryReadField(playerController, 0x470, ownedProxy) &&
            ownedProxy == proxy && proxy;
        if (!proxyValid) {
            g_proxyContactTrace.failedProofStage.store(!listenerValid ? 1 : !controllerValid ? 2 : 3, std::memory_order_relaxed);
            g_proxyContactTrace.identityFailures.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        g_proxyContactTrace.playerCallbacks.fetch_add(1, std::memory_order_relaxed);
        return true;
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

    bool installHavokTimingFixHook()
    {
        static bool installed = false;
        static bool installAttempted = false;
        if (installed) {
            return true;
        }
        if (installAttempted) {
            return false;
        }
        installAttempted = true;

        if (!validateBhkWorldSetDeltaTimeMainCallSite()) {
            ROCK_LOG_ERROR(Init, "HAVOK_TIMING_FIX hook not installed; FO4VR timing call site did not match verified bytes");
            return false;
        }

        REL::Relocation<std::uintptr_t> hookCallSite{ REL::Offset(kHookSite_BhkWorldSetDeltaTimeMainCall) };
        auto& trampoline = F4SE::GetTrampoline();
        const auto original = trampoline.write_call<5>(hookCallSite.address(), &hookedBhkWorldSetDeltaTime);
        g_originalBhkWorldSetDeltaTime = reinterpret_cast<BhkWorldSetDeltaTime_t>(original);
        installed = g_originalBhkWorldSetDeltaTime != nullptr;

        if (!installed) {
            ROCK_LOG_CRITICAL(Init, "HAVOK_TIMING_FIX hook install failed at 0x{:X}: original target was null", hookCallSite.address());
            return false;
        }

        ROCK_LOG_INFO(Init,
            "HAVOK_TIMING_FIX hook installed at 0x{:X}; original=0x{:X} enabled={} minHz={:.2f} maxSubsteps={}",
            hookCallSite.address(),
            original,
            g_rockConfig.rockHavokTimingFixEnabled ? "yes" : "no",
            g_rockConfig.rockHavokTimingFixMinPhysicsFrameRate,
            g_rockConfig.rockHavokTimingFixMaxSubsteps);
        return true;
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
            g_nativeMeleeSuppressionActive.store(false, std::memory_order_release);
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
            g_nativeMeleeSuppressionActive.store(false, std::memory_order_release);
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
            g_nativeMeleeSuppressionActive.store(false, std::memory_order_release);
            return false;
        }

        g_nativeMeleeSuppressionHooksInstalled.store(true, std::memory_order_release);
        ROCK_LOG_INFO(Init, "Native melee suppression hooks installed: weaponSwing={} hitFrame={} attackBlock={} playerSwingCallback={} vrMeleeImpact={} requested={}",
            weaponSwingInstalled ? "yes" : "no", hitFrameInstalled ? "yes" : "no", attackBlockInstalled ? "yes" : "no",
            playerWeaponSwingCallbackInstalled ? "yes" : "no", vrMeleeImpactInstalled ? "yes" : "no",
            g_rockConfig.rockEnableVanillaMelee ? "no" : "yes");
        return weaponSwingInstalled && hitFrameInstalled && attackBlockInstalled && playerWeaponSwingCallbackInstalled && vrMeleeImpactInstalled;
    }

    using ProcessConstraints_t = void (*)(void*, void*, void*, void*);
    static ProcessConstraints_t g_originalProcessConstraints = nullptr;

    void hookedProcessConstraintsCallback(void* listener, void* charProxy, void* manifold, void* simplexInput)
    {
        bool originalAttempted = false;
        if (!PhysicsInteraction::s_hooksEnabled.load(std::memory_order_acquire)) {
            if (g_originalProcessConstraints) {
                originalAttempted = true;
                g_originalProcessConstraints(listener, charProxy, manifold, simplexInput);
            }
            return;
        }

        __try {
            const bool playerControllerFilterEnabled = g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled;
            void* playerControllerPointer = resolvePlayerCharacterController();
            const bool playerController = isPlayerProxyListener(listener, charProxy, playerControllerPointer);
            RE::bhkWorld* playerBhkWorld = playerController ? resolvePlayerBhkWorld() : nullptr;
            RE::hknpWorld* playerHknpWorld = playerBhkWorld ? havok_runtime::getHknpWorldFromBhk(playerBhkWorld) : nullptr;
            const bool playerControllerFilterActive = playerControllerFilterEnabled && playerController && playerHknpWorld;

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
            const bool heldFilterActive = playerController && piReady && contactPolicy.mayFilterBeforeOriginal;

            if (!heldFilterActive && !playerControllerFilterActive) {
                if (g_originalProcessConstraints) {
                    originalAttempted = true;
                    g_originalProcessConstraints(listener, charProxy, manifold, simplexInput);
                }
                return;
            }

            const auto contactBuffers = held_grab_cc_policy::makeGeneratedContactBufferView(manifold, simplexInput);
            if (!contactBuffers.valid) {
                if (std::string_view(contactBuffers.reason) != "emptyContactBuffers") {
                    g_proxyContactTrace.invalidBuffers.fetch_add(1, std::memory_order_relaxed);
                }
                // Without body identities no targeted decision is possible.
                // Keep native support and attack constraints intact.
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
                    g_originalProcessConstraints(listener, charProxy, manifold, simplexInput);
                }
                return;
            }

            int removedHeldPairs = 0;
            int removedPlayerObjectPairs = 0;
            int removedPlayerNonSupportPairs = 0;
            int removedPlayerMovableStaticPairs = 0;
            int removedLooseWeaponPairs = 0;
            int preservedAttackPairs = 0;
            int preservedPlayerSupportPairs = 0;
            int preservedPlayerCarPairs = 0;
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
                        if (std::string_view(decision.reason) == "movableStaticSupportLayer") {
                            ++removedPlayerMovableStaticPairs;
                        } else if (std::string_view(decision.reason) == "looseWeapon") {
                            ++removedLooseWeaponPairs;
                        } else {
                            ++removedPlayerNonSupportPairs;
                        }
                        return true;
                    }
                    if (std::string_view(decision.reason) == "nativeAttack") {
                        ++preservedAttackPairs;
                    } else if (std::string_view(decision.reason) == "supportLayer") {
                        ++preservedPlayerSupportPairs;
                    } else if (std::string_view(decision.reason) == "carCollision") {
                        ++preservedPlayerCarPairs;
                    } else if (std::string_view(decision.reason) == "unknownTargetLayer") {
                        ++preservedUnknownTargetPairs;
                    }
                }
                return false;
            });

            if (filterResult.valid) {
                g_proxyContactTrace.removed.fetch_add(filterResult.removedPairCount, std::memory_order_relaxed);
                g_proxyContactTrace.movableStatics.fetch_add(removedPlayerMovableStaticPairs, std::memory_order_relaxed);
                g_proxyContactTrace.looseWeapons.fetch_add(removedLooseWeaponPairs, std::memory_order_relaxed);
                g_proxyContactTrace.held.fetch_add(removedHeldPairs, std::memory_order_relaxed);
                g_proxyContactTrace.supportKept.fetch_add(preservedPlayerSupportPairs + preservedPlayerCarPairs, std::memory_order_relaxed);
                g_proxyContactTrace.attacksKept.fetch_add(preservedAttackPairs, std::memory_order_relaxed);
                g_proxyContactTrace.unknownKept.fetch_add(preservedUnknownTargetPairs, std::memory_order_relaxed);
            }
            if (diagnosticsEnabled && filterResult.valid) {
                if (filterResult.removedPairCount > 0) {
                    ROCK_LOG_SAMPLE_DEBUG(CC,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Filtered {} character-controller contacts before original listener kept={} originalPairs={} heldRemoved={} playerObjectRemoved={} playerNonSupportRemoved={} playerMovableStaticRemoved={} playerSupportPreserved={} playerCarPreserved={} playerUnknownPreserved={}",
                        filterResult.removedPairCount,
                        filterResult.keptPairCount,
                        filterResult.originalPairCount,
                        removedHeldPairs,
                        removedPlayerObjectPairs,
                        removedPlayerNonSupportPairs,
                        removedPlayerMovableStaticPairs,
                        preservedPlayerSupportPairs,
                        preservedPlayerCarPairs,
                        preservedUnknownTargetPairs);
                } else if (g_rockConfig.rockDebugVerboseLogging) {
                    ROCK_LOG_SAMPLE_DEBUG(CC,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Character-controller pre-filter kept native contacts originalPairs={} reason={} playerSupportPreserved={} playerCarPreserved={} playerUnknownPreserved={}",
                        filterResult.originalPairCount,
                        filterResult.reason,
                        preservedPlayerSupportPairs,
                        preservedPlayerCarPairs,
                        preservedUnknownTargetPairs);
                }
            }

            if (g_originalProcessConstraints) {
                originalAttempted = true;
                g_originalProcessConstraints(listener, charProxy, manifold, simplexInput);
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            static int sehCount = 0;
            if (sehCount++ % 100 == 0) {
                logger::error("[ROCK::CC] SEH exception in hookedProcessConstraintsCallback (count={})", sehCount);
            }
            if (!originalAttempted && g_originalProcessConstraints) {
                __try {
                    originalAttempted = true;
                    g_originalProcessConstraints(listener, charProxy, manifold, simplexInput);
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
