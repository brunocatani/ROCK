#include "physics-interaction/core/PhysicsHooks.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokTimingFixPolicy.h"
#include "physics-interaction/native/NativeGrabHapticSuppressionPolicy.h"

#include "RockConfig.h"

#include "RE/Bethesda/Settings.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>

namespace rock
{
    namespace
    {
        using BhkWorldSetDeltaTime_t = void (*)(float);

        static BhkWorldSetDeltaTime_t g_originalBhkWorldSetDeltaTime = nullptr;
        static std::atomic<bool> g_havokTimingFixMissingOriginalLogged{ false };
        static std::atomic<bool> g_havokTimingFixWriteFailureLogged{ false };

        constexpr std::uintptr_t kFunc_BhkWorldSetDeltaTime = 0x1DF7120;
        constexpr std::uintptr_t kHookSite_BhkWorldSetDeltaTimeMainCall = 0x0D84BD0;
        static std::atomic<std::uint64_t> g_physicsHookFrameClock{ 1 };
        constexpr std::uint64_t kNativeGrabHapticRuntimeSettingCheckIntervalFrames = 90;
        constexpr DWORD kVirtualMemoryCommitReserve = 0x00001000u | 0x00002000u;
        constexpr DWORD kVirtualMemoryRelease = 0x00008000u;
        constexpr DWORD kPageExecuteRead = 0x00000020u;
        constexpr DWORD kPageExecuteReadWrite = 0x00000040u;

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

        static std::atomic<std::uint64_t> g_nativeGrabHapticRuntimeSettingNextCheckFrame{ 0 };
        static NativeGrabHapticBinarySettingState g_nativeGrabHapticRolloverState;
        static NativeGrabHapticFloatSettingState g_nativeGrabHapticHoverIntensityState;
        static NativeGrabHapticFloatSettingState g_nativeGrabHapticHoverDurationState;

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

            if (!g_rockConfig.rockHavokTimingFixEnabled) {
                return;
            }

            const float accumulatedDeltaSeconds =
                readBhkWorldFloatGlobal(offsets::kData_BhkWorldAccumulatedDeltaSeconds, rawDeltaSeconds);
            const float oldSubstepDeltaSeconds =
                readBhkWorldFloatGlobal(offsets::kData_BhkWorldSubstepDeltaSeconds, rawDeltaSeconds);
            const auto oldSubstepCount =
                readBhkWorldUintGlobal(offsets::kData_BhkWorldSubstepCount, 1);
            const auto decision = havok_timing_fix_policy::evaluateTimingFix(havok_timing_fix_policy::TimingFixInput{
                .rawDeltaSeconds = rawDeltaSeconds,
                .accumulatedDeltaSeconds = accumulatedDeltaSeconds,
                .minPhysicsFrameRate = g_rockConfig.rockHavokTimingFixMinPhysicsFrameRate,
                .maxSubsteps = g_rockConfig.rockHavokTimingFixMaxSubsteps,
            });

            if (!decision.valid) {
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

    }

    void advancePhysicsHookFrameClock()
    {
        g_physicsHookFrameClock.fetch_add(1, std::memory_order_acq_rel);
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
            .rockEnabled = g_rockConfig.rockEnabled,
            .suppressionEnabled = g_rockConfig.rockSuppressNativeGrabHoverHaptics,
        };
        const bool shouldSuppress = native_grab_haptic_suppression::shouldSuppressNativeGrabHoverHaptics(input);
        const bool shouldRestore = native_grab_haptic_suppression::shouldRestoreNativeGrabHoverHaptics(nativeGrabHapticSuppressionApplied(), input);
        if (!shouldSuppress && !shouldRestore) {
            return;
        }

        const auto currentFrame = g_physicsHookFrameClock.load(std::memory_order_acquire);
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

    bool isMovableStaticPlayerContactTarget(RE::bhkWorld* bhkWorld, RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t layer)
    {
        if (!bhkWorld || !world || !collision_layer_policy::isPlayerCharacterControllerSupportLayer(layer)) {
            return false;
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
        return baseForm && baseForm->Is(RE::ENUM_FORM_ID::kMSTT);
    }

    collision_layer_policy::PlayerCharacterControllerContactPolicyDecision evaluatePlayerControllerTargetBody(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* world,
        std::uint32_t rawBodyId,
        bool objectFilterEnabled)
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
                .filterEnabled = objectFilterEnabled,
                .playerController = true,
                .targetLayerKnown = true,
                .targetLayer = layer,
                .targetIsMovableStatic = isMovableStaticPlayerContactTarget(bhkWorld, world, bodyId, layer),
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
            const bool playerController = isPlayerCharacterController(controller);
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
                if (playerControllerFilterActive && playerControllerFilterEnabled &&
                    std::string_view(contactBuffers.reason) == "missingManifoldEntries") {
                    const auto clearResult = held_grab_cc_policy::clearGeneratedConstraintOnlyContacts(contactBuffers);
                    if (clearResult.valid) {
                        ROCK_LOG_SAMPLE_DEBUG(CC,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "Cleared {} player character-controller constraint-only contacts before original listener reason={} heldFilter={} playerObjectFilter={}",
                            clearResult.removedPairCount,
                            clearResult.reason,
                            heldFilterActive ? "on" : "off",
                            playerControllerFilterActive ? "on" : "off");
                    }
                }
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
            int removedPlayerMovableStaticPairs = 0;
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
                    const auto decision = evaluatePlayerControllerTargetBody(
                        playerBhkWorld,
                        playerHknpWorld,
                        bodyId,
                        playerControllerFilterEnabled);
                    if (decision.suppress) {
                        ++removedPlayerObjectPairs;
                        if (std::string_view(decision.reason) == "movableStaticSupportLayer") {
                            ++removedPlayerMovableStaticPairs;
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
                        "Filtered {} character-controller contacts before original listener kept={} originalPairs={} heldRemoved={} playerObjectRemoved={} playerNonSupportRemoved={} playerMovableStaticRemoved={} playerSupportPreserved={} playerUnknownPreserved={}",
                        filterResult.removedPairCount,
                        filterResult.keptPairCount,
                        filterResult.originalPairCount,
                        removedHeldPairs,
                        removedPlayerObjectPairs,
                        removedPlayerNonSupportPairs,
                        removedPlayerMovableStaticPairs,
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
