#include "physics-interaction/weapon/WeaponTransitionAnimationAcceleration.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/PlayerCharacter.h"

#include <REL/Relocation.h>
#include <Windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>

namespace rock::weapon_transition_animation_acceleration
{
    namespace
    {
        using Direction =
            weapon_transition_animation_acceleration_policy::Direction;
        using EvaluateWeaponSpeedChannel = void (*)(void*, std::uint8_t);

        constexpr std::uintptr_t kPrimaryWeaponSpeedEvaluator = 0x0E40160;
        constexpr std::uintptr_t kLeftWeaponSpeedEvaluator = 0x0E3FE90;
        constexpr std::size_t kChannelOutputOffset = 0x18;
        constexpr std::size_t kChannelOwnerOffset = 0x20;
        constexpr std::uintptr_t kEncodedStateMask = 0x7;
        constexpr std::uint32_t kInvalidNativeState = 0xFFFFFFFFu;
        constexpr std::array<std::uint8_t, 17> kExpectedEvaluatorEntry{
            0x40, 0x53,
            0x48, 0x83, 0xEC, 0x30,
            0x4C, 0x8B, 0x49, 0x20,
            0x48, 0x8B, 0xD9,
            0x48, 0x83, 0xC1, 0x20,
        };

        struct RuntimeLease
        {
            RE::PlayerCharacter* player{ nullptr };
            Identity identity{};
            Direction direction{ Direction::Draw };
            std::chrono::steady_clock::time_point requestedAt{};
            std::uint64_t sequence{ 0 };
            bool active{ false };
        };

        EvaluateWeaponSpeedChannel s_originalPrimaryEvaluator = nullptr;
        EvaluateWeaponSpeedChannel s_originalLeftEvaluator = nullptr;
        std::atomic<bool> s_installed{ false };
        std::atomic<DWORD> s_ownerThreadId{ 0 };

        // PlayerCharacter objects are engine-heap aligned. The low three bits
        // encode the exact native transition state (2 draw / 5 sheathe), so
        // the animation thread consumes one coherent atomic lease snapshot.
        std::atomic<std::uintptr_t> s_encodedLease{ 0 };
        std::atomic<std::uint32_t> s_primarySamples{ 0 };
        std::atomic<std::uint32_t> s_leftSamples{ 0 };
        std::atomic<std::uint32_t> s_lastPrimarySpeedBits{ 0 };
        std::atomic<std::uint32_t> s_lastLeftSpeedBits{ 0 };
        std::atomic<DWORD> s_evaluationThreadId{ 0 };
        std::atomic<bool> s_multipleEvaluationThreads{ false };
        RuntimeLease s_runtimeLease{};
        std::uint64_t s_nextSequence{ 0 };

        [[nodiscard]] bool claimOrValidateOwnerThread() noexcept
        {
            const DWORD currentThreadId = GetCurrentThreadId();
            DWORD expectedThreadId = 0;
            (void)s_ownerThreadId.compare_exchange_strong(
                expectedThreadId,
                currentThreadId,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
            return s_ownerThreadId.load(std::memory_order_acquire) ==
                   currentThreadId;
        }

        [[nodiscard]] std::uintptr_t encodeLease(
            RE::PlayerCharacter* player,
            const Direction direction) noexcept
        {
            auto* actor = static_cast<RE::Actor*>(player);
            const auto actorAddress = reinterpret_cast<std::uintptr_t>(actor);
            if (!actorAddress || (actorAddress & kEncodedStateMask) != 0) {
                return 0;
            }
            return actorAddress |
                   weapon_transition_animation_acceleration_policy::
                       transitionState(direction);
        }

        [[nodiscard]] float elapsedSeconds() noexcept
        {
            if (!s_runtimeLease.active) {
                return 0.0f;
            }
            return (std::max)(
                0.0f,
                std::chrono::duration<float>(
                    std::chrono::steady_clock::now() -
                    s_runtimeLease.requestedAt)
                    .count());
        }

        void resetSampleDiagnostics() noexcept
        {
            s_primarySamples.store(0, std::memory_order_relaxed);
            s_leftSamples.store(0, std::memory_order_relaxed);
            s_lastPrimarySpeedBits.store(0, std::memory_order_relaxed);
            s_lastLeftSpeedBits.store(0, std::memory_order_relaxed);
            s_evaluationThreadId.store(0, std::memory_order_relaxed);
            s_multipleEvaluationThreads.store(false, std::memory_order_relaxed);
        }

        void clearLease(
            const char* reason,
            const std::uint32_t nativeState) noexcept
        {
            s_encodedLease.store(0, std::memory_order_release);
            if (!s_runtimeLease.active) {
                return;
            }

            const auto primarySamples =
                s_primarySamples.load(std::memory_order_relaxed);
            const auto leftSamples =
                s_leftSamples.load(std::memory_order_relaxed);
            const float lastPrimarySpeed = std::bit_cast<float>(
                s_lastPrimarySpeedBits.load(std::memory_order_relaxed));
            const float lastLeftSpeed = std::bit_cast<float>(
                s_lastLeftSpeedBits.load(std::memory_order_relaxed));
            ROCK_LOG_INFO(
                Weapon,
                "Weapon transition animation acceleration ended sequence={} direction={} reason={} formID={:08X} instance={:#x} state={} elapsedMs={:.1f} primarySamples={} leftSamples={} lastPrimary={:.3f} lastLeft={:.3f} evaluatorThread={} multipleEvaluatorThreads={}",
                s_runtimeLease.sequence,
                s_runtimeLease.direction == Direction::Draw ?
                    "draw" :
                    "sheathe",
                reason ? reason : "unknown",
                s_runtimeLease.identity.formID,
                s_runtimeLease.identity.instanceData,
                nativeState,
                elapsedSeconds() * 1000.0f,
                primarySamples,
                leftSamples,
                lastPrimarySpeed,
                lastLeftSpeed,
                s_evaluationThreadId.load(std::memory_order_relaxed),
                s_multipleEvaluationThreads.load(
                    std::memory_order_relaxed) ?
                    "yes" :
                    "no");
            s_runtimeLease = {};
        }

        void recordEvaluationThread() noexcept
        {
            const DWORD currentThreadId = GetCurrentThreadId();
            DWORD expectedThreadId = 0;
            if (s_evaluationThreadId.compare_exchange_strong(
                    expectedThreadId,
                    currentThreadId,
                    std::memory_order_relaxed,
                    std::memory_order_relaxed) ||
                expectedThreadId == currentThreadId) {
                return;
            }
            s_multipleEvaluationThreads.store(true, std::memory_order_relaxed);
        }

        void accelerateEvaluatedChannel(
            void* channel,
            const bool leftChannel) noexcept
        {
            const auto encodedLease =
                s_encodedLease.load(std::memory_order_acquire);
            if (!channel || encodedLease == 0) {
                return;
            }

            RE::Actor* owner = nullptr;
            std::memcpy(
                &owner,
                reinterpret_cast<const std::byte*>(channel) +
                    kChannelOwnerOffset,
                sizeof(owner));
            auto* leasedPlayer = reinterpret_cast<RE::Actor*>(
                encodedLease & ~kEncodedStateMask);
            const std::uint32_t transitionState = static_cast<std::uint32_t>(
                encodedLease & kEncodedStateMask);
            const Direction direction =
                weapon_transition_animation_acceleration_policy::
                    directionForTransitionState(transitionState);
            const std::uint32_t nativeState =
                owner == leasedPlayer ?
                    f4vr::getNativeWeaponState(owner) :
                    kInvalidNativeState;
            if (!weapon_transition_animation_acceleration_policy::
                    shouldAccelerateSample(
                        owner == leasedPlayer,
                        direction,
                        nativeState)) {
                if (owner == leasedPlayer &&
                    nativeState ==
                        weapon_transition_animation_acceleration_policy::
                            terminalState(direction)) {
                    auto expectedLease = encodedLease;
                    (void)s_encodedLease.compare_exchange_strong(
                        expectedLease,
                        0,
                        std::memory_order_acq_rel,
                        std::memory_order_acquire);
                }
                return;
            }

            float sampledSpeed = 0.0f;
            std::memcpy(
                &sampledSpeed,
                reinterpret_cast<const std::byte*>(channel) +
                    kChannelOutputOffset,
                sizeof(sampledSpeed));
            const float acceleratedSpeed =
                std::isfinite(sampledSpeed) &&
                    sampledSpeed >=
                        weapon_transition_animation_acceleration_policy::
                            kAcceleratedSpeedMultiplier ?
                sampledSpeed :
                weapon_transition_animation_acceleration_policy::
                    kAcceleratedSpeedMultiplier;
            std::memcpy(
                reinterpret_cast<std::byte*>(channel) +
                    kChannelOutputOffset,
                &acceleratedSpeed,
                sizeof(acceleratedSpeed));

            recordEvaluationThread();
            auto& samples = leftChannel ? s_leftSamples : s_primarySamples;
            auto& lastSpeedBits = leftChannel ?
                s_lastLeftSpeedBits :
                s_lastPrimarySpeedBits;
            samples.fetch_add(1, std::memory_order_relaxed);
            lastSpeedBits.store(
                std::bit_cast<std::uint32_t>(sampledSpeed),
                std::memory_order_relaxed);
        }

        __declspec(noinline) void onEvaluatePrimaryWeaponSpeed(
            void* channel,
            const std::uint8_t graphPass)
        {
            if (s_originalPrimaryEvaluator) {
                s_originalPrimaryEvaluator(channel, graphPass);
            }
            accelerateEvaluatedChannel(channel, false);
        }

        __declspec(noinline) void onEvaluateLeftWeaponSpeed(
            void* channel,
            const std::uint8_t graphPass)
        {
            if (s_originalLeftEvaluator) {
                s_originalLeftEvaluator(channel, graphPass);
            }
            accelerateEvaluatedChannel(channel, true);
        }
    }

    bool install() noexcept
    {
        if (s_installed.load(std::memory_order_acquire)) {
            return true;
        }
        if (!REL::Module::IsVR() ||
            REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
            ROCK_LOG_ERROR(
                Init,
                "Weapon transition animation acceleration unavailable: unsupported runtime");
            return false;
        }

        void* primaryOriginal =
            reinterpret_cast<void*>(s_originalPrimaryEvaluator);
        const bool primaryInstalled = entry_trampoline_hook::install(
            "primary weapon-speed animation channel",
            kPrimaryWeaponSpeedEvaluator,
            kExpectedEvaluatorEntry.data(),
            kExpectedEvaluatorEntry.size(),
            reinterpret_cast<void*>(&onEvaluatePrimaryWeaponSpeed),
            primaryOriginal);
        s_originalPrimaryEvaluator =
            reinterpret_cast<EvaluateWeaponSpeedChannel>(primaryOriginal);

        void* leftOriginal = reinterpret_cast<void*>(s_originalLeftEvaluator);
        const bool leftInstalled = entry_trampoline_hook::install(
            "left weapon-speed animation channel",
            kLeftWeaponSpeedEvaluator,
            kExpectedEvaluatorEntry.data(),
            kExpectedEvaluatorEntry.size(),
            reinterpret_cast<void*>(&onEvaluateLeftWeaponSpeed),
            leftOriginal);
        s_originalLeftEvaluator =
            reinterpret_cast<EvaluateWeaponSpeedChannel>(leftOriginal);

        const bool ready =
            primaryInstalled && leftInstalled &&
            s_originalPrimaryEvaluator && s_originalLeftEvaluator;
        s_encodedLease.store(0, std::memory_order_release);
        s_ownerThreadId.store(0, std::memory_order_release);
        s_installed.store(ready, std::memory_order_release);
        if (!ready) {
            ROCK_LOG_ERROR(
                Init,
                "Weapon transition animation acceleration disabled: both hand-aware channels are required");
        }
        return ready;
    }

    RequestResult request(const RequestInput& input) noexcept
    {
        if (!s_installed.load(std::memory_order_acquire)) {
            return RequestResult::NotInstalled;
        }
        if (!input.player || !input.identity.valid()) {
            return RequestResult::MissingInput;
        }
        if (!claimOrValidateOwnerThread()) {
            return RequestResult::WrongThread;
        }

        const auto encodedLease = encodeLease(input.player, input.direction);
        if (!encodedLease) {
            return RequestResult::UnencodablePlayer;
        }
        if (s_runtimeLease.active &&
            s_runtimeLease.player == input.player &&
            s_runtimeLease.identity == input.identity &&
            s_runtimeLease.direction == input.direction &&
            s_encodedLease.load(std::memory_order_acquire) == encodedLease) {
            return RequestResult::AlreadyArmed;
        }
        if (s_runtimeLease.active) {
            clearLease("superseded", kInvalidNativeState);
        }

        resetSampleDiagnostics();
        s_runtimeLease = RuntimeLease{
            .player = input.player,
            .identity = input.identity,
            .direction = input.direction,
            .requestedAt = std::chrono::steady_clock::now(),
            .sequence = ++s_nextSequence,
            .active = true,
        };
        s_encodedLease.store(encodedLease, std::memory_order_release);
        ROCK_LOG_INFO(
            Weapon,
            "Weapon transition animation acceleration armed sequence={} direction={} formID={:08X} instance={:#x} equipIndex={} multiplier={:.1f}",
            s_runtimeLease.sequence,
            input.direction == Direction::Draw ? "draw" : "sheathe",
            input.identity.formID,
            input.identity.instanceData,
            input.identity.equipIndex,
            weapon_transition_animation_acceleration_policy::
                kAcceleratedSpeedMultiplier);
        return RequestResult::Armed;
    }

    void service(const ServiceInput& input) noexcept
    {
        if (!s_runtimeLease.active) {
            return;
        }
        if (!claimOrValidateOwnerThread()) {
            s_encodedLease.store(0, std::memory_order_release);
            return;
        }

        const bool identityMatches =
            input.player == s_runtimeLease.player &&
            input.identity == s_runtimeLease.identity;
        const float elapsed = elapsedSeconds();
        const auto action =
            weapon_transition_animation_acceleration_policy::
                classifyLifecycle(
                    input.runtimeAllowed,
                    identityMatches,
                    s_runtimeLease.direction,
                    input.nativeWeaponState,
                    elapsed);
        switch (action) {
        case weapon_transition_animation_acceleration_policy::
            LifecycleAction::Complete:
            clearLease("native-complete", input.nativeWeaponState);
            break;
        case weapon_transition_animation_acceleration_policy::
            LifecycleAction::Cancel: {
            const char* reason = !input.runtimeAllowed ?
                "runtime-unavailable" :
                !identityMatches ?
                    "identity-changed" :
                    !held_weapon_equip_state_policy::
                            isValidNativeWeaponState(
                                input.nativeWeaponState) ?
                        "invalid-native-state" :
                        "watchdog-timeout";
            clearLease(reason, input.nativeWeaponState);
            break;
        }
        case weapon_transition_animation_acceleration_policy::
            LifecycleAction::Accelerate:
        case weapon_transition_animation_acceleration_policy::
            LifecycleAction::KeepPending:
        default:
            break;
        }
    }

    void cancel(const char* reason) noexcept
    {
        s_encodedLease.store(0, std::memory_order_release);
        if (!s_runtimeLease.active) {
            return;
        }
        const DWORD ownerThreadId =
            s_ownerThreadId.load(std::memory_order_acquire);
        if (ownerThreadId != 0 && ownerThreadId != GetCurrentThreadId()) {
            ROCK_LOG_ERROR(
                Weapon,
                "Weapon transition animation acceleration cancellation arrived on non-owner thread owner={} caller={}; atomic lease disabled and runtime record retained for owner cleanup",
                ownerThreadId,
                GetCurrentThreadId());
            return;
        }
        clearLease(reason, kInvalidNativeState);
    }

}
