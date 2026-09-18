
#include <array>
#include <atomic>
#include <cstdint>
#include <string>

#include <windows.h>

#include "api/FRIKApiV2.h"
#define ROCK_API_EXPORTS
#include "RockConfig.h"
#include "rock_support/GameIniOverrides.h"
#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapture.h"
#include "physics-interaction/core/PhysicsCreationGatePolicy.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/debug/DebugBodyOverlay.h"
#include "physics-interaction/grab/SavedGrabOffsetStore.h"
#include "physics-interaction/hand/NativeWandVisualSuppression.h"
#include "physics-interaction/input/DebugControllerRuntime.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HeldScenePresentation.h"
#include "physics-interaction/native/HeldWeaponInstantTransition.h"
#include "physics-interaction/native/NativeCharacterProxySafety.h"
#include "physics-interaction/native/NativeCollisionFilterSafety.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/NativeRagdollSafety.h"
#include "physics-interaction/native/NativeShapeCastSafety.h"
#include "physics-interaction/native/WeaponActionTrace.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/telemetry/DynamicColliderTrace.h"
#include "physics-interaction/timing/RockGameTiming.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/AuthoredWeaponGripCacheStore.h"
#include "physics-interaction/weapon/WeaponTransitionAnimationAcceleration.h"
#include "physics-interaction/weapon/telemetry/VanillaWeaponAlignmentTelemetry.h"
#include "physics-interaction/weapon/telemetry/ScopeTransitionTelemetry.h"
#include "physics-interaction/weapon/telemetry/NativeScopeShotDiagnostics.h"
#include "physics-interaction/weapon/scope/NativeScopeData.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"

namespace
{
    using namespace rock;

    const F4SE::MessagingInterface* s_messaging = nullptr;

    PhysicsInteraction* s_physicsInteraction = nullptr;
    bool s_physicsPublished = false;

    bool s_frikAvailable = false;
    /*
     * ROCK handled kSkeletonReady for FRIK's current skeleton. FRIK broadcasts
     * it after that skeleton's first world final, so on that first frame
     * AfterArmSolve is skipped and FrameEnd ticks without physics.
     */
    bool s_frikSkeletonAnnounced = false;

    bool s_pluginLoaded = false;
    std::atomic<std::uint32_t> s_providerGeneration{ 1 };
    std::atomic<std::uint32_t> s_skeletonGeneration{ 1 };
    std::atomic<bool> s_physicsCreationRequested{ false };
    std::atomic<std::uint32_t> s_physicsCreationReadyDeferralFrames{ 0 };
    physics_creation_gate_policy::WorldStabilityState s_physicsCreationWorldStability{};
    std::uint32_t s_physicsCreationGateLogCounter = 0;

    struct PlayerPhysicsWorlds
    {
        RE::bhkWorld* bhk = nullptr;
        RE::hknpWorld* hknp = nullptr;
    };

    const char* physicsCreationBlockReasonName(physics_creation_gate_policy::CreationBlockReason reason)
    {
        using Reason = physics_creation_gate_policy::CreationBlockReason;
        switch (reason) {
        case Reason::None:
            return "none";
        case Reason::RockDisabled:
            return "rock-disabled";
        case Reason::ProviderUnavailable:
            return "provider-unavailable";
        case Reason::SkeletonNotReady:
            return "skeleton-not-ready";
        case Reason::ReadyEventDeferred:
            return "ready-event-deferred";
        case Reason::MenuBlocked:
            return "menu-blocked";
        case Reason::WorldUnavailable:
            return "world-unavailable";
        case Reason::WorldUnstable:
            return "world-unstable";
        default:
            return "unknown";
        }
    }

    void resetPhysicsCreationGate()
    {
        physics_creation_gate_policy::resetWorldStability(s_physicsCreationWorldStability);
        s_physicsCreationGateLogCounter = 0;
    }

    void requestDeferredPhysicsCreation()
    {
        s_physicsCreationRequested.store(true, std::memory_order_release);
        s_physicsCreationReadyDeferralFrames.store(
            physics_creation_gate_policy::kSkeletonReadyCreateDeferralFrames,
            std::memory_order_release);
        resetPhysicsCreationGate();
    }

    PlayerPhysicsWorlds samplePlayerPhysicsWorlds()
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            return {};
        }

        auto* cell = player->GetParentCell();
        if (!cell) {
            return {};
        }

        auto* bhk = cell->GetbhkWorld();
        if (!bhk) {
            return {};
        }

        return {
            .bhk = bhk,
            .hknp = havok_runtime::getHknpWorldFromBhk(bhk),
        };
    }

    std::uint32_t bumpGeneration(std::atomic<std::uint32_t>& generation)
    {
        const auto next = generation.fetch_add(1, std::memory_order_acq_rel) + 1;
        return next == 0 ? generation.fetch_add(1, std::memory_order_acq_rel) + 1 : next;
    }

    void publishPhysicsInteractionIfReady()
    {
        if (!s_physicsInteraction || !s_physicsInteraction->isInitialized() || s_physicsPublished) {
            return;
        }

        PhysicsInteraction::s_hooksEnabled.store(true, std::memory_order_release);
        rock::provider::setPhysicsInteractionInstance(s_physicsInteraction);
        s_physicsPublished = true;
        logger::info("ROCK: PhysicsInteraction initialized and published.");
    }

    void createPhysicsInteraction()
    {
        logger::info("ROCK: Creating PhysicsInteraction (skeleton became ready)...");

        s_physicsInteraction = new PhysicsInteraction(
            s_skeletonGeneration.load(std::memory_order_acquire),
            s_providerGeneration.load(std::memory_order_acquire));
        s_physicsInteraction->init();

        publishPhysicsInteractionIfReady();
        if (!s_physicsPublished) {
            logger::warn("ROCK: PhysicsInteraction init deferred; hooks/API remain disabled until lazy init succeeds.");
        }
    }

    void destroyPhysicsInteraction(rock::provider::RockProviderLifecycleReason reason);
    void reconcileNativeScopeGeometryOwnership();

    void ensurePhysicsInteractionForReadySkeleton(const runtime_state::RuntimeFrameSnapshot& runtime)
    {
        // ROCK creation is event-driven when FRIK announces skeleton readiness.
        // The frame loop re-requests a dropped creation only for a skeleton
        // FRIK has announced: this runs at FrameBegin, before the kSkeletonReady
        // FRIK broadcasts later in the frame that builds it, and that event
        // would rebuild whatever was created here.
        if (!s_physicsCreationRequested.load(std::memory_order_acquire) &&
            !s_physicsInteraction &&
            s_frikSkeletonAnnounced &&
            runtime.visualAuthorityAvailable &&
            runtime.localSkeletonReady) {
            s_physicsCreationRequested.store(true, std::memory_order_release);
            s_physicsCreationReadyDeferralFrames.store(0, std::memory_order_release);
        }

        if (!s_physicsCreationRequested.load(std::memory_order_acquire)) {
            return;
        }

        const auto worlds = samplePlayerPhysicsWorlds();
        const auto readyDeferralFrames = s_physicsCreationReadyDeferralFrames.load(std::memory_order_acquire);
        const physics_creation_gate_policy::CreationGateInput gateInput{
            .rockEnabled = true,
            .providerAvailable = s_frikAvailable && runtime.visualAuthorityAvailable,
            .skeletonReady = runtime.localSkeletonReady,
            .runtimeMenuBlocking = runtime.localMenuBlocking,
            .compatibilityConfigBlocking = runtime.compatibilityConfigBlocking,
            .bhkWorld = reinterpret_cast<std::uintptr_t>(worlds.bhk),
            .hknpWorld = reinterpret_cast<std::uintptr_t>(worlds.hknp),
            .readyDeferralFrames = readyDeferralFrames,
        };
        const auto decision = physics_creation_gate_policy::evaluateCreationGate(s_physicsCreationWorldStability, gateInput);
        if (readyDeferralFrames > 0) {
            s_physicsCreationReadyDeferralFrames.fetch_sub(1, std::memory_order_acq_rel);
        }

        if (!decision.keepRequestPending) {
            s_physicsCreationRequested.store(false, std::memory_order_release);
        }

        if (!decision.canCreate) {
            if ((s_physicsCreationGateLogCounter++ % 90u) == 0u) {
                logger::debug(
                    "ROCK: Physics creation deferred reason={} stableFrames={} bhk={} hknp={} localMenu={} compatibilityConfig={} readyDeferral={}",
                    physicsCreationBlockReasonName(decision.blockReason),
                    decision.stableWorldFrames,
                    static_cast<const void*>(worlds.bhk),
                    static_cast<const void*>(worlds.hknp),
                    runtime.localMenuBlocking ? "yes" : "no",
                    runtime.compatibilityConfigBlocking ? "yes" : "no",
                    readyDeferralFrames);
            }
            return;
        }

        if (s_physicsInteraction) {
            logger::info("ROCK: Recreating PhysicsInteraction after deferred skeleton-ready gate.");
            destroyPhysicsInteraction(rock::provider::RockProviderLifecycleReason::SkeletonReady);
        }

        createPhysicsInteraction();
        s_physicsCreationRequested.store(false, std::memory_order_release);
        s_physicsCreationReadyDeferralFrames.store(0, std::memory_order_release);
        resetPhysicsCreationGate();
    }

    void destroyPhysicsInteraction(
        rock::provider::RockProviderLifecycleReason reason = rock::provider::RockProviderLifecycleReason::ProviderLost)
    {
        if (!s_physicsInteraction) {
            return;
        }

        logger::info("ROCK: Destroying PhysicsInteraction (skeleton released)...");

        PhysicsInteraction::s_hooksEnabled.store(false, std::memory_order_release);
        s_physicsInteraction->noteProviderLifecycle(
            s_providerGeneration.load(std::memory_order_acquire),
            reason);
        s_physicsInteraction->shutdown(reason);
        rock::provider::dispatchFrameCallbacks(*s_physicsInteraction);

        rock::provider::setPhysicsInteractionInstance(nullptr);
        rock::provider::clearExternalBodiesForProviderLoss();
        s_physicsPublished = false;

        delete s_physicsInteraction;
        s_physicsInteraction = nullptr;

        // Every hand world claim belongs to an owner inside PhysicsInteraction;
        // none may outlive it in FRIK.
        frik_hand_world_authority::clearAllClaims();

        logger::info("ROCK: PhysicsInteraction destroyed.");
    }

    void clearUnavailableRuntimeInputState()
    {
        weapon_action_trace::invalidateContext();
        weapon_transition_animation_acceleration::cancel(
            "rock-runtime-unavailable");
        authored_weapon_grip_capture::setEnabled(false);
        input_remap_runtime::setGameplayInputAllowed(false);
        input_remap_runtime::setWeaponDrawn(false);
        input_remap_runtime::setRealMeleeWeaponEquipped(false);
        input_remap_runtime::setHandHeldWeapon(false, false);
        input_remap_runtime::setHandHeldWeapon(true, false);
        input_remap_runtime::setEquippedWeaponFiringGripInputActive(false);
        input_remap_runtime::setEquippedWeaponPrimaryDetached(false);
        input_remap_runtime::setEquippedWeaponShoulderSheathActive(false);
    }

    /*
     * Per-frame housekeeping that runs with or without a FRIK skeleton
     * (FRIK's FrameBegin phase): config reload, INI enforcement, runtime
     * state, input gating and the physics creation gate.
     */
    void prepareRuntimeFrame()
    {
        performance_profiler::ScopedTimer runtimeTimer(performance_profiler::Scope::RuntimePreparation);

        if (!s_pluginLoaded) {
            clearUnavailableRuntimeInputState();
            return;
        }

        const bool immersiveScopesWereEnabled = g_rockConfig.rockEnableImmersiveScopes;
        g_rockConfig.processPendingConfigReload();
        reconcileNativeScopeGeometryOwnership();
        if (immersiveScopesWereEnabled != g_rockConfig.rockEnableImmersiveScopes) {
            if (!g_rockConfig.rockEnableImmersiveScopes && s_physicsInteraction) {
                s_physicsInteraction->synchronizeNativeScopePresentationAfterFrikUpdate();
            }
            ROCK_LOG_INFO(Input, "Immersive scopes {}: activation={}",
                g_rockConfig.rockEnableImmersiveScopes ? "enabled" : "disabled",
                g_rockConfig.rockEnableImmersiveScopes ? "button-hold" : "vanilla-cone");
        }
        game_ini_overrides::update();
        advanceNativeRuntimeSettingFrameClock();
        enforceNativeMeleeRuntimeSuppression();

        if (!s_frikAvailable) {
            clearUnavailableRuntimeInputState();
            return;
        }

        input_remap_runtime::installInputRemapHooks();

        runtime_state::updateFrame(runtime_state::RuntimeFrameInput{
            .visualAuthorityAvailable = frik_visual_authority::isAvailable(),
            .visualSkeletonReadyHint = frik_visual_authority::isSkeletonReadyHint(),
            .compatibilityConfigBlocking = frik_visual_authority::isCompatibilityConfigBlocking(),
        });
        const auto& runtime = runtime_state::currentFrame();
        if (runtime.localSkeletonReady &&
            !runtime.compatibilityConfigBlocking) {
            native_wand_visual_suppression::enforce();
        }
        const bool authoredGripCaptureRuntimeEnabled =
            runtime.localSkeletonReady &&
            !runtime.compatibilityConfigBlocking;
        authored_weapon_grip_capture::setEnabled(
            authoredGripCaptureRuntimeEnabled);
        const bool gameplayInputAllowed =
            runtime.localSkeletonReady &&
            !runtime.localMenuBlocking &&
            !runtime.compatibilityConfigBlocking;
        input_remap_runtime::setWeaponDrawn(runtime.weaponDrawn);
        input_remap_runtime::setGameplayInputAllowed(gameplayInputAllowed);
        debug_controller_runtime::update(gameplayInputAllowed, runtime.deltaSeconds);

        ensurePhysicsInteractionForReadySkeleton(runtime);
    }

    // ROCK's interaction update proper; only meaningful with a skeleton.
    void updatePhysicsInteractionFrame()
    {
        if (!s_pluginLoaded || !s_frikAvailable || !s_physicsInteraction) {
            return;
        }
        /*
         * Native menu/equip animation work completed earlier in this game
         * frame. Reconcile the exact equipped instance first so authored
         * grip and collision never consume a stale hidden Weapon graph.
         */
        {
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::WeaponEquipTransition);
            s_physicsInteraction->updateEquippedWeaponTransition();
        }
        /*
         * Reconstruct the Bethesda-authored primary grip before ROCK's
         * collision/probe/grip pass so every weapon-relative subsystem sees
         * the same corrected frame that will be rendered.
         */
        {
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::AuthoredPrimaryGrip);
            s_physicsInteraction->updateAuthoredPrimaryFiringGrip();
        }
        {
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::InteractionUpdate);
            s_physicsInteraction->update();
        }
        {
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::ProviderPublication);
            publishPhysicsInteractionIfReady();
        }
    }

    // Never-zero ROCK frame sequence; zero is reserved for "no frame yet".
    std::uint64_t s_schedulerSequence = 0;

    [[nodiscard]] std::uint64_t nextFrameSequence(const std::uint64_t current) noexcept
    {
        const std::uint64_t next = current + 1;
        return next == 0 ? 1 : next;
    }

    using NativeScopeStateTransitionFunc = void (*)(RE::PlayerCharacter*, bool);
    NativeScopeStateTransitionFunc s_originalNativeScopeStateTransition = nullptr;
    /*
     * The native scope geometry decision site is shared ground: a scope mod
     * can gate activation there too. ROCK claims it only while immersive
     * scopes are on, since with them off the wrapper would only pass the
     * native verdict through. The claim waits for the config, is retried
     * when the setting turns on, and is never handed back.
     */
    enum class NativeScopeGeometryHook : std::uint8_t
    {
        Unclaimed,
        Claimed,
        Unavailable,
    };
    NativeScopeGeometryHook s_nativeScopeGeometryHook = NativeScopeGeometryHook::Unclaimed;
    bool s_immersiveScopesForcedOffLogged = false;
    bool s_nativeScopeHookLeftUninstalledLogged = false;
    bool s_manualScopeDirectTransitionActive = false;
    std::uint64_t s_manualScopeConfiguredWeaponGeneration = 0;
    std::uint32_t s_manualScopeConfiguredOverlayIndex = 0;
    std::uintptr_t s_manualScopeConfiguredWorldScope = 0;
    std::uint64_t s_manualScopeConfigureFailureLoggedGeneration = 0;

    bool configureNativeWorldScopeForManualTarget(
        const std::uint64_t weaponGenerationKey,
        const std::uint32_t overlayIndex)
    {
        if (weaponGenerationKey == 0) {
            return false;
        }
        static const bool configureEntryValidated = []() {
            if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
                logger::critical("ROCK: Native world-scope configuration requires Fallout4VR.exe 1.2.72.");
                return false;
            }

            constexpr std::array<std::uint8_t, 10> kExpectedConfigurePrefix{
                0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x20
            };
            const auto configureAddress = REL::Offset(rock::offsets::kFunc_NativeWorldScopeConfigure).address();
            std::array<std::uint8_t, kExpectedConfigurePrefix.size()> actualPrefix{};
            if (!rock::native_memory::guardedCopyFromMemory(
                    reinterpret_cast<const void*>(configureAddress),
                    actualPrefix.data(),
                    actualPrefix.size()) ||
                actualPrefix != kExpectedConfigurePrefix) {
                logger::critical("ROCK: Native world-scope configure validation failed at 0x{:X}.", configureAddress);
                return false;
            }
            return true;
        }();
        if (!configureEntryValidated) {
            return false;
        }

        std::uintptr_t worldScope = 0;
        const auto singletonAddress = REL::Offset(rock::offsets::kData_NativeWorldScopeSingleton).address();
        if (!rock::native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(singletonAddress), worldScope) || worldScope == 0) {
            if (s_manualScopeConfigureFailureLoggedGeneration != weaponGenerationKey) {
                ROCK_LOG_WARN(Input, "Manual scope target generation={:016X} has no native WSScope singleton", weaponGenerationKey);
                s_manualScopeConfigureFailureLoggedGeneration = weaponGenerationKey;
            }
            return false;
        }

        std::uintptr_t primaryVtable = 0;
        const auto expectedPrimaryVtable = REL::Offset(rock::offsets::kData_NativeWorldScopePrimaryVtable).address();
        if (!rock::native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(worldScope), primaryVtable) ||
            primaryVtable != expectedPrimaryVtable) {
            if (s_manualScopeConfigureFailureLoggedGeneration != weaponGenerationKey) {
                ROCK_LOG_WARN(Input,
                    "Manual scope target generation={:016X} rejected WSScope identity vtable=0x{:X} expected=0x{:X}",
                    weaponGenerationKey,
                    primaryVtable,
                    expectedPrimaryVtable);
                s_manualScopeConfigureFailureLoggedGeneration = weaponGenerationKey;
            }
            return false;
        }

        if (s_manualScopeConfiguredWeaponGeneration == weaponGenerationKey &&
            s_manualScopeConfiguredOverlayIndex == overlayIndex &&
            s_manualScopeConfiguredWorldScope == worldScope) {
            return true;
        }

        if (!native_scope_data::configureManual(reinterpret_cast<void*>(worldScope), overlayIndex)) {
            return false;
        }
        s_manualScopeConfiguredWeaponGeneration = weaponGenerationKey;
        s_manualScopeConfiguredOverlayIndex = overlayIndex;
        s_manualScopeConfiguredWorldScope = worldScope;
        s_manualScopeConfigureFailureLoggedGeneration = 0;
        ROCK_LOG_DEBUG(Input,
            "Configured native WSScope for manual target generation={:016X} overlay={}",
            weaponGenerationKey,
            overlayIndex);
        return true;
    }

    bool onNativeScopeGeometryDecision(RE::PlayerCharacter* player, const bool nativeGeometryDecision)
    {
        bool finalGeometryDecision = nativeGeometryDecision;
        bool buttonDecisionApplied = false;
        std::uint8_t nativeScopeFlags = 0;
        const bool nativeForceDecision = rock::native_memory::tryReadField(player, rock::offsets::kPlayerCharacter_NativeScopeFlags, nativeScopeFlags) &&
            (nativeScopeFlags & rock::offsets::kPlayerCharacter_NativeScopeForceDecisionMask) != 0;
        if (g_rockConfig.rockEnableImmersiveScopes && !nativeForceDecision && s_pluginLoaded && s_frikAvailable) {
            // The native geometry callback remains installed only as the
            // verified transition boundary. ROCK deliberately discards its
            // cone result and feeds the held firing-hand button level instead.
            finalGeometryDecision = input_remap_runtime::isManualScopeActivationRequested();
            buttonDecisionApplied = true;
        }

        if (s_originalNativeScopeStateTransition) {
            s_originalNativeScopeStateTransition(player, finalGeometryDecision);
        }
        /*
         * AL feeds only ROCK's patched post-call TEST below; the original
         * transition is void and already received finalGeometryDecision.
         * Button-only activation must skip Bethesda's cone-derived approach
         * fade both while held and while idle, otherwise the cone would still
         * darken the view despite no longer owning scope activation.
         */
        return buttonDecisionApplied ? true : finalGeometryDecision;
    }

    void applyManualScopeTransition(RE::PlayerCharacter* player, bool requested)
    {
        if (!s_originalNativeScopeStateTransition) {
            return;
        }
        // One observation per hold edge. The renderer can open even if the
        // aiming setter returns early or another plugin replaces its vtable
        // entry; distinguish these before changing native UI ownership.
        const bool trace = requested != s_manualScopeDirectTransitionActive;
        std::uint32_t gunBefore = 0;
        std::uintptr_t aimingVtable = 0;
        std::uintptr_t aimingTarget = 0;
        std::uint32_t readStage = 0;
        bool nativeEntryReadable = false;
        bool nativeEntryIntact = false;
        if (trace) {
            gunBefore = f4vr::getNativeGunState(player);
            // Player constructor 0x140EED807 writes ActorState +0x128;
            // scope transition 0x140EFAA8F calls its vtable slot +0x130.
            if (native_memory::tryReadField(player, 0x128, aimingVtable) && aimingVtable != 0) {
                readStage = 1;
                if (native_memory::tryReadField(reinterpret_cast<const void*>(aimingVtable), 0x130, aimingTarget)) {
                    readStage = 2;
                }
            }
            // An unchanged vtable target does not exclude an inline detour.
            constexpr std::array<std::uint8_t, 15> nativePrefix{
                0x48, 0x89, 0x6C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xEC, 0x30
            };
            std::array<std::uint8_t, nativePrefix.size()> actualPrefix{};
            nativeEntryReadable = native_memory::guardedCopyFromMemory(
                reinterpret_cast<const void*>(REL::Offset(0xF30020).address()), actualPrefix.data(), actualPrefix.size());
            nativeEntryIntact = nativeEntryReadable && actualPrefix == nativePrefix;
        }
        s_originalNativeScopeStateTransition(player, requested);
        if (trace) {
            ROCK_LOG_DEBUG(Input,
                "Manual scope aiming transition requested={} gunBefore={} gunAfter={} aimingVtable=0x{:X} aimingTarget=0x{:X} nativeAimingTarget={} readStage={} nativeEntryReadable={} nativeEntryIntact={}",
                requested, gunBefore, f4vr::getNativeGunState(player), aimingVtable, aimingTarget,
                aimingTarget == REL::Offset(0xF30020).address(), readStage, nativeEntryReadable, nativeEntryIntact);
        }
    }

    void driveManualScopeTransitionFallback()
    {
        if (!s_originalNativeScopeStateTransition) {
            return;
        }

        auto* player = RE::PlayerCharacter::GetSingleton();
        std::uint8_t nativeScopeFlags = 0;
        const bool nativeForceDecision = player &&
            rock::native_memory::tryReadField(player, rock::offsets::kPlayerCharacter_NativeScopeFlags, nativeScopeFlags) &&
            (nativeScopeFlags & rock::offsets::kPlayerCharacter_NativeScopeForceDecisionMask) != 0;
        if (nativeForceDecision) {
            // Reload/menu transitions temporarily own the native scope state.
            return;
        }

        std::uint64_t targetWeaponGenerationKey = 0;
        std::uint32_t targetOverlayIndex = 0;
        bool directTransitionRequired = false;
        const bool targetAvailable = s_pluginLoaded && s_frikAvailable &&
            s_physicsInteraction &&
            s_physicsInteraction->tryGetManualScopePresentationTarget(
                targetWeaponGenerationKey, targetOverlayIndex, directTransitionRequired);
        const bool requested = targetAvailable &&
            input_remap_runtime::isManualScopeActivationRequested() &&
            player &&
            configureNativeWorldScopeForManualTarget(targetWeaponGenerationKey, targetOverlayIndex);

        if (requested && directTransitionRequired) {
            /*
             * The native scope update precedes ROCK's input classification
             * and current sight-anchor publication. Drive its same transition
             * after those are ready so a new hold can begin in this frame.
             * The native admission hooks then preserve geometry and open
             * ScopeMenu for this verified target on the normal native path.
             */
            applyManualScopeTransition(player, true);
            if (!s_manualScopeDirectTransitionActive) {
                ROCK_LOG_DEBUG(Input, "Manual scope direct transition engaged for explicit scope target");
            }
            s_manualScopeDirectTransitionActive = true;
            return;
        }

        if (s_manualScopeDirectTransitionActive && player) {
            applyManualScopeTransition(player, false);
            ROCK_LOG_DEBUG(Input, "Manual scope direct transition released");
        }
        s_manualScopeDirectTransitionActive = false;
        if (!requested) {
            // Native equip/menu work can reconfigure the same singleton even
            // when the next hold uses the same generated weapon identity.
            s_manualScopeConfiguredWeaponGeneration = 0;
            s_manualScopeConfiguredOverlayIndex = 0;
            s_manualScopeConfiguredWorldScope = 0;
        }
    }

    bool isManualScopeEligibleForNative(const void* weaponIdentity, const void* instanceIdentity) noexcept
    {
        // HasScope describes the equipped optic even before activation. If
        // this follows the button, Bethesda's ordinary aiming path can set
        // gun state 6 first; the later held transition then returns early
        // without opening ScopeMenu. The geometry decision owns the button.
        if (!g_rockConfig.rockEnableImmersiveScopes || !weaponIdentity || !s_pluginLoaded || !s_frikAvailable || !s_physicsInteraction) {
            return false;
        }
        std::uint64_t generation = 0;
        std::uint32_t overlay = 0;
        bool direct = false;
        return s_physicsInteraction->tryGetManualScopePresentationTarget(
                   generation, overlay, direct, weaponIdentity, instanceIdentity) && direct;
    }

    // Name of the loaded module containing an address, for the log; empty when none.
    std::string moduleNameAtAddress(const std::uintptr_t address)
    {
        HMODULE module = nullptr;
        if (!GetModuleHandleExW(
                GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                reinterpret_cast<LPCWSTR>(address),
                &module) ||
            !module) {
            return {};
        }
        char path[MAX_PATH]{};
        const auto length = GetModuleFileNameA(module, path, MAX_PATH);
        if (length == 0) {
            return {};
        }
        const std::string full(path, length);
        const auto slash = full.find_last_of("\\/");
        return slash == std::string::npos ? full : full.substr(slash + 1);
    }

    bool installNativeScopeDataHooks()
    {
        // Off until the geometry decision site is claimed: these hooks pass
        // the native behaviour through while disabled.
        native_scope_data::setEnabled(false);
        return native_scope_data::install(&isManualScopeEligibleForNative);
    }

    bool claimNativeScopeGeometryDecision()
    {
        REL::Relocation<std::uintptr_t> callSite{ REL::Offset(rock::offsets::kHookSite_NativeScopeGeometryDecision) };
        const auto callSiteAddress = callSite.address();
        const auto* callBytes = reinterpret_cast<const std::uint8_t*>(callSiteAddress);
        if (!callBytes || callBytes[0] != 0xE8) {
            logger::critical("ROCK: Native scope geometry hook validation failed at 0x{:X}: expected CALL rel32, found 0x{:02X}.", callSiteAddress, callBytes ? callBytes[0] : 0u);
            return false;
        }

        const auto relativeTarget = *reinterpret_cast<const std::int32_t*>(callBytes + 1);
        const auto decodedTarget = callSiteAddress + 5u + relativeTarget;
        const auto expectedTarget = REL::Offset(rock::offsets::kFunc_NativeScopeStateTransition).address();
        if (decodedTarget != expectedTarget) {
            const auto holder = moduleNameAtAddress(decodedTarget);
            logger::critical("ROCK: Native scope geometry hook validation failed at 0x{:X}: target 0x{:X}, expected 0x{:X}{}{}.",
                callSiteAddress, decodedTarget, expectedTarget, holder.empty() ? "" : "; the site is held by ", holder);
            return false;
        }

        REL::Relocation<std::uintptr_t> postDecisionTest{ REL::Offset(rock::offsets::kPatchSite_NativeScopePostDecisionTest) };
        const auto postDecisionTestAddress = postDecisionTest.address();
        const auto* postDecisionTestBytes = reinterpret_cast<const std::uint8_t*>(postDecisionTestAddress);
        constexpr std::array<std::uint8_t, 2> kExpectedNativeDecisionTest{ 0x84, 0xDB }; // TEST BL,BL
        if (!postDecisionTestBytes || postDecisionTestBytes[0] != kExpectedNativeDecisionTest[0] || postDecisionTestBytes[1] != kExpectedNativeDecisionTest[1]) {
            logger::critical("ROCK: Native scope fade-decision validation failed at 0x{:X}: expected TEST BL,BL.", postDecisionTestAddress);
            return false;
        }

        auto& trampoline = F4SE::GetTrampoline();
        const auto original = trampoline.write_call<5>(callSiteAddress, &onNativeScopeGeometryDecision);
        s_originalNativeScopeStateTransition = reinterpret_cast<NativeScopeStateTransitionFunc>(original);
        if (!s_originalNativeScopeStateTransition) {
            logger::critical("ROCK: Native scope geometry hook original target is null.");
            return false;
        }

        /*
         * The caller uses its pre-hook BL value for the adjacent approach-fade
         * branch. Our wrapper returns true in AL while ROCK owns the decision
         * so native cone-derived approach fade cannot survive as a second path.
         * Point the existing two-byte TEST at that explicit result.
         */
        constexpr std::array<std::uint8_t, 2> kRockDecisionTest{ 0x84, 0xC0 }; // TEST AL,AL
        REL::safe_write(postDecisionTestAddress, kRockDecisionTest.data(), kRockDecisionTest.size());

        logger::info("ROCK: Native scope geometry/fade hook installed at 0x{:X}, original 0x{:X}.", callSiteAddress, original);
        return true;
    }

    /*
     * Runs once the config is loaded and after every reload. Claims the
     * geometry decision site the first time immersive scopes are on; when the
     * claim fails, the setting is forced off for the session so every reader
     * of it agrees with the hooks that are actually in place.
     */
    void reconcileNativeScopeGeometryOwnership()
    {
        if (g_rockConfig.rockEnableImmersiveScopes &&
            s_nativeScopeGeometryHook == NativeScopeGeometryHook::Unclaimed) {
            s_nativeScopeGeometryHook = claimNativeScopeGeometryDecision() ?
                NativeScopeGeometryHook::Claimed :
                NativeScopeGeometryHook::Unavailable;
        }
        if (g_rockConfig.rockEnableImmersiveScopes &&
            s_nativeScopeGeometryHook != NativeScopeGeometryHook::Claimed) {
            g_rockConfig.rockEnableImmersiveScopes = false;
            if (!s_immersiveScopesForcedOffLogged) {
                s_immersiveScopesForcedOffLogged = true;
                logger::warn("ROCK: Immersive scopes forced off for this session: the native scope geometry decision site is not available.");
            }
        }
        if (!g_rockConfig.rockEnableImmersiveScopes &&
            s_nativeScopeGeometryHook == NativeScopeGeometryHook::Unclaimed &&
            !s_nativeScopeHookLeftUninstalledLogged) {
            s_nativeScopeHookLeftUninstalledLogged = true;
            logger::info("ROCK: Native scope geometry hook left uninstalled: immersive scopes are off, the site stays free for other mods until the setting turns on.");
        }
        native_scope_data::setEnabled(g_rockConfig.rockEnableImmersiveScopes);
    }

    // The provider tick ran or began this frame (AfterArmSolve, or FrameEnd without a skeleton).
    bool s_providerTickedThisFrame = false;
    // AfterArmSolve ran this frame: the later skeleton phases may read ROCK's frame state.
    bool s_skeletonTickedThisFrame = false;

    /*
     * One ROCK tick: the provider V1 phases around ROCK's own update. With a
     * skeleton it runs from AfterArmSolve, after ROCK's hand readers were
     * refreshed; when the skeleton phases did not run it runs from FrameEnd
     * so provider consumers keep receiving frames and leases keep expiring
     * across loading screens and skeleton rebuilds. BeforeRock, ROCK update,
     * AfterRock, Complete and this frame's provider publication share the one
     * game-frame timing snapshot taken at FrameBegin. The profiler frame
     * measures this tick; FrameBeginPreparation and FramePrelude are the
     * phases outside it.
     */
    void runFrameTick(const bool withSkeletonPhysics)
    {
        s_providerTickedThisFrame = true;
        const auto& frameTiming = game_timing::currentFrameTiming();

        performance_profiler::refreshSettings(
            g_rockConfig.rockPerformanceProfilerEnabled,
            g_rockConfig.rockPerformanceProfilerLogIntervalFrames,
            g_rockConfig.rockPerformanceProfilerWarmupFrames,
            g_rockConfig.rockPerformanceProfilerOverlayText);
        performance_profiler::FrameScope profilerFrame;

        rock::provider::refreshNativeAnimationAuthorityLeasesV1();
        rock::provider::dispatchAnimationPhaseCallbacksV1(
            rock::provider::RockProviderAnimationPhaseV1::BeforeRock,
            frameTiming);

        if (withSkeletonPhysics) {
            dynamic_collider_trace::beginFrame(g_rockConfig.rockDebugGrabFrameLogging, s_schedulerSequence);
            updatePhysicsInteractionFrame();
            vanilla_weapon_alignment_telemetry::capture(
                vanilla_weapon_alignment_telemetry::Phase::AfterWeaponSolve, s_schedulerSequence);
            native_scope_shot_diagnostics::beginFrame();
            if (!s_physicsInteraction) native_scope_shot_diagnostics::clearPresentation();
            // Input classification runs inside the update. Apply the button
            // scope level after it so an unflagged scope does not wait for a
            // native cone callback that Bethesda will never issue.
            driveManualScopeTransitionFallback();
        }
        native_scope_data::restoreNativeHousing();
        native_scope_data::reportDiagnostics();

        rock::provider::dispatchAnimationPhaseCallbacksV1(
            rock::provider::RockProviderAnimationPhaseV1::AfterRock,
            frameTiming);
        rock::provider::dispatchAnimationPhaseCallbacksV1(
            rock::provider::RockProviderAnimationPhaseV1::Complete,
            frameTiming);
        // Every claim of this frame is published; FRIK re-solves the claimed
        // hands when the phase returns, before the read-only render snapshot.
        // The weapon-node write block is settled here, before FRIK's weapon pass.
        if (withSkeletonPhysics && s_pluginLoaded && s_frikAvailable && s_physicsInteraction) {
            s_physicsInteraction->finalizeFrikWeaponOwnershipForFrame();
            s_physicsInteraction->traceScopeColliderState();
        }
        if (!withSkeletonPhysics) {
            return;
        }
        if (s_physicsInteraction) {
            s_physicsInteraction->traceHeldPresentationPhase("after-rock");
            s_physicsInteraction->publishGripZoneIndicatorRenderFrame(
                runtime_state::currentFrame().frameIndex);
        }
        scope_transition_telemetry::capture(scope_transition_telemetry::Phase::AfterRock, s_schedulerSequence);
        vanilla_weapon_alignment_telemetry::capture(
            vanilla_weapon_alignment_telemetry::Phase::AfterRock, s_schedulerSequence);
    }

    /*
     * FrameBegin: the top of FRIK's frame, every frame, skeleton or not, after
     * the scope enter/exit broadcast. Housekeeping runs here, with the
     * telemetry captures that precede FRIK's body work.
     */
    void onFrikFrameBegin()
    {
        performance_profiler::ScopedTimer frameBeginTimer(performance_profiler::Scope::FrameBeginPreparation);
        s_schedulerSequence = nextFrameSequence(s_schedulerSequence);
        s_providerTickedThisFrame = false;
        s_skeletonTickedThisFrame = false;
        // The frame's one timing sample, before the runtime snapshot consumes it.
        (void)runtime_state::beginFrameTiming(input_remap_runtime::isMenuInputActive());
        native_scope_data::beginGameFrame();
        if (s_pluginLoaded && s_frikAvailable) {
            vanilla_weapon_alignment_telemetry::capture(
                vanilla_weapon_alignment_telemetry::Phase::BeforeRockPreFrik, s_schedulerSequence);
            scope_transition_telemetry::capture(scope_transition_telemetry::Phase::BeforeFrik, s_schedulerSequence);
            vanilla_weapon_alignment_telemetry::capture(
                vanilla_weapon_alignment_telemetry::Phase::BeforeFrik, s_schedulerSequence);
        }
        prepareRuntimeFrame();
    }

    /*
     * FrameEnd: the end of FRIK's frame, every frame, after AfterWorldFinal
     * when the skeleton phases ran and right after FRIK's early return when
     * they did not (no player, loading, a skeleton released mid-frame). A frame
     * that AfterArmSolve did not tick is ticked here, without physics. FRIK's
     * frame runs under one catch, so a C++ exception escaping FRIK or another
     * client's callback drops this phase for that frame; FrameBegin resets the
     * tick flags and nothing here is held open across frames, so the tick is
     * skipped once and resumes on the next frame.
     */
    void onFrikFrameEnd()
    {
        if (!s_providerTickedThisFrame) {
            runFrameTick(false);
        }
    }

    /*
     * ROCK's frame runs inside FRIK's frame at the AfterArmSolve phase (FRIK
     * API v2.3): both arms are solved, so every hand claim ROCK publishes
     * here is re-solved by FRIK before finger poses, weapon position and the
     * world final run, and the solved arm is readable through the API.
     */
    void onFrikAfterArmSolve()
    {
        if (!s_frikSkeletonAnnounced) {
            return;
        }
        // Claimed before anything below can throw, so FrameEnd never ticks this frame again.
        s_providerTickedThisFrame = true;
        s_skeletonTickedThisFrame = true;
        performance_profiler::ScopedTimer preludeTimer(performance_profiler::Scope::FramePrelude);
        frik_hand_world_authority::beginRockFrame(s_schedulerSequence);
        // Close hand authority on every exit, including a caught exception.
        struct RockFrameEnd
        {
            ~RockFrameEnd()
            {
                frik_hand_world_authority::endRockFrame();
            }
        };
        const RockFrameEnd rockFrameEnd{};
        vanilla_weapon_alignment_telemetry::capture(
            vanilla_weapon_alignment_telemetry::Phase::AfterFrik, s_schedulerSequence);
        if (s_pluginLoaded && s_frikAvailable && s_physicsInteraction) {
            s_physicsInteraction->resolveFrameHands();
            s_physicsInteraction->traceHeldPresentationPhase("before-rock");
        }
        scope_transition_telemetry::capture(scope_transition_telemetry::Phase::AfterFrik, s_schedulerSequence);
        preludeTimer.stop();
        runFrameTick(true);
    }

    /*
     * AfterWeaponPosition: FRIK's weapon offsets, two-handed grip and scope
     * camera are applied (FRIK skips the weapon node while ROCK blocks it).
     * ROCK reports its two-handed grip after FRIK's own grip invalidation,
     * re-applies presentation scale after FRIK's pass, and its immersive scope
     * overlay captures FRIK's camera calibration and publishes the rigid
     * weapon-local scope frame from the final weapon.
     */
    void onFrikAfterWeaponPosition()
    {
        if (s_skeletonTickedThisFrame && s_pluginLoaded && s_frikAvailable && s_physicsInteraction) {
            s_physicsInteraction->syncFrikOffHandGripReport();
            s_physicsInteraction->normalizeWeaponPresentationScaleAfterFrikWeaponPass();
            s_physicsInteraction->synchronizeNativeScopePresentationAfterFrikUpdate();
        }
    }

    // AfterWorldFinal: every bone world transform is final. Latch what was rendered.
    void onFrikAfterWorldFinal()
    {
        if (s_skeletonTickedThisFrame && s_pluginLoaded && s_frikAvailable && s_physicsInteraction) {
            s_physicsInteraction->captureRenderedHands();
            s_physicsInteraction->traceHeldPresentationPhase("after-world-final");
            s_physicsInteraction->publishDebugRenderFrame();
            dynamic_collider_trace::capturePresentedHands(runtime_state::currentFrame().frameIndex);
            vanilla_weapon_alignment_telemetry::capture(
                vanilla_weapon_alignment_telemetry::Phase::AfterWorldFinal, s_schedulerSequence);
        }
    }

    bool frikPowerArmorState() noexcept
    {
        return frik_visual_authority::isSkeletonReadyHint() && frik_visual_authority::canReportPowerArmor() ?
            frik_visual_authority::isInPowerArmor() :
            f4vr::isInPowerArmorFromBiped();
    }

    void reportFramePhaseFault(const std::uint32_t phase) noexcept
    {
        static std::uint32_t s_reported = 0;
        if (s_reported++ < 16) {
            try {
                logger::error("ROCK: exception escaped FRIK frame phase {}; the rest of ROCK's frame was abandoned.", phase);
            } catch (...) {
            }
        }
    }

    /*
     * FRIK's registry does not guard its callbacks and the callback is
     * noexcept, so a C++ exception escaping a phase handler would terminate
     * the process inside FRIK's frame. Catch those, unwinding ROCK's scoped
     * timers and locks, log, and continue with the next frame. Hardware
     * faults are left alone so a ROCK crash stays reportable.
     */
    void invokeFramePhaseGuarded(void (*handler)(), const std::uint32_t phase) noexcept
    {
        try {
            handler();
        } catch (...) {
            reportFramePhaseFault(phase);
        }
    }

    void FRIK_CALL onFrikFramePhase(const std::uint32_t phase, void*) noexcept
    {
        using FramePhase = frik::api::FRIKApiV2::FramePhase;
        switch (static_cast<FramePhase>(phase)) {
        case FramePhase::FrameBegin:
            invokeFramePhaseGuarded(&onFrikFrameBegin, phase);
            break;
        case FramePhase::NativeGraphOutput:
            invokeFramePhaseGuarded(&authored_weapon_grip_capture::onNativeGraphOutput, phase);
            break;
        case FramePhase::AfterArmSolve:
            invokeFramePhaseGuarded(&onFrikAfterArmSolve, phase);
            break;
        case FramePhase::AfterWeaponPosition:
            invokeFramePhaseGuarded(&onFrikAfterWeaponPosition, phase);
            break;
        case FramePhase::AfterWorldFinal:
            invokeFramePhaseGuarded(&onFrikAfterWorldFinal, phase);
            break;
        case FramePhase::FrameEnd:
            invokeFramePhaseGuarded(&onFrikFrameEnd, phase);
            break;
        default:
            break;
        }
    }

    /*
     * Registered once after FRIK loaded; registrations survive skeleton
     * rebuilds. ROCK fans NativeGraphOutput out to its own provider
     * consumers (PAPER), so it is the only registrant for that phase.
     */
    bool registerFrikFrameCallbacks()
    {
        using FramePhase = frik::api::FRIKApiV2::FramePhase;
        constexpr int kPriority = 100;
        for (const FramePhase phase : { FramePhase::FrameBegin, FramePhase::NativeGraphOutput, FramePhase::AfterArmSolve, FramePhase::AfterWeaponPosition, FramePhase::AfterWorldFinal, FramePhase::FrameEnd }) {
            if (!frik_visual_authority::registerFrameCallback("ROCK", phase, &onFrikFramePhase, nullptr, kPriority)) {
                logger::critical("ROCK: FRIK frame callback registration failed for phase {}.", static_cast<unsigned>(phase));
                (void)frik_visual_authority::unregisterFrameCallback("ROCK");
                return false;
            }
        }
        logger::info("ROCK: FRIK frame callbacks registered (FrameBegin, NativeGraphOutput, AfterArmSolve, AfterWeaponPosition, AfterWorldFinal, FrameEnd).");
        return true;
    }

    void onFRIKMessage(F4SE::MessagingInterface::Message* msg)
    {
        if (!msg || !s_frikAvailable) {
            return;
        }

        using LE = frik::api::FRIKApiV2::LifecycleEvent;

        switch (static_cast<LE>(msg->type)) {
        case LE::kSkeletonReady:
            logger::info("ROCK: Received kSkeletonReady from FRIK.");
            if (msg->data && msg->dataLen == sizeof(frik::api::FRIKApiV2::SkeletonLifecycleData)) {
                const auto* lifecycle = static_cast<const frik::api::FRIKApiV2::SkeletonLifecycleData*>(msg->data);
                logger::info("ROCK: FRIK skeleton generation {} powerArmor={}.", lifecycle->generation, lifecycle->inPowerArmor ? "yes" : "no");
            }
            weapon_action_trace::initialize();
            vanilla_weapon_alignment_telemetry::initialize();
            scope_transition_telemetry::initialize();
            dynamic_collider_trace::initialize();
            bumpGeneration(s_skeletonGeneration);
            if (!authored_weapon_grip_capture::installHook()) {
                logger::error(
                    "ROCK: Authored equipped-weapon grip capture hook is unavailable for this runtime build.");
            }
            authored_weapon_grip_capture::setEnabled(true);
            if (s_physicsInteraction) {
                logger::warn("ROCK: PhysicsInteraction already exists on kSkeletonReady; deferring recreation to ROCK frame gate.");
            }
            requestDeferredPhysicsCreation();
            s_frikSkeletonAnnounced = true;
            break;

        case LE::kSkeletonDestroying:
            logger::info("ROCK: Received kSkeletonDestroying from FRIK.");
            s_frikSkeletonAnnounced = false;
            weapon_action_trace::invalidateContext();
            vanilla_weapon_alignment_telemetry::shutdown();
            scope_transition_telemetry::shutdown();
            native_scope_shot_diagnostics::shutdown();
            frik_hand_world_authority::resetForSkeletonRelease();
            bumpGeneration(s_skeletonGeneration);
            authored_weapon_grip_capture::resetTransientState();
            s_physicsCreationRequested.store(false, std::memory_order_release);
            s_physicsCreationReadyDeferralFrames.store(0, std::memory_order_release);
            resetPhysicsCreationGate();
            if (s_physicsInteraction) {
                s_physicsInteraction->noteSkeletonLifecycle(
                    s_skeletonGeneration.load(std::memory_order_acquire),
                    rock::provider::RockProviderLifecycleReason::SkeletonDestroying);
            }
            destroyPhysicsInteraction(rock::provider::RockProviderLifecycleReason::SkeletonDestroying);
            dynamic_collider_trace::shutdown();
            // FRIK runs no skeleton phase without a skeleton: drop the input state now.
            clearUnavailableRuntimeInputState();
            break;

        case LE::kScopeEnter:
        case LE::kScopeExit:
            // Broadcast at the start of FRIK's frame, before any phase. The
            // native first-person arm update rebuilds its tree on the scope
            // edge; hold the controller-hand relation for those frames.
            frik_hand_world_authority::noteScopeEdge();
            break;

        default:
            break;
        }
    }

    void onF4SEMessage(F4SE::MessagingInterface::Message* msg)
    {
        if (!msg) {
            return;
        }

        if (msg->type == F4SE::MessagingInterface::kGameLoaded) {
            logger::info("ROCK: GameLoaded -- initializing FRIKApi and loading config...");
            const auto providerGeneration = bumpGeneration(s_providerGeneration);
            if (s_physicsInteraction) {
                s_physicsInteraction->noteProviderLifecycle(
                    providerGeneration,
                    rock::provider::RockProviderLifecycleReason::GameLoaded);
            }

            /*
             * FRIK API v2 is append-only since v2.2: initialize() checks the
             * loaded version and table size against the version this build
             * compiled with, so success proves every entry this build calls
             * exists. The error codes are the header's own contract.
             */
            const int frikErr = frik::api::FRIKApiV2::initialize(frik::api::FRIK_API_V2_VERSION);
            if (frikErr != 0) {
                switch (frikErr) {
                case 1:
                    logger::critical("ROCK: FRIK API v2 initialization FAILED (error 1). FRIK.dll is not loaded. ROCK is now DISABLED.");
                    break;
                case 2:
                    logger::critical(
                        "ROCK: FRIK API v2 initialization FAILED (error 2). "
                        "FRIKAPI_V2_GetApi export was not found; the loaded FRIK.dll predates API v2. Deploy a FRIK.dll built from hFRIK main. ROCK is now DISABLED.");
                    break;
                case 3:
                    logger::critical("ROCK: FRIK API v2 initialization FAILED (error 3). FRIKAPI_V2_GetApi returned null. ROCK is now DISABLED.");
                    break;
                case 4:
                    logger::critical(
                        "ROCK: FRIK API v2 initialization FAILED (error 4). "
                        "Loaded FRIK API v2 is older than required v{}. Deploy the matching rebuilt FRIK.dll. ROCK is now DISABLED.",
                        frik::api::FRIK_API_V2_VERSION);
                    break;
                case 5:
                    logger::critical(
                        "ROCK: FRIK API v2 initialization FAILED (error 5). "
                        "Loaded FRIK API v2 table is smaller than API v{} requires. Deploy the matching rebuilt FRIK.dll. ROCK is now DISABLED.",
                        frik::api::FRIK_API_V2_VERSION);
                    break;
                default:
                    logger::critical("ROCK: FRIK API v2 initialization FAILED (error {}). ROCK is now DISABLED.", frikErr);
                    break;
                }
                s_frikAvailable = false;
                return;
            }

            logger::info("ROCK: FRIK v{} API v2 (v{}) initialized successfully.", frik::api::FRIKApiV2::inst->getModVersion(), frik::api::FRIKApiV2::inst->getVersion());
            if (frik::api::FRIKApiV2::inst->getConfigValue) {
                // FRIK smooths the first-person hands the weapon and ROCK's hand
                // seats follow; the session log must say so when seats are read.
                const auto* frikApi = frik::api::FRIKApiV2::inst;
                std::array<char, 16> dampen{};
                std::array<char, 16> translation{};
                std::array<char, 16> rotation{};
                (void)frikApi->getConfigValue("Fallout4VRBody", "DampenHands", dampen.data(), static_cast<int>(dampen.size()), "?");
                (void)frikApi->getConfigValue("Fallout4VRBody", "DampenHandsTranslation", translation.data(), static_cast<int>(translation.size()), "?");
                (void)frikApi->getConfigValue("Fallout4VRBody", "DampenHandsRotation", rotation.data(), static_cast<int>(rotation.size()), "?");
                logger::info("ROCK: FRIK hand dampening DampenHands={} translation={} rotation={}.", dampen.data(), translation.data(), rotation.data());
            }

            g_rockConfig.load();
            input_remap_runtime::configurePipboyInput();
            g_rockConfig.subscribeForConfigChanged("ROCKPipboyInput", [](const std::string&) {
                input_remap_runtime::configurePipboyInput();
            });
            if (!game_ini_overrides::install()) {
                logger::critical("ROCK: Required VR INI enforcement is unavailable; initialization stopped.");
                return;
            }
            rock::saved_grab_offset::preload();
            rock::authored_weapon_grip_cache::preload();
            rock::installHavokTimingFixHook();
            if (!rock::held_scene_presentation::install()) {
                logger::warn(
                    "ROCK: Held-body scene presentation hook is unavailable; native presentation remains unchanged.");
            }
            runtime_state::initialize();
            (void)native_scope_shot_diagnostics::install();
            reconcileNativeScopeGeometryOwnership();
            logger::info("ROCK: Config loaded.");
            rock::input_remap_runtime::installInputRemapHooks();
            rock::debug::Install();

            s_frikAvailable = true;

            s_messaging->RegisterListener(onFRIKMessage, frik::api::FRIKApiV2::FRIK_F4SE_MOD_NAME);
            logger::info("ROCK: Registered FRIK lifecycle event listener on '{}'.", frik::api::FRIKApiV2::FRIK_F4SE_MOD_NAME);

            if (!registerFrikFrameCallbacks()) {
                logger::critical("ROCK: FRIK frame callbacks are unavailable. ROCK is now DISABLED.");
                s_frikAvailable = false;
                return;
            }
            // FRIK debounces the game's transient power-armor state and flips
            // it together with the skeleton generation (API v2.2).
            f4vr::setPowerArmorStateProvider(&frikPowerArmorState);

            logger::info("ROCK: Initialization complete. Waiting for skeleton...");
        }

        if (msg->type == F4SE::MessagingInterface::kPostLoadGame || msg->type == F4SE::MessagingInterface::kNewGame) {
            logger::info("ROCK: New game session -- resetting PhysicsInteraction...");
            const auto providerGeneration = bumpGeneration(s_providerGeneration);
            s_physicsCreationRequested.store(false, std::memory_order_release);
            s_physicsCreationReadyDeferralFrames.store(0, std::memory_order_release);
            resetPhysicsCreationGate();
            runtime_state::resetTransientState();
            authored_weapon_grip_capture::resetTransientState();
            if (s_physicsInteraction) {
                s_physicsInteraction->noteProviderLifecycle(
                    providerGeneration,
                    rock::provider::RockProviderLifecycleReason::ProviderLost);
            }

            destroyPhysicsInteraction();

            if (s_frikAvailable) {
                g_rockConfig.reload();
                input_remap_runtime::configurePipboyInput();
                game_ini_overrides::update();
                logger::info("ROCK: Config reloaded for new session.");
            }
        }
    }
}

namespace rock
{
    const F4SE::MessagingInterface* getROCKMessaging() { return s_messaging; }
}

extern "C" DLLEXPORT bool F4SEAPI F4SEPlugin_Query(const F4SE::QueryInterface* a_f4se, F4SE::PluginInfo* a_info)
{
    logger::init("ROCK");

    logger::info("=== ROCK v{} === F4SE Plugin Query ===", Version::NAME);
    logger::info("ROCK: Realistic Overengineered Character Kinetics");

    a_info->infoVersion = F4SE::PluginInfo::kVersion;
    a_info->name = "ROCK";

    {
        std::string tmp(Version::NAME);
        std::erase(tmp, '.');
        a_info->version = std::stoi(tmp);
    }

    if (a_f4se->IsEditor()) {
        logger::critical("ROCK: Loaded in editor, marking as incompatible.");
        return false;
    }

    if (!REL::Module::IsVR()) {
        logger::critical("ROCK: Fallout 4 VR runtime required; refusing to load in non-VR runtime.");
        return false;
    }

    const auto requiredRuntime = F4SE::RUNTIME_LATEST_VR;

    if (a_f4se->RuntimeVersion() < requiredRuntime) {
        logger::critical("ROCK: Unsupported runtime version {} (need >= {}).", a_f4se->RuntimeVersion().string(), requiredRuntime.string());
        return false;
    }

    logger::info("ROCK: F4SE v{} query passed. Plugin compatible.", a_f4se->F4SEVersion().string());
    return true;
}

extern "C" DLLEXPORT bool F4SEAPI F4SEPlugin_Load(const F4SE::LoadInterface* a_f4se)
{
    logger::info("ROCK: F4SEPlugin_Load -- initializing...");

    logger::info("ROCK: Init CommonLibF4 F4SE...");
    F4SE::Init(a_f4se, false);

    logger::info("ROCK: Register F4SE messaging listener...");
    s_messaging = F4SE::GetMessagingInterface();
    if (!s_messaging) {
        logger::critical("ROCK: Failed to get F4SE MessagingInterface. Cannot continue.");
        return false;
    }
    s_messaging->RegisterListener(onF4SEMessage);

    logger::info("ROCK: Allocate trampoline (2048 bytes)...");
    F4SE::AllocTrampoline(2048);

    logger::info("ROCK: Install native shape-cast safety...");
    if (!rock::native_shape_cast_safety::install()) {
        logger::warn(
            "ROCK: Native shape-cast safety is unavailable; native collision queries remain unchanged.");
    }

    logger::info("ROCK: Install native ragdoll teardown safety...");
    if (!rock::native_ragdoll_safety::install()) {
        logger::warn(
            "ROCK: Native ragdoll teardown safety is unavailable; native ragdoll updates remain unchanged.");
    }

    logger::info("ROCK: Install native character-proxy teardown safety...");
    if (!rock::native_character_proxy_safety::install()) {
        logger::warn(
            "ROCK: Native character-proxy teardown safety is unavailable; native character-proxy world lookups remain unchanged.");
    }

    logger::info("ROCK: Install native collision-filter teardown safety...");
    if (!rock::native_collision_filter_safety::install()) {
        logger::warn(
            "ROCK: Native collision-filter teardown safety is unavailable; native collision-filter lookups remain unchanged.");
    }

    logger::info("ROCK: Install held weapon instant-transition capability...");
    if (!rock::held_weapon_instant_transition::install()) {
        logger::warn("ROCK: Held trigger/grip-zone equip disabled because the exact native transition contract is unavailable.");
    }

    logger::info("ROCK: Install scoped weapon transition animation acceleration...");
    if (!rock::weapon_transition_animation_acceleration::install()) {
        logger::warn("ROCK: Weapon draw/sheath animation acceleration unavailable; native timing remains unchanged.");
    }

    logger::info("ROCK: Install native scope data hooks...");
    if (!installNativeScopeDataHooks()) {
        return false;
    }

    s_pluginLoaded = true;
    logger::info("ROCK: F4SEPlugin_Load complete. Waiting for GameLoaded event...");
    return true;
}
