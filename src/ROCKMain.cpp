
#include <array>
#include <atomic>
#include <cstdint>

#include "api/FRIKApiV2.h"
#define ROCK_API_EXPORTS
#include "RockConfig.h"
#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapture.h"
#include "physics-interaction/core/MainLoopHookPolicy.h"
#include "physics-interaction/core/PhysicsCreationGatePolicy.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/debug/DebugBodyOverlay.h"
#include "physics-interaction/grab/FrikWeaponOffsetCache.h"
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
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/AuthoredWeaponGripCacheStore.h"
#include "physics-interaction/weapon/WeaponTransitionAnimationAcceleration.h"

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

    void ensurePhysicsInteractionForReadySkeleton(const runtime_state::RuntimeFrameSnapshot& runtime)
    {
        // ROCK creation is event-driven when FRIK first announces skeleton
        // readiness. The frame loop also recovers a missed lifecycle message
        // when the live skeleton becomes ready later.
        if (!s_physicsCreationRequested.load(std::memory_order_acquire) &&
            !s_physicsInteraction &&
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

    void onFrameUpdate()
    {
        performance_profiler::refreshSettings(
            g_rockConfig.rockPerformanceProfilerEnabled,
            g_rockConfig.rockPerformanceProfilerLogIntervalFrames,
            g_rockConfig.rockPerformanceProfilerWarmupFrames,
            g_rockConfig.rockPerformanceProfilerOverlayText);
        performance_profiler::FrameScope profilerFrame;

        if (!s_pluginLoaded) {
            clearUnavailableRuntimeInputState();
            return;
        }

        g_rockConfig.processPendingConfigReload();
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

        if (s_physicsInteraction) {
            /*
             * Native menu/equip animation work completed earlier in this game
             * frame. Reconcile the exact equipped instance first so authored
             * grip and collision never consume a stale hidden Weapon graph.
             */
            s_physicsInteraction->updateEquippedWeaponTransition();
            /*
             * hFRIK has already restored its generic one-gun Weapon local.
             * Reconstruct the Bethesda-authored primary grip before ROCK's
             * collision/probe/grip pass so every weapon-relative subsystem
             * sees the same corrected frame that will be rendered.
             */
            s_physicsInteraction->updateAuthoredPrimaryFiringGrip();
            s_physicsInteraction->update();
            publishPhysicsInteractionIfReady();
        }
    }

    using GameLoopFunc = void (*)(std::uint64_t rcx);
    GameLoopFunc s_originalGameLoopFunc = nullptr;

    /*
     * Outer main-loop hook state. FRIK hooks the same call site at
     * kGameLoaded and displaces ROCK's load-time hook; once FRIK owns the
     * site ROCK wraps it again so one pass runs before FRIK's frame (the
     * hand world claim rebase). s_frikChainGameLoopFunc is FRIK's hook (or
     * its CommonLib thunk), displaced by the outer hook.
     */
    GameLoopFunc s_frikChainGameLoopFunc = nullptr;
    bool s_outerFrameHookInstalled = false;
    std::uint64_t s_schedulerSequence = 0;
    std::uint64_t s_schedulerSequenceAtInstall = 0;
    main_loop_hook_policy::OuterHookAttemptState s_outerHookAttemptState{};

    using NativeScopeStateTransitionFunc = void (*)(RE::PlayerCharacter*, bool);
    NativeScopeStateTransitionFunc s_originalNativeScopeStateTransition = nullptr;
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

        using ConfigureNativeWorldScope = void (*)(void*, std::uint32_t);
        const auto configure = reinterpret_cast<ConfigureNativeWorldScope>(
            REL::Offset(rock::offsets::kFunc_NativeWorldScopeConfigure).address());
        configure(reinterpret_cast<void*>(worldScope), overlayIndex);
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
        if (!nativeForceDecision && s_pluginLoaded && s_frikAvailable) {
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
        const bool targetAvailable = s_pluginLoaded && s_frikAvailable &&
            s_physicsInteraction &&
            s_physicsInteraction->tryGetManualScopeDirectTransitionTarget(targetWeaponGenerationKey, targetOverlayIndex);
        const bool requested = targetAvailable &&
            input_remap_runtime::isManualScopeActivationRequested() &&
            player &&
            configureNativeWorldScopeForManualTarget(targetWeaponGenerationKey, targetOverlayIndex);

        if (requested) {
            /*
             * Bethesda only reaches the hooked cone call for weapons whose
             * OMOD carries its native scope flag. Explicit scope models from
             * imperfect ports (the OMEN Watchman) still have a valid ROCK
             * sight anchor and world_scope presentation, but otherwise never
             * receive a state transition. This path is gated to explicit
             * scope models without native metadata, and drives the same
             * verified native transition while the hold is active.
             */
            s_originalNativeScopeStateTransition(player, true);
            if (!s_manualScopeDirectTransitionActive) {
                ROCK_LOG_DEBUG(Input, "Manual scope direct transition engaged for explicit scope target");
            }
            s_manualScopeDirectTransitionActive = true;
            return;
        }

        if (s_manualScopeDirectTransitionActive && player) {
            s_originalNativeScopeStateTransition(player, false);
            ROCK_LOG_DEBUG(Input, "Manual scope direct transition released");
        }
        s_manualScopeDirectTransitionActive = false;
    }

    bool hookNativeScopeGeometryDecision()
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
            logger::critical("ROCK: Native scope geometry hook validation failed at 0x{:X}: target 0x{:X}, expected 0x{:X}.", callSiteAddress, decodedTarget, expectedTarget);
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

    void onGameFrameUpdateHook(std::uint64_t rcx);

    /*
     * Runs before FRIK's frame once the outer hook is installed: rebases every
     * active hand world claim by its controller driver's motion, then hands
     * the frame to FRIK, whose hook calls onGameFrameUpdateHook afterwards.
     */
    void onOuterFrameHook(const std::uint64_t rcx)
    {
        s_schedulerSequence = main_loop_hook_policy::nextSchedulerSequence(s_schedulerSequence);
        if (s_pluginLoaded && s_frikAvailable) {
            frik_hand_world_authority::runPreFrikPass(s_schedulerSequence);
        }
        if (s_frikChainGameLoopFunc) {
            s_frikChainGameLoopFunc(rcx);
        }
    }

    bool describeModuleOwningAddress(const std::uintptr_t address, std::array<char, MAX_PATH>& outPath, bool& outIsFrik)
    {
        outPath.fill('\0');
        outIsFrik = false;
        HMODULE module = nullptr;
        if (address == 0 ||
            GetModuleHandleExA(
                GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                reinterpret_cast<LPCSTR>(address),
                &module) == FALSE ||
            !module) {
            return false;
        }
        const auto length = GetModuleFileNameA(module, outPath.data(), static_cast<DWORD>(outPath.size()));
        if (length == 0 || length >= outPath.size()) {
            return false;
        }
        const char* fileName = outPath.data();
        for (const char* cursor = outPath.data(); *cursor; ++cursor) {
            if (*cursor == '\\' || *cursor == '/') {
                fileName = cursor + 1;
            }
        }
        outIsFrik = _stricmp(fileName, "FRIK.dll") == 0;
        return true;
    }

    /*
     * Bounded per-frame attempt to wrap the main-loop call site above FRIK.
     * Decode and decision live in MainLoopHookPolicy; this reads the live
     * bytes, resolves module ownership, and writes the call.
     */
    void ensureOuterFrameHook()
    {
        using namespace main_loop_hook_policy;

        if (s_outerFrameHookInstalled || s_outerHookAttemptState.refused || !s_pluginLoaded || !s_frikAvailable) {
            return;
        }

        REL::Relocation hookCallSite{ REL::Offset(rock::offsets::kHookSite_MainLoop) };
        const std::uintptr_t siteAddress = hookCallSite.address();

        OuterHookProbe probe{};
        std::uintptr_t immediateTarget = 0;
        std::uintptr_t terminalTarget = 0;
        std::array<std::uint8_t, kRelativeCallSize> callBytes{};
        probe.readable =
            native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(siteAddress), callBytes.data(), callBytes.size()) &&
            decodeRelativeCallTarget(callBytes.data(), siteAddress, immediateTarget);

        std::array<char, MAX_PATH> ownerPath{};
        if (probe.readable) {
            terminalTarget = immediateTarget;
            std::array<std::uint8_t, kCommonLibAbsoluteJumpThunkSize> thunkBytes{};
            std::uintptr_t thunkTarget = 0;
            if (native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(immediateTarget), thunkBytes.data(), thunkBytes.size()) &&
                decodeCommonLibAbsoluteJumpTarget(thunkBytes.data(), thunkTarget)) {
                terminalTarget = thunkTarget;
            }
            const auto outerAddress = reinterpret_cast<std::uintptr_t>(&onOuterFrameHook);
            const auto innerAddress = reinterpret_cast<std::uintptr_t>(&onGameFrameUpdateHook);
            probe.immediateIsOuterHook = immediateTarget == outerAddress;
            probe.terminalIsOuterHook = terminalTarget == outerAddress;
            probe.terminalIsInnerHook = terminalTarget == innerAddress;
            bool ownedByFrik = false;
            if (!probe.terminalIsOuterHook && !probe.terminalIsInnerHook &&
                describeModuleOwningAddress(terminalTarget, ownerPath, ownedByFrik)) {
                probe.terminalOwnedByFrik = ownedByFrik;
            }
        }

        switch (decideOuterHook(s_outerHookAttemptState, probe)) {
        case OuterHookDecision::AlreadyInstalled:
            s_outerFrameHookInstalled = true;
            s_schedulerSequenceAtInstall = s_schedulerSequence;
            logger::info("ROCK: Outer main loop hook already present at 0x{:X}.", siteAddress);
            break;
        case OuterHookDecision::Install: {
            auto& trampoline = F4SE::GetTrampoline();
            const auto displaced = trampoline.write_call<5>(siteAddress, &onOuterFrameHook);
            if (displaced == 0) {
                s_outerHookAttemptState.refused = true;
                frik_hand_world_authority::setSchedulerState(frik_hand_world_authority::SchedulerState::Refused);
                logger::critical(
                    "ROCK: Outer main loop hook write at 0x{:X} returned no displaced target. Hand world claims stay disabled this session.",
                    siteAddress);
                break;
            }
            s_frikChainGameLoopFunc = reinterpret_cast<GameLoopFunc>(displaced);
            s_outerFrameHookInstalled = true;
            s_schedulerSequenceAtInstall = s_schedulerSequence;
            logger::info(
                "ROCK: Outer main loop hook installed at 0x{:X} above FRIK ({}), chained target 0x{:X}{}; verifying on the next frame.",
                siteAddress,
                ownerPath.data(),
                displaced,
                displaced == immediateTarget ? "" : " (differs from the decoded target)");
            break;
        }
        case OuterHookDecision::RetryLater:
            break;
        case OuterHookDecision::GiveUp:
            frik_hand_world_authority::setSchedulerState(frik_hand_world_authority::SchedulerState::Refused);
            logger::critical(
                "ROCK: FRIK never hooked the main loop at 0x{:X} within {} frames. Hand world claims (grab, weapon grip, dynamic hand presentation) stay disabled this session.",
                siteAddress,
                kMaxOuterHookAttempts);
            break;
        case OuterHookDecision::Refuse:
        default:
            frik_hand_world_authority::setSchedulerState(frik_hand_world_authority::SchedulerState::Refused);
            if (!probe.readable) {
                logger::critical(
                    "ROCK: Main loop call site 0x{:X} is not a readable CALL rel32. Hand world claims stay disabled this session.",
                    siteAddress);
            } else {
                logger::critical(
                    "ROCK: Main loop call site 0x{:X} resolves to 0x{:X} owned by '{}', not FRIK. ROCK will not wrap an unknown hook; hand world claims stay disabled this session.",
                    siteAddress,
                    terminalTarget,
                    ownerPath[0] ? ownerPath.data() : "<unknown module>");
            }
            break;
        }
    }

    void verifyOuterFrameHook()
    {
        if (!s_outerFrameHookInstalled ||
            frik_hand_world_authority::schedulerState() != frik_hand_world_authority::SchedulerState::Unverified) {
            return;
        }
        if (s_schedulerSequence != s_schedulerSequenceAtInstall) {
            frik_hand_world_authority::setSchedulerState(frik_hand_world_authority::SchedulerState::Verified);
            logger::info("ROCK: Pre-FRIK pass verified (sequence {}); hand world claims enabled.", s_schedulerSequence);
        }
    }

    /*
     * FRIK installs its hook at kGameLoaded and calls this chained hook after
     * its skeleton/weapon pass; ROCK's outer hook wraps FRIK's in turn. The
     * displaced call is an unrelated PlayerCharacter flag update; native scope
     * activation ran earlier. Publish the generation-bound rigid camera/overlay
     * frame here for the later mono render, then let ROCK apply any final
     * weapon authority in onFrameUpdate.
     */
    void onGameFrameUpdateHook(const std::uint64_t rcx)
    {
        if (s_originalGameLoopFunc) {
            s_originalGameLoopFunc(rcx);
        }

        ensureOuterFrameHook();
        verifyOuterFrameHook();
        frik_hand_world_authority::beginRockFrame(s_schedulerSequence);
        if (s_pluginLoaded && s_frikAvailable && s_physicsInteraction) {
            s_physicsInteraction->resolveFrameHands();
        }

        /*
         * One game-frame timing identity is created here, before any phase
         * callback. BeforeRock, ROCK update, AfterRock, Complete, and this
         * frame's provider publication all share this immutable snapshot.
         */
        const auto& frameTiming = runtime_state::beginFrameTiming(
            input_remap_runtime::isMenuInputActive());

        rock::provider::refreshNativeAnimationAuthorityLeasesV1();
        rock::provider::dispatchAnimationPhaseCallbacksV1(
            rock::provider::RockProviderAnimationPhaseV1::BeforeRock,
            frameTiming);

        if (s_pluginLoaded && s_frikAvailable && s_physicsInteraction) {
            s_physicsInteraction->synchronizeNativeScopePresentationAfterFrikUpdate();
        }

        onFrameUpdate();
        // Input classification runs inside onFrameUpdate. Apply the button
        // scope level after it so an unflagged scope does not wait for a native
        // cone callback that Bethesda will never issue.
        driveManualScopeTransitionFallback();

        rock::provider::dispatchAnimationPhaseCallbacksV1(
            rock::provider::RockProviderAnimationPhaseV1::AfterRock,
            frameTiming);
        rock::provider::dispatchAnimationPhaseCallbacksV1(
            rock::provider::RockProviderAnimationPhaseV1::Complete,
            frameTiming);
        if (s_physicsInteraction) {
            s_physicsInteraction->publishGripZoneIndicatorRenderFrame(
                runtime_state::currentFrame().frameIndex);
        }
        // Last in the frame: every claim of this frame is published, and the
        // rendered chain is not read again before FRIK's next solve.
        if (s_pluginLoaded && s_frikAvailable && s_physicsInteraction) {
            s_physicsInteraction->presentClaimedHands();
        }
    }

    bool hookMainLoop()
    {
        REL::Relocation hookCallSite{ REL::Offset(rock::offsets::kHookSite_MainLoop) };

        logger::info("ROCK: Hooking main loop at (0x{:X})...", hookCallSite.address());

        auto& trampoline = F4SE::GetTrampoline();
        const auto original = trampoline.write_call<5>(hookCallSite.address(), &onGameFrameUpdateHook);
        s_originalGameLoopFunc = reinterpret_cast<GameLoopFunc>(original);

        if (!s_originalGameLoopFunc) {
            logger::critical("ROCK: Failed to hook main loop — original function pointer is null!");
            return false;
        }

        logger::info("ROCK: Main loop hook installed, original: (0x{:X}).", original);
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
            frik_visual_authority::resetPresentedHandNodeCache();
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
            break;

        case LE::kSkeletonDestroying:
            logger::info("ROCK: Received kSkeletonDestroying from FRIK.");
            frik_visual_authority::resetPresentedHandNodeCache();
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
             * FRIK API v2 is a fixed table with an exact struct-size check, so a
             * successful initialize() already proves every entry this build
             * calls exists. The error codes are the header's own contract.
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
                        "Loaded FRIK API v2 table size does not match this ROCK build. Deploy the matching rebuilt FRIK.dll. ROCK is now DISABLED.");
                    break;
                default:
                    logger::critical("ROCK: FRIK API v2 initialization FAILED (error {}). ROCK is now DISABLED.", frikErr);
                    break;
                }
                s_frikAvailable = false;
                return;
            }

            logger::info("ROCK: FRIK v{} API v2 (v{}) initialized successfully.", frik::api::FRIKApiV2::inst->getModVersion(), frik::api::FRIKApiV2::inst->getVersion());

            g_rockConfig.load();
            rock::frik_weapon_offset_cache::preload();
            rock::saved_grab_offset::preload();
            rock::authored_weapon_grip_cache::preload();
            rock::installHavokTimingFixHook();
            if (!rock::held_scene_presentation::install()) {
                logger::warn(
                    "ROCK: Held-body scene presentation hook is unavailable; native presentation remains unchanged.");
            }
            runtime_state::initialize();
            logger::info("ROCK: Config loaded.");
            rock::input_remap_runtime::installInputRemapHooks();
            rock::debug::Install();

            s_frikAvailable = true;

            s_messaging->RegisterListener(onFRIKMessage, frik::api::FRIKApiV2::FRIK_F4SE_MOD_NAME);
            logger::info("ROCK: Registered FRIK lifecycle event listener on '{}'.", frik::api::FRIKApiV2::FRIK_F4SE_MOD_NAME);

            // FRIK hooks the main loop in its own kGameLoaded handler; whichever
            // order the dispatch used, the per-frame attempt finishes the job.
            ensureOuterFrameHook();

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

    logger::info("ROCK: Install main loop hook...");
    if (!hookMainLoop()) {
        return false;
    }

    logger::info("ROCK: Install native scope geometry hook...");
    if (!hookNativeScopeGeometryDecision()) {
        return false;
    }

    s_pluginLoaded = true;
    logger::info("ROCK: F4SEPlugin_Load complete. Waiting for GameLoaded event...");
    return true;
}
