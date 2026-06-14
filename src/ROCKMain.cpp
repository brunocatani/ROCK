
#include <atomic>
#include <cstdint>

#include "api/FRIKApi.h"
#define ROCK_API_EXPORTS
#include "RockConfig.h"
#include "api/ROCKProviderApi.h"
#include "physics-interaction/debug/DebugBodyOverlay.h"
#include "physics-interaction/core/PhysicsCreationGatePolicy.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/input/DebugControllerRuntime.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/SeeThroughScopesCompatibility.h"

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
        /*
         * ROCK creation is event-driven when FRIK first announces skeleton
         * readiness, but config hot reload can disable the module during that
         * event and re-enable it later. The frame loop is the only place that
         * sees the current config and live FRIK readiness together, so it owns
         * this narrow recovery path instead of forcing users to reload a save
         * to receive a second skeleton-ready message.
         */
        if (!s_physicsCreationRequested.load(std::memory_order_acquire) &&
            !s_physicsInteraction &&
            g_rockConfig.rockEnabled &&
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
            .rockEnabled = g_rockConfig.rockEnabled,
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

        logger::info("ROCK: PhysicsInteraction destroyed.");
    }

    void onFrameUpdate()
    {
        performance_profiler::refreshSettings(
            g_rockConfig.rockPerformanceProfilerEnabled,
            g_rockConfig.rockPerformanceProfilerLogIntervalFrames,
            g_rockConfig.rockPerformanceProfilerWarmupFrames,
            g_rockConfig.rockPerformanceProfilerOverlayText);
        performance_profiler::FrameScope profilerFrame;

        if (!s_pluginLoaded || !s_frikAvailable) {
            input_remap_runtime::setGameplayInputAllowed(false);
            input_remap_runtime::setWeaponDrawn(false);
            input_remap_runtime::setRightHandHeldWeapon(false);
            return;
        }

        g_rockConfig.processPendingConfigReload();
        see_through_scopes::updateFrame();
        input_remap_runtime::installInputRemapHooks();

        const bool menuInputActive = input_remap_runtime::isMenuInputActive();
        runtime_state::updateFrame(runtime_state::RuntimeFrameInput{
            .menuInputBlocking = menuInputActive,
            .visualAuthorityAvailable = frik_visual_authority::isAvailable(),
            .visualSkeletonReadyHint = frik_visual_authority::isSkeletonReadyHint(),
            .compatibilityConfigBlocking = frik_visual_authority::isCompatibilityConfigBlocking(),
        });
        const auto& runtime = runtime_state::currentFrame();
        const bool gameplayInputAllowed =
            g_rockConfig.rockEnabled &&
            runtime.localSkeletonReady &&
            !runtime.localMenuBlocking &&
            !runtime.compatibilityConfigBlocking;
        input_remap_runtime::setWeaponDrawn(runtime.weaponDrawn);
        input_remap_runtime::setGameplayInputAllowed(gameplayInputAllowed);
        input_remap_runtime::processPendingWeaponToggleRequests();
        debug_controller_runtime::update(gameplayInputAllowed, runtime.deltaSeconds);

        if (!g_rockConfig.rockEnabled) {
            s_physicsCreationRequested.store(false, std::memory_order_release);
            s_physicsCreationReadyDeferralFrames.store(0, std::memory_order_release);
            resetPhysicsCreationGate();
            if (s_physicsInteraction) {
                destroyPhysicsInteraction();
            }
            return;
        }

        ensurePhysicsInteractionForReadySkeleton(runtime);

        if (s_physicsInteraction) {
            s_physicsInteraction->update();
            publishPhysicsInteractionIfReady();
        }
    }

    using GameLoopFunc = void (*)(std::uint64_t rcx);
    GameLoopFunc s_originalGameLoopFunc = nullptr;

    // ROCK applies weapon visual/collision authority after the chained frame update
    // so FRIK finishes its skeleton and weapon pass before ROCK writes final state.
    void onGameFrameUpdateHook(const std::uint64_t rcx)
    {
        if (s_originalGameLoopFunc) {
            s_originalGameLoopFunc(rcx);
        }

        onFrameUpdate();
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

        using LE = frik::api::FRIKApi::LifecycleEvent;

        switch (static_cast<LE>(msg->type)) {
        case LE::kSkeletonReady:
            logger::info("ROCK: Received kSkeletonReady from FRIK.");
            bumpGeneration(s_skeletonGeneration);
            if (!g_rockConfig.rockEnabled) {
                logger::info("ROCK: Physics disabled in config, skipping creation.");
                break;
            }
            if (s_physicsInteraction) {
                logger::warn("ROCK: PhysicsInteraction already exists on kSkeletonReady; deferring recreation to ROCK frame gate.");
            }
            requestDeferredPhysicsCreation();
            break;

        case LE::kSkeletonDestroying:
            logger::info("ROCK: Received kSkeletonDestroying from FRIK.");
            bumpGeneration(s_skeletonGeneration);
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

        case LE::kPowerArmorChanged:
            bumpGeneration(s_skeletonGeneration);
            if (msg->data && msg->dataLen >= sizeof(bool)) {
                const bool isInPA = *static_cast<const bool*>(msg->data);
                logger::info("ROCK: Power Armor state changed: {}", isInPA ? "IN PA" : "NOT IN PA");
            }
            if (s_physicsInteraction) {
                s_physicsInteraction->noteSkeletonLifecycle(
                    s_skeletonGeneration.load(std::memory_order_acquire),
                    rock::provider::RockProviderLifecycleReason::PowerArmorChanged);
            }
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

            const int frikErr = frik::api::FRIKApi::initialize(frik::api::FRIK_API_VERSION);
            if (frikErr != 0) {
                switch (frikErr) {
                case 1:
                    logger::critical("ROCK: FRIKApi initialization FAILED (error 1). FRIK.dll is not loaded. ROCK is now DISABLED.");
                    break;
                case 2:
                    logger::critical("ROCK: FRIKApi initialization FAILED (error 2). FRIKAPI_GetApi export was not found. ROCK is now DISABLED.");
                    break;
                case 3:
                    logger::critical("ROCK: FRIKApi initialization FAILED (error 3). FRIKAPI_GetApi returned null. ROCK is now DISABLED.");
                    break;
                case 4:
                    logger::critical(
                        "ROCK: FRIKApi initialization FAILED (error 4). "
                        "Loaded FRIK API is older than required API v{}. Deploy the matching rebuilt FRIK.dll. ROCK is now DISABLED.",
                        frik::api::FRIK_API_VERSION);
                    break;
                default:
                    logger::critical("ROCK: FRIKApi initialization FAILED (error {}). ROCK is now DISABLED.", frikErr);
                    break;
                }
                s_frikAvailable = false;
                return;
            }

            logger::info("ROCK: FRIKApi v{} (API v{}) initialized successfully.", frik::api::FRIKApi::inst->getModVersion(), frik::api::FRIKApi::inst->getVersion());

            const auto* frikApi = frik::api::FRIKApi::inst;
            const bool hasCanonicalHandPoseContract =
                frikApi &&
                frikApi->setHandPoseCustomWithPriority != nullptr &&
                frikApi->getHandPoseLocalTransformsForPose != nullptr &&
                frikApi->setHandPoseCustomLocalTransformsWithPriority != nullptr &&
                frikApi->applyExternalHandWorldTransform != nullptr &&
                frikApi->clearExternalHandWorldTransform != nullptr;
            if (!hasCanonicalHandPoseContract) {
                logger::critical(
                    "ROCK: FRIKApi v5 contract mismatch. Loaded FRIK.dll does not expose the canonical 22-float hand-pose contract required by this ROCK build. Deploy the matching rebuilt FRIK.dll. ROCK is now DISABLED.");
                s_frikAvailable = false;
                return;
            }

            g_rockConfig.load();
            rock::installHavokTimingFixHook();
            runtime_state::initialize();
            see_through_scopes::refreshRuntimeState();
            logger::info("ROCK: Config loaded (rockEnabled={}).", g_rockConfig.rockEnabled);
            rock::input_remap_runtime::installInputRemapHooks();
            rock::debug::Install();

            s_frikAvailable = true;

            s_messaging->RegisterListener(onFRIKMessage, frik::api::FRIKApi::FRIK_F4SE_MOD_NAME);
            logger::info("ROCK: Registered FRIK lifecycle event listener on '{}'.", frik::api::FRIKApi::FRIK_F4SE_MOD_NAME);

            logger::info("ROCK: Initialization complete. Waiting for skeleton...");
        }

        if (msg->type == F4SE::MessagingInterface::kPostLoadGame || msg->type == F4SE::MessagingInterface::kNewGame) {
            logger::info("ROCK: New game session -- resetting PhysicsInteraction...");
            const auto providerGeneration = bumpGeneration(s_providerGeneration);
            s_physicsCreationRequested.store(false, std::memory_order_release);
            s_physicsCreationReadyDeferralFrames.store(0, std::memory_order_release);
            resetPhysicsCreationGate();
            runtime_state::resetTransientState();
            if (s_physicsInteraction) {
                s_physicsInteraction->noteProviderLifecycle(
                    providerGeneration,
                    rock::provider::RockProviderLifecycleReason::ProviderLost);
            }

            destroyPhysicsInteraction();

            if (s_frikAvailable) {
                see_through_scopes::resetRuntimeState();
                g_rockConfig.reload();
                see_through_scopes::refreshRuntimeState();
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

    logger::info("ROCK: Install main loop hook...");
    if (!hookMainLoop()) {
        return false;
    }

    logger::info("ROCK: Install See-Through Scopes late culling hook...");
    if (!rock::see_through_scopes::installLateCullingHook()) {
        return false;
    }

    s_pluginLoaded = true;
    logger::info("ROCK: F4SEPlugin_Load complete. Waiting for GameLoaded event...");
    return true;
}
