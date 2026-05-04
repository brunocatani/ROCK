

#include "api/FRIKApi.h"
#define ROCK_API_EXPORTS
#include "RockConfig.h"
#include "api/ROCKApi.h"
#include "api/ROCKProviderApi.h"
#include "physics-interaction/debug/DebugBodyOverlay.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/core/PhysicsInteraction.h"

namespace
{
    using namespace frik::rock;

    const F4SE::MessagingInterface* s_messaging = nullptr;

    PhysicsInteraction* s_physicsInteraction = nullptr;
    bool s_physicsPublished = false;

    bool s_frikAvailable = false;

    bool s_pluginLoaded = false;

    void publishPhysicsInteractionIfReady()
    {
        if (!s_physicsInteraction || !s_physicsInteraction->isInitialized() || s_physicsPublished) {
            return;
        }

        PhysicsInteraction::s_hooksEnabled.store(true, std::memory_order_release);
        rock::api::setPhysicsInteractionInstance(s_physicsInteraction);
        rock::provider::setPhysicsInteractionInstance(s_physicsInteraction);
        s_physicsPublished = true;
        logger::info("ROCK: PhysicsInteraction initialized and published.");
    }

    void createPhysicsInteraction()
    {
        logger::info("ROCK: Creating PhysicsInteraction (skeleton became ready)...");

        s_physicsInteraction = new PhysicsInteraction();
        s_physicsInteraction->init();

        publishPhysicsInteractionIfReady();
        if (!s_physicsPublished) {
            logger::warn("ROCK: PhysicsInteraction init deferred; hooks/API remain disabled until lazy init succeeds.");
        }
    }

    void destroyPhysicsInteraction()
    {
        if (!s_physicsInteraction) {
            return;
        }

        logger::info("ROCK: Destroying PhysicsInteraction (skeleton released)...");

        PhysicsInteraction::s_hooksEnabled.store(false, std::memory_order_release);

        rock::api::setPhysicsInteractionInstance(nullptr);
        rock::provider::setPhysicsInteractionInstance(nullptr);
        rock::provider::clearExternalBodiesForProviderLoss();
        s_physicsPublished = false;

        delete s_physicsInteraction;
        s_physicsInteraction = nullptr;

        logger::info("ROCK: PhysicsInteraction destroyed.");
    }

    void onFrameUpdate()
    {
        if (!s_pluginLoaded || !s_frikAvailable) {
            input_remap_runtime::setGameplayInputAllowed(false);
            input_remap_runtime::setWeaponDrawn(false);
            return;
        }

        g_rockConfig.processPendingConfigReload();
        input_remap_runtime::installInputRemapHooks();

        auto* frikApi = frik::api::FRIKApi::inst;
        const bool weaponDrawn = frikApi && frikApi->isWeaponDrawn();
        const bool menuInputActive = input_remap_runtime::isMenuInputActive();
        const bool gameplayInputAllowed = g_rockConfig.rockEnabled && frikApi && frikApi->isSkeletonReady() && !frikApi->isAnyMenuOpen() && !frikApi->isConfigOpen() &&
                                          !frikApi->isWristPipboyOpen() && !menuInputActive;
        input_remap_runtime::setWeaponDrawn(weaponDrawn);
        input_remap_runtime::setGameplayInputAllowed(gameplayInputAllowed);
        input_remap_runtime::processPendingWeaponToggleRequests();

        if (!g_rockConfig.rockEnabled) {
            if (s_physicsInteraction) {
                destroyPhysicsInteraction();
            }
            return;
        }

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
        REL::Relocation hookCallSite{ REL::Offset(frik::rock::offsets::kHookSite_MainLoop) };

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
            if (!g_rockConfig.rockEnabled) {
                logger::info("ROCK: Physics disabled in config, skipping creation.");
                break;
            }
            if (s_physicsInteraction) {
                logger::warn("ROCK: PhysicsInteraction already exists on kSkeletonReady! Destroying first.");
                destroyPhysicsInteraction();
            }
            createPhysicsInteraction();
            break;

        case LE::kSkeletonDestroying:
            logger::info("ROCK: Received kSkeletonDestroying from FRIK.");
            destroyPhysicsInteraction();
            break;

        case LE::kPowerArmorChanged:
            if (msg->data && msg->dataLen >= sizeof(bool)) {
                const bool isInPA = *static_cast<const bool*>(msg->data);
                logger::info("ROCK: Power Armor state changed: {}", isInPA ? "IN PA" : "NOT IN PA");
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

            g_rockConfig.load();
            logger::info("ROCK: Config loaded (rockEnabled={}).", g_rockConfig.rockEnabled);
            frik::rock::input_remap_runtime::installInputRemapHooks();
            frik::rock::debug::Install();

            s_frikAvailable = true;

            s_messaging->RegisterListener(onFRIKMessage, frik::api::FRIKApi::FRIK_F4SE_MOD_NAME);
            logger::info("ROCK: Registered FRIK lifecycle event listener on '{}'.", frik::api::FRIKApi::FRIK_F4SE_MOD_NAME);

            logger::info("ROCK: Initialization complete. Waiting for skeleton...");
        }

        if (msg->type == F4SE::MessagingInterface::kPostLoadGame || msg->type == F4SE::MessagingInterface::kNewGame) {
            logger::info("ROCK: New game session -- resetting PhysicsInteraction...");

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

    logger::info("ROCK: Allocate trampoline (1024 bytes)...");
    F4SE::AllocTrampoline(1024);

    logger::info("ROCK: Install main loop hook...");
    if (!hookMainLoop()) {
        return false;
    }

    s_pluginLoaded = true;
    logger::info("ROCK: F4SEPlugin_Load complete. Waiting for GameLoaded event...");
    return true;
}
