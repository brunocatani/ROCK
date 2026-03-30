// ROCKMain.cpp -- F4SE plugin entry point for the ROCK physics interaction DLL.
//
// WHY THIS APPROACH:
//
// ROCK is a standalone F4SE plugin DLL that provides physics-based hand interaction
// for Fallout 4 VR. It depends on FRIK for skeleton data (hand transforms, bone
// positions, player state) accessed through the FRIKApi consumer interface.
//
// We implement a DIRECT F4SE entry point rather than using the F4VR-CommonFramework's
// ModBase pattern for three reasons:
//
// 1. CONFIG INCOMPATIBILITY: ModBase requires a ConfigBase* (with embedded WIN32
//    resources, file-watch threads, version migration). ROCK uses a lean standalone
//    RockConfig that loads CSimpleIniA directly. Adapting it to ConfigBase would add
//    unnecessary complexity for zero benefit.
//
// 2. UNNECESSARY OVERHEAD: ModBase::onF4SEPluginLoad() initializes VRUI, VRControllers,
//    and other subsystems that ROCK does not use. A physics plugin should not pay for
//    UI infrastructure it never touches.
//
// 3. CLEAN SEPARATION: ROCK is a satellite DLL, not a standalone mod. It does not need
//    its own config UI, debug dump system, or file-watch reload. Direct F4SE registration
//    gives us full control over initialization order and lifecycle.
//
// FRAME UPDATE ORDERING:
//
// Both FRIK and ROCK hook the same main loop call site via F4SE trampolines. The hook
// chain follows last-writer-called-first semantics. FRIK installs its hook LATE (during
// GameLoaded), while ROCK installs during F4SEPlugin_Load (earlier). This means FRIK's
// hook executes first, updating the skeleton, and ROCK's hook executes second, reading
// the freshly-updated skeleton data. This is the correct order.
//
// FRIK DEPENDENCY:
//
// If FRIK is not loaded (FRIKApi::initialize() fails in GameLoaded), ROCK logs an error
// and disables itself. All frame update logic is gated behind FRIKApi availability and
// skeleton readiness checks.
//
// PHYSICSINTERACTION LIFECYCLE (Event-Driven, FRIKApi v4):
//
// Creation: FRIK dispatches kSkeletonReady → onFRIKMessage creates PhysicsInteraction.
//           This is synchronous — executes inside FRIK's initSkeleton() callstack.
// Update:   Each frame, if PhysicsInteraction exists, call update().
// Teardown: FRIK dispatches kSkeletonDestroying → onFRIKMessage destroys PhysicsInteraction.
//           This is synchronous — executes inside FRIK's releaseSkeleton() BEFORE skeleton
//           is deleted. Identical ordering to the monolith.
// Session:  On new game / load game, FRIK calls releaseSkeleton (fires kSkeletonDestroying)
//           then later initSkeleton (fires kSkeletonReady). No separate handling needed.

#include "api/FRIKApi.h"
#define ROCK_API_EXPORTS
#include "api/ROCKApi.h"
#include "RockConfig.h"
#include "physics-interaction/PhysicsInteraction.h"

namespace
{
    using namespace frik::rock;

    // =====================================================================
    // Module state
    // =====================================================================

    /// F4SE messaging interface -- acquired during F4SEPlugin_Load.
    /// Used by ROCK to dispatch physics events to external mods.
    const F4SE::MessagingInterface* s_messaging = nullptr;

    /// PhysicsInteraction instance -- created when FRIK's skeleton becomes ready,
    /// destroyed when the skeleton is released (PA transition, cell change, etc.).
    PhysicsInteraction* s_physicsInteraction = nullptr;

    /// Set to true after successful FRIKApi initialization in GameLoaded.
    /// If false, ROCK is disabled and the frame update is a no-op.
    bool s_frikAvailable = false;

    /// Set to true after F4SEPlugin_Load completes successfully.
    /// Guards against frame update callbacks arriving before initialization.
    bool s_pluginLoaded = false;

    // =====================================================================
    // PhysicsInteraction lifecycle
    // =====================================================================

    /// Create and initialize the PhysicsInteraction module.
    ///
    /// Called on the main thread when FRIK reports its skeleton is ready.
    /// The PhysicsInteraction constructor installs engine hooks (bump, CC radius,
    /// native grab, refresh manifold) -- these are one-time trampoline writes that
    /// persist for the DLL lifetime. The init() call creates per-session Havok state
    /// (collision bodies, layer registration, contact event subscription).
    ///
    /// After successful init, we:
    /// 1. Enable physics-thread hooks (s_hooksEnabled) so Havok callbacks can fire.
    /// 2. Register the instance with ROCKApi so external mods can query physics state.
    void createPhysicsInteraction()
    {
        logger::info("ROCK: Creating PhysicsInteraction (skeleton became ready)...");

        s_physicsInteraction = new PhysicsInteraction();
        s_physicsInteraction->init();

        // Enable physics-thread hooks AFTER everything is fully initialized.
        // This prevents hooks from accessing skeleton/world during init.
        PhysicsInteraction::s_hooksEnabled.store(true, std::memory_order_release);

        // Register with ROCKApi so external mods can query physics state.
        rock::api::setPhysicsInteractionInstance(s_physicsInteraction);

        logger::info("ROCK: PhysicsInteraction created and initialized.");
    }

    /// Tear down the PhysicsInteraction module.
    ///
    /// Called on the main thread when the skeleton becomes invalid (FRIK reports
    /// not ready after having been ready). This happens during:
    /// - Power Armor enter/exit
    /// - Cell transitions / loading screens
    /// - New game / save load
    ///
    /// Order of operations is critical:
    /// 1. Disable physics-thread hooks FIRST -- they run on Havok's physics thread
    ///    and access the skeleton. Without this, a hook could fire mid-teardown.
    /// 2. Clear the ROCKApi pointer so external mods see nullptr immediately.
    /// 3. Delete the PhysicsInteraction (calls shutdown internally).
    void destroyPhysicsInteraction()
    {
        if (!s_physicsInteraction) {
            return;
        }

        logger::info("ROCK: Destroying PhysicsInteraction (skeleton released)...");

        // Step 1: Disable physics-thread hooks FIRST.
        PhysicsInteraction::s_hooksEnabled.store(false, std::memory_order_release);

        // Step 2: Clear the ROCKApi pointer.
        rock::api::setPhysicsInteractionInstance(nullptr);

        // Step 3: Delete the instance (destructor calls shutdown).
        delete s_physicsInteraction;
        s_physicsInteraction = nullptr;

        logger::info("ROCK: PhysicsInteraction destroyed.");
    }

    // =====================================================================
    // Frame update
    // =====================================================================

    /// Called every game frame via the main loop hook.
    ///
    /// With event-driven lifecycle (v4), creation and destruction of PhysicsInteraction
    /// are handled by FRIK's lifecycle events (kSkeletonReady / kSkeletonDestroying).
    /// This function only needs to call update() if the instance exists.
    ///
    /// The rockEnabled config check is the only additional guard — if the user disables
    /// ROCK at runtime via ROCK.ini, we tear down here since no FRIK event covers that.
    void onFrameUpdate()
    {
        if (!s_pluginLoaded || !s_frikAvailable) {
            return;
        }

        // Runtime disable check: if user turned off ROCK in config, tear down.
        if (!g_rockConfig.rockEnabled) {
            if (s_physicsInteraction) {
                destroyPhysicsInteraction();
            }
            return;
        }

        // Normal frame: if PhysicsInteraction exists, update it.
        if (s_physicsInteraction) {
            s_physicsInteraction->update();
        }
    }

    // =====================================================================
    // Main loop hook (same mechanism as F4VR-CommonFramework's MainLoopHook)
    // =====================================================================

    /// We hook into the same game loop function call site that the framework uses.
    /// The trampoline chain preserves all previous hooks (including FRIK's).
    ///
    /// Hook address 0xd8405e is a CALL instruction inside the main game loop that
    /// executes every frame. By overwriting it with a trampoline jump to our function,
    /// we get called every frame and then call the original (which may be FRIK's hook
    /// or the vanilla game function).

    using GameLoopFunc = void (*)(std::uint64_t rcx);
    GameLoopFunc s_originalGameLoopFunc = nullptr;

    void onGameFrameUpdateHook(const std::uint64_t rcx)
    {
        // ROCK's frame update runs AFTER FRIK's because FRIK hooks this same site
        // later (setupMainGameLoopLate=true), making FRIK's hook execute first in
        // the trampoline chain. By the time we get here, FRIK has already updated
        // the skeleton for this frame.
        onFrameUpdate();

        // Call the original function (vanilla game code or another mod's hook).
        s_originalGameLoopFunc(rcx);
    }

    /// Install the main loop hook using the F4SE trampoline.
    /// Must be called after F4SE::AllocTrampoline().
    void hookMainLoop()
    {
        // Same call site as F4VR-CommonFramework's MainLoopHook.h
        REL::Relocation hookCallSite{ REL::Offset(0xd8405e) };

        logger::info("ROCK: Hooking main loop at (0x{:X})...", hookCallSite.address());

        auto& trampoline = F4SE::GetTrampoline();
        const auto original = trampoline.write_call<5>(hookCallSite.address(), &onGameFrameUpdateHook);
        s_originalGameLoopFunc = reinterpret_cast<GameLoopFunc>(original);

        if (!s_originalGameLoopFunc) {
            logger::critical("ROCK: Failed to hook main loop — original function pointer is null!");
        }

        logger::info("ROCK: Main loop hook installed, original: (0x{:X}).", original);
    }

    // =====================================================================
    // FRIK lifecycle event handler
    // =====================================================================

    /// Handles FRIK lifecycle events dispatched on the "F4VRBody" channel.
    ///
    /// These events are dispatched SYNCHRONOUSLY from FRIK's main thread during
    /// initSkeleton() and releaseSkeleton(). This means our handler executes
    /// inside FRIK's callstack — identical ordering to the monolith where FRIK
    /// directly called PhysicsInteraction methods.
    ///
    /// kSkeletonReady: Skeleton fully initialized, safe to create physics bodies.
    /// kSkeletonDestroying: Skeleton about to be torn down, must destroy physics NOW.
    /// kPowerArmorChanged: PA state changed (informational, skeleton already recreated).
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
                // Should not happen — kSkeletonDestroying should have fired first.
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
                // Future: adjust collision shapes, grab ranges, etc. for PA mode.
            }
            break;

        default:
            // Unknown message type from FRIK — ignore silently.
            break;
        }
    }

    // =====================================================================
    // F4SE messaging handler
    // =====================================================================

    /// Handles F4SE system messages (game loaded, new game, save loaded).
    ///
    /// kGameLoaded: Fires once after all F4SE plugins are loaded and the main menu
    ///   is shown. This is where we initialize the FRIKApi connection and load config.
    ///
    /// kPostLoadGame / kNewGame: Fires each time a save is loaded or a new game starts.
    ///   We tear down PhysicsInteraction so it can be re-created when the skeleton
    ///   becomes ready in the new session.
    void onF4SEMessage(F4SE::MessagingInterface::Message* msg)
    {
        if (!msg) {
            return;
        }

        if (msg->type == F4SE::MessagingInterface::kGameLoaded) {
            // ---------------------------------------------------------------
            // ONE-TIME: Initialize FRIK API connection and load ROCK config.
            // ---------------------------------------------------------------
            logger::info("ROCK: GameLoaded -- initializing FRIKApi and loading config...");

            // Initialize FRIKApi (gets function pointers from FRIK.dll).
            const int frikErr = frik::api::FRIKApi::initialize(4);
            if (frikErr != 0) {
                logger::critical("ROCK: FRIKApi initialization FAILED (error {}). "
                    "FRIK.dll must be loaded before ROCK. ROCK is now DISABLED.", frikErr);
                s_frikAvailable = false;
                return;
            }

            logger::info("ROCK: FRIKApi v{} (API v{}) initialized successfully.",
                frik::api::FRIKApi::inst->getModVersion(),
                frik::api::FRIKApi::inst->getVersion());

            // Load ROCK configuration from ROCK.ini.
            g_rockConfig.load();
            logger::info("ROCK: Config loaded (rockEnabled={}).", g_rockConfig.rockEnabled);

            s_frikAvailable = true;

            // Register listener on FRIK's messaging channel for lifecycle events.
            s_messaging->RegisterListener(onFRIKMessage, frik::api::FRIKApi::FRIK_F4SE_MOD_NAME);
            logger::info("ROCK: Registered FRIK lifecycle event listener on '{}'.", frik::api::FRIKApi::FRIK_F4SE_MOD_NAME);

            logger::info("ROCK: Initialization complete. Waiting for skeleton...");
        }

        if (msg->type == F4SE::MessagingInterface::kPostLoadGame ||
            msg->type == F4SE::MessagingInterface::kNewGame) {
            // ---------------------------------------------------------------
            // PER-SESSION: Reset PhysicsInteraction for clean slate.
            // ---------------------------------------------------------------
            logger::info("ROCK: New game session -- resetting PhysicsInteraction...");

            destroyPhysicsInteraction();

            // Reload config in case the user edited ROCK.ini between sessions.
            if (s_frikAvailable) {
                g_rockConfig.reload();
                logger::info("ROCK: Config reloaded for new session.");
            }
        }
    }
}

// =====================================================================
// Public accessors for ROCK internals
// =====================================================================

namespace rock
{
    const F4SE::MessagingInterface* getROCKMessaging()
    {
        return s_messaging;
    }
}

// =====================================================================
// F4SE exported entry points
// =====================================================================

/// F4SEPlugin_Query: Called by F4SE to check plugin compatibility.
///
/// We initialize logging here (earliest safe point), report plugin identity,
/// and verify runtime version compatibility. This runs before any other
/// plugin code executes.
extern "C" DLLEXPORT bool F4SEAPI F4SEPlugin_Query(
    const F4SE::QueryInterface* a_f4se,
    F4SE::PluginInfo* a_info)
{
    // Initialize ROCK's own log file.
    logger::init("ROCK");

    logger::info("=== ROCK v{} === F4SE Plugin Query ===", Version::NAME);
    logger::info("ROCK: Realistic Overengineered Character Kinetics");

    // Fill out plugin info for F4SE.
    a_info->infoVersion = F4SE::PluginInfo::kVersion;
    a_info->name = "ROCK";

    // Convert version string "X.Y.Z" to integer XYZ for F4SE's version field.
    {
        std::string tmp(Version::NAME);
        std::erase(tmp, '.');
        a_info->version = std::stoi(tmp);
    }

    // Compatibility checks.
    if (a_f4se->IsEditor()) {
        logger::critical("ROCK: Loaded in editor, marking as incompatible.");
        return false;
    }

    const auto requiredRuntime = REL::Module::IsF4()
        ? F4SE::RUNTIME_LATEST
        : F4SE::RUNTIME_LATEST_VR;

    if (a_f4se->RuntimeVersion() < requiredRuntime) {
        logger::critical("ROCK: Unsupported runtime version {} (need >= {}).",
            a_f4se->RuntimeVersion().string(), requiredRuntime.string());
        return false;
    }

    logger::info("ROCK: F4SE v{} query passed. Plugin compatible.",
        a_f4se->F4SEVersion().string());
    return true;
}

/// F4SEPlugin_Load: Called by F4SE to initialize the plugin.
///
/// This is where we:
/// 1. Initialize CommonLibF4's internal state.
/// 2. Register for F4SE messaging (GameLoaded, NewGame, PostLoadGame).
/// 3. Allocate trampoline space for hooks.
/// 4. Install the main loop hook for per-frame updates.
///
/// PhysicsInteraction is NOT created here -- it is deferred to the frame update
/// loop after FRIK confirms its skeleton is ready. This avoids accessing game
/// state that may not be initialized yet.
extern "C" DLLEXPORT bool F4SEAPI F4SEPlugin_Load(
    const F4SE::LoadInterface* a_f4se)
{
    logger::info("ROCK: F4SEPlugin_Load -- initializing...");

    // Step 1: Initialize CommonLibF4 internals (address library, etc.).
    logger::info("ROCK: Init CommonLibF4 F4SE...");
    F4SE::Init(a_f4se, false);

    // Step 2: Register for F4SE system messages.
    logger::info("ROCK: Register F4SE messaging listener...");
    s_messaging = F4SE::GetMessagingInterface();
    if (!s_messaging) {
        logger::critical("ROCK: Failed to get F4SE MessagingInterface. Cannot continue.");
        return false;
    }
    s_messaging->RegisterListener(onF4SEMessage);

    // Step 3: Allocate trampoline space for ROCK's hooks.
    // PhysicsInteraction installs 4 hooks (bump, CC radius, native grab, refresh manifold)
    // plus the main loop hook. 256 bytes is sufficient for 5 trampolines.
    logger::info("ROCK: Allocate trampoline (256 bytes)...");
    F4SE::AllocTrampoline(256);

    // Step 4: Install the main loop hook for per-frame updates.
    // This MUST happen during Load (not GameLoaded) so that FRIK's late hook
    // overwrites ours in the trampoline chain, ensuring FRIK runs first.
    logger::info("ROCK: Install main loop hook...");
    hookMainLoop();

    s_pluginLoaded = true;
    logger::info("ROCK: F4SEPlugin_Load complete. Waiting for GameLoaded event...");
    return true;
}
