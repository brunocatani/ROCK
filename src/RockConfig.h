#pragma once

// RockConfig.h — Standalone INI configuration for the ROCK DLL.
//
// WHY: ROCK is extracted from FRIK as its own DLL. All physics-interaction config values
// previously lived in FRIK's monolithic Config class under g_config.rock* fields.
// Now that ROCK is a separate binary, it needs its own config system. Rather than
// pulling in FRIK's heavy ConfigBase machinery (embedded resources, file-watch, version
// migration), we use a minimal self-contained load()/reload() approach: CSimpleIniA reads
// ROCK.ini directly from the same FRIK_Config folder, falling back to compiled-in
// defaults when the key is absent. This keeps ROCK independent of FRIK at link time.
//
// Field names are kept IDENTICAL to the monolith Config.h rock* fields so that the
// B6 rewire task is a mechanical g_config. -> g_rockConfig. substitution with no logic changes.
//
// INI section: [PhysicsInteraction] — matches the section name already used in FRIK.ini,
// so users who already have these values in their FRIK.ini can copy them to ROCK.ini
// without changing key names.

#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include <thomasmonkman-filewatch/FileWatch.hpp>

namespace frik::rock
{
    class RockConfig
    {
    public:
        // Load (or reload) values from ROCK.ini on disk.
        // On first call this also resolves the INI path via SHGetFolderPath.
        // Missing keys silently fall back to the compiled-in defaults below.
        // Starts the file-watcher for live hot-reload after the first read.
        void load();

        // Alias: re-reads the same INI file path already resolved by load().
        // Safe to call at any time for live-reload scenarios.
        void reload();

        // Tear down the file-watcher. Called during shutdown or before destruction.
        void stopFileWatch();

        // Subscribe to be notified when config changes are detected and reloaded.
        // The callback receives the subscriber key. Use the key to unsubscribe later.
        void subscribeForConfigChanged(const std::string& key, std::function<void(const std::string&)> callback);
        void unsubscribeFromConfigChanged(const std::string& key);

        // Set this flag BEFORE programmatically writing to ROCK.ini.
        // The next file-watch callback will see it, clear it, and skip the reload.
        void suppressNextFileWatchReload() { _ignoreNextIniFileChange.store(true); }

        // -----------------------------------------------------------------------
        // ROCK Physics Interaction
        // -----------------------------------------------------------------------

        bool rockEnabled = true;

        // --- Hand Collision Body ---
        float rockHandCollisionHalfExtentX = 0.06f;    // Palm half-width (Havok units, ~12cm)
        float rockHandCollisionHalfExtentY = 0.02f;    // Palm half-depth/finger direction (~2cm)
        float rockHandCollisionHalfExtentZ = 0.015f;   // Palm thickness/radius (~1.5cm)
        // Collision body offset from wand origin, in Havok units (meters, 1 HU = 70 game units).
        // Applied in computeHandCollisionTransform() via kHavokToGame * offset along wand axes.
        float rockHandCollisionOffsetX = 0.0f;         // Wand-local X offset. Auto-mirrors for left hand.
        float rockHandCollisionOffsetY = 0.086f;       // Wand-local Y offset — along barrel toward fingertips (HIGGS: 0.086)
        float rockHandCollisionOffsetZ = -0.005f;      // Wand-local Z offset — toward palm face (HIGGS: -0.005)
        float rockHandCollisionBoxRadius = 0.0f;       // Convex radius for rounded box edges (HIGGS: 0)

        // --- Palm Offset (game units, in hand-local space) ---
        // Defines where the palm CENTER is relative to the hand transform origin.
        // This is NOT the collision body offset — it's used in the per-frame pivotB
        // computation to drive the object's contact point TO the palm.
        // FO4VR wand axes: Y=fingertips, Z=dorsal(-Z=palm face)
        // Mapping from HIGGS: Z=6 -> our Forward(Y)=6, Y=-2.4 -> our Up(Z)=-2.4
        float rockPalmOffsetForward = 6.0f;    // Along barrel toward fingertips (wand Y axis)
        float rockPalmOffsetUp = -2.4f;        // Toward palm face (wand -Z axis). Negative = into palm.
        float rockPalmOffsetRight = 0.0f;      // Side offset (wand X axis, auto-mirrors for left)

        // --- Feature Toggles ---
        bool rockWeaponCollisionEnabled = false;       // Weapon collision body on layer 44 (disabled until grab debugging complete)

        // --- Debug ---
        bool rockDebugShowColliders = false;
        int rockDebugColliderShape = 4;                // 0=Trigger256, 1=Trigger512, 2=Activator, 3=Utility, 4=VaultSuit
        bool rockDebugVerboseLogging = true;           // Per-frame diagnostic logging (wand axes, AABB hits, grip positions, etc.)

        // --- Object Detection ---
        float rockNearDetectionRange = 25.0f;          // Near grab range (~35cm)
        float rockFarDetectionRange = 350.0f;

        // --- Constraint Motors: Linear ---
        float rockGrabLinearTau = 0.03f;               // Linear motor stiffness (HIGGS: 0.03)
        float rockGrabLinearDamping = 1.0f;            // Linear motor damping (HIGGS: 1.0)
        float rockGrabLinearProportionalRecovery = 4.1f;  // HIGGS: 4.0 (LSB-safe: 4.1)
        float rockGrabLinearConstantRecovery = 2.1f;      // HIGGS: 2.0 (LSB-safe: 2.1)

        // --- Constraint Motors: Angular ---
        float rockGrabAngularTau = 0.03f;              // Angular motor stiffness (HIGGS: 0.03)
        float rockGrabAngularDamping = 0.8f;           // Angular motor damping (HIGGS: 0.8)
        float rockGrabAngularProportionalRecovery = 4.1f; // HIGGS: 4.0 (LSB-safe: 4.1)
        float rockGrabAngularConstantRecovery = 2.1f;     // HIGGS: 2.0 (LSB-safe: 2.1)

        // --- Constraint Force ---
        float rockGrabConstraintMaxForce = 2000.0f;    // Max linear motor force (HIGGS: 2300)
        float rockGrabAngularToLinearForceRatio = 12.5f; // Angular force = linear / this ratio (HIGGS: 17)
        float rockGrabMaxForceToMassRatio = 500.0f;    // Force cap per kg (HIGGS: 600). Prevents light objects from exploding.
        float rockGrabFadeInStartAngularRatio = 100.0f; // Initial angular ratio at grab time, fades to above (HIGGS: 200)

        // --- Constraint Dynamics ---
        float rockGrabForceFadeInTime = 0.1f;          // Seconds to ramp motor force on grab (HIGGS: 0.15)
        float rockGrabTauMin = 0.01f;                  // Tau when held body is colliding (HIGGS: 0.01)
        float rockGrabTauMax = 0.8f;                   // Tau in normal hold (HIGGS objects: 0.03, actors: 0.8)
        float rockGrabTauLerpSpeed = 0.5f;             // Speed of tau transition between min/max (HIGGS: 0.05)
        float rockGrabCloseThreshold = 2.0f;           // Distance considered "close" (game units)
        float rockGrabFarThreshold = 15.0f;            // Distance considered "far" (game units)

        // --- Inertia Normalization ---
        float rockGrabMaxInertiaRatio = 10.0f;         // Max ratio between highest/lowest inertia axis (HIGGS: 10)

        // --- Grab Behavior ---
        float rockGrabMaxDeviation = 50.0f;            // Max distance before force-drop (game units)
        float rockGrabMaxDeviationTime = 2.0f;         // Seconds at max deviation before drop
        int rockGrabButtonID = 2;                      // VR button (2 = grip)
        float rockThrowVelocityMultiplier = 1.5f;      // Velocity scale on throw
        float rockGrabVelocityDamping = 0.1f;          // Velocity damping while held

        // --- Grab Offset (game units from grab surface point) ---
        float rockGrabOffsetForward = 0.0f;
        float rockGrabOffsetUp = 0.0f;
        float rockGrabOffsetRight = 0.0f;

        // --- Grab Lerp (initial snap-to-hand) ---
        float rockGrabLerpSpeed = 300.0f;              // Position lerp speed (game units/sec)
        float rockGrabLerpAngularSpeed = 10.0f;        // Rotation lerp speed (rad/sec)
        float rockGrabLerpMaxTime = 0.5f;              // Max lerp duration (seconds)

        // --- Velocity Clamp (safety net for tracking glitches) ---
        float rockMaxLinearVelocity = 200.0f;          // m/s — normal VR hand ~5-20
        float rockMaxAngularVelocity = 500.0f;         // rad/s — normal VR hand ~1-50, extreme ~125

        // --- Character Controller ---
        float rockCharControllerRadiusScale = 0.7f;    // Scale CC bubble (1.0=default, 0.7=30% smaller)
        bool rockCollideWithCharControllers = false;    // Hand collision with NPC character controllers

    private:
        // Start filesystem watch on the INI file for live hot-reload.
        // Called once at the end of load() after the INI path is resolved.
        void startFileWatch();

        // Resolved once by load(), reused by reload().
        std::string _iniFilePath;

        // Filesystem watch for changes to ROCK.ini — enables live hot-reload.
        std::unique_ptr<filewatch::FileWatch<std::string>> _fileWatch;

        // Last write time of the INI file — used to deduplicate OS events.
        // Windows fires 3-5 FILE_ACTION_MODIFIED events per single file save.
        std::atomic<std::filesystem::file_time_type> _lastIniFileWriteTime;

        // Callbacks to notify when config changes are detected and reloaded.
        std::unordered_map<std::string, std::function<void(const std::string&)>> _onConfigChangedSubscribers;

        // Self-write suppression: set to true before ROCK writes to the INI,
        // so the file-watch callback skips the resulting change event.
        std::atomic<bool> _ignoreNextIniFileChange = false;
    };

    // Module-wide singleton — mirrors the g_config pattern in FRIK.
    // Consumers include this header and reference g_rockConfig.rockEnabled, etc.
    inline RockConfig g_rockConfig;
}
