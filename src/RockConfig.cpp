// RockConfig.cpp — INI loading implementation for RockConfig.
//
// WHY: ROCK is a standalone DLL and cannot link against FRIK's ConfigBase machinery
// (which requires embedded WIN32 resources, a file-watcher thread, and version migration
// logic that are not appropriate for a lean physics plugin). Instead, we open CSimpleIniA
// directly, load the file, read every key with the same section name and key identifiers
// used in the monolith's Config.cpp, and fall back to the same compiled-in defaults.
//
// INI location: Documents\My Games\Fallout4VR\FRIK_Config\ROCK.ini
// This mirrors the existing FRIK_Config folder so users can keep all related config files
// in one place. The path is resolved once at load() time via SHGetFolderPath(CSIDL_MYDOCUMENTS).
//
// Section name: [PhysicsInteraction] — the same name used in FRIK.ini, which means users
// who already have custom [PhysicsInteraction] values in FRIK.ini can copy that section
// verbatim into ROCK.ini without renaming a single key.
//
// CSimpleIniA behaviour: if the file does not exist, LoadFile returns SI_FILE and every
// GetXxxValue call returns the supplied default — effectively giving us safe no-file operation.

#include "RockConfig.h"

#include <ShlObj.h>
#include <SimpleIni.h>
#include <filesystem>
#include <thread>

#include "physics-interaction/PhysicsLog.h"

namespace
{
    // Section name used in both FRIK.ini and ROCK.ini for backwards compatibility.
    constexpr auto SECTION = "PhysicsInteraction";

    // Resolve Documents\My Games\Fallout4VR\FRIK_Config\ROCK.ini once.
    std::string resolveIniPath()
    {
        char documents[MAX_PATH];
        if (SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_MYDOCUMENTS, nullptr, 0, documents))) {
            std::string base = std::string(documents) + R"(\My Games\Fallout4VR\FRIK_Config)";
            // Ensure the directory exists so that ROCK.ini can be created by the user or tooling.
            std::error_code ec;
            std::filesystem::create_directories(base, ec);
            return base + R"(\ROCK.ini)";
        }
        // Fallback: use a path relative to the working directory (game root).
        // This should never be reached on a properly configured Windows system.
        ROCK_LOG_WARN(Config, "SHGetFolderPath failed — using fallback ROCK.ini path");
        return R"(Data\F4SE\Plugins\ROCK.ini)";
    }
}

namespace frik::rock
{
    // -------------------------------------------------------------------------
    // load — resolve path and read all values from disk.
    // -------------------------------------------------------------------------
    void RockConfig::load()
    {
        _iniFilePath = resolveIniPath();
        ROCK_LOG_INFO(Config, "Loading ROCK config from: {}", _iniFilePath);

        CSimpleIniA ini;
        ini.SetUnicode(false);
        const SI_Error rc = ini.LoadFile(_iniFilePath.c_str());
        if (rc < 0) {
            ROCK_LOG_WARN(Config, "ROCK.ini not found or unreadable (code {}), using compiled-in defaults", static_cast<int>(rc));
            // Fall through — every GetXxxValue call below returns its default argument.
        }

        // --- Enable ---
        rockEnabled = ini.GetBoolValue(SECTION, "bEnabled", true);

        // --- Hand collision body ---
        rockHandCollisionHalfExtentX   = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionHalfExtentX",  0.06));
        rockHandCollisionHalfExtentY   = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionHalfExtentY",  0.02));
        rockHandCollisionHalfExtentZ   = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionHalfExtentZ",  0.015));
        rockHandCollisionOffsetX       = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionOffsetX",      0.0));
        rockHandCollisionOffsetY       = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionOffsetY",      0.086));
        rockHandCollisionOffsetZ       = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionOffsetZ",     -0.005));
        rockHandCollisionBoxRadius     = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionBoxRadius",    0.0));

        // --- Palm offset ---
        rockPalmOffsetForward = static_cast<float>(ini.GetDoubleValue(SECTION, "fPalmOffsetForward",  6.0));
        rockPalmOffsetUp      = static_cast<float>(ini.GetDoubleValue(SECTION, "fPalmOffsetUp",      -2.4));
        rockPalmOffsetRight   = static_cast<float>(ini.GetDoubleValue(SECTION, "fPalmOffsetRight",    0.0));

        // --- Feature toggles ---
        rockWeaponCollisionEnabled = ini.GetBoolValue(SECTION, "bWeaponCollisionEnabled", false);

        // --- Debug ---
        rockDebugShowColliders  = ini.GetBoolValue(SECTION,  "bDebugShowColliders",  false);
        rockDebugColliderShape  = static_cast<int>(ini.GetLongValue(SECTION, "iDebugColliderShape", 4));
        rockDebugVerboseLogging = ini.GetBoolValue(SECTION,  "bDebugVerboseLogging", true);

        // --- Object detection ---
        rockNearDetectionRange = static_cast<float>(ini.GetDoubleValue(SECTION, "fNearDetectionRange",  25.0));
        rockFarDetectionRange  = static_cast<float>(ini.GetDoubleValue(SECTION, "fFarDetectionRange",  350.0));

        // --- Constraint motors: linear ---
        rockGrabLinearTau                   = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearTau",                   0.03));
        rockGrabLinearDamping               = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearDamping",               1.0));
        rockGrabLinearProportionalRecovery  = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearProportionalRecovery",  4.1));
        rockGrabLinearConstantRecovery      = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearConstantRecovery",      2.1));

        // --- Constraint motors: angular ---
        rockGrabAngularTau                   = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularTau",                   0.03));
        rockGrabAngularDamping               = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularDamping",               0.8));
        rockGrabAngularProportionalRecovery  = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularProportionalRecovery",  4.1));
        rockGrabAngularConstantRecovery      = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularConstantRecovery",      2.1));

        // --- Constraint force ---
        rockGrabConstraintMaxForce        = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabConstraintMaxForce",        2000.0));
        rockGrabAngularToLinearForceRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularToLinearForceRatio",   12.5));
        rockGrabMaxForceToMassRatio       = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxForceToMassRatio",        500.0));
        rockGrabFadeInStartAngularRatio   = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFadeInStartAngularRatio",    100.0));

        // --- Constraint dynamics ---
        rockGrabForceFadeInTime  = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabForceFadeInTime",  0.1));
        rockGrabTauMin           = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauMin",           0.01));
        rockGrabTauMax           = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauMax",           0.8));
        rockGrabTauLerpSpeed     = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauLerpSpeed",     0.5));
        rockGrabCloseThreshold   = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabCloseThreshold",   2.0));
        rockGrabFarThreshold     = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFarThreshold",    15.0));

        // --- Inertia normalization ---
        rockGrabMaxInertiaRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxInertiaRatio", 10.0));

        // --- Grab behavior ---
        rockGrabMaxDeviation          = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviation",          50.0));
        rockGrabMaxDeviationTime      = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviationTime",       2.0));
        rockGrabButtonID              = static_cast<int>(  ini.GetLongValue(  SECTION, "iGrabButtonID",               2));
        rockThrowVelocityMultiplier   = static_cast<float>(ini.GetDoubleValue(SECTION, "fThrowVelocityMultiplier",    1.5));
        rockGrabVelocityDamping       = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabVelocityDamping",        0.1));

        // --- Grab offset ---
        rockGrabOffsetForward = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabOffsetForward", 0.0));
        rockGrabOffsetUp      = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabOffsetUp",      0.0));
        rockGrabOffsetRight   = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabOffsetRight",   0.0));

        // --- Grab lerp ---
        rockGrabLerpSpeed        = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpSpeed",        300.0));
        rockGrabLerpAngularSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpAngularSpeed",  10.0));
        rockGrabLerpMaxTime      = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpMaxTime",        0.5));

        // --- Velocity clamp ---
        rockMaxLinearVelocity  = static_cast<float>(ini.GetDoubleValue(SECTION, "fMaxLinearVelocity",  200.0));
        rockMaxAngularVelocity = static_cast<float>(ini.GetDoubleValue(SECTION, "fMaxAngularVelocity", 500.0));

        // --- Character controller ---
        rockCharControllerRadiusScale    = static_cast<float>(ini.GetDoubleValue(SECTION, "fCharControllerRadiusScale",   0.7));
        rockCollideWithCharControllers   = ini.GetBoolValue(SECTION, "bCollideWithCharControllers", false);

        ROCK_LOG_INFO(Config, "ROCK config loaded (rockEnabled={})", rockEnabled);

        // Start file-watching for live hot-reload after the first successful read.
        startFileWatch();
    }

    // -------------------------------------------------------------------------
    // reload — re-read the INI file that was resolved by load().
    // -------------------------------------------------------------------------
    void RockConfig::reload()
    {
        // NOTE: Unlike load(), reload() uses the current field value as the fallback default
        // when a key is missing from the INI. This means:
        //   - load(): missing key → compiled-in default (e.g., rockEnabled defaults to true)
        //   - reload(): missing key → current runtime value is preserved
        // This is intentional: reload() should not reset tuned values just because
        // a key was removed from the INI file during a live edit session.

        if (_iniFilePath.empty()) {
            ROCK_LOG_WARN(Config, "reload() called before load() — delegating to load()");
            load();
            return;
        }
        ROCK_LOG_INFO(Config, "Reloading ROCK config from: {}", _iniFilePath);

        CSimpleIniA ini;
        ini.SetUnicode(false);
        const SI_Error rc = ini.LoadFile(_iniFilePath.c_str());
        if (rc < 0) {
            ROCK_LOG_WARN(Config, "ROCK.ini reload failed (code {}), retaining current values", static_cast<int>(rc));
            return;
        }

        // Re-read every value.  Duplicating the call block here (rather than extracting to a shared
        // helper that takes a CSimpleIniA&) keeps the load/reload paths independently readable and
        // avoids introducing a mutable reference pathway that could be misused in future.
        rockEnabled = ini.GetBoolValue(SECTION, "bEnabled", rockEnabled);

        rockHandCollisionHalfExtentX   = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionHalfExtentX",  rockHandCollisionHalfExtentX));
        rockHandCollisionHalfExtentY   = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionHalfExtentY",  rockHandCollisionHalfExtentY));
        rockHandCollisionHalfExtentZ   = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionHalfExtentZ",  rockHandCollisionHalfExtentZ));
        rockHandCollisionOffsetX       = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionOffsetX",      rockHandCollisionOffsetX));
        rockHandCollisionOffsetY       = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionOffsetY",      rockHandCollisionOffsetY));
        rockHandCollisionOffsetZ       = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionOffsetZ",      rockHandCollisionOffsetZ));
        rockHandCollisionBoxRadius     = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionBoxRadius",    rockHandCollisionBoxRadius));

        rockPalmOffsetForward = static_cast<float>(ini.GetDoubleValue(SECTION, "fPalmOffsetForward", rockPalmOffsetForward));
        rockPalmOffsetUp      = static_cast<float>(ini.GetDoubleValue(SECTION, "fPalmOffsetUp",      rockPalmOffsetUp));
        rockPalmOffsetRight   = static_cast<float>(ini.GetDoubleValue(SECTION, "fPalmOffsetRight",   rockPalmOffsetRight));

        rockWeaponCollisionEnabled = ini.GetBoolValue(SECTION, "bWeaponCollisionEnabled", rockWeaponCollisionEnabled);

        rockDebugShowColliders  = ini.GetBoolValue(SECTION, "bDebugShowColliders",  rockDebugShowColliders);
        rockDebugColliderShape  = static_cast<int>(ini.GetLongValue(SECTION, "iDebugColliderShape", rockDebugColliderShape));
        rockDebugVerboseLogging = ini.GetBoolValue(SECTION, "bDebugVerboseLogging", rockDebugVerboseLogging);

        rockNearDetectionRange = static_cast<float>(ini.GetDoubleValue(SECTION, "fNearDetectionRange", rockNearDetectionRange));
        rockFarDetectionRange  = static_cast<float>(ini.GetDoubleValue(SECTION, "fFarDetectionRange",  rockFarDetectionRange));

        rockGrabLinearTau                   = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearTau",                  rockGrabLinearTau));
        rockGrabLinearDamping               = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearDamping",              rockGrabLinearDamping));
        rockGrabLinearProportionalRecovery  = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearProportionalRecovery", rockGrabLinearProportionalRecovery));
        rockGrabLinearConstantRecovery      = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearConstantRecovery",     rockGrabLinearConstantRecovery));

        rockGrabAngularTau                   = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularTau",                  rockGrabAngularTau));
        rockGrabAngularDamping               = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularDamping",              rockGrabAngularDamping));
        rockGrabAngularProportionalRecovery  = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularProportionalRecovery", rockGrabAngularProportionalRecovery));
        rockGrabAngularConstantRecovery      = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularConstantRecovery",     rockGrabAngularConstantRecovery));

        rockGrabConstraintMaxForce        = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabConstraintMaxForce",       rockGrabConstraintMaxForce));
        rockGrabAngularToLinearForceRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularToLinearForceRatio",rockGrabAngularToLinearForceRatio));
        rockGrabMaxForceToMassRatio       = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxForceToMassRatio",      rockGrabMaxForceToMassRatio));
        rockGrabFadeInStartAngularRatio   = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFadeInStartAngularRatio",  rockGrabFadeInStartAngularRatio));

        rockGrabForceFadeInTime  = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabForceFadeInTime",  rockGrabForceFadeInTime));
        rockGrabTauMin           = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauMin",           rockGrabTauMin));
        rockGrabTauMax           = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauMax",           rockGrabTauMax));
        rockGrabTauLerpSpeed     = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauLerpSpeed",     rockGrabTauLerpSpeed));
        rockGrabCloseThreshold   = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabCloseThreshold",   rockGrabCloseThreshold));
        rockGrabFarThreshold     = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFarThreshold",     rockGrabFarThreshold));

        rockGrabMaxInertiaRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxInertiaRatio", rockGrabMaxInertiaRatio));

        rockGrabMaxDeviation        = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviation",       rockGrabMaxDeviation));
        rockGrabMaxDeviationTime    = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviationTime",   rockGrabMaxDeviationTime));
        rockGrabButtonID            = static_cast<int>(  ini.GetLongValue(  SECTION, "iGrabButtonID",           rockGrabButtonID));
        rockThrowVelocityMultiplier = static_cast<float>(ini.GetDoubleValue(SECTION, "fThrowVelocityMultiplier",rockThrowVelocityMultiplier));
        rockGrabVelocityDamping     = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabVelocityDamping",    rockGrabVelocityDamping));

        rockGrabOffsetForward = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabOffsetForward", rockGrabOffsetForward));
        rockGrabOffsetUp      = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabOffsetUp",      rockGrabOffsetUp));
        rockGrabOffsetRight   = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabOffsetRight",   rockGrabOffsetRight));

        rockGrabLerpSpeed        = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpSpeed",        rockGrabLerpSpeed));
        rockGrabLerpAngularSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpAngularSpeed", rockGrabLerpAngularSpeed));
        rockGrabLerpMaxTime      = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpMaxTime",      rockGrabLerpMaxTime));

        rockMaxLinearVelocity  = static_cast<float>(ini.GetDoubleValue(SECTION, "fMaxLinearVelocity",  rockMaxLinearVelocity));
        rockMaxAngularVelocity = static_cast<float>(ini.GetDoubleValue(SECTION, "fMaxAngularVelocity", rockMaxAngularVelocity));

        rockCharControllerRadiusScale  = static_cast<float>(ini.GetDoubleValue(SECTION, "fCharControllerRadiusScale",  rockCharControllerRadiusScale));
        rockCollideWithCharControllers = ini.GetBoolValue(SECTION, "bCollideWithCharControllers", rockCollideWithCharControllers);

        ROCK_LOG_INFO(Config, "ROCK config reloaded (rockEnabled={})", rockEnabled);
    }

    // -------------------------------------------------------------------------
    // startFileWatch — filesystem watch for live INI hot-reload.
    // -------------------------------------------------------------------------
    //
    // WHY: Identical pattern to F4VR-CommonFramework's ConfigBase::startIniConfigFileWatch().
    // The FileWatch constructor blocks while setting up the OS watch handle via
    // ReadDirectoryChangesW, which can deadlock if called from the game thread during
    // mod initialization. So we spawn a detached thread to create the FileWatch object,
    // then its internal thread takes over for monitoring.
    //
    // DEDUPLICATION: Windows fires 3-5 FILE_ACTION_MODIFIED events per single Notepad save.
    // We use atomic compare-exchange on the file's last_write_time with a 200ms debounce
    // window. Only the first thread to CAS the new write time through gets to call reload().
    //
    // SELF-WRITE SUPPRESSION: If ROCK ever programmatically writes to ROCK.ini (e.g., from
    // an in-game settings UI), the caller must call suppressNextFileWatchReload() BEFORE the
    // write. The callback checks and clears the flag, skipping the resulting change event.
    void RockConfig::startFileWatch()
    {
        if (_fileWatch) {
            return;  // Already watching.
        }
        if (_iniFilePath.empty()) {
            ROCK_LOG_WARN(Config, "Cannot start file watch — INI path not resolved");
            return;
        }

        std::thread([this]() {
            ROCK_LOG_INFO(Config, "Starting file watch on '{}'", _iniFilePath);

            _fileWatch = std::make_unique<filewatch::FileWatch<std::string>>(
                _iniFilePath, [this](const std::string&, const filewatch::Event changeType) {
                    if (changeType != filewatch::Event::modified) {
                        return;
                    }

                    constexpr auto delay = std::chrono::milliseconds(200);

                    // Deduplicate: atomic CAS on last_write_time ensures only 1 thread processes per write.
                    auto prevWriteTime = _lastIniFileWriteTime.load();
                    std::error_code ec;
                    const auto writeTime = std::filesystem::last_write_time(_iniFilePath, ec);
                    if (ec || !_lastIniFileWriteTime.compare_exchange_strong(prevWriteTime, writeTime)
                        || writeTime - prevWriteTime < delay) {
                        return;
                    }

                    // Self-write suppression: skip if WE modified the file.
                    bool expected = true;
                    if (_ignoreNextIniFileChange.compare_exchange_strong(expected, false)) {
                        return;
                    }

                    // Wait for file stability (200ms since last write event).
                    // This prevents reading a half-written file while the editor is still flushing.
                    auto now = std::filesystem::file_time_type::clock::now();
                    auto lastEventTime = _lastIniFileWriteTime.load();
                    while (now - lastEventTime < delay) {
                        std::this_thread::sleep_for(
                            std::max(std::chrono::milliseconds(0), std::chrono::duration_cast<std::chrono::milliseconds>(delay - (now - lastEventTime))));
                        now = std::filesystem::file_time_type::clock::now();
                        lastEventTime = _lastIniFileWriteTime.load();
                    }

                    ROCK_LOG_INFO(Config, "ROCK.ini change detected, reloading...");
                    reload();

                    // Notify subscribers.
                    for (const auto& [key, subscriber] : _onConfigChangedSubscribers) {
                        ROCK_LOG_INFO(Config, "Notify config change subscriber '{}'", key);
                        subscriber(key);
                    }
                });
        }).detach();
    }

    // -------------------------------------------------------------------------
    // stopFileWatch — tear down the file-watcher.
    // -------------------------------------------------------------------------
    void RockConfig::stopFileWatch()
    {
        if (_fileWatch) {
            ROCK_LOG_INFO(Config, "Stopping file watch on ROCK.ini");
            _fileWatch.reset();
        }
    }

    // -------------------------------------------------------------------------
    // Config change subscriber management.
    // -------------------------------------------------------------------------
    void RockConfig::subscribeForConfigChanged(const std::string& key, std::function<void(const std::string&)> callback)
    {
        _onConfigChangedSubscribers[key] = std::move(callback);
    }

    void RockConfig::unsubscribeFromConfigChanged(const std::string& key)
    {
        _onConfigChangedSubscribers.erase(key);
    }
}
