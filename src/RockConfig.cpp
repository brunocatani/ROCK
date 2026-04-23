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

#include "resources.h"
#include "common/CommonUtils.h"
#include "physics-interaction/PhysicsLog.h"

namespace
{
    // Section name used in both FRIK.ini and ROCK.ini for backwards compatibility.
    constexpr auto SECTION = "PhysicsInteraction";

    // Resolve Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini once.
    // ROCK uses its own config folder, separate from FRIK's FRIK_Config.
    std::string resolveIniPath()
    {
        char documents[MAX_PATH];
        if (SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_MYDOCUMENTS, nullptr, 0, documents))) {
            // Directory creation is handled by createDirDeep() in load(), not here.
            return std::string(documents) + R"(\My Games\Fallout4VR\ROCK_Config\ROCK.ini)";
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
    // resetToDefaults — set all config fields to compiled-in defaults.
    // -------------------------------------------------------------------------
    // WHY: Separating the default assignment from INI reading enables the
    // load()/reload() semantic split. load() resets first so that readValuesFromIni()
    // falls back to compiled defaults for missing keys. reload() skips the reset so
    // that readValuesFromIni() falls back to the current runtime value instead.
    void RockConfig::resetToDefaults()
    {
        rockEnabled = true;

        // Hand collision body
        rockHandCollisionHalfExtentX = 0.09f;
        rockHandCollisionHalfExtentY = 0.015f;
        rockHandCollisionHalfExtentZ = 0.05f;
        rockHandCollisionOffsetX     = 0.0f;
        rockHandCollisionOffsetY     = 0.086f;
        rockHandCollisionOffsetZ     = -0.005f;
        rockHandCollisionBoxRadius   = 0.0f;
        rockHandCollisionOffsetHandspace = RE::NiPoint3(0.086f, -0.005f, 0.0f);

        // Palm offset
        rockPalmOffsetForward = 6.0f;
        rockPalmOffsetUp      = -2.4f;
        rockPalmOffsetRight   = 0.0f;
        rockPalmPositionHandspace = RE::NiPoint3(6.0f, -2.4f, 0.0f);
        rockPalmNormalHandspace = RE::NiPoint3(0.0f, -1.0f, 0.0f);
        rockPointingVectorHandspace = RE::NiPoint3(1.0f, 0.0f, 0.0f);

        // Feature toggles
        rockWeaponCollisionEnabled = false;

        // Selection highlight
        rockHighlightShaderFormID = 0x00249733;  // VansActivateFXS
        rockHighlightEnabled = true;

        // Debug
	        rockDebugShowColliders  = false;
	        rockDebugShowPalmBasis  = false;
	        rockDebugColliderShape  = 4;
	        rockDebugVerboseLogging = true;
	        rockDebugGrabFrameLogging = true;
	        rockDebugHandTransformParity = false;

        // Object detection
        rockNearDetectionRange = 25.0f;
        rockFarDetectionRange  = 350.0f;

        // Constraint motors: linear
        rockGrabLinearTau                  = 0.03f;
        rockGrabLinearDamping              = 1.0f;
        rockGrabLinearProportionalRecovery = 4.1f;
        rockGrabLinearConstantRecovery     = 2.1f;

        // Constraint motors: angular
        rockGrabAngularTau                  = 0.03f;
        rockGrabAngularDamping              = 0.8f;
        rockGrabAngularProportionalRecovery = 4.1f;
        rockGrabAngularConstantRecovery     = 2.1f;

        // Constraint force
        rockGrabConstraintMaxForce       = 2000.0f;
        rockGrabAngularToLinearForceRatio = 12.5f;
        rockGrabMaxForceToMassRatio      = 500.0f;
        rockGrabFadeInStartAngularRatio  = 100.0f;

        // Constraint dynamics
        rockGrabForceFadeInTime = 0.1f;
        rockGrabTauMin          = 0.01f;
        rockGrabTauMax          = 0.8f;
        rockGrabTauLerpSpeed    = 0.5f;
        rockGrabCloseThreshold  = 2.0f;
        rockGrabFarThreshold    = 15.0f;

        // Inertia normalization
        rockGrabMaxInertiaRatio = 10.0f;

        // Grab behavior
        rockGrabMaxDeviation        = 50.0f;
        rockGrabMaxDeviationTime    = 2.0f;
        rockGrabButtonID            = 2;
        rockThrowVelocityMultiplier = 1.5f;
        rockGrabVelocityDamping     = 0.1f;

        // Grab offset
        rockGrabOffsetForward = 0.0f;
        rockGrabOffsetUp      = 0.0f;
        rockGrabOffsetRight   = 0.0f;

        // Grab lerp
        rockGrabLerpSpeed        = 300.0f;
        rockGrabLerpAngularSpeed = 10.0f;
        rockGrabLerpMaxTime      = 0.5f;

        // Velocity clamp
        rockMaxLinearVelocity  = 200.0f;
        rockMaxAngularVelocity = 500.0f;

        // Character controller
        rockCharControllerRadiusScale  = 0.7f;
        rockCollideWithCharControllers = false;
    }

    // -------------------------------------------------------------------------
    // readValuesFromIni — single copy of all INI reads.
    // -------------------------------------------------------------------------
    // Each GetXxxValue call uses the CURRENT field value as the fallback default.
    // When called after resetToDefaults(), "current" = compiled default.
    // When called without reset (reload path), "current" = runtime value.
    void RockConfig::readValuesFromIni(CSimpleIniA& ini)
    {
		auto readVec3 = [&](const char* keyX, const char* keyY, const char* keyZ, RE::NiPoint3& value) {
			value.x = static_cast<float>(ini.GetDoubleValue(SECTION, keyX, value.x));
			value.y = static_cast<float>(ini.GetDoubleValue(SECTION, keyY, value.y));
			value.z = static_cast<float>(ini.GetDoubleValue(SECTION, keyZ, value.z));
		};

        // Enable
        rockEnabled = ini.GetBoolValue(SECTION, "bEnabled", rockEnabled);

        // Hand collision body
        rockHandCollisionHalfExtentX = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionHalfExtentX", rockHandCollisionHalfExtentX));
        rockHandCollisionHalfExtentY = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionHalfExtentY", rockHandCollisionHalfExtentY));
        rockHandCollisionHalfExtentZ = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionHalfExtentZ", rockHandCollisionHalfExtentZ));
        rockHandCollisionOffsetX     = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionOffsetX",     rockHandCollisionOffsetX));
        rockHandCollisionOffsetY     = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionOffsetY",     rockHandCollisionOffsetY));
        rockHandCollisionOffsetZ     = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionOffsetZ",     rockHandCollisionOffsetZ));
        rockHandCollisionBoxRadius   = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionBoxRadius",   rockHandCollisionBoxRadius));
        readVec3("fHandCollisionOffsetHandspaceX", "fHandCollisionOffsetHandspaceY", "fHandCollisionOffsetHandspaceZ",
			rockHandCollisionOffsetHandspace);

        // Palm offset
        rockPalmOffsetForward = static_cast<float>(ini.GetDoubleValue(SECTION, "fPalmOffsetForward", rockPalmOffsetForward));
        rockPalmOffsetUp      = static_cast<float>(ini.GetDoubleValue(SECTION, "fPalmOffsetUp",      rockPalmOffsetUp));
        rockPalmOffsetRight   = static_cast<float>(ini.GetDoubleValue(SECTION, "fPalmOffsetRight",   rockPalmOffsetRight));
        readVec3("fPalmPositionHandspaceX", "fPalmPositionHandspaceY", "fPalmPositionHandspaceZ",
			rockPalmPositionHandspace);
        readVec3("fPalmNormalHandspaceX", "fPalmNormalHandspaceY", "fPalmNormalHandspaceZ",
			rockPalmNormalHandspace);
        readVec3("fPointingVectorHandspaceX", "fPointingVectorHandspaceY", "fPointingVectorHandspaceZ",
			rockPointingVectorHandspace);

        // Feature toggles
        rockWeaponCollisionEnabled = ini.GetBoolValue(SECTION, "bWeaponCollisionEnabled", rockWeaponCollisionEnabled);

        // Selection highlight — FormID as hex string in INI (e.g., "00249733")
        {
            char hexBuf[16] = {};
            snprintf(hexBuf, sizeof(hexBuf), "%08X", rockHighlightShaderFormID);
            const char* hexStr = ini.GetValue(SECTION, "sHighlightShaderFormID", hexBuf);
            if (hexStr && hexStr[0]) {
                rockHighlightShaderFormID = static_cast<std::uint32_t>(std::strtoul(hexStr, nullptr, 16));
            }
        }
        rockHighlightEnabled = ini.GetBoolValue(SECTION, "bHighlightEnabled", rockHighlightEnabled);

        // Debug
	        rockDebugShowColliders  = ini.GetBoolValue(SECTION,  "bDebugShowColliders",  rockDebugShowColliders);
	        rockDebugShowPalmBasis  = ini.GetBoolValue(SECTION,  "bDebugShowPalmBasis",  rockDebugShowPalmBasis);
	        rockDebugColliderShape  = static_cast<int>(ini.GetLongValue(SECTION, "iDebugColliderShape", rockDebugColliderShape));
	        rockDebugVerboseLogging = ini.GetBoolValue(SECTION,  "bDebugVerboseLogging", rockDebugVerboseLogging);
	        rockDebugGrabFrameLogging = ini.GetBoolValue(SECTION, "bDebugGrabFrameLogging", rockDebugGrabFrameLogging);
	        rockDebugHandTransformParity = ini.GetBoolValue(SECTION, "bDebugHandTransformParity", rockDebugHandTransformParity);

        // Object detection
        rockNearDetectionRange = static_cast<float>(ini.GetDoubleValue(SECTION, "fNearDetectionRange", rockNearDetectionRange));
        rockFarDetectionRange  = static_cast<float>(ini.GetDoubleValue(SECTION, "fFarDetectionRange",  rockFarDetectionRange));

        // Constraint motors: linear
        rockGrabLinearTau                  = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearTau",                  rockGrabLinearTau));
        rockGrabLinearDamping              = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearDamping",              rockGrabLinearDamping));
        rockGrabLinearProportionalRecovery = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearProportionalRecovery", rockGrabLinearProportionalRecovery));
        rockGrabLinearConstantRecovery     = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearConstantRecovery",     rockGrabLinearConstantRecovery));

        // Constraint motors: angular
        rockGrabAngularTau                  = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularTau",                  rockGrabAngularTau));
        rockGrabAngularDamping              = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularDamping",              rockGrabAngularDamping));
        rockGrabAngularProportionalRecovery = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularProportionalRecovery", rockGrabAngularProportionalRecovery));
        rockGrabAngularConstantRecovery     = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularConstantRecovery",     rockGrabAngularConstantRecovery));

        // Constraint force
        rockGrabConstraintMaxForce        = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabConstraintMaxForce",        rockGrabConstraintMaxForce));
        rockGrabAngularToLinearForceRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularToLinearForceRatio", rockGrabAngularToLinearForceRatio));
        rockGrabMaxForceToMassRatio       = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxForceToMassRatio",       rockGrabMaxForceToMassRatio));
        rockGrabFadeInStartAngularRatio   = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFadeInStartAngularRatio",   rockGrabFadeInStartAngularRatio));

        // Constraint dynamics
        rockGrabForceFadeInTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabForceFadeInTime", rockGrabForceFadeInTime));
        rockGrabTauMin          = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauMin",          rockGrabTauMin));
        rockGrabTauMax          = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauMax",          rockGrabTauMax));
        rockGrabTauLerpSpeed    = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauLerpSpeed",    rockGrabTauLerpSpeed));
        rockGrabCloseThreshold  = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabCloseThreshold",  rockGrabCloseThreshold));
        rockGrabFarThreshold    = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFarThreshold",    rockGrabFarThreshold));

        // Inertia normalization
        rockGrabMaxInertiaRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxInertiaRatio", rockGrabMaxInertiaRatio));

        // Grab behavior
        rockGrabMaxDeviation        = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviation",        rockGrabMaxDeviation));
        rockGrabMaxDeviationTime    = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviationTime",    rockGrabMaxDeviationTime));
        rockGrabButtonID            = static_cast<int>(  ini.GetLongValue(  SECTION, "iGrabButtonID",            rockGrabButtonID));
        rockThrowVelocityMultiplier = static_cast<float>(ini.GetDoubleValue(SECTION, "fThrowVelocityMultiplier", rockThrowVelocityMultiplier));
        rockGrabVelocityDamping     = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabVelocityDamping",     rockGrabVelocityDamping));

        // Grab offset
        rockGrabOffsetForward = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabOffsetForward", rockGrabOffsetForward));
        rockGrabOffsetUp      = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabOffsetUp",      rockGrabOffsetUp));
        rockGrabOffsetRight   = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabOffsetRight",   rockGrabOffsetRight));

        // Grab lerp
        rockGrabLerpSpeed        = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpSpeed",        rockGrabLerpSpeed));
        rockGrabLerpAngularSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpAngularSpeed", rockGrabLerpAngularSpeed));
        rockGrabLerpMaxTime      = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpMaxTime",      rockGrabLerpMaxTime));

        // Velocity clamp
        rockMaxLinearVelocity  = static_cast<float>(ini.GetDoubleValue(SECTION, "fMaxLinearVelocity",  rockMaxLinearVelocity));
        rockMaxAngularVelocity = static_cast<float>(ini.GetDoubleValue(SECTION, "fMaxAngularVelocity", rockMaxAngularVelocity));

        // Character controller
        rockCharControllerRadiusScale  = static_cast<float>(ini.GetDoubleValue(SECTION, "fCharControllerRadiusScale",  rockCharControllerRadiusScale));
        rockCollideWithCharControllers = ini.GetBoolValue(SECTION, "bCollideWithCharControllers", rockCollideWithCharControllers);
    }

    // -------------------------------------------------------------------------
    // load — resolve path and read all values from disk.
    // -------------------------------------------------------------------------
    void RockConfig::load()
    {
        _iniFilePath = resolveIniPath();
        ROCK_LOG_INFO(Config, "Loading ROCK config from: {}", _iniFilePath);

        // Create parent directories if they don't exist (same pattern as ConfigBase::load).
        f4cf::common::createDirDeep(_iniFilePath);

        // Extract the fully-commented default ROCK.ini from the embedded DLL resource
        // if the file doesn't exist yet. This gives users a complete config file with
        // all keys, descriptions, and shader FormID candidates on first install.
        // Same pattern as ConfigBase::loadIniConfig() line 123.
        f4cf::common::createFileFromResourceIfNotExists(_iniFilePath, "ROCK", IDR_ROCK_INI, true);

        CSimpleIniA ini;
        ini.SetUnicode(false);
        const SI_Error rc = ini.LoadFile(_iniFilePath.c_str());
        if (rc < 0) {
            ROCK_LOG_WARN(Config, "ROCK.ini not found or unreadable (code {}), using compiled-in defaults", static_cast<int>(rc));
        }

        resetToDefaults();
        readValuesFromIni(ini);

        ROCK_LOG_INFO(Config, "ROCK config loaded (rockEnabled={})", rockEnabled);

        // Start file-watching for live hot-reload after the first successful read.
        startFileWatch();
    }

    // -------------------------------------------------------------------------
    // reload — re-read the INI file that was resolved by load().
    // -------------------------------------------------------------------------
    // Unlike load(), reload() does NOT call resetToDefaults() first. This means
    // readValuesFromIni() falls back to current runtime values for missing keys,
    // preserving tuned values that were removed from the INI during a live edit.
    void RockConfig::reload()
    {
        if (_iniFilePath.empty()) {
            ROCK_LOG_WARN(Config, "reload() called before load() — delegating to load()");
            load();
            return;
        }

        CSimpleIniA ini;
        ini.SetUnicode(false);
        const SI_Error rc = ini.LoadFile(_iniFilePath.c_str());
        if (rc < 0) {
            ROCK_LOG_WARN(Config, "ROCK.ini reload failed (code {}), retaining current values", static_cast<int>(rc));
            return;
        }

        readValuesFromIni(ini);
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

        // Join any previous init thread before starting a new one.
        if (_fileWatchInitThread.joinable()) {
            _fileWatchInitThread.join();
        }

        _fileWatchInitThread = std::thread([this]() {
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
        });

        // The thread's only job is to construct the FileWatch (which sets up the OS watch
        // handle). Once that returns, FileWatch's internal monitoring thread takes over.
        // Join here so we never have a detached thread capturing 'this'.
        _fileWatchInitThread.join();
    }

    // -------------------------------------------------------------------------
    // stopFileWatch — tear down the file-watcher.
    // -------------------------------------------------------------------------
    void RockConfig::stopFileWatch()
    {
        if (_fileWatchInitThread.joinable()) {
            _fileWatchInitThread.join();
        }
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
