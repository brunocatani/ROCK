

#include "RockConfig.h"

#include <ShlObj.h>
#include <SimpleIni.h>
#include <cmath>
#include <filesystem>
#include <thread>

#include "common/CommonUtils.h"
#include "physics-interaction/PhysicsLog.h"
#include "resources.h"

namespace
{

    constexpr auto SECTION = "PhysicsInteraction";
    const RE::NiPoint3 kDefaultPalmNormalHandspace{ 0.0f, 0.0f, 1.0f };

    std::string resolveIniPath()
    {
        char documents[MAX_PATH];
        if (SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_MYDOCUMENTS, nullptr, 0, documents))) {
            return std::string(documents) + R"(\My Games\Fallout4VR\ROCK_Config\ROCK.ini)";
        }

        ROCK_LOG_WARN(Config, "SHGetFolderPath failed — using fallback ROCK.ini path");
        return R"(Data\F4SE\Plugins\ROCK.ini)";
    }
}

namespace frik::rock
{

    void RockConfig::resetToDefaults()
    {
        rockEnabled = true;

        rockHandCollisionHalfExtentX = 0.09f;
        rockHandCollisionHalfExtentY = 0.05f;
        rockHandCollisionHalfExtentZ = 0.02f;
        rockHandCollisionBoxRadius = 0.0f;
        rockHandCollisionOffsetHandspace = RE::NiPoint3(0.086f, -0.005f, 0.0f);

        rockPalmPositionHandspace = RE::NiPoint3(0.0f, -2.4f, 6.0f);
        rockPalmNormalHandspace = kDefaultPalmNormalHandspace;
        rockPointingVectorHandspace = RE::NiPoint3(0.0f, 0.0f, 1.0f);
        rockReversePalmNormal = true;
        rockReverseFarGrabNormal = false;
        rockHandspaceBasisMode = 1;

        rockWeaponCollisionEnabled = false;
        rockWeaponCollisionBlocksProjectiles = false;
        rockWeaponCollisionBlocksSpells = false;
        rockWeaponCollisionRotationCorrectionEnabled = false;
        rockWeaponCollisionRotationDegrees = RE::NiPoint3(0.0f, 0.0f, 0.0f);
        rockWeaponCollisionConvexRadius = 0.01f;
        rockWeaponCollisionPointDedupGrid = 0.002f;

        rockHighlightShaderFormID = 0x00249733;
        rockHighlightEnabled = true;

        rockDebugShowColliders = false;
        rockDebugShowTargetColliders = false;
        rockDebugShowHandAxes = false;
        rockDebugShowGrabPivots = false;
        rockDebugShowPalmVectors = false;
        rockDebugShowPalmBasis = false;
        rockDebugDrawHandColliders = true;
        rockDebugDrawWeaponColliders = true;
        rockDebugMaxWeaponBodiesDrawn = 6;
        rockDebugMaxShapeGenerationsPerFrame = 2;
        rockDebugMaxConvexSupportVertices = 64;
        rockDebugUseBoundsForHeavyConvex = true;
        rockDebugVerboseLogging = false;
        rockDebugGrabFrameLogging = false;
        rockDebugHandTransformParity = false;
        rockHandFrameSource = 3;
        rockHandFrameSwapWands = false;

        rockNearDetectionRange = 25.0f;
        rockFarDetectionRange = 350.0f;

        rockGrabLinearTau = 0.03f;
        rockGrabLinearDamping = 1.0f;
        rockGrabLinearProportionalRecovery = 4.1f;
        rockGrabLinearConstantRecovery = 2.1f;

        rockGrabAngularTau = 0.03f;
        rockGrabAngularDamping = 0.8f;
        rockGrabAngularProportionalRecovery = 4.1f;
        rockGrabAngularConstantRecovery = 2.1f;

        rockGrabConstraintMaxForce = 2000.0f;
        rockGrabAngularToLinearForceRatio = 12.5f;
        rockGrabMaxForceToMassRatio = 500.0f;
        rockGrabFadeInStartAngularRatio = 100.0f;

        rockGrabForceFadeInTime = 0.1f;
        rockGrabTauMin = 0.01f;
        rockGrabTauMax = 0.8f;
        rockGrabTauLerpSpeed = 0.5f;
        rockGrabCloseThreshold = 2.0f;
        rockGrabFarThreshold = 15.0f;

        rockGrabMaxInertiaRatio = 10.0f;

        rockGrabMaxDeviation = 50.0f;
        rockGrabMaxDeviationTime = 2.0f;
        rockGrabButtonID = 2;
        rockThrowVelocityMultiplier = 1.5f;
        rockGrabVelocityDamping = 0.1f;

        rockGrabPivotAOffsetHandspace = RE::NiPoint3(0.0f, 0.0f, 0.0f);
        rockReverseGrabPivotAOffset = false;

        rockGrabLerpSpeed = 300.0f;
        rockGrabLerpAngularSpeed = 10.0f;
        rockGrabLerpMaxTime = 0.5f;

        rockMaxLinearVelocity = 200.0f;
        rockMaxAngularVelocity = 500.0f;

        rockCharControllerRadiusScale = 0.7f;
        rockCollideWithCharControllers = false;
    }

    void RockConfig::readValuesFromIni(CSimpleIniA& ini)
    {
        auto readVec3 = [&](const char* keyX, const char* keyY, const char* keyZ, RE::NiPoint3& value) {
            value.x = static_cast<float>(ini.GetDoubleValue(SECTION, keyX, value.x));
            value.y = static_cast<float>(ini.GetDoubleValue(SECTION, keyY, value.y));
            value.z = static_cast<float>(ini.GetDoubleValue(SECTION, keyZ, value.z));
        };

        rockEnabled = ini.GetBoolValue(SECTION, "bEnabled", rockEnabled);

        rockHandCollisionHalfExtentX = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionHalfExtentX", rockHandCollisionHalfExtentX));
        rockHandCollisionHalfExtentY = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionHalfExtentY", rockHandCollisionHalfExtentY));
        rockHandCollisionHalfExtentZ = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionHalfExtentZ", rockHandCollisionHalfExtentZ));
        rockHandCollisionBoxRadius = static_cast<float>(ini.GetDoubleValue(SECTION, "fHandCollisionBoxRadius", rockHandCollisionBoxRadius));
        readVec3("fHandCollisionOffsetHandspaceX", "fHandCollisionOffsetHandspaceY", "fHandCollisionOffsetHandspaceZ", rockHandCollisionOffsetHandspace);

        readVec3("fPalmPositionHandspaceX", "fPalmPositionHandspaceY", "fPalmPositionHandspaceZ", rockPalmPositionHandspace);
        readVec3("fPalmNormalHandspaceX", "fPalmNormalHandspaceY", "fPalmNormalHandspaceZ", rockPalmNormalHandspace);
        if (std::abs(rockPalmNormalHandspace.x) < 0.0001f && std::abs(rockPalmNormalHandspace.y + 1.0f) < 0.0001f && std::abs(rockPalmNormalHandspace.z) < 0.0001f) {
            rockPalmNormalHandspace = kDefaultPalmNormalHandspace;
        }
        readVec3("fPointingVectorHandspaceX", "fPointingVectorHandspaceY", "fPointingVectorHandspaceZ", rockPointingVectorHandspace);
        rockReversePalmNormal = ini.GetBoolValue(SECTION, "bReversePalmNormal", rockReversePalmNormal);
        rockReverseFarGrabNormal = ini.GetBoolValue(SECTION, "bReverseFarGrabNormal", rockReverseFarGrabNormal);
        rockHandspaceBasisMode = static_cast<int>(ini.GetLongValue(SECTION, "iHandspaceBasisMode", rockHandspaceBasisMode));

        rockWeaponCollisionEnabled = ini.GetBoolValue(SECTION, "bWeaponCollisionEnabled", rockWeaponCollisionEnabled);
        rockWeaponCollisionBlocksProjectiles = ini.GetBoolValue(SECTION, "bWeaponCollisionBlocksProjectiles", rockWeaponCollisionBlocksProjectiles);
        rockWeaponCollisionBlocksSpells = ini.GetBoolValue(SECTION, "bWeaponCollisionBlocksSpells", rockWeaponCollisionBlocksSpells);
        rockWeaponCollisionRotationCorrectionEnabled =
            ini.GetBoolValue(SECTION, "bWeaponCollisionRotationCorrectionEnabled", rockWeaponCollisionRotationCorrectionEnabled);
        readVec3("fWeaponCollisionRotationX", "fWeaponCollisionRotationY", "fWeaponCollisionRotationZ", rockWeaponCollisionRotationDegrees);
        rockWeaponCollisionConvexRadius = static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionConvexRadius", rockWeaponCollisionConvexRadius));
        rockWeaponCollisionPointDedupGrid = static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionPointDedupGrid", rockWeaponCollisionPointDedupGrid));

        {
            char hexBuf[16] = {};
            snprintf(hexBuf, sizeof(hexBuf), "%08X", rockHighlightShaderFormID);
            const char* hexStr = ini.GetValue(SECTION, "sHighlightShaderFormID", hexBuf);
            if (hexStr && hexStr[0]) {
                rockHighlightShaderFormID = static_cast<std::uint32_t>(std::strtoul(hexStr, nullptr, 16));
            }
        }
        rockHighlightEnabled = ini.GetBoolValue(SECTION, "bHighlightEnabled", rockHighlightEnabled);

        rockDebugShowColliders = ini.GetBoolValue(SECTION, "bDebugShowColliders", rockDebugShowColliders);
        rockDebugShowTargetColliders = ini.GetBoolValue(SECTION, "bDebugShowTargetColliders", rockDebugShowTargetColliders);
        rockDebugShowHandAxes = ini.GetBoolValue(SECTION, "bDebugShowHandAxes", rockDebugShowHandAxes);
        rockDebugShowGrabPivots = ini.GetBoolValue(SECTION, "bDebugShowGrabPivots", rockDebugShowGrabPivots);
        rockDebugShowPalmVectors = ini.GetBoolValue(SECTION, "bDebugShowPalmVectors", rockDebugShowPalmVectors);
        rockDebugShowPalmBasis = ini.GetBoolValue(SECTION, "bDebugShowPalmBasis", rockDebugShowPalmBasis);
        rockDebugDrawHandColliders = ini.GetBoolValue(SECTION, "bDebugDrawHandColliders", rockDebugDrawHandColliders);
        rockDebugDrawWeaponColliders = ini.GetBoolValue(SECTION, "bDebugDrawWeaponColliders", rockDebugDrawWeaponColliders);
        rockDebugMaxWeaponBodiesDrawn = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxWeaponBodiesDrawn", rockDebugMaxWeaponBodiesDrawn));
        rockDebugMaxShapeGenerationsPerFrame = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxShapeGenerationsPerFrame", rockDebugMaxShapeGenerationsPerFrame));
        rockDebugMaxConvexSupportVertices = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxConvexSupportVertices", rockDebugMaxConvexSupportVertices));
        rockDebugUseBoundsForHeavyConvex = ini.GetBoolValue(SECTION, "bDebugUseBoundsForHeavyConvex", rockDebugUseBoundsForHeavyConvex);
        rockDebugVerboseLogging = ini.GetBoolValue(SECTION, "bDebugVerboseLogging", rockDebugVerboseLogging);
        rockDebugGrabFrameLogging = ini.GetBoolValue(SECTION, "bDebugGrabFrameLogging", rockDebugGrabFrameLogging);
        rockDebugHandTransformParity = ini.GetBoolValue(SECTION, "bDebugHandTransformParity", rockDebugHandTransformParity);
        rockHandFrameSource = static_cast<int>(ini.GetLongValue(SECTION, "iHandFrameSource", rockHandFrameSource));
        rockHandFrameSwapWands = ini.GetBoolValue(SECTION, "bHandFrameSwapWands", rockHandFrameSwapWands);

        rockNearDetectionRange = static_cast<float>(ini.GetDoubleValue(SECTION, "fNearDetectionRange", rockNearDetectionRange));
        rockFarDetectionRange = static_cast<float>(ini.GetDoubleValue(SECTION, "fFarDetectionRange", rockFarDetectionRange));

        rockGrabLinearTau = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearTau", rockGrabLinearTau));
        rockGrabLinearDamping = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearDamping", rockGrabLinearDamping));
        rockGrabLinearProportionalRecovery = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearProportionalRecovery", rockGrabLinearProportionalRecovery));
        rockGrabLinearConstantRecovery = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearConstantRecovery", rockGrabLinearConstantRecovery));

        rockGrabAngularTau = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularTau", rockGrabAngularTau));
        rockGrabAngularDamping = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularDamping", rockGrabAngularDamping));
        rockGrabAngularProportionalRecovery = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularProportionalRecovery", rockGrabAngularProportionalRecovery));
        rockGrabAngularConstantRecovery = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularConstantRecovery", rockGrabAngularConstantRecovery));

        rockGrabConstraintMaxForce = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabConstraintMaxForce", rockGrabConstraintMaxForce));
        rockGrabAngularToLinearForceRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularToLinearForceRatio", rockGrabAngularToLinearForceRatio));
        rockGrabMaxForceToMassRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxForceToMassRatio", rockGrabMaxForceToMassRatio));
        rockGrabFadeInStartAngularRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFadeInStartAngularRatio", rockGrabFadeInStartAngularRatio));

        rockGrabForceFadeInTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabForceFadeInTime", rockGrabForceFadeInTime));
        rockGrabTauMin = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauMin", rockGrabTauMin));
        rockGrabTauMax = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauMax", rockGrabTauMax));
        rockGrabTauLerpSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauLerpSpeed", rockGrabTauLerpSpeed));
        rockGrabCloseThreshold = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabCloseThreshold", rockGrabCloseThreshold));
        rockGrabFarThreshold = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFarThreshold", rockGrabFarThreshold));

        rockGrabMaxInertiaRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxInertiaRatio", rockGrabMaxInertiaRatio));

        rockGrabMaxDeviation = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviation", rockGrabMaxDeviation));
        rockGrabMaxDeviationTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviationTime", rockGrabMaxDeviationTime));
        rockGrabButtonID = static_cast<int>(ini.GetLongValue(SECTION, "iGrabButtonID", rockGrabButtonID));
        rockThrowVelocityMultiplier = static_cast<float>(ini.GetDoubleValue(SECTION, "fThrowVelocityMultiplier", rockThrowVelocityMultiplier));
        rockGrabVelocityDamping = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabVelocityDamping", rockGrabVelocityDamping));

        readVec3("fGrabPivotAOffsetHandspaceX", "fGrabPivotAOffsetHandspaceY", "fGrabPivotAOffsetHandspaceZ", rockGrabPivotAOffsetHandspace);
        rockReverseGrabPivotAOffset = ini.GetBoolValue(SECTION, "bReverseGrabPivotAOffset", rockReverseGrabPivotAOffset);

        rockGrabLerpSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpSpeed", rockGrabLerpSpeed));
        rockGrabLerpAngularSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpAngularSpeed", rockGrabLerpAngularSpeed));
        rockGrabLerpMaxTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpMaxTime", rockGrabLerpMaxTime));

        rockMaxLinearVelocity = static_cast<float>(ini.GetDoubleValue(SECTION, "fMaxLinearVelocity", rockMaxLinearVelocity));
        rockMaxAngularVelocity = static_cast<float>(ini.GetDoubleValue(SECTION, "fMaxAngularVelocity", rockMaxAngularVelocity));

        rockCharControllerRadiusScale = static_cast<float>(ini.GetDoubleValue(SECTION, "fCharControllerRadiusScale", rockCharControllerRadiusScale));
        rockCollideWithCharControllers = ini.GetBoolValue(SECTION, "bCollideWithCharControllers", rockCollideWithCharControllers);
    }

    void RockConfig::load()
    {
        _iniFilePath = resolveIniPath();
        ROCK_LOG_INFO(Config, "Loading ROCK config from: {}", _iniFilePath);

        f4cf::common::createDirDeep(_iniFilePath);

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

        startFileWatch();
    }

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

    void RockConfig::processPendingReload()
    {
        if (!_reloadPending.exchange(false, std::memory_order_acq_rel)) {
            return;
        }

        ROCK_LOG_INFO(Config, "ROCK.ini change detected, reloading on frame thread...");
        reload();

        for (const auto& [key, subscriber] : _onConfigChangedSubscribers) {
            ROCK_LOG_INFO(Config, "Notify config change subscriber '{}'", key);
            subscriber(key);
        }
    }

    void RockConfig::startFileWatch()
    {
        if (_fileWatch) {
            return;
        }
        if (_iniFilePath.empty()) {
            ROCK_LOG_WARN(Config, "Cannot start file watch — INI path not resolved");
            return;
        }

        if (_fileWatchInitThread.joinable()) {
            _fileWatchInitThread.join();
        }

        _fileWatchInitThread = std::thread([this]() {
            ROCK_LOG_INFO(Config, "Starting file watch on '{}'", _iniFilePath);

            _fileWatch = std::make_unique<filewatch::FileWatch<std::string>>(_iniFilePath, [this](const std::string&, const filewatch::Event changeType) {
                if (changeType != filewatch::Event::modified) {
                    return;
                }

                constexpr auto delay = std::chrono::milliseconds(200);

                auto prevWriteTime = _lastIniFileWriteTime.load();
                std::error_code ec;
                const auto writeTime = std::filesystem::last_write_time(_iniFilePath, ec);
                if (ec || !_lastIniFileWriteTime.compare_exchange_strong(prevWriteTime, writeTime) || writeTime - prevWriteTime < delay) {
                    return;
                }

                bool expected = true;
                if (_ignoreNextIniFileChange.compare_exchange_strong(expected, false)) {
                    return;
                }

                auto now = std::filesystem::file_time_type::clock::now();
                auto lastEventTime = _lastIniFileWriteTime.load();
                while (now - lastEventTime < delay) {
                    std::this_thread::sleep_for(std::max(std::chrono::milliseconds(0), std::chrono::duration_cast<std::chrono::milliseconds>(delay - (now - lastEventTime))));
                    now = std::filesystem::file_time_type::clock::now();
                    lastEventTime = _lastIniFileWriteTime.load();
                }

                _reloadPending.store(true, std::memory_order_release);
            });
        });

        _fileWatchInitThread.join();
    }

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

    void RockConfig::subscribeForConfigChanged(const std::string& key, std::function<void(const std::string&)> callback) { _onConfigChangedSubscribers[key] = std::move(callback); }

    void RockConfig::unsubscribeFromConfigChanged(const std::string& key) { _onConfigChangedSubscribers.erase(key); }
}
