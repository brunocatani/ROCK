#pragma once

#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "RE/NetImmerse/NiPoint.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <SimpleIni.h>
#include <thomasmonkman-filewatch/FileWatch.hpp>

namespace frik::rock
{
    class RockConfig
    {
    public:
        ~RockConfig() { stopFileWatch(); }

        void load();

        void reload();

        void processPendingReload();

        void stopFileWatch();

        void subscribeForConfigChanged(const std::string& key, std::function<void(const std::string&)> callback);
        void unsubscribeFromConfigChanged(const std::string& key);

        void suppressNextFileWatchReload() { _ignoreNextIniFileChange.store(true); }

        bool rockEnabled = true;

        float rockHandCollisionHalfExtentX = 0.09f;
        float rockHandCollisionHalfExtentY = 0.015f;
        float rockHandCollisionHalfExtentZ = 0.05f;

        float rockHandCollisionOffsetX = 0.0f;
        float rockHandCollisionOffsetY = 0.086f;
        float rockHandCollisionOffsetZ = -0.005f;
        float rockHandCollisionBoxRadius = 0.0f;

        RE::NiPoint3 rockHandCollisionOffsetHandspace = RE::NiPoint3(0.086f, -0.005f, 0.0f);

        float rockPalmOffsetForward = 6.0f;
        float rockPalmOffsetUp = -2.4f;
        float rockPalmOffsetRight = 0.0f;

        RE::NiPoint3 rockPalmPositionHandspace = RE::NiPoint3(6.0f, -2.4f, 0.0f);
        RE::NiPoint3 rockPalmNormalHandspace = RE::NiPoint3(0.261f, -0.965f, -0.018f);
        RE::NiPoint3 rockPointingVectorHandspace = RE::NiPoint3(1.0f, 0.0f, 0.0f);

        bool rockWeaponCollisionEnabled = false;

        std::uint32_t rockHighlightShaderFormID = 0x00249733;
        bool rockHighlightEnabled = true;

        bool rockDebugShowColliders = false;
        bool rockDebugShowPalmBasis = false;
        int rockDebugColliderShape = 4;
        bool rockDebugVerboseLogging = true;
        bool rockDebugGrabFrameLogging = true;
        bool rockDebugHandTransformParity = false;

        float rockNearDetectionRange = 25.0f;
        float rockFarDetectionRange = 350.0f;

        float rockGrabLinearTau = 0.03f;
        float rockGrabLinearDamping = 1.0f;
        float rockGrabLinearProportionalRecovery = 4.1f;
        float rockGrabLinearConstantRecovery = 2.1f;

        float rockGrabAngularTau = 0.03f;
        float rockGrabAngularDamping = 0.8f;
        float rockGrabAngularProportionalRecovery = 4.1f;
        float rockGrabAngularConstantRecovery = 2.1f;

        float rockGrabConstraintMaxForce = 2000.0f;
        float rockGrabAngularToLinearForceRatio = 12.5f;
        float rockGrabMaxForceToMassRatio = 500.0f;
        float rockGrabFadeInStartAngularRatio = 100.0f;

        float rockGrabForceFadeInTime = 0.1f;
        float rockGrabTauMin = 0.01f;
        float rockGrabTauMax = 0.8f;
        float rockGrabTauLerpSpeed = 0.5f;
        float rockGrabCloseThreshold = 2.0f;
        float rockGrabFarThreshold = 15.0f;

        float rockGrabMaxInertiaRatio = 10.0f;

        float rockGrabMaxDeviation = 50.0f;
        float rockGrabMaxDeviationTime = 2.0f;
        int rockGrabButtonID = 2;
        float rockThrowVelocityMultiplier = 1.5f;
        float rockGrabVelocityDamping = 0.1f;

        float rockGrabOffsetForward = 0.0f;
        float rockGrabOffsetUp = 0.0f;
        float rockGrabOffsetRight = 0.0f;

        float rockGrabLerpSpeed = 300.0f;
        float rockGrabLerpAngularSpeed = 10.0f;
        float rockGrabLerpMaxTime = 0.5f;

        float rockMaxLinearVelocity = 200.0f;
        float rockMaxAngularVelocity = 500.0f;

        float rockCharControllerRadiusScale = 0.7f;
        bool rockCollideWithCharControllers = false;

    private:
        void resetToDefaults();

        void readValuesFromIni(CSimpleIniA& ini);

        void startFileWatch();

        std::string _iniFilePath;

        std::unique_ptr<filewatch::FileWatch<std::string>> _fileWatch;

        std::atomic<std::filesystem::file_time_type> _lastIniFileWriteTime;

        std::unordered_map<std::string, std::function<void(const std::string&)>> _onConfigChangedSubscribers;

        std::atomic<bool> _ignoreNextIniFileChange = false;

        std::atomic<bool> _reloadPending = false;

        std::thread _fileWatchInitThread;
    };

    inline RockConfig g_rockConfig;
}
