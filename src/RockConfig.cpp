

#include "RockConfig.h"

#include <ShlObj.h>
#include <SimpleIni.h>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <thread>

#include "common/CommonUtils.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/RockLoggingPolicy.h"
#include "resources.h"

namespace
{

    constexpr auto SECTION = "PhysicsInteraction";
    constexpr auto DEBUG_SECTION = "Debug";
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

        rockLogLevel = logging_policy::DefaultLogLevel;
        rockLogPattern = logging_policy::DefaultLogPattern;
        rockLogSampleMilliseconds = logging_policy::DefaultLogSampleMilliseconds;

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
        rockWeaponCollisionMaxLinearVelocity = 50.0f;
        rockWeaponCollisionMaxAngularVelocity = 100.0f;
        rockWeaponInteractionProbeRadius = 12.0f;
        rockOneHandedMeshPrimaryGripAuthorityEnabled = false;

        rockReloadUseVanillaStageObserver = true;
        rockReloadRequirePhysicalCompletion = true;
        rockReloadAllowStageFallbacks = true;
        rockReloadObserverStaleFrameTimeout = 180;
        rockReloadDebugStageLogging = false;

        rockNativeMeleeSuppressionEnabled = true;
        rockNativeMeleeSuppressWeaponSwing = true;
        rockNativeMeleeSuppressHitFrame = true;
        rockNativeMeleeDebugLogging = false;

        rockHighlightShaderFormID = 0x00249733;
        rockHighlightEnabled = true;

        rockDebugShowColliders = false;
        rockDebugShowTargetColliders = false;
        rockDebugShowHandAxes = false;
        rockDebugShowGrabPivots = false;
        rockDebugShowGrabFingerProbes = false;
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
        rockDebugShowGrabNotifications = false;
        rockDebugShowWeaponNotifications = false;
        rockDebugHandTransformParity = false;
        rockHandFrameSource = 3;
        rockHandFrameSwapWands = false;

        rockNearDetectionRange = 25.0f;
        rockFarDetectionRange = 350.0f;
        rockNearCastRadiusGameUnits = 6.0f;
        rockNearCastDistanceGameUnits = 25.0f;
        rockFarCastRadiusGameUnits = 21.0f;
        rockSelectionShapeCastFilterInfo = selection_query_policy::kDefaultShapeCastFilterInfo;
        rockFarClipRayFilterInfo = selection_query_policy::kDefaultFarClipRayFilterInfo;
        rockPullApplyVelocityTime = 0.2f;
        rockPullTrackHandTime = 0.1f;
        rockPullDestinationZOffsetHavok = 0.01f;
        rockPullDurationA = 0.715619f;
        rockPullDurationB = -0.415619f;
        rockPullDurationC = 0.656256f;
        rockPullMaxVelocityHavok = 10.0f;
        rockPullAutoGrabDistanceGameUnits = 18.0f;
        rockObjectPhysicsTreeMaxDepth = 12;
        rockDynamicPushAssistEnabled = true;
        rockDynamicPushMinSpeed = 0.35f;
        rockDynamicPushMaxImpulse = 2.0f;
        rockDynamicPushCooldownSeconds = 0.08f;

        rockGrabLinearTau = 0.03f;
        rockGrabLinearDamping = 0.8f;
        rockGrabLinearProportionalRecovery = 2.0f;
        rockGrabLinearConstantRecovery = 1.0f;

        rockGrabAngularTau = 0.03f;
        rockGrabAngularDamping = 0.8f;
        rockGrabAngularProportionalRecovery = 2.0f;
        rockGrabAngularConstantRecovery = 1.0f;

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
        rockGrabAdaptiveMotorEnabled = true;
        rockGrabAdaptivePositionFullError = 20.0f;
        rockGrabAdaptiveRotationFullError = 60.0f;
        rockGrabAdaptiveMaxForceMultiplier = 4.0f;

        rockGrabMaxInertiaRatio = 10.0f;

        rockGrabMaxDeviation = 50.0f;
        rockGrabMaxDeviationTime = 2.0f;
        rockGrabButtonID = 2;
        rockThrowVelocityMultiplier = 1.5f;
        rockGrabVelocityDamping = 0.25f;
        rockGrabPlayerSpaceCompensation = true;
        rockGrabPlayerSpaceWarpDistance = 35.0f;
        rockGrabResidualVelocityDamping = true;
        rockGrabHandLerpEnabled = true;
        rockGrabHandLerpTimeMin = 0.10f;
        rockGrabHandLerpTimeMax = 0.20f;
        rockGrabHandLerpMinDistance = 7.0f;
        rockGrabHandLerpMaxDistance = 14.0f;
        rockGrabMeshFingerPoseEnabled = true;
        rockGrabMeshJointPoseEnabled = true;
        rockGrabFingerPoseUpdateInterval = 3;
        rockGrabFingerMinValue = 0.2f;
        rockGrabFingerPoseSmoothingSpeed = 14.0f;

        rockGrabPivotAOffsetHandspace = RE::NiPoint3(0.0f, 0.0f, 0.0f);
        rockReverseGrabPivotAOffset = false;

        rockGrabLerpSpeed = 300.0f;
        rockGrabLerpAngularSpeed = 360.0f;
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
        auto readHexFilter = [&](const char* key, std::uint32_t currentValue, std::uint32_t fallback) {
            char hexBuf[16] = {};
            snprintf(hexBuf, sizeof(hexBuf), "%08X", currentValue);
            const char* hexStr = ini.GetValue(SECTION, key, hexBuf);
            if (!hexStr || !hexStr[0]) {
                return selection_query_policy::sanitizeFilterInfo(currentValue, fallback);
            }

            return selection_query_policy::sanitizeFilterInfo(static_cast<std::uint32_t>(std::strtoul(hexStr, nullptr, 16)), fallback);
        };

        rockLogLevel = logging_policy::clampLogLevel(static_cast<int>(ini.GetLongValue(DEBUG_SECTION, "iLogLevel", rockLogLevel)));
        rockLogPattern = ini.GetValue(DEBUG_SECTION, "sLogPattern", rockLogPattern.c_str());
        if (rockLogPattern.empty()) {
            rockLogPattern = logging_policy::DefaultLogPattern;
        }
        rockLogSampleMilliseconds =
            logging_policy::sanitizeSampleMilliseconds(static_cast<int>(ini.GetLongValue(DEBUG_SECTION, "iLogSampleMilliseconds", rockLogSampleMilliseconds)));
        logger::setLogLevelAndPattern(rockLogLevel, rockLogPattern);

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
        rockWeaponCollisionMaxLinearVelocity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionMaxLinearVelocity", rockWeaponCollisionMaxLinearVelocity));
        rockWeaponCollisionMaxAngularVelocity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionMaxAngularVelocity", rockWeaponCollisionMaxAngularVelocity));
        rockWeaponInteractionProbeRadius = static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponInteractionProbeRadius", rockWeaponInteractionProbeRadius));
        rockOneHandedMeshPrimaryGripAuthorityEnabled =
            ini.GetBoolValue(SECTION, "bOneHandedMeshPrimaryGripAuthorityEnabled", rockOneHandedMeshPrimaryGripAuthorityEnabled);

        rockReloadUseVanillaStageObserver = ini.GetBoolValue(SECTION, "bReloadUseVanillaStageObserver", rockReloadUseVanillaStageObserver);
        rockReloadRequirePhysicalCompletion = ini.GetBoolValue(SECTION, "bReloadRequirePhysicalCompletion", rockReloadRequirePhysicalCompletion);
        rockReloadAllowStageFallbacks = ini.GetBoolValue(SECTION, "bReloadAllowStageFallbacks", rockReloadAllowStageFallbacks);
        rockReloadObserverStaleFrameTimeout = static_cast<int>(ini.GetLongValue(SECTION, "iReloadObserverStaleFrameTimeout", rockReloadObserverStaleFrameTimeout));
        rockReloadDebugStageLogging = ini.GetBoolValue(SECTION, "bReloadDebugStageLogging", rockReloadDebugStageLogging);

        rockNativeMeleeSuppressionEnabled = ini.GetBoolValue(SECTION, "bNativeMeleeSuppressionEnabled", rockNativeMeleeSuppressionEnabled);
        rockNativeMeleeSuppressWeaponSwing = ini.GetBoolValue(SECTION, "bNativeMeleeSuppressWeaponSwing", rockNativeMeleeSuppressWeaponSwing);
        rockNativeMeleeSuppressHitFrame = ini.GetBoolValue(SECTION, "bNativeMeleeSuppressHitFrame", rockNativeMeleeSuppressHitFrame);
        rockNativeMeleeDebugLogging = ini.GetBoolValue(SECTION, "bNativeMeleeDebugLogging", rockNativeMeleeDebugLogging);

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
        rockDebugShowGrabFingerProbes = ini.GetBoolValue(SECTION, "bDebugShowGrabFingerProbes", rockDebugShowGrabFingerProbes);
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
        rockDebugShowGrabNotifications = ini.GetBoolValue(SECTION, "bDebugShowGrabNotifications", rockDebugShowGrabNotifications);
        rockDebugShowWeaponNotifications = ini.GetBoolValue(SECTION, "bDebugShowWeaponNotifications", rockDebugShowWeaponNotifications);
        rockDebugHandTransformParity = ini.GetBoolValue(SECTION, "bDebugHandTransformParity", rockDebugHandTransformParity);
        rockHandFrameSource = static_cast<int>(ini.GetLongValue(SECTION, "iHandFrameSource", rockHandFrameSource));
        rockHandFrameSwapWands = ini.GetBoolValue(SECTION, "bHandFrameSwapWands", rockHandFrameSwapWands);

        rockNearDetectionRange = static_cast<float>(ini.GetDoubleValue(SECTION, "fNearDetectionRange", rockNearDetectionRange));
        rockFarDetectionRange = static_cast<float>(ini.GetDoubleValue(SECTION, "fFarDetectionRange", rockFarDetectionRange));
        rockNearCastRadiusGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fNearCastRadiusGameUnits", rockNearCastRadiusGameUnits));
        rockNearCastDistanceGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fNearCastDistanceGameUnits", rockNearDetectionRange));
        rockFarCastRadiusGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fFarCastRadiusGameUnits", rockFarCastRadiusGameUnits));
        rockSelectionShapeCastFilterInfo =
            readHexFilter("sSelectionShapeCastFilterInfo", rockSelectionShapeCastFilterInfo, selection_query_policy::kDefaultShapeCastFilterInfo);
        rockFarClipRayFilterInfo = readHexFilter("sFarClipRayFilterInfo", rockFarClipRayFilterInfo, selection_query_policy::kDefaultFarClipRayFilterInfo);
        rockPullApplyVelocityTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullApplyVelocityTime", rockPullApplyVelocityTime));
        rockPullTrackHandTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullTrackHandTime", rockPullTrackHandTime));
        rockPullDestinationZOffsetHavok = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullDestinationZOffsetHavok", rockPullDestinationZOffsetHavok));
        rockPullDurationA = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullDurationA", rockPullDurationA));
        rockPullDurationB = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullDurationB", rockPullDurationB));
        rockPullDurationC = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullDurationC", rockPullDurationC));
        rockPullMaxVelocityHavok = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullMaxVelocityHavok", rockPullMaxVelocityHavok));
        rockPullAutoGrabDistanceGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullAutoGrabDistanceGameUnits", rockPullAutoGrabDistanceGameUnits));
        rockObjectPhysicsTreeMaxDepth = static_cast<int>(ini.GetLongValue(SECTION, "iObjectPhysicsTreeMaxDepth", rockObjectPhysicsTreeMaxDepth));
        rockDynamicPushAssistEnabled = ini.GetBoolValue(SECTION, "bDynamicPushAssistEnabled", rockDynamicPushAssistEnabled);
        rockDynamicPushMinSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fDynamicPushMinSpeed", rockDynamicPushMinSpeed));
        rockDynamicPushMaxImpulse = static_cast<float>(ini.GetDoubleValue(SECTION, "fDynamicPushMaxImpulse", rockDynamicPushMaxImpulse));
        rockDynamicPushCooldownSeconds = static_cast<float>(ini.GetDoubleValue(SECTION, "fDynamicPushCooldownSeconds", rockDynamicPushCooldownSeconds));

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
        rockGrabAdaptiveMotorEnabled = ini.GetBoolValue(SECTION, "bGrabAdaptiveMotorEnabled", rockGrabAdaptiveMotorEnabled);
        rockGrabAdaptivePositionFullError = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAdaptivePositionFullError", rockGrabAdaptivePositionFullError));
        rockGrabAdaptiveRotationFullError = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAdaptiveRotationFullError", rockGrabAdaptiveRotationFullError));
        rockGrabAdaptiveMaxForceMultiplier = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAdaptiveMaxForceMultiplier", rockGrabAdaptiveMaxForceMultiplier));

        rockGrabMaxInertiaRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxInertiaRatio", rockGrabMaxInertiaRatio));

        rockGrabMaxDeviation = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviation", rockGrabMaxDeviation));
        rockGrabMaxDeviationTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviationTime", rockGrabMaxDeviationTime));
        rockGrabButtonID = static_cast<int>(ini.GetLongValue(SECTION, "iGrabButtonID", rockGrabButtonID));
        rockThrowVelocityMultiplier = static_cast<float>(ini.GetDoubleValue(SECTION, "fThrowVelocityMultiplier", rockThrowVelocityMultiplier));
        rockGrabVelocityDamping = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabVelocityDamping", rockGrabVelocityDamping));
        rockGrabPlayerSpaceCompensation = ini.GetBoolValue(SECTION, "bGrabPlayerSpaceCompensation", rockGrabPlayerSpaceCompensation);
        rockGrabPlayerSpaceWarpDistance = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabPlayerSpaceWarpDistance", rockGrabPlayerSpaceWarpDistance));
        rockGrabResidualVelocityDamping = ini.GetBoolValue(SECTION, "bGrabResidualVelocityDamping", rockGrabResidualVelocityDamping);
        rockGrabHandLerpEnabled = ini.GetBoolValue(SECTION, "bGrabHandLerpEnabled", rockGrabHandLerpEnabled);
        rockGrabHandLerpTimeMin = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabHandLerpTimeMin", rockGrabHandLerpTimeMin));
        rockGrabHandLerpTimeMax = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabHandLerpTimeMax", rockGrabHandLerpTimeMax));
        rockGrabHandLerpMinDistance = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabHandLerpMinDistance", rockGrabHandLerpMinDistance));
        rockGrabHandLerpMaxDistance = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabHandLerpMaxDistance", rockGrabHandLerpMaxDistance));
        rockGrabMeshFingerPoseEnabled = ini.GetBoolValue(SECTION, "bGrabMeshFingerPoseEnabled", rockGrabMeshFingerPoseEnabled);
        rockGrabMeshJointPoseEnabled = ini.GetBoolValue(SECTION, "bGrabMeshJointPoseEnabled", rockGrabMeshJointPoseEnabled);
        rockGrabFingerPoseUpdateInterval = static_cast<int>(ini.GetLongValue(SECTION, "iGrabFingerPoseUpdateInterval", rockGrabFingerPoseUpdateInterval));
        rockGrabFingerMinValue = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerMinValue", rockGrabFingerMinValue));
        rockGrabFingerPoseSmoothingSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerPoseSmoothingSpeed", rockGrabFingerPoseSmoothingSpeed));

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

        ROCK_LOG_INFO(Config,
            "ROCK config loaded (rockEnabled={}, logLevel={} {}, sample={}ms)",
            rockEnabled,
            rockLogLevel,
            logging_policy::logLevelName(rockLogLevel),
            rockLogSampleMilliseconds);

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
        ROCK_LOG_INFO(Config,
            "ROCK config reloaded (rockEnabled={}, logLevel={} {}, sample={}ms)",
            rockEnabled,
            rockLogLevel,
            logging_policy::logLevelName(rockLogLevel),
            rockLogSampleMilliseconds);
    }

    void RockConfig::processPendingReload()
    {
        if (!_reloadPending.exchange(false, std::memory_order_acq_rel)) {
            return;
        }

        ROCK_LOG_INFO(Config, "ROCK.ini change detected, reloading on frame thread...");
        reload();

        for (const auto& [key, subscriber] : _onConfigChangedSubscribers) {
            ROCK_LOG_DEBUG(Config, "Notify config change subscriber '{}'", key);
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
            ROCK_LOG_DEBUG(Config, "Starting file watch on '{}'", _iniFilePath);

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
            ROCK_LOG_DEBUG(Config, "Stopping file watch on ROCK.ini");
            _fileWatch.reset();
        }
    }

    void RockConfig::subscribeForConfigChanged(const std::string& key, std::function<void(const std::string&)> callback) { _onConfigChangedSubscribers[key] = std::move(callback); }

    void RockConfig::unsubscribeFromConfigChanged(const std::string& key) { _onConfigChangedSubscribers.erase(key); }
}
