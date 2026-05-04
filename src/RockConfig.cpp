

#include "RockConfig.h"

#include <ShlObj.h>
#include <SimpleIni.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <thread>

#include "common/CommonUtils.h"
#include "physics-interaction/grab/GrabNodeNamePolicy.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/grab/NearbyGrabDamping.h"
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

        rockInputRemapEnabled = true;
        rockRightWeaponReadyButtonID = 32;
        rockSuppressRightGrabGameInput = true;
        rockSuppressRightFavoritesGameInput = true;
        rockSuppressNativeReadyWeaponAutoReady = true;

        rockLogLevel = logging_policy::DefaultLogLevel;
        rockLogPattern = logging_policy::DefaultLogPattern;
        rockLogSampleMilliseconds = logging_policy::DefaultLogSampleMilliseconds;

        rockPalmNormalHandspace = kDefaultPalmNormalHandspace;
        rockPointingVectorHandspace = RE::NiPoint3(0.0f, 0.0f, 1.0f);
        rockReversePalmNormal = true;
        rockReverseFarGrabNormal = false;

        rockWeaponCollisionEnabled = false;
        rockWeaponCollisionBlocksProjectiles = false;
        rockWeaponCollisionBlocksSpells = false;
        rockWeaponCollisionGroupingMode = weapon_collision_grouping_policy::kDefaultWeaponCollisionGroupingMode;
        rockWeaponCollisionConvexRadius = 0.01f;
        rockWeaponCollisionPointDedupGrid = 0.002f;
        rockWeaponCollisionMaxLinearVelocity = 50.0f;
        rockWeaponCollisionMaxAngularVelocity = 100.0f;
        rockWeaponInteractionProbeRadius = 12.0f;
        rockVisualOnlySidearmSupportGripEnabled = true;

        rockNativeMeleeSuppressionEnabled = true;
        rockNativeMeleeFullSuppression = true;
        rockNativeMeleeSuppressWeaponSwing = true;
        rockNativeMeleeSuppressHitFrame = true;
        rockNativeMeleeDebugLogging = false;

        rockHighlightEnabled = true;

        rockDebugShowColliders = false;
        rockDebugShowTargetColliders = false;
        rockDebugShowHandAxes = false;
        rockDebugShowGrabPivots = false;
        rockDebugShowGrabSurfaceFrame = false;
        rockDebugDrawGrabContactPatch = true;
        rockDebugShowGrabFingerProbes = false;
        rockDebugShowPalmVectors = false;
        rockDebugDrawHandColliders = true;
        rockDebugDrawHandBoneColliders = true;
        rockDebugDrawHandBoneContacts = true;
        rockDebugMaxHandBoneBodiesDrawn = 48;
        rockDebugDrawWeaponColliders = true;
        rockDebugMaxWeaponBodiesDrawn = 6;
        rockDebugMaxShapeGenerationsPerFrame = 2;
        rockDebugMaxConvexSupportVertices = 64;
        rockDebugUseBoundsForHeavyConvex = true;
        rockDebugVerboseLogging = false;
        rockDebugGrabFrameLogging = false;
        rockDebugGrabTransformTelemetry = false;
        rockDebugGrabTransformTelemetryText = true;
        rockDebugGrabTransformTelemetryAxes = true;
        rockDebugGrabTransformTelemetryLogIntervalFrames = 10;
        rockDebugGrabTransformTelemetryTextMode = 0;
        rockDebugShowGrabNotifications = false;
        rockDebugShowWeaponNotifications = false;
        rockDebugHandTransformParity = false;
        rockDebugWorldObjectOriginDiagnostics = false;
        rockDebugWorldObjectOriginLogIntervalFrames = 120;
        rockDebugWorldObjectOriginMismatchWarnGameUnits = 5.0f;
        rockDebugShowRootFlattenedFingerSkeletonMarkers = false;
        rockDebugShowSkeletonBoneVisualizer = true;
        rockDebugDrawSkeletonBoneAxes = true;
        rockDebugLogSkeletonBones = true;
        rockDebugSkeletonBoneMode = 1;
        rockDebugSkeletonBoneSource = 1;
        rockDebugMaxSkeletonBonesDrawn = 256;
        rockDebugMaxSkeletonBoneAxesDrawn = 80;
        rockDebugSkeletonBoneLogIntervalFrames = 120;
        rockDebugLogSkeletonBoneTruncation = true;
        rockDebugRootFlattenedFingerSkeletonMarkerSize = 1.4f;
        rockDebugSkeletonBonePointSize = 1.4f;
        rockDebugSkeletonBoneAxisLength = 4.0f;
        rockDebugSkeletonBoneLogFilter = "RArm_Hand,LArm_Hand,RArm_Finger23,LArm_Finger23,Chest,Pelvis";
        rockDebugSkeletonAxisBoneFilter = "";

        rockHandColliderRuntimeMode = 1;
        rockHandBoneCollidersRequirePalmAnchor = true;
        rockHandBoneCollidersRequireAllFingerBones = false;
        rockHandBoneColliderMaxLinearVelocity = 200.0f;
        rockHandBoneColliderMaxAngularVelocity = 500.0f;

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
        rockGrabAdaptiveMotorEnabled = false;
        rockGrabAdaptivePositionFullError = 20.0f;
        rockGrabAdaptiveRotationFullError = 60.0f;
        rockGrabAdaptiveMaxForceMultiplier = 4.0f;

        rockGrabMaxInertiaRatio = 10.0f;

        rockGrabMaxDeviation = 50.0f;
        rockGrabMaxDeviationTime = 2.0f;
        rockGrabButtonID = 2;
        rockThrowVelocityMultiplier = 1.5f;
        rockGrabReleaseHandCollisionDelaySeconds = 0.10f;
        rockGrabVelocityDamping = 0.25f;
        rockGrabPlayerSpaceCompensation = true;
        rockGrabPlayerSpaceWarpDistance = 35.0f;
        rockGrabPlayerSpaceWarpMinRotationDegrees = 0.6f;
        rockGrabPlayerSpaceTransformWarpEnabled = false;
        rockGrabResidualVelocityDamping = true;
        rockGrabNearbyDampingEnabled = true;
        rockGrabNearbyDampingRadius = 90.0f;
        rockGrabNearbyDampingSeconds = 0.35f;
        rockGrabNearbyLinearDamping = 0.65f;
        rockGrabNearbyAngularDamping = 0.85f;
        rockGrabObjectVisualHandAuthorityEnabled = false;
        rockGrabUseBoneDerivedPalmPivot = true;
        rockGrabUseSemanticFingerContactPivot = true;
        rockGrabMultiFingerContactValidationEnabled = true;
        rockGrabContactQualityMode = 1;
        rockGrabMinFingerContactGroups = 3;
        rockGrabMinFingerContactSpreadGameUnits = 1.0f;
        rockGrabFingerContactMeshSnapMaxDistanceGameUnits = 10.0f;
        rockGrabOppositionFrameEnabled = true;
        rockGrabOppositionContactMaxAgeFrames = 5;
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
        rockGrabMeshLocalTransformPoseEnabled = true;
        rockGrabFingerLocalTransformSmoothingSpeed = 14.0f;
        rockGrabFingerLocalTransformMaxCorrectionDegrees = 35.0f;
        rockGrabFingerSurfaceAimStrength = 0.75f;
        rockGrabThumbOppositionStrength = 1.0f;
        rockGrabThumbAlternateCurveStrength = 0.65f;
        rockGrabLateralWeight = 0.6f;
        rockGrabDirectionalWeight = 0.4f;
        rockGrabMaxTriangleDistance = 100.0f;
        rockGrabOrientationMode = 0;
        rockGrabMeshContactOnly = true;
        rockGrabRequireMeshContact = true;
        rockGrabSurfaceFrameMinConfidence = 0.2f;
        rockGrabSurfaceCapNormalDotThreshold = 0.85f;
        rockGrabSurfacePreserveRollForCaps = true;
        rockGrabContactPatchEnabled = true;
        rockGrabContactPatchProbeCount = 5;
        rockGrabContactPatchProbeSpacingGameUnits = 3.0f;
        rockGrabContactPatchProbeRadiusGameUnits = 2.0f;
        rockGrabContactPatchMeshSnapMaxDistanceGameUnits = 6.0f;
        rockGrabContactPatchMaxNormalAngleDegrees = 35.0f;
        rockGrabAlignmentMaxPivotToSurfaceDistance = 8.0f;
        rockGrabAlignmentMaxSelectionToMeshDistance = 8.0f;
        rockGrabAlignmentRequireResolvedOwnerMatch = true;
        rockGrabAlignmentUseHandParallelTangent = true;
        rockGrabNodeAnchorsEnabled = true;
        rockPrintGrabNodeInfo = false;
        rockGrabNodeNameRight = grab_node_name_policy::defaultGrabNodeName(false);
        rockGrabNodeNameLeft = grab_node_name_policy::defaultGrabNodeName(true);
        rockSelectedCloseFingerCurlEnabled = true;
        rockSelectedCloseFingerAnimMaxHandSpeed = 0.9f;
        rockSelectedCloseFingerAnimValue = 0.9f;
        rockPulledAngularDamping = 8.0f;

        rockRightGrabPivotAHandspace = RE::NiPoint3(6.0f, 0.2f, -2.0f);
        rockLeftGrabPivotAHandspace = RE::NiPoint3(6.0f, -0.2f, -2.0f);

        rockGrabLerpSpeed = 300.0f;
        rockGrabLerpAngularSpeed = 360.0f;
        rockGrabLerpMaxTime = 0.5f;

    }

    void RockConfig::readValuesFromIni(CSimpleIniA& ini)
    {
        auto readVec3 = [&](const char* keyX, const char* keyY, const char* keyZ, RE::NiPoint3& value) {
            value.x = static_cast<float>(ini.GetDoubleValue(SECTION, keyX, value.x));
            value.y = static_cast<float>(ini.GetDoubleValue(SECTION, keyY, value.y));
            value.z = static_cast<float>(ini.GetDoubleValue(SECTION, keyZ, value.z));
        };
        auto readOptionalVec3 = [&](const char* keyX, const char* keyY, const char* keyZ, RE::NiPoint3& value) {
            const bool hasAny = ini.GetValue(SECTION, keyX, nullptr) || ini.GetValue(SECTION, keyY, nullptr) || ini.GetValue(SECTION, keyZ, nullptr);
            if (!hasAny) {
                return false;
            }

            readVec3(keyX, keyY, keyZ, value);
            return true;
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
        rockInputRemapEnabled = ini.GetBoolValue(SECTION, "bInputRemapEnabled", rockInputRemapEnabled);
        rockRightWeaponReadyButtonID = static_cast<int>(ini.GetLongValue(SECTION, "iRightWeaponReadyButtonID", rockRightWeaponReadyButtonID));
        if (!input_remap_policy::isValidButtonId(rockRightWeaponReadyButtonID)) {
            ROCK_LOG_WARN(Config, "iRightWeaponReadyButtonID must be 0..63; using 32");
            rockRightWeaponReadyButtonID = 32;
        }
        rockSuppressRightGrabGameInput = ini.GetBoolValue(SECTION, "bSuppressRightGrabGameInput", rockSuppressRightGrabGameInput);
        rockSuppressRightFavoritesGameInput = ini.GetBoolValue(SECTION, "bSuppressRightFavoritesGameInput", rockSuppressRightFavoritesGameInput);
        rockSuppressNativeReadyWeaponAutoReady = ini.GetBoolValue(SECTION, "bSuppressNativeReadyWeaponAutoReady", rockSuppressNativeReadyWeaponAutoReady);

        readVec3("fPalmNormalHandspaceX", "fPalmNormalHandspaceY", "fPalmNormalHandspaceZ", rockPalmNormalHandspace);
        readVec3("fPointingVectorHandspaceX", "fPointingVectorHandspaceY", "fPointingVectorHandspaceZ", rockPointingVectorHandspace);
        rockReversePalmNormal = ini.GetBoolValue(SECTION, "bReversePalmNormal", rockReversePalmNormal);
        rockReverseFarGrabNormal = ini.GetBoolValue(SECTION, "bReverseFarGrabNormal", rockReverseFarGrabNormal);

        rockWeaponCollisionEnabled = ini.GetBoolValue(SECTION, "bWeaponCollisionEnabled", rockWeaponCollisionEnabled);
        rockWeaponCollisionBlocksProjectiles = ini.GetBoolValue(SECTION, "bWeaponCollisionBlocksProjectiles", rockWeaponCollisionBlocksProjectiles);
        rockWeaponCollisionBlocksSpells = ini.GetBoolValue(SECTION, "bWeaponCollisionBlocksSpells", rockWeaponCollisionBlocksSpells);
        rockWeaponCollisionGroupingMode = static_cast<int>(ini.GetLongValue(SECTION, "iWeaponCollisionGroupingMode", rockWeaponCollisionGroupingMode));
        const auto sanitizedWeaponCollisionGroupingMode = weapon_collision_grouping_policy::sanitizeWeaponCollisionGroupingMode(rockWeaponCollisionGroupingMode);
        if (static_cast<int>(sanitizedWeaponCollisionGroupingMode) != rockWeaponCollisionGroupingMode) {
            ROCK_LOG_WARN(Config,
                "Invalid iWeaponCollisionGroupingMode={} - using {}",
                rockWeaponCollisionGroupingMode,
                weapon_collision_grouping_policy::weaponCollisionGroupingModeName(sanitizedWeaponCollisionGroupingMode));
            rockWeaponCollisionGroupingMode = static_cast<int>(sanitizedWeaponCollisionGroupingMode);
        }
        rockWeaponCollisionConvexRadius = static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionConvexRadius", rockWeaponCollisionConvexRadius));
        rockWeaponCollisionPointDedupGrid = static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionPointDedupGrid", rockWeaponCollisionPointDedupGrid));
        rockWeaponCollisionMaxLinearVelocity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionMaxLinearVelocity", rockWeaponCollisionMaxLinearVelocity));
        rockWeaponCollisionMaxAngularVelocity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionMaxAngularVelocity", rockWeaponCollisionMaxAngularVelocity));
        rockWeaponInteractionProbeRadius = static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponInteractionProbeRadius", rockWeaponInteractionProbeRadius));
        rockVisualOnlySidearmSupportGripEnabled =
            ini.GetBoolValue(SECTION, "bVisualOnlySidearmSupportGripEnabled", rockVisualOnlySidearmSupportGripEnabled);

        rockNativeMeleeSuppressionEnabled = ini.GetBoolValue(SECTION, "bNativeMeleeSuppressionEnabled", rockNativeMeleeSuppressionEnabled);
        rockNativeMeleeFullSuppression = ini.GetBoolValue(SECTION, "bNativeMeleeFullSuppression", rockNativeMeleeFullSuppression);
        rockNativeMeleeSuppressWeaponSwing = ini.GetBoolValue(SECTION, "bNativeMeleeSuppressWeaponSwing", rockNativeMeleeSuppressWeaponSwing);
        rockNativeMeleeSuppressHitFrame = ini.GetBoolValue(SECTION, "bNativeMeleeSuppressHitFrame", rockNativeMeleeSuppressHitFrame);
        rockNativeMeleeDebugLogging = ini.GetBoolValue(SECTION, "bNativeMeleeDebugLogging", rockNativeMeleeDebugLogging);

        rockHighlightEnabled = ini.GetBoolValue(SECTION, "bHighlightEnabled", rockHighlightEnabled);

        rockDebugShowColliders = ini.GetBoolValue(SECTION, "bDebugShowColliders", rockDebugShowColliders);
        rockDebugShowTargetColliders = ini.GetBoolValue(SECTION, "bDebugShowTargetColliders", rockDebugShowTargetColliders);
        rockDebugShowHandAxes = ini.GetBoolValue(SECTION, "bDebugShowHandAxes", rockDebugShowHandAxes);
        rockDebugShowGrabPivots = ini.GetBoolValue(SECTION, "bDebugShowGrabPivots", rockDebugShowGrabPivots);
        rockDebugShowGrabSurfaceFrame = ini.GetBoolValue(SECTION, "bDebugShowGrabSurfaceFrame", rockDebugShowGrabSurfaceFrame);
        rockDebugDrawGrabContactPatch = ini.GetBoolValue(SECTION, "bDebugDrawGrabContactPatch", rockDebugDrawGrabContactPatch);
        rockDebugShowGrabFingerProbes = ini.GetBoolValue(SECTION, "bDebugShowGrabFingerProbes", rockDebugShowGrabFingerProbes);
        rockDebugShowPalmVectors = ini.GetBoolValue(SECTION, "bDebugShowPalmVectors", rockDebugShowPalmVectors);
        rockDebugDrawHandColliders = ini.GetBoolValue(SECTION, "bDebugDrawHandColliders", rockDebugDrawHandColliders);
        rockDebugDrawHandBoneColliders = ini.GetBoolValue(SECTION, "bDebugDrawHandBoneColliders", rockDebugDrawHandBoneColliders);
        rockDebugDrawHandBoneContacts = ini.GetBoolValue(SECTION, "bDebugDrawHandBoneContacts", rockDebugDrawHandBoneContacts);
        rockDebugMaxHandBoneBodiesDrawn = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxHandBoneBodiesDrawn", rockDebugMaxHandBoneBodiesDrawn));
        if (rockDebugMaxHandBoneBodiesDrawn < 0) {
            rockDebugMaxHandBoneBodiesDrawn = 0;
        } else if (rockDebugMaxHandBoneBodiesDrawn > 48) {
            rockDebugMaxHandBoneBodiesDrawn = 48;
        }
        rockDebugDrawWeaponColliders = ini.GetBoolValue(SECTION, "bDebugDrawWeaponColliders", rockDebugDrawWeaponColliders);
        rockDebugMaxWeaponBodiesDrawn = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxWeaponBodiesDrawn", rockDebugMaxWeaponBodiesDrawn));
        rockDebugMaxShapeGenerationsPerFrame = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxShapeGenerationsPerFrame", rockDebugMaxShapeGenerationsPerFrame));
        rockDebugMaxConvexSupportVertices = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxConvexSupportVertices", rockDebugMaxConvexSupportVertices));
        rockDebugUseBoundsForHeavyConvex = ini.GetBoolValue(SECTION, "bDebugUseBoundsForHeavyConvex", rockDebugUseBoundsForHeavyConvex);
        rockDebugVerboseLogging = ini.GetBoolValue(SECTION, "bDebugVerboseLogging", rockDebugVerboseLogging);
        rockDebugGrabFrameLogging = ini.GetBoolValue(SECTION, "bDebugGrabFrameLogging", rockDebugGrabFrameLogging);
        rockDebugGrabTransformTelemetry = ini.GetBoolValue(SECTION, "bDebugGrabTransformTelemetry", rockDebugGrabTransformTelemetry);
        rockDebugGrabTransformTelemetryText = ini.GetBoolValue(SECTION, "bDebugGrabTransformTelemetryText", rockDebugGrabTransformTelemetryText);
        rockDebugGrabTransformTelemetryAxes = ini.GetBoolValue(SECTION, "bDebugGrabTransformTelemetryAxes", rockDebugGrabTransformTelemetryAxes);
        rockDebugGrabTransformTelemetryLogIntervalFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iDebugGrabTransformTelemetryLogIntervalFrames", rockDebugGrabTransformTelemetryLogIntervalFrames));
        if (rockDebugGrabTransformTelemetryLogIntervalFrames < 1) {
            rockDebugGrabTransformTelemetryLogIntervalFrames = 1;
        }
        rockDebugGrabTransformTelemetryTextMode =
            static_cast<int>(ini.GetLongValue(SECTION, "iDebugGrabTransformTelemetryTextMode", rockDebugGrabTransformTelemetryTextMode));
        if (rockDebugGrabTransformTelemetryTextMode < 0 || rockDebugGrabTransformTelemetryTextMode > 1) {
            rockDebugGrabTransformTelemetryTextMode = 0;
        }
        rockDebugShowGrabNotifications = ini.GetBoolValue(SECTION, "bDebugShowGrabNotifications", rockDebugShowGrabNotifications);
        rockDebugShowWeaponNotifications = ini.GetBoolValue(SECTION, "bDebugShowWeaponNotifications", rockDebugShowWeaponNotifications);
        rockDebugHandTransformParity = ini.GetBoolValue(SECTION, "bDebugHandTransformParity", rockDebugHandTransformParity);
        rockDebugWorldObjectOriginDiagnostics =
            ini.GetBoolValue(SECTION, "bDebugWorldObjectOriginDiagnostics", rockDebugWorldObjectOriginDiagnostics);
        rockDebugWorldObjectOriginLogIntervalFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iDebugWorldObjectOriginLogIntervalFrames", rockDebugWorldObjectOriginLogIntervalFrames));
        if (rockDebugWorldObjectOriginLogIntervalFrames < 1) {
            rockDebugWorldObjectOriginLogIntervalFrames = 1;
        }
        rockDebugWorldObjectOriginMismatchWarnGameUnits = static_cast<float>(
            ini.GetDoubleValue(SECTION, "fDebugWorldObjectOriginMismatchWarnGameUnits", rockDebugWorldObjectOriginMismatchWarnGameUnits));
        if (!std::isfinite(rockDebugWorldObjectOriginMismatchWarnGameUnits) || rockDebugWorldObjectOriginMismatchWarnGameUnits < 0.0f) {
            rockDebugWorldObjectOriginMismatchWarnGameUnits = 0.0f;
        }
        rockDebugShowRootFlattenedFingerSkeletonMarkers =
            ini.GetBoolValue(SECTION,
                "bDebugShowRootFlattenedFingerSkeletonMarkers",
                ini.GetBoolValue(SECTION, "bDebugShowFrikFingerSkeletonMarkers", rockDebugShowRootFlattenedFingerSkeletonMarkers));
        rockDebugShowSkeletonBoneVisualizer = ini.GetBoolValue(SECTION, "bDebugShowSkeletonBoneVisualizer", rockDebugShowSkeletonBoneVisualizer);
        rockDebugSkeletonBoneMode = static_cast<int>(ini.GetLongValue(SECTION, "iDebugSkeletonBoneMode", rockDebugSkeletonBoneMode));
        if (rockDebugSkeletonBoneMode < 0 || rockDebugSkeletonBoneMode > 3) {
            rockDebugSkeletonBoneMode = 1;
        }
        rockDebugSkeletonBoneSource = static_cast<int>(ini.GetLongValue(SECTION, "iDebugSkeletonBoneSource", rockDebugSkeletonBoneSource));
        if (rockDebugSkeletonBoneSource != 1 && rockDebugSkeletonBoneSource != 2) {
            rockDebugSkeletonBoneSource = 1;
        }
        rockDebugDrawSkeletonBoneAxes = ini.GetBoolValue(SECTION, "bDebugDrawSkeletonBoneAxes", rockDebugDrawSkeletonBoneAxes);
        rockDebugLogSkeletonBones = ini.GetBoolValue(SECTION, "bDebugLogSkeletonBones", rockDebugLogSkeletonBones);
        rockDebugLogSkeletonBoneTruncation = ini.GetBoolValue(SECTION, "bDebugLogSkeletonBoneTruncation", rockDebugLogSkeletonBoneTruncation);
        rockDebugSkeletonBoneLogFilter = ini.GetValue(SECTION, "sDebugSkeletonBoneLogFilter", rockDebugSkeletonBoneLogFilter.c_str());
        rockDebugSkeletonAxisBoneFilter = ini.GetValue(SECTION, "sDebugSkeletonAxisBoneFilter", rockDebugSkeletonAxisBoneFilter.c_str());
        rockDebugSkeletonBoneLogIntervalFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iDebugSkeletonBoneLogIntervalFrames", rockDebugSkeletonBoneLogIntervalFrames));
        if (rockDebugSkeletonBoneLogIntervalFrames < 1) {
            rockDebugSkeletonBoneLogIntervalFrames = 1;
        }
        rockDebugMaxSkeletonBonesDrawn = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxSkeletonBonesDrawn", rockDebugMaxSkeletonBonesDrawn));
        if (rockDebugMaxSkeletonBonesDrawn < 0) {
            rockDebugMaxSkeletonBonesDrawn = 0;
        } else if (rockDebugMaxSkeletonBonesDrawn > 768) {
            rockDebugMaxSkeletonBonesDrawn = 768;
        }
        rockDebugMaxSkeletonBoneAxesDrawn = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxSkeletonBoneAxesDrawn", rockDebugMaxSkeletonBoneAxesDrawn));
        if (rockDebugMaxSkeletonBoneAxesDrawn < 0) {
            rockDebugMaxSkeletonBoneAxesDrawn = 0;
        } else if (rockDebugMaxSkeletonBoneAxesDrawn > 768) {
            rockDebugMaxSkeletonBoneAxesDrawn = 768;
        }
        rockDebugRootFlattenedFingerSkeletonMarkerSize =
            static_cast<float>(ini.GetDoubleValue(SECTION,
                "fDebugRootFlattenedFingerSkeletonMarkerSize",
                ini.GetDoubleValue(SECTION, "fDebugFrikFingerSkeletonMarkerSize", rockDebugRootFlattenedFingerSkeletonMarkerSize)));
        if (rockDebugRootFlattenedFingerSkeletonMarkerSize < 0.1f) {
            rockDebugRootFlattenedFingerSkeletonMarkerSize = 0.1f;
        }
        rockDebugSkeletonBonePointSize = static_cast<float>(ini.GetDoubleValue(SECTION, "fDebugSkeletonBonePointSize", rockDebugSkeletonBonePointSize));
        if (rockDebugSkeletonBonePointSize < 0.1f) {
            rockDebugSkeletonBonePointSize = 0.1f;
        }
        rockDebugSkeletonBoneAxisLength = static_cast<float>(ini.GetDoubleValue(SECTION, "fDebugSkeletonBoneAxisLength", rockDebugSkeletonBoneAxisLength));
        if (rockDebugSkeletonBoneAxisLength < 0.1f) {
            rockDebugSkeletonBoneAxisLength = 0.1f;
        }

        rockHandColliderRuntimeMode = static_cast<int>(ini.GetLongValue(SECTION, "iHandColliderRuntimeMode", rockHandColliderRuntimeMode));
        if (rockHandColliderRuntimeMode < 0 || rockHandColliderRuntimeMode > 1) {
            ROCK_LOG_WARN(Config, "Invalid iHandColliderRuntimeMode={} - using BoneDerivedHands", rockHandColliderRuntimeMode);
            rockHandColliderRuntimeMode = 1;
        }
        rockHandBoneCollidersRequirePalmAnchor = ini.GetBoolValue(SECTION, "bHandBoneCollidersRequirePalmAnchor", rockHandBoneCollidersRequirePalmAnchor);
        rockHandBoneCollidersRequireAllFingerBones = ini.GetBoolValue(SECTION, "bHandBoneCollidersRequireAllFingerBones", rockHandBoneCollidersRequireAllFingerBones);
        rockHandBoneColliderMaxLinearVelocity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fHandBoneColliderMaxLinearVelocity", rockHandBoneColliderMaxLinearVelocity));
        rockHandBoneColliderMaxAngularVelocity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fHandBoneColliderMaxAngularVelocity", rockHandBoneColliderMaxAngularVelocity));
        if (!std::isfinite(rockHandBoneColliderMaxLinearVelocity) || rockHandBoneColliderMaxLinearVelocity <= 0.0f) {
            rockHandBoneColliderMaxLinearVelocity = 200.0f;
        }
        if (!std::isfinite(rockHandBoneColliderMaxAngularVelocity) || rockHandBoneColliderMaxAngularVelocity <= 0.0f) {
            rockHandBoneColliderMaxAngularVelocity = 500.0f;
        }

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
        rockGrabAdaptiveMotorEnabled = ini.GetBoolValue(SECTION, "bGrabAdaptiveMotorEnabled", rockGrabAdaptiveMotorEnabled);
        rockGrabAdaptivePositionFullError = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAdaptivePositionFullError", rockGrabAdaptivePositionFullError));
        rockGrabAdaptiveRotationFullError = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAdaptiveRotationFullError", rockGrabAdaptiveRotationFullError));
        rockGrabAdaptiveMaxForceMultiplier = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAdaptiveMaxForceMultiplier", rockGrabAdaptiveMaxForceMultiplier));

        rockGrabMaxInertiaRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxInertiaRatio", rockGrabMaxInertiaRatio));

        rockGrabMaxDeviation = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviation", rockGrabMaxDeviation));
        rockGrabMaxDeviationTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviationTime", rockGrabMaxDeviationTime));
        rockGrabButtonID = static_cast<int>(ini.GetLongValue(SECTION, "iGrabButtonID", rockGrabButtonID));
        if (!input_remap_policy::isValidButtonId(rockGrabButtonID)) {
            ROCK_LOG_WARN(Config, "iGrabButtonID must be 0..63; using 2");
            rockGrabButtonID = 2;
        }
        rockThrowVelocityMultiplier = static_cast<float>(ini.GetDoubleValue(SECTION, "fThrowVelocityMultiplier", rockThrowVelocityMultiplier));
        rockGrabReleaseHandCollisionDelaySeconds =
            frik::rock::hand_collision_suppression_math::sanitizeDelaySeconds(
                static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabReleaseHandCollisionDelaySeconds", rockGrabReleaseHandCollisionDelaySeconds)));
        rockGrabVelocityDamping = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabVelocityDamping", rockGrabVelocityDamping));
        rockGrabPlayerSpaceCompensation = ini.GetBoolValue(SECTION, "bGrabPlayerSpaceCompensation", rockGrabPlayerSpaceCompensation);
        rockGrabPlayerSpaceWarpDistance = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabPlayerSpaceWarpDistance", rockGrabPlayerSpaceWarpDistance));
        rockGrabPlayerSpaceWarpMinRotationDegrees =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabPlayerSpaceWarpMinRotationDegrees", rockGrabPlayerSpaceWarpMinRotationDegrees));
        rockGrabPlayerSpaceTransformWarpEnabled = ini.GetBoolValue(SECTION, "bGrabPlayerSpaceTransformWarpEnabled", rockGrabPlayerSpaceTransformWarpEnabled);
        rockGrabResidualVelocityDamping = ini.GetBoolValue(SECTION, "bGrabResidualVelocityDamping", rockGrabResidualVelocityDamping);
        rockGrabNearbyDampingEnabled = ini.GetBoolValue(SECTION, "bGrabNearbyDampingEnabled", rockGrabNearbyDampingEnabled);
        rockGrabNearbyDampingRadius = nearby_grab_damping::sanitizeRadius(static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabNearbyDampingRadius", rockGrabNearbyDampingRadius)));
        rockGrabNearbyDampingSeconds =
            nearby_grab_damping::sanitizeDuration(static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabNearbyDampingSeconds", rockGrabNearbyDampingSeconds)));
        rockGrabNearbyLinearDamping =
            nearby_grab_damping::sanitizeDamping(static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabNearbyLinearDamping", rockGrabNearbyLinearDamping)));
        rockGrabNearbyAngularDamping =
            nearby_grab_damping::sanitizeDamping(static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabNearbyAngularDamping", rockGrabNearbyAngularDamping)));
        rockGrabObjectVisualHandAuthorityEnabled = ini.GetBoolValue(SECTION, "bGrabObjectVisualHandAuthorityEnabled", rockGrabObjectVisualHandAuthorityEnabled);
        rockGrabUseBoneDerivedPalmPivot = ini.GetBoolValue(SECTION, "bGrabUseBoneDerivedPalmPivot", rockGrabUseBoneDerivedPalmPivot);
        rockGrabUseSemanticFingerContactPivot = ini.GetBoolValue(SECTION, "bGrabUseSemanticFingerContactPivot", rockGrabUseSemanticFingerContactPivot);
        rockGrabMultiFingerContactValidationEnabled =
            ini.GetBoolValue(SECTION, "bGrabMultiFingerContactValidationEnabled", rockGrabMultiFingerContactValidationEnabled);
        rockGrabContactQualityMode = static_cast<int>(ini.GetLongValue(SECTION, "iGrabContactQualityMode", rockGrabContactQualityMode));
        rockGrabContactQualityMode = std::clamp(rockGrabContactQualityMode, 0, 2);
        rockGrabMinFingerContactGroups =
            static_cast<int>(ini.GetLongValue(SECTION, "iGrabMinFingerContactGroups", rockGrabMinFingerContactGroups));
        rockGrabMinFingerContactGroups = std::clamp(rockGrabMinFingerContactGroups, 1, 5);
        rockGrabMinFingerContactSpreadGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMinFingerContactSpreadGameUnits", rockGrabMinFingerContactSpreadGameUnits));
        if (!std::isfinite(rockGrabMinFingerContactSpreadGameUnits) || rockGrabMinFingerContactSpreadGameUnits < 0.0f) {
            rockGrabMinFingerContactSpreadGameUnits = 1.0f;
        }
        rockGrabFingerContactMeshSnapMaxDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerContactMeshSnapMaxDistanceGameUnits", rockGrabFingerContactMeshSnapMaxDistanceGameUnits));
        if (!std::isfinite(rockGrabFingerContactMeshSnapMaxDistanceGameUnits) || rockGrabFingerContactMeshSnapMaxDistanceGameUnits < 0.0f) {
            rockGrabFingerContactMeshSnapMaxDistanceGameUnits = 10.0f;
        }
        rockGrabOppositionFrameEnabled = ini.GetBoolValue(SECTION, "bGrabOppositionFrameEnabled", rockGrabOppositionFrameEnabled);
        rockGrabOppositionContactMaxAgeFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iGrabOppositionContactMaxAgeFrames", rockGrabOppositionContactMaxAgeFrames));
        rockGrabOppositionContactMaxAgeFrames = std::clamp(rockGrabOppositionContactMaxAgeFrames, 0, 60);
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
        rockGrabMeshLocalTransformPoseEnabled = ini.GetBoolValue(SECTION, "bGrabMeshLocalTransformPoseEnabled", rockGrabMeshLocalTransformPoseEnabled);
        rockGrabFingerLocalTransformSmoothingSpeed =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerLocalTransformSmoothingSpeed", rockGrabFingerLocalTransformSmoothingSpeed));
        if (!std::isfinite(rockGrabFingerLocalTransformSmoothingSpeed) || rockGrabFingerLocalTransformSmoothingSpeed < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabFingerLocalTransformSmoothingSpeed={} -- using 14.0", rockGrabFingerLocalTransformSmoothingSpeed);
            rockGrabFingerLocalTransformSmoothingSpeed = 14.0f;
        }
        rockGrabFingerLocalTransformMaxCorrectionDegrees =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerLocalTransformMaxCorrectionDegrees", rockGrabFingerLocalTransformMaxCorrectionDegrees));
        if (!std::isfinite(rockGrabFingerLocalTransformMaxCorrectionDegrees) || rockGrabFingerLocalTransformMaxCorrectionDegrees < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabFingerLocalTransformMaxCorrectionDegrees={} -- using 35.0", rockGrabFingerLocalTransformMaxCorrectionDegrees);
            rockGrabFingerLocalTransformMaxCorrectionDegrees = 35.0f;
        }
        rockGrabFingerSurfaceAimStrength = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerSurfaceAimStrength", rockGrabFingerSurfaceAimStrength));
        rockGrabFingerSurfaceAimStrength = std::clamp(std::isfinite(rockGrabFingerSurfaceAimStrength) ? rockGrabFingerSurfaceAimStrength : 0.75f, 0.0f, 1.0f);
        rockGrabThumbOppositionStrength = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThumbOppositionStrength", rockGrabThumbOppositionStrength));
        rockGrabThumbOppositionStrength = std::clamp(std::isfinite(rockGrabThumbOppositionStrength) ? rockGrabThumbOppositionStrength : 1.0f, 0.0f, 1.0f);
        rockGrabThumbAlternateCurveStrength =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThumbAlternateCurveStrength", rockGrabThumbAlternateCurveStrength));
        rockGrabThumbAlternateCurveStrength = std::clamp(std::isfinite(rockGrabThumbAlternateCurveStrength) ? rockGrabThumbAlternateCurveStrength : 0.65f, 0.0f, 1.0f);
        rockGrabLateralWeight = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLateralWeight", rockGrabLateralWeight));
        rockGrabDirectionalWeight = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabDirectionalWeight", rockGrabDirectionalWeight));
        rockGrabMaxTriangleDistance = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxTriangleDistance", rockGrabMaxTriangleDistance));
        rockGrabOrientationMode = static_cast<int>(ini.GetLongValue(SECTION, "iGrabOrientationMode", rockGrabOrientationMode));
        if (rockGrabOrientationMode < 0 || rockGrabOrientationMode > 2) {
            ROCK_LOG_WARN(Config, "Invalid iGrabOrientationMode={} -- using PreserveObjectRotation", rockGrabOrientationMode);
            rockGrabOrientationMode = 0;
        }
        rockGrabMeshContactOnly = ini.GetBoolValue(SECTION, "bGrabMeshContactOnly", rockGrabMeshContactOnly);
        rockGrabRequireMeshContact = ini.GetBoolValue(SECTION, "bGrabRequireMeshContact", rockGrabRequireMeshContact);
        rockGrabSurfaceFrameMinConfidence = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabSurfaceFrameMinConfidence", rockGrabSurfaceFrameMinConfidence));
        if (!std::isfinite(rockGrabSurfaceFrameMinConfidence) || rockGrabSurfaceFrameMinConfidence < 0.0f) {
            rockGrabSurfaceFrameMinConfidence = 0.0f;
        }
        rockGrabSurfaceCapNormalDotThreshold = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabSurfaceCapNormalDotThreshold", rockGrabSurfaceCapNormalDotThreshold));
        rockGrabSurfaceCapNormalDotThreshold = std::clamp(rockGrabSurfaceCapNormalDotThreshold, 0.0f, 1.0f);
        rockGrabSurfacePreserveRollForCaps = ini.GetBoolValue(SECTION, "bGrabSurfacePreserveRollForCaps", rockGrabSurfacePreserveRollForCaps);
        rockGrabContactPatchEnabled = ini.GetBoolValue(SECTION, "bGrabContactPatchEnabled", rockGrabContactPatchEnabled);
        rockGrabContactPatchProbeCount = static_cast<int>(ini.GetLongValue(SECTION, "iGrabContactPatchProbeCount", rockGrabContactPatchProbeCount));
        rockGrabContactPatchProbeCount = std::clamp(rockGrabContactPatchProbeCount, 1, 5);
        rockGrabContactPatchProbeSpacingGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabContactPatchProbeSpacingGameUnits", rockGrabContactPatchProbeSpacingGameUnits));
        if (!std::isfinite(rockGrabContactPatchProbeSpacingGameUnits) || rockGrabContactPatchProbeSpacingGameUnits < 0.0f) {
            rockGrabContactPatchProbeSpacingGameUnits = 3.0f;
        }
        rockGrabContactPatchProbeRadiusGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabContactPatchProbeRadiusGameUnits", rockGrabContactPatchProbeRadiusGameUnits));
        if (!std::isfinite(rockGrabContactPatchProbeRadiusGameUnits) || rockGrabContactPatchProbeRadiusGameUnits <= 0.0f) {
            rockGrabContactPatchProbeRadiusGameUnits = 2.0f;
        }
        rockGrabContactPatchMeshSnapMaxDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabContactPatchMeshSnapMaxDistanceGameUnits", rockGrabContactPatchMeshSnapMaxDistanceGameUnits));
        if (!std::isfinite(rockGrabContactPatchMeshSnapMaxDistanceGameUnits) || rockGrabContactPatchMeshSnapMaxDistanceGameUnits < 0.0f) {
            rockGrabContactPatchMeshSnapMaxDistanceGameUnits = 6.0f;
        }
        rockGrabContactPatchMaxNormalAngleDegrees =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabContactPatchMaxNormalAngleDegrees", rockGrabContactPatchMaxNormalAngleDegrees));
        rockGrabContactPatchMaxNormalAngleDegrees = std::clamp(rockGrabContactPatchMaxNormalAngleDegrees, 0.0f, 179.0f);
        rockGrabAlignmentMaxPivotToSurfaceDistance =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAlignmentMaxPivotToSurfaceDistance", rockGrabAlignmentMaxPivotToSurfaceDistance));
        if (!std::isfinite(rockGrabAlignmentMaxPivotToSurfaceDistance)) {
            rockGrabAlignmentMaxPivotToSurfaceDistance = 8.0f;
        }
        rockGrabAlignmentMaxSelectionToMeshDistance =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAlignmentMaxSelectionToMeshDistance", rockGrabAlignmentMaxSelectionToMeshDistance));
        if (!std::isfinite(rockGrabAlignmentMaxSelectionToMeshDistance)) {
            rockGrabAlignmentMaxSelectionToMeshDistance = 8.0f;
        }
        rockGrabAlignmentRequireResolvedOwnerMatch = ini.GetBoolValue(SECTION, "bGrabAlignmentRequireResolvedOwnerMatch", rockGrabAlignmentRequireResolvedOwnerMatch);
        rockGrabAlignmentUseHandParallelTangent = ini.GetBoolValue(SECTION, "bGrabAlignmentUseHandParallelTangent", rockGrabAlignmentUseHandParallelTangent);
        rockGrabNodeAnchorsEnabled = ini.GetBoolValue(SECTION, "bGrabNodeAnchorsEnabled", rockGrabNodeAnchorsEnabled);
        rockPrintGrabNodeInfo = ini.GetBoolValue(SECTION, "bPrintGrabNodeInfo", rockPrintGrabNodeInfo);
        rockGrabNodeNameRight =
            grab_node_name_policy::sanitizeConfiguredGrabNodeName(ini.GetValue(SECTION, "sGrabNodeNameRight", rockGrabNodeNameRight.c_str()), false);
        rockGrabNodeNameLeft = grab_node_name_policy::sanitizeConfiguredGrabNodeName(ini.GetValue(SECTION, "sGrabNodeNameLeft", rockGrabNodeNameLeft.c_str()), true);
        rockSelectedCloseFingerCurlEnabled = ini.GetBoolValue(SECTION, "bSelectedCloseFingerCurlEnabled", rockSelectedCloseFingerCurlEnabled);
        rockSelectedCloseFingerAnimMaxHandSpeed =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSelectedCloseFingerAnimMaxHandSpeed", rockSelectedCloseFingerAnimMaxHandSpeed));
        rockSelectedCloseFingerAnimValue = static_cast<float>(ini.GetDoubleValue(SECTION, "fSelectedCloseFingerAnimValue", rockSelectedCloseFingerAnimValue));
        rockPulledAngularDamping = static_cast<float>(ini.GetDoubleValue(SECTION, "fPulledAngularDamping", rockPulledAngularDamping));

        readOptionalVec3("fRightGrabPivotAHandspaceX", "fRightGrabPivotAHandspaceY", "fRightGrabPivotAHandspaceZ", rockRightGrabPivotAHandspace);
        readOptionalVec3("fLeftGrabPivotAHandspaceX", "fLeftGrabPivotAHandspaceY", "fLeftGrabPivotAHandspaceZ", rockLeftGrabPivotAHandspace);

        rockGrabLerpSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpSpeed", rockGrabLerpSpeed));
        rockGrabLerpAngularSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpAngularSpeed", rockGrabLerpAngularSpeed));
        rockGrabLerpMaxTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLerpMaxTime", rockGrabLerpMaxTime));

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

        resetToDefaults();
        readValuesFromIni(ini);
        ROCK_LOG_INFO(Config,
            "ROCK config reloaded (rockEnabled={}, logLevel={} {}, sample={}ms)",
            rockEnabled,
            rockLogLevel,
            logging_policy::logLevelName(rockLogLevel),
            rockLogSampleMilliseconds);
    }

    std::filesystem::path RockConfig::getConfigDirectory() const
    {
        if (_iniFilePath.empty()) {
            return std::filesystem::path(resolveIniPath()).parent_path();
        }
        return std::filesystem::path(_iniFilePath).parent_path();
    }

    void RockConfig::processPendingConfigReload()
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
