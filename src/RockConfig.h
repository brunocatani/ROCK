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

#include "physics-interaction/hand/HandSelection.h"
#include "physics-interaction/weapon/WeaponSemantics.h"

namespace rock
{
    class RockConfig
    {
    public:
        ~RockConfig() { stopFileWatch(); }

        void load();

        void reload();

        void processPendingConfigReload();

        void stopFileWatch();

        void subscribeForConfigChanged(const std::string& key, std::function<void(const std::string&)> callback);
        void unsubscribeFromConfigChanged(const std::string& key);
        [[nodiscard]] std::filesystem::path getConfigDirectory() const;

        void suppressNextFileWatchReload() { _ignoreNextIniFileChange.store(true); }

        bool rockEnabled = true;

        bool rockInputRemapEnabled = true;
        int rockRightWeaponReadyButtonID = 32;
        bool rockSuppressRightGrabGameInput = true;
        bool rockSuppressRightFavoritesGameInput = true;
        bool rockSuppressNativeReadyWeaponAutoReady = true;

        int rockLogLevel = 2;
        std::string rockLogPattern = "%Y-%m-%d %H:%M:%S.%e [%l] %v";
        int rockLogSampleMilliseconds = 2000;

        RE::NiPoint3 rockPalmNormalHandspace = RE::NiPoint3(0.0f, 0.0f, 1.0f);
        RE::NiPoint3 rockPointingVectorHandspace = RE::NiPoint3(0.0f, 0.0f, 1.0f);
        bool rockReversePalmNormal = true;
        bool rockReverseFarGrabNormal = false;

        bool rockWeaponCollisionEnabled = true;
        bool rockWeaponCollisionBlocksProjectiles = false;
        bool rockWeaponCollisionBlocksSpells = false;
        int rockWeaponCollisionGroupingMode = weapon_collision_grouping_policy::kDefaultWeaponCollisionGroupingMode;
        float rockWeaponCollisionConvexRadius = 0.01f;
        float rockWeaponCollisionPointDedupGrid = 0.002f;
        float rockWeaponCollisionMaxLinearVelocity = 50.0f;
        float rockWeaponCollisionMaxAngularVelocity = 100.0f;
        float rockWeaponInteractionProbeRadius = 12.0f;
        bool rockVisualOnlySidearmSupportGripEnabled = true;

        bool rockNativeMeleeSuppressionEnabled = true;
        bool rockNativeMeleeFullSuppression = true;
        bool rockNativeMeleeSuppressWeaponSwing = true;
        bool rockNativeMeleeSuppressHitFrame = true;
        bool rockNativeMeleeDebugLogging = false;

        bool rockHighlightEnabled = true;

        bool rockDebugShowColliders = true;
        bool rockDebugShowTargetColliders = true;
        bool rockDebugShowHandAxes = true;
        bool rockDebugShowGrabPivots = true;
        bool rockDebugShowGrabSurfaceFrame = true;
        bool rockDebugDrawGrabContactPatch = true;
        bool rockDebugShowGrabFingerProbes = true;
        bool rockDebugShowPalmVectors = true;
        bool rockDebugDrawHandColliders = true;
        bool rockDebugDrawHandBoneColliders = true;
        bool rockDebugDrawHandBoneContacts = true;
        int rockDebugMaxHandBoneBodiesDrawn = 48;
        bool rockDebugDrawWeaponColliders = true;
        int rockDebugMaxWeaponBodiesDrawn = 100;
        int rockDebugMaxShapeGenerationsPerFrame = 100;
        int rockDebugMaxConvexSupportVertices = 6;
        bool rockDebugUseBoundsForHeavyConvex = true;
        bool rockDebugVerboseLogging = false;
        bool rockDebugGrabFrameLogging = false;
        bool rockDebugGrabTransformTelemetry = false;
        bool rockDebugGrabTransformTelemetryText = true;
        bool rockDebugGrabTransformTelemetryAxes = true;
        int rockDebugGrabTransformTelemetryLogIntervalFrames = 10;
        int rockDebugGrabTransformTelemetryTextMode = 0;
        bool rockDebugShowGrabNotifications = false;
        bool rockDebugShowWeaponNotifications = false;
        bool rockDebugHandTransformParity = false;
        bool rockDebugWorldObjectOriginDiagnostics = false;
        int rockDebugWorldObjectOriginLogIntervalFrames = 120;
        float rockDebugWorldObjectOriginMismatchWarnGameUnits = 5.0f;
        bool rockDebugShowRootFlattenedFingerSkeletonMarkers = true;
        bool rockDebugShowSkeletonBoneVisualizer = true;
        bool rockDebugDrawSkeletonBoneAxes = true;
        bool rockDebugLogSkeletonBones = true;
        int rockDebugSkeletonBoneMode = 1;
        int rockDebugSkeletonBoneSource = 1;
        int rockDebugMaxSkeletonBonesDrawn = 256;
        int rockDebugMaxSkeletonBoneAxesDrawn = 80;
        int rockDebugSkeletonBoneLogIntervalFrames = 120;
        bool rockDebugLogSkeletonBoneTruncation = true;
        float rockDebugRootFlattenedFingerSkeletonMarkerSize = 1.4f;
        float rockDebugSkeletonBonePointSize = 1.4f;
        float rockDebugSkeletonBoneAxisLength = 4.0f;
        std::string rockDebugSkeletonBoneLogFilter = "RArm_Hand,LArm_Hand,RArm_Finger23,LArm_Finger23,Chest,Pelvis";
        std::string rockDebugSkeletonAxisBoneFilter = "";

        int rockHandColliderRuntimeMode = 1;
        bool rockHandBoneCollidersRequirePalmAnchor = true;
        bool rockHandBoneCollidersRequireAllFingerBones = true;
        float rockHandBoneColliderMaxLinearVelocity = 200.0f;
        float rockHandBoneColliderMaxAngularVelocity = 500.0f;

        float rockNearDetectionRange = 25.0f;
        float rockFarDetectionRange = 350.0f;
        float rockNearCastRadiusGameUnits = 6.0f;
        float rockNearCastDistanceGameUnits = 25.0f;
        float rockFarCastRadiusGameUnits = 21.0f;
        std::uint32_t rockSelectionShapeCastFilterInfo = selection_query_policy::kDefaultShapeCastFilterInfo;
        std::uint32_t rockFarClipRayFilterInfo = selection_query_policy::kDefaultFarClipRayFilterInfo;
        float rockPullApplyVelocityTime = 0.2f;
        float rockPullTrackHandTime = 0.1f;
        float rockPullDestinationZOffsetHavok = 0.01f;
        float rockPullDurationA = 0.715619f;
        float rockPullDurationB = -0.415619f;
        float rockPullDurationC = 0.656256f;
        float rockPullMaxVelocityHavok = 10.0f;
        float rockPullAutoGrabDistanceGameUnits = 18.0f;
        int rockObjectPhysicsTreeMaxDepth = 12;
        bool rockDynamicPushAssistEnabled = true;
        float rockDynamicPushMinSpeed = 0.35f;
        float rockDynamicPushMaxImpulse = 2.0f;
        float rockDynamicPushCooldownSeconds = 0.08f;

        float rockGrabLinearTau = 0.03f;
        float rockGrabLinearDamping = 0.8f;
        float rockGrabLinearProportionalRecovery = 2.0f;
        float rockGrabLinearConstantRecovery = 1.0f;

        float rockGrabAngularTau = 0.03f;
        float rockGrabAngularDamping = 0.8f;
        float rockGrabAngularProportionalRecovery = 2.0f;
        float rockGrabAngularConstantRecovery = 1.0f;

        float rockGrabConstraintMaxForce = 2000.0f;
        float rockGrabAngularToLinearForceRatio = 12.5f;
        float rockGrabMaxForceToMassRatio = 500.0f;
        float rockGrabFadeInStartAngularRatio = 100.0f;

        float rockGrabForceFadeInTime = 0.1f;
        float rockGrabTauMin = 0.01f;
        float rockGrabTauMax = 0.8f;
        float rockGrabTauLerpSpeed = 0.5f;
        bool rockGrabAdaptiveMotorEnabled = false;
        float rockGrabAdaptivePositionFullError = 20.0f;
        float rockGrabAdaptiveRotationFullError = 60.0f;
        float rockGrabAdaptiveMaxForceMultiplier = 4.0f;

        float rockGrabMaxInertiaRatio = 10.0f;

        float rockGrabMaxDeviation = 50.0f;
        float rockGrabMaxDeviationTime = 2.0f;
        int rockGrabButtonID = 2;
        float rockThrowVelocityMultiplier = 1.5f;
        float rockGrabReleaseHandCollisionDelaySeconds = 0.10f;
        float rockGrabVelocityDamping = 0.25f;
        bool rockGrabPlayerSpaceCompensation = true;
        float rockGrabPlayerSpaceWarpDistance = 35.0f;
        float rockGrabPlayerSpaceWarpMinRotationDegrees = 0.6f;
        bool rockGrabPlayerSpaceTransformWarpEnabled = false;
        bool rockGrabResidualVelocityDamping = true;
        bool rockGrabNearbyDampingEnabled = true;
        float rockGrabNearbyDampingRadius = 90.0f;
        float rockGrabNearbyDampingSeconds = 0.35f;
        float rockGrabNearbyLinearDamping = 0.65f;
        float rockGrabNearbyAngularDamping = 0.85f;
        bool rockGrabObjectVisualHandAuthorityEnabled = false;
        bool rockGrabUseBoneDerivedPalmPivot = true;
        bool rockGrabUseSemanticFingerContactPivot = true;
        bool rockGrabMultiFingerContactValidationEnabled = true;
        int rockGrabContactQualityMode = 1;
        int rockGrabMinFingerContactGroups = 3;
        float rockGrabMinFingerContactSpreadGameUnits = 1.0f;
        float rockGrabFingerContactMeshSnapMaxDistanceGameUnits = 10.0f;
        bool rockGrabOppositionFrameEnabled = true;
        int rockGrabOppositionContactMaxAgeFrames = 5;
        bool rockGrabHandLerpEnabled = true;
        float rockGrabHandLerpTimeMin = 0.10f;
        float rockGrabHandLerpTimeMax = 0.20f;
        float rockGrabHandLerpMinDistance = 7.0f;
        float rockGrabHandLerpMaxDistance = 14.0f;
        bool rockGrabMeshFingerPoseEnabled = true;
        bool rockGrabMeshJointPoseEnabled = true;
        int rockGrabFingerPoseUpdateInterval = 3;
        float rockGrabFingerMinValue = 0.2f;
        float rockGrabFingerPoseSmoothingSpeed = 14.0f;
        bool rockGrabMeshLocalTransformPoseEnabled = true;
        float rockGrabFingerLocalTransformSmoothingSpeed = 14.0f;
        float rockGrabFingerLocalTransformMaxCorrectionDegrees = 35.0f;
        float rockGrabFingerSurfaceAimStrength = 0.75f;
        float rockGrabThumbOppositionStrength = 1.0f;
        float rockGrabThumbAlternateCurveStrength = 0.65f;
        float rockGrabLateralWeight = 0.6f;
        float rockGrabDirectionalWeight = 0.4f;
        float rockGrabMaxTriangleDistance = 100.0f;
        int rockGrabOrientationMode = 0;
        bool rockGrabMeshContactOnly = true;
        bool rockGrabRequireMeshContact = true;
        float rockGrabSurfaceFrameMinConfidence = 0.2f;
        float rockGrabSurfaceCapNormalDotThreshold = 0.85f;
        bool rockGrabSurfacePreserveRollForCaps = true;
        bool rockGrabContactPatchEnabled = true;
        int rockGrabContactPatchProbeCount = 5;
        float rockGrabContactPatchProbeSpacingGameUnits = 3.0f;
        float rockGrabContactPatchProbeRadiusGameUnits = 2.0f;
        float rockGrabContactPatchMeshSnapMaxDistanceGameUnits = 6.0f;
        float rockGrabContactPatchMaxNormalAngleDegrees = 35.0f;
        float rockGrabAlignmentMaxPivotToSurfaceDistance = 8.0f;
        float rockGrabAlignmentMaxSelectionToMeshDistance = 8.0f;
        bool rockGrabAlignmentRequireResolvedOwnerMatch = true;
        bool rockGrabAlignmentUseHandParallelTangent = true;
        bool rockGrabNodeAnchorsEnabled = true;
        bool rockPrintGrabNodeInfo = false;
        std::string rockGrabNodeNameRight = "ROCK:GrabR";
        std::string rockGrabNodeNameLeft = "ROCK:GrabL";
        bool rockSelectedCloseFingerCurlEnabled = true;
        float rockSelectedCloseFingerAnimMaxHandSpeed = 0.9f;
        float rockSelectedCloseFingerAnimValue = 0.9f;
        float rockPulledAngularDamping = 8.0f;

        RE::NiPoint3 rockRightGrabPivotAHandspace = RE::NiPoint3(6.0f, 0.2f, -2.0f);
        RE::NiPoint3 rockLeftGrabPivotAHandspace = RE::NiPoint3(6.0f, -0.2f, -2.0f);

        float rockGrabLerpSpeed = 300.0f;
        float rockGrabLerpAngularSpeed = 360.0f;
        float rockGrabLerpMaxTime = 0.5f;

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
