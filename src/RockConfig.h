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

#ifndef MAX_PATH
#define ROCK_DEFINED_MAX_PATH_FOR_FILEWATCH 1
#define MAX_PATH 260
#endif
#include <thomasmonkman-filewatch/FileWatch.hpp>
#if defined(ROCK_DEFINED_MAX_PATH_FOR_FILEWATCH)
#undef MAX_PATH
#undef ROCK_DEFINED_MAX_PATH_FOR_FILEWATCH
#endif

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

        [[nodiscard]] bool persistPhysicsBool(const char* key, bool value);
        [[nodiscard]] bool persistGrabPivotAHandspace(bool isLeft, const RE::NiPoint3& value);

        bool rockEnabled = true;

        bool rockInputRemapEnabled = true;
        int rockRightWeaponReadyButtonID = 32;
        bool rockSuppressRightGrabGameInput = true;
        bool rockSuppressRightFavoritesGameInput = true;
        bool rockSuppressNativeReadyWeaponAutoReady = true;
        bool rockGrabInputIntentStateEnabled = true;
        float rockGrabInputLeewaySeconds = 0.12f;
        float rockGrabInputForceSeconds = 0.08f;

        int rockLogLevel = 2;
        std::string rockLogPattern = "%Y-%m-%d %H:%M:%S.%e [%l] %v";
        int rockLogSampleMilliseconds = 2000;
        bool rockPerformanceProfilerEnabled = false;
        int rockPerformanceProfilerLogIntervalFrames = 300;
        int rockPerformanceProfilerWarmupFrames = 120;
        bool rockPerformanceProfilerOverlayText = false;

        RE::NiPoint3 rockPalmNormalHandspace = RE::NiPoint3(0.0f, 0.0f, 1.0f);
        RE::NiPoint3 rockPointingVectorHandspace = RE::NiPoint3(0.0f, 0.0f, 1.0f);
        bool rockReversePalmNormal = true;
        bool rockReverseFarGrabNormal = true;

        bool rockWeaponCollisionEnabled = true;
        bool rockWeaponCollisionBlocksProjectiles = false;
        bool rockWeaponCollisionBlocksSpells = false;
        bool rockWeaponCollisionStaticWorldEnabled = true;
        bool rockWeaponCollisionNativeVisualRemapEnabled = true;
        int rockWeaponCollisionGroupingMode = weapon_collision_grouping_policy::kDefaultWeaponCollisionGroupingMode;
        float rockWeaponCollisionConvexRadius = 0.01f;
        float rockWeaponCollisionPointDedupGrid = 0.002f;
        int rockWeaponCollisionSupportFitTargetPoints = 96;
        float rockWeaponCollisionSupportFitMaxErrorGameUnits = 0.5f;
        float rockWeaponCollisionMaxLinearVelocity = 50.0f;
        float rockWeaponCollisionMaxAngularVelocity = 100.0f;
        float rockWeaponInteractionProbeRadius = 12.0f;
        bool rockVisualOnlySidearmSupportGripEnabled = true;
        bool rockSeeThroughScopesCompatibilityEnabled = true;
        bool rockSeeThroughScopesReticleAlignmentEnabled = true;
        bool rockSeeThroughScopesRightEyeDominant = true;
        float rockSeeThroughScopesEyeOffsetGameUnits = 2.3f;
        float rockSeeThroughScopesReticleOffsetXGameUnits = 0.372727f;
        float rockSeeThroughScopesReticleOffsetZGameUnits = -0.149692f;
        float rockSeeThroughScopesLookDotThreshold = 0.98f;
        float rockSeeThroughScopesDistanceThresholdGameUnits = 20.0f;

        bool rockSoftContactEnabled = true;
        bool rockSoftContactHandHandEnabled = true;
        bool rockSoftContactWeaponHandEnabled = true;
        bool rockSoftContactBodyEnabled = true;
        bool rockSoftContactWorldEnabled = true;
        float rockSoftContactRadiusPaddingGameUnits = 0.75f;
        float rockSoftContactMaxCorrectionGameUnits = 7.0f;
        float rockSoftContactWeaponHandRadiusPaddingGameUnits = 0.25f;
        float rockSoftContactWeaponHandMaxCorrectionGameUnits = 3.0f;
        float rockSoftContactWeaponHandCorrectionScale = 0.35f;
        float rockSoftContactWeaponHandHardStopPenetrationGameUnits = 4.0f;
        int rockSoftContactVisualPriority = 80;
        float rockSoftContactWorldRadiusPaddingGameUnits = 1.5f;
        float rockSoftContactWorldContactPaddingGameUnits = 0.35f;
        float rockSoftContactWorldSkinGameUnits = 0.5f;
        float rockSoftContactWorldPostReleaseReentryMinApproachDistanceGameUnits = 0.025f;
        float rockSoftContactWorldCachedPlaneMaxTangentDriftGameUnits = 10.0f;
        float rockSoftContactWorldMaxCorrectionGameUnits = 18.0f;
        bool rockSoftContactWorldHapticsEnabled = true;
        float rockSoftContactWorldHapticDurationSeconds = 0.035f;
        float rockSoftContactWorldHapticBaseIntensity = 0.18f;
        float rockSoftContactWorldHapticMaxIntensity = 0.55f;
        float rockSoftContactWorldHapticSpeedScale = 0.006f;
        float rockSoftContactWorldHapticMinApproachSpeedGameUnits = 3.0f;
        float rockSoftContactWorldHapticCooldownSeconds = 0.12f;

        bool rockNativeMeleeSuppressionEnabled = true;
        bool rockNativeMeleeFullSuppression = true;
        bool rockNativeMeleeSuppressWeaponSwing = true;
        bool rockNativeMeleeSuppressHitFrame = true;
        bool rockNativeMeleeDebugLogging = false;
        bool rockNativeCharacterControllerObjectContactFilterEnabled = true;

        bool rockHighlightEnabled = true;

        bool rockDebugShowColliders = false;
        bool rockDebugShowTargetColliders = false;
        bool rockDebugShowHandAxes = false;
        bool rockDebugShowGrabPivots = false;
        bool rockDebugShowGrabPocketNormal = false;
        bool rockDebugDrawGrabContactPatch = false;
        bool rockDebugDrawGrabForceTorque = false;
        bool rockDebugDrawGrabForceTorqueText = false;
        bool rockDebugDrawGrabPivotSourceCollider = false;
        bool rockDebugDrawGrabPivotSourceEvidence = false;
        bool rockDebugShowGrabFingerProbes = false;
        bool rockDebugShowPalmVectors = false;
        bool rockDebugDrawHandColliders = false;
        bool rockDebugDrawHandBoneColliders = false;
        bool rockDebugDrawHandBoneContacts = false;
        bool rockDebugDrawSoftContacts = false;
        bool rockDebugDrawGrabAuthorityProxy = false;
        int rockDebugMaxHandBoneBodiesDrawn = 48;
        int rockDebugMaxBodyBoneBodiesDrawn = 32;
        bool rockDebugDrawWeaponColliders = false;
        int rockDebugMaxWeaponBodiesDrawn = 100;
        int rockDebugMaxShapeGenerationsPerFrame = 100;
        int rockDebugMaxConvexSupportVertices = 6;
        bool rockDebugUseBoundsForHeavyConvex = true;
        bool rockDebugContactTargetIdentityLogging = false;
        int rockDebugContactTargetIdentitySampleMilliseconds = 500;
        bool rockDebugVerboseLogging = false;
        bool rockDebugGrabFrameLogging = false;
        bool rockDebugGrabTransformTelemetry = false;
        bool rockDebugGrabTransformTelemetryText = false;
        bool rockDebugGrabTransformTelemetryAxes = false;
        int rockDebugGrabTransformTelemetryLogIntervalFrames = 1;
        int rockDebugGrabTransformTelemetryTextMode = 0;
        bool rockDebugShowGrabNotifications = false;
        bool rockDebugShowWeaponNotifications = false;
        bool rockDebugHandTransformParity = false;
        bool rockDebugWorldObjectOriginDiagnostics = false;
        int rockDebugWorldObjectOriginLogIntervalFrames = 120;
        float rockDebugWorldObjectOriginMismatchWarnGameUnits = 5.0f;
        bool rockDebugShowRootFlattenedFingerSkeletonMarkers = false;
        bool rockDebugShowSkeletonBoneVisualizer = false;
        bool rockDebugDrawSkeletonBoneAxes = false;
        bool rockDebugLogSkeletonBones = false;
        int rockDebugSkeletonBoneMode = 1;
        int rockDebugSkeletonBoneSource = 1;
        int rockDebugMaxSkeletonBonesDrawn = 256;
        int rockDebugMaxSkeletonBoneAxesDrawn = 80;
        int rockDebugSkeletonBoneLogIntervalFrames = 120;
        bool rockDebugLogSkeletonBoneTruncation = false;
        float rockDebugRootFlattenedFingerSkeletonMarkerSize = 1.4f;
        float rockDebugSkeletonBonePointSize = 1.4f;
        float rockDebugSkeletonBoneAxisLength = 4.0f;
        std::string rockDebugSkeletonBoneLogFilter = "RArm_Hand,LArm_Hand,RArm_Finger23,LArm_Finger23,Chest,Pelvis";
        std::string rockDebugSkeletonAxisBoneFilter = "";

        int rockHandColliderRuntimeMode = 1;
        bool rockBodyBoneCollidersEnabled = true;
        bool rockBodyBoneCollisionStaticWorldEnabled = true;
        float rockBodyBoneColliderStandardRadiusScale = 1.0f;
        float rockBodyBoneColliderStandardLengthScale = 1.0f;
        float rockBodyBoneColliderStandardConvexRadiusScale = 1.0f;
        float rockBodyBoneColliderPowerArmorRadiusScale = 1.0f;
        float rockBodyBoneColliderPowerArmorLengthScale = 1.0f;
        float rockBodyBoneColliderPowerArmorConvexRadiusScale = 1.0f;
        float rockBodyBoneColliderTorsoRadiusScale = 1.0f;
        float rockBodyBoneColliderArmRadiusScale = 1.0f;
        float rockBodyBoneColliderLegRadiusScale = 1.0f;
        float rockBodyBoneColliderFootRadiusScale = 1.0f;
        float rockBodyBoneColliderTorsoLengthScale = 1.0f;
        float rockBodyBoneColliderArmLengthScale = 1.0f;
        float rockBodyBoneColliderLegLengthScale = 1.0f;
        float rockBodyBoneColliderFootLengthScale = 1.0f;
        std::string rockBodyBoneColliderZoneScaleOverrides = "";
        bool rockHandCollisionStaticWorldEnabled = true;
        bool rockHandBoneCollidersRequirePalmAnchor = true;
        bool rockHandBoneCollidersRequireAllFingerBones = true;
        float rockHandBoneColliderMaxLinearVelocity = 200.0f;
        float rockHandBoneColliderMaxAngularVelocity = 500.0f;

        float rockNearDetectionRange = 25.0f;
        float rockFarDetectionRange = 350.0f;
        float rockNearCastRadiusGameUnits = 6.0f;
        float rockNearCastDistanceGameUnits = 25.0f;
        float rockFarCastRadiusGameUnits = 21.0f;
        bool rockFarSelectionHmdConeEnabled = true;
        float rockFarSelectionHmdConeHalfAngleDegrees = selection_query_policy::kDefaultFarSelectionHmdConeHalfAngleDegrees;
        float rockCloseSelectionBehindPalmToleranceGameUnits = 2.0f;
        std::uint32_t rockSelectionShapeCastFilterInfo = selection_query_policy::kDefaultShapeCastFilterInfo;
        std::uint32_t rockFarClipRayFilterInfo = selection_query_policy::kDefaultFarClipRayFilterInfo;
        float rockPullApplyVelocityTime = 0.2f;
        float rockPullOwnerGraceSeconds = 1.0f;
        float rockPullTrackHandTime = 0.1f;
        float rockPullDestinationZOffsetHavok = 0.01f;
        float rockPullDurationA = 0.715619f;
        float rockPullDurationB = -0.415619f;
        float rockPullDurationC = 0.656256f;
        float rockPullMaxVelocityHavok = 10.0f;
        float rockPullAutoGrabDistanceGameUnits = 18.0f;
        float rockPullCatchRetryMaxTimeSeconds = 0.65f;
        bool rockPullCatchWideReacquireEnabled = true;
        float rockPullCatchWideReacquireRadiusGameUnits = 32.0f;
        float rockPullCatchWideReacquireMaxBodyDistanceGameUnits = 42.0f;
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
        int rockGrabAngularAuthorityMode = 0;

        float rockGrabConstraintMaxForce = 2000.0f;
        float rockGrabMassResponsiveMaxForce = 9000.0f;
        float rockGrabAngularToLinearForceRatio = 12.5f;
        float rockGrabMaxForceToMassRatio = 500.0f;
        float rockGrabFadeInStartAngularRatio = 100.0f;

        float rockGrabForceFadeInTime = 0.1f;
        RE::NiPoint3 rockRightGrabAuthorityProxyOffsetGameUnits = RE::NiPoint3(0.0f, 0.0f, 0.0f);
        RE::NiPoint3 rockLeftGrabAuthorityProxyOffsetGameUnits = RE::NiPoint3(0.0f, 0.0f, 0.0f);
        float rockGrabLooseWeaponSharedConstraintLinearTauMultiplier = 1.0f;
        float rockGrabLooseWeaponSharedConstraintAngularTauMultiplier = 1.0f;
        float rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier = 1.0f;
        float rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier = 1.0f;
        float rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier = 1.0f;
        float rockGrabLooseWeaponSharedConstraintMaxForceMultiplier = 4.5f;
        float rockGrabLooseWeaponSharedConstraintAngularForceMultiplier = 1.0f;
        float rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier = 1.0f;
        float rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier = 1.0f;
        float rockGrabTauMin = 0.01f;
        float rockGrabTauMax = 0.8f;
        float rockGrabTauLerpSpeed = 0.5f;
        bool rockGrabAdaptiveMotorEnabled = true;
        float rockGrabAdaptivePositionFullError = 20.0f;
        float rockGrabAdaptiveRotationFullError = 60.0f;
        float rockGrabAdaptiveMaxForceMultiplier = 1.0f;
        bool rockGrabLongObjectAngularScalingEnabled = true;
        float rockGrabLongObjectReferenceLeverGameUnits = 24.0f;
        float rockGrabLongObjectMinAngularScale = 0.35f;

        float rockGrabMaxInertiaRatio = 10.0f;

        float rockGrabMaxDeviation = 50.0f;
        float rockGrabMaxDeviationTime = 2.0f;
        int rockGrabButtonID = 2;
        float rockThrowVelocityMultiplier = 1.5f;
        bool rockGrabControllerDerivedThrowVelocityEnabled = true;
        float rockGrabThrowObjectVelocityBlend = 0.35f;
        float rockGrabThrowTangentialVelocityScale = 1.0f;
        float rockGrabThrowMaxVelocityHavok = 12.0f;
        float rockGrabThrowAngularVelocityScale = 1.0f;
        float rockGrabThrowMaxAngularVelocityRadiansPerSecond = 18.0f;
        float rockGrabReleaseHandCollisionDelaySeconds = 0.10f;
        bool rockShoulderStashEnabled = true;
        bool rockShoulderStashUseBodyZoneColliders = true;
        bool rockShoulderStashUseHmdBackVolume = true;
        float rockShoulderStashEnterPaddingGameUnits = 5.0f;
        float rockShoulderStashExitPaddingGameUnits = 8.0f;
        float rockShoulderStashMinDwellSeconds = 0.08f;
        float rockShoulderStashMaxSpeedGameUnitsPerSecond = 140.0f;
        int rockShoulderStashRecentContactFrames = 4;
        int rockShoulderStashSustainedContactMissFrames = 18;
        RE::NiPoint3 rockShoulderStashHmdBackRightOffsetGameUnits = RE::NiPoint3(14.0f, -12.0f, -6.85f);
        RE::NiPoint3 rockShoulderStashHmdBackLeftOffsetGameUnits = RE::NiPoint3(-14.0f, -12.0f, -6.85f);
        float rockShoulderStashHmdBackRadiusGameUnits = 11.0f;
        bool rockShoulderStashSkipActivateBooks = true;
        bool rockShoulderStashSkipActivateNotes = true;
        bool rockShoulderStashShowCollectedNotifications = true;
        float rockGrabVelocityDamping = 0.25f;
        bool rockGrabPlayerSpaceCompensation = true;
        float rockGrabPlayerSpaceWarpDistance = 35.0f;
        float rockGrabPlayerSpaceWarpMinRotationDegrees = 0.6f;
        bool rockGrabPlayerSpaceTransformWarpEnabled = true;
        bool rockGrabResidualVelocityDamping = true;
        bool rockGrabNearbyDampingEnabled = true;
        float rockGrabNearbyDampingRadius = 90.0f;
        float rockGrabNearbyDampingSeconds = 0.35f;
        float rockGrabNearbyLinearDamping = 3.0f;
        float rockGrabNearbyAngularDamping = 5.5f;
        float rockGrabTouchAcquireDistanceGameUnits = 4.0f;
        float rockGrabNearConvergeDistanceGameUnits = 28.0f;
        float rockGrabPocketDepthGameUnits = 7.0f;
        float rockGrabPocketRadiusGameUnits = 9.0f;
        float rockGrabGripInsetGameUnits = 2.0f;
        float rockGrabGripMaxInsetGameUnits = 6.0f;
        float rockGrabConvergeMaxTimeSeconds = 0.35f;
        int rockGrabConvergeStableFrames = 3;
        float rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond = 40.0f;
        float rockGrabAcquisitionVisualStartDistanceGameUnits = 28.0f;
        bool rockGrabMultiFingerContactValidationEnabled = true;
        int rockGrabContactQualityMode = 1;
        int rockGrabMinFingerContactGroups = 3;
        float rockGrabMinFingerContactSpreadGameUnits = 1.0f;
        float rockGrabFingerContactMeshSnapMaxDistanceGameUnits = 10.0f;
        float rockGrabSurfaceBehindPalmToleranceGameUnits = 1.5f;
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
        bool rockGrabFingerRejectBacksideHits = true;
        float rockGrabFingerSurfacePlaneToleranceGameUnits = 1.5f;
        float rockGrabThumbOppositionStrength = 1.0f;
        float rockGrabThumbAlternateCurveStrength = 0.65f;
        bool rockGrabThumbSurfaceSafetyEnabled = true;
        float rockGrabThumbSurfaceSafetyMarginGameUnits = 1.0f;
        float rockGrabLateralWeight = 0.6f;
        float rockGrabDirectionalWeight = 0.4f;
        float rockGrabMaxTriangleDistance = 100.0f;
        bool rockGrabMeshContactOnly = true;
        bool rockGrabRequireMeshContact = true;
        bool rockGrabContactPatchEnabled = true;
        int rockGrabContactPatchProbeCount = 5;
        float rockGrabContactPatchProbeSpacingGameUnits = 3.0f;
        float rockGrabContactPatchProbeRadiusGameUnits = 2.0f;
        float rockGrabContactPatchMeshSnapMaxDistanceGameUnits = 6.0f;
        float rockGrabContactPatchMaxNormalAngleDegrees = 35.0f;
        float rockGrabAlignmentMaxSelectionToMeshDistance = 8.0f;
        bool rockGrabNodeAnchorsEnabled = true;
        bool rockGrabNodeRejectOppositeHandAnchor = true;
        bool rockPrintGrabNodeInfo = false;
        std::string rockGrabNodeNameRight = "ROCK:GrabR";
        std::string rockGrabNodeNameLeft = "ROCK:GrabL";
        std::string rockGrabNodeNameBlacklist = "ROCK:GrabR,ROCK:GrabL";
        bool rockSelectedCloseFingerCurlEnabled = true;
        float rockSelectedCloseFingerAnimMaxHandSpeed = 0.9f;
        float rockSelectedCloseFingerAnimValue = 0.9f;
        float rockPulledAngularDamping = 8.0f;
        float rockPulledGrabHandAdjustDistanceGameUnits = 10.5f;

        RE::NiPoint3 rockRightGrabPivotAHandspace = RE::NiPoint3(6.0f, 0.2f, -2.0f);
        RE::NiPoint3 rockLeftGrabPivotAHandspace = RE::NiPoint3(6.0f, -0.2f, -2.0f);

        float rockGrabLerpSpeed = 300.0f;
        float rockGrabLerpAngularSpeed = 360.0f;
        float rockGrabLerpMaxTime = 0.5f;

        bool rockGrabHapticsEnabled = true;
        float rockGrabHapticDurationSeconds = 0.055f;
        float rockGrabHapticBaseIntensity = 0.12f;
        float rockGrabHapticMaxIntensity = 0.80f;
        float rockGrabHapticMassScale = 0.06f;
        float rockGrabHapticMassExponent = 0.60f;
        float rockPullStartHapticIntensity = 0.18f;
        float rockPullCatchHapticIntensity = 0.22f;
        float rockSelectionLockHapticIntensity = 0.15f;
        float rockSelectionLockReleaseHapticIntensity = 0.10f;
        float rockSelectionLockReleaseHapticDurationSeconds = 0.02f;
        bool rockHeldImpactHapticsEnabled = true;
        float rockHeldImpactHapticDurationSeconds = 0.035f;
        float rockHeldImpactHapticBaseIntensity = 0.12f;
        float rockHeldImpactHapticMaxIntensity = 0.85f;
        float rockHeldImpactHapticSpeedScale = 0.006f;
        float rockHeldImpactHapticMassScale = 0.035f;
        float rockHeldImpactHapticMassExponent = 0.55f;
        float rockHeldImpactHapticMinSpeedGameUnits = 8.0f;
        float rockHeldImpactHapticCooldownSeconds = 0.12f;
        float rockHeldImpactHapticDampedMultiplier = 0.55f;
        bool rockShoulderStashHapticsEnabled = true;
        float rockShoulderStashCandidateHapticDurationSeconds = 0.075f;
        float rockShoulderStashCandidateHapticBaseIntensity = 0.20f;
        float rockShoulderStashCandidateHapticIntensity = 0.42f;
        float rockShoulderStashCandidateHapticIntervalSeconds = 0.075f;
        float rockShoulderStashCommitHapticDurationSeconds = 0.12f;
        float rockShoulderStashCommitHapticIntensity = 0.85f;

    private:
        void resetToDefaults();

        void readValuesFromIni(CSimpleIniA& ini);

        [[nodiscard]] bool saveRuntimeIni(CSimpleIniA& ini, const char* reason);

        void startFileWatch();

        std::string _iniFilePath;

        std::unique_ptr<filewatch::FileWatch<std::string>> _fileWatch;

        std::atomic<std::filesystem::file_time_type> _lastIniFileWriteTime;

        std::unordered_map<std::string, std::function<void(const std::string&)>> _onConfigChangedSubscribers;

        std::atomic<bool> _ignoreNextIniFileChange = false;

        std::atomic<bool> _selfIniWriteInProgress = false;

        std::atomic<std::filesystem::file_time_type> _lastSelfIniWriteTime{};

        std::atomic<bool> _reloadPending = false;

        std::thread _fileWatchInitThread;
    };

    inline RockConfig g_rockConfig;
}
