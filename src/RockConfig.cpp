

#include "RockConfig.h"

#include <ShlObj.h>
#include <SimpleIni.h>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <thread>
#include <unordered_set>

#include "rock_support/ResourceUtils.h"
#include "physics-interaction/grab/GrabNodeNamePolicy.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/grab/NearbyGrabDamping.h"
#include "physics-interaction/debug/DebugConfigPolicy.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/RockLoggingPolicy.h"
#include "resources.h"

namespace
{

    constexpr auto SECTION = "PhysicsInteraction";
    constexpr auto CONFIG_SECTION = "Config";
    constexpr int kCurrentConfigSchemaVersion = 1;
    constexpr auto CORE_SECTION = "Core";
    constexpr auto INPUT_SECTION = "Input";
    constexpr auto HAND_FRAME_SECTION = "HandFrame";
    constexpr auto WEAPON_COLLISION_SECTION = "WeaponCollision";
    constexpr auto WEAPON_HANDLING_SECTION = "WeaponHandling";
    constexpr auto DYNAMIC_HAND_COLLISION_SECTION = "DynamicHandCollision";
    constexpr auto NATIVE_SUPPRESSION_SECTION = "NativeSuppression";
    constexpr auto SELECTION_FEEDBACK_SECTION = "SelectionFeedback";
    constexpr auto DEBUG_SECTION = "Debug";
    constexpr auto DEBUG_OVERLAY_SECTION = "DebugOverlay";
    constexpr auto DEBUG_LOGGING_SECTION = "DebugLogging";
    constexpr auto DEBUG_SKELETON_SECTION = "DebugSkeleton";
    constexpr auto REALISTIC_WEAPONS_SECTION = "RealisticWeapons";
    constexpr auto WEAPON_HANDEDNESS_SECTION = "WeaponHandedness";
    constexpr auto GUNSTOCK_SECTION = "Gunstock";
    constexpr auto AMBIDEXTROUS_FIRING_SECTION = "AmbidextrousFiring";
    constexpr auto NATIVE_SCOPES_SECTION = "NativeScopes";
    constexpr auto EXPERIMENTAL_SECTION = "Experimental";
    constexpr int kDefaultWeaponCollisionSupportFitTargetPoints = 96;
    constexpr int kMinWeaponCollisionSupportFitTargetPoints = 4;
    constexpr int kMaxWeaponCollisionSupportFitTargetPoints = 252;
    constexpr int kDefaultWeaponCollisionVisualStabilizationFrames = 8;
    constexpr int kMaxWeaponCollisionVisualStabilizationFrames = 60;
    constexpr float kDefaultWeaponCollisionSupportFitMaxErrorGameUnits = 0.5f;
    constexpr float kDefaultWeaponCollisionDynamicInverseInertiaMultiplier = 1.2f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintLinearTauMultiplier = 1.0f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintAngularTauMultiplier = 1.0f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintCollisionTauMultiplier = 1.0f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintLinearDampingMultiplier = 1.0f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintAngularDampingMultiplier = 1.0f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintMaxForceMultiplier = 4.5f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintAngularForceMultiplier = 2.0f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier = 1.0f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier = 1.0f;
    constexpr float kMaxMouthConsumeHmdOffsetGameUnits = 120.0f;
    const RE::NiPoint3 kDefaultMouthConsumeHmdOffsetGameUnits{ 0.0f, 7.0f, -7.0f };
    constexpr float kDefaultGrabThrowObjectVelocityBlend = 0.35f;
    constexpr float kDefaultGrabThrowTangentialVelocityScale = 1.0f;
    constexpr float kDefaultGrabThrowMaxVelocityHavok = 12.0f;
    constexpr float kDefaultGrabThrowAngularVelocityScale = 1.0f;
    constexpr float kDefaultGrabThrowMaxAngularVelocityRadiansPerSecond = 18.0f;
    constexpr float kDefaultGrabLongObjectReferenceLeverGameUnits = 24.0f;
    constexpr float kDefaultGrabLongObjectMinAngularScale = 0.35f;
    constexpr float kDefaultGrabEffectiveMotorMassFloor = 2.0f;
    constexpr float kDefaultGrabPhysicsRateReferenceHz = 90.0f;
    constexpr float kDefaultGrabPhysicsRateForceScaleExponent = 0.5f;
    constexpr float kDefaultGrabPhysicsRateMinForceScale = 0.75f;
    constexpr float kDefaultGrabPhysicsRateMaxForceScale = 1.35f;
    constexpr float kDefaultGrabPositionOnlyAngularScale = 0.55f;
    constexpr float kDefaultGrabSmallObjectReferenceLeverGameUnits = 12.0f;
    constexpr float kDefaultGrabSmallObjectAngularScale = 0.65f;
    constexpr float kDefaultGrabLowContactSupportAngularScale = 0.75f;
    constexpr float kDefaultGrabMinAngularAuthorityScale = 0.30f;
    constexpr float kDefaultGrabWeakPivotTwistScale = 0.35f;
    constexpr float kDefaultGrabMinInertia = 0.01f;
    constexpr float kDefaultGrabThumbSurfaceSafetyMarginGameUnits = 1.0f;
    constexpr float kDefaultNearCastRadiusGameUnits = 3.5f;
    constexpr float kDefaultNearCastDistanceGameUnits = 7.0f;
    const RE::NiPoint3 kDefaultPalmNormalHandspace{ 0.0f, 1.0f, 0.0f };
    constexpr int kDefaultHighlightIntensityMode = 3;
    constexpr const char* kDefaultHighlightColor = "orange";

    class TrackedIni final
    {
    public:
        explicit TrackedIni(CSimpleIniA& ini) : _ini(ini) {}

        const char* GetValue(const char* section, const char* key, const char* fallback = nullptr, bool* hasMultiple = nullptr)
        {
            const char* readSection = chooseReadSection(section, key);
            return _ini.GetValue(readSection, key, fallback, hasMultiple);
        }

        long GetLongValue(const char* section, const char* key, long fallback = 0, bool* hasMultiple = nullptr)
        {
            const char* readSection = chooseReadSection(section, key);
            return _ini.GetLongValue(readSection, key, fallback, hasMultiple);
        }

        double GetDoubleValue(const char* section, const char* key, double fallback = 0.0, bool* hasMultiple = nullptr)
        {
            const char* readSection = chooseReadSection(section, key);
            return _ini.GetDoubleValue(readSection, key, fallback, hasMultiple);
        }

        bool GetBoolValue(const char* section, const char* key, bool fallback = false, bool* hasMultiple = nullptr)
        {
            const char* readSection = chooseReadSection(section, key);
            return _ini.GetBoolValue(readSection, key, fallback, hasMultiple);
        }

        void warnUnknownKeys() const
        {
            constexpr std::size_t kMaximumDetailedWarnings = 32;
            std::size_t unknownCount = 0;
            CSimpleIniA::TNamesDepend sections;
            _ini.GetAllSections(sections);
            for (const auto& section : sections) {
                CSimpleIniA::TNamesDepend keys;
                if (!_ini.GetAllKeys(section.pItem, keys)) {
                    continue;
                }
                for (const auto& key : keys) {
                    if (!_recognized.contains(normalizedIdentity(section.pItem, key.pItem))) {
                        if (unknownCount < kMaximumDetailedWarnings) {
                            ROCK_LOG_WARN(Config, "Unknown ROCK.ini setting [{}] {}; ignored", section.pItem, key.pItem);
                        }
                        ++unknownCount;
                    }
                }
            }
            if (unknownCount > kMaximumDetailedWarnings) {
                ROCK_LOG_WARN(Config,
                    "ROCK.ini contains {} additional unknown settings; all were ignored",
                    unknownCount - kMaximumDetailedWarnings);
            }
            if (!_legacyPhysicsInteractionKeys.empty()) {
                ROCK_LOG_WARN(Config,
                    "ROCK.ini uses {} legacy [PhysicsInteraction] settings that now have dedicated sections; values remain active for schema 1",
                    _legacyPhysicsInteractionKeys.size());
            }
        }

    private:
        [[nodiscard]] static std::string normalizedIdentity(const char* section, const char* key)
        {
            std::string identity;
            if (section) {
                identity.append(section);
            }
            identity.push_back('\x1f');
            if (key) {
                identity.append(key);
            }
            std::transform(identity.begin(), identity.end(), identity.begin(), [](const unsigned char character) {
                return static_cast<char>(std::tolower(character));
            });
            return identity;
        }

        void record(const char* section, const char* key)
        {
            _recognized.insert(normalizedIdentity(section, key));
        }

        [[nodiscard]] static bool startsWith(const std::string_view value, const std::string_view prefix)
        {
            return value.starts_with(prefix);
        }

        [[nodiscard]] static const char* canonicalSection(const char* requestedSection, const char* key)
        {
            if (!requestedSection || std::strcmp(requestedSection, SECTION) != 0 || !key) {
                return requestedSection;
            }

            const std::string_view name(key);
            if (name == "bEnabled" || startsWith(name, "bHavokTimingFix") || startsWith(name, "fHavokTimingFix") || startsWith(name, "iHavokTimingFix")) {
                return CORE_SECTION;
            }
            if (startsWith(name, "bInputRemap") || startsWith(name, "bSuppress") || startsWith(name, "fPipboyPause") ||
                startsWith(name, "sSuppressTake") || startsWith(name, "bGrabInputIntent") || startsWith(name, "fGrabInputLeeway") ||
                startsWith(name, "fGrabInputForce")) {
                return INPUT_SECTION;
            }
            if (startsWith(name, "fPalmNormal") || startsWith(name, "fPointingVector") || startsWith(name, "bReversePalm") ||
                startsWith(name, "bReverseFar") || startsWith(name, "fRightGrabLegacyPalm") || startsWith(name, "fLeftGrabLegacyPalm") ||
                startsWith(name, "fRightGrabAuthorityProxy") || startsWith(name, "fLeftGrabAuthorityProxy") ||
                startsWith(name, "fRightCustomOGA") || startsWith(name, "fLeftCustomOGA")) {
                return HAND_FRAME_SECTION;
            }
            if (startsWith(name, "bWeaponCollision") || startsWith(name, "fWeaponCollision") || startsWith(name, "iWeaponCollision") ||
                startsWith(name, "fWeaponSizeClass")) {
                return WEAPON_COLLISION_SECTION;
            }
            if (startsWith(name, "fWeaponInteraction") || startsWith(name, "fWeaponAuthoredGrip") || startsWith(name, "fFiringGripProximity") ||
                startsWith(name, "bWeaponSupport") || startsWith(name, "fWeaponSupport") || startsWith(name, "bWeaponVisualReturn") ||
                startsWith(name, "fWeaponVisualReturn")) {
                return WEAPON_HANDLING_SECTION;
            }
            if (startsWith(name, "bHandCollisionDynamic") || startsWith(name, "fHandCollisionDynamic") || startsWith(name, "iHandCollisionDynamic") ||
                startsWith(name, "bHandDynamicInteractions") || startsWith(name, "bDynamicCollidersNpc") ||
                startsWith(name, "bHandCollisionSurface") || startsWith(name, "fHandCollisionSurface")) {
                return DYNAMIC_HAND_COLLISION_SECTION;
            }
            if (startsWith(name, "bNativeMelee") || startsWith(name, "bNativeCharacterController")) {
                return NATIVE_SUPPRESSION_SECTION;
            }
            if (startsWith(name, "bHighlight") || startsWith(name, "iHighlight") || startsWith(name, "sHighlight") ||
                startsWith(name, "bSelectionBeam") || startsWith(name, "fSelectionBeam")) {
                return SELECTION_FEEDBACK_SECTION;
            }
            return requestedSection;
        }

        const char* chooseReadSection(const char* requestedSection, const char* key)
        {
            const char* preferredSection = canonicalSection(requestedSection, key);
            record(preferredSection, key);
            if (preferredSection == requestedSection || _ini.GetValue(preferredSection, key, nullptr)) {
                return preferredSection;
            }
            if (_ini.GetValue(requestedSection, key, nullptr)) {
                // Schema 1 compatibility only. Remove this fallback when the
                // runtime and canonical template advance to schema 2.
                record(requestedSection, key);
                _legacyPhysicsInteractionKeys.insert(normalizedIdentity(requestedSection, key));
                return requestedSection;
            }
            return preferredSection;
        }

        CSimpleIniA& _ini;
        std::unordered_set<std::string> _recognized;
        std::unordered_set<std::string> _legacyPhysicsInteractionKeys;
    };

    std::string resolveIniPath()
    {
        char documents[MAX_PATH];
        if (SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_MYDOCUMENTS, nullptr, 0, documents))) {
            return std::string(documents) + R"(\My Games\Fallout4VR\ROCK_Config\ROCK.ini)";
        }

        ROCK_LOG_WARN(Config, "SHGetFolderPath failed — using fallback ROCK.ini path");
        return R"(Data\F4SE\Plugins\ROCK.ini)";
    }

    template <class Ini>
    float readClampedFloat(Ini& ini, const char* section, const char* key, float currentValue, float fallback, float minValue, float maxValue)
    {
        float value = static_cast<float>(ini.GetDoubleValue(section, key, currentValue));
        if (!std::isfinite(value)) {
            ROCK_LOG_WARN(Config, "Invalid {}={} -- using {:.2f}", key, value, fallback);
            value = fallback;
        }
        return std::clamp(value, minValue, maxValue);
    }

    template <class Ini>
    int readSelectionAimAngleDegrees(Ini& ini, const char* section, const char* key, int currentValue)
    {
        const int configuredValue = static_cast<int>(ini.GetLongValue(section, key, currentValue));
        const int sanitizedValue = rock::selection_query_policy::sanitizeSelectionAimAngleDegrees(configuredValue);
        if (configuredValue != sanitizedValue) {
            ROCK_LOG_WARN(Config, "Invalid {}={} -- using {}", key, configuredValue, sanitizedValue);
        }
        return sanitizedValue;
    }

    template <class Ini>
    int readHighlightIntensityMode(Ini& ini, const char* section, const char* key, int currentValue)
    {
        const int configuredValue = static_cast<int>(ini.GetLongValue(section, key, currentValue));
        if (configuredValue >= 1 && configuredValue <= 4) {
            return configuredValue;
        }

        ROCK_LOG_WARN(Config, "Invalid {}={} -- using {}", key, configuredValue, kDefaultHighlightIntensityMode);
        return kDefaultHighlightIntensityMode;
    }

    template <class Ini>
    std::string readHighlightColor(Ini& ini, const char* section, const char* key, const std::string& currentValue)
    {
        std::string configuredValue = ini.GetValue(section, key, currentValue.c_str());
        for (auto& ch : configuredValue) {
            if (ch >= 'A' && ch <= 'Z') {
                ch = static_cast<char>(ch - 'A' + 'a');
            }
        }

        if (configuredValue == "red" || configuredValue == "blue" || configuredValue == "orange" || configuredValue == "white") {
            return configuredValue;
        }

        ROCK_LOG_WARN(Config, "Invalid {}='{}' -- using {}", key, configuredValue, kDefaultHighlightColor);
        return kDefaultHighlightColor;
    }
}

namespace rock
{

    void RockConfig::resetToDefaults()
    {
        rockEnabled = true;
        rockHavokTimingFixEnabled = true;
        rockHavokTimingFixMinPhysicsFrameRate = havok_timing_fix_policy::kDefaultMinPhysicsFrameRate;
        rockHavokTimingFixMaxSubsteps = havok_timing_fix_policy::kDefaultMaxSubsteps;

        rockSuppressRightFavoritesGameInput = true;
        rockSuppressPipboyGameInputWhileHolding = true;
        rockPipboyPauseHoldSeconds = pipboy_pause_gesture_policy::kDefaultHoldSeconds;
        rockSuppressTakeEquipGameInputWhileHolding = true;
        rockSuppressTakeEquipFormTypes = "WEAP,ARMO,AMMO,MISC,INGR,ALCH,BOOK,KEYM,SLGM";
        rockSuppressNativeGrabHoverHaptics = true;
        rockGrabInputIntentStateEnabled = true;
        rockGrabInputLeewaySeconds = 0.12f;
        rockGrabInputForceSeconds = 0.08f;

        rockSavedGrabOffsetRecordingEnabled = false;
        rockLogLevel = logging_policy::DefaultLogLevel;
        rockLogPattern = logging_policy::DefaultLogPattern;
        rockLogSampleMilliseconds = logging_policy::DefaultLogSampleMilliseconds;
        rockDebugEnabled = true;
        rockDebugOverlayEnabled = true;
        rockDebugLoggingEnabled = true;
        rockDebugControllerEnabled = false;
        rockDebugMonitorEnabled = false;
        rockDebugProviderOverlayEnabled = true;
        rockDebugProviderColliderFocusEnabled = true;
        rockDebugColliderClockLogging = false;
        rockDebugDynamicWeaponLogging = false;
        rockPerformanceProfilerEnabled = false;
        rockPerformanceProfilerLogIntervalFrames = 300;
        rockPerformanceProfilerWarmupFrames = 120;
        rockPerformanceProfilerOverlayText = false;

        rockPalmNormalHandspace = kDefaultPalmNormalHandspace;
        rockPointingVectorHandspace = RE::NiPoint3(0.0f, 1.0f, 0.0f);
        rockReversePalmNormal = true;
        rockReverseFarGrabNormal = true;

        rockLeftHandedMode = false;
        rockGunstockModeEnabled = false;
        rockGunstockAlignmentPitchDegrees = 0.0f;
        rockGunstockAlignmentYawDegrees = 0.0f;
        rockGunstockAlignmentRollDegrees = 0.0f;
        rockAmbidextrousFiringGripEnabled = true;
        rockFiringGripPromotionRadius = 5.0f;
        rockLeftFiringAimYawDegrees = 0.0f;
        rockLeftFiringAimPitchDegrees = 0.0f;
        rockLeftFiringAimOffsetXGameUnits = 0.0f;
        rockLeftFiringAimOffsetYGameUnits = 0.0f;
        rockLeftFiringAimOffsetZGameUnits = 0.0f;
        rockWeaponCollisionEnabled = true;
        rockWeaponCollisionBlocksProjectiles = false;
        rockWeaponCollisionBlocksSpells = false;
        rockWeaponCollisionStaticWorldEnabled = true;
        rockWeaponCollisionDynamicBoxEnabled = true;
        rockWeaponCollisionDynamicBoxPaddingGameUnits = 0.5f;
        rockWeaponCollisionDynamicInverseInertiaMultiplier = kDefaultWeaponCollisionDynamicInverseInertiaMultiplier;
        rockWeaponCollisionDynamicDivergenceTeleportGameUnits = 80.0f;
        rockWeaponCollisionDynamicDivergenceTeleportDwellSeconds = 0.3f;
        rockWeaponCollisionDynamicRenderMinTranslationGameUnits = 0.05f;
        rockWeaponCollisionDynamicRenderMinRotationDegrees = 0.25f;
        rockWeaponCollisionGroupingMode = weapon_collision_grouping_policy::kDefaultWeaponCollisionGroupingMode;
        rockWeaponCollisionVisualStabilizationFrames = kDefaultWeaponCollisionVisualStabilizationFrames;
        rockWeaponCollisionConvexRadius = 0.01f;
        rockWeaponCollisionPointDedupGrid = 0.002f;
        rockWeaponCollisionSupportFitTargetPoints = kDefaultWeaponCollisionSupportFitTargetPoints;
        rockWeaponCollisionSupportFitMaxErrorGameUnits = kDefaultWeaponCollisionSupportFitMaxErrorGameUnits;
        rockWeaponSizeClassPistolMaxWeight = 6.0f;
        rockWeaponSizeClassRifleMaxWeight = 20.0f;
        rockWeaponInteractionTouchRadius = 2.0f;
        rockWeaponInteractionProbeRadius = 12.0f;
        rockWeaponAuthoredGripActivationRadius = 16.0f;
        rockFiringGripProximitySupportRadius = 6.0f;
        rockRealisticGrenadeFuseSeconds = 5.0f;
        rockWeaponSupportGripHandLerpEnabled = true;
        rockWeaponSupportGripHandLerpTimeMin = 0.12f;
        rockWeaponSupportGripHandLerpTimeMax = 0.20f;
        rockWeaponSupportGripHandLerpMinDistance = 1.0f;
        rockWeaponSupportGripHandLerpMaxDistance = 14.0f;
        rockWeaponSupportSurfaceSeatEnabled = true;
        rockWeaponSupportSurfaceSeatMaxDegrees = 35.0f;
        rockWeaponVisualReturnEnabled = true;
        rockWeaponVisualReturnTimeMin = 0.12f;
        rockWeaponVisualReturnTimeMax = 0.20f;
        rockWeaponVisualReturnMinDistance = 1.0f;
        rockWeaponVisualReturnMaxDistance = 14.0f;
        rockWeaponVisualReturnMinAngleDegrees = 5.0f;
        rockWeaponVisualReturnMaxAngleDegrees = 90.0f;
        rockManualScopeHoldSeconds = 0.30f;
        rockNativeScopeForceFiringGripFallback = false;
        rockNativeScopeFiringGripFallbackOffsetXGameUnits = 0.0f;
        rockNativeScopeFiringGripFallbackOffsetYGameUnits = 0.0f;
        rockNativeScopeFiringGripFallbackOffsetZGameUnits = 0.0f;
        rockNativeScopeFiringGripFallbackPitchDegrees = 0.0f;
        rockNativeScopeFiringGripFallbackYawDegrees = 0.0f;
        rockNativeScopeFiringGripFallbackRollDegrees = 0.0f;
        rockNativeScopeOverlayOffsetXGameUnits = 0.0f;
        rockNativeScopeOverlayOffsetYGameUnits = 0.0f;
        rockNativeScopeOverlayOffsetZGameUnits = 0.0f;
        rockNativeScopeOverlayPitchDegrees = 0.0f;
        rockNativeScopeOverlayYawDegrees = 0.0f;
        rockNativeScopeOverlayRollDegrees = 0.0f;

        rockHandCollisionDynamicDrive = true;
        rockHandDynamicInteractionsEnabled = true;
        rockDynamicColliderNpcBodyCollisionEnabled = false;
        rockHandCollisionDynamicMaxLinearVelocityHavok = 15.0f;
        rockHandCollisionDynamicContactPressMaxVelocityHavok = 1.0f;
        rockHandCollisionDynamicCompoundMass = 2.0f;
        rockHandCollisionDynamicInverseInertiaMultiplier = 1.0f;
        rockHandCollisionDynamicDivergenceTeleportGameUnits = 40.0f;
        rockHandCollisionDynamicDivergenceTeleportDwellSeconds = 0.3f;
        rockHandCollisionDynamicTeleportRecoverySeconds = 0.25f;
        rockHandCollisionDynamicRenderFollowMinDeviationGameUnits = 0.05f;
        rockHandCollisionDynamicRenderFollowSmoothingSpeed = 45.0f;
        rockHandCollisionDynamicVisualPriority = 80;
        rockHandCollisionDynamicHapticsEnabled = true;
        rockHandCollisionDynamicHapticDurationSeconds = 0.035f;
        rockHandCollisionDynamicHapticBaseIntensity = 0.18f;
        rockHandCollisionDynamicHapticMaxIntensity = 0.55f;
        rockHandCollisionDynamicHapticSpeedScale = 0.006f;
        rockHandCollisionDynamicHapticMinApproachSpeedGameUnitsPerSecond = 3.0f;
        rockHandCollisionDynamicHapticCooldownSeconds = 0.12f;
        rockHandCollisionSurfaceFingerResponseEnabled = true;
        rockHandCollisionSurfaceFingerProbeDeltaOpenUnits = 0.10f;
        rockHandCollisionSurfaceFingerResponseGain = 1.0f;
        rockHandCollisionSurfaceFingerMaximumDeflectionOpenUnits = 0.85f;
        rockHandCollisionSurfaceFingerMinimumHelpfulTravelGameUnits = 0.01f;
        rockHandCollisionSurfaceFingerSmoothingSpeed = 30.0f;
        rockHandCollisionSurfaceFingerReleaseDelaySeconds = 0.12f;

        rockNativeMeleeSuppressionEnabled = false;
        rockNativeMeleeFullSuppression = false;
        rockNativeMeleeSuppressWeaponSwing = false;
        rockNativeMeleeSuppressHitFrame = false;
        rockNativeMeleeDebugLogging = false;
        rockNativeCharacterControllerObjectContactFilterEnabled = true;

        rockHighlightEnabled = true;
        rockHighlightIntensityMode = kDefaultHighlightIntensityMode;
        rockHighlightColor = kDefaultHighlightColor;
        rockSelectionBeamEnabled = true;
        rockSelectionBeamSegmentSizeGameUnits = selection_beam_policy::kDefaultSegmentSizeGameUnits;
        rockSelectionBeamCurveLiftGameUnits = selection_beam_policy::kDefaultCurveLiftGameUnits;
        rockSelectionBeamAlpha = selection_beam_policy::kDefaultAlpha;

        rockDebugShowColliders = true;
        rockDebugShowTargetColliders = false;
        rockDebugShowHandAxes = false;
        rockDebugShowGrabPivots = false;
        rockDebugShowGrabPocketNormal = false;
        rockDebugDrawGrabContactPatch = false;
        rockDebugDrawGrabForceTorque = false;
        rockDebugDrawGrabForceTorqueText = false;
        rockDebugDrawGrabPivotSourceCollider = false;
        rockDebugDrawGrabPivotSourceEvidence = false;
        rockDebugDrawGrabSupportFrame = false;
        rockDebugDrawGrabPockets = false;
        rockDebugShowGrabFingerProbes = false;
        rockDebugShowGrabFingerSweptArc = false;
        rockDebugShowGrabFingerSweptArcText = false;
        rockDebugShowGrabFingerSweptArcLiveSkeleton = false;
        rockDebugShowPalmVectors = false;
        rockDebugDrawHandColliders = false;
        rockDebugDrawHandBoneColliders = false;
        rockDebugDrawDynamicHandColliders = false;
        rockDebugDrawHandBoneContacts = false;
        rockDebugDrawGrabAuthorityProxy = false;
        rockDebugMaxHandBoneBodiesDrawn = 48;
        rockDebugMaxBodyBoneBodiesDrawn = 32;
        rockDebugDrawWeaponColliders = false;
        rockDebugDrawNativeScopeActivation = false;
        rockDebugDrawAuthoredGripActivationZones = false;
        rockDebugDrawGunstockAlignment = false;
        rockDebugDrawDynamicWeaponColliders = true;
        rockDebugDumpWeaponAnimNodes = false;
        rockDebugMaxWeaponBodiesDrawn = 100;
        rockDebugWeaponAnimNodeDumpIntervalFrames = 120;
        rockDebugMaxShapeCapturesPerFrame = static_cast<int>(debug_overlay_runtime::kDefaultMaxShapeCapturesPerFrame);
        rockDebugMaxConvexSupportVertices = 8;
        rockDebugMaxCompoundChildren = static_cast<int>(debug_overlay_policy::kDefaultMaxCompoundChildren);
        rockDebugMaxCompoundDepth = static_cast<int>(debug_overlay_policy::kDefaultMaxCompoundDepth);
        rockDebugMaxShapeQueuedJobs = static_cast<int>(debug_overlay_runtime::kDefaultMaxShapeQueuedJobs);
        rockDebugMaxShapeCompletedJobs = static_cast<int>(debug_overlay_runtime::kDefaultMaxShapeCompletedJobs);
        rockDebugMaxShapeUploadsPerFrame = static_cast<int>(debug_overlay_runtime::kDefaultMaxShapeUploadsPerFrame);
        rockDebugMaxShapeCacheEntries = static_cast<int>(debug_overlay_policy::kDefaultShapeCacheBudget);
        rockDebugMaxShapeCacheBytes = static_cast<int>(debug_overlay_runtime::kDefaultMaxShapeCacheBytes);
        rockDebugMaxBodyInstances = static_cast<int>(debug_overlay_runtime::kDefaultMaxBodyInstances);
        rockDebugMaxLineVertices = static_cast<int>(debug_overlay_policy::kDefaultLineVertexBudget);
        rockDebugMaxTextVertices = static_cast<int>(debug_overlay_runtime::kDefaultMaxTextVertices);
        rockDebugUseBoundsForHeavyConvex = true;
        rockDebugVerboseLogging = false;
        rockDebugGrabFrameLogging = false;
        rockDebugVideoSyncMarker = false;
        rockDebugVideoSyncMarkerSize = 4.0f;
        rockDebugGrabTimelineTrace = false;
        rockDebugGrabAfterSolveAnomalySampling = false;
        rockDebugGrabTransformTelemetry = false;
        rockDebugGrabTransformTelemetryText = false;
        rockDebugGrabTransformTelemetryAxes = false;
        rockDebugGrabTimelineTraceIntervalFrames = 1;
        rockDebugGrabTransformTelemetryLogIntervalFrames = 1;
        rockDebugGrabTransformTelemetryTextMode = 0;
        rockDebugShowGrabNotifications = false;
        rockDebugShowWeaponNotifications = false;
        rockDebugWeaponOmodDumpEnabled = true;
        rockDebugWeaponOmodCoverageAudit = true;
        rockDebugWeaponOmodCoverageAuditIntervalFrames = 450;
        rockDebugHandTransformParity = false;
        rockDebugWorldObjectOriginDiagnostics = false;
        rockDebugWorldObjectOriginLogIntervalFrames = 120;
        rockDebugWorldObjectOriginMismatchWarnGameUnits = 5.0f;
        rockDebugCustomCalibrationOffset = false;
        rockDebugShowRootFlattenedFingerSkeletonMarkers = false;
        rockDebugShowSkeletonBoneVisualizer = true;
        rockDebugDrawSkeletonBoneAxes = false;
        rockDebugLogSkeletonBones = false;
        rockDebugSkeletonBoneMode = 1;
        rockDebugSkeletonBoneSource = 1;
        rockDebugMaxSkeletonBonesDrawn = 256;
        rockDebugMaxSkeletonBoneAxesDrawn = 80;
        rockDebugSkeletonBoneLogIntervalFrames = 120;
        rockDebugLogSkeletonBoneTruncation = false;
        rockDebugRootFlattenedFingerSkeletonMarkerSize = 1.4f;
        rockDebugSkeletonBonePointSize = 1.4f;
        rockDebugSkeletonBoneAxisLength = 4.0f;
        rockDebugSkeletonBoneLogFilter = "RArm_Hand,LArm_Hand,RArm_Finger23,LArm_Finger23,Chest,Pelvis";
        rockDebugSkeletonAxisBoneFilter = "";

        rockHandColliderRuntimeMode = 1;
        rockBodyBoneCollidersEnabled = true;
        rockBodyBoneLegAndFootCollidersEnabled = false;
        rockBodyBoneCollisionStaticWorldEnabled = true;
        rockBodyBoneColliderStandardRadiusScale = 1.0f;
        rockBodyBoneColliderStandardLengthScale = 1.0f;
        rockBodyBoneColliderStandardConvexRadiusScale = 1.0f;
        rockBodyBoneColliderPowerArmorRadiusScale = 1.0f;
        rockBodyBoneColliderPowerArmorLengthScale = 1.0f;
        rockBodyBoneColliderPowerArmorConvexRadiusScale = 1.0f;
        rockBodyBoneColliderTorsoRadiusScale = 1.0f;
        rockBodyBoneColliderArmRadiusScale = 1.0f;
        rockBodyBoneColliderLegRadiusScale = 1.0f;
        rockBodyBoneColliderFootRadiusScale = 1.0f;
        rockBodyBoneColliderTorsoLengthScale = 1.0f;
        rockBodyBoneColliderArmLengthScale = 1.0f;
        rockBodyBoneColliderLegLengthScale = 1.0f;
        rockBodyBoneColliderFootLengthScale = 1.0f;
        rockBodyBoneColliderZoneScaleOverrides = "";
        rockBodyBoneColliderRadiusScaleOverrides = "";
        rockHandCollisionStaticWorldEnabled = true;
        rockGlobalSurfaceGrabEnabled = true;
        rockExperimentalSurfaceMeshGrabEnabled = true;
        rockExperimentalSurfaceMeshGrabMaxProjectionDistanceGameUnits = 48.0f;
        rockExperimentalSurfaceMeshGrabMaxTriangles = 20000;
        rockExperimentalSurfaceMeshGrabMaxPatchTriangles = 2048;
        rockHandBoneColliderRadiusScaleOverrides = "";
        rockHandPalmColliderDimensionScaleOverrides = "";
        rockHandBoneCollidersRequirePalmAnchor = true;
        rockHandBoneCollidersRequireAllFingerBones = true;
        rockNearDetectionRange = 25.0f;
        rockFarDetectionRange = 350.0f;
        rockNearCastRadiusGameUnits = kDefaultNearCastRadiusGameUnits;
        rockNearCastDistanceGameUnits = kDefaultNearCastDistanceGameUnits;
        rockFarCastRadiusGameUnits = 21.0f;
        rockCloseSelectionAngleDegrees = selection_query_policy::kDefaultSelectionAimAngleDegrees;
        rockFarSelectionAngleDegrees = selection_query_policy::kDefaultSelectionAimAngleDegrees;
        rockFarSelectionHmdConeEnabled = true;
        rockFarSelectionHmdConeHalfAngleDegrees = selection_query_policy::kDefaultFarSelectionHmdConeHalfAngleDegrees;
        rockFarSelectionBlockedReferenceFormIds.clear();
        rockFarSelectionBlockedBaseFormIds.clear();
        rockFarSelectionBlockedFormTypes.clear();
        rockFarSelectionBlockedLayers.clear();
        rockCloseSelectionBehindPalmToleranceGameUnits = 2.0f;
        rockSelectionShapeCastFilterInfo = selection_query_policy::kDefaultShapeCastFilterInfo;
        rockFarClipRayFilterInfo = selection_query_policy::kDefaultFarClipRayFilterInfo;
        rockPullApplyVelocityTime = 0.2f;
        rockPullOwnerGraceSeconds = 1.0f;
        rockPullTrackHandTime = 0.1f;
        rockPullDestinationZOffsetHavok = 0.01f;
        rockPullDurationA = 0.715619f;
        rockPullDurationB = -0.415619f;
        rockPullDurationC = 0.656256f;
        rockPullMaxVelocityHavok = 10.0f;
        rockPullAutoGrabDistanceGameUnits = 18.0f;
        rockPullCatchRetryMaxTimeSeconds = 0.65f;
        rockPullCatchWideReacquireEnabled = true;
        rockPullCatchWideReacquireRadiusGameUnits = 32.0f;
        rockPullCatchWideReacquireMaxBodyDistanceGameUnits = 42.0f;
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
        rockGrabMaxForceToMassRatio = 500.0f;
        rockForceGrabAttachSettleSeconds = 0.10f;
        rockGrabEffectiveMotorMassFloorEnabled = true;
        rockGrabEffectiveMotorMassFloor = kDefaultGrabEffectiveMotorMassFloor;
        rockGrabPhysicsRateForceScalingEnabled = true;
        rockGrabPhysicsRateReferenceHz = kDefaultGrabPhysicsRateReferenceHz;
        rockGrabPhysicsRateForceScaleExponent = kDefaultGrabPhysicsRateForceScaleExponent;
        rockGrabPhysicsRateMinForceScale = kDefaultGrabPhysicsRateMinForceScale;
        rockGrabPhysicsRateMaxForceScale = kDefaultGrabPhysicsRateMaxForceScale;
        rockGrabForceFadeInTime = 0.1f;
        rockRightGrabAuthorityProxyOffsetGameUnits = RE::NiPoint3(0.0f, -2.0f, 0.0f);
        rockLeftGrabAuthorityProxyOffsetGameUnits = RE::NiPoint3(0.0f, -2.0f, 0.0f);
        rockRightCustomOGAOffsetGameUnits = RE::NiPoint3(0.0f, -2.0f, 0.0f);
        rockLeftCustomOGAOffsetGameUnits = RE::NiPoint3(0.0f, -2.0f, 0.0f);
        rockGrabLooseWeaponSharedConstraintLinearTauMultiplier = kDefaultGrabLooseWeaponSharedConstraintLinearTauMultiplier;
        rockGrabLooseWeaponSharedConstraintAngularTauMultiplier = kDefaultGrabLooseWeaponSharedConstraintAngularTauMultiplier;
        rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier = kDefaultGrabLooseWeaponSharedConstraintCollisionTauMultiplier;
        rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier = kDefaultGrabLooseWeaponSharedConstraintLinearDampingMultiplier;
        rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier = kDefaultGrabLooseWeaponSharedConstraintAngularDampingMultiplier;
        rockGrabLooseWeaponSharedConstraintMaxForceMultiplier = kDefaultGrabLooseWeaponSharedConstraintMaxForceMultiplier;
        rockGrabLooseWeaponSharedConstraintAngularForceMultiplier = kDefaultGrabLooseWeaponSharedConstraintAngularForceMultiplier;
        rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier = kDefaultGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier;
        rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier = kDefaultGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier;
        rockGrabTauMin = 0.01f;
        rockGrabTauLerpSpeed = 0.5f;
        rockGrabLongObjectAngularScalingEnabled = true;
        rockGrabLongObjectReferenceLeverGameUnits = kDefaultGrabLongObjectReferenceLeverGameUnits;
        rockGrabLongObjectMinAngularScale = kDefaultGrabLongObjectMinAngularScale;
        rockGrabPivotQualityAngularScalingEnabled = true;
        rockGrabPositionOnlyAngularScale = kDefaultGrabPositionOnlyAngularScale;
        rockGrabSmallObjectReferenceLeverGameUnits = kDefaultGrabSmallObjectReferenceLeverGameUnits;
        rockGrabSmallObjectAngularScale = kDefaultGrabSmallObjectAngularScale;
        rockGrabLowContactSupportAngularScale = kDefaultGrabLowContactSupportAngularScale;
        rockGrabMinAngularAuthorityScale = kDefaultGrabMinAngularAuthorityScale;
        rockGrabWeakPivotTwistScale = kDefaultGrabWeakPivotTwistScale;

        rockGrabMaxInertiaRatio = 10.0f;
        rockGrabMinInertia = kDefaultGrabMinInertia;

        rockGrabMaxDeviation = 50.0f;
        rockGrabMaxDeviationTime = 2.0f;
        rockGrabButtonID = 2;
        rockThrowVelocityMultiplier = 1.5f;
        rockGrabControllerDerivedThrowVelocityEnabled = true;
        rockGrabThrowObjectVelocityBlend = kDefaultGrabThrowObjectVelocityBlend;
        rockGrabThrowTangentialVelocityScale = kDefaultGrabThrowTangentialVelocityScale;
        rockGrabThrowMaxVelocityHavok = kDefaultGrabThrowMaxVelocityHavok;
        rockGrabThrowAngularVelocityScale = kDefaultGrabThrowAngularVelocityScale;
        rockGrabThrowMaxAngularVelocityRadiansPerSecond = kDefaultGrabThrowMaxAngularVelocityRadiansPerSecond;
        rockGrabReleaseHandCollisionDelaySeconds = 0.10f;
        rockShoulderStashEnabled = true;
        rockEquippedWeaponShoulderStashEnabled = true;
        rockShoulderStashUseBodyZoneColliders = true;
        rockShoulderStashUseHmdBackVolume = true;
        rockShoulderStashEnterPaddingGameUnits = 5.0f;
        rockShoulderStashExitPaddingGameUnits = 8.0f;
        rockShoulderStashMinDwellSeconds = 0.08f;
        rockShoulderStashMaxSpeedGameUnitsPerSecond = 140.0f;
        rockShoulderStashRecentContactFrames = 4;
        rockShoulderStashSustainedContactMissFrames = 18;
        rockShoulderStashHmdBackRightOffsetGameUnits = RE::NiPoint3(14.0f, -18.0f, -6.85f);
        rockShoulderStashHmdBackLeftOffsetGameUnits = RE::NiPoint3(-14.0f, -18.0f, -6.85f);
        rockShoulderStashHmdBackRadiusGameUnits = 11.0f;
        rockShoulderStashHmdBackEnterPaddingGameUnits = 0.0f;
        rockShoulderStashHmdBackExitPaddingGameUnits = 2.0f;
        rockShoulderStashHmdBackMinBehindGameUnits = 4.0f;
        rockShoulderStashShowCollectedNotifications = true;
        rockMouthConsumeEnabled = true;
        rockMouthConsumeAllowPoison = false;
        rockMouthConsumeHmdOffsetGameUnits = RE::NiPoint3(0.0f, 7.0f, -7.0f);
        rockMouthConsumeRadiusGameUnits = 5.5f;
        rockMouthConsumeEnterPaddingGameUnits = 0.0f;
        rockMouthConsumeExitPaddingGameUnits = 1.0f;
        rockMouthConsumeMinDwellSeconds = 0.08f;
        rockMouthConsumeMaxSpeedGameUnitsPerSecond = 120.0f;
        rockGrabNearbyDampingEnabled = true;
        rockGrabNearbyDampingRadius = 90.0f;
        rockGrabNearbyDampingSeconds = 0.35f;
        rockGrabNearbyLinearDamping = 3.0f;
        rockGrabNearbyAngularDamping = 5.5f;
        rockGrabHeldMassMovementSlowdownEnabled = true;
        rockGrabHeldMassMovementMassProportion = 0.675f;
        rockGrabHeldMassMovementMassExponent = 1.0f;
        rockGrabHeldMassMovementMaxReduction = 75.0f;
        rockGrabHeldMassMovementFadeOutSeconds = 5.0f;
        rockGrabTouchAcquireDistanceGameUnits = 4.0f;
        rockGrabNearConvergeDistanceGameUnits = 28.0f;
        rockGrabPocketDepthGameUnits = 7.0f;
        rockGrabPocketRadiusGameUnits = 9.0f;
        rockGrabSeatDepthMaxGameUnits = 30.0f;
        rockGrabSeatDepthFootprintRadiusGameUnits = 10.0f;
        rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits = 6.0f;
        rockGrabSeatDepthSkinGameUnits = 0.5f;
        rockGrabGripInsetGameUnits = 2.0f;
        rockGrabGripMaxInsetGameUnits = 6.0f;
        rockGrabConvergeMaxTimeSeconds = 0.35f;
        rockGrabConvergeStableFrames = 3;
        rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond = 40.0f;
        rockGrabAcquisitionVisualStartDistanceGameUnits = 28.0f;
        rockGrabMultiFingerContactValidationEnabled = true;
        rockGrabContactQualityMode = 1;
        rockGrabMinFingerContactGroups = 3;
        rockGrabMinFingerContactSpreadGameUnits = 1.0f;
        rockGrabFingerContactMeshSnapMaxDistanceGameUnits = 10.0f;
        rockGrabSurfaceBehindPalmToleranceGameUnits = 1.5f;
        rockGrabOppositionContactMaxAgeFrames = 5;
        rockGrabPinchPocketEnabled = true;
        rockGrabPinchCloseSelectionEnabled = true;
        rockGrabPinchCompactMaxExtentGameUnits = grab_pinch_pocket_policy::kDefaultCompactMaxExtentGameUnits;
        rockGrabPinchThinRodMaxLengthGameUnits = grab_pinch_pocket_policy::kDefaultThinRodMaxLengthGameUnits;
        rockGrabPinchThinRodMaxCrossSectionGameUnits = grab_pinch_pocket_policy::kDefaultThinRodMaxCrossSectionGameUnits;
        rockGrabPinchMaxPocketDistanceGameUnits = grab_pinch_pocket_policy::kDefaultMaxPocketDistanceGameUnits;
        rockGrabPinchMinFingerGapGameUnits = grab_pinch_pocket_policy::kDefaultMinFingerGapGameUnits;
        rockGrabPinchMaxFingerGapGameUnits = grab_pinch_pocket_policy::kDefaultMaxFingerGapGameUnits;
        rockGrabPinchThumbIndexMaxOpenValue = grab_pinch_pocket_policy::kDefaultThumbIndexMaxOpenValue;
        rockGrabPinchOtherFingerCurlValue = grab_pinch_pocket_policy::kDefaultOtherFingerCurlValue;
        rockGrabPinchSurfaceInsetGameUnits = grab_pinch_pocket_policy::kDefaultSurfaceInsetGameUnits;
        rockGrabPinchDetectionDirectionHandspace = RE::NiPoint3(grab_pinch_pocket_policy::kDefaultDetectionDirectionHandspaceX,
            grab_pinch_pocket_policy::kDefaultDetectionDirectionHandspaceY,
            grab_pinch_pocket_policy::kDefaultDetectionDirectionHandspaceZ);
        rockGrabPinchDetectionAxisBlend = grab_pinch_pocket_policy::kDefaultDetectionAxisBlend;
        rockGrabHandLerpEnabled = true;
        rockGrabHeldRenderClockAnchor = true;
        rockGrabHeldRenderBodyPose = false;
        rockGrabRenderClockProbeOffsetGameUnits = 0.0f;
        rockGrabSceneWriterProbeOffsetZGameUnits = 0.0f;
        rockGrabHeldScenePoseSync = true;
        rockGrabScenePoseSyncFullAnchorGapGameUnits = 8.0f;
        rockGrabScenePoseSyncSolverGapGameUnits = 15.0f;
        rockGrabHandLerpTimeMin = 0.10f;
        rockGrabHandLerpTimeMax = 0.20f;
        rockGrabHandLerpMinDistance = 7.0f;
        rockGrabHandLerpMaxDistance = 14.0f;
        rockGrabHandReturnEnabled = true;
        rockGrabHandReturnTimeMin = 0.10f;
        rockGrabHandReturnTimeMax = 0.20f;
        rockGrabHandReturnMinDistance = 7.0f;
        rockGrabHandReturnMaxDistance = 14.0f;
        rockGrabHandReturnMinAngleDegrees = 5.0f;
        rockGrabHandReturnMaxAngleDegrees = 90.0f;
        rockGrabMeshFingerPoseEnabled = true;
        rockGrabMeshJointPoseEnabled = true;
        rockGrabFingerPoseUpdateInterval = 3;
        rockGrabFingerMinValue = 0.2f;
        rockGrabFingerPoseSmoothingSpeed = 14.0f;
        rockGrabMeshLocalTransformPoseEnabled = true;
        rockGrabFingerLocalTransformSmoothingSpeed = 14.0f;
        rockGrabFingerLocalTransformMaxCorrectionDegrees = 35.0f;
        rockGrabFingerSurfaceAimStrength = 0.75f;
        rockGrabFingerRejectBacksideHits = true;
        rockGrabFingerSurfacePlaneToleranceGameUnits = 1.5f;
        rockGrabFingerSweepContactRadiusGameUnits = 1.0f;
        rockGrabFingerSweepMaxOpenValue = 2.0f;
        rockGrabThumbSweepMaxOpenValue = 2.0f;
        rockGrabFingerPoseResolveWindowSeconds = 2.0f;
        rockGrabThumbOppositionStrength = 1.0f;
        rockGrabThumbAlternateCurveStrength = 0.65f;
        rockGrabThumbSurfaceSafetyEnabled = true;
        rockGrabThumbSurfaceSafetyMarginGameUnits = kDefaultGrabThumbSurfaceSafetyMarginGameUnits;
        rockGrabLateralWeight = 0.6f;
        rockGrabDirectionalWeight = 0.4f;
        rockGrabMaxTriangleDistance = 100.0f;
        rockGrabMeshContactOnly = true;
        rockGrabRequireMeshContact = true;
        rockGrabContactPatchEnabled = true;
        rockGrabContactPatchProbeCount = 9;
        rockGrabContactPatchProbeSpacingGameUnits = 3.0f;
        rockGrabContactPatchProbeRadiusGameUnits = 2.0f;
        rockGrabContactPatchMeshSnapMaxDistanceGameUnits = 6.0f;
        rockGrabContactPatchMaxNormalAngleDegrees = 35.0f;
        rockGrabAlignmentMaxSelectionToMeshDistance = 8.0f;
        rockGrabNodeAnchorsEnabled = true;
        rockGrabNodeRejectOppositeHandAnchor = true;
        rockPrintGrabNodeInfo = false;
        rockGrabNodeNameRight = grab_node_name_policy::defaultGrabNodeName(false);
        rockGrabNodeNameLeft = grab_node_name_policy::defaultGrabNodeName(true);
        rockGrabNodeNameBlacklist = std::string(grab_node_name_policy::kDefaultGrabNodeNameBlacklist);
        rockSelectedCloseFingerCurlEnabled = true;
        rockSelectedCloseFingerAnimMaxHandSpeed = 0.9f;
        rockSelectedCloseFingerAnimValue = 0.9f;
        rockPulledAngularDamping = 8.0f;
        rockPullToObjectCenterEnabled = true;
        rockPullLongAxisPresentationEnabled = true;
        rockForceGrabSeatAlignmentEnabled = true;
        rockPullPresentationMinElongationRatio = 2.0f;
        rockPullPresentationAngularGainPerSecond = 6.0f;
        rockPullPresentationMaxAngularSpeedRadiansPerSecond = 8.0f;
        rockPullPresentationGripAxisTiltDegrees = 10.0f;

        rockRightGrabLegacyPalmPivotAHandspace = RE::NiPoint3(6.0f, -2.0f, 0.2f);
        rockLeftGrabLegacyPalmPivotAHandspace = RE::NiPoint3(6.0f, -2.0f, -0.2f);

        rockGrabHapticsEnabled = true;
        rockGrabHapticDurationSeconds = 0.055f;
        rockGrabHapticBaseIntensity = 0.12f;
        rockGrabHapticMaxIntensity = 0.80f;
        rockGrabHapticMassScale = 0.06f;
        rockGrabHapticMassExponent = 0.60f;
        rockPullStartHapticIntensity = 0.18f;
        rockPullCatchHapticIntensity = 0.22f;
        rockSelectionLockHapticIntensity = 0.15f;
        rockSelectionLockReleaseHapticIntensity = 0.10f;
        rockSelectionLockReleaseHapticDurationSeconds = 0.02f;
        rockSurfaceGrabHapticsEnabled = true;
        rockSurfaceGrabHapticDurationSeconds = 0.075f;
        rockSurfaceGrabHapticIntensity = 0.85f;
        rockHeldImpactHapticsEnabled = true;
        rockHeldImpactHapticDurationSeconds = 0.035f;
        rockHeldImpactHapticBaseIntensity = 0.12f;
        rockHeldImpactHapticMaxIntensity = 0.85f;
        rockHeldImpactHapticSpeedScale = 0.006f;
        rockHeldImpactHapticMassScale = 0.035f;
        rockHeldImpactHapticMassExponent = 0.55f;
        rockHeldImpactHapticMinSpeedGameUnits = 8.0f;
        rockHeldImpactHapticCooldownSeconds = 0.12f;
        rockHeldImpactHapticDampedMultiplier = 0.55f;
        rockShoulderStashHapticsEnabled = true;
        rockShoulderStashCandidateHapticDurationSeconds = 0.075f;
        rockShoulderStashCandidateHapticBaseIntensity = 0.20f;
        rockShoulderStashCandidateHapticIntensity = 0.42f;
        rockShoulderStashCandidateHapticIntervalSeconds = 0.075f;
        rockShoulderStashCommitHapticDurationSeconds = 0.12f;
        rockShoulderStashCommitHapticIntensity = 0.85f;
        rockMouthConsumeHapticsEnabled = true;
        rockMouthConsumeCandidateHapticDurationSeconds = 0.050f;
        rockMouthConsumeCandidateHapticBaseIntensity = 0.22f;
        rockMouthConsumeCandidateHapticIntensity = 0.45f;
        rockMouthConsumeCandidateHapticIntervalSeconds = 0.075f;
        rockMouthConsumeCommitHapticDurationSeconds = 0.12f;
        rockMouthConsumeCommitHapticIntensity = 0.85f;

    }

    void RockConfig::readValuesFromIni(CSimpleIniA& rawIni)
    {
        TrackedIni ini(rawIni);
        const int schemaVersion = static_cast<int>(ini.GetLongValue(CONFIG_SECTION, "iSchemaVersion", 0));
        if (schemaVersion == 0) {
            ROCK_LOG_WARN(Config, "ROCK.ini has no schema version; legacy section compatibility remains active for schema 1");
        } else if (schemaVersion != kCurrentConfigSchemaVersion) {
            ROCK_LOG_WARN(Config,
                "ROCK.ini schema {} does not match runtime schema {}; recognized settings will be loaded and unknown settings ignored",
                schemaVersion,
                kCurrentConfigSchemaVersion);
        }
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
        auto sanitizeMouthConsumeOffset = [&]() {
            auto sanitizeComponent = [](float value, float fallback) {
                if (!std::isfinite(value)) {
                    return fallback;
                }
                return std::clamp(value, -kMaxMouthConsumeHmdOffsetGameUnits, kMaxMouthConsumeHmdOffsetGameUnits);
            };

            const RE::NiPoint3 original = rockMouthConsumeHmdOffsetGameUnits;
            rockMouthConsumeHmdOffsetGameUnits.x = sanitizeComponent(original.x, kDefaultMouthConsumeHmdOffsetGameUnits.x);
            rockMouthConsumeHmdOffsetGameUnits.y = sanitizeComponent(original.y, kDefaultMouthConsumeHmdOffsetGameUnits.y);
            rockMouthConsumeHmdOffsetGameUnits.z = sanitizeComponent(original.z, kDefaultMouthConsumeHmdOffsetGameUnits.z);
            if (original.x != rockMouthConsumeHmdOffsetGameUnits.x ||
                original.y != rockMouthConsumeHmdOffsetGameUnits.y ||
                original.z != rockMouthConsumeHmdOffsetGameUnits.z) {
                ROCK_LOG_WARN(Config,
                    "Mouth consume HMD offset must be finite and within +/-{:.1f} game units; using ({:.1f}, {:.1f}, {:.1f})",
                    kMaxMouthConsumeHmdOffsetGameUnits,
                    rockMouthConsumeHmdOffsetGameUnits.x,
                    rockMouthConsumeHmdOffsetGameUnits.y,
                    rockMouthConsumeHmdOffsetGameUnits.z);
            }
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
        rockSavedGrabOffsetRecordingEnabled =
            ini.GetBoolValue(DEBUG_SECTION, "bSavedGrabOffsetRecordingEnabled", rockSavedGrabOffsetRecordingEnabled);
        rockLogLevel = logging_policy::clampLogLevel(static_cast<int>(ini.GetLongValue(DEBUG_SECTION, "iLogLevel", rockLogLevel)));
        rockLogPattern = ini.GetValue(DEBUG_SECTION, "sLogPattern", rockLogPattern.c_str());
        if (rockLogPattern.empty()) {
            rockLogPattern = logging_policy::DefaultLogPattern;
        }
        rockLogSampleMilliseconds =
            logging_policy::sanitizeSampleMilliseconds(static_cast<int>(ini.GetLongValue(DEBUG_SECTION, "iLogSampleMilliseconds", rockLogSampleMilliseconds)));
        rockDebugEnabled = ini.GetBoolValue(DEBUG_SECTION, "bEnabled", rockDebugEnabled);
        rockDebugOverlayEnabled = ini.GetBoolValue(DEBUG_SECTION, "bOverlayEnabled", rockDebugOverlayEnabled);
        rockDebugLoggingEnabled = ini.GetBoolValue(DEBUG_SECTION, "bLoggingEnabled", rockDebugLoggingEnabled);
        rockDebugControllerEnabled = ini.GetBoolValue(DEBUG_SECTION, "bControllerEnabled", rockDebugControllerEnabled);
        rockDebugMonitorEnabled = ini.GetBoolValue(DEBUG_SECTION, "bMonitorEnabled", rockDebugMonitorEnabled);
        rockDebugProviderOverlayEnabled = ini.GetBoolValue(DEBUG_SECTION, "bProviderOverlayEnabled", rockDebugProviderOverlayEnabled);
        rockDebugProviderColliderFocusEnabled =
            ini.GetBoolValue(DEBUG_SECTION, "bProviderColliderFocusEnabled", rockDebugProviderColliderFocusEnabled);
        rockPerformanceProfilerEnabled = ini.GetBoolValue(DEBUG_SECTION, "bPerformanceProfilerEnabled", rockPerformanceProfilerEnabled);
        rockPerformanceProfilerLogIntervalFrames =
            std::clamp(static_cast<int>(ini.GetLongValue(DEBUG_SECTION, "iPerformanceProfilerLogIntervalFrames", rockPerformanceProfilerLogIntervalFrames)), 30, 54000);
        rockPerformanceProfilerWarmupFrames =
            std::clamp(static_cast<int>(ini.GetLongValue(DEBUG_SECTION, "iPerformanceProfilerWarmupFrames", rockPerformanceProfilerWarmupFrames)), 0, 54000);
        rockPerformanceProfilerOverlayText = ini.GetBoolValue(DEBUG_SECTION, "bPerformanceProfilerOverlayText", rockPerformanceProfilerOverlayText);
        logger::setLogLevelAndPattern(rockLogLevel, rockLogPattern);

        rockEnabled = ini.GetBoolValue(SECTION, "bEnabled", rockEnabled);
        rockHavokTimingFixEnabled = ini.GetBoolValue(SECTION, "bHavokTimingFixEnabled", rockHavokTimingFixEnabled);
        rockHavokTimingFixMinPhysicsFrameRate = havok_timing_fix_policy::sanitizeMinPhysicsFrameRate(
            static_cast<float>(ini.GetDoubleValue(SECTION, "fHavokTimingFixMinPhysicsFrameRate", rockHavokTimingFixMinPhysicsFrameRate)));
        rockHavokTimingFixMaxSubsteps = havok_timing_fix_policy::sanitizeMaxSubsteps(
            static_cast<int>(ini.GetLongValue(SECTION, "iHavokTimingFixMaxSubsteps", rockHavokTimingFixMaxSubsteps)));
        rockSuppressRightFavoritesGameInput = ini.GetBoolValue(SECTION, "bSuppressRightFavoritesGameInput", rockSuppressRightFavoritesGameInput);
        rockSuppressNativeVats = ini.GetBoolValue(SECTION, "bSuppressNativeVats", rockSuppressNativeVats);
        rockSuppressNativeVans = ini.GetBoolValue(SECTION, "bSuppressNativeVans", rockSuppressNativeVans);
        rockSuppressPipboyGameInputWhileHolding = ini.GetBoolValue(SECTION, "bSuppressPipboyGameInputWhileHolding", rockSuppressPipboyGameInputWhileHolding);
        rockPipboyPauseHoldSeconds = pipboy_pause_gesture_policy::sanitizedHoldSeconds(
            static_cast<float>(ini.GetDoubleValue(SECTION, "fPipboyPauseHoldSeconds", rockPipboyPauseHoldSeconds)));
        rockSuppressTakeEquipGameInputWhileHolding =
            ini.GetBoolValue(SECTION, "bSuppressTakeEquipGameInputWhileHolding", rockSuppressTakeEquipGameInputWhileHolding);
        rockSuppressTakeEquipFormTypes = ini.GetValue(SECTION, "sSuppressTakeEquipFormTypes", rockSuppressTakeEquipFormTypes.c_str());
        rockSuppressNativeGrabHoverHaptics = ini.GetBoolValue(SECTION, "bSuppressNativeGrabHoverHaptics", rockSuppressNativeGrabHoverHaptics);
        rockGrabInputIntentStateEnabled = ini.GetBoolValue(SECTION, "bGrabInputIntentStateEnabled", rockGrabInputIntentStateEnabled);
        rockGrabInputLeewaySeconds = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabInputLeewaySeconds", rockGrabInputLeewaySeconds));
        if (!std::isfinite(rockGrabInputLeewaySeconds) || rockGrabInputLeewaySeconds < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabInputLeewaySeconds={} -- using 0.12", rockGrabInputLeewaySeconds);
            rockGrabInputLeewaySeconds = 0.12f;
        }
        rockGrabInputLeewaySeconds = std::clamp(rockGrabInputLeewaySeconds, 0.0f, 0.5f);
        rockGrabInputForceSeconds = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabInputForceSeconds", rockGrabInputForceSeconds));
        if (!std::isfinite(rockGrabInputForceSeconds) || rockGrabInputForceSeconds < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabInputForceSeconds={} -- using 0.08", rockGrabInputForceSeconds);
            rockGrabInputForceSeconds = 0.08f;
        }
        rockGrabInputForceSeconds = std::clamp(rockGrabInputForceSeconds, 0.0f, 0.3f);

        readVec3("fPalmNormalHandspaceX", "fPalmNormalHandspaceY", "fPalmNormalHandspaceZ", rockPalmNormalHandspace);
        readVec3("fPointingVectorHandspaceX", "fPointingVectorHandspaceY", "fPointingVectorHandspaceZ", rockPointingVectorHandspace);
        rockReversePalmNormal = ini.GetBoolValue(SECTION, "bReversePalmNormal", rockReversePalmNormal);
        rockReverseFarGrabNormal = ini.GetBoolValue(SECTION, "bReverseFarGrabNormal", rockReverseFarGrabNormal);

        rockLeftHandedMode = ini.GetBoolValue(
            WEAPON_HANDEDNESS_SECTION,
            "bLeftHandedMode",
            rockLeftHandedMode);
        rockGunstockModeEnabled = ini.GetBoolValue(
            GUNSTOCK_SECTION,
            "bAlignBarrelToControllerForward",
            rockGunstockModeEnabled);
        rockGunstockAlignmentPitchDegrees = readClampedFloat(
            ini,
            GUNSTOCK_SECTION,
            "fAlignmentPitchDegrees",
            rockGunstockAlignmentPitchDegrees,
            0.0f,
            -180.0f,
            180.0f);
        rockGunstockAlignmentYawDegrees = readClampedFloat(
            ini,
            GUNSTOCK_SECTION,
            "fAlignmentYawDegrees",
            rockGunstockAlignmentYawDegrees,
            0.0f,
            -180.0f,
            180.0f);
        rockGunstockAlignmentRollDegrees = readClampedFloat(
            ini,
            GUNSTOCK_SECTION,
            "fAlignmentRollDegrees",
            rockGunstockAlignmentRollDegrees,
            0.0f,
            -180.0f,
            180.0f);
        rockAmbidextrousFiringGripEnabled = ini.GetBoolValue(
            AMBIDEXTROUS_FIRING_SECTION,
            "bAmbidextrousFiringGripEnabled",
            rockAmbidextrousFiringGripEnabled);
        rockFiringGripPromotionRadius = readClampedFloat(
            ini,
            AMBIDEXTROUS_FIRING_SECTION,
            "fFiringGripPromotionRadius",
            rockFiringGripPromotionRadius,
            5.0f,
            0.25f,
            30.0f);
        rockLeftFiringAimYawDegrees = readClampedFloat(
            ini,
            AMBIDEXTROUS_FIRING_SECTION,
            "fLeftFiringAimYawDegrees",
            rockLeftFiringAimYawDegrees,
            0.0f,
            -30.0f,
            30.0f);
        rockLeftFiringAimPitchDegrees = readClampedFloat(
            ini,
            AMBIDEXTROUS_FIRING_SECTION,
            "fLeftFiringAimPitchDegrees",
            rockLeftFiringAimPitchDegrees,
            0.0f,
            -30.0f,
            30.0f);
        rockLeftFiringAimOffsetXGameUnits = readClampedFloat(
            ini,
            AMBIDEXTROUS_FIRING_SECTION,
            "fLeftFiringAimOffsetXGameUnits",
            rockLeftFiringAimOffsetXGameUnits,
            0.0f,
            -15.0f,
            15.0f);
        rockLeftFiringAimOffsetYGameUnits = readClampedFloat(
            ini,
            AMBIDEXTROUS_FIRING_SECTION,
            "fLeftFiringAimOffsetYGameUnits",
            rockLeftFiringAimOffsetYGameUnits,
            0.0f,
            -15.0f,
            15.0f);
        rockLeftFiringAimOffsetZGameUnits = readClampedFloat(
            ini,
            AMBIDEXTROUS_FIRING_SECTION,
            "fLeftFiringAimOffsetZGameUnits",
            rockLeftFiringAimOffsetZGameUnits,
            0.0f,
            -15.0f,
            15.0f);
        rockWeaponCollisionEnabled = ini.GetBoolValue(SECTION, "bWeaponCollisionEnabled", rockWeaponCollisionEnabled);
        rockWeaponCollisionBlocksProjectiles = ini.GetBoolValue(SECTION, "bWeaponCollisionBlocksProjectiles", rockWeaponCollisionBlocksProjectiles);
        rockWeaponCollisionBlocksSpells = ini.GetBoolValue(SECTION, "bWeaponCollisionBlocksSpells", rockWeaponCollisionBlocksSpells);
        rockWeaponCollisionStaticWorldEnabled = ini.GetBoolValue(SECTION, "bWeaponCollisionStaticWorldEnabled", rockWeaponCollisionStaticWorldEnabled);
        rockWeaponCollisionDynamicBoxEnabled = ini.GetBoolValue(SECTION, "bWeaponCollisionDynamicBoxEnabled", rockWeaponCollisionDynamicBoxEnabled);
        rockWeaponCollisionDynamicBoxPaddingGameUnits = readClampedFloat(ini,
            SECTION,
            "fWeaponCollisionDynamicBoxPaddingGameUnits",
            rockWeaponCollisionDynamicBoxPaddingGameUnits,
            0.5f,
            0.0f,
            20.0f);
        rockWeaponCollisionDynamicInverseInertiaMultiplier = readClampedFloat(ini,
            SECTION,
            "fWeaponCollisionDynamicInverseInertiaMultiplier",
            rockWeaponCollisionDynamicInverseInertiaMultiplier,
            kDefaultWeaponCollisionDynamicInverseInertiaMultiplier,
            0.25f,
            4.0f);
        rockWeaponCollisionDynamicDivergenceTeleportGameUnits = readClampedFloat(ini,
            SECTION,
            "fWeaponCollisionDynamicDivergenceTeleportGameUnits",
            rockWeaponCollisionDynamicDivergenceTeleportGameUnits,
            80.0f,
            0.0f,
            1000.0f);
        rockWeaponCollisionDynamicDivergenceTeleportDwellSeconds = readClampedFloat(ini,
            SECTION,
            "fWeaponCollisionDynamicDivergenceTeleportDwellSeconds",
            rockWeaponCollisionDynamicDivergenceTeleportDwellSeconds,
            0.3f,
            0.0f,
            5.0f);
        rockWeaponCollisionDynamicRenderMinTranslationGameUnits = readClampedFloat(ini,
            SECTION,
            "fWeaponCollisionDynamicRenderMinTranslationGameUnits",
            rockWeaponCollisionDynamicRenderMinTranslationGameUnits,
            0.05f,
            0.0f,
            10.0f);
        rockWeaponCollisionDynamicRenderMinRotationDegrees = readClampedFloat(ini,
            SECTION,
            "fWeaponCollisionDynamicRenderMinRotationDegrees",
            rockWeaponCollisionDynamicRenderMinRotationDegrees,
            0.25f,
            0.0f,
            45.0f);
        rockWeaponCollisionGroupingMode = static_cast<int>(ini.GetLongValue(SECTION, "iWeaponCollisionGroupingMode", rockWeaponCollisionGroupingMode));
        const auto sanitizedWeaponCollisionGroupingMode = weapon_collision_grouping_policy::sanitizeWeaponCollisionGroupingMode(rockWeaponCollisionGroupingMode);
        if (static_cast<int>(sanitizedWeaponCollisionGroupingMode) != rockWeaponCollisionGroupingMode) {
            ROCK_LOG_WARN(Config,
                "Unsupported iWeaponCollisionGroupingMode={} - using {}",
                rockWeaponCollisionGroupingMode,
                weapon_collision_grouping_policy::weaponCollisionGroupingModeName(sanitizedWeaponCollisionGroupingMode));
            rockWeaponCollisionGroupingMode = static_cast<int>(sanitizedWeaponCollisionGroupingMode);
        }
        rockWeaponCollisionVisualStabilizationFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iWeaponCollisionVisualStabilizationFrames", rockWeaponCollisionVisualStabilizationFrames));
        if (rockWeaponCollisionVisualStabilizationFrames < 0 ||
            rockWeaponCollisionVisualStabilizationFrames > kMaxWeaponCollisionVisualStabilizationFrames) {
            ROCK_LOG_WARN(Config,
                "Invalid iWeaponCollisionVisualStabilizationFrames={} - using {}",
                rockWeaponCollisionVisualStabilizationFrames,
                kDefaultWeaponCollisionVisualStabilizationFrames);
            rockWeaponCollisionVisualStabilizationFrames = kDefaultWeaponCollisionVisualStabilizationFrames;
        }
        rockWeaponCollisionConvexRadius = static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionConvexRadius", rockWeaponCollisionConvexRadius));
        rockWeaponCollisionPointDedupGrid = static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionPointDedupGrid", rockWeaponCollisionPointDedupGrid));
        rockWeaponCollisionSupportFitTargetPoints =
            static_cast<int>(ini.GetLongValue(SECTION, "iWeaponCollisionSupportFitTargetPoints", rockWeaponCollisionSupportFitTargetPoints));
        if (rockWeaponCollisionSupportFitTargetPoints < kMinWeaponCollisionSupportFitTargetPoints ||
            rockWeaponCollisionSupportFitTargetPoints > kMaxWeaponCollisionSupportFitTargetPoints) {
            ROCK_LOG_WARN(Config,
                "Invalid iWeaponCollisionSupportFitTargetPoints={} - using {}",
                rockWeaponCollisionSupportFitTargetPoints,
                kDefaultWeaponCollisionSupportFitTargetPoints);
            rockWeaponCollisionSupportFitTargetPoints = kDefaultWeaponCollisionSupportFitTargetPoints;
        }
        rockWeaponCollisionSupportFitMaxErrorGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionSupportFitMaxErrorGameUnits", rockWeaponCollisionSupportFitMaxErrorGameUnits));
        if (!std::isfinite(rockWeaponCollisionSupportFitMaxErrorGameUnits) || rockWeaponCollisionSupportFitMaxErrorGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config,
                "Invalid fWeaponCollisionSupportFitMaxErrorGameUnits={} - using {:.2f}",
                rockWeaponCollisionSupportFitMaxErrorGameUnits,
                kDefaultWeaponCollisionSupportFitMaxErrorGameUnits);
            rockWeaponCollisionSupportFitMaxErrorGameUnits = kDefaultWeaponCollisionSupportFitMaxErrorGameUnits;
        }
        rockWeaponSizeClassPistolMaxWeight = readClampedFloat(ini,
            SECTION,
            "fWeaponSizeClassPistolMaxWeight",
            rockWeaponSizeClassPistolMaxWeight,
            6.0f,
            0.0f,
            200.0f);
        rockWeaponSizeClassRifleMaxWeight = readClampedFloat(ini,
            SECTION,
            "fWeaponSizeClassRifleMaxWeight",
            rockWeaponSizeClassRifleMaxWeight,
            20.0f,
            0.0f,
            200.0f);
        if (rockWeaponSizeClassPistolMaxWeight >= rockWeaponSizeClassRifleMaxWeight) {
            ROCK_LOG_WARN(Config,
                "Invalid weapon size class weight thresholds: fWeaponSizeClassPistolMaxWeight={:.2f} >= fWeaponSizeClassRifleMaxWeight={:.2f} - using defaults",
                rockWeaponSizeClassPistolMaxWeight,
                rockWeaponSizeClassRifleMaxWeight);
            rockWeaponSizeClassPistolMaxWeight = 6.0f;
            rockWeaponSizeClassRifleMaxWeight = 20.0f;
        }
        rockWeaponInteractionTouchRadius = readClampedFloat(ini,
            SECTION,
            "fWeaponInteractionTouchRadius",
            rockWeaponInteractionTouchRadius,
            2.0f,
            0.25f,
            6.0f);
        rockWeaponInteractionProbeRadius = static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponInteractionProbeRadius", rockWeaponInteractionProbeRadius));
        rockWeaponAuthoredGripActivationRadius = readClampedFloat(ini,
            SECTION,
            "fWeaponAuthoredGripActivationRadius",
            rockWeaponAuthoredGripActivationRadius,
            16.0f,
            2.0f,
            32.0f);
        rockFiringGripProximitySupportRadius = readClampedFloat(ini,
            SECTION,
            "fFiringGripProximitySupportRadius",
            rockFiringGripProximitySupportRadius,
            6.0f,
            0.25f,
            30.0f);
        rockRealisticGrenadeFuseSeconds = readClampedFloat(ini,
            REALISTIC_WEAPONS_SECTION,
            "fRealisticGrenadeFuseSeconds",
            rockRealisticGrenadeFuseSeconds,
            5.0f,
            0.0f,
            30.0f);
        rockWeaponSupportGripHandLerpEnabled = ini.GetBoolValue(SECTION, "bWeaponSupportGripHandLerpEnabled", rockWeaponSupportGripHandLerpEnabled);
        rockWeaponSupportGripHandLerpTimeMin = readClampedFloat(ini,
            SECTION,
            "fWeaponSupportGripHandLerpTimeMin",
            rockWeaponSupportGripHandLerpTimeMin,
            0.12f,
            0.0f,
            1.0f);
        rockWeaponSupportGripHandLerpTimeMax = readClampedFloat(ini,
            SECTION,
            "fWeaponSupportGripHandLerpTimeMax",
            rockWeaponSupportGripHandLerpTimeMax,
            0.20f,
            rockWeaponSupportGripHandLerpTimeMin,
            1.0f);
        rockWeaponSupportGripHandLerpMinDistance = readClampedFloat(ini,
            SECTION,
            "fWeaponSupportGripHandLerpMinDistance",
            rockWeaponSupportGripHandLerpMinDistance,
            1.0f,
            0.0f,
            80.0f);
        rockWeaponSupportGripHandLerpMaxDistance = readClampedFloat(ini,
            SECTION,
            "fWeaponSupportGripHandLerpMaxDistance",
            rockWeaponSupportGripHandLerpMaxDistance,
            14.0f,
            rockWeaponSupportGripHandLerpMinDistance,
            120.0f);
        rockWeaponSupportSurfaceSeatEnabled = ini.GetBoolValue(
            SECTION,
            "bWeaponSupportSurfaceSeatEnabled",
            rockWeaponSupportSurfaceSeatEnabled);
        rockWeaponSupportSurfaceSeatMaxDegrees = readClampedFloat(
            ini,
            SECTION,
            "fWeaponSupportSurfaceSeatMaxDegrees",
            rockWeaponSupportSurfaceSeatMaxDegrees,
            35.0f,
            0.0f,
            75.0f);
        rockWeaponVisualReturnEnabled = ini.GetBoolValue(SECTION, "bWeaponVisualReturnEnabled", rockWeaponVisualReturnEnabled);
        rockWeaponVisualReturnTimeMin = readClampedFloat(ini,
            SECTION,
            "fWeaponVisualReturnTimeMin",
            rockWeaponVisualReturnTimeMin,
            0.12f,
            0.0f,
            1.0f);
        rockWeaponVisualReturnTimeMax = readClampedFloat(ini,
            SECTION,
            "fWeaponVisualReturnTimeMax",
            rockWeaponVisualReturnTimeMax,
            0.20f,
            rockWeaponVisualReturnTimeMin,
            1.0f);
        rockWeaponVisualReturnMinDistance = readClampedFloat(ini,
            SECTION,
            "fWeaponVisualReturnMinDistance",
            rockWeaponVisualReturnMinDistance,
            1.0f,
            0.0f,
            80.0f);
        rockWeaponVisualReturnMaxDistance = readClampedFloat(ini,
            SECTION,
            "fWeaponVisualReturnMaxDistance",
            rockWeaponVisualReturnMaxDistance,
            14.0f,
            rockWeaponVisualReturnMinDistance,
            120.0f);
        rockWeaponVisualReturnMinAngleDegrees = readClampedFloat(ini,
            SECTION,
            "fWeaponVisualReturnMinAngleDegrees",
            rockWeaponVisualReturnMinAngleDegrees,
            5.0f,
            0.0f,
            180.0f);
        rockWeaponVisualReturnMaxAngleDegrees = readClampedFloat(ini,
            SECTION,
            "fWeaponVisualReturnMaxAngleDegrees",
            rockWeaponVisualReturnMaxAngleDegrees,
            90.0f,
            rockWeaponVisualReturnMinAngleDegrees,
            180.0f);

        rockManualScopeHoldSeconds = readClampedFloat(ini,
            NATIVE_SCOPES_SECTION,
            "fManualScopeHoldSeconds",
            rockManualScopeHoldSeconds,
            0.30f,
            0.05f,
            2.0f);
        rockNativeScopeForceFiringGripFallback =
            ini.GetBoolValue(
                NATIVE_SCOPES_SECTION,
                "bNativeScopeForceFiringGripFallback",
                rockNativeScopeForceFiringGripFallback);
        rockNativeScopeFiringGripFallbackOffsetXGameUnits = readClampedFloat(ini,
            NATIVE_SCOPES_SECTION,
            "fNativeScopeFiringGripFallbackOffsetXGameUnits",
            rockNativeScopeFiringGripFallbackOffsetXGameUnits,
            0.0f,
            -100.0f,
            100.0f);
        rockNativeScopeFiringGripFallbackOffsetYGameUnits = readClampedFloat(ini,
            NATIVE_SCOPES_SECTION,
            "fNativeScopeFiringGripFallbackOffsetYGameUnits",
            rockNativeScopeFiringGripFallbackOffsetYGameUnits,
            0.0f,
            -100.0f,
            100.0f);
        rockNativeScopeFiringGripFallbackOffsetZGameUnits = readClampedFloat(ini,
            NATIVE_SCOPES_SECTION,
            "fNativeScopeFiringGripFallbackOffsetZGameUnits",
            rockNativeScopeFiringGripFallbackOffsetZGameUnits,
            0.0f,
            -100.0f,
            100.0f);
        rockNativeScopeFiringGripFallbackPitchDegrees = readClampedFloat(ini,
            NATIVE_SCOPES_SECTION,
            "fNativeScopeFiringGripFallbackPitchDegrees",
            rockNativeScopeFiringGripFallbackPitchDegrees,
            0.0f,
            -180.0f,
            180.0f);
        rockNativeScopeFiringGripFallbackYawDegrees = readClampedFloat(ini,
            NATIVE_SCOPES_SECTION,
            "fNativeScopeFiringGripFallbackYawDegrees",
            rockNativeScopeFiringGripFallbackYawDegrees,
            0.0f,
            -180.0f,
            180.0f);
        rockNativeScopeFiringGripFallbackRollDegrees = readClampedFloat(ini,
            NATIVE_SCOPES_SECTION,
            "fNativeScopeFiringGripFallbackRollDegrees",
            rockNativeScopeFiringGripFallbackRollDegrees,
            0.0f,
            -180.0f,
            180.0f);
        rockNativeScopeOverlayOffsetXGameUnits = readClampedFloat(ini,
            NATIVE_SCOPES_SECTION,
            "fNativeScopeOverlayOffsetXGameUnits",
            rockNativeScopeOverlayOffsetXGameUnits,
            0.0f,
            -100.0f,
            100.0f);
        rockNativeScopeOverlayOffsetYGameUnits = readClampedFloat(ini,
            NATIVE_SCOPES_SECTION,
            "fNativeScopeOverlayOffsetYGameUnits",
            rockNativeScopeOverlayOffsetYGameUnits,
            0.0f,
            -100.0f,
            100.0f);
        rockNativeScopeOverlayOffsetZGameUnits = readClampedFloat(ini,
            NATIVE_SCOPES_SECTION,
            "fNativeScopeOverlayOffsetZGameUnits",
            rockNativeScopeOverlayOffsetZGameUnits,
            0.0f,
            -100.0f,
            100.0f);
        rockNativeScopeOverlayPitchDegrees = readClampedFloat(ini,
            NATIVE_SCOPES_SECTION,
            "fNativeScopeOverlayPitchDegrees",
            rockNativeScopeOverlayPitchDegrees,
            0.0f,
            -180.0f,
            180.0f);
        rockNativeScopeOverlayYawDegrees = readClampedFloat(ini,
            NATIVE_SCOPES_SECTION,
            "fNativeScopeOverlayYawDegrees",
            rockNativeScopeOverlayYawDegrees,
            0.0f,
            -180.0f,
            180.0f);
        rockNativeScopeOverlayRollDegrees = readClampedFloat(ini,
            NATIVE_SCOPES_SECTION,
            "fNativeScopeOverlayRollDegrees",
            rockNativeScopeOverlayRollDegrees,
            0.0f,
            -180.0f,
            180.0f);

        rockHandCollisionDynamicDrive = ini.GetBoolValue(SECTION, "bHandCollisionDynamicDrive", rockHandCollisionDynamicDrive);
        rockHandDynamicInteractionsEnabled = ini.GetBoolValue(
            SECTION,
            "bHandDynamicInteractionsEnabled",
            rockHandDynamicInteractionsEnabled);
        rockDynamicColliderNpcBodyCollisionEnabled = ini.GetBoolValue(
            SECTION,
            "bDynamicCollidersNpcBodyCollisionEnabled",
            rockDynamicColliderNpcBodyCollisionEnabled);
        rockHandCollisionDynamicMaxLinearVelocityHavok = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicMaxLinearVelocityHavok",
            rockHandCollisionDynamicMaxLinearVelocityHavok,
            15.0f,
            0.0f,
            200.0f);
        rockHandCollisionDynamicContactPressMaxVelocityHavok = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicContactPressMaxVelocityHavok",
            rockHandCollisionDynamicContactPressMaxVelocityHavok,
            1.0f,
            0.0f,
            50.0f);
        rockHandCollisionDynamicCompoundMass = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicCompoundMass",
            rockHandCollisionDynamicCompoundMass,
            2.0f,
            0.1f,
            50.0f);
        rockHandCollisionDynamicInverseInertiaMultiplier = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicInverseInertiaMultiplier",
            rockHandCollisionDynamicInverseInertiaMultiplier,
            1.0f,
            0.05f,
            10.0f);
        rockHandCollisionDynamicDivergenceTeleportGameUnits = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicDivergenceTeleportGameUnits",
            rockHandCollisionDynamicDivergenceTeleportGameUnits,
            40.0f,
            0.0f,
            500.0f);
        rockHandCollisionDynamicDivergenceTeleportDwellSeconds = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicDivergenceTeleportDwellSeconds",
            rockHandCollisionDynamicDivergenceTeleportDwellSeconds,
            0.3f,
            0.0f,
            5.0f);
        rockHandCollisionDynamicTeleportRecoverySeconds = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicTeleportRecoverySeconds",
            rockHandCollisionDynamicTeleportRecoverySeconds,
            0.25f,
            0.0f,
            2.0f);
        rockHandCollisionDynamicRenderFollowMinDeviationGameUnits = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicRenderFollowMinDeviationGameUnits",
            rockHandCollisionDynamicRenderFollowMinDeviationGameUnits,
            0.05f,
            0.0f,
            5.0f);
        rockHandCollisionDynamicRenderFollowSmoothingSpeed = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicRenderFollowSmoothingSpeed",
            rockHandCollisionDynamicRenderFollowSmoothingSpeed,
            45.0f,
            0.0f,
            240.0f);
        rockHandCollisionDynamicVisualPriority = static_cast<int>(ini.GetLongValue(SECTION, "iHandCollisionDynamicVisualPriority", rockHandCollisionDynamicVisualPriority));
        rockHandCollisionDynamicVisualPriority = std::clamp(rockHandCollisionDynamicVisualPriority, 0, 99);
        rockHandCollisionDynamicHapticsEnabled =
            ini.GetBoolValue(SECTION, "bHandCollisionDynamicHapticsEnabled", rockHandCollisionDynamicHapticsEnabled);
        rockHandCollisionDynamicHapticDurationSeconds = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicHapticDurationSeconds",
            rockHandCollisionDynamicHapticDurationSeconds,
            0.035f,
            0.0f,
            0.2f);
        rockHandCollisionDynamicHapticBaseIntensity = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicHapticBaseIntensity",
            rockHandCollisionDynamicHapticBaseIntensity,
            0.18f,
            0.0f,
            1.0f);
        rockHandCollisionDynamicHapticMaxIntensity = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicHapticMaxIntensity",
            rockHandCollisionDynamicHapticMaxIntensity,
            0.55f,
            rockHandCollisionDynamicHapticBaseIntensity,
            1.0f);
        rockHandCollisionDynamicHapticSpeedScale = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicHapticSpeedScale",
            rockHandCollisionDynamicHapticSpeedScale,
            0.006f,
            0.0f,
            1.0f);
        rockHandCollisionDynamicHapticMinApproachSpeedGameUnitsPerSecond = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicHapticMinApproachSpeedGameUnitsPerSecond",
            rockHandCollisionDynamicHapticMinApproachSpeedGameUnitsPerSecond,
            3.0f,
            0.0f,
            5000.0f);
        rockHandCollisionDynamicHapticCooldownSeconds = readClampedFloat(ini,
            SECTION,
            "fHandCollisionDynamicHapticCooldownSeconds",
            rockHandCollisionDynamicHapticCooldownSeconds,
            0.12f,
            0.0f,
            5.0f);
        rockHandCollisionSurfaceFingerResponseEnabled = ini.GetBoolValue(
            SECTION,
            "bHandCollisionSurfaceFingerResponseEnabled",
            rockHandCollisionSurfaceFingerResponseEnabled);
        rockHandCollisionSurfaceFingerProbeDeltaOpenUnits = readClampedFloat(ini,
            SECTION,
            "fHandCollisionSurfaceFingerProbeDeltaOpenUnits",
            rockHandCollisionSurfaceFingerProbeDeltaOpenUnits,
            0.10f,
            0.01f,
            0.50f);
        rockHandCollisionSurfaceFingerResponseGain = readClampedFloat(ini,
            SECTION,
            "fHandCollisionSurfaceFingerResponseGain",
            rockHandCollisionSurfaceFingerResponseGain,
            1.0f,
            0.0f,
            4.0f);
        rockHandCollisionSurfaceFingerMaximumDeflectionOpenUnits = readClampedFloat(ini,
            SECTION,
            "fHandCollisionSurfaceFingerMaximumDeflectionOpenUnits",
            rockHandCollisionSurfaceFingerMaximumDeflectionOpenUnits,
            0.85f,
            0.0f,
            1.0f);
        rockHandCollisionSurfaceFingerMinimumHelpfulTravelGameUnits = readClampedFloat(ini,
            SECTION,
            "fHandCollisionSurfaceFingerMinimumHelpfulTravelGameUnits",
            rockHandCollisionSurfaceFingerMinimumHelpfulTravelGameUnits,
            0.01f,
            0.0001f,
            1.0f);
        rockHandCollisionSurfaceFingerSmoothingSpeed = readClampedFloat(ini,
            SECTION,
            "fHandCollisionSurfaceFingerSmoothingSpeed",
            rockHandCollisionSurfaceFingerSmoothingSpeed,
            30.0f,
            0.0f,
            240.0f);
        rockHandCollisionSurfaceFingerReleaseDelaySeconds = readClampedFloat(ini,
            SECTION,
            "fHandCollisionSurfaceFingerReleaseDelaySeconds",
            rockHandCollisionSurfaceFingerReleaseDelaySeconds,
            0.12f,
            0.0f,
            2.0f);

        rockNativeMeleeSuppressionEnabled = ini.GetBoolValue(SECTION, "bNativeMeleeSuppressionEnabled", rockNativeMeleeSuppressionEnabled);
        rockNativeMeleeFullSuppression = ini.GetBoolValue(SECTION, "bNativeMeleeFullSuppression", rockNativeMeleeFullSuppression);
        rockNativeMeleeSuppressWeaponSwing = ini.GetBoolValue(SECTION, "bNativeMeleeSuppressWeaponSwing", rockNativeMeleeSuppressWeaponSwing);
        rockNativeMeleeSuppressHitFrame = ini.GetBoolValue(SECTION, "bNativeMeleeSuppressHitFrame", rockNativeMeleeSuppressHitFrame);
        rockNativeMeleeDebugLogging = ini.GetBoolValue(DEBUG_LOGGING_SECTION, "bNativeMelee", rockNativeMeleeDebugLogging);
        rockNativeCharacterControllerObjectContactFilterEnabled = ini.GetBoolValue(
            SECTION, "bNativeCharacterControllerObjectContactFilterEnabled", rockNativeCharacterControllerObjectContactFilterEnabled);

        rockHighlightEnabled = ini.GetBoolValue(SECTION, "bHighlightEnabled", rockHighlightEnabled);
        rockHighlightIntensityMode = readHighlightIntensityMode(ini, SECTION, "iHighlightIntensityMode", rockHighlightIntensityMode);
        rockHighlightColor = readHighlightColor(ini, SECTION, "sHighlightColor", rockHighlightColor);
        rockSelectionBeamEnabled = ini.GetBoolValue(SECTION, "bSelectionBeamEnabled", rockSelectionBeamEnabled);
        rockSelectionBeamSegmentSizeGameUnits = readClampedFloat(ini,
            SECTION,
            "fSelectionBeamSegmentSizeGameUnits",
            rockSelectionBeamSegmentSizeGameUnits,
            selection_beam_policy::kDefaultSegmentSizeGameUnits,
            0.2f,
            6.0f);
        rockSelectionBeamCurveLiftGameUnits = readClampedFloat(ini,
            SECTION,
            "fSelectionBeamCurveLiftGameUnits",
            rockSelectionBeamCurveLiftGameUnits,
            selection_beam_policy::kDefaultCurveLiftGameUnits,
            0.0f,
            80.0f);
        rockSelectionBeamAlpha = readClampedFloat(ini,
            SECTION,
            "fSelectionBeamAlpha",
            rockSelectionBeamAlpha,
            selection_beam_policy::kDefaultAlpha,
            0.05f,
            1.0f);

        rockDebugShowColliders = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bShowColliders", rockDebugShowColliders);
        rockDebugShowTargetColliders = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bShowTargetColliders", rockDebugShowTargetColliders);
        rockDebugShowHandAxes = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bShowHandAxes", rockDebugShowHandAxes);
        rockDebugShowGrabPivots = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bShowGrabPivots", rockDebugShowGrabPivots);
        rockDebugShowGrabPocketNormal = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bShowGrabPocketNormal", rockDebugShowGrabPocketNormal);
        rockDebugDrawGrabContactPatch = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawGrabContactPatch", rockDebugDrawGrabContactPatch);
        rockDebugDrawGrabForceTorque = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawGrabForceTorque", rockDebugDrawGrabForceTorque);
        rockDebugDrawGrabForceTorqueText = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawGrabForceTorqueText", rockDebugDrawGrabForceTorqueText);
        rockDebugDrawGrabPivotSourceCollider =
            ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawGrabPivotSourceCollider", rockDebugDrawGrabPivotSourceCollider);
        rockDebugDrawGrabPivotSourceEvidence =
            ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawGrabPivotSourceEvidence", rockDebugDrawGrabPivotSourceEvidence);
        rockDebugDrawGrabSupportFrame = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawGrabSupportFrame", rockDebugDrawGrabSupportFrame);
        rockDebugDrawGrabPockets = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawGrabPockets", rockDebugDrawGrabPockets);
        rockDebugShowGrabFingerProbes = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bShowGrabFingerProbes", rockDebugShowGrabFingerProbes);
        rockDebugShowGrabFingerSweptArc = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bShowGrabFingerSweptArc", rockDebugShowGrabFingerSweptArc);
        rockDebugShowGrabFingerSweptArcText = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bShowGrabFingerSweptArcText", rockDebugShowGrabFingerSweptArcText);
        rockDebugShowGrabFingerSweptArcLiveSkeleton =
            ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bShowGrabFingerSweptArcLiveSkeleton", rockDebugShowGrabFingerSweptArcLiveSkeleton);
        rockDebugShowPalmVectors = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bShowPalmVectors", rockDebugShowPalmVectors);
        rockDebugDrawHandColliders = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawHandColliders", rockDebugDrawHandColliders);
        rockDebugDrawHandBoneColliders = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawHandBoneColliders", rockDebugDrawHandBoneColliders);
        rockDebugDrawDynamicHandColliders = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawDynamicHandColliders", rockDebugDrawDynamicHandColliders);
        rockDebugDrawHandBoneContacts = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawHandBoneContacts", rockDebugDrawHandBoneContacts);
        rockDebugDrawGrabAuthorityProxy = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawGrabAuthorityProxy", rockDebugDrawGrabAuthorityProxy);
        rockDebugMaxHandBoneBodiesDrawn =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxHandBoneBodiesDrawn", rockDebugMaxHandBoneBodiesDrawn));
        if (rockDebugMaxHandBoneBodiesDrawn < 0) {
            rockDebugMaxHandBoneBodiesDrawn = 0;
        } else if (rockDebugMaxHandBoneBodiesDrawn > 48) {
            rockDebugMaxHandBoneBodiesDrawn = 48;
        }
        rockDebugMaxBodyBoneBodiesDrawn =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxBodyBoneBodiesDrawn", rockDebugMaxBodyBoneBodiesDrawn));
        if (rockDebugMaxBodyBoneBodiesDrawn < 0) {
            rockDebugMaxBodyBoneBodiesDrawn = 0;
        } else if (rockDebugMaxBodyBoneBodiesDrawn > 64) {
            rockDebugMaxBodyBoneBodiesDrawn = 64;
        }
        rockDebugDrawWeaponColliders = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawWeaponColliders", rockDebugDrawWeaponColliders);
        rockDebugDrawNativeScopeActivation =
            ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawNativeScopeActivation", rockDebugDrawNativeScopeActivation);
        rockDebugDrawAuthoredGripActivationZones =
            ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawAuthoredGripActivationZones", rockDebugDrawAuthoredGripActivationZones);
        rockDebugDrawGunstockAlignment =
            ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawGunstockAlignment", rockDebugDrawGunstockAlignment);
        rockDebugDrawDynamicWeaponColliders =
            ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawDynamicWeaponColliders", rockDebugDrawDynamicWeaponColliders);
        rockDebugMaxWeaponBodiesDrawn =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxWeaponBodiesDrawn", rockDebugMaxWeaponBodiesDrawn));

        rockDebugDumpWeaponAnimNodes = ini.GetBoolValue(DEBUG_LOGGING_SECTION, "bDumpWeaponAnimNodes", rockDebugDumpWeaponAnimNodes);
        rockDebugWeaponAnimNodeDumpIntervalFrames =
            static_cast<int>(ini.GetLongValue(DEBUG_LOGGING_SECTION, "iWeaponAnimNodeDumpIntervalFrames", rockDebugWeaponAnimNodeDumpIntervalFrames));
        if (rockDebugWeaponAnimNodeDumpIntervalFrames < 1) {
            rockDebugWeaponAnimNodeDumpIntervalFrames = 1;
        }
        rockDebugMaxShapeCapturesPerFrame =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxShapeCapturesPerFrame", rockDebugMaxShapeCapturesPerFrame));
        rockDebugMaxConvexSupportVertices =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxConvexSupportVertices", rockDebugMaxConvexSupportVertices));
        rockDebugMaxCompoundChildren =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxCompoundChildren", rockDebugMaxCompoundChildren));
        rockDebugMaxCompoundDepth = static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxCompoundDepth", rockDebugMaxCompoundDepth));
        rockDebugMaxShapeQueuedJobs =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxShapeQueuedJobs", rockDebugMaxShapeQueuedJobs));
        rockDebugMaxShapeCompletedJobs =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxShapeCompletedJobs", rockDebugMaxShapeCompletedJobs));
        rockDebugMaxShapeUploadsPerFrame =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxShapeUploadsPerFrame", rockDebugMaxShapeUploadsPerFrame));
        rockDebugMaxShapeCacheEntries =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxShapeCacheEntries", rockDebugMaxShapeCacheEntries));
        rockDebugMaxShapeCacheBytes =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxShapeCacheBytes", rockDebugMaxShapeCacheBytes));
        rockDebugMaxBodyInstances =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxBodyInstances", rockDebugMaxBodyInstances));
        rockDebugMaxLineVertices =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxLineVertices", rockDebugMaxLineVertices));
        rockDebugMaxTextVertices =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iMaxTextVertices", rockDebugMaxTextVertices));
        debug_overlay_runtime::RequestedLimits requestedOverlayLimits{};
        requestedOverlayLimits.maxShapeCapturesPerFrame = rockDebugMaxShapeCapturesPerFrame;
        requestedOverlayLimits.maxConvexSupportVertices = rockDebugMaxConvexSupportVertices;
        requestedOverlayLimits.maxCompoundChildren = rockDebugMaxCompoundChildren;
        requestedOverlayLimits.maxCompoundDepth = rockDebugMaxCompoundDepth;
        requestedOverlayLimits.maxShapeQueuedJobs = rockDebugMaxShapeQueuedJobs;
        requestedOverlayLimits.maxShapeCompletedJobs = rockDebugMaxShapeCompletedJobs;
        requestedOverlayLimits.maxShapeUploadsPerFrame = rockDebugMaxShapeUploadsPerFrame;
        requestedOverlayLimits.maxShapeCacheEntries = rockDebugMaxShapeCacheEntries;
        requestedOverlayLimits.maxShapeCacheBytes = rockDebugMaxShapeCacheBytes;
        requestedOverlayLimits.maxBodyInstances = rockDebugMaxBodyInstances;
        requestedOverlayLimits.maxLineVertices = rockDebugMaxLineVertices;
        requestedOverlayLimits.maxTextVertices = rockDebugMaxTextVertices;
        const auto overlayLimits = debug_overlay_runtime::sanitize(requestedOverlayLimits);
        rockDebugMaxShapeCapturesPerFrame = static_cast<int>(overlayLimits.maxShapeCapturesPerFrame);
        rockDebugMaxConvexSupportVertices = static_cast<int>(overlayLimits.maxConvexSupportVertices);
        rockDebugMaxCompoundChildren = static_cast<int>(overlayLimits.maxCompoundChildren);
        rockDebugMaxCompoundDepth = static_cast<int>(overlayLimits.maxCompoundDepth);
        rockDebugMaxShapeQueuedJobs = static_cast<int>(overlayLimits.maxShapeQueuedJobs);
        rockDebugMaxShapeCompletedJobs = static_cast<int>(overlayLimits.maxShapeCompletedJobs);
        rockDebugMaxShapeUploadsPerFrame = static_cast<int>(overlayLimits.maxShapeUploadsPerFrame);
        rockDebugMaxShapeCacheEntries = static_cast<int>(overlayLimits.maxShapeCacheEntries);
        rockDebugMaxShapeCacheBytes = static_cast<int>(overlayLimits.maxShapeCacheBytes);
        rockDebugMaxBodyInstances = static_cast<int>(overlayLimits.maxBodyInstances);
        rockDebugMaxLineVertices = static_cast<int>(overlayLimits.maxLineVertices);
        rockDebugMaxTextVertices = static_cast<int>(overlayLimits.maxTextVertices);
        rockDebugUseBoundsForHeavyConvex =
            ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bUseBoundsForHeavyConvex", rockDebugUseBoundsForHeavyConvex);
        rockDebugVideoSyncMarker = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bVideoSyncMarker", rockDebugVideoSyncMarker);
        rockDebugVideoSyncMarkerSize =
            static_cast<float>(ini.GetDoubleValue(DEBUG_OVERLAY_SECTION, "fVideoSyncMarkerSize", rockDebugVideoSyncMarkerSize));

        rockDebugVerboseLogging = ini.GetBoolValue(DEBUG_LOGGING_SECTION, "bVerbose", rockDebugVerboseLogging);
        rockDebugGrabFrameLogging = ini.GetBoolValue(DEBUG_LOGGING_SECTION, "bGrabFrame", rockDebugGrabFrameLogging);
        rockDebugColliderClockLogging = ini.GetBoolValue(DEBUG_LOGGING_SECTION, "bColliderClock", rockDebugColliderClockLogging);
        rockDebugDynamicWeaponLogging = ini.GetBoolValue(DEBUG_LOGGING_SECTION, "bDynamicWeapon", rockDebugDynamicWeaponLogging);
        rockDebugGrabTimelineTrace = ini.GetBoolValue(DEBUG_LOGGING_SECTION, "bGrabTimeline", rockDebugGrabTimelineTrace);
        rockDebugGrabAfterSolveAnomalySampling =
            ini.GetBoolValue(DEBUG_LOGGING_SECTION, "bGrabAfterSolveAnomalySampling", rockDebugGrabAfterSolveAnomalySampling);
        rockDebugGrabTimelineTraceIntervalFrames =
            static_cast<int>(ini.GetLongValue(DEBUG_LOGGING_SECTION, "iGrabTimelineIntervalFrames", rockDebugGrabTimelineTraceIntervalFrames));
        if (rockDebugGrabTimelineTraceIntervalFrames < 1) {
            rockDebugGrabTimelineTraceIntervalFrames = 1;
        }
        rockDebugGrabTransformTelemetry = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bGrabTransformTelemetry", rockDebugGrabTransformTelemetry);
        rockDebugGrabTransformTelemetryText = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bGrabTransformTelemetryText", rockDebugGrabTransformTelemetryText);
        rockDebugGrabTransformTelemetryAxes = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bGrabTransformTelemetryAxes", rockDebugGrabTransformTelemetryAxes);
        rockDebugGrabTransformTelemetryLogIntervalFrames =
            static_cast<int>(ini.GetLongValue(DEBUG_LOGGING_SECTION, "iGrabTransformTelemetryIntervalFrames", rockDebugGrabTransformTelemetryLogIntervalFrames));
        if (rockDebugGrabTransformTelemetryLogIntervalFrames < 1) {
            rockDebugGrabTransformTelemetryLogIntervalFrames = 1;
        }
        rockDebugGrabTransformTelemetryTextMode =
            static_cast<int>(ini.GetLongValue(DEBUG_OVERLAY_SECTION, "iGrabTransformTelemetryTextMode", rockDebugGrabTransformTelemetryTextMode));
        if (rockDebugGrabTransformTelemetryTextMode < 0 || rockDebugGrabTransformTelemetryTextMode > 1) {
            rockDebugGrabTransformTelemetryTextMode = 0;
        }
        rockDebugShowGrabNotifications = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bShowGrabNotifications", rockDebugShowGrabNotifications);
        rockDebugShowWeaponNotifications = ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bShowWeaponNotifications", rockDebugShowWeaponNotifications);
        rockDebugWeaponOmodDumpEnabled = ini.GetBoolValue(DEBUG_LOGGING_SECTION, "bWeaponOmodDump", rockDebugWeaponOmodDumpEnabled);
        rockDebugWeaponOmodCoverageAudit = ini.GetBoolValue(DEBUG_LOGGING_SECTION, "bWeaponOmodCoverageAudit", rockDebugWeaponOmodCoverageAudit);
        rockDebugWeaponOmodCoverageAuditIntervalFrames = static_cast<int>(
            ini.GetLongValue(DEBUG_LOGGING_SECTION, "iWeaponOmodCoverageAuditIntervalFrames", rockDebugWeaponOmodCoverageAuditIntervalFrames));
        if (rockDebugWeaponOmodCoverageAuditIntervalFrames < 30) {
            rockDebugWeaponOmodCoverageAuditIntervalFrames = 30;
        }
        rockDebugHandTransformParity = ini.GetBoolValue(DEBUG_LOGGING_SECTION, "bHandTransformParity", rockDebugHandTransformParity);
        rockDebugWorldObjectOriginDiagnostics =
            ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bWorldObjectOriginDiagnostics", rockDebugWorldObjectOriginDiagnostics);
        rockDebugWorldObjectOriginLogIntervalFrames =
            static_cast<int>(ini.GetLongValue(DEBUG_LOGGING_SECTION, "iWorldObjectOriginIntervalFrames", rockDebugWorldObjectOriginLogIntervalFrames));
        if (rockDebugWorldObjectOriginLogIntervalFrames < 1) {
            rockDebugWorldObjectOriginLogIntervalFrames = 1;
        }
        rockDebugWorldObjectOriginMismatchWarnGameUnits = static_cast<float>(
            ini.GetDoubleValue(DEBUG_LOGGING_SECTION, "fWorldObjectOriginMismatchWarnGameUnits", rockDebugWorldObjectOriginMismatchWarnGameUnits));
        if (!std::isfinite(rockDebugWorldObjectOriginMismatchWarnGameUnits) || rockDebugWorldObjectOriginMismatchWarnGameUnits < 0.0f) {
            rockDebugWorldObjectOriginMismatchWarnGameUnits = 0.0f;
        }
        rockDebugCustomCalibrationOffset =
            ini.GetBoolValue(DEBUG_OVERLAY_SECTION, "bDrawCustomCalibrationOffset", rockDebugCustomCalibrationOffset);
        rockDebugShowRootFlattenedFingerSkeletonMarkers =
            ini.GetBoolValue(DEBUG_SKELETON_SECTION, "bShowRootFlattenedFingerMarkers", rockDebugShowRootFlattenedFingerSkeletonMarkers);
        rockDebugShowSkeletonBoneVisualizer =
            ini.GetBoolValue(DEBUG_SKELETON_SECTION, "bShowBoneVisualizer", rockDebugShowSkeletonBoneVisualizer);
        rockDebugSkeletonBoneMode = static_cast<int>(ini.GetLongValue(DEBUG_SKELETON_SECTION, "iBoneMode", rockDebugSkeletonBoneMode));
        if (rockDebugSkeletonBoneMode < 0 || rockDebugSkeletonBoneMode > 3) {
            rockDebugSkeletonBoneMode = 1;
        }
        rockDebugSkeletonBoneSource = static_cast<int>(ini.GetLongValue(DEBUG_SKELETON_SECTION, "iBoneSource", rockDebugSkeletonBoneSource));
        if (rockDebugSkeletonBoneSource != 1 && rockDebugSkeletonBoneSource != 2) {
            rockDebugSkeletonBoneSource = 1;
        }
        rockDebugDrawSkeletonBoneAxes = ini.GetBoolValue(DEBUG_SKELETON_SECTION, "bDrawBoneAxes", rockDebugDrawSkeletonBoneAxes);
        rockDebugLogSkeletonBones = ini.GetBoolValue(DEBUG_SKELETON_SECTION, "bLogBones", rockDebugLogSkeletonBones);
        rockDebugLogSkeletonBoneTruncation =
            ini.GetBoolValue(DEBUG_SKELETON_SECTION, "bLogBoneTruncation", rockDebugLogSkeletonBoneTruncation);
        rockDebugSkeletonBoneLogFilter = ini.GetValue(DEBUG_SKELETON_SECTION, "sBoneLogFilter", rockDebugSkeletonBoneLogFilter.c_str());
        rockDebugSkeletonAxisBoneFilter = ini.GetValue(DEBUG_SKELETON_SECTION, "sAxisBoneFilter", rockDebugSkeletonAxisBoneFilter.c_str());
        rockDebugSkeletonBoneLogIntervalFrames =
            static_cast<int>(ini.GetLongValue(DEBUG_SKELETON_SECTION, "iBoneLogIntervalFrames", rockDebugSkeletonBoneLogIntervalFrames));
        if (rockDebugSkeletonBoneLogIntervalFrames < 1) {
            rockDebugSkeletonBoneLogIntervalFrames = 1;
        }
        rockDebugMaxSkeletonBonesDrawn =
            static_cast<int>(ini.GetLongValue(DEBUG_SKELETON_SECTION, "iMaxBonesDrawn", rockDebugMaxSkeletonBonesDrawn));
        if (rockDebugMaxSkeletonBonesDrawn < 0) {
            rockDebugMaxSkeletonBonesDrawn = 0;
        } else if (rockDebugMaxSkeletonBonesDrawn > 768) {
            rockDebugMaxSkeletonBonesDrawn = 768;
        }
        rockDebugMaxSkeletonBoneAxesDrawn =
            static_cast<int>(ini.GetLongValue(DEBUG_SKELETON_SECTION, "iMaxBoneAxesDrawn", rockDebugMaxSkeletonBoneAxesDrawn));
        if (rockDebugMaxSkeletonBoneAxesDrawn < 0) {
            rockDebugMaxSkeletonBoneAxesDrawn = 0;
        } else if (rockDebugMaxSkeletonBoneAxesDrawn > 768) {
            rockDebugMaxSkeletonBoneAxesDrawn = 768;
        }
        rockDebugRootFlattenedFingerSkeletonMarkerSize =
            static_cast<float>(ini.GetDoubleValue(DEBUG_SKELETON_SECTION, "fRootFlattenedFingerMarkerSize", rockDebugRootFlattenedFingerSkeletonMarkerSize));
        if (rockDebugRootFlattenedFingerSkeletonMarkerSize < 0.1f) {
            rockDebugRootFlattenedFingerSkeletonMarkerSize = 0.1f;
        }
        rockDebugSkeletonBonePointSize =
            static_cast<float>(ini.GetDoubleValue(DEBUG_SKELETON_SECTION, "fBonePointSize", rockDebugSkeletonBonePointSize));
        if (rockDebugSkeletonBonePointSize < 0.1f) {
            rockDebugSkeletonBonePointSize = 0.1f;
        }
        rockDebugSkeletonBoneAxisLength =
            static_cast<float>(ini.GetDoubleValue(DEBUG_SKELETON_SECTION, "fBoneAxisLength", rockDebugSkeletonBoneAxisLength));
        if (rockDebugSkeletonBoneAxisLength < 0.1f) {
            rockDebugSkeletonBoneAxisLength = 0.1f;
        }

        rockHandColliderRuntimeMode = static_cast<int>(ini.GetLongValue(SECTION, "iHandColliderRuntimeMode", rockHandColliderRuntimeMode));
        if (rockHandColliderRuntimeMode < 0 || rockHandColliderRuntimeMode > 1) {
            ROCK_LOG_WARN(Config, "Invalid iHandColliderRuntimeMode={} - using BoneDerivedHands", rockHandColliderRuntimeMode);
            rockHandColliderRuntimeMode = 1;
        }
        rockBodyBoneCollidersEnabled = ini.GetBoolValue(EXPERIMENTAL_SECTION, "bBodyBoneCollidersEnabled", rockBodyBoneCollidersEnabled);
        rockBodyBoneLegAndFootCollidersEnabled =
            ini.GetBoolValue(EXPERIMENTAL_SECTION, "bBodyBoneLegAndFootCollidersEnabled", rockBodyBoneLegAndFootCollidersEnabled);
        rockBodyBoneCollisionStaticWorldEnabled = ini.GetBoolValue(SECTION, "bBodyBoneCollisionStaticWorldEnabled", rockBodyBoneCollisionStaticWorldEnabled);
        auto readBodyBoneScale = [&](const char* key, float currentValue) {
            const auto value = static_cast<float>(ini.GetDoubleValue(SECTION, key, currentValue));
            if (!std::isfinite(value)) {
                ROCK_LOG_WARN(Config, "Invalid {}={} - using 1.0", key, value);
                return 1.0f;
            }
            if (value < 0.05f || value > 8.0f) {
                const float clamped = std::clamp(value, 0.05f, 8.0f);
                ROCK_LOG_WARN(Config, "Clamped {} from {} to {}", key, value, clamped);
                return clamped;
            }
            return value;
        };
        rockBodyBoneColliderStandardRadiusScale = readBodyBoneScale("fBodyBoneColliderStandardRadiusScale", rockBodyBoneColliderStandardRadiusScale);
        rockBodyBoneColliderStandardLengthScale = readBodyBoneScale("fBodyBoneColliderStandardLengthScale", rockBodyBoneColliderStandardLengthScale);
        rockBodyBoneColliderStandardConvexRadiusScale =
            readBodyBoneScale("fBodyBoneColliderStandardConvexRadiusScale", rockBodyBoneColliderStandardConvexRadiusScale);
        rockBodyBoneColliderPowerArmorRadiusScale = readBodyBoneScale("fBodyBoneColliderPowerArmorRadiusScale", rockBodyBoneColliderPowerArmorRadiusScale);
        rockBodyBoneColliderPowerArmorLengthScale = readBodyBoneScale("fBodyBoneColliderPowerArmorLengthScale", rockBodyBoneColliderPowerArmorLengthScale);
        rockBodyBoneColliderPowerArmorConvexRadiusScale =
            readBodyBoneScale("fBodyBoneColliderPowerArmorConvexRadiusScale", rockBodyBoneColliderPowerArmorConvexRadiusScale);
        rockBodyBoneColliderTorsoRadiusScale = readBodyBoneScale("fBodyBoneColliderTorsoRadiusScale", rockBodyBoneColliderTorsoRadiusScale);
        rockBodyBoneColliderArmRadiusScale = readBodyBoneScale("fBodyBoneColliderArmRadiusScale", rockBodyBoneColliderArmRadiusScale);
        rockBodyBoneColliderLegRadiusScale = readBodyBoneScale("fBodyBoneColliderLegRadiusScale", rockBodyBoneColliderLegRadiusScale);
        rockBodyBoneColliderFootRadiusScale = readBodyBoneScale("fBodyBoneColliderFootRadiusScale", rockBodyBoneColliderFootRadiusScale);
        rockBodyBoneColliderTorsoLengthScale = readBodyBoneScale("fBodyBoneColliderTorsoLengthScale", rockBodyBoneColliderTorsoLengthScale);
        rockBodyBoneColliderArmLengthScale = readBodyBoneScale("fBodyBoneColliderArmLengthScale", rockBodyBoneColliderArmLengthScale);
        rockBodyBoneColliderLegLengthScale = readBodyBoneScale("fBodyBoneColliderLegLengthScale", rockBodyBoneColliderLegLengthScale);
        rockBodyBoneColliderFootLengthScale = readBodyBoneScale("fBodyBoneColliderFootLengthScale", rockBodyBoneColliderFootLengthScale);
        rockBodyBoneColliderZoneScaleOverrides = ini.GetValue(SECTION, "sBodyBoneColliderZoneScaleOverrides", rockBodyBoneColliderZoneScaleOverrides.c_str());
        rockBodyBoneColliderRadiusScaleOverrides = ini.GetValue(SECTION, "sBodyBoneColliderRadiusScaleOverrides", rockBodyBoneColliderRadiusScaleOverrides.c_str());
        rockHandCollisionStaticWorldEnabled = ini.GetBoolValue(SECTION, "bHandCollisionStaticWorldEnabled", rockHandCollisionStaticWorldEnabled);
        rockGlobalSurfaceGrabEnabled = ini.GetBoolValue(SECTION, "bGlobalSurfaceGrabEnabled", rockGlobalSurfaceGrabEnabled);
        rockExperimentalSurfaceMeshGrabEnabled =
            ini.GetBoolValue(
                EXPERIMENTAL_SECTION,
                "bExperimentalSurfaceMeshGrabEnabled",
                rockExperimentalSurfaceMeshGrabEnabled);
        rockExperimentalSurfaceMeshGrabMaxProjectionDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(
                EXPERIMENTAL_SECTION,
                "fExperimentalSurfaceMeshGrabMaxProjectionDistanceGameUnits",
                rockExperimentalSurfaceMeshGrabMaxProjectionDistanceGameUnits));
        if (!std::isfinite(
                rockExperimentalSurfaceMeshGrabMaxProjectionDistanceGameUnits)) {
            rockExperimentalSurfaceMeshGrabMaxProjectionDistanceGameUnits =
                48.0f;
        }
        rockExperimentalSurfaceMeshGrabMaxProjectionDistanceGameUnits =
            std::clamp(
                rockExperimentalSurfaceMeshGrabMaxProjectionDistanceGameUnits,
                1.0f,
                128.0f);
        rockExperimentalSurfaceMeshGrabMaxTriangles = std::clamp(
            static_cast<int>(ini.GetLongValue(
                EXPERIMENTAL_SECTION,
                "iExperimentalSurfaceMeshGrabMaxTriangles",
                rockExperimentalSurfaceMeshGrabMaxTriangles)),
            256,
            100000);
        rockExperimentalSurfaceMeshGrabMaxPatchTriangles = std::clamp(
            static_cast<int>(ini.GetLongValue(
                EXPERIMENTAL_SECTION,
                "iExperimentalSurfaceMeshGrabMaxPatchTriangles",
                rockExperimentalSurfaceMeshGrabMaxPatchTriangles)),
            64,
            2048);
        rockExperimentalSurfaceMeshGrabMaxPatchTriangles = std::min(
            rockExperimentalSurfaceMeshGrabMaxPatchTriangles,
            rockExperimentalSurfaceMeshGrabMaxTriangles);
        rockHandBoneColliderRadiusScaleOverrides = ini.GetValue(SECTION, "sHandBoneColliderRadiusScaleOverrides", rockHandBoneColliderRadiusScaleOverrides.c_str());
        rockHandPalmColliderDimensionScaleOverrides =
            ini.GetValue(SECTION, "sHandPalmColliderDimensionScaleOverrides", rockHandPalmColliderDimensionScaleOverrides.c_str());
        rockHandBoneCollidersRequirePalmAnchor = ini.GetBoolValue(SECTION, "bHandBoneCollidersRequirePalmAnchor", rockHandBoneCollidersRequirePalmAnchor);
        rockHandBoneCollidersRequireAllFingerBones = ini.GetBoolValue(SECTION, "bHandBoneCollidersRequireAllFingerBones", rockHandBoneCollidersRequireAllFingerBones);
        rockNearDetectionRange = static_cast<float>(ini.GetDoubleValue(SECTION, "fNearDetectionRange", rockNearDetectionRange));
        rockFarDetectionRange = static_cast<float>(ini.GetDoubleValue(SECTION, "fFarDetectionRange", rockFarDetectionRange));
        rockNearCastRadiusGameUnits = readClampedFloat(ini,
            SECTION,
            "fNearCastRadiusGameUnits",
            rockNearCastRadiusGameUnits,
            kDefaultNearCastRadiusGameUnits,
            0.0f,
            kDefaultNearCastRadiusGameUnits);
        rockNearCastDistanceGameUnits = readClampedFloat(ini,
            SECTION,
            "fNearCastDistanceGameUnits",
            rockNearCastDistanceGameUnits,
            kDefaultNearCastDistanceGameUnits,
            0.1f,
            kDefaultNearCastDistanceGameUnits);
        rockFarCastRadiusGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fFarCastRadiusGameUnits", rockFarCastRadiusGameUnits));
        rockCloseSelectionAngleDegrees =
            readSelectionAimAngleDegrees(ini, SECTION, "iCloseSelectionAngleDegrees", rockCloseSelectionAngleDegrees);
        rockFarSelectionAngleDegrees =
            readSelectionAimAngleDegrees(ini, SECTION, "iFarSelectionAngleDegrees", rockFarSelectionAngleDegrees);
        rockFarSelectionHmdConeEnabled = ini.GetBoolValue(SECTION, "bFarSelectionHmdConeEnabled", rockFarSelectionHmdConeEnabled);
        rockFarSelectionHmdConeHalfAngleDegrees =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fFarSelectionHmdConeHalfAngleDegrees", rockFarSelectionHmdConeHalfAngleDegrees));
        rockFarSelectionBlockedReferenceFormIds =
            ini.GetValue(SECTION, "sFarSelectionBlockedReferenceFormIDs", rockFarSelectionBlockedReferenceFormIds.c_str());
        rockFarSelectionBlockedBaseFormIds = ini.GetValue(SECTION, "sFarSelectionBlockedBaseFormIDs", rockFarSelectionBlockedBaseFormIds.c_str());
        rockFarSelectionBlockedFormTypes = ini.GetValue(SECTION, "sFarSelectionBlockedFormTypes", rockFarSelectionBlockedFormTypes.c_str());
        rockFarSelectionBlockedLayers = ini.GetValue(SECTION, "sFarSelectionBlockedLayers", rockFarSelectionBlockedLayers.c_str());
        const float sanitizedFarSelectionHmdConeHalfAngleDegrees =
            selection_query_policy::sanitizeFarSelectionHmdConeHalfAngleDegrees(rockFarSelectionHmdConeHalfAngleDegrees);
        if (sanitizedFarSelectionHmdConeHalfAngleDegrees != rockFarSelectionHmdConeHalfAngleDegrees) {
            ROCK_LOG_WARN(Config,
                "Invalid fFarSelectionHmdConeHalfAngleDegrees={} -- using {}",
                rockFarSelectionHmdConeHalfAngleDegrees,
                sanitizedFarSelectionHmdConeHalfAngleDegrees);
            rockFarSelectionHmdConeHalfAngleDegrees = sanitizedFarSelectionHmdConeHalfAngleDegrees;
        }
        rockCloseSelectionBehindPalmToleranceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fCloseSelectionBehindPalmToleranceGameUnits", rockCloseSelectionBehindPalmToleranceGameUnits));
        if (!std::isfinite(rockCloseSelectionBehindPalmToleranceGameUnits) || rockCloseSelectionBehindPalmToleranceGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fCloseSelectionBehindPalmToleranceGameUnits={} -- using 2.0", rockCloseSelectionBehindPalmToleranceGameUnits);
            rockCloseSelectionBehindPalmToleranceGameUnits = 2.0f;
        }
        rockSelectionShapeCastFilterInfo =
            readHexFilter("sSelectionShapeCastFilterInfo", rockSelectionShapeCastFilterInfo, selection_query_policy::kDefaultShapeCastFilterInfo);
        rockFarClipRayFilterInfo = readHexFilter("sFarClipRayFilterInfo", rockFarClipRayFilterInfo, selection_query_policy::kDefaultFarClipRayFilterInfo);
        rockPullApplyVelocityTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullApplyVelocityTime", rockPullApplyVelocityTime));
        rockPullOwnerGraceSeconds = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullOwnerGraceSeconds", rockPullOwnerGraceSeconds));
        if (!std::isfinite(rockPullOwnerGraceSeconds) || rockPullOwnerGraceSeconds < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fPullOwnerGraceSeconds={} -- using 1.0", rockPullOwnerGraceSeconds);
            rockPullOwnerGraceSeconds = 1.0f;
        }
        rockPullOwnerGraceSeconds = std::clamp(rockPullOwnerGraceSeconds, 0.0f, 3.0f);
        rockPullTrackHandTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullTrackHandTime", rockPullTrackHandTime));
        rockPullDestinationZOffsetHavok = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullDestinationZOffsetHavok", rockPullDestinationZOffsetHavok));
        rockPullDurationA = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullDurationA", rockPullDurationA));
        rockPullDurationB = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullDurationB", rockPullDurationB));
        rockPullDurationC = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullDurationC", rockPullDurationC));
        rockPullMaxVelocityHavok = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullMaxVelocityHavok", rockPullMaxVelocityHavok));
        rockPullAutoGrabDistanceGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullAutoGrabDistanceGameUnits", rockPullAutoGrabDistanceGameUnits));
        rockPullCatchRetryMaxTimeSeconds = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullCatchRetryMaxTimeSeconds", rockPullCatchRetryMaxTimeSeconds));
        if (!std::isfinite(rockPullCatchRetryMaxTimeSeconds) || rockPullCatchRetryMaxTimeSeconds < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fPullCatchRetryMaxTimeSeconds={} -- using 0.65", rockPullCatchRetryMaxTimeSeconds);
            rockPullCatchRetryMaxTimeSeconds = 0.65f;
        }
        rockPullCatchWideReacquireEnabled = ini.GetBoolValue(SECTION, "bPullCatchWideReacquireEnabled", rockPullCatchWideReacquireEnabled);
        rockPullCatchWideReacquireRadiusGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fPullCatchWideReacquireRadiusGameUnits", rockPullCatchWideReacquireRadiusGameUnits));
        if (!std::isfinite(rockPullCatchWideReacquireRadiusGameUnits) || rockPullCatchWideReacquireRadiusGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fPullCatchWideReacquireRadiusGameUnits={} -- using 32.0", rockPullCatchWideReacquireRadiusGameUnits);
            rockPullCatchWideReacquireRadiusGameUnits = 32.0f;
        }
        rockPullCatchWideReacquireRadiusGameUnits = std::clamp(rockPullCatchWideReacquireRadiusGameUnits, 0.0f, 120.0f);
        rockPullCatchWideReacquireMaxBodyDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fPullCatchWideReacquireMaxBodyDistanceGameUnits", rockPullCatchWideReacquireMaxBodyDistanceGameUnits));
        if (!std::isfinite(rockPullCatchWideReacquireMaxBodyDistanceGameUnits) || rockPullCatchWideReacquireMaxBodyDistanceGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fPullCatchWideReacquireMaxBodyDistanceGameUnits={} -- using 42.0", rockPullCatchWideReacquireMaxBodyDistanceGameUnits);
            rockPullCatchWideReacquireMaxBodyDistanceGameUnits = 42.0f;
        }
        rockPullCatchWideReacquireMaxBodyDistanceGameUnits = std::clamp(rockPullCatchWideReacquireMaxBodyDistanceGameUnits, 0.0f, 160.0f);
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
        rockGrabMaxForceToMassRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxForceToMassRatio", rockGrabMaxForceToMassRatio));
        rockForceGrabAttachSettleSeconds = readClampedFloat(ini,
            SECTION,
            "fForceGrabAttachSettleSeconds",
            rockForceGrabAttachSettleSeconds,
            0.10f,
            0.0f,
            1.0f);
        rockGrabEffectiveMotorMassFloorEnabled =
            ini.GetBoolValue(SECTION, "bGrabEffectiveMotorMassFloorEnabled", rockGrabEffectiveMotorMassFloorEnabled);
        rockGrabEffectiveMotorMassFloor = readClampedFloat(ini,
            SECTION,
            "fGrabEffectiveMotorMassFloor",
            rockGrabEffectiveMotorMassFloor,
            kDefaultGrabEffectiveMotorMassFloor,
            0.0f,
            100.0f);
        rockGrabPhysicsRateForceScalingEnabled =
            ini.GetBoolValue(SECTION, "bGrabPhysicsRateForceScalingEnabled", rockGrabPhysicsRateForceScalingEnabled);
        rockGrabPhysicsRateReferenceHz = readClampedFloat(ini,
            SECTION,
            "fGrabPhysicsRateReferenceHz",
            rockGrabPhysicsRateReferenceHz,
            kDefaultGrabPhysicsRateReferenceHz,
            1.0f,
            240.0f);
        rockGrabPhysicsRateForceScaleExponent = readClampedFloat(ini,
            SECTION,
            "fGrabPhysicsRateForceScaleExponent",
            rockGrabPhysicsRateForceScaleExponent,
            kDefaultGrabPhysicsRateForceScaleExponent,
            0.0f,
            2.0f);
        rockGrabPhysicsRateMinForceScale = readClampedFloat(ini,
            SECTION,
            "fGrabPhysicsRateMinForceScale",
            rockGrabPhysicsRateMinForceScale,
            kDefaultGrabPhysicsRateMinForceScale,
            0.1f,
            2.0f);
        rockGrabPhysicsRateMaxForceScale = readClampedFloat(ini,
            SECTION,
            "fGrabPhysicsRateMaxForceScale",
            rockGrabPhysicsRateMaxForceScale,
            kDefaultGrabPhysicsRateMaxForceScale,
            rockGrabPhysicsRateMinForceScale,
            3.0f);

        rockGrabForceFadeInTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabForceFadeInTime", rockGrabForceFadeInTime));
        rockRightGrabAuthorityProxyOffsetGameUnits.x =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fRightGrabAuthorityProxyOffsetXGameUnits", rockRightGrabAuthorityProxyOffsetGameUnits.x));
        rockRightGrabAuthorityProxyOffsetGameUnits.y =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fRightGrabAuthorityProxyOffsetYGameUnits", rockRightGrabAuthorityProxyOffsetGameUnits.y));
        rockRightGrabAuthorityProxyOffsetGameUnits.z =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fRightGrabAuthorityProxyOffsetZGameUnits", rockRightGrabAuthorityProxyOffsetGameUnits.z));
        rockLeftGrabAuthorityProxyOffsetGameUnits.x =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fLeftGrabAuthorityProxyOffsetXGameUnits", rockLeftGrabAuthorityProxyOffsetGameUnits.x));
        rockLeftGrabAuthorityProxyOffsetGameUnits.y =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fLeftGrabAuthorityProxyOffsetYGameUnits", rockLeftGrabAuthorityProxyOffsetGameUnits.y));
        rockLeftGrabAuthorityProxyOffsetGameUnits.z =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fLeftGrabAuthorityProxyOffsetZGameUnits", rockLeftGrabAuthorityProxyOffsetGameUnits.z));
        rockRightCustomOGAOffsetGameUnits.x =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fRightCustomOGAOffsetXGameUnits", rockRightCustomOGAOffsetGameUnits.x));
        rockRightCustomOGAOffsetGameUnits.y =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fRightCustomOGAOffsetYGameUnits", rockRightCustomOGAOffsetGameUnits.y));
        rockRightCustomOGAOffsetGameUnits.z =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fRightCustomOGAOffsetZGameUnits", rockRightCustomOGAOffsetGameUnits.z));
        rockLeftCustomOGAOffsetGameUnits.x =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fLeftCustomOGAOffsetXGameUnits", rockLeftCustomOGAOffsetGameUnits.x));
        rockLeftCustomOGAOffsetGameUnits.y =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fLeftCustomOGAOffsetYGameUnits", rockLeftCustomOGAOffsetGameUnits.y));
        rockLeftCustomOGAOffsetGameUnits.z =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fLeftCustomOGAOffsetZGameUnits", rockLeftCustomOGAOffsetGameUnits.z));
        rockGrabLooseWeaponSharedConstraintLinearTauMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintLinearTauMultiplier",
            rockGrabLooseWeaponSharedConstraintLinearTauMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintLinearTauMultiplier,
            0.05f,
            4.0f);
        rockGrabLooseWeaponSharedConstraintAngularTauMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintAngularTauMultiplier",
            rockGrabLooseWeaponSharedConstraintAngularTauMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintAngularTauMultiplier,
            0.05f,
            4.0f);
        rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintCollisionTauMultiplier",
            rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintCollisionTauMultiplier,
            0.05f,
            4.0f);
        rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintLinearDampingMultiplier",
            rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintLinearDampingMultiplier,
            0.05f,
            4.0f);
        rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintAngularDampingMultiplier",
            rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintAngularDampingMultiplier,
            0.05f,
            4.0f);
        rockGrabLooseWeaponSharedConstraintMaxForceMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintMaxForceMultiplier",
            rockGrabLooseWeaponSharedConstraintMaxForceMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintMaxForceMultiplier,
            0.05f,
            8.0f);
        rockGrabLooseWeaponSharedConstraintAngularForceMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintAngularForceMultiplier",
            rockGrabLooseWeaponSharedConstraintAngularForceMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintAngularForceMultiplier,
            0.05f,
            8.0f);
        rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier",
            rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier,
            0.05f,
            4.0f);
        rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier",
            rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier,
            0.05f,
            4.0f);

        rockGrabTauMin = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauMin", rockGrabTauMin));
        rockGrabTauLerpSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauLerpSpeed", rockGrabTauLerpSpeed));
        rockGrabLongObjectAngularScalingEnabled = ini.GetBoolValue(SECTION, "bGrabLongObjectAngularScalingEnabled", rockGrabLongObjectAngularScalingEnabled);
        rockGrabLongObjectReferenceLeverGameUnits = static_cast<float>(
            ini.GetDoubleValue(SECTION, "fGrabLongObjectReferenceLeverGameUnits", rockGrabLongObjectReferenceLeverGameUnits));
        rockGrabLongObjectReferenceLeverGameUnits =
            std::clamp(std::isfinite(rockGrabLongObjectReferenceLeverGameUnits) ? rockGrabLongObjectReferenceLeverGameUnits : kDefaultGrabLongObjectReferenceLeverGameUnits,
                1.0f,
                240.0f);
        rockGrabLongObjectMinAngularScale = static_cast<float>(
            ini.GetDoubleValue(SECTION, "fGrabLongObjectMinAngularScale", rockGrabLongObjectMinAngularScale));
        rockGrabLongObjectMinAngularScale =
            std::clamp(std::isfinite(rockGrabLongObjectMinAngularScale) ? rockGrabLongObjectMinAngularScale : kDefaultGrabLongObjectMinAngularScale,
                0.05f,
                1.0f);
        rockGrabPivotQualityAngularScalingEnabled =
            ini.GetBoolValue(SECTION, "bGrabPivotQualityAngularScalingEnabled", rockGrabPivotQualityAngularScalingEnabled);
        rockGrabPositionOnlyAngularScale = readClampedFloat(ini,
            SECTION,
            "fGrabPositionOnlyAngularScale",
            rockGrabPositionOnlyAngularScale,
            kDefaultGrabPositionOnlyAngularScale,
            0.05f,
            1.0f);
        rockGrabSmallObjectReferenceLeverGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabSmallObjectReferenceLeverGameUnits",
            rockGrabSmallObjectReferenceLeverGameUnits,
            kDefaultGrabSmallObjectReferenceLeverGameUnits,
            1.0f,
            120.0f);
        rockGrabSmallObjectAngularScale = readClampedFloat(ini,
            SECTION,
            "fGrabSmallObjectAngularScale",
            rockGrabSmallObjectAngularScale,
            kDefaultGrabSmallObjectAngularScale,
            0.05f,
            1.0f);
        rockGrabLowContactSupportAngularScale = readClampedFloat(ini,
            SECTION,
            "fGrabLowContactSupportAngularScale",
            rockGrabLowContactSupportAngularScale,
            kDefaultGrabLowContactSupportAngularScale,
            0.05f,
            1.0f);
        rockGrabMinAngularAuthorityScale = readClampedFloat(ini,
            SECTION,
            "fGrabMinAngularAuthorityScale",
            rockGrabMinAngularAuthorityScale,
            kDefaultGrabMinAngularAuthorityScale,
            0.05f,
            1.0f);
        rockGrabWeakPivotTwistScale = readClampedFloat(ini,
            SECTION,
            "fGrabWeakPivotTwistScale",
            rockGrabWeakPivotTwistScale,
            kDefaultGrabWeakPivotTwistScale,
            0.0f,
            1.0f);

        rockGrabMaxInertiaRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxInertiaRatio", rockGrabMaxInertiaRatio));
        rockGrabMinInertia = readClampedFloat(ini,
            SECTION,
            "fGrabMinInertia",
            rockGrabMinInertia,
            kDefaultGrabMinInertia,
            0.0001f,
            100.0f);

        rockGrabMaxDeviation = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviation", rockGrabMaxDeviation));
        rockGrabMaxDeviationTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviationTime", rockGrabMaxDeviationTime));
        rockGrabButtonID = static_cast<int>(ini.GetLongValue(SECTION, "iGrabButtonID", rockGrabButtonID));
        if (!input_remap_policy::isAllowedGrabButtonId(rockGrabButtonID)) {
            ROCK_LOG_WARN(Config, "iGrabButtonID must be 0..63 and cannot be SteamVR trigger button {}; using 2", input_remap_policy::kOpenVrSteamVrTriggerButtonId);
            rockGrabButtonID = 2;
        }
        rockThrowVelocityMultiplier = static_cast<float>(ini.GetDoubleValue(SECTION, "fThrowVelocityMultiplier", rockThrowVelocityMultiplier));
        rockGrabControllerDerivedThrowVelocityEnabled =
            ini.GetBoolValue(SECTION, "bGrabControllerDerivedThrowVelocityEnabled", rockGrabControllerDerivedThrowVelocityEnabled);
        rockGrabThrowObjectVelocityBlend =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThrowObjectVelocityBlend", rockGrabThrowObjectVelocityBlend));
        rockGrabThrowObjectVelocityBlend = std::clamp(
            std::isfinite(rockGrabThrowObjectVelocityBlend) ? rockGrabThrowObjectVelocityBlend : kDefaultGrabThrowObjectVelocityBlend,
            0.0f,
            1.0f);
        rockGrabThrowTangentialVelocityScale =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThrowTangentialVelocityScale", rockGrabThrowTangentialVelocityScale));
        rockGrabThrowTangentialVelocityScale = std::clamp(
            std::isfinite(rockGrabThrowTangentialVelocityScale) ? rockGrabThrowTangentialVelocityScale : kDefaultGrabThrowTangentialVelocityScale,
            0.0f,
            2.0f);
        rockGrabThrowMaxVelocityHavok = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThrowMaxVelocityHavok", rockGrabThrowMaxVelocityHavok));
        rockGrabThrowMaxVelocityHavok = std::clamp(
            std::isfinite(rockGrabThrowMaxVelocityHavok) ? rockGrabThrowMaxVelocityHavok : kDefaultGrabThrowMaxVelocityHavok,
            1.0f,
            60.0f);
        rockGrabThrowAngularVelocityScale =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThrowAngularVelocityScale", rockGrabThrowAngularVelocityScale));
        rockGrabThrowAngularVelocityScale = std::clamp(
            std::isfinite(rockGrabThrowAngularVelocityScale) ? rockGrabThrowAngularVelocityScale : kDefaultGrabThrowAngularVelocityScale,
            0.0f,
            2.0f);
        rockGrabThrowMaxAngularVelocityRadiansPerSecond = static_cast<float>(
            ini.GetDoubleValue(SECTION, "fGrabThrowMaxAngularVelocityRadiansPerSecond", rockGrabThrowMaxAngularVelocityRadiansPerSecond));
        rockGrabThrowMaxAngularVelocityRadiansPerSecond = std::clamp(
            std::isfinite(rockGrabThrowMaxAngularVelocityRadiansPerSecond) ? rockGrabThrowMaxAngularVelocityRadiansPerSecond : kDefaultGrabThrowMaxAngularVelocityRadiansPerSecond,
            0.0f,
            60.0f);
        rockGrabReleaseHandCollisionDelaySeconds =
            rock::hand_collision_suppression_math::sanitizeDelaySeconds(
                static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabReleaseHandCollisionDelaySeconds", rockGrabReleaseHandCollisionDelaySeconds)));
        rockGrabNearbyDampingEnabled = ini.GetBoolValue(SECTION, "bGrabNearbyDampingEnabled", rockGrabNearbyDampingEnabled);
        rockGrabNearbyDampingRadius =
            nearby_grab_damping::sanitizeRadius(static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabNearbyDampingRadius", rockGrabNearbyDampingRadius)));
        rockGrabNearbyDampingSeconds =
            nearby_grab_damping::sanitizeDuration(static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabNearbyDampingSeconds", rockGrabNearbyDampingSeconds)));
        rockGrabNearbyLinearDamping =
            nearby_grab_damping::sanitizeHknpDampingCoefficient(static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabNearbyLinearDamping", rockGrabNearbyLinearDamping)));
        rockGrabNearbyAngularDamping =
            nearby_grab_damping::sanitizeHknpDampingCoefficient(static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabNearbyAngularDamping", rockGrabNearbyAngularDamping)));
        rockGrabHeldMassMovementSlowdownEnabled =
            ini.GetBoolValue(SECTION, "bGrabHeldMassMovementSlowdownEnabled", rockGrabHeldMassMovementSlowdownEnabled);
        rockGrabHeldMassMovementMassProportion = readClampedFloat(ini,
            SECTION,
            "fGrabHeldMassMovementMassProportion",
            rockGrabHeldMassMovementMassProportion,
            0.675f,
            0.0f,
            10.0f);
        rockGrabHeldMassMovementMassExponent = readClampedFloat(ini,
            SECTION,
            "fGrabHeldMassMovementMassExponent",
            rockGrabHeldMassMovementMassExponent,
            1.0f,
            0.0f,
            4.0f);
        rockGrabHeldMassMovementMaxReduction = readClampedFloat(ini,
            SECTION,
            "fGrabHeldMassMovementMaxReduction",
            rockGrabHeldMassMovementMaxReduction,
            75.0f,
            0.0f,
            99.0f);
        rockGrabHeldMassMovementFadeOutSeconds = readClampedFloat(ini,
            SECTION,
            "fGrabHeldMassMovementFadeOutSeconds",
            rockGrabHeldMassMovementFadeOutSeconds,
            5.0f,
            0.0f,
            60.0f);
        rockGrabTouchAcquireDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTouchAcquireDistanceGameUnits", rockGrabTouchAcquireDistanceGameUnits));
        if (!std::isfinite(rockGrabTouchAcquireDistanceGameUnits) || rockGrabTouchAcquireDistanceGameUnits <= 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabTouchAcquireDistanceGameUnits={} -- using 4.0", rockGrabTouchAcquireDistanceGameUnits);
            rockGrabTouchAcquireDistanceGameUnits = 4.0f;
        }
        rockGrabNearConvergeDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabNearConvergeDistanceGameUnits", rockGrabNearConvergeDistanceGameUnits));
        if (!std::isfinite(rockGrabNearConvergeDistanceGameUnits) || rockGrabNearConvergeDistanceGameUnits < rockGrabTouchAcquireDistanceGameUnits) {
            ROCK_LOG_WARN(Config,
                "Invalid fGrabNearConvergeDistanceGameUnits={} -- using touch distance {}",
                rockGrabNearConvergeDistanceGameUnits,
                rockGrabTouchAcquireDistanceGameUnits);
            rockGrabNearConvergeDistanceGameUnits = rockGrabTouchAcquireDistanceGameUnits;
        }
        rockGrabPocketDepthGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabPocketDepthGameUnits", rockGrabPocketDepthGameUnits));
        if (!std::isfinite(rockGrabPocketDepthGameUnits) || rockGrabPocketDepthGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabPocketDepthGameUnits={} -- using 7.0", rockGrabPocketDepthGameUnits);
            rockGrabPocketDepthGameUnits = 7.0f;
        }
        rockGrabPocketRadiusGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabPocketRadiusGameUnits", rockGrabPocketRadiusGameUnits));
        if (!std::isfinite(rockGrabPocketRadiusGameUnits) || rockGrabPocketRadiusGameUnits <= 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabPocketRadiusGameUnits={} -- using 9.0", rockGrabPocketRadiusGameUnits);
            rockGrabPocketRadiusGameUnits = 9.0f;
        }
        // Seat depth stop: 0 disables the correction entirely.
        rockGrabSeatDepthMaxGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabSeatDepthMaxGameUnits", rockGrabSeatDepthMaxGameUnits));
        if (!std::isfinite(rockGrabSeatDepthMaxGameUnits) || rockGrabSeatDepthMaxGameUnits < 0.0f || rockGrabSeatDepthMaxGameUnits > 100.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabSeatDepthMaxGameUnits={} -- using 30.0", rockGrabSeatDepthMaxGameUnits);
            rockGrabSeatDepthMaxGameUnits = 30.0f;
        }
        rockGrabSeatDepthFootprintRadiusGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabSeatDepthFootprintRadiusGameUnits", rockGrabSeatDepthFootprintRadiusGameUnits));
        if (!std::isfinite(rockGrabSeatDepthFootprintRadiusGameUnits) ||
            rockGrabSeatDepthFootprintRadiusGameUnits < 1.0f ||
            rockGrabSeatDepthFootprintRadiusGameUnits > 30.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabSeatDepthFootprintRadiusGameUnits={} -- using 10.0", rockGrabSeatDepthFootprintRadiusGameUnits);
            rockGrabSeatDepthFootprintRadiusGameUnits = 10.0f;
        }
        rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabSeatPenetrationBackstopFootprintRadiusGameUnits",
                rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits));
        if (!std::isfinite(rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits) ||
            rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits < 1.0f ||
            rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits > 30.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabSeatPenetrationBackstopFootprintRadiusGameUnits={} -- using 6.0",
                rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits);
            rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits = 6.0f;
        }
        rockGrabSeatDepthSkinGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabSeatDepthSkinGameUnits", rockGrabSeatDepthSkinGameUnits));
        if (!std::isfinite(rockGrabSeatDepthSkinGameUnits) || rockGrabSeatDepthSkinGameUnits < 0.0f || rockGrabSeatDepthSkinGameUnits > 5.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabSeatDepthSkinGameUnits={} -- using 0.5", rockGrabSeatDepthSkinGameUnits);
            rockGrabSeatDepthSkinGameUnits = 0.5f;
        }
        rockGrabGripInsetGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabGripInsetGameUnits", rockGrabGripInsetGameUnits));
        if (!std::isfinite(rockGrabGripInsetGameUnits) || rockGrabGripInsetGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabGripInsetGameUnits={} -- using 2.0", rockGrabGripInsetGameUnits);
            rockGrabGripInsetGameUnits = 2.0f;
        }
        rockGrabGripMaxInsetGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabGripMaxInsetGameUnits", rockGrabGripMaxInsetGameUnits));
        if (!std::isfinite(rockGrabGripMaxInsetGameUnits) || rockGrabGripMaxInsetGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabGripMaxInsetGameUnits={} -- using 6.0", rockGrabGripMaxInsetGameUnits);
            rockGrabGripMaxInsetGameUnits = 6.0f;
        }
        rockGrabConvergeMaxTimeSeconds = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabConvergeMaxTimeSeconds", rockGrabConvergeMaxTimeSeconds));
        if (!std::isfinite(rockGrabConvergeMaxTimeSeconds) || rockGrabConvergeMaxTimeSeconds < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabConvergeMaxTimeSeconds={} -- using 0.35", rockGrabConvergeMaxTimeSeconds);
            rockGrabConvergeMaxTimeSeconds = 0.35f;
        }
        rockGrabConvergeStableFrames = static_cast<int>(ini.GetLongValue(SECTION, "iGrabConvergeStableFrames", rockGrabConvergeStableFrames));
        if (rockGrabConvergeStableFrames < 1) {
            ROCK_LOG_WARN(Config, "Invalid iGrabConvergeStableFrames={} -- using 3", rockGrabConvergeStableFrames);
            rockGrabConvergeStableFrames = 3;
        }
        rockGrabConvergeStableFrames = std::clamp(rockGrabConvergeStableFrames, 1, 12);
        rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond =
            static_cast<float>(ini.GetDoubleValue(
                SECTION,
                "fGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond",
                rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond));
        if (!std::isfinite(rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond) || rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond < 0.0f) {
            ROCK_LOG_WARN(Config,
                "Invalid fGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond={} -- using 40.0",
                rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond);
            rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond = 40.0f;
        }
        rockGrabAcquisitionVisualStartDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(
                SECTION,
                "fGrabAcquisitionVisualStartDistanceGameUnits",
                rockGrabAcquisitionVisualStartDistanceGameUnits));
        if (!std::isfinite(rockGrabAcquisitionVisualStartDistanceGameUnits) || rockGrabAcquisitionVisualStartDistanceGameUnits <= 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabAcquisitionVisualStartDistanceGameUnits={} -- using 28.0", rockGrabAcquisitionVisualStartDistanceGameUnits);
            rockGrabAcquisitionVisualStartDistanceGameUnits = 28.0f;
        }
        rockGrabAcquisitionVisualStartDistanceGameUnits =
            grab_three_phase::computeAcquisitionVisualEnvelopeGameUnits(
                rockGrabTouchAcquireDistanceGameUnits,
                rockGrabNearConvergeDistanceGameUnits,
                rockGrabAcquisitionVisualStartDistanceGameUnits);
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
        rockGrabSurfaceBehindPalmToleranceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabSurfaceBehindPalmToleranceGameUnits", rockGrabSurfaceBehindPalmToleranceGameUnits));
        if (!std::isfinite(rockGrabSurfaceBehindPalmToleranceGameUnits) || rockGrabSurfaceBehindPalmToleranceGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabSurfaceBehindPalmToleranceGameUnits={} -- using 1.5", rockGrabSurfaceBehindPalmToleranceGameUnits);
            rockGrabSurfaceBehindPalmToleranceGameUnits = 1.5f;
        }
        rockGrabOppositionContactMaxAgeFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iGrabOppositionContactMaxAgeFrames", rockGrabOppositionContactMaxAgeFrames));
        rockGrabOppositionContactMaxAgeFrames = std::clamp(rockGrabOppositionContactMaxAgeFrames, 0, 60);
        rockGrabPinchPocketEnabled = ini.GetBoolValue(SECTION, "bGrabPinchPocketEnabled", rockGrabPinchPocketEnabled);
        rockGrabPinchCloseSelectionEnabled = ini.GetBoolValue(SECTION, "bGrabPinchCloseSelectionEnabled", rockGrabPinchCloseSelectionEnabled);
        rockGrabPinchCompactMaxExtentGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchCompactMaxExtentGameUnits",
            rockGrabPinchCompactMaxExtentGameUnits,
            grab_pinch_pocket_policy::kDefaultCompactMaxExtentGameUnits,
            1.0f,
            grab_pinch_pocket_policy::kDefaultCompactMaxExtentGameUnits);
        rockGrabPinchThinRodMaxLengthGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchThinRodMaxLengthGameUnits",
            rockGrabPinchThinRodMaxLengthGameUnits,
            grab_pinch_pocket_policy::kDefaultThinRodMaxLengthGameUnits,
            1.0f,
            120.0f);
        rockGrabPinchThinRodMaxCrossSectionGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchThinRodMaxCrossSectionGameUnits",
            rockGrabPinchThinRodMaxCrossSectionGameUnits,
            grab_pinch_pocket_policy::kDefaultThinRodMaxCrossSectionGameUnits,
            0.1f,
            40.0f);
        rockGrabPinchMaxPocketDistanceGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchMaxPocketDistanceGameUnits",
            rockGrabPinchMaxPocketDistanceGameUnits,
            grab_pinch_pocket_policy::kDefaultMaxPocketDistanceGameUnits,
            0.1f,
            80.0f);
        rockGrabPinchMinFingerGapGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchMinFingerGapGameUnits",
            rockGrabPinchMinFingerGapGameUnits,
            grab_pinch_pocket_policy::kDefaultMinFingerGapGameUnits,
            0.0f,
            40.0f);
        rockGrabPinchMaxFingerGapGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchMaxFingerGapGameUnits",
            rockGrabPinchMaxFingerGapGameUnits,
            grab_pinch_pocket_policy::kDefaultMaxFingerGapGameUnits,
            0.1f,
            80.0f);
        if (rockGrabPinchMaxFingerGapGameUnits < rockGrabPinchMinFingerGapGameUnits) {
            rockGrabPinchMaxFingerGapGameUnits = rockGrabPinchMinFingerGapGameUnits;
        }
        rockGrabPinchThumbIndexMaxOpenValue = readClampedFloat(ini,
            SECTION,
            "fGrabPinchThumbIndexMaxOpenValue",
            rockGrabPinchThumbIndexMaxOpenValue,
            grab_pinch_pocket_policy::kDefaultThumbIndexMaxOpenValue,
            0.0f,
            1.0f);
        rockGrabPinchOtherFingerCurlValue = readClampedFloat(ini,
            SECTION,
            "fGrabPinchOtherFingerCurlValue",
            rockGrabPinchOtherFingerCurlValue,
            grab_pinch_pocket_policy::kDefaultOtherFingerCurlValue,
            0.0f,
            1.0f);
        rockGrabPinchSurfaceInsetGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchSurfaceInsetGameUnits",
            rockGrabPinchSurfaceInsetGameUnits,
            grab_pinch_pocket_policy::kDefaultSurfaceInsetGameUnits,
            0.0f,
            8.0f);
        readVec3("fGrabPinchDetectionDirectionHandspaceX",
            "fGrabPinchDetectionDirectionHandspaceY",
            "fGrabPinchDetectionDirectionHandspaceZ",
            rockGrabPinchDetectionDirectionHandspace);
        rockGrabPinchDetectionAxisBlend = readClampedFloat(ini,
            SECTION,
            "fGrabPinchDetectionAxisBlend",
            rockGrabPinchDetectionAxisBlend,
            grab_pinch_pocket_policy::kDefaultDetectionAxisBlend,
            0.0f,
            1.0f);
        {
            auto pinchDetectionConfig = grab_pinch_pocket_policy::Config{};
            pinchDetectionConfig.detectionDirectionHandspace = rockGrabPinchDetectionDirectionHandspace;
            pinchDetectionConfig.detectionAxisBlend = rockGrabPinchDetectionAxisBlend;
            const auto sanitizedPinchDetectionConfig = grab_pinch_pocket_policy::sanitizeConfig(pinchDetectionConfig);
            rockGrabPinchDetectionDirectionHandspace = sanitizedPinchDetectionConfig.detectionDirectionHandspace;
            rockGrabPinchDetectionAxisBlend = sanitizedPinchDetectionConfig.detectionAxisBlend;
        }
        rockGrabHandLerpEnabled = ini.GetBoolValue(SECTION, "bGrabHandLerpEnabled", rockGrabHandLerpEnabled);
        rockGrabHeldRenderClockAnchor = ini.GetBoolValue(SECTION, "bGrabHeldRenderClockAnchor", rockGrabHeldRenderClockAnchor);
        rockGrabHeldRenderBodyPose = ini.GetBoolValue(SECTION, "bGrabHeldRenderBodyPose", rockGrabHeldRenderBodyPose);
        rockGrabRenderClockProbeOffsetGameUnits = readClampedFloat(ini,
            DEBUG_OVERLAY_SECTION,
            "fGrabRenderClockProbeOffsetGameUnits",
            rockGrabRenderClockProbeOffsetGameUnits,
            0.0f,
            -50.0f,
            50.0f);
        rockGrabSceneWriterProbeOffsetZGameUnits = readClampedFloat(ini,
            DEBUG_OVERLAY_SECTION,
            "fGrabSceneWriterProbeOffsetZGameUnits",
            rockGrabSceneWriterProbeOffsetZGameUnits,
            0.0f,
            -50.0f,
            50.0f);
        rockGrabHeldScenePoseSync = ini.GetBoolValue(SECTION, "bGrabHeldScenePoseSync", rockGrabHeldScenePoseSync);
        rockGrabScenePoseSyncFullAnchorGapGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabScenePoseSyncFullAnchorGapGameUnits",
            rockGrabScenePoseSyncFullAnchorGapGameUnits,
            4.0f,
            0.0f,
            100.0f);
        rockGrabScenePoseSyncSolverGapGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabScenePoseSyncSolverGapGameUnits",
            rockGrabScenePoseSyncSolverGapGameUnits,
            15.0f,
            0.0f,
            200.0f);
        rockGrabHandLerpTimeMin = readClampedFloat(ini,
            SECTION,
            "fGrabHandLerpTimeMin",
            rockGrabHandLerpTimeMin,
            0.10f,
            0.0f,
            1.0f);
        rockGrabHandLerpTimeMax = readClampedFloat(ini,
            SECTION,
            "fGrabHandLerpTimeMax",
            rockGrabHandLerpTimeMax,
            0.20f,
            rockGrabHandLerpTimeMin,
            1.0f);
        rockGrabHandLerpMinDistance = readClampedFloat(ini,
            SECTION,
            "fGrabHandLerpMinDistance",
            rockGrabHandLerpMinDistance,
            7.0f,
            0.0f,
            80.0f);
        rockGrabHandLerpMaxDistance = readClampedFloat(ini,
            SECTION,
            "fGrabHandLerpMaxDistance",
            rockGrabHandLerpMaxDistance,
            14.0f,
            rockGrabHandLerpMinDistance,
            120.0f);
        rockGrabHandReturnEnabled = ini.GetBoolValue(SECTION, "bGrabHandReturnEnabled", rockGrabHandReturnEnabled);
        rockGrabHandReturnTimeMin = readClampedFloat(ini,
            SECTION,
            "fGrabHandReturnTimeMin",
            rockGrabHandReturnTimeMin,
            0.10f,
            0.0f,
            1.0f);
        rockGrabHandReturnTimeMax = readClampedFloat(ini,
            SECTION,
            "fGrabHandReturnTimeMax",
            rockGrabHandReturnTimeMax,
            0.20f,
            rockGrabHandReturnTimeMin,
            1.0f);
        rockGrabHandReturnMinDistance = readClampedFloat(ini,
            SECTION,
            "fGrabHandReturnMinDistance",
            rockGrabHandReturnMinDistance,
            7.0f,
            0.0f,
            80.0f);
        rockGrabHandReturnMaxDistance = readClampedFloat(ini,
            SECTION,
            "fGrabHandReturnMaxDistance",
            rockGrabHandReturnMaxDistance,
            14.0f,
            rockGrabHandReturnMinDistance,
            120.0f);
        rockGrabHandReturnMinAngleDegrees = readClampedFloat(ini,
            SECTION,
            "fGrabHandReturnMinAngleDegrees",
            rockGrabHandReturnMinAngleDegrees,
            5.0f,
            0.0f,
            180.0f);
        rockGrabHandReturnMaxAngleDegrees = readClampedFloat(ini,
            SECTION,
            "fGrabHandReturnMaxAngleDegrees",
            rockGrabHandReturnMaxAngleDegrees,
            90.0f,
            rockGrabHandReturnMinAngleDegrees,
            180.0f);
        rockGrabMeshFingerPoseEnabled = ini.GetBoolValue(SECTION, "bGrabMeshFingerPoseEnabled", rockGrabMeshFingerPoseEnabled);
        rockGrabMeshJointPoseEnabled = ini.GetBoolValue(SECTION, "bGrabMeshJointPoseEnabled", rockGrabMeshJointPoseEnabled);
        rockGrabFingerPoseUpdateInterval = static_cast<int>(ini.GetLongValue(SECTION, "iGrabFingerPoseUpdateInterval", rockGrabFingerPoseUpdateInterval));
        rockGrabFingerPoseUpdateInterval = std::clamp(rockGrabFingerPoseUpdateInterval, 1, 60);
        rockGrabFingerMinValue = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerMinValue", rockGrabFingerMinValue));
        if (!std::isfinite(rockGrabFingerMinValue)) {
            ROCK_LOG_WARN(Config, "Invalid fGrabFingerMinValue={} -- using 0.2", rockGrabFingerMinValue);
            rockGrabFingerMinValue = 0.2f;
        }
        rockGrabFingerMinValue = std::clamp(rockGrabFingerMinValue, 0.0f, 1.0f);
        rockGrabFingerPoseSmoothingSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerPoseSmoothingSpeed", rockGrabFingerPoseSmoothingSpeed));
        if (!std::isfinite(rockGrabFingerPoseSmoothingSpeed) || rockGrabFingerPoseSmoothingSpeed < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabFingerPoseSmoothingSpeed={} -- using 14.0", rockGrabFingerPoseSmoothingSpeed);
            rockGrabFingerPoseSmoothingSpeed = 14.0f;
        }
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
        rockGrabFingerRejectBacksideHits = ini.GetBoolValue(SECTION, "bGrabFingerRejectBacksideHits", rockGrabFingerRejectBacksideHits);
        rockGrabFingerSurfacePlaneToleranceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerSurfacePlaneToleranceGameUnits", rockGrabFingerSurfacePlaneToleranceGameUnits));
        if (!std::isfinite(rockGrabFingerSurfacePlaneToleranceGameUnits) || rockGrabFingerSurfacePlaneToleranceGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabFingerSurfacePlaneToleranceGameUnits={} -- using 1.5", rockGrabFingerSurfacePlaneToleranceGameUnits);
            rockGrabFingerSurfacePlaneToleranceGameUnits = 1.5f;
        }
        rockGrabFingerSweepContactRadiusGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerSweepContactRadiusGameUnits", rockGrabFingerSweepContactRadiusGameUnits));
        if (!std::isfinite(rockGrabFingerSweepContactRadiusGameUnits) || rockGrabFingerSweepContactRadiusGameUnits <= 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabFingerSweepContactRadiusGameUnits={} -- using 1.0", rockGrabFingerSweepContactRadiusGameUnits);
            rockGrabFingerSweepContactRadiusGameUnits = 1.0f;
        }
        rockGrabFingerSweepContactRadiusGameUnits = std::clamp(rockGrabFingerSweepContactRadiusGameUnits, 0.05f, 4.0f);
        rockGrabFingerSweepMaxOpenValue =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerSweepMaxOpenValue", rockGrabFingerSweepMaxOpenValue));
        if (!std::isfinite(rockGrabFingerSweepMaxOpenValue) || rockGrabFingerSweepMaxOpenValue < 1.0f || rockGrabFingerSweepMaxOpenValue > 2.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabFingerSweepMaxOpenValue={} -- using 2.0 (valid range 1.0-2.0)", rockGrabFingerSweepMaxOpenValue);
            rockGrabFingerSweepMaxOpenValue = 2.0f;
        }
        rockGrabThumbSweepMaxOpenValue =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThumbSweepMaxOpenValue", rockGrabThumbSweepMaxOpenValue));
        if (!std::isfinite(rockGrabThumbSweepMaxOpenValue) || rockGrabThumbSweepMaxOpenValue < 1.0f || rockGrabThumbSweepMaxOpenValue > 2.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabThumbSweepMaxOpenValue={} -- using 2.0 (valid range 1.0-2.0)", rockGrabThumbSweepMaxOpenValue);
            rockGrabThumbSweepMaxOpenValue = 2.0f;
        }
        rockGrabFingerPoseResolveWindowSeconds =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerPoseResolveWindowSeconds", rockGrabFingerPoseResolveWindowSeconds));
        if (!std::isfinite(rockGrabFingerPoseResolveWindowSeconds) || rockGrabFingerPoseResolveWindowSeconds < 0.25f || rockGrabFingerPoseResolveWindowSeconds > 10.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabFingerPoseResolveWindowSeconds={} -- using 2.0 (valid range 0.25-10)", rockGrabFingerPoseResolveWindowSeconds);
            rockGrabFingerPoseResolveWindowSeconds = 2.0f;
        }
        rockGrabThumbOppositionStrength = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThumbOppositionStrength", rockGrabThumbOppositionStrength));
        rockGrabThumbOppositionStrength = std::clamp(std::isfinite(rockGrabThumbOppositionStrength) ? rockGrabThumbOppositionStrength : 1.0f, 0.0f, 1.0f);
        rockGrabThumbAlternateCurveStrength =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThumbAlternateCurveStrength", rockGrabThumbAlternateCurveStrength));
        rockGrabThumbAlternateCurveStrength = std::clamp(std::isfinite(rockGrabThumbAlternateCurveStrength) ? rockGrabThumbAlternateCurveStrength : 0.65f, 0.0f, 1.0f);
        rockGrabThumbSurfaceSafetyEnabled = ini.GetBoolValue(SECTION, "bGrabThumbSurfaceSafetyEnabled", rockGrabThumbSurfaceSafetyEnabled);
        rockGrabThumbSurfaceSafetyMarginGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThumbSurfaceSafetyMarginGameUnits", rockGrabThumbSurfaceSafetyMarginGameUnits));
        rockGrabThumbSurfaceSafetyMarginGameUnits = std::clamp(
            std::isfinite(rockGrabThumbSurfaceSafetyMarginGameUnits) ? rockGrabThumbSurfaceSafetyMarginGameUnits : kDefaultGrabThumbSurfaceSafetyMarginGameUnits,
            0.0f,
            5.0f);
        rockGrabLateralWeight = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLateralWeight", rockGrabLateralWeight));
        rockGrabDirectionalWeight = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabDirectionalWeight", rockGrabDirectionalWeight));
        rockGrabMaxTriangleDistance = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxTriangleDistance", rockGrabMaxTriangleDistance));
        rockGrabMeshContactOnly = ini.GetBoolValue(SECTION, "bGrabMeshContactOnly", rockGrabMeshContactOnly);
        rockGrabRequireMeshContact = ini.GetBoolValue(SECTION, "bGrabRequireMeshContact", rockGrabRequireMeshContact);
        rockGrabContactPatchEnabled = ini.GetBoolValue(SECTION, "bGrabContactPatchEnabled", rockGrabContactPatchEnabled);
        rockGrabContactPatchProbeCount = static_cast<int>(ini.GetLongValue(SECTION, "iGrabContactPatchProbeCount", rockGrabContactPatchProbeCount));
        rockGrabContactPatchProbeCount = std::clamp(rockGrabContactPatchProbeCount, 1, 9);
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
        rockGrabAlignmentMaxSelectionToMeshDistance =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAlignmentMaxSelectionToMeshDistance", rockGrabAlignmentMaxSelectionToMeshDistance));
        if (!std::isfinite(rockGrabAlignmentMaxSelectionToMeshDistance)) {
            rockGrabAlignmentMaxSelectionToMeshDistance = 8.0f;
        }
        rockGrabNodeAnchorsEnabled = ini.GetBoolValue(SECTION, "bGrabNodeAnchorsEnabled", rockGrabNodeAnchorsEnabled);
        rockGrabNodeRejectOppositeHandAnchor = ini.GetBoolValue(SECTION, "bGrabNodeRejectOppositeHandAnchor", rockGrabNodeRejectOppositeHandAnchor);
        rockPrintGrabNodeInfo = ini.GetBoolValue(SECTION, "bPrintGrabNodeInfo", rockPrintGrabNodeInfo);
        rockGrabNodeNameRight =
            grab_node_name_policy::sanitizeConfiguredGrabNodeName(ini.GetValue(SECTION, "sGrabNodeNameRight", rockGrabNodeNameRight.c_str()), false);
        rockGrabNodeNameLeft = grab_node_name_policy::sanitizeConfiguredGrabNodeName(ini.GetValue(SECTION, "sGrabNodeNameLeft", rockGrabNodeNameLeft.c_str()), true);
        rockGrabNodeNameBlacklist = ini.GetValue(SECTION, "sGrabNodeNameBlacklist", rockGrabNodeNameBlacklist.c_str());
        rockSelectedCloseFingerCurlEnabled = ini.GetBoolValue(SECTION, "bSelectedCloseFingerCurlEnabled", rockSelectedCloseFingerCurlEnabled);
        rockSelectedCloseFingerAnimMaxHandSpeed =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSelectedCloseFingerAnimMaxHandSpeed", rockSelectedCloseFingerAnimMaxHandSpeed));
        if (!std::isfinite(rockSelectedCloseFingerAnimMaxHandSpeed) || rockSelectedCloseFingerAnimMaxHandSpeed < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fSelectedCloseFingerAnimMaxHandSpeed={} -- using 0.9", rockSelectedCloseFingerAnimMaxHandSpeed);
            rockSelectedCloseFingerAnimMaxHandSpeed = 0.9f;
        }
        rockSelectedCloseFingerAnimValue = static_cast<float>(ini.GetDoubleValue(SECTION, "fSelectedCloseFingerAnimValue", rockSelectedCloseFingerAnimValue));
        if (!std::isfinite(rockSelectedCloseFingerAnimValue)) {
            ROCK_LOG_WARN(Config, "Invalid fSelectedCloseFingerAnimValue={} -- using 0.9", rockSelectedCloseFingerAnimValue);
            rockSelectedCloseFingerAnimValue = 0.9f;
        }
        rockSelectedCloseFingerAnimValue = std::clamp(rockSelectedCloseFingerAnimValue, 0.0f, 1.0f);
        rockPulledAngularDamping = static_cast<float>(ini.GetDoubleValue(SECTION, "fPulledAngularDamping", rockPulledAngularDamping));
        rockPullToObjectCenterEnabled = ini.GetBoolValue(SECTION, "bPullToObjectCenterEnabled", rockPullToObjectCenterEnabled);
        rockPullLongAxisPresentationEnabled = ini.GetBoolValue(SECTION, "bPullLongAxisPresentationEnabled", rockPullLongAxisPresentationEnabled);
        rockForceGrabSeatAlignmentEnabled = ini.GetBoolValue(SECTION, "bForceGrabSeatAlignmentEnabled", rockForceGrabSeatAlignmentEnabled);
        rockGrabSeatRollAlignmentEnabled = ini.GetBoolValue(SECTION, "bGrabSeatRollAlignmentEnabled", rockGrabSeatRollAlignmentEnabled);
        rockPullPresentationMinElongationRatio =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fPullPresentationMinElongationRatio", rockPullPresentationMinElongationRatio));
        if (!std::isfinite(rockPullPresentationMinElongationRatio) || rockPullPresentationMinElongationRatio < 1.0f) {
            ROCK_LOG_WARN(Config, "Invalid fPullPresentationMinElongationRatio={} -- using 2.0", rockPullPresentationMinElongationRatio);
            rockPullPresentationMinElongationRatio = 2.0f;
        }
        rockGrabSeatRollMinSecondElongationRatio =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabSeatRollMinSecondElongationRatio", rockGrabSeatRollMinSecondElongationRatio));
        if (!std::isfinite(rockGrabSeatRollMinSecondElongationRatio) || rockGrabSeatRollMinSecondElongationRatio < 1.0f ||
            rockGrabSeatRollMinSecondElongationRatio > 10.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabSeatRollMinSecondElongationRatio={} -- using 1.25", rockGrabSeatRollMinSecondElongationRatio);
            rockGrabSeatRollMinSecondElongationRatio = 1.25f;
        }
        rockPullPresentationAngularGainPerSecond =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fPullPresentationAngularGainPerSecond", rockPullPresentationAngularGainPerSecond));
        if (!std::isfinite(rockPullPresentationAngularGainPerSecond) || rockPullPresentationAngularGainPerSecond < 0.0f ||
            rockPullPresentationAngularGainPerSecond > 30.0f) {
            ROCK_LOG_WARN(Config, "Invalid fPullPresentationAngularGainPerSecond={} -- using 6.0", rockPullPresentationAngularGainPerSecond);
            rockPullPresentationAngularGainPerSecond = 6.0f;
        }
        rockPullPresentationMaxAngularSpeedRadiansPerSecond = static_cast<float>(
            ini.GetDoubleValue(SECTION, "fPullPresentationMaxAngularSpeedRadiansPerSecond", rockPullPresentationMaxAngularSpeedRadiansPerSecond));
        if (!std::isfinite(rockPullPresentationMaxAngularSpeedRadiansPerSecond) || rockPullPresentationMaxAngularSpeedRadiansPerSecond < 0.0f ||
            rockPullPresentationMaxAngularSpeedRadiansPerSecond > 40.0f) {
            ROCK_LOG_WARN(Config, "Invalid fPullPresentationMaxAngularSpeedRadiansPerSecond={} -- using 8.0", rockPullPresentationMaxAngularSpeedRadiansPerSecond);
            rockPullPresentationMaxAngularSpeedRadiansPerSecond = 8.0f;
        }
        rockPullPresentationGripAxisTiltDegrees = static_cast<float>(
            ini.GetDoubleValue(SECTION, "fPullPresentationGripAxisTiltDegrees", rockPullPresentationGripAxisTiltDegrees));
        if (!std::isfinite(rockPullPresentationGripAxisTiltDegrees) || rockPullPresentationGripAxisTiltDegrees < -45.0f ||
            rockPullPresentationGripAxisTiltDegrees > 45.0f) {
            ROCK_LOG_WARN(Config, "Invalid fPullPresentationGripAxisTiltDegrees={} -- using 10.0", rockPullPresentationGripAxisTiltDegrees);
            rockPullPresentationGripAxisTiltDegrees = 10.0f;
        }

        readOptionalVec3("fRightGrabLegacyPalmPivotAHandspaceX", "fRightGrabLegacyPalmPivotAHandspaceY", "fRightGrabLegacyPalmPivotAHandspaceZ", rockRightGrabLegacyPalmPivotAHandspace);
        readOptionalVec3("fLeftGrabLegacyPalmPivotAHandspaceX", "fLeftGrabLegacyPalmPivotAHandspaceY", "fLeftGrabLegacyPalmPivotAHandspaceZ", rockLeftGrabLegacyPalmPivotAHandspace);

        auto readClampedFloat = [&](const char* key, float& value, float fallback, float minValue, float maxValue) {
            value = static_cast<float>(ini.GetDoubleValue(SECTION, key, value));
            if (!std::isfinite(value)) {
                ROCK_LOG_WARN(Config, "Invalid {}={} -- using {}", key, value, fallback);
                value = fallback;
            }
            value = std::clamp(value, minValue, maxValue);
        };

        rockShoulderStashEnabled = ini.GetBoolValue(SECTION, "bShoulderStashEnabled", rockShoulderStashEnabled);
        rockEquippedWeaponShoulderStashEnabled = ini.GetBoolValue(
            SECTION,
            "bEquippedWeaponShoulderStashEnabled",
            rockEquippedWeaponShoulderStashEnabled);
        rockShoulderStashUseBodyZoneColliders =
            ini.GetBoolValue(SECTION, "bShoulderStashUseBodyZoneColliders", rockShoulderStashUseBodyZoneColliders);
        rockShoulderStashUseHmdBackVolume =
            ini.GetBoolValue(SECTION, "bShoulderStashUseHmdBackVolume", rockShoulderStashUseHmdBackVolume);
        readClampedFloat("fShoulderStashEnterPaddingGameUnits", rockShoulderStashEnterPaddingGameUnits, 5.0f, 0.0f, 40.0f);
        readClampedFloat("fShoulderStashExitPaddingGameUnits", rockShoulderStashExitPaddingGameUnits, 8.0f, 0.0f, 60.0f);
        readClampedFloat("fShoulderStashMinDwellSeconds", rockShoulderStashMinDwellSeconds, 0.08f, 0.0f, 1.0f);
        readClampedFloat("fShoulderStashMaxSpeedGameUnitsPerSecond", rockShoulderStashMaxSpeedGameUnitsPerSecond, 140.0f, 0.0f, 1000.0f);
        rockShoulderStashRecentContactFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iShoulderStashRecentContactFrames", rockShoulderStashRecentContactFrames));
        rockShoulderStashRecentContactFrames = std::clamp(rockShoulderStashRecentContactFrames, 0, 60);
        rockShoulderStashSustainedContactMissFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iShoulderStashSustainedContactMissFrames", rockShoulderStashSustainedContactMissFrames));
        rockShoulderStashSustainedContactMissFrames = std::clamp(rockShoulderStashSustainedContactMissFrames, 0, 120);
        readOptionalVec3("fShoulderStashHmdBackRightOffsetXGameUnits",
            "fShoulderStashHmdBackRightOffsetYGameUnits",
            "fShoulderStashHmdBackRightOffsetZGameUnits",
            rockShoulderStashHmdBackRightOffsetGameUnits);
        readOptionalVec3("fShoulderStashHmdBackLeftOffsetXGameUnits",
            "fShoulderStashHmdBackLeftOffsetYGameUnits",
            "fShoulderStashHmdBackLeftOffsetZGameUnits",
            rockShoulderStashHmdBackLeftOffsetGameUnits);
        readClampedFloat("fShoulderStashHmdBackRadiusGameUnits", rockShoulderStashHmdBackRadiusGameUnits, 11.0f, 1.0f, 80.0f);
        readClampedFloat(
            "fShoulderStashHmdBackEnterPaddingGameUnits", rockShoulderStashHmdBackEnterPaddingGameUnits, 0.0f, 0.0f, 40.0f);
        readClampedFloat(
            "fShoulderStashHmdBackExitPaddingGameUnits", rockShoulderStashHmdBackExitPaddingGameUnits, 2.0f, 0.0f, 60.0f);
        readClampedFloat(
            "fShoulderStashHmdBackMinBehindGameUnits", rockShoulderStashHmdBackMinBehindGameUnits, 4.0f, 0.0f, 40.0f);
        rockShoulderStashShowCollectedNotifications =
            ini.GetBoolValue(SECTION, "bShoulderStashShowCollectedNotifications", rockShoulderStashShowCollectedNotifications);
        rockMouthConsumeEnabled = ini.GetBoolValue(SECTION, "bMouthConsumeEnabled", rockMouthConsumeEnabled);
        rockMouthConsumeAllowPoison = ini.GetBoolValue(SECTION, "bMouthConsumeAllowPoison", rockMouthConsumeAllowPoison);
        readOptionalVec3("fMouthConsumeHmdOffsetXGameUnits",
            "fMouthConsumeHmdOffsetYGameUnits",
            "fMouthConsumeHmdOffsetZGameUnits",
            rockMouthConsumeHmdOffsetGameUnits);
        sanitizeMouthConsumeOffset();
        readClampedFloat("fMouthConsumeRadiusGameUnits", rockMouthConsumeRadiusGameUnits, 5.5f, 1.0f, 80.0f);
        readClampedFloat("fMouthConsumeEnterPaddingGameUnits", rockMouthConsumeEnterPaddingGameUnits, 0.0f, 0.0f, 40.0f);
        readClampedFloat("fMouthConsumeExitPaddingGameUnits", rockMouthConsumeExitPaddingGameUnits, 1.0f, 0.0f, 60.0f);
        readClampedFloat("fMouthConsumeMinDwellSeconds", rockMouthConsumeMinDwellSeconds, 0.08f, 0.0f, 1.0f);
        readClampedFloat("fMouthConsumeMaxSpeedGameUnitsPerSecond", rockMouthConsumeMaxSpeedGameUnitsPerSecond, 120.0f, 0.0f, 1000.0f);

        rockGrabHapticsEnabled = ini.GetBoolValue(SECTION, "bGrabHapticsEnabled", rockGrabHapticsEnabled);
        readClampedFloat("fGrabHapticDurationSeconds", rockGrabHapticDurationSeconds, 0.055f, 0.0f, 0.2f);
        readClampedFloat("fGrabHapticBaseIntensity", rockGrabHapticBaseIntensity, 0.12f, 0.0f, 1.0f);
        readClampedFloat("fGrabHapticMaxIntensity", rockGrabHapticMaxIntensity, 0.80f, rockGrabHapticBaseIntensity, 1.0f);
        readClampedFloat("fGrabHapticMassScale", rockGrabHapticMassScale, 0.06f, 0.0f, 1.0f);
        readClampedFloat("fGrabHapticMassExponent", rockGrabHapticMassExponent, 0.60f, 0.0f, 2.0f);
        readClampedFloat("fPullStartHapticIntensity", rockPullStartHapticIntensity, 0.18f, 0.0f, 1.0f);
        readClampedFloat("fPullCatchHapticIntensity", rockPullCatchHapticIntensity, 0.22f, 0.0f, 1.0f);
        readClampedFloat("fSelectionLockHapticIntensity", rockSelectionLockHapticIntensity, 0.15f, 0.0f, 1.0f);
        readClampedFloat("fSelectionLockReleaseHapticIntensity", rockSelectionLockReleaseHapticIntensity, 0.10f, 0.0f, 1.0f);
        readClampedFloat("fSelectionLockReleaseHapticDurationSeconds", rockSelectionLockReleaseHapticDurationSeconds, 0.02f, 0.0f, 0.2f);
        rockSurfaceGrabHapticsEnabled =
            ini.GetBoolValue(SECTION, "bSurfaceGrabHapticsEnabled", rockSurfaceGrabHapticsEnabled);
        readClampedFloat("fSurfaceGrabHapticDurationSeconds", rockSurfaceGrabHapticDurationSeconds, 0.075f, 0.0f, 0.2f);
        readClampedFloat("fSurfaceGrabHapticIntensity", rockSurfaceGrabHapticIntensity, 0.85f, 0.0f, 1.0f);
        rockHeldImpactHapticsEnabled = ini.GetBoolValue(SECTION, "bHeldImpactHapticsEnabled", rockHeldImpactHapticsEnabled);
        readClampedFloat("fHeldImpactHapticDurationSeconds", rockHeldImpactHapticDurationSeconds, 0.035f, 0.0f, 0.2f);
        readClampedFloat("fHeldImpactHapticBaseIntensity", rockHeldImpactHapticBaseIntensity, 0.12f, 0.0f, 1.0f);
        readClampedFloat("fHeldImpactHapticMaxIntensity", rockHeldImpactHapticMaxIntensity, 0.85f, rockHeldImpactHapticBaseIntensity, 1.0f);
        readClampedFloat("fHeldImpactHapticSpeedScale", rockHeldImpactHapticSpeedScale, 0.006f, 0.0f, 1.0f);
        readClampedFloat("fHeldImpactHapticMassScale", rockHeldImpactHapticMassScale, 0.035f, 0.0f, 1.0f);
        readClampedFloat("fHeldImpactHapticMassExponent", rockHeldImpactHapticMassExponent, 0.55f, 0.0f, 2.0f);
        readClampedFloat("fHeldImpactHapticMinSpeedGameUnits", rockHeldImpactHapticMinSpeedGameUnits, 8.0f, 0.0f, 1000.0f);
        readClampedFloat("fHeldImpactHapticCooldownSeconds", rockHeldImpactHapticCooldownSeconds, 0.12f, 0.0f, 1.0f);
        readClampedFloat("fHeldImpactHapticDampedMultiplier", rockHeldImpactHapticDampedMultiplier, 0.55f, 0.0f, 1.0f);
        rockShoulderStashHapticsEnabled =
            ini.GetBoolValue(SECTION, "bShoulderStashHapticsEnabled", rockShoulderStashHapticsEnabled);
        readClampedFloat(
            "fShoulderStashCandidateHapticDurationSeconds", rockShoulderStashCandidateHapticDurationSeconds, 0.075f, 0.0f, 0.2f);
        readClampedFloat(
            "fShoulderStashCandidateHapticBaseIntensity", rockShoulderStashCandidateHapticBaseIntensity, 0.20f, 0.0f, 1.0f);
        readClampedFloat("fShoulderStashCandidateHapticIntensity",
            rockShoulderStashCandidateHapticIntensity,
            0.42f,
            rockShoulderStashCandidateHapticBaseIntensity,
            1.0f);
        readClampedFloat("fShoulderStashCandidateHapticIntervalSeconds", rockShoulderStashCandidateHapticIntervalSeconds, 0.075f, 0.0f, 2.0f);
        readClampedFloat(
            "fShoulderStashCommitHapticDurationSeconds", rockShoulderStashCommitHapticDurationSeconds, 0.12f, 0.0f, 0.2f);
        readClampedFloat("fShoulderStashCommitHapticIntensity", rockShoulderStashCommitHapticIntensity, 0.85f, 0.0f, 1.0f);
        rockMouthConsumeHapticsEnabled = ini.GetBoolValue(SECTION, "bMouthConsumeHapticsEnabled", rockMouthConsumeHapticsEnabled);
        readClampedFloat(
            "fMouthConsumeCandidateHapticDurationSeconds", rockMouthConsumeCandidateHapticDurationSeconds, 0.050f, 0.0f, 0.2f);
        readClampedFloat(
            "fMouthConsumeCandidateHapticBaseIntensity", rockMouthConsumeCandidateHapticBaseIntensity, 0.22f, 0.0f, 1.0f);
        readClampedFloat("fMouthConsumeCandidateHapticIntensity",
            rockMouthConsumeCandidateHapticIntensity,
            0.45f,
            rockMouthConsumeCandidateHapticBaseIntensity,
            1.0f);
        readClampedFloat("fMouthConsumeCandidateHapticIntervalSeconds", rockMouthConsumeCandidateHapticIntervalSeconds, 0.075f, 0.0f, 2.0f);
        readClampedFloat("fMouthConsumeCommitHapticDurationSeconds", rockMouthConsumeCommitHapticDurationSeconds, 0.12f, 0.0f, 0.2f);
        readClampedFloat("fMouthConsumeCommitHapticIntensity", rockMouthConsumeCommitHapticIntensity, 0.85f, 0.0f, 1.0f);

        resolveEffectiveDebugSettings();
        ini.warnUnknownKeys();
    }

    void RockConfig::resolveEffectiveDebugSettings()
    {
        using debug_config_policy::childEnabled;
        using debug_config_policy::subsystemEnabled;

        rockDebugOverlayEnabled = subsystemEnabled(rockDebugEnabled, rockDebugOverlayEnabled);
        rockDebugLoggingEnabled = subsystemEnabled(rockDebugEnabled, rockDebugLoggingEnabled);
        rockDebugControllerEnabled = subsystemEnabled(rockDebugEnabled, rockDebugControllerEnabled);
        rockDebugMonitorEnabled = subsystemEnabled(rockDebugEnabled, rockDebugMonitorEnabled);
        rockPerformanceProfilerEnabled = subsystemEnabled(rockDebugEnabled, rockPerformanceProfilerEnabled);
        rockPerformanceProfilerOverlayText =
            rockPerformanceProfilerEnabled && rockDebugOverlayEnabled && rockPerformanceProfilerOverlayText;
        rockDebugProviderOverlayEnabled = rockDebugOverlayEnabled && rockDebugProviderOverlayEnabled;

        rockDebugShowColliders = childEnabled(rockDebugOverlayEnabled, rockDebugShowColliders);
        const bool colliderOverlayEnabled = rockDebugShowColliders;
        rockDebugShowTargetColliders = colliderOverlayEnabled && rockDebugShowTargetColliders;
        rockDebugDrawHandColliders = colliderOverlayEnabled && rockDebugDrawHandColliders;
        rockDebugDrawHandBoneColliders = colliderOverlayEnabled && rockDebugDrawHandBoneColliders;
        rockDebugDrawDynamicHandColliders = colliderOverlayEnabled && rockDebugDrawDynamicHandColliders;
        rockDebugDrawWeaponColliders = colliderOverlayEnabled && rockDebugDrawWeaponColliders;
        rockDebugDrawDynamicWeaponColliders = colliderOverlayEnabled && rockDebugDrawDynamicWeaponColliders;
        rockDebugDrawGrabAuthorityProxy = colliderOverlayEnabled && rockDebugDrawGrabAuthorityProxy;
        rockDebugDrawGrabPivotSourceCollider = colliderOverlayEnabled && rockDebugDrawGrabPivotSourceCollider;
        rockDebugProviderColliderFocusEnabled = colliderOverlayEnabled && rockDebugProviderColliderFocusEnabled;

        rockDebugShowHandAxes = rockDebugOverlayEnabled && rockDebugShowHandAxes;
        rockDebugShowGrabPivots = rockDebugOverlayEnabled && rockDebugShowGrabPivots;
        rockDebugShowGrabPocketNormal = rockDebugOverlayEnabled && rockDebugShowGrabPocketNormal;
        rockDebugDrawGrabContactPatch = rockDebugOverlayEnabled && rockDebugDrawGrabContactPatch;
        rockDebugDrawGrabForceTorque = rockDebugOverlayEnabled && rockDebugDrawGrabForceTorque;
        rockDebugDrawGrabForceTorqueText = rockDebugDrawGrabForceTorque && rockDebugDrawGrabForceTorqueText;
        rockDebugDrawGrabPivotSourceEvidence = rockDebugDrawGrabForceTorque && rockDebugDrawGrabPivotSourceEvidence;
        rockDebugDrawGrabPivotSourceCollider = rockDebugDrawGrabForceTorque && rockDebugDrawGrabPivotSourceCollider;
        rockDebugDrawGrabSupportFrame = rockDebugOverlayEnabled && rockDebugDrawGrabSupportFrame;
        rockDebugDrawGrabPockets = rockDebugOverlayEnabled && rockDebugDrawGrabPockets;
        rockDebugShowGrabFingerProbes = rockDebugOverlayEnabled && rockDebugShowGrabFingerProbes;
        rockDebugShowGrabFingerSweptArc = rockDebugOverlayEnabled && rockDebugShowGrabFingerSweptArc;
        rockDebugShowGrabFingerSweptArcText = rockDebugShowGrabFingerSweptArc && rockDebugShowGrabFingerSweptArcText;
        rockDebugShowGrabFingerSweptArcLiveSkeleton =
            rockDebugShowGrabFingerSweptArc && rockDebugShowGrabFingerSweptArcLiveSkeleton;
        rockDebugShowPalmVectors = rockDebugOverlayEnabled && rockDebugShowPalmVectors;
        rockDebugDrawHandBoneContacts = rockDebugOverlayEnabled && rockDebugDrawHandBoneContacts;
        rockDebugDrawNativeScopeActivation = rockDebugOverlayEnabled && rockDebugDrawNativeScopeActivation;
        rockDebugDrawAuthoredGripActivationZones = rockDebugOverlayEnabled && rockDebugDrawAuthoredGripActivationZones;
        rockDebugDrawGunstockAlignment = rockDebugOverlayEnabled && rockDebugDrawGunstockAlignment;
        rockDebugVideoSyncMarker = rockDebugOverlayEnabled && rockDebugVideoSyncMarker;
        rockDebugGrabTransformTelemetry = rockDebugOverlayEnabled && rockDebugGrabTransformTelemetry;
        rockDebugGrabTransformTelemetryText = rockDebugGrabTransformTelemetry && rockDebugGrabTransformTelemetryText;
        rockDebugGrabTransformTelemetryAxes = rockDebugGrabTransformTelemetry && rockDebugGrabTransformTelemetryAxes;
        rockDebugShowGrabNotifications = rockDebugEnabled && rockDebugShowGrabNotifications;
        rockDebugShowWeaponNotifications = rockDebugEnabled && rockDebugShowWeaponNotifications;
        rockDebugWorldObjectOriginDiagnostics = rockDebugOverlayEnabled && rockDebugWorldObjectOriginDiagnostics;
        rockDebugCustomCalibrationOffset = rockDebugOverlayEnabled && rockDebugCustomCalibrationOffset;
        if (!rockDebugOverlayEnabled) {
            rockGrabRenderClockProbeOffsetGameUnits = 0.0f;
            rockGrabSceneWriterProbeOffsetZGameUnits = 0.0f;
        }
        rockDebugShowRootFlattenedFingerSkeletonMarkers =
            rockDebugOverlayEnabled && rockDebugShowRootFlattenedFingerSkeletonMarkers;
        rockDebugShowSkeletonBoneVisualizer = rockDebugOverlayEnabled && rockDebugShowSkeletonBoneVisualizer;
        rockDebugDrawSkeletonBoneAxes = rockDebugShowSkeletonBoneVisualizer && rockDebugDrawSkeletonBoneAxes;

        rockNativeMeleeDebugLogging = rockDebugLoggingEnabled && rockNativeMeleeDebugLogging;
        rockDebugVerboseLogging = rockDebugLoggingEnabled && rockDebugVerboseLogging;
        rockDebugGrabFrameLogging = rockDebugLoggingEnabled && rockDebugGrabFrameLogging;
        rockDebugGrabTimelineTrace = rockDebugLoggingEnabled && rockDebugGrabTimelineTrace;
        rockDebugGrabAfterSolveAnomalySampling = rockDebugLoggingEnabled && rockDebugGrabAfterSolveAnomalySampling;
        rockDebugColliderClockLogging = rockDebugLoggingEnabled && rockDebugColliderClockLogging;
        rockDebugDynamicWeaponLogging = rockDebugLoggingEnabled && rockDebugDynamicWeaponLogging;
        rockDebugDumpWeaponAnimNodes = rockDebugLoggingEnabled && rockDebugDumpWeaponAnimNodes;
        rockDebugWeaponOmodDumpEnabled = rockDebugLoggingEnabled && rockDebugWeaponOmodDumpEnabled;
        rockDebugWeaponOmodCoverageAudit = rockDebugLoggingEnabled && rockDebugWeaponOmodCoverageAudit;
        rockDebugHandTransformParity = rockDebugLoggingEnabled && rockDebugHandTransformParity;
        rockDebugLogSkeletonBones = rockDebugLoggingEnabled && rockDebugLogSkeletonBones;
        rockDebugLogSkeletonBoneTruncation = rockDebugLoggingEnabled && rockDebugLogSkeletonBoneTruncation;
    }

    void RockConfig::load()
    {
        _iniFilePath = resolveIniPath();
        ROCK_LOG_INFO(Config, "Loading ROCK config from: {}", _iniFilePath);

        rock::resources::createDirectoryTreeForFile(_iniFilePath);

        rock::resources::createFileFromResourceIfMissing(_iniFilePath, "ROCK", IDR_ROCK_INI, true);

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

    bool RockConfig::saveRuntimeIni(CSimpleIniA& ini, const char* reason)
    {
        const std::string path = _iniFilePath.empty() ? resolveIniPath() : _iniFilePath;
        _selfIniWriteInProgress.store(true, std::memory_order_release);
        suppressNextFileWatchReload();

        const SI_Error saveRc = ini.SaveFile(path.c_str(), false);
        std::error_code ec;
        const auto writeTime = std::filesystem::last_write_time(path, ec);
        if (!ec) {
            _lastSelfIniWriteTime.store(writeTime, std::memory_order_release);
            _lastIniFileWriteTime.store(writeTime, std::memory_order_release);
        }

        _selfIniWriteInProgress.store(false, std::memory_order_release);
        _ignoreNextIniFileChange.store(false, std::memory_order_release);

        if (saveRc < 0) {
            ROCK_LOG_WARN(Config, "Failed to persist ROCK.ini runtime change '{}' (code {})", reason ? reason : "unknown", static_cast<int>(saveRc));
            return false;
        }

        ROCK_LOG_DEBUG(Config, "Persisted ROCK.ini runtime change '{}'", reason ? reason : "unknown");
        return true;
    }

    bool RockConfig::persistPhysicsBool(const char* key, bool value)
    {
        return persistBool(SECTION, key, value);
    }

    bool RockConfig::persistDebugOverlayBool(const char* key, bool value)
    {
        return persistBool(DEBUG_OVERLAY_SECTION, key, value);
    }

    bool RockConfig::persistBool(const char* section, const char* key, bool value)
    {
        if (!section || !section[0] || !key || !key[0]) {
            return false;
        }

        const std::string path = _iniFilePath.empty() ? resolveIniPath() : _iniFilePath;
        CSimpleIniA ini;
        ini.SetUnicode(false);
        const SI_Error loadRc = ini.LoadFile(path.c_str());
        if (loadRc < 0) {
            ROCK_LOG_WARN(Config, "Cannot persist ROCK.ini bool '{}': load failed with code {}", key, static_cast<int>(loadRc));
            return false;
        }

        const SI_Error setRc = ini.SetBoolValue(section, key, value, nullptr, true);
        if (setRc < 0) {
            ROCK_LOG_WARN(Config, "Cannot persist ROCK.ini bool '{}': set failed with code {}", key, static_cast<int>(setRc));
            return false;
        }

        return saveRuntimeIni(ini, key);
    }

    bool RockConfig::persistGrabLegacyPalmPivotAHandspace(bool isLeft, const RE::NiPoint3& value)
    {
        const std::string path = _iniFilePath.empty() ? resolveIniPath() : _iniFilePath;
        CSimpleIniA ini;
        ini.SetUnicode(false);
        const SI_Error loadRc = ini.LoadFile(path.c_str());
        if (loadRc < 0) {
            ROCK_LOG_WARN(Config, "Cannot persist ROCK.ini {} legacy palm pivot A: load failed with code {}", isLeft ? "left" : "right", static_cast<int>(loadRc));
            return false;
        }

        const char* keyX = isLeft ? "fLeftGrabLegacyPalmPivotAHandspaceX" : "fRightGrabLegacyPalmPivotAHandspaceX";
        const char* keyY = isLeft ? "fLeftGrabLegacyPalmPivotAHandspaceY" : "fRightGrabLegacyPalmPivotAHandspaceY";
        const char* keyZ = isLeft ? "fLeftGrabLegacyPalmPivotAHandspaceZ" : "fRightGrabLegacyPalmPivotAHandspaceZ";
        bool ok = true;
        ok &= ini.SetDoubleValue(HAND_FRAME_SECTION, keyX, value.x, nullptr, true) >= 0;
        ok &= ini.SetDoubleValue(HAND_FRAME_SECTION, keyY, value.y, nullptr, true) >= 0;
        ok &= ini.SetDoubleValue(HAND_FRAME_SECTION, keyZ, value.z, nullptr, true) >= 0;
        if (!ok) {
            ROCK_LOG_WARN(Config, "Cannot persist ROCK.ini {} legacy palm pivot A: set failed", isLeft ? "left" : "right");
            return false;
        }

        return saveRuntimeIni(ini, isLeft ? "left legacy palm pivot A" : "right legacy palm pivot A");
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
                if (changeType != filewatch::Event::modified &&
                    changeType != filewatch::Event::added &&
                    changeType != filewatch::Event::renamed_new) {
                    return;
                }

                constexpr auto delay = std::chrono::milliseconds(200);

                auto prevWriteTime = _lastIniFileWriteTime.load();
                std::error_code ec;
                const auto writeTime = std::filesystem::last_write_time(_iniFilePath, ec);
                if (ec || writeTime - prevWriteTime < delay) {
                    return;
                }

                const auto selfWriteTime = _lastSelfIniWriteTime.load(std::memory_order_acquire);
                if (_selfIniWriteInProgress.load(std::memory_order_acquire) ||
                    (selfWriteTime != std::filesystem::file_time_type{} && writeTime <= selfWriteTime)) {
                    _lastIniFileWriteTime.store(writeTime, std::memory_order_release);
                    if (!_selfIniWriteInProgress.load(std::memory_order_acquire)) {
                        _lastSelfIniWriteTime.store(std::filesystem::file_time_type{}, std::memory_order_release);
                    }
                    _ignoreNextIniFileChange.store(false, std::memory_order_release);
                    return;
                }

                if (!_lastIniFileWriteTime.compare_exchange_strong(prevWriteTime, writeTime)) {
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
