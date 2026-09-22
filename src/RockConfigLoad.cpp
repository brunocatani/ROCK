#include "RockConfig.h"

#include <SimpleIni.h>
#include <algorithm>
#include <charconv>
#include <cmath>
#include <exception>
#include <filesystem>
#include <stdexcept>

#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/grab/NearbyGrabDamping.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/RockLoggingPolicy.h"

namespace
{

    constexpr auto SECTION = "PhysicsInteraction";
    constexpr auto LOGGING_SECTION = "Logging";
    constexpr auto DEBUG_SECTION = "Debug";
    constexpr auto REALISTIC_WEAPONS_SECTION = "RealisticWeapons";
    constexpr auto IMMERSIVE_WEAPONS_SECTION = "ImmersiveWeapons";
    constexpr auto AMBIDEXTROUS_FIRING_SECTION = "AmbidextrousFiring";
    constexpr auto NATIVE_SCOPES_SECTION = "NativeScopes";
    constexpr float kDefaultWeaponCollisionVisualStabilizationSeconds = 0.0889f;
    constexpr float kMaxWeaponCollisionVisualStabilizationSeconds = 60.0f / 90.0f;
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
    constexpr float kDefaultGrabThrowMaxVelocityHavok = 12.0f;
    constexpr float kDefaultGrabThrowAngularVelocityScale = 1.0f;
    constexpr float kDefaultGrabThrowMaxAngularVelocityRadiansPerSecond = 18.0f;
    constexpr float kDefaultGrabLongObjectReferenceLeverGameUnits = 24.0f;
    constexpr float kDefaultGrabLongObjectMinAngularScale = 0.35f;
    constexpr float kDefaultGrabEffectiveMotorMassFloor = 2.0f;
    constexpr float kDefaultGrabPositionOnlyAngularScale = 0.55f;
    constexpr float kDefaultGrabSmallObjectReferenceLeverGameUnits = 12.0f;
    constexpr float kDefaultGrabSmallObjectAngularScale = 0.65f;
    constexpr float kDefaultGrabLowContactSupportAngularScale = 0.75f;
    constexpr float kDefaultGrabMinAngularAuthorityScale = 0.30f;
    constexpr float kDefaultGrabWeakPivotTwistScale = 0.35f;
    constexpr float kDefaultGrabMinInertia = 0.01f;
    constexpr float kDefaultGrabThumbSurfaceSafetyMarginGameUnits = 1.0f;
    constexpr int kDefaultHighlightIntensityMode = 2;
    constexpr const char* kDefaultHighlightColor = "blue";

    class RockIniReader
    {
    public:
        RockIniReader(CSimpleIniA& storage, const bool materializeMissingDefaults) noexcept :
            _storage(storage),
            _materializeMissingDefaults(materializeMissingDefaults)
        {}

        [[nodiscard]] bool GetBoolValue(
            const char* section,
            const char* key,
            const bool defaultValue)
        {
            if (shouldMaterialize(section, key)) {
                requireSet(
                    _storage.SetBoolValue(
                        section,
                        key,
                        defaultValue,
                        nullptr,
                        true),
                    section,
                    key);
            }
            return _storage.GetBoolValue(section, key, defaultValue);
        }

        [[nodiscard]] long GetLongValue(
            const char* section,
            const char* key,
            const long defaultValue)
        {
            if (shouldMaterialize(section, key)) {
                requireSet(
                    _storage.SetLongValue(
                        section,
                        key,
                        defaultValue,
                        nullptr,
                        false,
                        true),
                    section,
                    key);
            }
            return _storage.GetLongValue(section, key, defaultValue);
        }

        [[nodiscard]] double GetDoubleValue(
            const char* section,
            const char* key,
            const double defaultValue)
        {
            if (shouldMaterialize(section, key)) {
                // Loadable numeric tuning values are floats. Preserve their
                // round-trip defaults instead of SimpleIni's six-decimal rounding.
                std::array<char, 64> text{};
                const auto [end, error] = std::to_chars(text.data(), text.data() + text.size() - 1,
                    static_cast<float>(defaultValue));
                if (error != std::errc{}) throw std::runtime_error("Cannot format compiled float default");
                *end = '\0';
                requireSet(
                    _storage.SetValue(
                        section,
                        key,
                        text.data(),
                        nullptr,
                        true),
                    section,
                    key);
            }
            return _storage.GetDoubleValue(section, key, defaultValue);
        }

        [[nodiscard]] const char* GetValue(
            const char* section,
            const char* key,
            const char* defaultValue)
        {
            if (defaultValue && shouldMaterialize(section, key)) {
                requireSet(
                    _storage.SetValue(
                        section,
                        key,
                        defaultValue,
                        nullptr,
                        true),
                    section,
                    key);
            }
            return _storage.GetValue(section, key, defaultValue);
        }

        [[nodiscard]] bool materializingMissingDefaults() const noexcept
        {
            return _materializeMissingDefaults;
        }

    private:
        [[nodiscard]] bool shouldMaterialize(
            const char* section,
            const char* key) const noexcept
        {
            return _materializeMissingDefaults &&
                   section && section[0] && key && key[0] &&
                   !_storage.GetValue(section, key, nullptr);
        }

        static void requireSet(
            const SI_Error result,
            const char* section,
            const char* key)
        {
            if (result >= 0) {
                return;
            }

            throw std::runtime_error(
                "Failed to materialize compiled ROCK.ini default [" +
                std::string(section ? section : "") + "] " +
                std::string(key ? key : ""));
        }

        CSimpleIniA& _storage;
        bool _materializeMissingDefaults = false;
    };

    float readClampedFloat(RockIniReader& ini, const char* section, const char* key, float currentValue, float fallback, float minValue, float maxValue)
    {
        float value = static_cast<float>(ini.GetDoubleValue(section, key, currentValue));
        if (!std::isfinite(value)) {
            ROCK_LOG_WARN(Config, "Invalid {}={} -- using {:.2f}", key, value, fallback);
            value = fallback;
        }
        return std::clamp(value, minValue, maxValue);
    }

    int readHighlightIntensityMode(RockIniReader& ini, const char* section, const char* key, int currentValue)
    {
        const int configuredValue = static_cast<int>(ini.GetLongValue(section, key, currentValue));
        if (configuredValue >= 1 && configuredValue <= 4) {
            return configuredValue;
        }

        ROCK_LOG_WARN(Config, "Invalid {}={} -- using {}", key, configuredValue, kDefaultHighlightIntensityMode);
        return kDefaultHighlightIntensityMode;
    }

    std::string readHighlightColor(RockIniReader& ini, const char* section, const char* key, const std::string& currentValue)
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
    RockConfigValues RockConfig::parseValues(CSimpleIniA& source)
    {
        RockConfig parsed;
        parsed.readValuesFromIni(source);
        return std::move(static_cast<RockConfigValues&>(parsed));
    }

    void RockConfig::buildCompiledDefaults(CSimpleIniA& target)
    {
        RockConfig defaults;
        defaults.readValuesFromIni(target, true);
    }

    void RockConfig::readValuesFromIni(
        CSimpleIniA& storage,
        const bool materializeMissingDefaults)
    {
        RockIniReader ini(storage, materializeMissingDefaults);
        auto readVec3 = [&](const char* keyX, const char* keyY, const char* keyZ, RE::NiPoint3& value) {
            value.x = static_cast<float>(ini.GetDoubleValue(SECTION, keyX, value.x));
            value.y = static_cast<float>(ini.GetDoubleValue(SECTION, keyY, value.y));
            value.z = static_cast<float>(ini.GetDoubleValue(SECTION, keyZ, value.z));
        };
        auto readOptionalVec3 = [&](const char* keyX, const char* keyY, const char* keyZ, RE::NiPoint3& value) {
            const bool hasAny = ini.GetValue(SECTION, keyX, nullptr) || ini.GetValue(SECTION, keyY, nullptr) || ini.GetValue(SECTION, keyZ, nullptr);
            if (!hasAny && !ini.materializingMissingDefaults()) {
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
        rockDeveloperModeEnabled = ini.GetBoolValue(DEBUG_SECTION, "bDeveloperModeEnabled", rockDeveloperModeEnabled);
        rockLogLevel = logging_policy::clampLogLevel(static_cast<int>(ini.GetLongValue(LOGGING_SECTION, "iLogLevel", rockLogLevel)));
        rockLogPattern = ini.GetValue(DEBUG_SECTION, "sLogPattern", rockLogPattern.c_str());
        if (rockLogPattern.empty()) {
            rockLogPattern = logging_policy::DefaultLogPattern;
        }
        rockLogSampleMilliseconds =
            logging_policy::sanitizeSampleMilliseconds(static_cast<int>(ini.GetLongValue(DEBUG_SECTION, "iLogSampleMilliseconds", rockLogSampleMilliseconds)));
        rockLogFlushImmediate = ini.GetBoolValue(DEBUG_SECTION, "bLogFlushImmediate", rockLogFlushImmediate);
        rockDebugPipboyPauseInput = ini.GetBoolValue(DEBUG_SECTION, "bDebugPipboyPauseInput", rockDebugPipboyPauseInput);
        rockPerformanceProfilerEnabled = ini.GetBoolValue(DEBUG_SECTION, "bPerformanceProfilerEnabled", rockPerformanceProfilerEnabled);
        rockPerformanceProfilerLogIntervalFrames =
            std::clamp(static_cast<int>(ini.GetLongValue(DEBUG_SECTION, "iPerformanceProfilerLogIntervalFrames", rockPerformanceProfilerLogIntervalFrames)), 30, 54000);
        rockPerformanceProfilerWarmupFrames =
            std::clamp(static_cast<int>(ini.GetLongValue(DEBUG_SECTION, "iPerformanceProfilerWarmupFrames", rockPerformanceProfilerWarmupFrames)), 0, 54000);
        rockPerformanceProfilerOverlayText = ini.GetBoolValue(DEBUG_SECTION, "bPerformanceProfilerOverlayText", rockPerformanceProfilerOverlayText);

        rockHavokTimingFixEnabled = ini.GetBoolValue(SECTION, "bHavokTimingFixEnabled", rockHavokTimingFixEnabled);
        rockVatsPhysicsFixes = ini.GetBoolValue(SECTION, "bVatsPhysicsFixes", rockVatsPhysicsFixes);
        rockHavokTimingFixMinPhysicsFrameRate = havok_timing_fix_policy::sanitizeMinPhysicsFrameRate(
            static_cast<float>(ini.GetDoubleValue(SECTION, "fHavokTimingFixMinPhysicsFrameRate", rockHavokTimingFixMinPhysicsFrameRate)));
        rockHavokTimingFixMaxSubsteps = havok_timing_fix_policy::sanitizeMaxSubsteps(
            static_cast<int>(ini.GetLongValue(SECTION, "iHavokTimingFixMaxSubsteps", rockHavokTimingFixMaxSubsteps)));
        rockSuppressNativeVats = ini.GetBoolValue(SECTION, "bSuppressNativeVats", rockSuppressNativeVats);
        rockPipboyPauseHoldSeconds = pipboy_pause_gesture_policy::sanitizedHoldSeconds(
            static_cast<float>(ini.GetDoubleValue(SECTION, "fPipboyPauseHoldSeconds", rockPipboyPauseHoldSeconds)));
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

        rockImmersiveAidEnabled = ini.GetBoolValue("ImmersiveAid", "bEnabled", rockImmersiveAidEnabled);
        rockImmersiveRecoil = ini.GetBoolValue(
            IMMERSIVE_WEAPONS_SECTION, "bImmersiveRecoil", rockImmersiveRecoil);
        rockBipodMode = ini.GetBoolValue(
            IMMERSIVE_WEAPONS_SECTION, "bBipodMode", rockBipodMode);
        rockLaserRecoilPercent = readClampedFloat(ini, IMMERSIVE_WEAPONS_SECTION, "fLaserRecoilPercent",
            rockLaserRecoilPercent, 100.0f, 0.0f, 300.0f);
        rockPistolOneHandRecoilPercent = readClampedFloat(ini, IMMERSIVE_WEAPONS_SECTION, "fPistolOneHandRecoilPercent",
            rockPistolOneHandRecoilPercent, 300.0f, 0.0f, 300.0f);
        rockPistolTwoHandRecoilPercent = readClampedFloat(ini, IMMERSIVE_WEAPONS_SECTION, "fPistolTwoHandRecoilPercent",
            rockPistolTwoHandRecoilPercent, 80.0f, 0.0f, 300.0f);
        rockRifleOneHandRecoilPercent = readClampedFloat(ini, IMMERSIVE_WEAPONS_SECTION, "fRifleOneHandRecoilPercent",
            rockRifleOneHandRecoilPercent, 300.0f, 0.0f, 300.0f);
        rockRifleTwoHandRecoilPercent = readClampedFloat(ini, IMMERSIVE_WEAPONS_SECTION, "fRifleTwoHandRecoilPercent",
            rockRifleTwoHandRecoilPercent, 80.0f, 0.0f, 300.0f);
        rockShotgunOneHandRecoilPercent = readClampedFloat(ini, IMMERSIVE_WEAPONS_SECTION, "fShotgunOneHandRecoilPercent",
            rockShotgunOneHandRecoilPercent, 300.0f, 0.0f, 300.0f);
        rockShotgunTwoHandRecoilPercent = readClampedFloat(ini, IMMERSIVE_WEAPONS_SECTION, "fShotgunTwoHandRecoilPercent",
            rockShotgunTwoHandRecoilPercent, 80.0f, 0.0f, 300.0f);
        rockHeavyOneHandRecoilPercent = readClampedFloat(ini, IMMERSIVE_WEAPONS_SECTION, "fHeavyOneHandRecoilPercent",
            rockHeavyOneHandRecoilPercent, 300.0f, 0.0f, 300.0f);
        rockHeavyTwoHandRecoilPercent = readClampedFloat(ini, IMMERSIVE_WEAPONS_SECTION, "fHeavyTwoHandRecoilPercent",
            rockHeavyTwoHandRecoilPercent, 292.1f, 0.0f, 300.0f);
        rockDefaultOneHandRecoilPercent = readClampedFloat(ini, IMMERSIVE_WEAPONS_SECTION, "fDefaultOneHandRecoilPercent",
            rockDefaultOneHandRecoilPercent, 300.0f, 0.0f, 300.0f);
        rockDefaultTwoHandRecoilPercent = readClampedFloat(ini, IMMERSIVE_WEAPONS_SECTION, "fDefaultTwoHandRecoilPercent",
            rockDefaultTwoHandRecoilPercent, 80.0f, 0.0f, 300.0f);

        rockDetachEitherHand = ini.GetBoolValue(
            IMMERSIVE_WEAPONS_SECTION,
            "bDetachEitherHand",
            rockDetachEitherHand);
        rockFiringGripDetachPosePreservationEnabled =
            ini.GetBoolValue(
                IMMERSIVE_WEAPONS_SECTION,
                "bFiringGripDetachPosePreservationEnabled",
                rockFiringGripDetachPosePreservationEnabled);
        rockKeepPreviousWeaponInHandOnEquip = ini.GetBoolValue(
            IMMERSIVE_WEAPONS_SECTION, "bKeepPreviousWeaponInHandOnEquip", rockKeepPreviousWeaponInHandOnEquip);
        rockWeaponDropMode = static_cast<int>(ini.GetLongValue(
            IMMERSIVE_WEAPONS_SECTION, "iWeaponDropMode", rockWeaponDropMode));
        if (rockWeaponDropMode < 1 || rockWeaponDropMode > 3) {
            ROCK_LOG_WARN(Config, "Invalid iWeaponDropMode={} -- using 2", rockWeaponDropMode);
            rockWeaponDropMode = 2;
        }
        rockWeaponGrabMode = static_cast<int>(ini.GetLongValue(
            IMMERSIVE_WEAPONS_SECTION, "iWeaponGrabMode", rockWeaponGrabMode));
        if (rockWeaponGrabMode < 1 || rockWeaponGrabMode > 3) {
            ROCK_LOG_WARN(Config, "Invalid iWeaponGrabMode={} -- using 1", rockWeaponGrabMode);
            rockWeaponGrabMode = 1;
        }
        rockGrabAnywhereOnWeapon = ini.GetBoolValue(
            IMMERSIVE_WEAPONS_SECTION,
            "bGrabAnywhereOnWeapon",
            rockGrabAnywhereOnWeapon);
        rockMeleeGripPitchDegrees = readClampedFloat(
            ini, IMMERSIVE_WEAPONS_SECTION, "fMeleeGripPitchDegrees",
            rockMeleeGripPitchDegrees, 0.0f, -180.0f, 180.0f);
        rockFiringGripReattachRadiusGameUnits = readClampedFloat(
            ini,
            IMMERSIVE_WEAPONS_SECTION,
            "fFiringGripReattachRadiusGameUnits",
            rockFiringGripReattachRadiusGameUnits,
            10.0f,
            0.25f,
            30.0f);
        rockFiringGripReattachCylinderRadiusGameUnits = readClampedFloat(
            ini,
            IMMERSIVE_WEAPONS_SECTION,
            "fFiringGripReattachCylinderRadiusGameUnits",
            rockFiringGripReattachCylinderRadiusGameUnits,
            3.0f,
            0.1f,
            30.0f);
        rockGripZoneIndicatorDiameterGameUnits = readClampedFloat(
            ini,
            IMMERSIVE_WEAPONS_SECTION,
            "fGripZoneIndicatorDiameterGameUnits",
            rockGripZoneIndicatorDiameterGameUnits,
            grip_zone_indicator_policy::kDefaultDiameterGameUnits,
            grip_zone_indicator_policy::kMinimumDiameterGameUnits,
            grip_zone_indicator_policy::kMaximumDiameterGameUnits);
        rockFiringGripHapticDurationSeconds = readClampedFloat(
            ini,
            IMMERSIVE_WEAPONS_SECTION,
            "fFiringGripHapticDurationSeconds",
            rockFiringGripHapticDurationSeconds,
            0.10f,
            0.01f,
            0.50f);
        rockFiringGripAttachHapticIntensity = readClampedFloat(
            ini,
            IMMERSIVE_WEAPONS_SECTION,
            "fFiringGripAttachHapticIntensity",
            rockFiringGripAttachHapticIntensity,
            0.85f,
            0.0f,
            1.0f);
        rockFiringGripDetachHapticIntensity = readClampedFloat(
            ini,
            IMMERSIVE_WEAPONS_SECTION,
            "fFiringGripDetachHapticIntensity",
            rockFiringGripDetachHapticIntensity,
            0.30f,
            0.0f,
            1.0f);

        rockLeftHandedMode = ini.GetBoolValue(
            AMBIDEXTROUS_FIRING_SECTION,
            "bLeftHandedMode",
            rockLeftHandedMode);
        rockAmbidextrousFiringGripEnabled = ini.GetBoolValue(
            AMBIDEXTROUS_FIRING_SECTION,
            "bAmbidextrousFiringGripEnabled",
            rockAmbidextrousFiringGripEnabled);
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
        rockWeaponCollisionBlocksProjectiles = ini.GetBoolValue(SECTION, "bWeaponCollisionBlocksProjectiles", rockWeaponCollisionBlocksProjectiles);
        rockLeftFiringGripOffsetGameUnits.x = readClampedFloat(ini, AMBIDEXTROUS_FIRING_SECTION,
            "fLeftFiringGripOffsetXGameUnits", rockLeftFiringGripOffsetGameUnits.x, 0.0f, -15.0f, 15.0f);
        rockLeftFiringGripOffsetGameUnits.y = readClampedFloat(ini, AMBIDEXTROUS_FIRING_SECTION,
            "fLeftFiringGripOffsetYGameUnits", rockLeftFiringGripOffsetGameUnits.y, 0.0f, -15.0f, 15.0f);
        rockLeftFiringGripOffsetGameUnits.z = readClampedFloat(ini, AMBIDEXTROUS_FIRING_SECTION,
            "fLeftFiringGripOffsetZGameUnits", rockLeftFiringGripOffsetGameUnits.z, 0.0f, -15.0f, 15.0f);
        rockRightSupportGripOffsetGameUnits.x = readClampedFloat(ini, AMBIDEXTROUS_FIRING_SECTION,
            "fRightSupportGripOffsetXGameUnits", rockRightSupportGripOffsetGameUnits.x, 0.0f, -15.0f, 15.0f);
        rockRightSupportGripOffsetGameUnits.y = readClampedFloat(ini, AMBIDEXTROUS_FIRING_SECTION,
            "fRightSupportGripOffsetYGameUnits", rockRightSupportGripOffsetGameUnits.y, 0.0f, -15.0f, 15.0f);
        rockRightSupportGripOffsetGameUnits.z = readClampedFloat(ini, AMBIDEXTROUS_FIRING_SECTION,
            "fRightSupportGripOffsetZGameUnits", rockRightSupportGripOffsetGameUnits.z, 0.0f, -15.0f, 15.0f);
        rockWeaponCollisionBlocksSpells = ini.GetBoolValue(SECTION, "bWeaponCollisionBlocksSpells", rockWeaponCollisionBlocksSpells);
        npcDynamicCollisions = ini.GetBoolValue(SECTION, "npcDynamicCollisions", npcDynamicCollisions);
        rockBladePenetrationEnabled = ini.GetBoolValue(SECTION, "bBladePenetrationEnabled", rockBladePenetrationEnabled);
        rockWeaponShellCollisionGraceMs = readClampedFloat(ini, SECTION,
            "fWeaponShellCollisionGraceMs", rockWeaponShellCollisionGraceMs,
            shell_casing_grace::kDefaultMilliseconds, 0.0f, shell_casing_grace::kMaximumMilliseconds);
        rockWeaponCollisionPreserveGaps = ini.GetBoolValue(SECTION, "bWeaponCollisionPreserveGaps", rockWeaponCollisionPreserveGaps);
        rockWeaponCollisionVisualStabilizationSeconds =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionVisualStabilizationSeconds", rockWeaponCollisionVisualStabilizationSeconds));
        if (!std::isfinite(rockWeaponCollisionVisualStabilizationSeconds) ||
            rockWeaponCollisionVisualStabilizationSeconds < 0.0f ||
            rockWeaponCollisionVisualStabilizationSeconds > kMaxWeaponCollisionVisualStabilizationSeconds) {
            ROCK_LOG_WARN(Config,
                "Invalid fWeaponCollisionVisualStabilizationSeconds={} - using {}",
                rockWeaponCollisionVisualStabilizationSeconds,
                kDefaultWeaponCollisionVisualStabilizationSeconds);
            rockWeaponCollisionVisualStabilizationSeconds = kDefaultWeaponCollisionVisualStabilizationSeconds;
        }
        rockWeaponCollisionMaxLinearVelocity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionMaxLinearVelocity", rockWeaponCollisionMaxLinearVelocity));
        rockWeaponCollisionMaxAngularVelocity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionMaxAngularVelocity", rockWeaponCollisionMaxAngularVelocity));
        rockWeaponCollisionGripRecoveryDistanceGameUnits = readClampedFloat(
            ini,
            SECTION,
            "fWeaponCollisionGripRecoveryDistanceGameUnits",
            rockWeaponCollisionGripRecoveryDistanceGameUnits,
            210.0f,
            35.0f,
            700.0f);
        rockWeaponInteractionTouchRadius = readClampedFloat(ini,
            SECTION,
            "fWeaponInteractionTouchRadius",
            rockWeaponInteractionTouchRadius,
            2.0f,
            0.25f,
            6.0f);
        rockWeaponInteractionProbeRadius = static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponInteractionProbeRadius", rockWeaponInteractionProbeRadius));
        rockFiringGripProximitySupportRadius = readClampedFloat(ini,
            SECTION,
            "fFiringGripProximitySupportRadius",
            rockFiringGripProximitySupportRadius,
            8.0f,
            0.25f,
            30.0f);
        rockImmersiveGrenades = ini.GetBoolValue(REALISTIC_WEAPONS_SECTION, "bImmersiveGrenades", rockImmersiveGrenades);
        rockRealisticGrenadeFuseSeconds = readClampedFloat(ini,
            REALISTIC_WEAPONS_SECTION,
            "fRealisticGrenadeFuseSeconds",
            rockRealisticGrenadeFuseSeconds,
            4.0f,
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
        rockEnableImmersiveScopes = ini.GetBoolValue(
            NATIVE_SCOPES_SECTION, "bEnableImmersiveScopes", rockEnableImmersiveScopes);
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
            10.0f,
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
            10.0f,
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

        rockEnableVanillaMelee = ini.GetBoolValue(SECTION, "bEnableVanillaMelee", rockEnableVanillaMelee);
        rockRockyModeEnabled = ini.GetBoolValue(SECTION, "bRockyModeEnabled", rockRockyModeEnabled);
        rockRockyModeHoldSeconds = readClampedFloat(ini, SECTION, "fRockyModeHoldSeconds",
            rockRockyModeHoldSeconds, bare_fist_gesture::kDefaultHoldSeconds,
            bare_fist_gesture::kMinimumHoldSeconds, bare_fist_gesture::kMaximumHoldSeconds);
        rockNativeCharacterControllerObjectContactFilterEnabled = ini.GetBoolValue(
            SECTION, "bNativeCharacterControllerObjectContactFilterEnabled", rockNativeCharacterControllerObjectContactFilterEnabled);

        rockHighlightEnabled = ini.GetBoolValue(SECTION, "bHighlightEnabled", rockHighlightEnabled);
        rockHighlightIntensityMode = readHighlightIntensityMode(ini, SECTION, "iHighlightIntensityMode", rockHighlightIntensityMode);
        rockHighlightColor = readHighlightColor(ini, SECTION, "sHighlightColor", rockHighlightColor);
        rockDebugShowColliders = ini.GetBoolValue(SECTION, "bDebugShowColliders", rockDebugShowColliders);
        rockDebugShowTargetColliders = ini.GetBoolValue(SECTION, "bDebugShowTargetColliders", rockDebugShowTargetColliders);
        rockDebugDrawColliderPhaseDiagnostics = ini.GetBoolValue(
            SECTION,
            "bDebugDrawColliderPhaseDiagnostics",
            rockDebugDrawColliderPhaseDiagnostics);
        rockDebugShowHandAxes = ini.GetBoolValue(SECTION, "bDebugShowHandAxes", rockDebugShowHandAxes);
        rockDebugShowGrabPivots = ini.GetBoolValue(SECTION, "bDebugShowGrabPivots", rockDebugShowGrabPivots);
        rockDebugShowGrabPocketNormal = ini.GetBoolValue(SECTION, "bDebugShowGrabPocketNormal", rockDebugShowGrabPocketNormal);
        rockDebugDrawGrabContactPatch = ini.GetBoolValue(SECTION, "bDebugDrawGrabContactPatch", rockDebugDrawGrabContactPatch);
        rockDebugDrawGrabForceTorque = ini.GetBoolValue(SECTION, "bDebugDrawGrabForceTorque", rockDebugDrawGrabForceTorque);
        rockDebugDrawGrabForceTorqueText = ini.GetBoolValue(SECTION, "bDebugDrawGrabForceTorqueText", rockDebugDrawGrabForceTorqueText);
        rockDebugDrawGrabPivotSourceCollider =
            ini.GetBoolValue(SECTION, "bDebugDrawGrabPivotSourceCollider", rockDebugDrawGrabPivotSourceCollider);
        rockDebugDrawGrabPivotSourceEvidence =
            ini.GetBoolValue(SECTION, "bDebugDrawGrabPivotSourceEvidence", rockDebugDrawGrabPivotSourceEvidence);
        rockDebugDrawGrabSupportFrame = ini.GetBoolValue(SECTION, "bDebugDrawGrabSupportFrame", rockDebugDrawGrabSupportFrame);
        rockDebugDrawGrabPockets = ini.GetBoolValue(SECTION, "bDebugDrawGrabPockets", rockDebugDrawGrabPockets);
        rockDebugShowGrabFingerProbes = ini.GetBoolValue(SECTION, "bDebugShowGrabFingerProbes", rockDebugShowGrabFingerProbes);
        rockDebugShowGrabFingerSweptArc = ini.GetBoolValue(SECTION, "bDebugShowGrabFingerSweptArc", rockDebugShowGrabFingerSweptArc);
        rockDebugShowGrabFingerSweptArcText = ini.GetBoolValue(SECTION, "bDebugShowGrabFingerSweptArcText", rockDebugShowGrabFingerSweptArcText);
        rockDebugShowGrabFingerSweptArcLiveSkeleton = ini.GetBoolValue(SECTION, "bDebugShowGrabFingerSweptArcLiveSkeleton", rockDebugShowGrabFingerSweptArcLiveSkeleton);
        rockDebugShowPalmVectors = ini.GetBoolValue(SECTION, "bDebugShowPalmVectors", rockDebugShowPalmVectors);
        rockDebugDrawHandColliders = ini.GetBoolValue(SECTION, "bDebugDrawHandColliders", rockDebugDrawHandColliders);
        rockDebugDrawHandBoneColliders = ini.GetBoolValue(SECTION, "bDebugDrawHandBoneColliders", rockDebugDrawHandBoneColliders);
        rockDebugDrawBodyBoneColliders = ini.GetBoolValue(
            SECTION,
            "bDebugDrawBodyBoneColliders",
            rockDebugDrawBodyBoneColliders);
        rockDebugDrawDynamicHandColliders = ini.GetBoolValue(SECTION, "bDebugDrawDynamicHandColliders", rockDebugDrawDynamicHandColliders);
        rockDebugDrawHandBoneContacts = ini.GetBoolValue(SECTION, "bDebugDrawHandBoneContacts", rockDebugDrawHandBoneContacts);
        rockDebugDrawGrabAuthorityProxy = ini.GetBoolValue(SECTION, "bDebugDrawGrabAuthorityProxy", rockDebugDrawGrabAuthorityProxy);
        rockDebugMaxHandBoneBodiesDrawn = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxHandBoneBodiesDrawn", rockDebugMaxHandBoneBodiesDrawn));
        if (rockDebugMaxHandBoneBodiesDrawn < 0) {
            rockDebugMaxHandBoneBodiesDrawn = 0;
        } else if (rockDebugMaxHandBoneBodiesDrawn > 48) {
            rockDebugMaxHandBoneBodiesDrawn = 48;
        }
        rockDebugMaxBodyBoneBodiesDrawn = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxBodyBoneBodiesDrawn", rockDebugMaxBodyBoneBodiesDrawn));
        if (rockDebugMaxBodyBoneBodiesDrawn < 0) {
            rockDebugMaxBodyBoneBodiesDrawn = 0;
        } else if (rockDebugMaxBodyBoneBodiesDrawn > 64) {
            rockDebugMaxBodyBoneBodiesDrawn = 64;
        }
        rockDebugDrawWeaponColliders = ini.GetBoolValue(SECTION, "bDebugDrawWeaponColliders", rockDebugDrawWeaponColliders);
        rockDebugDrawGrabbedWeaponPartCollider = ini.GetBoolValue(
            SECTION,
            "bDebugDrawGrabbedWeaponPartCollider",
            rockDebugDrawGrabbedWeaponPartCollider);
        rockDebugDrawNativeScopeActivation =
            ini.GetBoolValue(SECTION, "bDebugDrawNativeScopeActivation", rockDebugDrawNativeScopeActivation);
        rockDebugNativeScopeShotAlignment =
            ini.GetBoolValue(SECTION, "bDebugNativeScopeShotAlignment", rockDebugNativeScopeShotAlignment);
        rockDebugDrawAuthoredGripActivationZones =
            ini.GetBoolValue(SECTION, "bDebugDrawAuthoredGripActivationZones", rockDebugDrawAuthoredGripActivationZones);
        rockDebugDrawWeaponAuthority =
            ini.GetBoolValue(SECTION, "bDebugDrawWeaponAuthority", rockDebugDrawWeaponAuthority);
        rockDebugGripFailureTelemetry =
            ini.GetBoolValue(SECTION, "bDebugGripFailureTelemetry", rockDebugGripFailureTelemetry);
        rockDebugDrawLooseWeaponGripZones =
            ini.GetBoolValue(SECTION, "bDebugDrawLooseWeaponGripZones", rockDebugDrawLooseWeaponGripZones);
        rockDebugDrawDynamicWeaponColliders = ini.GetBoolValue(SECTION, "bDebugDrawDynamicWeaponColliders", rockDebugDrawDynamicWeaponColliders);
        rockDebugDumpWeaponAnimNodes = ini.GetBoolValue(SECTION, "bDebugDumpWeaponAnimNodes", rockDebugDumpWeaponAnimNodes);
        rockDebugMaxWeaponBodiesDrawn = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxWeaponBodiesDrawn", rockDebugMaxWeaponBodiesDrawn));
        rockDebugWeaponAnimNodeDumpIntervalFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iDebugWeaponAnimNodeDumpIntervalFrames", rockDebugWeaponAnimNodeDumpIntervalFrames));
        if (rockDebugWeaponAnimNodeDumpIntervalFrames < 1) {
            rockDebugWeaponAnimNodeDumpIntervalFrames = 1;
        }
        rockDebugMaxShapeCapturesPerFrame = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxShapeCapturesPerFrame", rockDebugMaxShapeCapturesPerFrame));
        rockDebugMaxConvexSupportVertices = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxConvexSupportVertices", rockDebugMaxConvexSupportVertices));
        rockDebugMaxCompoundChildren = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxCompoundChildren", rockDebugMaxCompoundChildren));
        rockDebugMaxCompoundDepth = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxCompoundDepth", rockDebugMaxCompoundDepth));
        rockDebugMaxShapeQueuedJobs = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxShapeQueuedJobs", rockDebugMaxShapeQueuedJobs));
        rockDebugMaxShapeCompletedJobs = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxShapeCompletedJobs", rockDebugMaxShapeCompletedJobs));
        rockDebugMaxShapeUploadsPerFrame = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxShapeUploadsPerFrame", rockDebugMaxShapeUploadsPerFrame));
        rockDebugMaxShapeCacheEntries = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxShapeCacheEntries", rockDebugMaxShapeCacheEntries));
        rockDebugMaxShapeCacheBytes = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxShapeCacheBytes", rockDebugMaxShapeCacheBytes));
        rockDebugMaxBodyInstances = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxBodyInstances", rockDebugMaxBodyInstances));
        rockDebugMaxLineVertices = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxLineVertices", rockDebugMaxLineVertices));
        rockDebugMaxTextVertices = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxTextVertices", rockDebugMaxTextVertices));
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
        rockDebugUseBoundsForHeavyConvex = ini.GetBoolValue(SECTION, "bDebugUseBoundsForHeavyConvex", rockDebugUseBoundsForHeavyConvex);
        rockDebugVerboseLogging = ini.GetBoolValue(SECTION, "bDebugVerboseLogging", rockDebugVerboseLogging);
        rockDebugGrabFrameLogging = ini.GetBoolValue(SECTION, "bDebugGrabFrameLogging", rockDebugGrabFrameLogging);
        rockDebugVideoSyncMarker = ini.GetBoolValue(SECTION, "bDebugVideoSyncMarker", rockDebugVideoSyncMarker);
        rockDebugVideoSyncMarkerSize = static_cast<float>(ini.GetDoubleValue(SECTION, "fDebugVideoSyncMarkerSize", rockDebugVideoSyncMarkerSize));
        rockDebugGrabTimelineTrace = ini.GetBoolValue(SECTION, "bDebugGrabTimelineTrace", rockDebugGrabTimelineTrace);
        rockDebugGrabAfterSolveAnomalySampling =
            ini.GetBoolValue(SECTION, "bDebugGrabAfterSolveAnomalySampling", rockDebugGrabAfterSolveAnomalySampling);
        rockDebugGrabTimelineTraceIntervalFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iDebugGrabTimelineTraceIntervalFrames", rockDebugGrabTimelineTraceIntervalFrames));
        if (rockDebugGrabTimelineTraceIntervalFrames < 1) {
            rockDebugGrabTimelineTraceIntervalFrames = 1;
        }
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
        rockDebugWeaponOmodDumpEnabled = ini.GetBoolValue(SECTION, "bDebugWeaponOmodDump", rockDebugWeaponOmodDumpEnabled);
        rockDebugHandTransformParity = ini.GetBoolValue(SECTION, "bDebugHandTransformParity", rockDebugHandTransformParity);
        rockDebugHandWorldAuthority = ini.GetBoolValue(SECTION, "bDebugHandWorldAuthority", rockDebugHandWorldAuthority);
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
            ini.GetBoolValue(SECTION, "bDebugShowRootFlattenedFingerSkeletonMarkers", rockDebugShowRootFlattenedFingerSkeletonMarkers);
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
            static_cast<float>(ini.GetDoubleValue(SECTION, "fDebugRootFlattenedFingerSkeletonMarkerSize", rockDebugRootFlattenedFingerSkeletonMarkerSize));
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

        rockBodyBoneCollidersEnabled = ini.GetBoolValue(SECTION, "bBodyBoneCollidersEnabled", rockBodyBoneCollidersEnabled);
        rockBodyBoneLegAndFootCollidersEnabled =
            ini.GetBoolValue(SECTION, "bBodyBoneLegAndFootCollidersEnabled", rockBodyBoneLegAndFootCollidersEnabled);
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
        rockSurfaceMeshGrabEnabled =
            ini.GetBoolValue(
                SECTION,
                "bSurfaceMeshGrabEnabled",
                rockSurfaceMeshGrabEnabled);
        rockSurfaceMeshGrabMaxProjectionDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(
                SECTION,
                "fSurfaceMeshGrabMaxProjectionDistanceGameUnits",
                rockSurfaceMeshGrabMaxProjectionDistanceGameUnits));
        if (!std::isfinite(
                rockSurfaceMeshGrabMaxProjectionDistanceGameUnits)) {
            rockSurfaceMeshGrabMaxProjectionDistanceGameUnits =
                48.0f;
        }
        rockSurfaceMeshGrabMaxProjectionDistanceGameUnits =
            std::clamp(
                rockSurfaceMeshGrabMaxProjectionDistanceGameUnits,
                1.0f,
                128.0f);
        rockSurfaceMeshGrabMaxTriangles = std::clamp(
            static_cast<int>(ini.GetLongValue(
                SECTION,
                "iSurfaceMeshGrabMaxTriangles",
                rockSurfaceMeshGrabMaxTriangles)),
            256,
            100000);
        rockHandBoneColliderRadiusScaleOverrides = ini.GetValue(SECTION, "sHandBoneColliderRadiusScaleOverrides", rockHandBoneColliderRadiusScaleOverrides.c_str());
        rockHandPalmColliderDimensionScaleOverrides =
            ini.GetValue(SECTION, "sHandPalmColliderDimensionScaleOverrides", rockHandPalmColliderDimensionScaleOverrides.c_str());
        rockHandBoneCollidersRequirePalmAnchor = ini.GetBoolValue(SECTION, "bHandBoneCollidersRequirePalmAnchor", rockHandBoneCollidersRequirePalmAnchor);
        rockHandBoneColliderMaxLinearVelocity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fHandBoneColliderMaxLinearVelocity", rockHandBoneColliderMaxLinearVelocity));
        rockHandBoneColliderMaxAngularVelocity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fHandBoneColliderMaxAngularVelocity", rockHandBoneColliderMaxAngularVelocity));
        if (!std::isfinite(rockHandBoneColliderMaxLinearVelocity) || rockHandBoneColliderMaxLinearVelocity <= 0.0f) {
            rockHandBoneColliderMaxLinearVelocity = 800.0f;
        }
        if (!std::isfinite(rockHandBoneColliderMaxAngularVelocity) || rockHandBoneColliderMaxAngularVelocity <= 0.0f) {
            rockHandBoneColliderMaxAngularVelocity = 800.0f;
        }

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
        rockGrabFreeLinearAcceleration = readClampedFloat(ini, SECTION, "fGrabFreeLinearAcceleration",
            rockGrabFreeLinearAcceleration, 1000.0f, 1.0f, 10000.0f);
        rockGrabFreeAngularAcceleration = readClampedFloat(ini, SECTION, "fGrabFreeAngularAcceleration",
            rockGrabFreeAngularAcceleration, 6000.0f, 1.0f, 60000.0f);
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
        rockThrowVelocityMultiplier = static_cast<float>(ini.GetDoubleValue(SECTION, "fThrowVelocityMultiplier", rockThrowVelocityMultiplier));
        rockGrabControllerDerivedThrowVelocityEnabled =
            ini.GetBoolValue(SECTION, "bGrabControllerDerivedThrowVelocityEnabled", rockGrabControllerDerivedThrowVelocityEnabled);
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
        rockGrabConvergeMaxTimeSeconds = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabConvergeMaxTimeSeconds", rockGrabConvergeMaxTimeSeconds));
        if (!std::isfinite(rockGrabConvergeMaxTimeSeconds) || rockGrabConvergeMaxTimeSeconds < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabConvergeMaxTimeSeconds={} -- using 0.35", rockGrabConvergeMaxTimeSeconds);
            rockGrabConvergeMaxTimeSeconds = 0.35f;
        }
        rockGrabConvergeStableSeconds = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabConvergeStableSeconds", rockGrabConvergeStableSeconds));
        if (!std::isfinite(rockGrabConvergeStableSeconds) || rockGrabConvergeStableSeconds <= 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabConvergeStableSeconds={} -- using {}", rockGrabConvergeStableSeconds, 0.0333f);
            rockGrabConvergeStableSeconds = 0.0333f;
        }
        // Bounds are the historical 1..12-frame tuning range at 90 Hz.
        rockGrabConvergeStableSeconds = std::clamp(rockGrabConvergeStableSeconds, 0.0111f, 12.0f / 90.0f);
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
        rockGrabOppositionContactMaxAgeSeconds =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabOppositionContactMaxAgeSeconds", rockGrabOppositionContactMaxAgeSeconds));
        rockGrabOppositionContactMaxAgeSeconds =
            std::isfinite(rockGrabOppositionContactMaxAgeSeconds) ?
                std::clamp(rockGrabOppositionContactMaxAgeSeconds, 0.0f, 60.0f / 90.0f) :
                0.0556f;
        rockGrabPinchPocketEnabled = ini.GetBoolValue(SECTION, "bGrabPinchPocketEnabled", rockGrabPinchPocketEnabled);
        rockGrabPinchCloseSelectionEnabled = ini.GetBoolValue(SECTION, "bGrabPinchCloseSelectionEnabled", rockGrabPinchCloseSelectionEnabled);
        rockGrabPinchMaxVolumeCubicGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchMaxVolumeCubicGameUnits",
            rockGrabPinchMaxVolumeCubicGameUnits,
            grab_pinch_pocket_policy::kDefaultMaxVolumeCubicGameUnits,
            0.001f,
            grab_pinch_pocket_policy::kMaxVolumeCubicGameUnits);
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
            ROCK_LOG_WARN(Config, "Invalid fGrabFingerSweepContactRadiusGameUnits={} -- using 0.6", rockGrabFingerSweepContactRadiusGameUnits);
            rockGrabFingerSweepContactRadiusGameUnits = 0.6f;
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
            ROCK_LOG_WARN(Config, "Invalid fGrabThumbSweepMaxOpenValue={} -- using 1.5 (valid range 1.0-2.0)", rockGrabThumbSweepMaxOpenValue);
            rockGrabThumbSweepMaxOpenValue = 1.5f;
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
        if (!std::isfinite(rockPullPresentationGripAxisTiltDegrees) || rockPullPresentationGripAxisTiltDegrees < 0.0f ||
            rockPullPresentationGripAxisTiltDegrees > 45.0f) {
            ROCK_LOG_WARN(Config, "Invalid fPullPresentationGripAxisTiltDegrees={} -- using 15.0", rockPullPresentationGripAxisTiltDegrees);
            rockPullPresentationGripAxisTiltDegrees = 15.0f;
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
        rockShoulderStashRecentContactSeconds =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fShoulderStashRecentContactSeconds", rockShoulderStashRecentContactSeconds));
        rockShoulderStashRecentContactSeconds =
            std::isfinite(rockShoulderStashRecentContactSeconds) ?
                std::clamp(rockShoulderStashRecentContactSeconds, 0.0f, 60.0f / 90.0f) :
                0.0444f;
        rockShoulderStashSustainedContactMissSeconds =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fShoulderStashSustainedContactMissSeconds", rockShoulderStashSustainedContactMissSeconds));
        rockShoulderStashSustainedContactMissSeconds =
            std::isfinite(rockShoulderStashSustainedContactMissSeconds) ?
                std::clamp(rockShoulderStashSustainedContactMissSeconds, 0.0f, 120.0f / 90.0f) :
                18.0f / 90.0f;
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

    }

}
