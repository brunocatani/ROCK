#include "RockConfig.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <exception>
#include <filesystem>
#include <format>
#include <stdexcept>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/RockLoggingPolicy.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/weapon/equip/EquippedWeaponDropPolicy.h"

namespace rock::config
{
    // Reads loader values by TOML table and key. Over a defaults list, each
    // read declares the value passed in as the compiled default, with the
    // type of the read, and returns it. Over loaded values, a read returns the
    // loaded value, or the value passed in when the file does not set it.
    class SettingReader
    {
    public:
        explicit SettingReader(const LoadedValues& values) noexcept : _values(&values) {}

        explicit SettingReader(std::vector<SettingDefault>& defaults) noexcept : _defaults(&defaults) {}

        [[nodiscard]] bool boolean(const char* table, const char* key, bool current)
        {
            if (_defaults) {
                declare(table, key, ValueType::Boolean, current ? "true" : "false");
                return current;
            }
            const auto* value = loaded(table, key);
            return value ? *value == "true" : current;
        }

        [[nodiscard]] long integer(const char* table, const char* key, long current)
        {
            if (_defaults) {
                declare(table, key, ValueType::Integer, std::to_string(current));
                return current;
            }
            if (const auto* value = loaded(table, key)) {
                long parsed{};
                const auto [end, error] = std::from_chars(value->data(), value->data() + value->size(), parsed);
                if (error == std::errc{} && end == value->data() + value->size())
                    return parsed;
            }
            return current;
        }

        [[nodiscard]] double number(const char* table, const char* key, double current)
        {
            if (_defaults) {
                declare(table, key, ValueType::Float, floatText(static_cast<float>(current)));
                return current;
            }
            if (const auto* value = loaded(table, key)) {
                float parsed{};
                const auto [end, error] = std::from_chars(value->data(), value->data() + value->size(), parsed);
                if (error == std::errc{} && end == value->data() + value->size())
                    return parsed;
            }
            return current;
        }

        [[nodiscard]] std::string text(const char* table, const char* key, std::string_view current)
        {
            if (_defaults) {
                declare(table, key, ValueType::String, current);
                return std::string(current);
            }
            const auto* value = loaded(table, key);
            return value ? *value : std::string(current);
        }

        [[nodiscard]] RE::NiPoint3 float3(const char* table, const char* key, const RE::NiPoint3& current)
        {
            if (_defaults) {
                declare(table, key, ValueType::Float3,
                    std::format("{}, {}, {}", floatText(current.x), floatText(current.y), floatText(current.z)));
                return current;
            }
            if (const auto* value = loaded(table, key)) {
                if (const auto parsed = parseFloat3(*value))
                    return RE::NiPoint3((*parsed)[0], (*parsed)[1], (*parsed)[2]);
            }
            return current;
        }

    private:
        static std::string floatText(float value)
        {
            std::array<char, 64> text{};
            const auto [end, error] = std::to_chars(text.data(), text.data() + text.size(), value);
            if (error != std::errc{})
                throw std::runtime_error("Cannot format compiled float default");
            return std::string(text.data(), end);
        }

        void declare(const char* table, const char* key, ValueType type, std::string_view value)
        {
            auto canonical = normalizeValue(type, value);
            if (!canonical)
                throw std::runtime_error(std::format("Invalid compiled default for {}", settingPath(table, key)));
            for (const auto& existing : *_defaults) {
                if (existing.table == table && existing.key == key) {
                    if (existing.type != type || existing.value != *canonical)
                        throw std::runtime_error(
                            std::format("{} is read with two different defaults", settingPath(table, key)));
                    return;
                }
            }
            _defaults->push_back({ table, key, std::move(*canonical), type });
        }

        [[nodiscard]] const std::string* loaded(const char* table, const char* key) const
        {
            const auto found = _values->find(settingPath(table, key));
            return found == _values->end() ? nullptr : &found->second;
        }

        const LoadedValues* _values = nullptr;
        std::vector<SettingDefault>* _defaults = nullptr;
    };
}

namespace
{
    using rock::config::SettingReader;

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
    constexpr float kDefaultGrabMinInertia = 0.01f;
    constexpr float kDefaultGrabThumbSurfaceSafetyMarginGameUnits = 1.0f;
    constexpr int kDefaultHighlightIntensityMode = 2;
    constexpr const char* kDefaultHighlightColor = "blue";

    float readClampedFloat(SettingReader& ini, const char* table, const char* key, float currentValue, float fallback,
        float minValue, float maxValue)
    {
        float value = static_cast<float>(ini.number(table, key, currentValue));
        if (!std::isfinite(value)) {
            ROCK_LOG_WARN(Config, "Invalid {}.{}={} -- using {:.2f}", table, key, value, fallback);
            value = fallback;
        }
        return std::clamp(value, minValue, maxValue);
    }

    // Loaded components are always finite; each is clamped on its own.
    RE::NiPoint3 readClampedFloat3(SettingReader& ini, const char* table, const char* key, const RE::NiPoint3& current,
        float minValue, float maxValue)
    {
        auto value = ini.float3(table, key, current);
        value.x = std::clamp(value.x, minValue, maxValue);
        value.y = std::clamp(value.y, minValue, maxValue);
        value.z = std::clamp(value.z, minValue, maxValue);
        return value;
    }

    int readHighlightIntensityMode(SettingReader& ini, const char* table, const char* key, int currentValue)
    {
        const int configuredValue = static_cast<int>(ini.integer(table, key, currentValue));
        if (configuredValue >= 1 && configuredValue <= 4) {
            return configuredValue;
        }

        ROCK_LOG_WARN(Config, "Invalid {}.{}={} -- using {}", table, key, configuredValue,
            kDefaultHighlightIntensityMode);
        return kDefaultHighlightIntensityMode;
    }

    int readSelectionAimAngleDegrees(SettingReader& ini, const char* table, const char* key, int currentValue)
    {
        const int configuredValue = static_cast<int>(ini.integer(table, key, currentValue));
        const int sanitizedValue = rock::selection_query_policy::sanitizeSelectionAimAngleDegrees(configuredValue);
        if (configuredValue != sanitizedValue) {
            ROCK_LOG_WARN(Config, "Invalid {}.{}={} -- using {}", table, key, configuredValue, sanitizedValue);
        }
        return sanitizedValue;
    }

    // Keys of the ControllerInput.Left and ControllerInput.Right tables.
    struct ButtonLayoutKey
    {
        const char* key;
        rock::button_layout_policy::Slot slot;
    };

    constexpr ButtonLayoutKey kLeftButtonLayoutKeys[] = {
        { "Trigger", rock::button_layout_policy::Slot::Trigger },
        { "Grab", rock::button_layout_policy::Slot::Grab },
        { "X", rock::button_layout_policy::Slot::LowerFace },
        { "Y", rock::button_layout_policy::Slot::UpperFace },
        { "StickClick", rock::button_layout_policy::Slot::StickClick },
    };
    constexpr ButtonLayoutKey kRightButtonLayoutKeys[] = {
        { "Trigger", rock::button_layout_policy::Slot::Trigger },
        { "Grab", rock::button_layout_policy::Slot::Grab },
        { "A", rock::button_layout_policy::Slot::LowerFace },
        { "B", rock::button_layout_policy::Slot::UpperFace },
        { "StickClick", rock::button_layout_policy::Slot::StickClick },
    };

    rock::button_layout_policy::HandLayout readButtonLayout(SettingReader& ini, bool isLeft,
        const rock::button_layout_policy::HandLayout& current)
    {
        namespace layout_policy = rock::button_layout_policy;
        const char* table = isLeft ? "ControllerInput.Left" : "ControllerInput.Right";
        auto layout = current;
        const auto& keys = isLeft ? kLeftButtonLayoutKeys : kRightButtonLayoutKeys;
        for (const auto& entry : keys) {
            const std::string currentName{ layout_policy::sourceName(layout.source(entry.slot), isLeft) };
            const std::string configured = ini.text(table, entry.key, currentName);
            auto& source = layout.sources[static_cast<std::size_t>(entry.slot)];
            if (const auto parsed = layout_policy::parseSource(configured)) {
                source = *parsed;
                continue;
            }
            const auto fallback = layout_policy::defaultSource(entry.slot);
            ROCK_LOG_WARN(Config, "Invalid {}.{}='{}' -- using {}", table, entry.key, configured,
                layout_policy::sourceName(fallback, isLeft));
            source = fallback;
        }
        return layout;
    }

    std::string readHighlightColor(SettingReader& ini, const char* table, const char* key,
        const std::string& currentValue)
    {
        std::string configuredValue = ini.text(table, key, currentValue.c_str());
        for (auto& ch : configuredValue) {
            if (ch >= 'A' && ch <= 'Z') {
                ch = static_cast<char>(ch - 'A' + 'a');
            }
        }

        if (configuredValue == "red" || configuredValue == "blue" || configuredValue == "orange" ||
            configuredValue == "white") {
            return configuredValue;
        }

        ROCK_LOG_WARN(Config, "Invalid {}.{}='{}' -- using {}", table, key, configuredValue, kDefaultHighlightColor);
        return kDefaultHighlightColor;
    }

    // FiringHand.DefaultHand = right | left; returns true for left.
    bool readDefaultHandLeft(SettingReader& ini, const char* table, const char* key, const bool currentLeft)
    {
        std::string configuredValue = ini.text(table, key, currentLeft ? "left" : "right");
        for (auto& ch : configuredValue) {
            if (ch >= 'A' && ch <= 'Z') {
                ch = static_cast<char>(ch - 'A' + 'a');
            }
        }
        if (configuredValue == "left") {
            return true;
        }
        if (configuredValue != "right") {
            ROCK_LOG_WARN(Config, "Invalid {}.{}='{}' -- using right", table, key, configuredValue);
        }
        return false;
    }
}

namespace rock
{
    RockConfigValues RockConfig::parseValues(const config::LoadedValues& source)
    {
        RockConfig parsed;
        config::SettingReader reader(source);
        parsed.readValues(reader);
        return std::move(static_cast<RockConfigValues&>(parsed));
    }

    std::vector<config::SettingDefault> RockConfig::compiledDefaults()
    {
        std::vector<config::SettingDefault> defaults;
        RockConfig compiled;
        config::SettingReader reader(defaults);
        compiled.readValues(reader);
        return defaults;
    }

    void RockConfig::readValues(config::SettingReader& ini)
    {
        auto sanitizeMouthConsumeOffset = [&]() {
            auto sanitizeComponent = [](float value, float fallback) {
                if (!std::isfinite(value)) {
                    return fallback;
                }
                return std::clamp(value, -kMaxMouthConsumeHmdOffsetGameUnits, kMaxMouthConsumeHmdOffsetGameUnits);
            };

            const RE::NiPoint3 original = rockMouthConsumeHmdOffsetGameUnits;
            rockMouthConsumeHmdOffsetGameUnits.x =
                sanitizeComponent(original.x, kDefaultMouthConsumeHmdOffsetGameUnits.x);
            rockMouthConsumeHmdOffsetGameUnits.y =
                sanitizeComponent(original.y, kDefaultMouthConsumeHmdOffsetGameUnits.y);
            rockMouthConsumeHmdOffsetGameUnits.z =
                sanitizeComponent(original.z, kDefaultMouthConsumeHmdOffsetGameUnits.z);
            if (original.x != rockMouthConsumeHmdOffsetGameUnits.x ||
                original.y != rockMouthConsumeHmdOffsetGameUnits.y ||
                original.z != rockMouthConsumeHmdOffsetGameUnits.z) {
                ROCK_LOG_WARN(Config,
                    "Mouth consume HMD offset must be finite and within +/-{:.1f} game units; using ({:.1f}, {:.1f}, "
                    "{:.1f})",
                    kMaxMouthConsumeHmdOffsetGameUnits, rockMouthConsumeHmdOffsetGameUnits.x,
                    rockMouthConsumeHmdOffsetGameUnits.y, rockMouthConsumeHmdOffsetGameUnits.z);
            }
        };
        rockDeveloperModeEnabled = ini.boolean("DeveloperControls", "DeveloperModeEnabled", rockDeveloperModeEnabled);
        rockLogLevel = logging_policy::clampLogLevel(
            static_cast<int>(ini.integer("Logging", "LogLevel", rockLogLevel)));
        rockLogPattern = ini.text("LoggingAndProfiling", "LogPattern", rockLogPattern.c_str());
        if (rockLogPattern.empty()) {
            rockLogPattern = logging_policy::DefaultLogPattern;
        }
        rockLogSampleMilliseconds = logging_policy::sanitizeSampleMilliseconds(
            static_cast<int>(ini.integer("LoggingAndProfiling", "LogSampleMilliseconds", rockLogSampleMilliseconds)));
        rockLogFlushImmediate = ini.boolean("LoggingAndProfiling", "LogFlushImmediate", rockLogFlushImmediate);
        rockPerformanceProfilerEnabled =
            ini.boolean("LoggingAndProfiling", "PerformanceProfilerEnabled", rockPerformanceProfilerEnabled);
        rockPerformanceProfilerLogIntervalFrames =
            std::clamp(static_cast<int>(ini.integer("LoggingAndProfiling", "PerformanceProfilerLogIntervalFrames",
                           rockPerformanceProfilerLogIntervalFrames)),
                30, 54000);
        rockPerformanceProfilerWarmupFrames =
            std::clamp(static_cast<int>(ini.integer("LoggingAndProfiling", "PerformanceProfilerWarmupFrames",
                           rockPerformanceProfilerWarmupFrames)),
                0, 54000);

        rockHavokTimingFixEnabled = ini.boolean("PhysicsTiming", "HavokTimingFixEnabled", rockHavokTimingFixEnabled);
        rockVatsPhysicsFixes = ini.boolean("PhysicsTiming", "VatsPhysicsFixes", rockVatsPhysicsFixes);
        rockHavokTimingFixMinPhysicsFrameRate = havok_timing_fix_policy::sanitizeMinPhysicsFrameRate(static_cast<float>(
            ini.number("PhysicsTiming", "HavokTimingFixMinPhysicsFrameRate", rockHavokTimingFixMinPhysicsFrameRate)));
        rockHavokTimingFixMaxSubsteps = havok_timing_fix_policy::sanitizeMaxSubsteps(
            static_cast<int>(ini.integer("PhysicsTiming", "HavokTimingFixMaxSubsteps", rockHavokTimingFixMaxSubsteps)));
        rockSuppressNativeVats = ini.boolean("ControllerInput", "SuppressNativeVats", rockSuppressNativeVats);
        rockVanillaFavoritesMenu =
            ini.boolean("ControllerInput", "VanillaFavoritesMenu", rockVanillaFavoritesMenu);
        rockPipboyPauseHoldSeconds = pipboy_pause_gesture_policy::sanitizedHoldSeconds(static_cast<float>(
            ini.number("ControllerInput", "PipboyPauseHoldSeconds", rockPipboyPauseHoldSeconds)));
        rockButtonLayouts[0] = readButtonLayout(ini, true, rockButtonLayouts[0]);
        rockButtonLayouts[1] = readButtonLayout(ini, false, rockButtonLayouts[1]);
        rockFarGrabMode = far_pull_gesture::sanitizeMode(
            static_cast<int>(ini.integer("Grabbing", "FarGrabMode", rockFarGrabMode)));
        rockCloseSelectionAngleDegrees =
            readSelectionAimAngleDegrees(ini, "Grabbing", "CloseGrabDirection", rockCloseSelectionAngleDegrees);
        rockFarSelectionAngleDegrees =
            readSelectionAimAngleDegrees(ini, "Grabbing", "FarGrabDirection", rockFarSelectionAngleDegrees);
        rockGrabInputIntentStateEnabled =
            ini.boolean("GrabAcquisition", "GrabInputIntentStateEnabled", rockGrabInputIntentStateEnabled);
        rockLootToHand = ini.boolean("Grabbing", "LootToHand", rockLootToHand);
        rockQuickLootPanelPlacement =
            static_cast<int>(ini.integer("Grabbing", "QuickLootPanelPlacement", rockQuickLootPanelPlacement));
        if (rockQuickLootPanelPlacement < 1 || rockQuickLootPanelPlacement > 3) {
            ROCK_LOG_WARN(Config, "Invalid Grabbing.QuickLootPanelPlacement={} -- using At Container",
                rockQuickLootPanelPlacement);
            rockQuickLootPanelPlacement = 3;
        }
        rockActivatePromptPlacement =
            static_cast<int>(ini.integer("Grabbing", "ActivatePromptPlacement", rockActivatePromptPlacement));
        if (rockActivatePromptPlacement < 1 || rockActivatePromptPlacement > 3) {
            ROCK_LOG_WARN(Config, "Invalid Grabbing.ActivatePromptPlacement={} -- using At Object",
                rockActivatePromptPlacement);
            rockActivatePromptPlacement = 3;
        }
        rockGrabInputLeewaySeconds =
            static_cast<float>(ini.number("GrabAcquisition", "GrabInputLeewaySeconds", rockGrabInputLeewaySeconds));
        if (!std::isfinite(rockGrabInputLeewaySeconds) || rockGrabInputLeewaySeconds < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabInputLeewaySeconds={} -- using 0.12", rockGrabInputLeewaySeconds);
            rockGrabInputLeewaySeconds = 0.12f;
        }
        rockGrabInputLeewaySeconds = std::clamp(rockGrabInputLeewaySeconds, 0.0f, 0.5f);
        rockGrabInputForceSeconds =
            static_cast<float>(ini.number("GrabAcquisition", "GrabInputForceSeconds", rockGrabInputForceSeconds));
        if (!std::isfinite(rockGrabInputForceSeconds) || rockGrabInputForceSeconds < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabInputForceSeconds={} -- using 0.08", rockGrabInputForceSeconds);
            rockGrabInputForceSeconds = 0.08f;
        }
        rockGrabInputForceSeconds = std::clamp(rockGrabInputForceSeconds, 0.0f, 0.3f);

        rockImmersiveAidEnabled = ini.boolean("ImmersiveAid", "Enabled", rockImmersiveAidEnabled);
        rockImmersiveRecoil = ini.boolean("WeaponRecoil", "ImmersiveRecoil", rockImmersiveRecoil);
        rockEmergencyDisableRecoil =
            ini.boolean("DeveloperControls", "EmergencyDisableRecoil", rockEmergencyDisableRecoil);
        rockBipodMode = ini.boolean("WeaponHandling", "BipodMode", rockBipodMode);
        rockGunstockMode = ini.boolean("WeaponHandling", "GunstockMode", rockGunstockMode);
        rockFrikUserWeaponOffsets =
            ini.boolean("WeaponHandling", "FrikUserWeaponOffsets", rockFrikUserWeaponOffsets);
        rockBipodHandsFreeRadiusMeters = static_cast<float>(ini.number("WeaponHandling", "BipodHandsFreeRadiusMeters", rockBipodHandsFreeRadiusMeters));
        if (!std::isfinite(rockBipodHandsFreeRadiusMeters) || rockBipodHandsFreeRadiusMeters <= 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid WeaponHandling.BipodHandsFreeRadiusMeters={} -- using 2.0",
                rockBipodHandsFreeRadiusMeters);
            rockBipodHandsFreeRadiusMeters = 2.0f;
        }
        rockLaserRecoilPercent =
            readClampedFloat(ini, "WeaponRecoil", "LaserRecoilPercent", rockLaserRecoilPercent, 100.0f, 0.0f, 300.0f);
        rockPistolOneHandRecoilPercent = readClampedFloat(ini, "WeaponRecoil", "PistolOneHandRecoilPercent",
            rockPistolOneHandRecoilPercent, 300.0f, 0.0f, 300.0f);
        rockPistolTwoHandRecoilPercent = readClampedFloat(ini, "WeaponRecoil", "PistolTwoHandRecoilPercent",
            rockPistolTwoHandRecoilPercent, 80.0f, 0.0f, 300.0f);
        rockRifleOneHandRecoilPercent = readClampedFloat(ini, "WeaponRecoil", "RifleOneHandRecoilPercent",
            rockRifleOneHandRecoilPercent, 300.0f, 0.0f, 300.0f);
        rockRifleTwoHandRecoilPercent = readClampedFloat(ini, "WeaponRecoil", "RifleTwoHandRecoilPercent",
            rockRifleTwoHandRecoilPercent, 80.0f, 0.0f, 300.0f);
        rockShotgunOneHandRecoilPercent = readClampedFloat(ini, "WeaponRecoil", "ShotgunOneHandRecoilPercent",
            rockShotgunOneHandRecoilPercent, 300.0f, 0.0f, 300.0f);
        rockShotgunTwoHandRecoilPercent = readClampedFloat(ini, "WeaponRecoil", "ShotgunTwoHandRecoilPercent",
            rockShotgunTwoHandRecoilPercent, 80.0f, 0.0f, 300.0f);
        rockHeavyOneHandRecoilPercent = readClampedFloat(ini, "WeaponRecoil", "HeavyOneHandRecoilPercent",
            rockHeavyOneHandRecoilPercent, 300.0f, 0.0f, 300.0f);
        rockHeavyTwoHandRecoilPercent = readClampedFloat(ini, "WeaponRecoil", "HeavyTwoHandRecoilPercent",
            rockHeavyTwoHandRecoilPercent, 300.0f, 0.0f, 300.0f);
        rockDefaultOneHandRecoilPercent = readClampedFloat(ini, "WeaponRecoil", "DefaultOneHandRecoilPercent",
            rockDefaultOneHandRecoilPercent, 300.0f, 0.0f, 300.0f);
        rockDefaultTwoHandRecoilPercent = readClampedFloat(ini, "WeaponRecoil", "DefaultTwoHandRecoilPercent",
            rockDefaultTwoHandRecoilPercent, 80.0f, 0.0f, 300.0f);

        rockDetachEitherHand = ini.boolean("WeaponHandling", "DetachEitherHand", rockDetachEitherHand);
        rockWeaponEquipMode =
            static_cast<int>(ini.integer("WeaponHandling", "WeaponEquipMode", rockWeaponEquipMode));
        if (rockWeaponEquipMode < equipped_weapon_drop_policy::kFirstEquipMode ||
            rockWeaponEquipMode > equipped_weapon_drop_policy::kAutoEquipMode) {
            ROCK_LOG_WARN(Config, "Invalid WeaponHandling.WeaponEquipMode={} -- using 2", rockWeaponEquipMode);
            rockWeaponEquipMode = 2;
        }
        rockWeaponGrabMode =
            static_cast<int>(ini.integer("WeaponHandling", "WeaponGrabMode", rockWeaponGrabMode));
        if (rockWeaponGrabMode < 1 || rockWeaponGrabMode > 3) {
            ROCK_LOG_WARN(Config, "Invalid WeaponHandling.WeaponGrabMode={} -- using 1", rockWeaponGrabMode);
            rockWeaponGrabMode = 1;
        }
        rockAutoEquipSettleSeconds = readClampedFloat(ini, "WeaponHandling", "AutoEquipSettleSeconds",
            rockAutoEquipSettleSeconds, 0.15f, 0.0f, 2.0f);
        rockGrabAnywhereOnWeapon =
            ini.boolean("WeaponHandling", "GrabAnywhereOnWeapon", rockGrabAnywhereOnWeapon);
        rockMeleeGripPitchDegrees = readClampedFloat(ini, "WeaponHandling", "MeleeGripPitchDegrees",
            rockMeleeGripPitchDegrees, 0.0f, -180.0f, 180.0f);
        rockWeaponFilteringPreset = static_cast<int>(
            ini.integer("WeaponHandling", "WeaponFilteringPreset", rockWeaponFilteringPreset));
        if (rockWeaponFilteringPreset < 0 || rockWeaponFilteringPreset > 4) {
            ROCK_LOG_WARN(Config, "Invalid WeaponHandling.WeaponFilteringPreset={} -- using Off", rockWeaponFilteringPreset);
            rockWeaponFilteringPreset = 0;
        }
        rockScopedFilterRotationSeconds = readClampedFloat(ini, "WeaponHandling", "ScopedFilterRotationSeconds", rockScopedFilterRotationSeconds, 0.15f, 0.0f, 1.0f);
        rockScopedFilterTranslationSeconds = readClampedFloat(ini, "WeaponHandling", "ScopedFilterTranslationSeconds", rockScopedFilterTranslationSeconds, 0.08f, 0.0f, 1.0f);
        rockScopedFilterReleaseDegreesPerSecond = readClampedFloat(ini, "WeaponHandling", "ScopedFilterReleaseDegreesPerSecond", rockScopedFilterReleaseDegreesPerSecond, 45.0f, 0.0f, 720.0f);
        rockFiringGripReattachRadiusGameUnits = readClampedFloat(ini, "WeaponHandling", "FiringGripReattachRadiusGameUnits", rockFiringGripReattachRadiusGameUnits, 10.0f, 0.25f, 30.0f);
        rockFiringGripReattachCylinderRadiusGameUnits =
            readClampedFloat(ini, "WeaponCollisionAndSupport", "FiringGripReattachCylinderRadiusGameUnits",
                rockFiringGripReattachCylinderRadiusGameUnits, 3.0f, 0.1f, 30.0f);
        rockGripZoneIndicatorDiameterGameUnits =
            readClampedFloat(ini, "WeaponHandling", "GripZoneIndicatorDiameterGameUnits",
                rockGripZoneIndicatorDiameterGameUnits, grip_zone_indicator_policy::kDefaultDiameterGameUnits,
                grip_zone_indicator_policy::kMinimumDiameterGameUnits,
                grip_zone_indicator_policy::kMaximumDiameterGameUnits);
        rockFiringGripHapticDurationSeconds = readClampedFloat(ini, "Haptics", "FiringGripHapticDurationSeconds",
            rockFiringGripHapticDurationSeconds, 0.10f, 0.01f, 0.50f);
        rockFiringGripAttachHapticIntensity = readClampedFloat(ini, "Haptics", "FiringGripAttachHapticIntensity",
            rockFiringGripAttachHapticIntensity, 0.85f, 0.0f, 1.0f);
        rockFiringGripDetachHapticIntensity = readClampedFloat(ini, "Haptics", "FiringGripDetachHapticIntensity",
            rockFiringGripDetachHapticIntensity, 0.30f, 0.0f, 1.0f);
        rockAmmoCounterStyle =
            static_cast<int>(ini.integer("WeaponHandling", "AmmoCounterStyle", rockAmmoCounterStyle));
        if (rockAmmoCounterStyle < 1 || rockAmmoCounterStyle > 3) {
            ROCK_LOG_WARN(Config, "Invalid WeaponHandling.AmmoCounterStyle={} -- using Minimal", rockAmmoCounterStyle);
            rockAmmoCounterStyle = 1;
        }

        rockDefaultHandLeft =
            readDefaultHandLeft(ini, "FiringHand", "DefaultHand", rockDefaultHandLeft);
        rockLeftFiringAimYawDegrees = readClampedFloat(ini, "FiringHand.Left", "AimYawDegrees",
            rockLeftFiringAimYawDegrees, 0.0f, -30.0f, 30.0f);
        rockLeftFiringAimPitchDegrees = readClampedFloat(ini, "FiringHand.Left", "AimPitchDegrees",
            rockLeftFiringAimPitchDegrees, 0.0f, -30.0f, 30.0f);
        {
            const auto aimOffset = readClampedFloat3(ini, "FiringHand.Left", "AimOffsetGameUnits",
                RE::NiPoint3(rockLeftFiringAimOffsetXGameUnits, rockLeftFiringAimOffsetYGameUnits,
                    rockLeftFiringAimOffsetZGameUnits),
                -15.0f, 15.0f);
            rockLeftFiringAimOffsetXGameUnits = aimOffset.x;
            rockLeftFiringAimOffsetYGameUnits = aimOffset.y;
            rockLeftFiringAimOffsetZGameUnits = aimOffset.z;
        }
        rockWeaponCollisionBlocksProjectiles =
            ini.boolean("WeaponCollisionAndSupport", "WeaponCollisionBlocksProjectiles", rockWeaponCollisionBlocksProjectiles);
        rockLeftFiringGripOffsetGameUnits = readClampedFloat3(ini, "FiringHand.Left", "GripOffsetGameUnits",
            rockLeftFiringGripOffsetGameUnits, -15.0f, 15.0f);
        rockRightSupportGripOffsetGameUnits = readClampedFloat3(ini, "FiringHand.Right", "SupportGripOffsetGameUnits",
            rockRightSupportGripOffsetGameUnits, -15.0f, 15.0f);
        rockWeaponCollisionBlocksSpells =
            ini.boolean("WeaponCollisionAndSupport", "WeaponCollisionBlocksSpells", rockWeaponCollisionBlocksSpells);
        rockWeaponShellCollisionGraceMs =
            readClampedFloat(ini, "WeaponCollisionAndSupport", "WeaponShellCollisionGraceMs", rockWeaponShellCollisionGraceMs,
                shell_casing_grace::kDefaultMilliseconds, 0.0f, shell_casing_grace::kMaximumMilliseconds);
        rockWeaponCollisionPreserveGaps =
            ini.boolean("WeaponCollisionAndSupport", "WeaponCollisionPreserveGaps", rockWeaponCollisionPreserveGaps);
        rockWeaponCollisionVisualStabilizationSeconds = static_cast<float>(ini.number("WeaponCollisionAndSupport", "WeaponCollisionVisualStabilizationSeconds", rockWeaponCollisionVisualStabilizationSeconds));
        if (!std::isfinite(rockWeaponCollisionVisualStabilizationSeconds) ||
            rockWeaponCollisionVisualStabilizationSeconds < 0.0f ||
            rockWeaponCollisionVisualStabilizationSeconds > kMaxWeaponCollisionVisualStabilizationSeconds) {
            ROCK_LOG_WARN(Config, "Invalid WeaponCollisionAndSupport.WeaponCollisionVisualStabilizationSeconds={} - using {}",
                rockWeaponCollisionVisualStabilizationSeconds, kDefaultWeaponCollisionVisualStabilizationSeconds);
            rockWeaponCollisionVisualStabilizationSeconds = kDefaultWeaponCollisionVisualStabilizationSeconds;
        }
        rockWeaponCollisionMaxLinearVelocity = static_cast<float>(
            ini.number("WeaponCollisionAndSupport", "WeaponCollisionMaxLinearVelocity", rockWeaponCollisionMaxLinearVelocity));
        rockWeaponCollisionMaxAngularVelocity = static_cast<float>(
            ini.number("WeaponCollisionAndSupport", "WeaponCollisionMaxAngularVelocity", rockWeaponCollisionMaxAngularVelocity));
        rockWeaponCollisionGripRecoveryDistanceGameUnits =
            readClampedFloat(ini, "WeaponCollisionAndSupport", "WeaponCollisionGripRecoveryDistanceGameUnits",
                rockWeaponCollisionGripRecoveryDistanceGameUnits, 210.0f, 35.0f, 700.0f);
        rockWeaponInteractionTouchRadius = readClampedFloat(ini, "WeaponCollisionAndSupport", "WeaponInteractionTouchRadius",
            rockWeaponInteractionTouchRadius, 2.0f, 0.25f, 6.0f);
        rockWeaponInteractionProbeRadius = static_cast<float>(
            ini.number("WeaponCollisionAndSupport", "WeaponInteractionProbeRadius", rockWeaponInteractionProbeRadius));
        rockDebugWeaponSupportDynamicOnly =
            ini.boolean("WeaponCollisionAndSupport", "DebugWeaponSupportDynamicOnly", rockDebugWeaponSupportDynamicOnly);
        rockFiringGripProximitySupportRadius = readClampedFloat(ini, "WeaponCollisionAndSupport", "FiringGripProximitySupportRadius",
            rockFiringGripProximitySupportRadius, 8.0f, 0.25f, 30.0f);
        rockImmersiveGrenades = ini.boolean("Grenades", "ImmersiveGrenades", rockImmersiveGrenades);
        rockRealisticGrenadeFuseSeconds = readClampedFloat(ini, "Grenades", "RealisticGrenadeFuseSeconds",
            rockRealisticGrenadeFuseSeconds, 4.0f, 0.0f, 30.0f);
        rockWeaponSupportGripHandLerpEnabled =
            ini.boolean("WeaponCollisionAndSupport", "WeaponSupportGripHandLerpEnabled", rockWeaponSupportGripHandLerpEnabled);
        rockWeaponSupportGripHandLerpTimeMin = readClampedFloat(ini, "WeaponCollisionAndSupport", "WeaponSupportGripHandLerpTimeMin",
            rockWeaponSupportGripHandLerpTimeMin, 0.12f, 0.0f, 1.0f);
        rockWeaponSupportGripHandLerpTimeMax = readClampedFloat(ini, "WeaponCollisionAndSupport", "WeaponSupportGripHandLerpTimeMax",
            rockWeaponSupportGripHandLerpTimeMax, 0.20f, rockWeaponSupportGripHandLerpTimeMin, 1.0f);
        rockWeaponSupportGripHandLerpMinDistance = readClampedFloat(ini, "WeaponCollisionAndSupport", "WeaponSupportGripHandLerpMinDistance", rockWeaponSupportGripHandLerpMinDistance, 1.0f, 0.0f, 80.0f);
        rockWeaponSupportGripHandLerpMaxDistance =
            readClampedFloat(ini, "WeaponCollisionAndSupport", "WeaponSupportGripHandLerpMaxDistance",
                rockWeaponSupportGripHandLerpMaxDistance, 14.0f, rockWeaponSupportGripHandLerpMinDistance, 120.0f);
        rockWeaponSupportSurfaceSeatEnabled =
            ini.boolean("WeaponCollisionAndSupport", "WeaponSupportSurfaceSeatEnabled", rockWeaponSupportSurfaceSeatEnabled);
        rockWeaponSupportSurfaceSeatMaxDegrees = readClampedFloat(ini, "WeaponCollisionAndSupport", "WeaponSupportSurfaceSeatMaxDegrees",
            rockWeaponSupportSurfaceSeatMaxDegrees, 35.0f, 0.0f, 75.0f);
        rockEnableImmersiveScopes =
            ini.boolean("Scopes", "EnableImmersiveScopes", rockEnableImmersiveScopes);
        rockManualScopeHoldSeconds = readClampedFloat(ini, "Scopes", "ManualScopeHoldSeconds",
            rockManualScopeHoldSeconds, 0.30f, 0.05f, 2.0f);
        {
            const auto fallbackOffset = readClampedFloat3(ini, "ScopePlacement",
                "NativeScopeFiringGripFallbackOffsetGameUnits",
                RE::NiPoint3(rockNativeScopeFiringGripFallbackOffsetXGameUnits,
                    rockNativeScopeFiringGripFallbackOffsetYGameUnits, rockNativeScopeFiringGripFallbackOffsetZGameUnits),
                -100.0f, 100.0f);
            rockNativeScopeFiringGripFallbackOffsetXGameUnits = fallbackOffset.x;
            rockNativeScopeFiringGripFallbackOffsetYGameUnits = fallbackOffset.y;
            rockNativeScopeFiringGripFallbackOffsetZGameUnits = fallbackOffset.z;
            // [pitch, yaw, roll]
            const auto fallbackRotation = readClampedFloat3(ini, "ScopePlacement",
                "NativeScopeFiringGripFallbackRotationDegrees",
                RE::NiPoint3(rockNativeScopeFiringGripFallbackPitchDegrees, rockNativeScopeFiringGripFallbackYawDegrees,
                    rockNativeScopeFiringGripFallbackRollDegrees),
                -180.0f, 180.0f);
            rockNativeScopeFiringGripFallbackPitchDegrees = fallbackRotation.x;
            rockNativeScopeFiringGripFallbackYawDegrees = fallbackRotation.y;
            rockNativeScopeFiringGripFallbackRollDegrees = fallbackRotation.z;
            const auto overlayOffset = readClampedFloat3(ini, "ScopePlacement", "NativeScopeOverlayOffsetGameUnits",
                RE::NiPoint3(rockNativeScopeOverlayOffsetXGameUnits, rockNativeScopeOverlayOffsetYGameUnits,
                    rockNativeScopeOverlayOffsetZGameUnits),
                -100.0f, 100.0f);
            rockNativeScopeOverlayOffsetXGameUnits = overlayOffset.x;
            rockNativeScopeOverlayOffsetYGameUnits = overlayOffset.y;
            rockNativeScopeOverlayOffsetZGameUnits = overlayOffset.z;
            const auto overlayRotation = readClampedFloat3(ini, "ScopePlacement", "NativeScopeOverlayRotationDegrees",
                RE::NiPoint3(rockNativeScopeOverlayPitchDegrees, rockNativeScopeOverlayYawDegrees,
                    rockNativeScopeOverlayRollDegrees),
                -180.0f, 180.0f);
            rockNativeScopeOverlayPitchDegrees = overlayRotation.x;
            rockNativeScopeOverlayYawDegrees = overlayRotation.y;
            rockNativeScopeOverlayRollDegrees = overlayRotation.z;
        }

        rockVanillaMeleeSwingSpeed = readClampedFloat(ini, "NativeGameplay", "VanillaMeleeSwingSpeed",
            rockVanillaMeleeSwingSpeed, 300.0f, 50.0f, 2000.0f);
        rockVanillaGunBashSpeed = readClampedFloat(ini, "NativeGameplay", "VanillaGunBashSpeed",
            rockVanillaGunBashSpeed, 400.0f, 50.0f, 2000.0f);

        rockHighlightEnabled = ini.boolean("SelectionFeedback", "HighlightEnabled", rockHighlightEnabled);
        rockHighlightIntensityMode =
            readHighlightIntensityMode(ini, "SelectionFeedback", "HighlightIntensityMode", rockHighlightIntensityMode);
        rockHighlightColor = readHighlightColor(ini, "SelectionFeedback", "HighlightColor", rockHighlightColor);
        rockDebugShowColliders = ini.boolean("DebugOverlay", "DebugShowColliders", rockDebugShowColliders);
        rockDebugShowTargetColliders =
            ini.boolean("DebugOverlay", "DebugShowTargetColliders", rockDebugShowTargetColliders);
        rockDebugShowHandAxes = ini.boolean("DebugOverlay", "DebugShowHandAxes", rockDebugShowHandAxes);
        rockDebugShowGrabPivots = ini.boolean("DebugOverlay", "DebugShowGrabPivots", rockDebugShowGrabPivots);
        rockDebugShowGrabPocketNormal =
            ini.boolean("DebugOverlay", "DebugShowGrabPocketNormal", rockDebugShowGrabPocketNormal);
        rockDebugDrawGrabForceTorque =
            ini.boolean("DebugOverlay", "DebugDrawGrabForceTorque", rockDebugDrawGrabForceTorque);
        rockDebugDrawGrabForceTorqueText =
            ini.boolean("DebugOverlay", "DebugDrawGrabForceTorqueText", rockDebugDrawGrabForceTorqueText);
        rockDebugDrawGrabPivotSourceCollider =
            ini.boolean("DebugOverlay", "DebugDrawGrabPivotSourceCollider", rockDebugDrawGrabPivotSourceCollider);
        rockDebugDrawGrabPivotSourceEvidence =
            ini.boolean("DebugOverlay", "DebugDrawGrabPivotSourceEvidence", rockDebugDrawGrabPivotSourceEvidence);
        rockDebugDrawGrabSupportFrame =
            ini.boolean("DebugOverlay", "DebugDrawGrabSupportFrame", rockDebugDrawGrabSupportFrame);
        rockDebugDrawGrabPockets = ini.boolean("DebugOverlay", "DebugDrawGrabPockets", rockDebugDrawGrabPockets);
        rockDebugDrawGrabFingerProbes =
            ini.boolean("DebugOverlay", "DebugDrawGrabFingerProbes", rockDebugDrawGrabFingerProbes);
        rockDebugShowPalmVectors = ini.boolean("DebugOverlay", "DebugShowPalmVectors", rockDebugShowPalmVectors);
        rockDebugDrawHandColliders = ini.boolean("DebugOverlay", "DebugDrawHandColliders", rockDebugDrawHandColliders);
        rockDebugDrawHandBoneColliders =
            ini.boolean("DebugOverlay", "DebugDrawHandBoneColliders", rockDebugDrawHandBoneColliders);
        rockDebugDrawBodyBoneColliders =
            ini.boolean("DebugOverlay", "DebugDrawBodyBoneColliders", rockDebugDrawBodyBoneColliders);
        rockDebugDrawDynamicHandColliders =
            ini.boolean("DebugOverlay", "DebugDrawDynamicHandColliders", rockDebugDrawDynamicHandColliders);
        rockDebugDrawHandBoneContacts =
            ini.boolean("DebugOverlay", "DebugDrawHandBoneContacts", rockDebugDrawHandBoneContacts);
        rockDebugDrawGrabAuthorityProxy =
            ini.boolean("Haptics", "DebugDrawGrabAuthorityProxy", rockDebugDrawGrabAuthorityProxy);
        rockDebugMaxHandBoneBodiesDrawn = static_cast<int>(
            ini.integer("DebugOverlay", "DebugMaxHandBoneBodiesDrawn", rockDebugMaxHandBoneBodiesDrawn));
        if (rockDebugMaxHandBoneBodiesDrawn < 0) {
            rockDebugMaxHandBoneBodiesDrawn = 0;
        } else if (rockDebugMaxHandBoneBodiesDrawn > 48) {
            rockDebugMaxHandBoneBodiesDrawn = 48;
        }
        rockDebugMaxBodyBoneBodiesDrawn = static_cast<int>(
            ini.integer("DebugOverlay", "DebugMaxBodyBoneBodiesDrawn", rockDebugMaxBodyBoneBodiesDrawn));
        if (rockDebugMaxBodyBoneBodiesDrawn < 0) {
            rockDebugMaxBodyBoneBodiesDrawn = 0;
        } else if (rockDebugMaxBodyBoneBodiesDrawn > 64) {
            rockDebugMaxBodyBoneBodiesDrawn = 64;
        }
        rockDebugDrawWeaponColliders =
            ini.boolean("DebugOverlay", "DebugDrawWeaponColliders", rockDebugDrawWeaponColliders);
        rockDebugDrawGrabbedWeaponPartCollider =
            ini.boolean("DebugOverlay", "DebugDrawGrabbedWeaponPartCollider", rockDebugDrawGrabbedWeaponPartCollider);
        rockDebugDrawNativeScopeActivation =
            ini.boolean("DebugOverlay", "DebugDrawNativeScopeActivation", rockDebugDrawNativeScopeActivation);
        rockDebugDrawAuthoredGripActivationZones = ini.boolean("DebugOverlay", "DebugDrawAuthoredGripActivationZones",
            rockDebugDrawAuthoredGripActivationZones);
        rockDebugDrawWeaponAuthority =
            ini.boolean("DebugOverlay", "DebugDrawWeaponAuthority", rockDebugDrawWeaponAuthority);
        rockDebugDrawGunstock = ini.boolean("DebugOverlay", "DebugDrawGunstock", rockDebugDrawGunstock);
        rockDebugDrawControllerAim = ini.boolean("DebugOverlay", "DebugDrawControllerAim", rockDebugDrawControllerAim);
        rockDebugDrawLooseWeaponGripZones =
            ini.boolean("DebugOverlay", "DebugDrawLooseWeaponGripZones", rockDebugDrawLooseWeaponGripZones);
        rockDebugDrawDynamicWeaponColliders =
            ini.boolean("DebugOverlay", "DebugDrawDynamicWeaponColliders", rockDebugDrawDynamicWeaponColliders);
        rockDebugMaxWeaponBodiesDrawn =
            static_cast<int>(ini.integer("DebugOverlay", "DebugMaxWeaponBodiesDrawn", rockDebugMaxWeaponBodiesDrawn));
        rockDebugMaxShapeCapturesPerFrame = static_cast<int>(
            ini.integer("DebugOverlay", "DebugMaxShapeCapturesPerFrame", rockDebugMaxShapeCapturesPerFrame));
        rockDebugMaxConvexSupportVertices = static_cast<int>(
            ini.integer("DebugOverlay", "DebugMaxConvexSupportVertices", rockDebugMaxConvexSupportVertices));
        rockDebugMaxCompoundChildren =
            static_cast<int>(ini.integer("DebugOverlay", "DebugMaxCompoundChildren", rockDebugMaxCompoundChildren));
        rockDebugMaxCompoundDepth =
            static_cast<int>(ini.integer("DebugOverlay", "DebugMaxCompoundDepth", rockDebugMaxCompoundDepth));
        rockDebugMaxShapeQueuedJobs =
            static_cast<int>(ini.integer("DebugOverlay", "DebugMaxShapeQueuedJobs", rockDebugMaxShapeQueuedJobs));
        rockDebugMaxShapeCompletedJobs =
            static_cast<int>(ini.integer("DebugOverlay", "DebugMaxShapeCompletedJobs", rockDebugMaxShapeCompletedJobs));
        rockDebugMaxShapeUploadsPerFrame = static_cast<int>(
            ini.integer("DebugOverlay", "DebugMaxShapeUploadsPerFrame", rockDebugMaxShapeUploadsPerFrame));
        rockDebugMaxShapeCacheEntries =
            static_cast<int>(ini.integer("DebugOverlay", "DebugMaxShapeCacheEntries", rockDebugMaxShapeCacheEntries));
        rockDebugMaxShapeCacheBytes =
            static_cast<int>(ini.integer("DebugOverlay", "DebugMaxShapeCacheBytes", rockDebugMaxShapeCacheBytes));
        rockDebugMaxBodyInstances =
            static_cast<int>(ini.integer("DebugOverlay", "DebugMaxBodyInstances", rockDebugMaxBodyInstances));
        rockDebugMaxLineVertices =
            static_cast<int>(ini.integer("DebugOverlay", "DebugMaxLineVertices", rockDebugMaxLineVertices));
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
        rockDebugUseBoundsForHeavyConvex =
            ini.boolean("DebugOverlay", "DebugUseBoundsForHeavyConvex", rockDebugUseBoundsForHeavyConvex);
        rockDebugVerboseLogging = ini.boolean("LogsAndTelemetry", "DebugVerboseLogging", rockDebugVerboseLogging);
        rockDebugTestbenchTelemetry =
            ini.boolean("LogsAndTelemetry", "DebugTestbenchTelemetry", rockDebugTestbenchTelemetry);
        rockDebugShowRootFlattenedFingerSkeletonMarkers = ini.boolean("SkeletonDebug", "DebugShowRootFlattenedFingerSkeletonMarkers", rockDebugShowRootFlattenedFingerSkeletonMarkers);
        rockDebugShowSkeletonBoneVisualizer =
            ini.boolean("SkeletonDebug", "DebugShowSkeletonBoneVisualizer", rockDebugShowSkeletonBoneVisualizer);
        rockDebugSkeletonBoneMode =
            static_cast<int>(ini.integer("SkeletonDebug", "DebugSkeletonBoneMode", rockDebugSkeletonBoneMode));
        if (rockDebugSkeletonBoneMode < 0 || rockDebugSkeletonBoneMode > 3) {
            rockDebugSkeletonBoneMode = 1;
        }
        rockDebugSkeletonBoneSource =
            static_cast<int>(ini.integer("SkeletonDebug", "DebugSkeletonBoneSource", rockDebugSkeletonBoneSource));
        if (rockDebugSkeletonBoneSource != 1 && rockDebugSkeletonBoneSource != 2) {
            rockDebugSkeletonBoneSource = 1;
        }
        rockDebugDrawSkeletonBoneAxes =
            ini.boolean("SkeletonDebug", "DebugDrawSkeletonBoneAxes", rockDebugDrawSkeletonBoneAxes);
        rockDebugLogSkeletonBones = ini.boolean("SkeletonDebug", "DebugLogSkeletonBones", rockDebugLogSkeletonBones);
        rockDebugLogSkeletonBoneTruncation =
            ini.boolean("SkeletonDebug", "DebugLogSkeletonBoneTruncation", rockDebugLogSkeletonBoneTruncation);
        rockDebugSkeletonBoneLogFilter =
            ini.text("SkeletonDebug", "DebugSkeletonBoneLogFilter", rockDebugSkeletonBoneLogFilter.c_str());
        rockDebugSkeletonAxisBoneFilter =
            ini.text("SkeletonDebug", "DebugSkeletonAxisBoneFilter", rockDebugSkeletonAxisBoneFilter.c_str());
        rockDebugSkeletonBoneLogIntervalFrames = static_cast<int>(
            ini.integer("SkeletonDebug", "DebugSkeletonBoneLogIntervalFrames", rockDebugSkeletonBoneLogIntervalFrames));
        if (rockDebugSkeletonBoneLogIntervalFrames < 1) {
            rockDebugSkeletonBoneLogIntervalFrames = 1;
        }
        rockDebugMaxSkeletonBonesDrawn =
            static_cast<int>(ini.integer("SkeletonDebug", "DebugMaxSkeletonBonesDrawn", rockDebugMaxSkeletonBonesDrawn));
        if (rockDebugMaxSkeletonBonesDrawn < 0) {
            rockDebugMaxSkeletonBonesDrawn = 0;
        } else if (rockDebugMaxSkeletonBonesDrawn > 768) {
            rockDebugMaxSkeletonBonesDrawn = 768;
        }
        rockDebugMaxSkeletonBoneAxesDrawn = static_cast<int>(
            ini.integer("SkeletonDebug", "DebugMaxSkeletonBoneAxesDrawn", rockDebugMaxSkeletonBoneAxesDrawn));
        if (rockDebugMaxSkeletonBoneAxesDrawn < 0) {
            rockDebugMaxSkeletonBoneAxesDrawn = 0;
        } else if (rockDebugMaxSkeletonBoneAxesDrawn > 768) {
            rockDebugMaxSkeletonBoneAxesDrawn = 768;
        }
        rockDebugRootFlattenedFingerSkeletonMarkerSize = static_cast<float>(ini.number("SkeletonDebug", "DebugRootFlattenedFingerSkeletonMarkerSize", rockDebugRootFlattenedFingerSkeletonMarkerSize));
        if (rockDebugRootFlattenedFingerSkeletonMarkerSize < 0.1f) {
            rockDebugRootFlattenedFingerSkeletonMarkerSize = 0.1f;
        }
        rockDebugSkeletonBonePointSize = static_cast<float>(
            ini.number("SkeletonDebug", "DebugSkeletonBonePointSize", rockDebugSkeletonBonePointSize));
        if (rockDebugSkeletonBonePointSize < 0.1f) {
            rockDebugSkeletonBonePointSize = 0.1f;
        }
        rockDebugSkeletonBoneAxisLength = static_cast<float>(
            ini.number("SkeletonDebug", "DebugSkeletonBoneAxisLength", rockDebugSkeletonBoneAxisLength));
        if (rockDebugSkeletonBoneAxisLength < 0.1f) {
            rockDebugSkeletonBoneAxisLength = 0.1f;
        }

        rockHandCollisionStaticWorldEnabled =
            ini.boolean("HandAndBodyCollision", "HandCollisionStaticWorldEnabled", rockHandCollisionStaticWorldEnabled);
        rockGlobalSurfaceGrabEnabled =
            ini.boolean("HandAndBodyCollision", "GlobalSurfaceGrabEnabled", rockGlobalSurfaceGrabEnabled);
        rockSurfaceMeshGrabEnabled = ini.boolean("HandAndBodyCollision", "SurfaceMeshGrabEnabled", rockSurfaceMeshGrabEnabled);
        rockSurfaceMeshGrabMaxProjectionDistanceGameUnits = static_cast<float>(ini.number("HandAndBodyCollision", "SurfaceMeshGrabMaxProjectionDistanceGameUnits", rockSurfaceMeshGrabMaxProjectionDistanceGameUnits));
        if (!std::isfinite(rockSurfaceMeshGrabMaxProjectionDistanceGameUnits)) {
            rockSurfaceMeshGrabMaxProjectionDistanceGameUnits = 48.0f;
        }
        rockSurfaceMeshGrabMaxProjectionDistanceGameUnits =
            std::clamp(rockSurfaceMeshGrabMaxProjectionDistanceGameUnits, 1.0f, 128.0f);
        rockSurfaceMeshGrabMaxTriangles =
            std::clamp(static_cast<int>(
                           ini.integer("HandAndBodyCollision", "SurfaceMeshGrabMaxTriangles", rockSurfaceMeshGrabMaxTriangles)),
                256, 100000);
        rockHandBoneColliderRadiusScaleOverrides = ini.text("HandAndBodyCollision", "HandBoneColliderRadiusScaleOverrides",
            rockHandBoneColliderRadiusScaleOverrides.c_str());
        rockHandPalmColliderDimensionScaleOverrides = ini.text("HandAndBodyCollision", "HandPalmColliderDimensionScaleOverrides",
            rockHandPalmColliderDimensionScaleOverrides.c_str());
        rockHandBoneCollidersRequirePalmAnchor =
            ini.boolean("HandAndBodyCollision", "HandBoneCollidersRequirePalmAnchor", rockHandBoneCollidersRequirePalmAnchor);
        rockHandBoneColliderMaxLinearVelocity = static_cast<float>(
            ini.number("HandAndBodyCollision", "HandBoneColliderMaxLinearVelocity", rockHandBoneColliderMaxLinearVelocity));
        rockHandBoneColliderMaxAngularVelocity = static_cast<float>(
            ini.number("HandAndBodyCollision", "HandBoneColliderMaxAngularVelocity", rockHandBoneColliderMaxAngularVelocity));
        if (!std::isfinite(rockHandBoneColliderMaxLinearVelocity) || rockHandBoneColliderMaxLinearVelocity <= 0.0f) {
            rockHandBoneColliderMaxLinearVelocity = 800.0f;
        }
        if (!std::isfinite(rockHandBoneColliderMaxAngularVelocity) || rockHandBoneColliderMaxAngularVelocity <= 0.0f) {
            rockHandBoneColliderMaxAngularVelocity = 800.0f;
        }

        rockObjectPhysicsTreeMaxDepth =
            static_cast<int>(ini.integer("SelectionAndPulling", "ObjectPhysicsTreeMaxDepth", rockObjectPhysicsTreeMaxDepth));

        rockGrabLinearTau = static_cast<float>(ini.number("GrabDrive", "GrabLinearTau", rockGrabLinearTau));
        rockGrabLinearDamping =
            static_cast<float>(ini.number("GrabDrive", "GrabLinearDamping", rockGrabLinearDamping));
        rockGrabLinearProportionalRecovery = static_cast<float>(
            ini.number("GrabDrive", "GrabLinearProportionalRecovery", rockGrabLinearProportionalRecovery));
        rockGrabLinearConstantRecovery = static_cast<float>(
            ini.number("GrabDrive", "GrabLinearConstantRecovery", rockGrabLinearConstantRecovery));

        rockGrabAngularTau = static_cast<float>(ini.number("GrabDrive", "GrabAngularTau", rockGrabAngularTau));
        rockGrabAngularDamping =
            static_cast<float>(ini.number("GrabDrive", "GrabAngularDamping", rockGrabAngularDamping));
        rockGrabAngularProportionalRecovery = static_cast<float>(
            ini.number("GrabDrive", "GrabAngularProportionalRecovery", rockGrabAngularProportionalRecovery));
        rockGrabAngularConstantRecovery = static_cast<float>(
            ini.number("GrabDrive", "GrabAngularConstantRecovery", rockGrabAngularConstantRecovery));

        rockGrabConstraintMaxForce =
            static_cast<float>(ini.number("GrabDrive", "GrabConstraintMaxForce", rockGrabConstraintMaxForce));
        rockGrabMaxForceToMassRatio =
            static_cast<float>(ini.number("GrabDrive", "GrabMaxForceToMassRatio", rockGrabMaxForceToMassRatio));
        rockGrabFreeLinearAcceleration = readClampedFloat(ini, "GrabDrive", "GrabFreeLinearAcceleration",
            rockGrabFreeLinearAcceleration, 1000.0f, 1.0f, 10000.0f);
        rockGrabFreeAngularAcceleration = readClampedFloat(ini, "GrabDrive", "GrabFreeAngularAcceleration",
            rockGrabFreeAngularAcceleration, 6000.0f, 1.0f, 60000.0f);
        rockForceGrabAttachSettleSeconds = readClampedFloat(ini, "GrabDrive", "ForceGrabAttachSettleSeconds",
            rockForceGrabAttachSettleSeconds, 0.10f, 0.0f, 1.0f);
        rockGrabEffectiveMotorMassFloorEnabled =
            ini.boolean("GrabDrive", "GrabEffectiveMotorMassFloorEnabled", rockGrabEffectiveMotorMassFloorEnabled);
        rockGrabEffectiveMotorMassFloor = readClampedFloat(ini, "GrabDrive", "GrabEffectiveMotorMassFloor",
            rockGrabEffectiveMotorMassFloor, kDefaultGrabEffectiveMotorMassFloor, 0.0f, 100.0f);
        rockGrabForceFadeInTime =
            static_cast<float>(ini.number("GrabDrive", "GrabForceFadeInTime", rockGrabForceFadeInTime));
        rockRightGrabAuthorityProxyOffsetGameUnits = ini.float3("PalmAndGrabOffsets",
            "RightGrabAuthorityProxyOffsetGameUnits", rockRightGrabAuthorityProxyOffsetGameUnits);
        rockLeftGrabAuthorityProxyOffsetGameUnits = ini.float3("PalmAndGrabOffsets",
            "LeftGrabAuthorityProxyOffsetGameUnits", rockLeftGrabAuthorityProxyOffsetGameUnits);
        rockGrabLooseWeaponSharedConstraintLinearTauMultiplier =
            readClampedFloat(ini, "GrabDrive", "GrabLooseWeaponSharedConstraintLinearTauMultiplier",
                rockGrabLooseWeaponSharedConstraintLinearTauMultiplier,
                kDefaultGrabLooseWeaponSharedConstraintLinearTauMultiplier, 0.05f, 4.0f);
        rockGrabLooseWeaponSharedConstraintAngularTauMultiplier =
            readClampedFloat(ini, "GrabDrive", "GrabLooseWeaponSharedConstraintAngularTauMultiplier",
                rockGrabLooseWeaponSharedConstraintAngularTauMultiplier,
                kDefaultGrabLooseWeaponSharedConstraintAngularTauMultiplier, 0.05f, 4.0f);
        rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier =
            readClampedFloat(ini, "GrabDrive", "GrabLooseWeaponSharedConstraintCollisionTauMultiplier",
                rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier,
                kDefaultGrabLooseWeaponSharedConstraintCollisionTauMultiplier, 0.05f, 4.0f);
        rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier =
            readClampedFloat(ini, "GrabDrive", "GrabLooseWeaponSharedConstraintLinearDampingMultiplier",
                rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier,
                kDefaultGrabLooseWeaponSharedConstraintLinearDampingMultiplier, 0.05f, 4.0f);
        rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier =
            readClampedFloat(ini, "GrabDrive", "GrabLooseWeaponSharedConstraintAngularDampingMultiplier",
                rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier,
                kDefaultGrabLooseWeaponSharedConstraintAngularDampingMultiplier, 0.05f, 4.0f);
        rockGrabLooseWeaponSharedConstraintMaxForceMultiplier = readClampedFloat(ini, "GrabDrive", "GrabLooseWeaponSharedConstraintMaxForceMultiplier", rockGrabLooseWeaponSharedConstraintMaxForceMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintMaxForceMultiplier, 0.05f, 8.0f);
        rockGrabLooseWeaponSharedConstraintAngularForceMultiplier =
            readClampedFloat(ini, "GrabDrive", "GrabLooseWeaponSharedConstraintAngularForceMultiplier",
                rockGrabLooseWeaponSharedConstraintAngularForceMultiplier,
                kDefaultGrabLooseWeaponSharedConstraintAngularForceMultiplier, 0.05f, 8.0f);
        rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier =
            readClampedFloat(ini, "GrabDrive", "GrabLooseWeaponSharedConstraintLinearRecoveryMultiplier",
                rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier,
                kDefaultGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier, 0.05f, 4.0f);
        rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier =
            readClampedFloat(ini, "GrabDrive", "GrabLooseWeaponSharedConstraintAngularRecoveryMultiplier",
                rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier,
                kDefaultGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier, 0.05f, 4.0f);

        rockGrabTauMin = static_cast<float>(ini.number("GrabDrive", "GrabTauMin", rockGrabTauMin));
        rockGrabTauLerpSpeed =
            static_cast<float>(ini.number("GrabDrive", "GrabTauLerpSpeed", rockGrabTauLerpSpeed));
        rockGrabLongObjectAngularScalingEnabled =
            ini.boolean("GrabDrive", "GrabLongObjectAngularScalingEnabled", rockGrabLongObjectAngularScalingEnabled);
        rockGrabLongObjectReferenceLeverGameUnits = static_cast<float>(ini.number("GrabDrive", "GrabLongObjectReferenceLeverGameUnits", rockGrabLongObjectReferenceLeverGameUnits));
        rockGrabLongObjectReferenceLeverGameUnits = std::clamp(std::isfinite(rockGrabLongObjectReferenceLeverGameUnits)
                ? rockGrabLongObjectReferenceLeverGameUnits
                : kDefaultGrabLongObjectReferenceLeverGameUnits,
            1.0f, 240.0f);
        rockGrabLongObjectMinAngularScale = static_cast<float>(
            ini.number("GrabDrive", "GrabLongObjectMinAngularScale", rockGrabLongObjectMinAngularScale));
        rockGrabLongObjectMinAngularScale =
            std::clamp(std::isfinite(rockGrabLongObjectMinAngularScale) ? rockGrabLongObjectMinAngularScale
                                                                        : kDefaultGrabLongObjectMinAngularScale,
                0.05f, 1.0f);

        rockGrabMaxInertiaRatio =
            static_cast<float>(ini.number("GrabDrive", "GrabMaxInertiaRatio", rockGrabMaxInertiaRatio));
        rockGrabMinInertia = readClampedFloat(ini, "GrabDrive", "GrabMinInertia", rockGrabMinInertia,
            kDefaultGrabMinInertia, 0.0001f, 100.0f);

        rockThrowVelocityMultiplier =
            static_cast<float>(ini.number("GrabSafetyAndRelease", "ThrowVelocityMultiplier", rockThrowVelocityMultiplier));
        rockGrabControllerDerivedThrowVelocityEnabled = ini.boolean("GrabSafetyAndRelease", "GrabControllerDerivedThrowVelocityEnabled", rockGrabControllerDerivedThrowVelocityEnabled);
        rockGrabThrowMaxVelocityHavok = static_cast<float>(
            ini.number("GrabSafetyAndRelease", "GrabThrowMaxVelocityHavok", rockGrabThrowMaxVelocityHavok));
        rockGrabThrowMaxVelocityHavok =
            std::clamp(std::isfinite(rockGrabThrowMaxVelocityHavok) ? rockGrabThrowMaxVelocityHavok
                                                                    : kDefaultGrabThrowMaxVelocityHavok,
                1.0f, 60.0f);
        rockGrabThrowAngularVelocityScale = static_cast<float>(
            ini.number("GrabSafetyAndRelease", "GrabThrowAngularVelocityScale", rockGrabThrowAngularVelocityScale));
        rockGrabThrowAngularVelocityScale =
            std::clamp(std::isfinite(rockGrabThrowAngularVelocityScale) ? rockGrabThrowAngularVelocityScale
                                                                        : kDefaultGrabThrowAngularVelocityScale,
                0.0f, 2.0f);
        rockGrabThrowMaxAngularVelocityRadiansPerSecond = static_cast<float>(ini.number("GrabSafetyAndRelease", "GrabThrowMaxAngularVelocityRadiansPerSecond", rockGrabThrowMaxAngularVelocityRadiansPerSecond));
        rockGrabThrowMaxAngularVelocityRadiansPerSecond =
            std::clamp(std::isfinite(rockGrabThrowMaxAngularVelocityRadiansPerSecond)
                    ? rockGrabThrowMaxAngularVelocityRadiansPerSecond
                    : kDefaultGrabThrowMaxAngularVelocityRadiansPerSecond,
                0.0f, 60.0f);
        rockGrabReleaseHandCollisionDelaySeconds =
            rock::hand_collision_suppression_math::sanitizeDelaySeconds(static_cast<float>(ini.number("GrabSafetyAndRelease", "GrabReleaseHandCollisionDelaySeconds", rockGrabReleaseHandCollisionDelaySeconds)));
        rockGrabHeldMassMovementSlowdownEnabled =
            ini.boolean("GrabSafetyAndRelease", "GrabHeldMassMovementSlowdownEnabled", rockGrabHeldMassMovementSlowdownEnabled);
        rockGrabHeldMassMovementMassProportion = readClampedFloat(ini, "GrabSafetyAndRelease", "GrabHeldMassMovementMassProportion",
            rockGrabHeldMassMovementMassProportion, 0.675f, 0.0f, 10.0f);
        rockGrabHeldMassMovementMassExponent = readClampedFloat(ini, "GrabSafetyAndRelease", "GrabHeldMassMovementMassExponent",
            rockGrabHeldMassMovementMassExponent, 1.0f, 0.0f, 4.0f);
        rockGrabHeldMassMovementMaxReduction = readClampedFloat(ini, "GrabSafetyAndRelease", "GrabHeldMassMovementMaxReduction",
            rockGrabHeldMassMovementMaxReduction, 75.0f, 0.0f, 99.0f);
        rockGrabHeldMassMovementFadeOutSeconds = readClampedFloat(ini, "GrabSafetyAndRelease", "GrabHeldMassMovementFadeOutSeconds",
            rockGrabHeldMassMovementFadeOutSeconds, 5.0f, 0.0f, 60.0f);
        rockGrabTouchAcquireDistanceGameUnits = static_cast<float>(
            ini.number("GrabAcquisition", "GrabTouchAcquireDistanceGameUnits", rockGrabTouchAcquireDistanceGameUnits));
        if (!std::isfinite(rockGrabTouchAcquireDistanceGameUnits) || rockGrabTouchAcquireDistanceGameUnits <= 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabTouchAcquireDistanceGameUnits={} -- using 4.0",
                rockGrabTouchAcquireDistanceGameUnits);
            rockGrabTouchAcquireDistanceGameUnits = 4.0f;
        }
        rockGrabNearConvergeDistanceGameUnits = static_cast<float>(
            ini.number("GrabAcquisition", "GrabNearConvergeDistanceGameUnits", rockGrabNearConvergeDistanceGameUnits));
        if (!std::isfinite(rockGrabNearConvergeDistanceGameUnits) ||
            rockGrabNearConvergeDistanceGameUnits < rockGrabTouchAcquireDistanceGameUnits) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabNearConvergeDistanceGameUnits={} -- using touch distance {}",
                rockGrabNearConvergeDistanceGameUnits, rockGrabTouchAcquireDistanceGameUnits);
            rockGrabNearConvergeDistanceGameUnits = rockGrabTouchAcquireDistanceGameUnits;
        }
        rockGrabPocketDepthGameUnits =
            static_cast<float>(ini.number("GrabAcquisition", "GrabPocketDepthGameUnits", rockGrabPocketDepthGameUnits));
        if (!std::isfinite(rockGrabPocketDepthGameUnits) || rockGrabPocketDepthGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabPocketDepthGameUnits={} -- using 7.0", rockGrabPocketDepthGameUnits);
            rockGrabPocketDepthGameUnits = 7.0f;
        }
        rockGrabPocketRadiusGameUnits = static_cast<float>(
            ini.number("GrabAcquisition", "GrabPocketRadiusGameUnits", rockGrabPocketRadiusGameUnits));
        if (!std::isfinite(rockGrabPocketRadiusGameUnits) || rockGrabPocketRadiusGameUnits <= 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabPocketRadiusGameUnits={} -- using 9.0", rockGrabPocketRadiusGameUnits);
            rockGrabPocketRadiusGameUnits = 9.0f;
        }
        // Seat depth stop: 0 disables the correction entirely.
        rockGrabSeatDepthMaxGameUnits = static_cast<float>(
            ini.number("GrabAcquisition", "GrabSeatDepthMaxGameUnits", rockGrabSeatDepthMaxGameUnits));
        if (!std::isfinite(rockGrabSeatDepthMaxGameUnits) || rockGrabSeatDepthMaxGameUnits < 0.0f ||
            rockGrabSeatDepthMaxGameUnits > 100.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabSeatDepthMaxGameUnits={} -- using 30.0", rockGrabSeatDepthMaxGameUnits);
            rockGrabSeatDepthMaxGameUnits = 30.0f;
        }
        rockGrabSeatDepthFootprintRadiusGameUnits = static_cast<float>(ini.number("GrabAcquisition", "GrabSeatDepthFootprintRadiusGameUnits", rockGrabSeatDepthFootprintRadiusGameUnits));
        if (!std::isfinite(rockGrabSeatDepthFootprintRadiusGameUnits) ||
            rockGrabSeatDepthFootprintRadiusGameUnits < 1.0f || rockGrabSeatDepthFootprintRadiusGameUnits > 30.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabSeatDepthFootprintRadiusGameUnits={} -- using 10.0",
                rockGrabSeatDepthFootprintRadiusGameUnits);
            rockGrabSeatDepthFootprintRadiusGameUnits = 10.0f;
        }
        rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits =
            static_cast<float>(ini.number("GrabAcquisition", "GrabSeatPenetrationBackstopFootprintRadiusGameUnits",
                rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits));
        if (!std::isfinite(rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits) ||
            rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits < 1.0f ||
            rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits > 30.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabSeatPenetrationBackstopFootprintRadiusGameUnits={} -- using 6.0",
                rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits);
            rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits = 6.0f;
        }
        rockGrabSeatDepthSkinGameUnits = static_cast<float>(
            ini.number("GrabAcquisition", "GrabSeatDepthSkinGameUnits", rockGrabSeatDepthSkinGameUnits));
        if (!std::isfinite(rockGrabSeatDepthSkinGameUnits) || rockGrabSeatDepthSkinGameUnits < 0.0f ||
            rockGrabSeatDepthSkinGameUnits > 5.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabSeatDepthSkinGameUnits={} -- using 0.5",
                rockGrabSeatDepthSkinGameUnits);
            rockGrabSeatDepthSkinGameUnits = 0.5f;
        }
        rockGrabCloseSeatMode =
            static_cast<int>(ini.integer("GrabAcquisition", "GrabCloseSeatMode", rockGrabCloseSeatMode));
        if (rockGrabCloseSeatMode != 0 && rockGrabCloseSeatMode != 1) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabCloseSeatMode={} -- using 0", rockGrabCloseSeatMode);
            rockGrabCloseSeatMode = 0;
        }
        rockGrabGripInsetGameUnits =
            static_cast<float>(ini.number("GrabAcquisition", "GrabGripInsetGameUnits", rockGrabGripInsetGameUnits));
        if (!std::isfinite(rockGrabGripInsetGameUnits) || rockGrabGripInsetGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabGripInsetGameUnits={} -- using 2.0", rockGrabGripInsetGameUnits);
            rockGrabGripInsetGameUnits = 2.0f;
        }
        rockGrabConvergeMaxTimeSeconds = static_cast<float>(
            ini.number("GrabAcquisition", "GrabConvergeMaxTimeSeconds", rockGrabConvergeMaxTimeSeconds));
        if (!std::isfinite(rockGrabConvergeMaxTimeSeconds) || rockGrabConvergeMaxTimeSeconds < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabConvergeMaxTimeSeconds={} -- using 0.35",
                rockGrabConvergeMaxTimeSeconds);
            rockGrabConvergeMaxTimeSeconds = 0.35f;
        }
        rockGrabConvergeStableSeconds = static_cast<float>(
            ini.number("GrabAcquisition", "GrabConvergeStableSeconds", rockGrabConvergeStableSeconds));
        if (!std::isfinite(rockGrabConvergeStableSeconds) || rockGrabConvergeStableSeconds <= 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabConvergeStableSeconds={} -- using {}", rockGrabConvergeStableSeconds,
                0.0333f);
            rockGrabConvergeStableSeconds = 0.0333f;
        }
        // Bounds are the historical 1..12-frame tuning range at 90 Hz.
        rockGrabConvergeStableSeconds = std::clamp(rockGrabConvergeStableSeconds, 0.0111f, 12.0f / 90.0f);
        rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond = static_cast<float>(ini.number("GrabAcquisition", "GrabConvergeMaxSeparatingSpeedGameUnitsPerSecond", rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond));
        if (!std::isfinite(rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond) ||
            rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabConvergeMaxSeparatingSpeedGameUnitsPerSecond={} -- using 40.0",
                rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond);
            rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond = 40.0f;
        }
        rockGrabAcquisitionVisualStartDistanceGameUnits = static_cast<float>(ini.number("GrabAcquisition", "GrabAcquisitionVisualStartDistanceGameUnits", rockGrabAcquisitionVisualStartDistanceGameUnits));
        if (!std::isfinite(rockGrabAcquisitionVisualStartDistanceGameUnits) ||
            rockGrabAcquisitionVisualStartDistanceGameUnits <= 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabAcquisition.GrabAcquisitionVisualStartDistanceGameUnits={} -- using 28.0",
                rockGrabAcquisitionVisualStartDistanceGameUnits);
            rockGrabAcquisitionVisualStartDistanceGameUnits = 28.0f;
        }
        rockGrabAcquisitionVisualStartDistanceGameUnits =
            grab_three_phase::computeAcquisitionVisualEnvelopeGameUnits(rockGrabTouchAcquireDistanceGameUnits,
                rockGrabNearConvergeDistanceGameUnits, rockGrabAcquisitionVisualStartDistanceGameUnits);
        rockGrabSurfaceBehindPalmToleranceGameUnits = static_cast<float>(ini.number("GrabContactEvidence", "GrabSurfaceBehindPalmToleranceGameUnits", rockGrabSurfaceBehindPalmToleranceGameUnits));
        if (!std::isfinite(rockGrabSurfaceBehindPalmToleranceGameUnits) ||
            rockGrabSurfaceBehindPalmToleranceGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid GrabContactEvidence.GrabSurfaceBehindPalmToleranceGameUnits={} -- using 1.5",
                rockGrabSurfaceBehindPalmToleranceGameUnits);
            rockGrabSurfaceBehindPalmToleranceGameUnits = 1.5f;
        }
        rockGrabOppositionContactMaxAgeSeconds = static_cast<float>(
            ini.number("GrabContactEvidence", "GrabOppositionContactMaxAgeSeconds", rockGrabOppositionContactMaxAgeSeconds));
        rockGrabOppositionContactMaxAgeSeconds = std::isfinite(rockGrabOppositionContactMaxAgeSeconds)
            ? std::clamp(rockGrabOppositionContactMaxAgeSeconds, 0.0f, 60.0f / 90.0f)
            : 0.0556f;
        rockGrabPinchPocketEnabled = ini.boolean("GrabContactEvidence", "GrabPinchPocketEnabled", rockGrabPinchPocketEnabled);
        rockGrabPinchCloseSelectionEnabled =
            ini.boolean("GrabContactEvidence", "GrabPinchCloseSelectionEnabled", rockGrabPinchCloseSelectionEnabled);
        rockGrabPinchMaxVolumeCubicGameUnits = readClampedFloat(ini, "GrabContactEvidence", "GrabPinchMaxVolumeCubicGameUnits",
            rockGrabPinchMaxVolumeCubicGameUnits, grab_pinch_pocket_policy::kDefaultMaxVolumeCubicGameUnits, 0.001f,
            grab_pinch_pocket_policy::kMaxVolumeCubicGameUnits);
        rockGrabPinchMaxPocketDistanceGameUnits = readClampedFloat(ini, "GrabContactEvidence", "GrabPinchMaxPocketDistanceGameUnits",
            rockGrabPinchMaxPocketDistanceGameUnits, grab_pinch_pocket_policy::kDefaultMaxPocketDistanceGameUnits, 0.1f,
            80.0f);
        rockGrabPinchMinFingerGapGameUnits = readClampedFloat(ini, "GrabContactEvidence", "GrabPinchMinFingerGapGameUnits",
            rockGrabPinchMinFingerGapGameUnits, grab_pinch_pocket_policy::kDefaultMinFingerGapGameUnits, 0.0f, 40.0f);
        rockGrabPinchMaxFingerGapGameUnits = readClampedFloat(ini, "GrabContactEvidence", "GrabPinchMaxFingerGapGameUnits",
            rockGrabPinchMaxFingerGapGameUnits, grab_pinch_pocket_policy::kDefaultMaxFingerGapGameUnits, 0.1f, 80.0f);
        if (rockGrabPinchMaxFingerGapGameUnits < rockGrabPinchMinFingerGapGameUnits) {
            rockGrabPinchMaxFingerGapGameUnits = rockGrabPinchMinFingerGapGameUnits;
        }
        rockGrabPinchThumbIndexMaxOpenValue = readClampedFloat(ini, "GrabContactEvidence", "GrabPinchThumbIndexMaxOpenValue",
            rockGrabPinchThumbIndexMaxOpenValue, grab_pinch_pocket_policy::kDefaultThumbIndexMaxOpenValue, 0.0f, 1.0f);
        rockGrabPinchOtherFingerCurlValue = readClampedFloat(ini, "GrabContactEvidence", "GrabPinchOtherFingerCurlValue",
            rockGrabPinchOtherFingerCurlValue, grab_pinch_pocket_policy::kDefaultOtherFingerCurlValue, 0.0f, 1.0f);

        rockGrabPinchDetectionDirectionHandspace = ini.float3("GrabContactEvidence",
            "GrabPinchDetectionDirectionHandspace", rockGrabPinchDetectionDirectionHandspace);
        rockGrabPinchDetectionAxisBlend = readClampedFloat(ini, "GrabContactEvidence", "GrabPinchDetectionAxisBlend",
            rockGrabPinchDetectionAxisBlend, grab_pinch_pocket_policy::kDefaultDetectionAxisBlend, 0.0f, 1.0f);
        {
            auto pinchDetectionConfig = grab_pinch_pocket_policy::Config{};
            pinchDetectionConfig.detectionDirectionHandspace = rockGrabPinchDetectionDirectionHandspace;
            pinchDetectionConfig.detectionAxisBlend = rockGrabPinchDetectionAxisBlend;
            const auto sanitizedPinchDetectionConfig = grab_pinch_pocket_policy::sanitizeConfig(pinchDetectionConfig);
            rockGrabPinchDetectionDirectionHandspace = sanitizedPinchDetectionConfig.detectionDirectionHandspace;
            rockGrabPinchDetectionAxisBlend = sanitizedPinchDetectionConfig.detectionAxisBlend;
        }
        rockGrabHandLerpEnabled = ini.boolean("GrabContactEvidence", "GrabHandLerpEnabled", rockGrabHandLerpEnabled);
        rockGrabHandLerpTimeMin =
            readClampedFloat(ini, "GrabContactEvidence", "GrabHandLerpTimeMin", rockGrabHandLerpTimeMin, 0.10f, 0.0f, 1.0f);
        rockGrabHandLerpTimeMax = readClampedFloat(ini, "GrabContactEvidence", "GrabHandLerpTimeMax", rockGrabHandLerpTimeMax, 0.20f,
            rockGrabHandLerpTimeMin, 1.0f);
        rockGrabHandLerpMinDistance =
            readClampedFloat(ini, "GrabContactEvidence", "GrabHandLerpMinDistance", rockGrabHandLerpMinDistance, 7.0f, 0.0f, 80.0f);
        rockGrabHandLerpMaxDistance = readClampedFloat(ini, "GrabContactEvidence", "GrabHandLerpMaxDistance",
            rockGrabHandLerpMaxDistance, 14.0f, rockGrabHandLerpMinDistance, 120.0f);
        rockGrabHandReturnEnabled = ini.boolean("GrabContactEvidence", "GrabHandReturnEnabled", rockGrabHandReturnEnabled);
        rockGrabHandReturnTimeMin =
            readClampedFloat(ini, "GrabContactEvidence", "GrabHandReturnTimeMin", rockGrabHandReturnTimeMin, 0.10f, 0.0f, 1.0f);
        rockGrabHandReturnTimeMax = readClampedFloat(ini, "GrabContactEvidence", "GrabHandReturnTimeMax", rockGrabHandReturnTimeMax,
            0.20f, rockGrabHandReturnTimeMin, 1.0f);
        rockGrabHandReturnMinDistance = readClampedFloat(ini, "GrabContactEvidence", "GrabHandReturnMinDistance",
            rockGrabHandReturnMinDistance, 7.0f, 0.0f, 80.0f);
        rockGrabHandReturnMaxDistance = readClampedFloat(ini, "GrabContactEvidence", "GrabHandReturnMaxDistance",
            rockGrabHandReturnMaxDistance, 14.0f, rockGrabHandReturnMinDistance, 120.0f);
        rockGrabHandReturnMinAngleDegrees = readClampedFloat(ini, "GrabContactEvidence", "GrabHandReturnMinAngleDegrees",
            rockGrabHandReturnMinAngleDegrees, 5.0f, 0.0f, 180.0f);
        rockGrabHandReturnMaxAngleDegrees = readClampedFloat(ini, "GrabContactEvidence", "GrabHandReturnMaxAngleDegrees",
            rockGrabHandReturnMaxAngleDegrees, 90.0f, rockGrabHandReturnMinAngleDegrees, 180.0f);
        rockGrabFingerPoseMode =
            static_cast<int>(ini.integer("FingerPoses", "GrabFingerPoseMode", rockGrabFingerPoseMode));
        if (rockGrabFingerPoseMode != 1 && rockGrabFingerPoseMode != 2) {
            ROCK_LOG_WARN(Config, "Invalid FingerPoses.GrabFingerPoseMode={} -- using 1", rockGrabFingerPoseMode);
            rockGrabFingerPoseMode = 1;
        }
        rockWeaponGrabPoseMode =
            static_cast<int>(ini.integer("FingerPoses", "WeaponGrabPoseMode", rockWeaponGrabPoseMode));
        if (rockWeaponGrabPoseMode < 0 || rockWeaponGrabPoseMode > 2) {
            ROCK_LOG_WARN(Config, "Invalid FingerPoses.WeaponGrabPoseMode={} -- using 0", rockWeaponGrabPoseMode);
            rockWeaponGrabPoseMode = 0;
        }
        rockLooseGrabFingerPoseMode =
            static_cast<int>(ini.integer("FingerPoses", "LooseGrabFingerPoseMode", rockLooseGrabFingerPoseMode));
        if (rockLooseGrabFingerPoseMode < 0 || rockLooseGrabFingerPoseMode > 1) {
            ROCK_LOG_WARN(Config, "Invalid FingerPoses.LooseGrabFingerPoseMode={} -- using 0", rockLooseGrabFingerPoseMode);
            rockLooseGrabFingerPoseMode = 0;
        }
        rockGrabMeshFingerPoseEnabled =
            ini.boolean("FingerPoses", "GrabMeshFingerPoseEnabled", rockGrabMeshFingerPoseEnabled);
        rockGrabMeshJointPoseEnabled =
            ini.boolean("FingerPoses", "GrabMeshJointPoseEnabled", rockGrabMeshJointPoseEnabled);
        rockGrabFingerMinValue =
            static_cast<float>(ini.number("FingerPoses", "GrabFingerMinValue", rockGrabFingerMinValue));
        if (!std::isfinite(rockGrabFingerMinValue)) {
            ROCK_LOG_WARN(Config, "Invalid FingerPoses.GrabFingerMinValue={} -- using 0.2", rockGrabFingerMinValue);
            rockGrabFingerMinValue = 0.2f;
        }
        rockGrabFingerMinValue = std::clamp(rockGrabFingerMinValue, 0.0f, 1.0f);
        rockGrabFingerPoseSmoothingSpeed = static_cast<float>(
            ini.number("FingerPoses", "GrabFingerPoseSmoothingSpeed", rockGrabFingerPoseSmoothingSpeed));
        if (!std::isfinite(rockGrabFingerPoseSmoothingSpeed) || rockGrabFingerPoseSmoothingSpeed < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid FingerPoses.GrabFingerPoseSmoothingSpeed={} -- using 14.0",
                rockGrabFingerPoseSmoothingSpeed);
            rockGrabFingerPoseSmoothingSpeed = 14.0f;
        }
        rockGrabMeshLocalTransformPoseEnabled =
            ini.boolean("FingerPoses", "GrabMeshLocalTransformPoseEnabled", rockGrabMeshLocalTransformPoseEnabled);
        rockGrabFingerLocalTransformSmoothingSpeed = static_cast<float>(ini.number("FingerPoses", "GrabFingerLocalTransformSmoothingSpeed", rockGrabFingerLocalTransformSmoothingSpeed));
        if (!std::isfinite(rockGrabFingerLocalTransformSmoothingSpeed) ||
            rockGrabFingerLocalTransformSmoothingSpeed < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid FingerPoses.GrabFingerLocalTransformSmoothingSpeed={} -- using 14.0",
                rockGrabFingerLocalTransformSmoothingSpeed);
            rockGrabFingerLocalTransformSmoothingSpeed = 14.0f;
        }
        rockGrabFingerLocalTransformMaxCorrectionDegrees = static_cast<float>(ini.number("FingerPoses", "GrabFingerLocalTransformMaxCorrectionDegrees", rockGrabFingerLocalTransformMaxCorrectionDegrees));
        if (!std::isfinite(rockGrabFingerLocalTransformMaxCorrectionDegrees) ||
            rockGrabFingerLocalTransformMaxCorrectionDegrees < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid FingerPoses.GrabFingerLocalTransformMaxCorrectionDegrees={} -- using 35.0",
                rockGrabFingerLocalTransformMaxCorrectionDegrees);
            rockGrabFingerLocalTransformMaxCorrectionDegrees = 35.0f;
        }
        rockGrabFingerSurfaceAimStrength = static_cast<float>(
            ini.number("FingerPoses", "GrabFingerSurfaceAimStrength", rockGrabFingerSurfaceAimStrength));
        rockGrabFingerSurfaceAimStrength = std::clamp(
            std::isfinite(rockGrabFingerSurfaceAimStrength) ? rockGrabFingerSurfaceAimStrength : 0.75f, 0.0f, 1.0f);
        rockGrabFingerRejectBacksideHits =
            ini.boolean("FingerPoses", "GrabFingerRejectBacksideHits", rockGrabFingerRejectBacksideHits);
        rockGrabFingerSurfacePlaneToleranceGameUnits = static_cast<float>(ini.number("FingerPoses", "GrabFingerSurfacePlaneToleranceGameUnits", rockGrabFingerSurfacePlaneToleranceGameUnits));
        if (!std::isfinite(rockGrabFingerSurfacePlaneToleranceGameUnits) ||
            rockGrabFingerSurfacePlaneToleranceGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid FingerPoses.GrabFingerSurfacePlaneToleranceGameUnits={} -- using 1.5",
                rockGrabFingerSurfacePlaneToleranceGameUnits);
            rockGrabFingerSurfacePlaneToleranceGameUnits = 1.5f;
        }
        rockGrabFingerSweepContactRadiusGameUnits = static_cast<float>(ini.number("FingerPoses", "GrabFingerSweepContactRadiusGameUnits", rockGrabFingerSweepContactRadiusGameUnits));
        if (!std::isfinite(rockGrabFingerSweepContactRadiusGameUnits) ||
            rockGrabFingerSweepContactRadiusGameUnits <= 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid FingerPoses.GrabFingerSweepContactRadiusGameUnits={} -- using 0.6",
                rockGrabFingerSweepContactRadiusGameUnits);
            rockGrabFingerSweepContactRadiusGameUnits = 0.6f;
        }
        rockGrabFingerSweepContactRadiusGameUnits = std::clamp(rockGrabFingerSweepContactRadiusGameUnits, 0.05f, 4.0f);
        rockGrabFingerSweepMaxOpenValue = static_cast<float>(
            ini.number("FingerPoses", "GrabFingerSweepMaxOpenValue", rockGrabFingerSweepMaxOpenValue));
        if (!std::isfinite(rockGrabFingerSweepMaxOpenValue) || rockGrabFingerSweepMaxOpenValue < 1.0f ||
            rockGrabFingerSweepMaxOpenValue > 2.0f) {
            ROCK_LOG_WARN(Config, "Invalid FingerPoses.GrabFingerSweepMaxOpenValue={} -- using 2.0 (valid range 1.0-2.0)",
                rockGrabFingerSweepMaxOpenValue);
            rockGrabFingerSweepMaxOpenValue = 2.0f;
        }
        rockGrabThumbSweepMaxOpenValue = static_cast<float>(
            ini.number("FingerPoses", "GrabThumbSweepMaxOpenValue", rockGrabThumbSweepMaxOpenValue));
        if (!std::isfinite(rockGrabThumbSweepMaxOpenValue) || rockGrabThumbSweepMaxOpenValue < 1.0f ||
            rockGrabThumbSweepMaxOpenValue > 2.0f) {
            ROCK_LOG_WARN(Config, "Invalid FingerPoses.GrabThumbSweepMaxOpenValue={} -- using 1.5 (valid range 1.0-2.0)",
                rockGrabThumbSweepMaxOpenValue);
            rockGrabThumbSweepMaxOpenValue = 1.5f;
        }
        rockGrabThumbOppositionStrength = static_cast<float>(
            ini.number("FingerPoses", "GrabThumbOppositionStrength", rockGrabThumbOppositionStrength));
        rockGrabThumbOppositionStrength = std::clamp(
            std::isfinite(rockGrabThumbOppositionStrength) ? rockGrabThumbOppositionStrength : 1.0f, 0.0f, 1.0f);
        rockGrabThumbAlternateCurveStrength = static_cast<float>(
            ini.number("FingerPoses", "GrabThumbAlternateCurveStrength", rockGrabThumbAlternateCurveStrength));
        rockGrabThumbAlternateCurveStrength =
            std::clamp(std::isfinite(rockGrabThumbAlternateCurveStrength) ? rockGrabThumbAlternateCurveStrength : 0.65f,
                0.0f, 1.0f);
        rockGrabThumbSurfaceSafetyEnabled =
            ini.boolean("FingerPoses", "GrabThumbSurfaceSafetyEnabled", rockGrabThumbSurfaceSafetyEnabled);
        rockGrabThumbSurfaceSafetyMarginGameUnits = static_cast<float>(ini.number("FingerPoses", "GrabThumbSurfaceSafetyMarginGameUnits", rockGrabThumbSurfaceSafetyMarginGameUnits));
        rockGrabThumbSurfaceSafetyMarginGameUnits = std::clamp(std::isfinite(rockGrabThumbSurfaceSafetyMarginGameUnits)
                ? rockGrabThumbSurfaceSafetyMarginGameUnits
                : kDefaultGrabThumbSurfaceSafetyMarginGameUnits,
            0.0f, 5.0f);
        rockGrabLateralWeight =
            static_cast<float>(ini.number("MeshContact", "GrabLateralWeight", rockGrabLateralWeight));
        rockGrabDirectionalWeight =
            static_cast<float>(ini.number("MeshContact", "GrabDirectionalWeight", rockGrabDirectionalWeight));
        rockGrabMaxTriangleDistance =
            static_cast<float>(ini.number("MeshContact", "GrabMaxTriangleDistance", rockGrabMaxTriangleDistance));
        rockGrabMeshContactOnly = ini.boolean("MeshContact", "GrabMeshContactOnly", rockGrabMeshContactOnly);
        rockGrabContactPatchMaxNormalAngleDegrees = static_cast<float>(ini.number("MeshContact", "GrabContactPatchMaxNormalAngleDegrees", rockGrabContactPatchMaxNormalAngleDegrees));
        rockGrabContactPatchMaxNormalAngleDegrees = std::clamp(rockGrabContactPatchMaxNormalAngleDegrees, 0.0f, 179.0f);
        rockGrabAlignmentMaxSelectionToMeshDistance = static_cast<float>(ini.number("MeshContact", "GrabAlignmentMaxSelectionToMeshDistance", rockGrabAlignmentMaxSelectionToMeshDistance));
        if (!std::isfinite(rockGrabAlignmentMaxSelectionToMeshDistance)) {
            rockGrabAlignmentMaxSelectionToMeshDistance = 8.0f;
        }
        rockSelectedCloseFingerCurlEnabled =
            ini.boolean("CloseHandCurl", "SelectedCloseFingerCurlEnabled", rockSelectedCloseFingerCurlEnabled);
        rockSelectedCloseFingerAnimMaxHandSpeed = static_cast<float>(ini.number("CloseHandCurl", "SelectedCloseFingerAnimMaxHandSpeed", rockSelectedCloseFingerAnimMaxHandSpeed));
        if (!std::isfinite(rockSelectedCloseFingerAnimMaxHandSpeed) || rockSelectedCloseFingerAnimMaxHandSpeed < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid CloseHandCurl.SelectedCloseFingerAnimMaxHandSpeed={} -- using 0.9",
                rockSelectedCloseFingerAnimMaxHandSpeed);
            rockSelectedCloseFingerAnimMaxHandSpeed = 0.9f;
        }
        rockSelectedCloseFingerAnimValue = static_cast<float>(
            ini.number("CloseHandCurl", "SelectedCloseFingerAnimValue", rockSelectedCloseFingerAnimValue));
        if (!std::isfinite(rockSelectedCloseFingerAnimValue)) {
            ROCK_LOG_WARN(Config, "Invalid CloseHandCurl.SelectedCloseFingerAnimValue={} -- using 0.9",
                rockSelectedCloseFingerAnimValue);
            rockSelectedCloseFingerAnimValue = 0.9f;
        }
        rockSelectedCloseFingerAnimValue = std::clamp(rockSelectedCloseFingerAnimValue, 0.0f, 1.0f);
        rockPullToObjectCenterEnabled =
            ini.boolean("SelectionAndPulling", "PullToObjectCenterEnabled", rockPullToObjectCenterEnabled);
        rockPullLongAxisPresentationEnabled =
            ini.boolean("SelectionAndPulling", "PullLongAxisPresentationEnabled", rockPullLongAxisPresentationEnabled);
        rockForceGrabSeatAlignmentEnabled =
            ini.boolean("SelectionAndPulling", "ForceGrabSeatAlignmentEnabled", rockForceGrabSeatAlignmentEnabled);
        rockGrabSeatRollAlignmentEnabled =
            ini.boolean("SelectionAndPulling", "GrabSeatRollAlignmentEnabled", rockGrabSeatRollAlignmentEnabled);
        rockPullPresentationMinElongationRatio = static_cast<float>(
            ini.number("SelectionAndPulling", "PullPresentationMinElongationRatio", rockPullPresentationMinElongationRatio));
        if (!std::isfinite(rockPullPresentationMinElongationRatio) || rockPullPresentationMinElongationRatio < 1.0f) {
            ROCK_LOG_WARN(Config, "Invalid SelectionAndPulling.PullPresentationMinElongationRatio={} -- using 2.0",
                rockPullPresentationMinElongationRatio);
            rockPullPresentationMinElongationRatio = 2.0f;
        }
        rockGrabSeatRollMinSecondElongationRatio = static_cast<float>(ini.number("SelectionAndPulling", "GrabSeatRollMinSecondElongationRatio", rockGrabSeatRollMinSecondElongationRatio));
        if (!std::isfinite(rockGrabSeatRollMinSecondElongationRatio) ||
            rockGrabSeatRollMinSecondElongationRatio < 1.0f || rockGrabSeatRollMinSecondElongationRatio > 10.0f) {
            ROCK_LOG_WARN(Config, "Invalid SelectionAndPulling.GrabSeatRollMinSecondElongationRatio={} -- using 1.25",
                rockGrabSeatRollMinSecondElongationRatio);
            rockGrabSeatRollMinSecondElongationRatio = 1.25f;
        }
        rockPullPresentationAngularGainPerSecond = static_cast<float>(ini.number("SelectionAndPulling", "PullPresentationAngularGainPerSecond", rockPullPresentationAngularGainPerSecond));
        if (!std::isfinite(rockPullPresentationAngularGainPerSecond) ||
            rockPullPresentationAngularGainPerSecond < 0.0f || rockPullPresentationAngularGainPerSecond > 30.0f) {
            ROCK_LOG_WARN(Config, "Invalid SelectionAndPulling.PullPresentationAngularGainPerSecond={} -- using 6.0",
                rockPullPresentationAngularGainPerSecond);
            rockPullPresentationAngularGainPerSecond = 6.0f;
        }
        rockPullPresentationMaxAngularSpeedRadiansPerSecond = static_cast<float>(ini.number("SelectionAndPulling", "PullPresentationMaxAngularSpeedRadiansPerSecond", rockPullPresentationMaxAngularSpeedRadiansPerSecond));
        if (!std::isfinite(rockPullPresentationMaxAngularSpeedRadiansPerSecond) ||
            rockPullPresentationMaxAngularSpeedRadiansPerSecond < 0.0f ||
            rockPullPresentationMaxAngularSpeedRadiansPerSecond > 40.0f) {
            ROCK_LOG_WARN(Config, "Invalid SelectionAndPulling.PullPresentationMaxAngularSpeedRadiansPerSecond={} -- using 8.0",
                rockPullPresentationMaxAngularSpeedRadiansPerSecond);
            rockPullPresentationMaxAngularSpeedRadiansPerSecond = 8.0f;
        }
        rockPullPresentationGripAxisTiltDegrees = static_cast<float>(ini.number("SelectionAndPulling", "PullPresentationGripAxisTiltDegrees", rockPullPresentationGripAxisTiltDegrees));
        if (!std::isfinite(rockPullPresentationGripAxisTiltDegrees) || rockPullPresentationGripAxisTiltDegrees < 0.0f ||
            rockPullPresentationGripAxisTiltDegrees > 45.0f) {
            ROCK_LOG_WARN(Config, "Invalid SelectionAndPulling.PullPresentationGripAxisTiltDegrees={} -- using 15.0",
                rockPullPresentationGripAxisTiltDegrees);
            rockPullPresentationGripAxisTiltDegrees = 15.0f;
        }

        auto readClampedFloat = [&](const char* table, const char* key, float& value, float fallback, float minValue,
                                    float maxValue) {
            value = static_cast<float>(ini.number(table, key, value));
            if (!std::isfinite(value)) {
                ROCK_LOG_WARN(Config, "Invalid {}.{}={} -- using {}", table, key, value, fallback);
                value = fallback;
            }
            value = std::clamp(value, minValue, maxValue);
        };

        rockShoulderStashEnabled = ini.boolean("GrabSafetyAndRelease", "ShoulderStashEnabled", rockShoulderStashEnabled);
        rockEquippedWeaponShoulderStashEnabled =
            ini.boolean("GrabSafetyAndRelease", "EquippedWeaponShoulderStashEnabled", rockEquippedWeaponShoulderStashEnabled);
        rockShoulderStashUseBodyZoneColliders =
            ini.boolean("GrabSafetyAndRelease", "ShoulderStashUseBodyZoneColliders", rockShoulderStashUseBodyZoneColliders);
        rockShoulderStashUseHmdBackVolume =
            ini.boolean("GrabSafetyAndRelease", "ShoulderStashUseHmdBackVolume", rockShoulderStashUseHmdBackVolume);
        readClampedFloat("GrabSafetyAndRelease", "ShoulderStashEnterPaddingGameUnits", rockShoulderStashEnterPaddingGameUnits, 5.0f, 0.0f,
            40.0f);
        readClampedFloat("GrabSafetyAndRelease", "ShoulderStashExitPaddingGameUnits", rockShoulderStashExitPaddingGameUnits, 8.0f, 0.0f,
            60.0f);
        rockShoulderStashRecentContactSeconds = static_cast<float>(
            ini.number("GrabSafetyAndRelease", "ShoulderStashRecentContactSeconds", rockShoulderStashRecentContactSeconds));
        rockShoulderStashRecentContactSeconds = std::isfinite(rockShoulderStashRecentContactSeconds)
            ? std::clamp(rockShoulderStashRecentContactSeconds, 0.0f, 60.0f / 90.0f)
            : 0.0444f;
        rockShoulderStashSustainedContactMissSeconds = static_cast<float>(ini.number("GrabSafetyAndRelease", "ShoulderStashSustainedContactMissSeconds", rockShoulderStashSustainedContactMissSeconds));
        rockShoulderStashSustainedContactMissSeconds = std::isfinite(rockShoulderStashSustainedContactMissSeconds)
            ? std::clamp(rockShoulderStashSustainedContactMissSeconds, 0.0f, 120.0f / 90.0f)
            : 18.0f / 90.0f;
        rockShoulderStashHmdBackRightOffsetGameUnits = ini.float3("GrabSafetyAndRelease",
            "ShoulderStashHmdBackRightOffsetGameUnits", rockShoulderStashHmdBackRightOffsetGameUnits);
        rockShoulderStashHmdBackLeftOffsetGameUnits = ini.float3("GrabSafetyAndRelease",
            "ShoulderStashHmdBackLeftOffsetGameUnits", rockShoulderStashHmdBackLeftOffsetGameUnits);
        readClampedFloat("GrabSafetyAndRelease", "ShoulderStashHmdBackRadiusGameUnits", rockShoulderStashHmdBackRadiusGameUnits, 11.0f, 1.0f,
            80.0f);
        readClampedFloat("GrabSafetyAndRelease", "ShoulderStashHmdBackEnterPaddingGameUnits", rockShoulderStashHmdBackEnterPaddingGameUnits,
            0.0f, 0.0f, 40.0f);
        readClampedFloat("GrabSafetyAndRelease", "ShoulderStashHmdBackExitPaddingGameUnits", rockShoulderStashHmdBackExitPaddingGameUnits,
            2.0f, 0.0f, 60.0f);
        readClampedFloat("GrabSafetyAndRelease", "ShoulderStashHmdBackMinBehindGameUnits", rockShoulderStashHmdBackMinBehindGameUnits, 4.0f,
            0.0f, 40.0f);
        rockShoulderStashShowCollectedNotifications = ini.boolean("GrabSafetyAndRelease", "ShoulderStashShowCollectedNotifications", rockShoulderStashShowCollectedNotifications);
        rockMouthConsumeEnabled = ini.boolean("GrabSafetyAndRelease", "MouthConsumeEnabled", rockMouthConsumeEnabled);
        rockMouthConsumeAllowPoison =
            ini.boolean("GrabSafetyAndRelease", "MouthConsumeAllowPoison", rockMouthConsumeAllowPoison);
        rockMouthConsumeHmdOffsetGameUnits =
            ini.float3("GrabSafetyAndRelease", "MouthConsumeHmdOffsetGameUnits", rockMouthConsumeHmdOffsetGameUnits);
        sanitizeMouthConsumeOffset();
        readClampedFloat("GrabSafetyAndRelease", "MouthConsumeRadiusGameUnits", rockMouthConsumeRadiusGameUnits, 5.5f, 1.0f, 80.0f);
        readClampedFloat("GrabSafetyAndRelease", "MouthConsumeEnterPaddingGameUnits", rockMouthConsumeEnterPaddingGameUnits, 0.0f, 0.0f,
            40.0f);
        readClampedFloat("GrabSafetyAndRelease", "MouthConsumeExitPaddingGameUnits", rockMouthConsumeExitPaddingGameUnits, 1.0f, 0.0f, 60.0f);
        readClampedFloat("GrabSafetyAndRelease", "MouthConsumeMinDwellSeconds", rockMouthConsumeMinDwellSeconds, 0.08f, 0.0f, 1.0f);
        readClampedFloat("GrabSafetyAndRelease", "MouthConsumeMaxSpeedGameUnitsPerSecond", rockMouthConsumeMaxSpeedGameUnitsPerSecond, 120.0f,
            0.0f, 1000.0f);

        rockGrabHapticsEnabled = ini.boolean("Haptics", "GrabHapticsEnabled", rockGrabHapticsEnabled);
        readClampedFloat("Haptics", "GrabHapticDurationSeconds", rockGrabHapticDurationSeconds, 0.055f, 0.0f, 0.2f);
        readClampedFloat("Haptics", "GrabHapticBaseIntensity", rockGrabHapticBaseIntensity, 0.12f, 0.0f, 1.0f);
        readClampedFloat("Haptics", "GrabHapticMaxIntensity", rockGrabHapticMaxIntensity, 0.80f, rockGrabHapticBaseIntensity,
            1.0f);
        readClampedFloat("Haptics", "GrabHapticMassScale", rockGrabHapticMassScale, 0.06f, 0.0f, 1.0f);
        readClampedFloat("Haptics", "GrabHapticMassExponent", rockGrabHapticMassExponent, 0.60f, 0.0f, 2.0f);
        readClampedFloat("Haptics", "PullStartHapticIntensity", rockPullStartHapticIntensity, 0.18f, 0.0f, 1.0f);
        readClampedFloat("Haptics", "PullCatchHapticIntensity", rockPullCatchHapticIntensity, 0.22f, 0.0f, 1.0f);
        readClampedFloat("Haptics", "SelectionLockHapticIntensity", rockSelectionLockHapticIntensity, 0.15f, 0.0f, 1.0f);
        readClampedFloat("Haptics", "SelectionLockReleaseHapticIntensity", rockSelectionLockReleaseHapticIntensity, 0.10f, 0.0f,
            1.0f);
        readClampedFloat("Haptics", "SelectionLockReleaseHapticDurationSeconds", rockSelectionLockReleaseHapticDurationSeconds,
            0.02f, 0.0f, 0.2f);
        rockSurfaceGrabHapticsEnabled =
            ini.boolean("Haptics", "SurfaceGrabHapticsEnabled", rockSurfaceGrabHapticsEnabled);
        readClampedFloat("Haptics", "SurfaceGrabHapticDurationSeconds", rockSurfaceGrabHapticDurationSeconds, 0.075f, 0.0f, 0.2f);
        readClampedFloat("Haptics", "SurfaceGrabHapticIntensity", rockSurfaceGrabHapticIntensity, 0.85f, 0.0f, 1.0f);
        rockHeldImpactHapticsEnabled =
            ini.boolean("Haptics", "HeldImpactHapticsEnabled", rockHeldImpactHapticsEnabled);
        readClampedFloat("Haptics", "HeldImpactHapticDurationSeconds", rockHeldImpactHapticDurationSeconds, 0.035f, 0.0f, 0.2f);
        readClampedFloat("Haptics", "HeldImpactHapticBaseIntensity", rockHeldImpactHapticBaseIntensity, 0.12f, 0.0f, 1.0f);
        readClampedFloat("Haptics", "HeldImpactHapticMaxIntensity", rockHeldImpactHapticMaxIntensity, 0.85f,
            rockHeldImpactHapticBaseIntensity, 1.0f);
        readClampedFloat("Haptics", "HeldImpactHapticSpeedScale", rockHeldImpactHapticSpeedScale, 0.006f, 0.0f, 1.0f);
        readClampedFloat("Haptics", "HeldImpactHapticMassScale", rockHeldImpactHapticMassScale, 0.035f, 0.0f, 1.0f);
        readClampedFloat("Haptics", "HeldImpactHapticMassExponent", rockHeldImpactHapticMassExponent, 0.55f, 0.0f, 2.0f);
        readClampedFloat("Haptics", "HeldImpactHapticMinSpeedGameUnits", rockHeldImpactHapticMinSpeedGameUnits, 8.0f, 0.0f,
            1000.0f);
        readClampedFloat("Haptics", "HeldImpactHapticCooldownSeconds", rockHeldImpactHapticCooldownSeconds, 0.12f, 0.0f, 1.0f);
        readClampedFloat("Haptics", "HeldImpactHapticDampedMultiplier", rockHeldImpactHapticDampedMultiplier, 0.55f, 0.0f, 1.0f);
        rockShoulderStashHapticsEnabled =
            ini.boolean("Haptics", "ShoulderStashHapticsEnabled", rockShoulderStashHapticsEnabled);
        readClampedFloat("Haptics", "ShoulderStashCandidateHapticDurationSeconds",
            rockShoulderStashCandidateHapticDurationSeconds, 0.075f, 0.0f, 0.2f);
        readClampedFloat("Haptics", "ShoulderStashCandidateHapticBaseIntensity", rockShoulderStashCandidateHapticBaseIntensity,
            0.20f, 0.0f, 1.0f);
        readClampedFloat("Haptics", "ShoulderStashCandidateHapticIntensity", rockShoulderStashCandidateHapticIntensity, 0.42f,
            rockShoulderStashCandidateHapticBaseIntensity, 1.0f);
        readClampedFloat("Haptics", "ShoulderStashCandidateHapticIntervalSeconds",
            rockShoulderStashCandidateHapticIntervalSeconds, 0.075f, 0.0f, 2.0f);
        readClampedFloat("Haptics", "ShoulderStashCommitHapticDurationSeconds", rockShoulderStashCommitHapticDurationSeconds,
            0.12f, 0.0f, 0.2f);
        readClampedFloat("Haptics", "ShoulderStashCommitHapticIntensity", rockShoulderStashCommitHapticIntensity, 0.85f, 0.0f,
            1.0f);
        rockMouthConsumeHapticsEnabled =
            ini.boolean("Haptics", "MouthConsumeHapticsEnabled", rockMouthConsumeHapticsEnabled);
        readClampedFloat("Haptics", "MouthConsumeCandidateHapticDurationSeconds", rockMouthConsumeCandidateHapticDurationSeconds,
            0.050f, 0.0f, 0.2f);
        readClampedFloat("Haptics", "MouthConsumeCandidateHapticBaseIntensity", rockMouthConsumeCandidateHapticBaseIntensity,
            0.22f, 0.0f, 1.0f);
        readClampedFloat("Haptics", "MouthConsumeCandidateHapticIntensity", rockMouthConsumeCandidateHapticIntensity, 0.45f,
            rockMouthConsumeCandidateHapticBaseIntensity, 1.0f);
        readClampedFloat("Haptics", "MouthConsumeCandidateHapticIntervalSeconds", rockMouthConsumeCandidateHapticIntervalSeconds,
            0.075f, 0.0f, 2.0f);
        readClampedFloat("Haptics", "MouthConsumeCommitHapticDurationSeconds", rockMouthConsumeCommitHapticDurationSeconds, 0.12f,
            0.0f, 0.2f);
        readClampedFloat("Haptics", "MouthConsumeCommitHapticIntensity", rockMouthConsumeCommitHapticIntensity, 0.85f, 0.0f,
            1.0f);
    }

}
