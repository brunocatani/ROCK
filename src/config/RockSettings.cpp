#include "config/RockSettings.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/input/FunctionBindingText.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <format>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rock::config
{
    namespace
    {
        namespace fb = function_binding_policy;
        namespace binding_text = function_binding_text;
        using RPS::Settings::ConversionSource;
        using RPS::Settings::LegacyKey;
        using RPS::Settings::LegacyKind;

        constexpr LegacyKey kLegacyKeys[] = {
#include "config/LegacyIniMigrationTable.inl"
        };

        // Keys ROCK.toml held before a later version renamed them. Frozen like
        // the INI table: a row leaves only with its option. The firing-hand
        // offsets and the developer [x, y, z] options became transforms, for
        // Mods Config's transform editor. The two scope rotations have no row:
        // their [pitch, yaw, roll] order is not rot's, so they start again at zero.
        constexpr RPS::Settings::RenamedKey kRenamedKeys[] = {
            { "FiringHand.Left", "AimOffsetGameUnits", "FiringHand.Left", "AimOffset",
                RPS::Settings::RenameKind::Float3ToTransformPosition },
            { "FiringHand.Left", "GripOffsetGameUnits", "FiringHand.Left", "GripOffset",
                RPS::Settings::RenameKind::Float3ToTransformPosition },
            { "FiringHand.Right", "SupportGripOffsetGameUnits", "FiringHand.Right", "SupportGripOffset",
                RPS::Settings::RenameKind::Float3ToTransformPosition },
            { "PalmAndGrabOffsets", "RightGrabAuthorityProxyOffsetGameUnits", "PalmAndGrabOffsets",
                "RightGrabAuthorityProxyOffset", RPS::Settings::RenameKind::Float3ToTransformPosition },
            { "PalmAndGrabOffsets", "LeftGrabAuthorityProxyOffsetGameUnits", "PalmAndGrabOffsets",
                "LeftGrabAuthorityProxyOffset", RPS::Settings::RenameKind::Float3ToTransformPosition },
            { "GrabSafetyAndRelease", "ShoulderStashHmdBackRightOffsetGameUnits", "GrabSafetyAndRelease",
                "ShoulderStashHmdBackRightOffset", RPS::Settings::RenameKind::Float3ToTransformPosition },
            { "GrabSafetyAndRelease", "ShoulderStashHmdBackLeftOffsetGameUnits", "GrabSafetyAndRelease",
                "ShoulderStashHmdBackLeftOffset", RPS::Settings::RenameKind::Float3ToTransformPosition },
            { "GrabSafetyAndRelease", "MouthConsumeHmdOffsetGameUnits", "GrabSafetyAndRelease", "MouthConsumeHmdOffset",
                RPS::Settings::RenameKind::Float3ToTransformPosition },
            { "GrabContactEvidence", "GrabPinchDetectionDirectionHandspace", "GrabContactEvidence",
                "GrabPinchDetectionDirection", RPS::Settings::RenameKind::Float3ToTransformPosition },
            { "ScopePlacement", "NativeScopeFiringGripFallbackOffsetGameUnits", "ScopePlacement",
                "NativeScopeFiringGripFallbackOffset", RPS::Settings::RenameKind::Float3ToTransformPosition },
            { "ScopePlacement", "NativeScopeOverlayOffsetGameUnits", "ScopePlacement", "NativeScopeOverlayOffset",
                RPS::Settings::RenameKind::Float3ToTransformPosition },
        };

        // The hold times ROCK.toml held before each binding carried its own duration.
        struct RetiredTime
        {
            std::string_view table;
            std::string_view key;
            float fallback;
            float minimum;
            float maximum;
        };
        constexpr RetiredTime kRetiredHold{ "Bindings", "HoldSeconds", 0.35f, 0.1f, 2.0f };
        constexpr RetiredTime kRetiredDoubleTap{ "Bindings", "DoubleTapSeconds", 0.30f, 0.15f, 0.6f };
        constexpr RetiredTime kRetiredScopeHold{ "Scopes", "ManualScopeHoldSeconds", 0.30f, 0.05f, 2.0f };
        constexpr RetiredTime kRetiredPauseHold{ "ControllerInput", "PipboyPauseHoldSeconds", 0.35f, 0.15f, 2.0f };
        constexpr std::array kRetiredTimes{ kRetiredHold, kRetiredDoubleTap, kRetiredScopeHold, kRetiredPauseHold };

        /**
         * A retired hold time in its old range, or its old default when the source has none.
         */
        float retiredSeconds(const ConversionSource& source, const RetiredTime& time)
        {
            const auto value = source.value(time.table, time.key, ValueType::Float);
            float parsed = time.fallback;
            if (value)
                std::from_chars(value->data(), value->data() + value->size(), parsed);
            return std::clamp(parsed, time.minimum, time.maximum);
        }

        /**
         * One binding's conversion to its CommonFramework table: from the old string form, and with the retired hold
         * times as its duration. A file holding neither, or already holding the table without an old string, is left
         * as it is. What the table cannot say is logged and becomes the default.
         */
        template <fb::Function F>
        std::optional<std::string> convertBinding(const ConversionSource& source)
        {
            const auto key = fb::functionKey(F);
            const auto old = source.value("Bindings", key, ValueType::String);
            const bool retired = std::any_of(kRetiredTimes.begin(), kRetiredTimes.end(),
                [&](const RetiredTime& time) { return source.value(time.table, time.key, ValueType::Float).has_value(); });
            if (!old && (!retired || source.value("Bindings", key, ValueType::Table)))
                return std::nullopt;
            const binding_text::LegacyTimes times{
                .hold = retiredSeconds(source, kRetiredHold),
                .scopeHold = retiredSeconds(source, kRetiredScopeHold),
                .pauseHold = retiredSeconds(source, kRetiredPauseHold),
                .doubleTap = retiredSeconds(source, kRetiredDoubleTap),
            };
            const auto converted =
                binding_text::fromLegacyBinding(F, old ? std::optional<std::string_view>(*old) : std::nullopt, times);
            for ([[maybe_unused]] const auto& warning : converted.warnings)
                ROCK_LOG_WARN(Config, "Bindings.{}: {}", key, warning);
            return binding_text::formatBindingTable(F, converted.value);
        }

        template <std::size_t... Index>
        constexpr auto bindingConversions(std::index_sequence<Index...>)
        {
            return std::array<RPS::Settings::ValueConversion, sizeof...(Index)>{ RPS::Settings::ValueConversion{
                "Bindings", fb::functionKey(static_cast<fb::Function>(Index)),
                &convertBinding<static_cast<fb::Function>(Index)> }... };
        }

        // Each binding's conversion from the old string form and the retired hold times. Frozen like the INI table.
        constexpr auto kConversions = bindingConversions(std::make_index_sequence<fb::kFunctionCount>{});

        constexpr std::string_view kPlayerHeader =
            "## ROCK.toml - player options\n"
            "## Change these in game on ROCK's page in Mods Config FO4 VR, or edit this file. Changes are picked up "
            "while the game runs.\n"
            "## Each option has its help above it. Options missing from this file use their defaults.\n"
            "## Values are typed: true or false, whole numbers, decimal numbers, \"text in quotes\", lists such as "
            "[0.0, 1.0, 2.0]\n"
            "## and tables such as { pos = [0.0, 1.0, 2.0] }.\n"
            "## If this file cannot be read, ROCK.log names the line and ROCK keeps the last values it read.\n"
            "## When ROCK starts it adds new options with their defaults, rewrites its help lines (the lines starting "
            "with ##),\n"
            "## and moves options it no longer reads into the block at the end, as comments.\n"
            "## Your own notes are kept when they start with a single # on the lines above an option.\n"
            "## Developer options live in ROCK_Developer.toml, which exists only while a developer option is "
            "changed.\n";
        constexpr std::string_view kDeveloperHeader =
            "## ROCK_Developer.toml - developer options\n"
            "## Edit this file by hand: create it if needed, and add an option under its [table] line, as\n"
            "## data/config/ROCK_Developer_example.toml in ROCK's source lists them with their help.\n"
            "## ROCK keeps only options that differ from their defaults: one set to its default is removed,\n"
            "## and this file is removed when no options remain. Changes are picked up while the game runs.\n"
            "## If this file cannot be read, ROCK.log names the line and ROCK keeps the last values it read.\n"
            "## When ROCK starts it rewrites its help lines (the lines starting with ##) and moves options it no "
            "longer\n"
            "## reads into the block at the end, as comments. Your own notes are kept when they start with a single "
            "#\n"
            "## on the lines above an option.\n";
    }

    std::unique_ptr<Store> makeStore(const std::filesystem::path& directory, std::span<const SettingDefault> defaults)
    {
        std::vector<RPS::Settings::Definition> definitions;
        definitions.reserve(std::size(kSettingMetadata));
        for (const auto& metadata : kSettingMetadata) {
            const auto found = std::find_if(defaults.begin(), defaults.end(),
                [&](const SettingDefault& entry) { return entry.table == metadata.table && entry.key == metadata.key; });
            if (found == defaults.end())
                throw std::runtime_error(
                    std::format("Catalog option {} has no compiled default", settingPath(metadata.table, metadata.key)));
            definitions.push_back({
                .table = std::string(metadata.table),
                .key = std::string(metadata.key),
                .category = std::string(metadata.category),
                .description = std::string(metadata.description),
                .group = storeGroup(metadata.group),
                .type = found->type,
                .defaultValue = found->value,
            });
        }
        for (const auto& entry : defaults) {
            if (std::none_of(std::begin(kSettingMetadata), std::end(kSettingMetadata),
                    [&](const SettingMetadata& metadata) { return metadata.table == entry.table && metadata.key == entry.key; }))
                throw std::runtime_error(
                    std::format("Compiled option {} has no catalog entry", settingPath(entry.table, entry.key)));
        }
        return std::make_unique<Store>(
            RPS::Settings::Files{
                .directory = directory,
                .modName = "ROCK",
                .playerFileName = std::string(kPlayerFileName),
                .developerFileName = std::string(kDeveloperFileName),
                .playerHeader = std::string(kPlayerHeader),
                .developerHeader = std::string(kDeveloperHeader),
            },
            std::move(definitions),
            RPS::Settings::LegacyIni{ { directory / "ROCK.ini", directory / "ROCK_Developer.ini" }, kLegacyKeys },
            kRenamedKeys, kConversions);
    }
}
