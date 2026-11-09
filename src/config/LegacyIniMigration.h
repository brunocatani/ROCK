// One-time carry of a player's settings from the INI files ROCK used before
// TOML (ROCK.ini, ROCK_Developer.ini) into ROCK.toml and ROCK_Developer.toml.
// ConfigurationStore runs it at startup and deletes the INIs once the TOML
// files are written and read back; this file only translates text.
#pragma once

#include "config/ConfigurationStore.h"

#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace rock::config::legacy_ini
{
    inline constexpr std::string_view kConsumerFileName = "ROCK.ini";
    inline constexpr std::string_view kDeveloperFileName = "ROCK_Developer.ini";

    enum class LegacyKind : std::uint8_t
    {
        Value,           // Same value, new table and key.
        Float3Component, // One of the X/Y/Z or pitch/yaw/roll keys of a list.
        LeftHandedMode,  // bLeftHandedMode = true becomes DefaultHand = "left".
    };

    // An INI key ROCK shipped, the option it became and the defaults it had in
    // ROCK 0.92.2 and in the last INI version, as the old INI wrote them.
    struct LegacyKey
    {
        std::string_view oldKey;
        std::string_view table;
        std::string_view key;
        LegacyKind kind;
        std::uint8_t component;
        std::optional<std::string_view> releaseDefault;
        std::optional<std::string_view> lastIniDefault;
    };

    [[nodiscard]] std::span<const LegacyKey> legacyKeys() noexcept;

    struct Translation
    {
        // Canonical values keyed by settingPath(), split by the file that owns
        // each option now (an option may have changed files since the INI).
        std::map<std::string, std::string, std::less<>> consumer;
        std::map<std::string, std::string, std::less<>> developer;
        // Old keys ROCK no longer has, and "key = value" lines whose value the
        // option's type cannot hold. Both are dropped.
        std::vector<std::string> retired;
        std::vector<std::string> invalid;
    };

    // Reads the INI texts in order (player file, then developer file; a later
    // key wins, as in the old loader) and keeps only values the player
    // changed: a value equal to the option's old default is left out, so the
    // current default applies. Section headers are ignored, as they were.
    [[nodiscard]] Translation translate(std::span<const std::string_view> iniTexts, std::span<const Setting> settings);

    // Every table row names an option of the catalog with a matching type;
    // returns the first row that does not.
    [[nodiscard]] std::optional<std::string> findInvalidRow(std::span<const Setting> settings);
}
