#include "config/LegacyIniMigration.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <format>

namespace rock::config::legacy_ini
{
    namespace
    {
        constexpr LegacyKey kLegacyKeys[] = {
#include "config/LegacyIniMigrationTable.inl"
        };

        std::string_view trim(std::string_view text) noexcept
        {
            const auto first = text.find_first_not_of(" \t");
            if (first == std::string_view::npos)
                return {};
            return text.substr(first, text.find_last_not_of(" \t") - first + 1);
        }

        // key = value lines; comments (; or #), blank lines and section
        // headers are skipped. The value is everything after the first '=',
        // so string options holding '=' or ';' keep their text.
        void readIni(std::string_view text, std::map<std::string, std::string, std::less<>>& values)
        {
            if (text.starts_with("\xEF\xBB\xBF"))
                text.remove_prefix(3);
            while (!text.empty()) {
                const auto end = text.find('\n');
                auto line = text.substr(0, end);
                text = end == std::string_view::npos ? std::string_view{} : text.substr(end + 1);
                if (!line.empty() && line.back() == '\r')
                    line.remove_suffix(1);
                line = trim(line);
                if (line.empty() || line.front() == ';' || line.front() == '#' || line.front() == '[')
                    continue;
                const auto equals = line.find('=');
                if (equals == std::string_view::npos)
                    continue;
                const auto key = trim(line.substr(0, equals));
                if (!key.empty())
                    values.insert_or_assign(std::string(key), std::string(trim(line.substr(equals + 1))));
            }
        }

        std::optional<float> floatOf(std::string_view text)
        {
            const auto canonical = normalizeValue(ValueType::Float, text);
            if (!canonical)
                return std::nullopt;
            float number{};
            std::from_chars(canonical->data(), canonical->data() + canonical->size(), number);
            return number;
        }

        // The value equals the option's default in a version that wrote it.
        template <class Equal>
        bool isOldDefault(const LegacyKey& row, Equal&& equal)
        {
            return (row.releaseDefault && equal(*row.releaseDefault)) ||
                (row.lastIniDefault && equal(*row.lastIniDefault));
        }

        const Setting* findSetting(std::span<const Setting> settings, std::string_view table, std::string_view key)
        {
            const auto found = std::find_if(settings.begin(), settings.end(),
                [&](const Setting& setting) { return setting.table == table && setting.key == key; });
            return found == settings.end() ? nullptr : &*found;
        }
    }

    std::span<const LegacyKey> legacyKeys() noexcept { return kLegacyKeys; }

    Translation translate(std::span<const std::string_view> iniTexts, std::span<const Setting> settings)
    {
        std::map<std::string, std::string, std::less<>> values;
        for (const auto text : iniTexts)
            readIni(text, values);

        std::map<std::string_view, const LegacyKey*, std::less<>> rows;
        for (const auto& row : kLegacyKeys)
            rows.emplace(row.oldKey, &row);

        Translation result;
        const auto assign = [&](const Setting& setting, std::string value) {
            auto& target = setting.group == Group::Developer ? result.developer : result.consumer;
            const auto path = settingPath(setting.table, setting.key);
            // The developer file holds only options that differ from their defaults.
            if (setting.group == Group::Developer && value == setting.defaultValue)
                target.erase(path);
            else
                target.insert_or_assign(path, std::move(value));
        };

        struct List
        {
            const Setting* setting = nullptr;
            std::array<std::optional<float>, 3> components{};
        };
        std::map<std::string, List, std::less<>> lists;

        for (const auto& [oldKey, text] : values) {
            const auto row = rows.find(oldKey);
            if (row == rows.end()) {
                result.retired.push_back(oldKey);
                continue;
            }
            const auto& legacy = *row->second;
            const auto* setting = findSetting(settings, legacy.table, legacy.key);
            if (!setting) {
                result.retired.push_back(oldKey);
                continue;
            }
            switch (legacy.kind) {
            case LegacyKind::Value: {
                const auto canonical = normalizeValue(setting->type, text);
                if (!canonical) {
                    result.invalid.push_back(std::format("{} = {}", oldKey, text));
                    break;
                }
                if (!isOldDefault(legacy, [&](std::string_view old) { return normalizeValue(setting->type, old) == canonical; }))
                    assign(*setting, *canonical);
                break;
            }
            case LegacyKind::LeftHandedMode: {
                const auto canonical = normalizeValue(ValueType::Boolean, text);
                if (!canonical) {
                    result.invalid.push_back(std::format("{} = {}", oldKey, text));
                    break;
                }
                if (!isOldDefault(legacy, [&](std::string_view old) { return normalizeValue(ValueType::Boolean, old) == canonical; }))
                    assign(*setting, *canonical == "true" ? "left" : "right");
                break;
            }
            case LegacyKind::Float3Component: {
                const auto number = floatOf(text);
                if (!number) {
                    result.invalid.push_back(std::format("{} = {}", oldKey, text));
                    break;
                }
                if (isOldDefault(legacy, [&](std::string_view old) { return floatOf(old) == number; }))
                    break;
                auto& list = lists[settingPath(setting->table, setting->key)];
                list.setting = setting;
                list.components[legacy.component] = *number;
                break;
            }
            }
        }

        // A list starts from the current default; only the components the
        // player changed replace it.
        for (const auto& [path, list] : lists) {
            auto merged = parseFloat3(list.setting->defaultValue).value_or(std::array<float, 3>{});
            for (std::size_t index = 0; index < 3; ++index) {
                if (list.components[index])
                    merged[index] = *list.components[index];
            }
            if (const auto canonical =
                    normalizeValue(ValueType::Float3, std::format("{}, {}, {}", merged[0], merged[1], merged[2])))
                assign(*list.setting, *canonical);
        }
        return result;
    }

    std::optional<std::string> findInvalidRow(std::span<const Setting> settings)
    {
        for (const auto& row : kLegacyKeys) {
            const auto* setting = findSetting(settings, row.table, row.key);
            if (!setting)
                return std::format("INI key {} maps to unknown option {}.{}", row.oldKey, row.table, row.key);
            const bool typeMatches = row.kind == LegacyKind::Float3Component ? setting->type == ValueType::Float3
                : row.kind == LegacyKind::LeftHandedMode                     ? setting->type == ValueType::String
                                                                             : setting->type != ValueType::Float3;
            if (!typeMatches)
                return std::format("INI key {} does not fit the type of {}.{}", row.oldKey, row.table, row.key);
        }
        return std::nullopt;
    }
}
