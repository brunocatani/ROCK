#include "config/ConfigurationStore.h"
#include "config/SettingMetadata.h"

#include <Windows.h>
#include <algorithm>
#include <charconv>
#include <cmath>
#include <format>
#include <optional>
#include <map>
#include <set>
#include <cstring>

namespace rock::config
{
    namespace
    {
        std::optional<std::string> normalize(ValueType type, std::string_view value)
        {
            if (type == ValueType::String) {
                if (value.find_first_of("\r\n") != std::string_view::npos) return std::nullopt;
                return std::string(value);
            }
            const auto first = value.find_first_not_of(" \t");
            if (first == std::string_view::npos) return std::nullopt;
            value = value.substr(first, value.find_last_not_of(" \t") - first + 1);
            if (type == ValueType::Boolean) {
                std::string lower(value);
                for (auto& c : lower) if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
                if (lower == "true" || lower == "1" || lower == "yes" || lower == "on") return "true";
                if (lower == "false" || lower == "0" || lower == "no" || lower == "off") return "false";
                return std::nullopt;
            }
            if (type == ValueType::Integer) {
                int number{};
                const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), number);
                if (error != std::errc{} || end != value.data() + value.size()) return std::nullopt;
                return std::to_string(number);
            }
            float number{};
            const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), number);
            if (error != std::errc{} || end != value.data() + value.size() || !std::isfinite(number)) return std::nullopt;
            std::array<char, 64> text{};
            const auto [last, formatted] = std::to_chars(text.data(), text.data() + text.size(), number == 0.0f ? 0.0f : number);
            if (formatted != std::errc{}) return std::nullopt;
            return std::string(text.data(), last);
        }

        ValueType typeOf(std::string_view key)
        {
            switch (key.empty() ? '\0' : key.front()) {
            case 'b': return ValueType::Boolean;
            case 'i': return ValueType::Integer;
            case 'f': return ValueType::Float;
            default: return ValueType::String;
            }
        }

        bool sameKey(std::string_view lhs, std::string_view rhs) noexcept
        {
            return lhs.size() == rhs.size() &&
                (lhs.empty() || _strnicmp(lhs.data(), rhs.data(), lhs.size()) == 0);
        }

        struct IniValue
        {
            const char* value;
            const char* comment;
            int order;
        };
        using KeyIndex = std::map<CSimpleIniA::Entry, IniValue, CSimpleIniA::Entry::KeyOrder>;

        // Borrowed only while the source INI is unchanged. Multi-key parsing
        // retains physical load order, including repeated section headers.
        KeyIndex indexKeys(const CSimpleIniA& ini)
        {
            KeyIndex result;
            CSimpleIniA::TNamesDepend sections;
            ini.GetAllSections(sections);
            for (const auto& section : sections) {
                for (const auto& [key, value] : *ini.GetSection(section.pItem)) {
                    auto [found, inserted] = result.try_emplace(key, IniValue{ value, key.pComment, key.nOrder });
                    if (!inserted && key.nOrder > found->second.order)
                        found->second = { value, key.pComment, key.nOrder };
                }
            }
            return result;
        }

        void removeKey(CSimpleIniA& ini, const char* key)
        {
            CSimpleIniA::TNamesDepend sections;
            ini.GetAllSections(sections);
            for (const auto& section : sections)
                ini.Delete(section.pItem, key, true);
        }

        bool hasKeys(const CSimpleIniA& ini)
        {
            CSimpleIniA::TNamesDepend sections;
            ini.GetAllSections(sections);
            for (const auto& section : sections) {
                if (ini.GetSectionSize(section.pItem) > 0) return true;
            }
            return false;
        }
    }

    Group settingGroup(std::string_view, std::string_view key) noexcept
    {
        for (const auto& setting : kSettingMetadata) {
            if (sameKey(setting.key, key)) return setting.group;
        }
        return Group::Consumer;
    }

    ConfigurationStore::ConfigurationStore(std::filesystem::path directory, const CSimpleIniA& defaults) :
        _directory(std::move(directory))
    {
        CSimpleIniA::TNamesDepend sections;
        defaults.GetAllSections(sections);
        for (const auto& section : sections) {
            CSimpleIniA::TNamesDepend keys;
            defaults.GetAllKeys(section.pItem, keys);
            for (const auto& key : keys) {
                Setting setting{
                    .section = section.pItem,
                    .key = key.pItem,
                    .defaultValue = defaults.GetValue(section.pItem, key.pItem, ""),
                    .category = section.pItem,
                    .group = settingGroup(section.pItem, key.pItem),
                    .type = typeOf(key.pItem),
                };
                setting.value = setting.defaultValue;
                for (std::size_t index = 0; index < std::size(kSettingMetadata); ++index) {
                    const auto& metadata = kSettingMetadata[index];
                    if (metadata.key == setting.key && metadata.section == setting.section) {
                        setting.category = metadata.category;
                        setting.description = metadata.description;
                        setting.displayOrder = index;
                        break;
                    }
                }
                _settings.push_back(std::move(setting));
            }
        }
        std::stable_sort(_settings.begin(), _settings.end(), [](const Setting& a, const Setting& b) {
            return a.displayOrder < b.displayOrder;
        });
    }

    std::filesystem::path ConfigurationStore::path(Group group) const
    {
        return _directory / (group == Group::Developer ? "ROCK_Developer.ini" : "ROCK.ini");
    }

    bool ConfigurationStore::readFile(Group group, CSimpleIniA& ini)
    {
        std::error_code ec;
        const auto source = path(group);
        const bool exists = std::filesystem::exists(source, ec);
        if (ec) {
            _error = std::format("Cannot inspect {}: {}", source.string(), ec.message());
            return false;
        }
        if (!exists) return true;
        ini.SetUnicode(false);
        ini.SetMultiKey(true);
        if (ini.LoadFile(source.c_str()) < 0) {
            _error = std::format("Cannot read {}", source.string());
            return false;
        }
        return true;
    }

    bool ConfigurationStore::writeFile(Group group, CSimpleIniA& ini, bool replace)
    {
        const auto destination = path(group);
        std::error_code ec;
        if (group == Group::Developer && !hasKeys(ini)) {
            std::filesystem::remove(destination, ec);
            if (ec) _error = std::format("Cannot remove empty {}: {}", destination.string(), ec.message());
            return !ec;
        }
        CSimpleIniA organized;
        if (!organizeFile(group, ini, organized)) return false;
        std::filesystem::create_directories(_directory, ec);
        if (ec) {
            _error = std::format("Cannot create {}: {}", _directory.string(), ec.message());
            return false;
        }
        auto temporary = destination;
        temporary += L".creating";
        if (organized.SaveFile(temporary.c_str(), false) < 0) {
            _error = std::format("Cannot write {}", temporary.string());
            std::filesystem::remove(temporary, ec);
            return false;
        }
        const DWORD flags = MOVEFILE_WRITE_THROUGH | (replace ? MOVEFILE_REPLACE_EXISTING : 0);
        if (!MoveFileExW(temporary.c_str(), destination.c_str(), flags)) {
            const auto error = GetLastError();
            std::filesystem::remove(temporary, ec);
            if (!replace && (error == ERROR_ALREADY_EXISTS || error == ERROR_FILE_EXISTS)) return true;
            _error = std::format("Cannot publish {}: Windows error {}", destination.string(), error);
            return false;
        }
        return true;
    }

    bool ConfigurationStore::load(bool createConsumer)
    {
        _error.clear();
        if (createConsumer) {
            std::error_code ec;
            const bool exists = std::filesystem::exists(path(Group::Consumer), ec);
            if (ec) {
                _error = std::format("Cannot inspect ROCK.ini: {}", ec.message());
                return false;
            }
            if (!exists) {
                CSimpleIniA defaults;
                if (!materializeConsumerDefaults(defaults)) return false;
                if (!writeFile(Group::Consumer, defaults, false)) return false;
            }
        }
        CSimpleIniA consumer;
        CSimpleIniA developer;
        if (!readFile(Group::Consumer, consumer) || !readFile(Group::Developer, developer)) return false;
        const auto consumerKeys = indexKeys(consumer);
        const auto developerKeys = indexKeys(developer);
        auto updated = _settings;
        bool changed = _revision == 0;
        for (auto& setting : updated) {
            const auto& keys = setting.group == Group::Consumer ? consumerKeys : developerKeys;
            const auto found = keys.find(CSimpleIniA::Entry(setting.key.c_str()));
            const char* value = found != keys.end() ? found->second.value : nullptr;
            const bool specified = value != nullptr;
            const std::string next = specified ? normalize(setting.type, value).value_or(value) : setting.defaultValue;
            changed |= setting.value != next || setting.specified != specified;
            setting.value = next;
            setting.specified = specified;
        }
        _settings = std::move(updated);
        if (changed) ++_revision;
        return true;
    }

    bool ConfigurationStore::isDefault(const Setting& setting, std::string_view value)
    {
        const auto normalized = normalize(setting.type, value);
        const auto defaultValue = normalize(setting.type, setting.defaultValue);
        return normalized && defaultValue && *normalized == *defaultValue;
    }

    bool ConfigurationStore::setValue(Group group, std::string_view section,
        std::string_view key, std::string_view value)
    {
        const Change change{ section, key, value };
        return setValues(group, std::span(&change, 1));
    }

    bool ConfigurationStore::setValues(Group group, std::span<const Change> changes)
    {
        _error.clear();
        CSimpleIniA current;
        if (!readFile(group, current)) return false;
        if (group == Group::Consumer) {
            std::error_code ec;
            const bool exists = std::filesystem::exists(path(group), ec);
            if (ec) { _error = ec.message(); return false; }
            if (!exists && !materializeConsumerDefaults(current)) return false;
        }
        for (const auto& change : changes) {
            const auto found = std::find_if(_settings.begin(), _settings.end(), [&](const Setting& setting) {
                return setting.group == group && sameKey(setting.key, change.key);
            });
            if (found == _settings.end()) {
                _error = "Unknown setting or incorrect configuration file";
                return false;
            }
            const auto normalized = normalize(found->type, change.value);
            if (!normalized) {
                _error = "Invalid setting value";
                return false;
            }
            std::string comment;
            {
                const auto keys = indexKeys(current);
                const auto existing = keys.find(CSimpleIniA::Entry(found->key.c_str()));
                if (existing != keys.end() && existing->second.comment) comment = existing->second.comment;
            }
            // Remove every copy before saving/resetting. An old value under a
            // different decorative heading must never resurrect an override.
            removeKey(current, found->key.c_str());
            if (!(group == Group::Developer && isDefault(*found, *normalized)) &&
                current.SetValue(found->section.c_str(), found->key.c_str(), normalized->c_str(),
                    comment.empty() ? nullptr : comment.c_str(), true) < 0) {
                _error = "Cannot update setting";
                return false;
            }
        }
        // Read the current file before each edit, preserving external edits;
        // a multi-axis control publishes all components in one replacement.
        return writeFile(group, current, true);
    }

    void ConfigurationStore::appendLoadedValues(CSimpleIniA& target) const
    {
        for (const auto& setting : _settings) {
            if (setting.specified && target.SetValue(setting.section.c_str(), setting.key.c_str(), setting.value.c_str()) < 0) {
                throw std::runtime_error("Cannot compose ROCK configuration");
            }
        }
    }

    bool ConfigurationStore::materializeConsumerDefaults(CSimpleIniA& ini)
    {
        for (const auto& setting : _settings) {
            if (setting.group == Group::Consumer && ini.SetValue(setting.section.c_str(),
                    setting.key.c_str(), setting.defaultValue.c_str()) < 0) {
                _error = "Cannot materialize compiled consumer defaults";
                return false;
            }
        }
        return true;
    }

    bool ConfigurationStore::organizeFile(Group group, const CSimpleIniA& source, CSimpleIniA& output)
    {
        const auto header = group == Group::Consumer ?
            "; ROCK.ini - regular options\n; Created with all regular defaults when missing.\n" :
            "; ROCK_Developer.ini - developer options\n; Created only after a non-default developer change.\n"
            "; New files contain only changed options. Existing entries are preserved.\n"
            "; Resetting an option removes its entry; an empty developer file is removed.\n";
        if (output.LoadData(std::string(header) +
                "; Missing options use compiled defaults. Section numbers match the wheel menu.\n"
                "; Section headers are decorative; key names are case-insensitive within their owning file.\n"
                "; If a key is repeated, its last occurrence in the file wins.\n\n") < 0) {
            _error = "Cannot prepare configuration header";
            return false;
        }
        const auto keys = indexKeys(source);
        std::set<CSimpleIniA::Entry, CSimpleIniA::Entry::KeyOrder> writtenKeys;
        output.SetMultiKey(true);
        std::string_view previousCategory;
        for (const auto& setting : _settings) {
            if (setting.group != group) continue;
            const auto found = keys.find(CSimpleIniA::Entry(setting.key.c_str()));
            if (found == keys.end()) continue;
            const auto* value = found->second.value;

            std::string comment;
            if (setting.category != previousCategory) {
                comment = std::format("; === {} ===\n", setting.category);
                previousCategory = setting.category;
            }
            const char* existing = found->second.comment;
            if (existing) {
                // Rebuild only our category banner. Keep authored per-option help.
                std::string_view remaining(existing);
                while (!remaining.empty()) {
                    const auto end = remaining.find('\n');
                    auto line = remaining.substr(0, end);
                    if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
                    if (!line.empty() && !(line.starts_with("; === ") && line.ends_with(" ==="))) {
                        comment.append(line);
                        comment += '\n';
                    }
                    if (end == std::string_view::npos) break;
                    remaining.remove_prefix(end + 1);
                }
            } else if (!setting.description.empty()) {
                comment += "; ";
                for (const char c : setting.description) {
                    comment += c;
                    if (c == '\n') comment += "; ";
                }
                comment += '\n';
            }
            if (output.SetValue(setting.section.c_str(), setting.key.c_str(), value,
                    comment.empty() ? nullptr : comment.c_str()) < 0) {
                _error = "Cannot organize configuration option";
                return false;
            }
            writtenKeys.emplace(setting.key.c_str());
        }

        // Explicit writes organize recognized settings using their effective
        // value/comment once. Preserve unknown and other-file entries as data;
        // copying shadowed recognized keys would reintroduce stale overrides.
        CSimpleIniA::TNamesDepend sections;
        source.GetAllSections(sections);
        for (const auto& section : sections) {
            for (const auto& [key, value] : *source.GetSection(section.pItem)) {
                if (!writtenKeys.contains(key) &&
                    output.SetValue(section.pItem, key.pItem, value, key.pComment) < 0) {
                    _error = "Cannot preserve external configuration option";
                    return false;
                }
            }
        }
        return true;
    }
}
