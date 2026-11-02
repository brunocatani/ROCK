#include "config/ConfigurationStore.h"
#include "config/SettingMetadata.h"

#include <Windows.h>
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <format>
#include <fstream>
#include <map>
#include <optional>
#include <set>

namespace rock::config
{
    namespace
    {
        std::optional<std::string> normalize(ValueType type, std::string_view value)
        {
            if (type == ValueType::String) {
                if (value.find_first_of("\r\n") != std::string_view::npos)
                    return std::nullopt;
                return std::string(value);
            }
            const auto first = value.find_first_not_of(" \t");
            if (first == std::string_view::npos)
                return std::nullopt;
            value = value.substr(first, value.find_last_not_of(" \t") - first + 1);
            if (type == ValueType::Boolean) {
                std::string lower(value);
                for (auto& c : lower)
                    if (c >= 'A' && c <= 'Z')
                        c += 'a' - 'A';
                if (lower == "true" || lower == "1" || lower == "yes" || lower == "on")
                    return "true";
                if (lower == "false" || lower == "0" || lower == "no" || lower == "off")
                    return "false";
                return std::nullopt;
            }
            if (type == ValueType::Integer) {
                int number{};
                const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), number);
                if (error != std::errc{} || end != value.data() + value.size())
                    return std::nullopt;
                return std::to_string(number);
            }
            float number{};
            const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), number);
            if (error != std::errc{} || end != value.data() + value.size() || !std::isfinite(number))
                return std::nullopt;
            std::array<char, 64> text{};
            const auto [last, formatted] =
                std::to_chars(text.data(), text.data() + text.size(), number == 0.0f ? 0.0f : number);
            if (formatted != std::errc{})
                return std::nullopt;
            return std::string(text.data(), last);
        }

        ValueType typeOf(std::string_view key)
        {
            switch (key.empty() ? '\0' : key.front()) {
            case 'b':
                return ValueType::Boolean;
            case 'i':
                return ValueType::Integer;
            case 'f':
                return ValueType::Float;
            default:
                return ValueType::String;
            }
        }

        bool sameKey(std::string_view lhs, std::string_view rhs) noexcept
        {
            return lhs.size() == rhs.size() && (lhs.empty() || _strnicmp(lhs.data(), rhs.data(), lhs.size()) == 0);
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
                if (ini.GetSectionSize(section.pItem) > 0)
                    return true;
            }
            return false;
        }

        template <class Visitor>
        void forEachLine(std::string_view text, Visitor&& visit)
        {
            while (!text.empty()) {
                const auto end = text.find('\n');
                auto line = text.substr(0, end);
                if (!line.empty() && line.back() == '\r')
                    line.remove_suffix(1);
                if (!line.empty())
                    visit(line);
                if (end == std::string_view::npos)
                    break;
                text.remove_prefix(end + 1);
            }
        }

        // SimpleIni prints every '\n' of a comment as a line break, so a trailing
        // one would leave a blank line between an option's help and its key.
        void appendCommentLine(std::string& comment, std::string_view line)
        {
            if (!comment.empty())
                comment += '\n';
            comment.append(line);
        }

        constexpr std::string_view kConsumerHeader =
            "; ROCK.ini - player options\n"
            "; Change these in game in the RobCo PALM wheel menu, or edit this file. Changes are picked up while "
            "the game runs.\n"
            "; Each option has its help above it. Options missing from this file use their defaults.\n"
            "; When ROCK starts it adds new options with their defaults, rewrites its help lines (the lines "
            "starting with ;),\n"
            "; and moves options it no longer reads into the [Unused] block at the end, as comments.\n"
            "; Your own notes are kept when they start with #.\n"
            "; Section headers are labels only. If an option appears twice, the last one wins.\n"
            "; Developer options live in ROCK_Developer.ini, which exists only while a developer option is "
            "changed.\n"
            "; README.txt in the ROCK download explains both files.\n\n";
        constexpr std::string_view kDeveloperHeader =
            "; ROCK_Developer.ini - developer options\n"
            "; PALM saves only developer options that differ from their defaults. Resetting an option removes it,\n"
            "; and this file is removed when no options remain. Changes are picked up while the game runs.\n"
            "; When ROCK starts it rewrites its help lines (the lines starting with ;) and moves options it no "
            "longer\n"
            "; reads into the [Unused] block at the end, as comments. Your own notes are kept when they start "
            "with #.\n"
            "; Section headers are labels only. If an option appears twice, the last one wins.\n"
            "; README.txt in the ROCK download explains both files.\n\n";

        // SimpleIni drops comments that end a file, so retired entries live in
        // the comment of an empty section that every rewrite carries forward.
        constexpr const char* kUnusedSection = "Unused";
        constexpr std::string_view kUnusedPreamble[] = {
            "; === Not read by this version of ROCK ===",
            "; ROCK ignores the lines below. They keep your old values for reference; delete them whenever you "
            "like.",
        };

        bool isUnusedPreamble(std::string_view line)
        {
            return std::find(std::begin(kUnusedPreamble), std::end(kUnusedPreamble), line) != std::end(kUnusedPreamble);
        }
    }

    Group settingGroup(std::string_view, std::string_view key) noexcept
    {
        for (const auto& setting : kSettingMetadata) {
            if (sameKey(setting.key, key))
                return setting.group;
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
        std::stable_sort(_settings.begin(), _settings.end(),
            [](const Setting& a, const Setting& b) { return a.displayOrder < b.displayOrder; });
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
        if (!exists)
            return true;
        ini.SetUnicode(false);
        ini.SetMultiKey(true);
        if (ini.LoadFile(source.c_str()) < 0) {
            _error = std::format("Cannot read {}", source.string());
            return false;
        }
        return true;
    }

    bool ConfigurationStore::writeFile(Group group, const CSimpleIniA& ini, bool replace)
    {
        CSimpleIniA organized;
        if (!organizeFile(group, ini, organized))
            return false;
        if (group == Group::Developer && !hasKeys(organized))
            return removeDeveloperFile();
        std::string contents;
        if (organized.Save(contents, false) < 0) {
            _error = "Cannot serialize configuration";
            return false;
        }
        return publish(group, contents, replace);
    }

    bool ConfigurationStore::removeDeveloperFile()
    {
        const auto destination = path(Group::Developer);
        std::error_code ec;
        std::filesystem::remove(destination, ec);
        if (ec)
            _error = std::format("Cannot remove empty {}: {}", destination.string(), ec.message());
        return !ec;
    }

    bool ConfigurationStore::publish(Group group, std::string_view contents, bool replace)
    {
        const auto destination = path(group);
        std::error_code ec;
        std::filesystem::create_directories(_directory, ec);
        if (ec) {
            _error = std::format("Cannot create {}: {}", _directory.string(), ec.message());
            return false;
        }
        auto temporary = destination;
        temporary += L".creating";
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
        output.close();
        if (!output) {
            _error = std::format("Cannot write {}", temporary.string());
            std::filesystem::remove(temporary, ec);
            return false;
        }
        const DWORD flags = MOVEFILE_WRITE_THROUGH | (replace ? MOVEFILE_REPLACE_EXISTING : 0);
        if (!MoveFileExW(temporary.c_str(), destination.c_str(), flags)) {
            const auto error = GetLastError();
            std::filesystem::remove(temporary, ec);
            if (!replace && (error == ERROR_ALREADY_EXISTS || error == ERROR_FILE_EXISTS))
                return true;
            _error = std::format("Cannot publish {}: Windows error {}", destination.string(), error);
            return false;
        }
        return true;
    }

    bool ConfigurationStore::refreshFile(Group group)
    {
        const auto source = path(group);
        std::string contents;
        {
            std::ifstream input(source, std::ios::binary);
            if (!input) {
                _error = std::format("Cannot read {}", source.string());
                return false;
            }
            contents.assign(std::istreambuf_iterator<char>(input), {});
            if (input.bad()) {
                _error = std::format("Cannot read {}", source.string());
                return false;
            }
        }
        CSimpleIniA existing;
        existing.SetUnicode(false);
        existing.SetMultiKey(true);
        if (existing.LoadData(contents) < 0) {
            _error = std::format("Cannot parse {}", source.string());
            return false;
        }
        // Organizing writes each recognized key once with its effective (last)
        // value, so the rewrite cannot change what the loader reads.
        CSimpleIniA organized;
        if (!organizeFile(group, existing, organized))
            return false;
        if (group == Group::Developer && !hasKeys(organized))
            return removeDeveloperFile();
        std::string updated;
        if (organized.Save(updated, false) < 0) {
            _error = "Cannot serialize configuration";
            return false;
        }
        return updated == contents || publish(group, updated, true);
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
                // Organizing an empty file writes every player option at its default.
                const CSimpleIniA empty;
                if (!writeFile(Group::Consumer, empty, false))
                    return false;
            } else {
                // An unwritable INI can still be read. Keep the refresh error
                // for the owner to report, then load the user's existing values.
                (void)refreshFile(Group::Consumer);
            }
            const bool developerExists = std::filesystem::exists(path(Group::Developer), ec);
            if (!ec && developerExists) {
                const auto consumerError = _error;
                if (!refreshFile(Group::Developer) && !consumerError.empty())
                    _error = consumerError + "; " + _error;
            }
        }
        CSimpleIniA consumer;
        CSimpleIniA developer;
        if (!readFile(Group::Consumer, consumer) || !readFile(Group::Developer, developer))
            return false;
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
        if (changed)
            ++_revision;
        return true;
    }

    bool ConfigurationStore::isDefault(const Setting& setting, std::string_view value)
    {
        const auto normalized = normalize(setting.type, value);
        const auto defaultValue = normalize(setting.type, setting.defaultValue);
        return normalized && defaultValue && *normalized == *defaultValue;
    }

    bool ConfigurationStore::setValue(Group group, std::string_view section, std::string_view key,
        std::string_view value)
    {
        const Change change{ section, key, value };
        return setValues(group, std::span(&change, 1));
    }

    bool ConfigurationStore::setValues(Group group, std::span<const Change> changes)
    {
        _error.clear();
        CSimpleIniA current;
        if (!readFile(group, current))
            return false;
        for (const auto& change : changes) {
            const auto found = std::find_if(_settings.begin(), _settings.end(),
                [&](const Setting& setting) { return setting.group == group && sameKey(setting.key, change.key); });
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
                if (existing != keys.end() && existing->second.comment)
                    comment = existing->second.comment;
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
            if (setting.specified &&
                target.SetValue(setting.section.c_str(), setting.key.c_str(), setting.value.c_str()) < 0) {
                throw std::runtime_error("Cannot compose ROCK configuration");
            }
        }
    }

    // Lines starting with ';' belong to ROCK and are rebuilt from the catalog
    // on every write; lines starting with '#' are the player's notes and stay
    // with their option. ROCK.ini also gains every missing player option.
    bool ConfigurationStore::organizeFile(Group group, const CSimpleIniA& source, CSimpleIniA& output)
    {
        if (output.LoadData(std::string(group == Group::Consumer ? kConsumerHeader : kDeveloperHeader)) < 0) {
            _error = "Cannot prepare configuration header";
            return false;
        }
        const auto keys = indexKeys(source);
        std::set<CSimpleIniA::Entry, CSimpleIniA::Entry::KeyOrder> writtenKeys;
        output.SetMultiKey(true);
        std::string_view previousCategory;
        for (const auto& setting : _settings) {
            if (setting.group != group)
                continue;
            const auto found = keys.find(CSimpleIniA::Entry(setting.key.c_str()));
            if (found == keys.end() && group != Group::Consumer)
                continue;
            const char* value = found != keys.end() ? found->second.value : setting.defaultValue.c_str();

            std::string comment;
            if (setting.category != previousCategory) {
                appendCommentLine(comment, std::format("; === {} ===", setting.category));
                previousCategory = setting.category;
            }
            forEachLine(setting.description,
                [&](std::string_view line) { appendCommentLine(comment, std::format("; {}", line)); });
            if (found != keys.end() && found->second.comment) {
                forEachLine(found->second.comment, [&](std::string_view line) {
                    if (line.front() == '#')
                        appendCommentLine(comment, line);
                });
            }
            if (output.SetValue(setting.section.c_str(), setting.key.c_str(), value,
                    comment.empty() ? nullptr : comment.c_str()) < 0) {
                _error = "Cannot organize configuration option";
                return false;
            }
            writtenKeys.emplace(setting.key.c_str());
        }

        // Every other entry is one ROCK does not read from this file: a retired
        // option, an option owned by the other file, or a foreign key. Keep it
        // as a comment so the value stays visible but can never apply. Shadowed
        // copies of recognized keys are dropped; they never applied either.
        std::vector<std::string> unused;
        const auto addUnused = [&](std::string line) {
            // Trailing blanks would not survive a reload and would break deduplication.
            line.erase(line.find_last_not_of(" \t") + 1);
            if (std::find(unused.begin(), unused.end(), line) == unused.end())
                unused.push_back(std::move(line));
        };
        CSimpleIniA::TNamesDepend sections;
        source.GetAllSections(sections);
        sections.sort(CSimpleIniA::Entry::LoadOrder());
        for (const auto& section : sections) {
            if (section.pComment && sameKey(section.pItem, kUnusedSection)) {
                forEachLine(section.pComment, [&](std::string_view line) {
                    if (!isUnusedPreamble(line))
                        addUnused(std::string(line));
                });
            }
        }
        for (const auto& section : sections) {
            CSimpleIniA::TNamesDepend entries;
            source.GetAllKeys(section.pItem, entries);
            entries.sort(CSimpleIniA::Entry::LoadOrder());
            for (const auto& key : entries) {
                if (writtenKeys.contains(key))
                    continue;
                CSimpleIniA::TNamesDepend values;
                source.GetAllValues(section.pItem, key.pItem, values);
                for (const auto& value : values)
                    addUnused(std::format("; {} = {}", key.pItem, value.pItem));
            }
        }
        if (unused.empty())
            return true;
        std::string comment;
        for (const auto line : kUnusedPreamble)
            appendCommentLine(comment, line);
        for (const auto& line : unused)
            appendCommentLine(comment, line);
        if (output.SetValue(kUnusedSection, nullptr, nullptr, comment.c_str()) < 0) {
            _error = "Cannot keep unused configuration entries";
            return false;
        }
        return true;
    }
}
