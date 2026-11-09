#include "config/ConfigurationStore.h"
#include "config/LegacyIniMigration.h"
#include "config/SettingMetadata.h"

#include <toml++/toml.hpp>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <algorithm>
#include <charconv>
#include <climits>
#include <cmath>
#include <format>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace rock::config
{
    namespace
    {
        constexpr std::string_view kUnusedHeading = "## === Not read by this version of ROCK ===";
        constexpr std::string_view kUnusedPreamble =
            "## ROCK ignores the lines below. They keep your old values for reference; delete them whenever you "
            "like.";

        constexpr std::string_view kConsumerHeader =
            "## ROCK.toml - player options\n"
            "## Change these in game in the RobCo PALM wheel menu, or edit this file. Changes are picked up while the "
            "game runs.\n"
            "## Each option has its help above it. Options missing from this file use their defaults.\n"
            "## Values are typed: true or false, whole numbers, decimal numbers, \"text in quotes\" and lists such as "
            "[0.0, 1.0, 2.0].\n"
            "## If this file cannot be read, ROCK.log names the line and ROCK keeps the last values it read.\n"
            "## When ROCK starts it adds new options with their defaults, rewrites its help lines (the lines starting "
            "with ##),\n"
            "## and moves options it no longer reads into the block at the end, as comments.\n"
            "## Your own notes are kept when they start with a single # on the lines above an option.\n"
            "## Developer options live in ROCK_Developer.toml, which exists only while a developer option is "
            "changed.\n";
        constexpr std::string_view kDeveloperHeader =
            "## ROCK_Developer.toml - developer options\n"
            "## PALM saves only developer options that differ from their defaults. Resetting an option removes it,\n"
            "## and this file is removed when no options remain. Changes are picked up while the game runs.\n"
            "## If this file cannot be read, ROCK.log names the line and ROCK keeps the last values it read.\n"
            "## When ROCK starts it rewrites its help lines (the lines starting with ##) and moves options it no "
            "longer\n"
            "## reads into the block at the end, as comments. Your own notes are kept when they start with a single "
            "#\n"
            "## on the lines above an option.\n";

        std::string_view trim(std::string_view text) noexcept
        {
            const auto first = text.find_first_not_of(" \t");
            if (first == std::string_view::npos)
                return {};
            return text.substr(first, text.find_last_not_of(" \t") - first + 1);
        }

        std::string formatFloat(float value)
        {
            std::array<char, 64> text{};
            const auto [end, error] =
                std::to_chars(text.data(), text.data() + text.size(), value == 0.0f ? 0.0f : value);
            if (error != std::errc{})
                return "0.0";
            std::string result(text.data(), end);
            if (result.find_first_of(".eE") == std::string::npos)
                result += ".0";
            return result;
        }

        std::optional<float> parseFloat(std::string_view text)
        {
            text = trim(text);
            float number{};
            const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), number);
            if (text.empty() || error != std::errc{} || end != text.data() + text.size() || !std::isfinite(number))
                return std::nullopt;
            return number;
        }

        std::string formatFloat3(const std::array<float, 3>& value)
        {
            return std::format("[{}, {}, {}]", formatFloat(value[0]), formatFloat(value[1]), formatFloat(value[2]));
        }

        std::string quoteString(std::string_view text)
        {
            std::string result = "\"";
            for (const char c : text) {
                switch (c) {
                case '"':
                    result += "\\\"";
                    break;
                case '\\':
                    result += "\\\\";
                    break;
                case '\t':
                    result += "\\t";
                    break;
                default:
                    if (static_cast<unsigned char>(c) < 0x20 || c == 0x7F)
                        result += std::format("\\u{:04X}", static_cast<unsigned char>(c));
                    else
                        result += c;
                }
            }
            return result + "\"";
        }

        // TOML text of a canonical value.
        std::string tomlText(ValueType type, std::string_view canonical)
        {
            return type == ValueType::String ? quoteString(canonical) : std::string(canonical);
        }

        std::optional<float> numberOf(const toml::node& node)
        {
            double number{};
            if (const auto* floating = node.as_floating_point())
                number = floating->get();
            else if (const auto* integer = node.as_integer())
                number = static_cast<double>(integer->get());
            else
                return std::nullopt;
            const auto narrowed = static_cast<float>(number);
            if (!std::isfinite(narrowed))
                return std::nullopt;
            return narrowed;
        }

        std::optional<std::string> canonicalFromNode(ValueType type, const toml::node& node)
        {
            switch (type) {
            case ValueType::Boolean:
                if (const auto* value = node.as_boolean())
                    return value->get() ? "true" : "false";
                return std::nullopt;
            case ValueType::Integer:
                if (const auto* value = node.as_integer()) {
                    const auto number = value->get();
                    if (number < INT_MIN || number > INT_MAX)
                        return std::nullopt;
                    return std::to_string(number);
                }
                return std::nullopt;
            case ValueType::Float:
                if (const auto number = numberOf(node))
                    return formatFloat(*number);
                return std::nullopt;
            case ValueType::String:
                if (const auto* value = node.as_string()) {
                    if (value->get().find_first_of("\r\n") != std::string::npos)
                        return std::nullopt;
                    return value->get();
                }
                return std::nullopt;
            case ValueType::Float3:
                if (const auto* array = node.as_array(); array && array->size() == 3) {
                    std::array<float, 3> components{};
                    for (std::size_t index = 0; index < 3; ++index) {
                        const auto component = numberOf(*array->get(index));
                        if (!component)
                            return std::nullopt;
                        components[index] = *component;
                    }
                    return formatFloat3(components);
                }
                return std::nullopt;
            }
            return std::nullopt;
        }

        std::string_view typeDescription(ValueType type) noexcept
        {
            switch (type) {
            case ValueType::Boolean:
                return "true or false";
            case ValueType::Integer:
                return "a whole number";
            case ValueType::Float:
                return "a number";
            case ValueType::String:
                return "one line of text in quotes";
            case ValueType::Float3:
                return "a list of three numbers such as [0.0, 0.0, 0.0]";
            }
            return "a valid value";
        }

        // The node as TOML text on one line, for values ROCK keeps but cannot use.
        std::string rawText(const toml::node& node)
        {
            std::ostringstream stream;
            stream << toml::toml_formatter{ node, toml::format_flags::none };
            auto text = stream.str();
            std::replace(text.begin(), text.end(), '\n', ' ');
            return text;
        }

        // Splits a TOML key or table name on dots outside quotes and removes the quotes.
        std::string joinedKeyPath(std::string_view prefix, std::string_view text)
        {
            std::string result(prefix);
            std::string part;
            char quote = 0;
            const auto flush = [&]() {
                const auto trimmed = trim(part);
                if (!result.empty())
                    result += '.';
                result += trimmed;
                part.clear();
            };
            for (const char c : text) {
                if (quote) {
                    if (c == quote)
                        quote = 0;
                    else
                        part += c;
                } else if (c == '"' || c == '\'') {
                    quote = c;
                } else if (c == '.') {
                    flush();
                } else {
                    part += c;
                }
            }
            flush();
            return result;
        }

        std::size_t findOutsideQuotes(std::string_view text, char wanted) noexcept
        {
            char quote = 0;
            for (std::size_t index = 0; index < text.size(); ++index) {
                const char c = text[index];
                if (quote) {
                    if (c == '\\' && quote == '"')
                        ++index;
                    else if (c == quote)
                        quote = 0;
                } else if (c == '"' || c == '\'') {
                    quote = c;
                } else if (c == wanted) {
                    return index;
                }
            }
            return std::string_view::npos;
        }

        std::size_t occurrences(std::string_view text, std::string_view token) noexcept
        {
            std::size_t count = 0;
            for (auto at = text.find(token); at != std::string_view::npos; at = text.find(token, at + token.size()))
                ++count;
            return count;
        }

        // Comments toml++ does not keep: the player's notes (single '#'), each
        // attached to the key below it, and the lines of the unused block.
        // Lines starting with "##" belong to ROCK and are regenerated.
        struct Comments
        {
            std::map<std::string, std::vector<std::string>, std::less<>> notes;
            std::vector<std::string> unused;
        };

        Comments scanComments(std::string_view text)
        {
            Comments result;
            std::string table;
            std::vector<std::string> pending;
            std::string_view multiLineDelimiter;
            bool inUnused = false;
            while (!text.empty()) {
                const auto end = text.find('\n');
                auto line = text.substr(0, end);
                text = end == std::string_view::npos ? std::string_view{} : text.substr(end + 1);
                if (!line.empty() && line.back() == '\r')
                    line.remove_suffix(1);
                const auto trimmed = trim(line);
                if (!multiLineDelimiter.empty()) {
                    if (occurrences(trimmed, multiLineDelimiter) % 2 == 1)
                        multiLineDelimiter = {};
                    continue;
                }
                if (trimmed.empty())
                    continue;
                if (trimmed.front() == '#') {
                    if (trimmed.starts_with("##")) {
                        inUnused |= trimmed == kUnusedHeading;
                        continue;
                    }
                    (inUnused ? result.unused : pending).emplace_back(trimmed);
                    continue;
                }
                if (trimmed.front() == '[') {
                    const bool arrayOfTables = trimmed.starts_with("[[");
                    const auto open = arrayOfTables ? 2u : 1u;
                    const auto close = trimmed.find(arrayOfTables ? "]]" : "]", open);
                    if (close != std::string_view::npos)
                        table = joinedKeyPath({}, trimmed.substr(open, close - open));
                    continue;
                }
                const auto equals = findOutsideQuotes(trimmed, '=');
                if (equals == std::string_view::npos)
                    continue; // Continuation of a multi-line array or inline table.
                const auto path = joinedKeyPath(table, trimmed.substr(0, equals));
                if (!pending.empty()) {
                    auto& notes = result.notes[path];
                    notes.insert(notes.end(), pending.begin(), pending.end());
                    pending.clear();
                }
                const auto value = trim(trimmed.substr(equals + 1));
                for (const std::string_view delimiter : { std::string_view("\"\"\""), std::string_view("'''") }) {
                    if (value.starts_with(delimiter) && occurrences(value, delimiter) % 2 == 1)
                        multiLineDelimiter = delimiter;
                }
            }
            // Notes no key follows stay visible in the unused block.
            result.unused.insert(result.unused.end(), pending.begin(), pending.end());
            return result;
        }

        enum class FileState
        {
            Missing,
            Parsed,
            Failed,
        };

        struct Document
        {
            FileState state = FileState::Missing;
            std::string text;
            toml::table table;
            Comments comments;
            std::string problem;
        };

        Document readDocument(const std::filesystem::path& file)
        {
            Document document;
            const auto name = file.filename().string();
            std::error_code ec;
            const bool exists = std::filesystem::exists(file, ec);
            if (ec) {
                document.state = FileState::Failed;
                document.problem = std::format("Cannot inspect {}: {}", file.string(), ec.message());
                return document;
            }
            if (!exists)
                return document;
            {
                std::ifstream input(file, std::ios::binary);
                if (input)
                    document.text.assign(std::istreambuf_iterator<char>(input), {});
                if (!input || input.bad()) {
                    document.state = FileState::Failed;
                    document.problem = std::format("Cannot read {}", file.string());
                    return document;
                }
            }
            try {
                document.table = toml::parse(document.text, name);
            } catch (const toml::parse_error& error) {
                document.state = FileState::Failed;
                document.problem = std::format(
                    "{} line {}, column {}: {}. ROCK keeps the last values it read from this file and does not "
                    "rewrite it until the error is fixed.",
                    name, error.source().begin.line, error.source().begin.column, error.description());
                return document;
            }
            document.comments = scanComments(document.text);
            document.state = FileState::Parsed;
            return document;
        }

        template <class Visitor>
        void forEachLeaf(const toml::table& table, const std::string& prefix, Visitor&& visit)
        {
            for (const auto& [key, node] : table) {
                auto path = prefix.empty() ? std::string(key.str()) : std::format("{}.{}", prefix, key.str());
                if (const auto* child = node.as_table())
                    forEachLeaf(*child, path, visit);
                else
                    visit(path, node);
            }
        }

        using Overrides = std::map<std::string, std::optional<std::string>, std::less<>>;

        // Writes every recognized entry of the group once, grouped by table in
        // catalog order with the catalog's help above it. Consumer files gain
        // every missing option at its default; developer files hold only the
        // entries they already have or an override adds. Overrides map a
        // setting path to its new canonical value, or to nullopt to drop it.
        std::string organize(Group group, std::span<const Setting> settings, const Document* document,
            const Overrides& overrides, bool& hasEntries)
        {
            struct Entry
            {
                const Setting* setting;
                std::string path;
                std::string value;
            };
            std::vector<Entry> entries;
            std::set<std::string, std::less<>> recognized;
            for (const auto& setting : settings) {
                if (setting.group != group)
                    continue;
                auto path = settingPath(setting.table, setting.key);
                recognized.insert(path);
                std::string value;
                const toml::node* node = document ? document->table.at_path(path).node() : nullptr;
                if (const auto found = overrides.find(path); found != overrides.end()) {
                    if (!found->second)
                        continue;
                    value = tomlText(setting.type, *found->second);
                } else if (node) {
                    if (const auto canonical = canonicalFromNode(setting.type, *node))
                        value = tomlText(setting.type, *canonical);
                    else if (!node->is_table())
                        value = rawText(*node); // Kept as the player wrote it; load reported it.
                    else
                        value = tomlText(setting.type, setting.value);
                } else if (group == Group::Consumer) {
                    value = tomlText(setting.type, setting.defaultValue);
                } else {
                    continue;
                }
                entries.push_back({ &setting, std::move(path), std::move(value) });
            }
            hasEntries = !entries.empty();

            std::vector<std::string_view> tables;
            for (const auto& entry : entries) {
                if (std::find(tables.begin(), tables.end(), entry.setting->table) == tables.end())
                    tables.push_back(entry.setting->table);
            }
            std::string text(group == Group::Consumer ? kConsumerHeader : kDeveloperHeader);
            std::string_view previousCategory;
            for (const auto table : tables) {
                bool firstInTable = true;
                for (const auto& entry : entries) {
                    if (entry.setting->table != table)
                        continue;
                    if (firstInTable) {
                        if (entry.setting->category != previousCategory)
                            text += std::format("\n## === {} ===\n", entry.setting->category);
                        else
                            text += '\n';
                        previousCategory = entry.setting->category;
                        text += std::format("[{}]\n", table);
                        firstInTable = false;
                    } else {
                        text += '\n';
                    }
                    std::string_view description = entry.setting->description;
                    while (!description.empty()) {
                        const auto end = description.find('\n');
                        text += std::format("## {}\n", description.substr(0, end));
                        description =
                            end == std::string_view::npos ? std::string_view{} : description.substr(end + 1);
                    }
                    if (document) {
                        if (const auto notes = document->comments.notes.find(entry.path);
                            notes != document->comments.notes.end()) {
                            for (const auto& note : notes->second)
                                text += note + '\n';
                        }
                    }
                    text += std::format("{} = {}\n", entry.setting->key, entry.value);
                }
            }

            // Every other entry is one ROCK does not read from this file: a
            // retired option, an option owned by the other file, or a foreign
            // key. Keep it as a comment so the value stays visible but can
            // never apply.
            std::vector<std::string> unused;
            const auto addUnused = [&](std::string line) {
                if (std::find(unused.begin(), unused.end(), line) == unused.end())
                    unused.push_back(std::move(line));
            };
            if (document) {
                for (const auto& line : document->comments.unused)
                    addUnused(line);
                std::vector<std::pair<std::uint32_t, std::string>> leaves;
                forEachLeaf(document->table, {}, [&](const std::string& path, const toml::node& node) {
                    if (recognized.contains(path))
                        return;
                    std::string lines;
                    if (const auto notes = document->comments.notes.find(path);
                        notes != document->comments.notes.end()) {
                        for (const auto& note : notes->second)
                            lines += note + '\n';
                    }
                    lines += std::format("# {} = {}", path, rawText(node));
                    leaves.emplace_back(node.source().begin.line, std::move(lines));
                });
                std::stable_sort(leaves.begin(), leaves.end(),
                    [](const auto& a, const auto& b) { return a.first < b.first; });
                for (auto& [line, lines] : leaves)
                    addUnused(std::move(lines));
            }
            if (!unused.empty()) {
                text += std::format("\n{}\n{}\n", kUnusedHeading, kUnusedPreamble);
                for (const auto& line : unused)
                    text += line + '\n';
            }
            return text;
        }
    }

    std::string settingPath(std::string_view table, std::string_view key)
    {
        return table.empty() ? std::string(key) : std::format("{}.{}", table, key);
    }

    std::optional<std::array<float, 3>> parseFloat3(std::string_view value)
    {
        value = trim(value);
        if (value.size() >= 2 && value.front() == '[' && value.back() == ']')
            value = value.substr(1, value.size() - 2);
        std::array<float, 3> result{};
        for (std::size_t index = 0; index < 3; ++index) {
            const auto comma = value.find(',');
            if ((comma == std::string_view::npos) != (index == 2))
                return std::nullopt;
            const auto component = parseFloat(value.substr(0, comma));
            if (!component)
                return std::nullopt;
            result[index] = *component;
            value = comma == std::string_view::npos ? std::string_view{} : value.substr(comma + 1);
        }
        return result;
    }

    std::optional<std::string> normalizeValue(ValueType type, std::string_view value)
    {
        switch (type) {
        case ValueType::String:
            if (value.find_first_of("\r\n") != std::string_view::npos)
                return std::nullopt;
            return std::string(value);
        case ValueType::Boolean: {
            std::string lower(trim(value));
            for (auto& c : lower)
                if (c >= 'A' && c <= 'Z')
                    c += 'a' - 'A';
            if (lower == "true" || lower == "1" || lower == "yes" || lower == "on")
                return "true";
            if (lower == "false" || lower == "0" || lower == "no" || lower == "off")
                return "false";
            return std::nullopt;
        }
        case ValueType::Integer: {
            value = trim(value);
            int number{};
            const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), number);
            if (value.empty() || error != std::errc{} || end != value.data() + value.size())
                return std::nullopt;
            return std::to_string(number);
        }
        case ValueType::Float:
            if (const auto number = parseFloat(value))
                return formatFloat(*number);
            return std::nullopt;
        case ValueType::Float3:
            if (const auto vector = parseFloat3(value))
                return formatFloat3(*vector);
            return std::nullopt;
        }
        return std::nullopt;
    }

    ConfigurationStore::ConfigurationStore(std::filesystem::path directory, std::span<const SettingDefault> defaults) :
        _directory(std::move(directory))
    {
        for (std::size_t index = 0; index < std::size(kSettingMetadata); ++index) {
            const auto& metadata = kSettingMetadata[index];
            const auto path = settingPath(metadata.table, metadata.key);
            const auto found = std::find_if(defaults.begin(), defaults.end(),
                [&](const SettingDefault& entry) { return entry.table == metadata.table && entry.key == metadata.key; });
            if (found == defaults.end())
                throw std::runtime_error(std::format("Catalog option {} has no compiled default", path));
            if (normalizeValue(found->type, found->value) != found->value)
                throw std::runtime_error(std::format("Compiled default of {} is not canonical", path));
            if (std::any_of(_settings.begin(), _settings.end(),
                    [&](const Setting& setting) { return setting.table == metadata.table && setting.key == metadata.key; }))
                throw std::runtime_error(std::format("Catalog lists {} twice", path));
            _settings.push_back(Setting{
                .table = std::string(metadata.table),
                .key = std::string(metadata.key),
                .defaultValue = found->value,
                .value = found->value,
                .category = std::string(metadata.category),
                .description = std::string(metadata.description),
                .group = metadata.group,
                .type = found->type,
                .displayOrder = index,
            });
        }
        for (const auto& entry : defaults) {
            if (std::none_of(_settings.begin(), _settings.end(),
                    [&](const Setting& setting) { return setting.table == entry.table && setting.key == entry.key; }))
                throw std::runtime_error(
                    std::format("Compiled option {} has no catalog entry", settingPath(entry.table, entry.key)));
        }
        if (const auto problem = legacy_ini::findInvalidRow(_settings))
            throw std::runtime_error(*problem);
    }

    std::filesystem::path ConfigurationStore::path(Group group) const
    {
        return _directory / (group == Group::Developer ? kDeveloperFileName : kConsumerFileName);
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

    void ConfigurationStore::migrateLegacyFiles()
    {
        const std::array legacyFiles{ _directory / legacy_ini::kConsumerFileName,
            _directory / legacy_ini::kDeveloperFileName };
        std::vector<std::filesystem::path> present;
        for (const auto& file : legacyFiles) {
            std::error_code ec;
            if (std::filesystem::exists(file, ec)) {
                present.push_back(file);
            } else if (ec) {
                _messages.push_back(std::format("Cannot inspect {}: {}", file.string(), ec.message()));
                return;
            }
        }
        if (present.empty())
            return;

        std::error_code ec;
        const bool consumerExists = std::filesystem::exists(path(Group::Consumer), ec);
        if (ec) {
            _messages.push_back(std::format("Cannot inspect {}: {}", path(Group::Consumer).string(), ec.message()));
            return;
        }
        if (consumerExists) {
            // The TOML files already hold this player's settings.
            _notices.push_back(std::format("{} exists, so the old INI files are not read", kConsumerFileName));
        } else {
            std::vector<std::string> texts;
            for (const auto& file : present) {
                std::ifstream input(file, std::ios::binary);
                std::string text(std::istreambuf_iterator<char>(input), {});
                if (!input && !input.eof()) {
                    _messages.push_back(std::format(
                        "Cannot read {}; it is kept and migrated at the next start", file.string()));
                    return;
                }
                texts.push_back(std::move(text));
            }
            const std::vector<std::string_view> views(texts.begin(), texts.end());
            const auto translation = legacy_ini::translate(views, _settings);
            const auto overridesOf = [](const auto& values) {
                Overrides overrides;
                for (const auto& [entryPath, value] : values)
                    overrides.emplace(entryPath, value);
                return overrides;
            };

            // The developer file first: ROCK.toml is written last, so it exists
            // only once everything is carried over, and an interrupted
            // migration runs again at the next start.
            bool developerWritten = false;
            if (!translation.developer.empty() && !std::filesystem::exists(path(Group::Developer), ec)) {
                bool hasEntries = false;
                const auto text =
                    organize(Group::Developer, _settings, nullptr, overridesOf(translation.developer), hasEntries);
                if (hasEntries) {
                    if (!publish(Group::Developer, text, false)) {
                        _messages.push_back(std::exchange(_error, {}) + "; the old INI files are kept");
                        return;
                    }
                    developerWritten = true;
                }
            }
            bool hasEntries = false;
            const auto text =
                organize(Group::Consumer, _settings, nullptr, overridesOf(translation.consumer), hasEntries);
            if (!publish(Group::Consumer, text, false)) {
                _messages.push_back(std::exchange(_error, {}) + "; the old INI files are kept");
                return;
            }
            // Delete the INIs only once the new files read back.
            for (const auto group : { Group::Consumer, Group::Developer }) {
                if (group == Group::Developer && !developerWritten)
                    continue;
                if (const auto document = readDocument(path(group)); document.state != FileState::Parsed) {
                    _messages.push_back(std::format("{} could not be read back after migration; the old INI files "
                                                    "are kept",
                        path(group).filename().string()));
                    return;
                }
            }

            _notices.push_back(std::format("Carried {} changed settings from the old INI files into {}{}",
                translation.consumer.size() + (developerWritten ? translation.developer.size() : 0),
                kConsumerFileName, developerWritten ? std::format(" and {}", kDeveloperFileName) : std::string{}));
            if (!translation.retired.empty()) {
                std::string names;
                for (const auto& name : translation.retired)
                    names += (names.empty() ? "" : ", ") + name;
                _notices.push_back(std::format(
                    "{} old settings ROCK no longer has were not carried over: {}", translation.retired.size(), names));
            }
            for (const auto& line : translation.invalid)
                _messages.push_back(std::format("Old INI setting not carried over, its value does not fit: {}", line));
        }

        for (const auto& file : present) {
            std::filesystem::remove(file, ec);
            if (ec)
                _messages.push_back(std::format(
                    "Cannot delete {}: {}; ROCK tries again at the next start", file.string(), ec.message()));
            else
                _notices.push_back(std::format("Deleted {}", file.string()));
        }
    }

    void ConfigurationStore::load(bool organizeFiles)
    {
        _error.clear();
        _messages.clear();
        _notices.clear();
        if (organizeFiles)
            migrateLegacyFiles();
        auto updated = _settings;
        bool changed = _revision == 0;
        for (const auto group : { Group::Consumer, Group::Developer }) {
            const auto file = path(group);
            const auto document = readDocument(file);
            if (document.state == FileState::Failed) {
                _messages.push_back(document.problem);
                continue;
            }
            std::vector<Setting*> missing;
            for (auto& setting : updated) {
                if (setting.group != group)
                    continue;
                std::string next = setting.defaultValue;
                bool specified = false;
                if (const auto node = document.table.at_path(settingPath(setting.table, setting.key))) {
                    const auto canonical = canonicalFromNode(setting.type, *node.node());
                    if (!canonical) {
                        _messages.push_back(std::format("{} line {}: {}.{} must be {}; keeping {}",
                            file.filename().string(), node.node()->source().begin.line, setting.table, setting.key,
                            typeDescription(setting.type), setting.value));
                        continue;
                    }
                    next = *canonical;
                    specified = true;
                } else {
                    missing.push_back(&setting);
                }
                changed |= setting.value != next || setting.specified != specified;
                setting.value = std::move(next);
                setting.specified = specified;
            }
            if (!organizeFiles || (document.state == FileState::Missing && group == Group::Developer))
                continue;
            // Organizing writes each recognized key once with the value it
            // holds, so the rewrite cannot change what the loader reads. An
            // unwritable file can still be read; report and keep going.
            bool hasEntries = false;
            const auto* source = document.state == FileState::Parsed ? &document : nullptr;
            const auto text = organize(group, updated, source, {}, hasEntries);
            bool written = true;
            if (group == Group::Developer && !hasEntries)
                written = removeDeveloperFile();
            else if (text != document.text)
                written = publish(group, text, document.state == FileState::Parsed);
            if (!written) {
                _messages.push_back(std::exchange(_error, {}));
            } else if (group == Group::Consumer) {
                // The organized player file now holds these at their defaults,
                // so the next reload reads them as set without a revision change.
                for (auto* setting : missing)
                    setting->specified = true;
            }
        }
        _settings = std::move(updated);
        if (changed)
            ++_revision;
    }

    bool ConfigurationStore::isDefault(const Setting& setting, std::string_view value)
    {
        const auto normalized = normalizeValue(setting.type, value);
        return normalized && *normalized == setting.defaultValue;
    }

    bool ConfigurationStore::setValue(Group group, std::string_view table, std::string_view key,
        std::string_view value)
    {
        const Change change{ table, key, value };
        return setValues(group, std::span(&change, 1));
    }

    bool ConfigurationStore::setValues(Group group, std::span<const Change> changes)
    {
        _error.clear();
        // Read the current file before each edit, preserving external edits.
        const auto document = readDocument(path(group));
        if (document.state == FileState::Failed) {
            _error = document.problem;
            return false;
        }
        Overrides overrides;
        for (const auto& change : changes) {
            const auto found = std::find_if(_settings.begin(), _settings.end(), [&](const Setting& setting) {
                return setting.group == group && setting.table == change.table && setting.key == change.key;
            });
            if (found == _settings.end()) {
                _error = "Unknown setting or incorrect configuration file";
                return false;
            }
            auto normalized = normalizeValue(found->type, change.value);
            if (!normalized) {
                _error = std::format("Invalid setting value: {} must be {}", settingPath(found->table, found->key),
                    typeDescription(found->type));
                return false;
            }
            auto& override = overrides[settingPath(found->table, found->key)];
            if (group == Group::Developer && *normalized == found->defaultValue)
                override.reset();
            else
                override = std::move(normalized);
        }
        bool hasEntries = false;
        const auto text = organize(
            group, _settings, document.state == FileState::Parsed ? &document : nullptr, overrides, hasEntries);
        if (group == Group::Developer && !hasEntries)
            return removeDeveloperFile();
        return publish(group, text, true);
    }

    LoadedValues ConfigurationStore::loadedValues() const
    {
        LoadedValues values;
        for (const auto& setting : _settings) {
            if (setting.specified)
                values.emplace(settingPath(setting.table, setting.key), setting.value);
        }
        return values;
    }
}
