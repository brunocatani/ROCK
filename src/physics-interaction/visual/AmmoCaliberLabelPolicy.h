#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <string_view>

/*
 * The ammo counters' caliber label, from an ammo form's display name.
 *
 * RPS UI widens a counter to fit its label (the HUD styles' bracket frame
 * grows with it), so a full name such as "9x19mm Parabellum Round" stretches
 * the counter into a long strip. The label is the cartridge's short
 * designation instead, as ammunition boxes print it, about kCaliberBudget
 * characters, the width of the short vanilla names (".38", "10mm"):
 *
 *  - Sort tags and nicknames go: "[AmmoCaliber]", a "(...)" part and a
 *    quoted nickname ("6.5mm Winchester Short Magnum \"Leopard\"").
 *  - Generic words go: a trailing "Round", "Ammo", "Cartridge" or "Shell"
 *    (singular or plural), and "Caliber" anywhere (".50 Caliber" -> ".50").
 *  - The designation is the first word holding a digit, with a following
 *    "Gauge" ("16 Gauge Vintage" -> "16 Gauge"). Words before it go
 *    ("HK 4.6x30mm" -> "4.6x30").
 *  - A two-number designation names the cartridge by itself, so the words
 *    after it go, and a metric one drops its "mm" as cartridge tables write
 *    it: "9x19mm Parabellum" -> "9x19", "7.62x54mmR" -> "7.62x54R",
 *    ".30-06 Springfield" -> ".30-06".
 *  - Otherwise the words after it are added while the label stays within
 *    the budget: their all-caps words (".300 AAC Blackout" -> ".300 AAC"),
 *    else the initials of several words (".22 Long Rifle" -> ".22 LR",
 *    ".40 Smith & Wesson" -> ".40 S&W"), else the one word or its usual
 *    short form (".357 Magnum" -> ".357 Mag"). A designation in "mm" drops
 *    it when that lets a code or initials fit ("6.8mm Remington SPC" ->
 *    "6.8 SPC"), never for a word ("6mm Dasher" -> "6mm").
 *  - A name without a digit stays whole within the budget, otherwise it
 *    keeps its first all-caps word ("BB Pellet" -> "BB"), else its last
 *    word, the thing it is ("Flamer Fuel" -> "Fuel").
 *
 * A designation is never shortened beyond these rules; only the label's
 * capacity cuts it. Lengths count UTF-8 characters, and no cut splits one.
 * Builds a string, so callers keep the result instead of calling it every
 * frame.
 */
namespace rock::ammo_counters
{
    // The caliber label: at most kCaliberCapacity - 1 bytes, terminated.
    inline constexpr std::size_t kCaliberCapacity = 24;
    using Caliber = std::array<char, kCaliberCapacity>;
    inline constexpr std::size_t kCaliberBudget = 8;

    namespace caliber_detail
    {
        inline constexpr std::size_t kMaxWords = 16;

        inline bool isSpace(const char c) noexcept { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }
        inline bool isDigit(const char c) noexcept { return c >= '0' && c <= '9'; }
        inline bool isUpper(const char c) noexcept { return c >= 'A' && c <= 'Z'; }
        inline bool isLower(const char c) noexcept { return c >= 'a' && c <= 'z'; }
        inline bool isContinuation(const char c) noexcept { return (static_cast<unsigned char>(c) & 0xC0) == 0x80; }
        inline char lower(const char c) noexcept { return isUpper(c) ? static_cast<char>(c - 'A' + 'a') : c; }
        inline char upper(const char c) noexcept { return isLower(c) ? static_cast<char>(c - 'a' + 'A') : c; }

        inline bool same(const std::string_view a, const std::string_view b) noexcept
        {
            return a.size() == b.size() &&
                std::equal(a.begin(), a.end(), b.begin(), [](const char x, const char y) { return lower(x) == lower(y); });
        }

        inline std::size_t characters(const std::string_view text) noexcept
        {
            return static_cast<std::size_t>(std::count_if(text.begin(), text.end(), [](const char c) { return !isContinuation(c); }));
        }

        // A bounded string: appends stop at the capacity, never inside a character.
        struct Text
        {
            std::array<char, kCaliberCapacity> chars{};
            std::size_t size{ 0 };

            void append(const std::string_view text) noexcept
            {
                std::size_t count = std::min(text.size(), chars.size() - 1 - size);
                while (count > 0 && count < text.size() && isContinuation(text[count]))
                    --count;
                std::copy_n(text.data(), count, chars.data() + size);
                size += count;
            }
            [[nodiscard]] std::string_view view() const noexcept { return { chars.data(), size }; }
        };

        struct Words
        {
            std::array<std::string_view, kMaxWords> items{};
            std::size_t count{ 0 };

            // Erases words first..last.
            void erase(const std::size_t first, const std::size_t last) noexcept
            {
                std::copy(items.begin() + last + 1, items.begin() + count, items.begin() + first);
                count -= last - first + 1;
            }
            void erase(const std::size_t index) noexcept { erase(index, index); }
        };

        inline Words split(const std::string_view name) noexcept
        {
            Words words{};
            std::size_t i = 0;
            while (i < name.size() && words.count < kMaxWords) {
                while (i < name.size() && isSpace(name[i]))
                    ++i;
                const std::size_t start = i;
                while (i < name.size() && !isSpace(name[i]))
                    ++i;
                if (i > start)
                    words.items[words.count++] = name.substr(start, i - start);
            }
            return words;
        }

        inline bool hasDigit(const std::string_view word) noexcept { return std::any_of(word.begin(), word.end(), isDigit); }

        // Two numbers joined by x, - or /: "9x19mm", ".30-06", ".577/450".
        // metric: joined by x.
        inline bool isPair(const std::string_view word, const bool metric) noexcept
        {
            for (std::size_t i = 0; i + 2 < word.size(); ++i) {
                const char joint = word[i + 1];
                const bool joins = metric ? (joint == 'x' || joint == 'X') : (joint == 'x' || joint == 'X' || joint == '-' || joint == '/');
                if (isDigit(word[i]) && joins && (isDigit(word[i + 2]) || word[i + 2] == '.'))
                    return true;
            }
            return false;
        }

        // The word without the "mm" that follows its number ("7.62x54mmR" ->
        // "7.62x54R"); the word itself without one.
        inline Text withoutMillimetres(const std::string_view word) noexcept
        {
            Text text{};
            for (std::size_t i = 1; i + 1 < word.size(); ++i) {
                if (isDigit(word[i - 1]) && lower(word[i]) == 'm' && lower(word[i + 1]) == 'm') {
                    text.append(word.substr(0, i));
                    text.append(word.substr(i + 2));
                    return text;
                }
            }
            text.append(word);
            return text;
        }

        // Two or more letters, none lower case: "AAC", "SPC", "SP-5".
        inline bool isAcronym(const std::string_view word) noexcept
        {
            std::size_t letters = 0;
            for (const char c : word) {
                if (isLower(c))
                    return false;
                letters += isUpper(c) ? 1 : 0;
            }
            return letters >= 2;
        }

        inline std::string_view shortForm(const std::string_view word) noexcept
        {
            constexpr std::array<std::array<std::string_view, 2>, 6> kShortForms{ {
                { "Magnum", "Mag" },
                { "Remington", "Rem" },
                { "Winchester", "Win" },
                { "Carbine", "Carb" },
                { "Automatic", "Auto" },
                { "Special", "Spl" },
            } };
            for (const auto& [full, abbreviated] : kShortForms) {
                if (same(word, full))
                    return abbreviated;
            }
            return word;
        }

        // The words after the designation, compacted.
        inline Text qualifier(const Words& words, const std::size_t first) noexcept
        {
            Text text{};
            const std::size_t count = words.count - first;
            for (std::size_t i = first; i < words.count; ++i) {
                if (!isAcronym(words.items[i]))
                    continue;
                if (text.size > 0)
                    text.append(" ");
                text.append(words.items[i]);
            }
            if (text.size > 0 || count == 0)
                return text;
            if (count == 1) {
                text.append(shortForm(words.items[first]));
                return text;
            }
            for (std::size_t i = first; i < words.count; ++i) {
                const auto word = words.items[i];
                if (word == "&") {
                    text.append("&");
                    continue;
                }
                std::size_t length = 1;
                while (length < word.size() && isContinuation(word[length]))
                    ++length;
                if (length == 1) {
                    const char initial = upper(word[0]);
                    text.append({ &initial, 1 });
                } else {
                    text.append(word.substr(0, length));
                }
            }
            return text;
        }

        inline void join(Text& out, const std::string_view designation, const std::string_view rest) noexcept
        {
            out.append(designation);
            if (!rest.empty()) {
                out.append(" ");
                out.append(rest);
            }
        }

        inline bool fits(const std::string_view designation, const std::string_view rest) noexcept
        {
            return characters(designation) + (rest.empty() ? 0 : 1 + characters(rest)) <= kCaliberBudget;
        }

        inline Text compact(Words words) noexcept
        {
            Text text{};
            // Sort tags and nicknames go: "[AmmoCaliber]", "(Old)", "\"Leopard\"".
            for (std::size_t i = 0; i < words.count;) {
                const char open = words.items[i].front();
                const char close = open == '[' ? ']' : open == '(' ? ')' : open == '"' ? '"' : '\0';
                std::size_t last = i;
                const auto closes = [&](const std::size_t at) {
                    const auto word = words.items[at];
                    return word.back() == close && (at > i || word.size() > 1);
                };
                while (close != '\0' && last < words.count && !closes(last))
                    ++last;
                if (close == '\0' || last == words.count || last - i + 1 == words.count) {
                    ++i;
                    continue;
                }
                words.erase(i, last);
            }
            const auto generic = [](const std::string_view word) {
                for (const std::string_view candidate : { "round", "rounds", "ammo", "cartridge", "cartridges", "shell", "shells" }) {
                    if (same(word, candidate))
                        return true;
                }
                return false;
            };
            while (words.count > 1 && generic(words.items[words.count - 1]))
                words.erase(words.count - 1);
            for (std::size_t i = 0; i < words.count && words.count > 1;) {
                if (same(words.items[i], "caliber"))
                    words.erase(i);
                else
                    ++i;
            }
            if (words.count == 0)
                return text;

            std::size_t index = 0;
            while (index < words.count && !hasDigit(words.items[index]))
                ++index;
            if (index == words.count) {
                Text name{};
                for (std::size_t i = 0; i < words.count; ++i) {
                    if (i > 0)
                        name.append(" ");
                    name.append(words.items[i]);
                }
                if (characters(name.view()) <= kCaliberBudget)
                    return name;
                std::size_t kept = words.count - 1;
                for (std::size_t i = 0; i < words.count; ++i) {
                    if (isAcronym(words.items[i])) {
                        kept = i;
                        break;
                    }
                }
                text.append(words.items[kept]);
                return text;
            }

            const auto word = words.items[index];
            if (isPair(word, true))
                return withoutMillimetres(word);
            if (isPair(word, false)) {
                text.append(word);
                return text;
            }

            Text designation{};
            designation.append(word);
            std::size_t rest = index + 1;
            if (rest < words.count && same(words.items[rest], "gauge")) {
                designation.append(" ");
                designation.append(words.items[rest]);
                ++rest;
            }
            const auto extra = qualifier(words, rest);
            if (fits(designation.view(), extra.view())) {
                join(text, designation.view(), extra.view());
                return text;
            }
            // Only codes and initials stand beside a bare number ("6.8 SPC");
            // a word keeps the unit ("6mm Dasher" shows "6mm").
            const bool code = words.count - rest > 1 || (words.count - rest == 1 && isAcronym(words.items[rest]));
            const auto shorter = withoutMillimetres(designation.view());
            if (code && shorter.size != designation.size && fits(shorter.view(), extra.view())) {
                join(text, shorter.view(), extra.view());
                return text;
            }
            return designation;
        }
    }

    // Empty for an empty name.
    [[nodiscard]] inline Caliber caliberLabel(const std::string_view name) noexcept
    {
        const auto text = caliber_detail::compact(caliber_detail::split(name));
        Caliber label{};
        std::copy_n(text.chars.data(), text.size, label.data());
        return label;
    }
}
