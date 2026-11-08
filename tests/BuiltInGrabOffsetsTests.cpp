#include "physics-interaction/grab/BuiltInGrabOffsets.h"

#include <iostream>
#include <string>
#include <tuple>

using namespace rock::saved_grab_offset;

namespace
{
    int failures = 0;

    void check(bool value, const std::string& message)
    {
        if (!value) {
            std::cerr << message << '\n';
            ++failures;
        }
    }

    bool lowercase(std::string_view text)
    {
        for (const char c : text) {
            if (c >= 'A' && c <= 'Z') {
                return false;
            }
        }
        return true;
    }
}

int main()
{
    const auto offsets = built_in::offsets();
    const auto aliases = built_in::aliases();
    const auto poses = built_in::handPoses();
    check(!offsets.empty() && !poses.empty(), "built-in tables are empty");

    // The lookup is a binary search: the generated tables must stay sorted and unique.
    for (std::size_t i = 0; i < offsets.size(); ++i) {
        const auto& entry = offsets[i];
        const std::string name = std::string(entry.plugin) + " " + std::to_string(entry.localFormId);
        check(lowercase(entry.plugin), name + ": plugin not lowercase");
        check(entry.right != built_in::kNone || entry.left != built_in::kNone, name + ": no hand");
        for (const auto index : { entry.right, entry.left }) {
            check(index == built_in::kNone || (index >= 0 && static_cast<std::size_t>(index) < poses.size()),
                name + ": hand pose index out of range");
        }
        if (i > 0) {
            const auto& previous = offsets[i - 1];
            check(std::tie(previous.plugin, previous.localFormId, previous.variant) <
                    std::tie(entry.plugin, entry.localFormId, entry.variant),
                name + ": offsets not strictly sorted");
        }
    }
    for (std::size_t i = 0; i < aliases.size(); ++i) {
        const auto& alias = aliases[i];
        check(lowercase(alias.plugin) && alias.offset < offsets.size(), "alias with bad plugin or offset index");
        if (i > 0) {
            check(std::tie(aliases[i - 1].plugin, aliases[i - 1].localFormId) < std::tie(alias.plugin, alias.localFormId),
                "aliases not strictly sorted");
        }
    }

    // Every shipped pose passes the same validation as a saved offset file:
    // finite values, proper rotations, and exact finger rotations on the FRIK rig.
    for (const auto& entry : offsets) {
        for (const bool left : { false, true }) {
            HandOffset hand{};
            const bool expected = (left ? entry.left : entry.right) != built_in::kNone;
            const FormRef ref{ std::string(entry.plugin), entry.localFormId, std::string(entry.variant) };
            check(built_in::find(ref, left, hand) == expected, std::string(entry.plugin) + ": lookup missed its own hand");
            if (!expected) {
                continue;
            }
            SavedGrabOffsetFile file{};
            file.object = ref;
            (left ? file.left : file.right) = hand;
            SavedGrabOffsetFile parsed{};
            std::string error;
            const bool ok = parse(serialize(file), parsed, &error);
            const auto& back = left ? parsed.left : parsed.right;
            check(ok && error.empty() && back.present && back.hasExactFingerRotations == hand.hasExactFingerRotations,
                std::string(entry.plugin) + " " + std::to_string(entry.localFormId) + ": pose fails validation (" +
                    error + ")");
        }
    }

    // Plugin names match case-insensitively, aliases resolve to their source's
    // pose, and variants never borrow another form's pose.
    if (!aliases.empty()) {
        const auto& alias = aliases.front();
        const auto& source = offsets[alias.offset];
        std::string upper(alias.plugin);
        for (auto& c : upper) {
            c = (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c;
        }
        HandOffset viaAlias{};
        HandOffset direct{};
        const bool hasRight = source.right != built_in::kNone;
        check(built_in::find(FormRef{ upper, alias.localFormId, {} }, false, viaAlias) == hasRight,
            "alias lookup is case-sensitive or broken");
        if (hasRight) {
            check(built_in::find(FormRef{ std::string(source.plugin), source.localFormId, std::string(source.variant) },
                      false, direct) &&
                    viaAlias.translateGame[0] == direct.translateGame[0] && viaAlias.rotate[4] == direct.rotate[4],
                "alias does not return its source's pose");
        }
        HandOffset variant{};
        check(!built_in::find(FormRef{ std::string(alias.plugin), alias.localFormId, "single" }, false, variant),
            "a variant borrowed an alias");
    }
    HandOffset missing{};
    check(!built_in::find(FormRef{ "NotAPlugin.esp", 0x00ABCDEF, {} }, false, missing), "unknown form found a pose");

    if (failures) {
        std::cerr << failures << " built-in grab offset failure(s)\n";
        return 1;
    }
    return 0;
}
