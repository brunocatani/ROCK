#include "physics-interaction/grab/BuiltInGrabOffsets.h"

#include <algorithm>
#include <iterator>
#include <string>
#include <tuple>

namespace rock::saved_grab_offset::built_in
{
    namespace
    {
        std::string lowercase(std::string_view text)
        {
            std::string out(text);
            for (auto& c : out) {
                if (c >= 'A' && c <= 'Z') {
                    c = static_cast<char>(c - 'A' + 'a');
                }
            }
            return out;
        }

        bool copyHand(std::int32_t index, HandOffset& out)
        {
            const auto poses = handPoses();
            if (index < 0 || static_cast<std::size_t>(index) >= poses.size()) {
                return false;
            }
            const auto& pose = poses[static_cast<std::size_t>(index)];
            out = {};
            out.present = true;
            std::copy(std::begin(pose.translateGame), std::end(pose.translateGame), out.translateGame);
            std::copy(std::begin(pose.rotate), std::end(pose.rotate), out.rotate);
            out.hasFingerPose = (pose.flags & kFingerValues) != 0;
            std::copy(std::begin(pose.fingerValues), std::end(pose.fingerValues), out.fingerValues);
            out.hasFingerJointValues = out.hasFingerPose && (pose.flags & kFingerJointValues) != 0;
            std::copy(std::begin(pose.fingerJointValues), std::end(pose.fingerJointValues), out.fingerJointValues);
            const auto exact = exactFingerRotations();
            if (pose.exactFingerRotations >= 0 && static_cast<std::size_t>(pose.exactFingerRotations) < exact.size()) {
                const auto& source = exact[static_cast<std::size_t>(pose.exactFingerRotations)];
                for (std::size_t joint = 0; joint < 15; ++joint) {
                    std::copy(std::begin(source.rotations[joint]), std::end(source.rotations[joint]),
                        out.exactFingerRotations[joint]);
                }
                out.hasExactFingerRotations = out.hasFingerJointValues;
            }
            return true;
        }
    }

    bool find(const FormRef& object, bool left, HandOffset& out)
    {
        if (object.empty()) {
            return false;
        }
        const std::string plugin = lowercase(object.plugin);
        const auto key = std::tuple<std::string_view, std::uint32_t, std::string_view>(plugin, object.localFormId,
            object.variant);

        const auto table = offsets();
        const auto offsetKey = [](const Offset& entry) {
            return std::tuple<std::string_view, std::uint32_t, std::string_view>(entry.plugin, entry.localFormId,
                entry.variant);
        };
        auto it = std::lower_bound(table.begin(), table.end(), key,
            [&](const Offset& entry, const auto& value) { return offsetKey(entry) < value; });
        const Offset* match = it != table.end() && offsetKey(*it) == key ? &*it : nullptr;

        if (!match && object.variant.empty()) {
            const auto aliasTable = aliases();
            const auto aliasKey = std::pair<std::string_view, std::uint32_t>(plugin, object.localFormId);
            const auto alias = std::lower_bound(aliasTable.begin(), aliasTable.end(), aliasKey,
                [](const Alias& entry, const auto& value) {
                    return std::pair<std::string_view, std::uint32_t>(entry.plugin, entry.localFormId) < value;
                });
            if (alias != aliasTable.end() && alias->plugin == plugin && alias->localFormId == object.localFormId &&
                alias->offset < table.size()) {
                match = &table[alias->offset];
            }
        }
        return match && copyHand(left ? match->left : match->right, out);
    }

    bool findNamed(std::string_view name, bool left, HandOffset& out)
    {
        const auto table = offsets();
        for (const auto& named : namedOffsets()) {
            if (named.name == name && named.offset < table.size()) {
                const auto& entry = table[named.offset];
                return copyHand(left ? entry.left : entry.right, out);
            }
        }
        return false;
    }
}
