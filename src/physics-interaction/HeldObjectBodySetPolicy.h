#pragma once

#include <cstdint>
#include <unordered_set>
#include <vector>

namespace frik::rock::held_object_body_set_policy
{
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;

    inline std::vector<std::uint32_t> makePrimaryFirstUniqueBodyList(std::uint32_t primaryBodyId, const std::vector<std::uint32_t>& heldBodyIds)
    {
        std::vector<std::uint32_t> result;
        std::unordered_set<std::uint32_t> seen;
        result.reserve(heldBodyIds.size() + 1);

        auto append = [&](std::uint32_t bodyId) {
            if (bodyId == kInvalidBodyId || !seen.insert(bodyId).second) {
                return;
            }
            result.push_back(bodyId);
        };

        append(primaryBodyId);
        for (const auto bodyId : heldBodyIds) {
            append(bodyId);
        }
        return result;
    }

    inline bool containsBody(const std::vector<std::uint32_t>& bodyIds, std::uint32_t bodyId)
    {
        if (bodyId == kInvalidBodyId) {
            return false;
        }
        for (const auto heldBodyId : bodyIds) {
            if (heldBodyId == bodyId) {
                return true;
            }
        }
        return false;
    }

    inline bool containsAnyBody(const std::vector<std::uint32_t>& first,
        const std::vector<std::uint32_t>& second,
        std::uint32_t bodyId)
    {
        return containsBody(first, bodyId) || containsBody(second, bodyId);
    }
}
