// Collision filter infos of the selection queries.
#pragma once

#include <cstdint>

namespace rock::selection_query_policy
{
    inline constexpr std::uint32_t kShapeCastFilterInfo = 0x000B002D;
    inline constexpr std::uint32_t kFarClipRayFilterInfo = 0x02420028;
}
