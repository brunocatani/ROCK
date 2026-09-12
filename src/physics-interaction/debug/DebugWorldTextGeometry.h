#pragma once

#include <array>
#include <cmath>

namespace rock::debug_world_text_geometry
{
    template<class Point>
    bool validBasis(const Point& anchor, const Point& right, const Point& down, float size) noexcept
    {
        const auto finite = [](const Point& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); };
        const auto cx = right.y * down.z - right.z * down.y;
        const auto cy = right.z * down.x - right.x * down.z;
        const auto cz = right.x * down.y - right.y * down.x;
        return finite(anchor) && finite(right) && finite(down) && std::isfinite(size) && size > 0 &&
            std::isfinite(cx * cx + cy * cy + cz * cz) && cx * cx + cy * cy + cz * cz > 0.000001f;
    }

    // One shared world quad. The existing stereo vertex shader projects its
    // corners independently; no per-eye pixel centers or pixel offsets.
    template<class Point>
    std::array<Point, 4> pixelCorners(const Point& anchor, const Point& right, const Point& down,
        float x, float y, float size) noexcept
    {
        const auto at = [&](float px, float py) {
            return Point{ anchor.x + right.x * px + down.x * py,
                anchor.y + right.y * px + down.y * py, anchor.z + right.z * px + down.z * py };
        };
        return { at(x, y), at(x + size, y), at(x + size, y + size), at(x, y + size) };
    }
}
