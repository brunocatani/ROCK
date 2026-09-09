#pragma once

#include <array>
#include <cstdint>
#include <span>

namespace rock::segment_visibility
{
    inline constexpr std::size_t kMaxSegments = 512;
    struct Segment
    {
        std::uint32_t firstIndex = 0;
        std::uint32_t triangles = 0;
        std::uint32_t parent = 0xFFFFFFFF;
        std::uint32_t children = 0;
        std::uint8_t useChildren = 0;
        std::array<std::uint8_t, 3> padding{};
    };
    static_assert(sizeof(Segment) == 0x14);
    struct Range { std::uint32_t first = 0, count = 0; };
    struct VisibleRanges
    {
        std::array<Range, kMaxSegments> ranges{};
        std::size_t count = 0;
        bool valid = false;
        bool contains(std::uint32_t triangle) const noexcept
        {
            for (std::size_t i=0; i<count; ++i) {
                if (triangle >= ranges[i].first && triangle-ranges[i].first < ranges[i].count) return true;
            }
            return false;
        }
    };

    // Native 141C1D200/141C1D360 choose a whole segment or its children.
    // 141C1D3E0 and draw consumer 141DA03D0 establish index offset / triangle count.
    inline VisibleRanges resolve(std::span<const Segment> segments, std::span<const std::uint32_t> roots,
        std::uint32_t triangleCount) noexcept
    {
        VisibleRanges result{};
        if (segments.size()>kMaxSegments || roots.size()>kMaxSegments) return result;
        std::array<std::uint32_t,kMaxSegments> queue{};
        std::array<bool,kMaxSegments> seen{};
        std::size_t read=0, count=0;
        auto add = [&](std::uint32_t index) {
            if (index>=segments.size()) return false;
            if (!seen[index]) { seen[index]=true; queue[count++]=index; }
            return true;
        };
        for (auto root:roots) if (!add(root)) return result;
        while (read<count) {
            const auto index=queue[read++];
            const auto& segment=segments[index];
            if (!segment.useChildren && segment.triangles) {
                const auto first=segment.firstIndex/3;
                if (segment.firstIndex%3 || first>triangleCount || segment.triangles>triangleCount-first) return result;
                result.ranges[result.count++]={first,segment.triangles};
            } else {
                if (segment.children>segments.size()-index-1) return result;
                for (std::uint32_t i=0;i<segment.children;++i) if (!add(index+1+i)) return result;
            }
        }
        result.valid=true;
        return result;
    }
}
