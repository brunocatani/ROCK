#pragma once

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <functional>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

namespace rock::debug_overlay_line_batch
{
    /*
     * Debug line batching exists to keep diagnostic richness from multiplying
     * D3D Map/Draw calls. The renderer still owns GPU resources; this value
     * layer only accounts for logical lines and enforces a hard vertex budget
     * before the VR submit path receives the frame.
     */
    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct Rgba
    {
        float r = 1.0f;
        float g = 1.0f;
        float b = 1.0f;
        float a = 1.0f;

        bool operator==(const Rgba&) const = default;
    };

    struct LineSegment
    {
        Vec3 start{};
        Vec3 end{};
        Rgba color{};
    };

    struct QuantizedVec3
    {
        std::int64_t x = 0;
        std::int64_t y = 0;
        std::int64_t z = 0;

        bool operator==(const QuantizedVec3&) const = default;
        bool operator<(const QuantizedVec3& rhs) const { return std::tie(x, y, z) < std::tie(rhs.x, rhs.y, rhs.z); }
    };

    struct QuantizedRgba
    {
        std::int64_t r = 0;
        std::int64_t g = 0;
        std::int64_t b = 0;
        std::int64_t a = 0;

        bool operator==(const QuantizedRgba&) const = default;
    };

    struct LineKey
    {
        QuantizedVec3 start{};
        QuantizedVec3 end{};
        QuantizedRgba color{};

        bool operator==(const LineKey&) const = default;
    };

    struct LineKeyHash
    {
        std::size_t operator()(const LineKey& key) const
        {
            std::size_t h = 0;
            const auto mix = [&h](std::int64_t value) {
                h ^= std::hash<std::int64_t>{}(value) + 0x9e3779b9 + (h << 6) + (h >> 2);
            };
            mix(key.start.x);
            mix(key.start.y);
            mix(key.start.z);
            mix(key.end.x);
            mix(key.end.y);
            mix(key.end.z);
            mix(key.color.r);
            mix(key.color.g);
            mix(key.color.b);
            mix(key.color.a);
            return h;
        }
    };

    inline QuantizedVec3 quantizeVec3(const Vec3& value)
    {
        constexpr float scale = 1000.0f;
        return QuantizedVec3{
            static_cast<std::int64_t>(std::llround(value.x * scale)),
            static_cast<std::int64_t>(std::llround(value.y * scale)),
            static_cast<std::int64_t>(std::llround(value.z * scale)),
        };
    }

    inline QuantizedRgba quantizeRgba(const Rgba& value)
    {
        constexpr float scale = 1000.0f;
        return QuantizedRgba{
            static_cast<std::int64_t>(std::llround(value.r * scale)),
            static_cast<std::int64_t>(std::llround(value.g * scale)),
            static_cast<std::int64_t>(std::llround(value.b * scale)),
            static_cast<std::int64_t>(std::llround(value.a * scale)),
        };
    }

    inline LineKey makeLineKey(const Vec3& start, const Vec3& end, const Rgba& color)
    {
        auto qStart = quantizeVec3(start);
        auto qEnd = quantizeVec3(end);
        if (qEnd < qStart) {
            std::swap(qStart, qEnd);
        }

        return LineKey{ qStart, qEnd, quantizeRgba(color) };
    }

    class LineBatch
    {
    public:
        void clear()
        {
            _segments.clear();
            _lineKeys.clear();
            _rejectedLines = 0;
        }

        bool addLine(const Vec3& start, const Vec3& end, const Rgba& color, std::size_t maxVertices)
        {
            const auto key = makeLineKey(start, end, color);
            if (key.start == key.end || _lineKeys.find(key) != _lineKeys.end()) {
                ++_rejectedLines;
                return false;
            }

            if (!canAppendLine(maxVertices)) {
                ++_rejectedLines;
                return false;
            }

            _lineKeys.insert(key);
            _segments.push_back(LineSegment{ start, end, color });
            return true;
        }

        bool addPointMarker(const Vec3& center, float size, const Rgba& color, std::size_t maxVertices)
        {
            if (vertexCount() + 6 > maxVertices) {
                _rejectedLines += 3;
                return false;
            }

            const bool xAdded = addLine(Vec3{ center.x - size, center.y, center.z }, Vec3{ center.x + size, center.y, center.z }, color, maxVertices);
            const bool yAdded = addLine(Vec3{ center.x, center.y - size, center.z }, Vec3{ center.x, center.y + size, center.z }, color, maxVertices);
            const bool zAdded = addLine(Vec3{ center.x, center.y, center.z - size }, Vec3{ center.x, center.y, center.z + size }, color, maxVertices);
            return xAdded || yAdded || zAdded;
        }

        [[nodiscard]] std::size_t vertexCount() const { return _segments.size() * 2; }
        [[nodiscard]] std::size_t lineCount() const { return _segments.size(); }
        [[nodiscard]] std::size_t rejectedLineCount() const { return _rejectedLines; }
        [[nodiscard]] bool empty() const { return _segments.empty(); }
        [[nodiscard]] const std::vector<LineSegment>& segments() const { return _segments; }

    private:
        [[nodiscard]] bool canAppendLine(std::size_t maxVertices) const { return vertexCount() + 2 <= maxVertices; }

        std::vector<LineSegment> _segments{};
        std::unordered_set<LineKey, LineKeyHash> _lineKeys{};
        std::size_t _rejectedLines = 0;
    };
}
