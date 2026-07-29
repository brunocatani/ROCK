#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <tuple>
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
        std::size_t operator()(const LineKey& key) const noexcept
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
        bool prepare(std::size_t maxVertices)
        {
            const std::size_t maxLines = maxVertices / 2;
            if (maxLines == 0 || maxLines > (std::numeric_limits<std::size_t>::max)() / 2) {
                return false;
            }

            std::size_t slotCount = 1;
            while (slotCount < maxLines * 2) {
                if (slotCount > (std::numeric_limits<std::size_t>::max)() / 2) {
                    return false;
                }
                slotCount *= 2;
            }

            try {
                _segments.reserve(maxLines);
                _slots.resize(slotCount);
            } catch (...) {
                _segments.clear();
                _slots.clear();
                _preparedMaxVertices = 0;
                return false;
            }
            _preparedMaxVertices = maxLines * 2;
            clear();
            return true;
        }

        void clear()
        {
            _segments.clear();
            _rejectedLines = 0;
            _activeMaxVertices = _preparedMaxVertices;
            if (++_generation == 0) {
                for (auto& slot : _slots) {
                    slot.generation = 0;
                }
                _generation = 1;
            }
        }

        void beginFrame(std::size_t maxVertices)
        {
            clear();
            _activeMaxVertices = (std::min)(maxVertices, _preparedMaxVertices);
        }

        bool addLine(const Vec3& start, const Vec3& end, const Rgba& color)
        {
            return addLine(start, end, color, _activeMaxVertices);
        }

        bool addLine(const Vec3& start, const Vec3& end, const Rgba& color, std::size_t maxVertices)
        {
            if (!finite(start) || !finite(end) || !finite(color)) {
                ++_rejectedLines;
                return false;
            }

            const auto key = makeLineKey(start, end, color);
            if (key.start == key.end || !canAppendLine(maxVertices)) {
                ++_rejectedLines;
                return false;
            }

            if (containsOrInsert(key)) {
                ++_rejectedLines;
                return false;
            }

            _segments.push_back(LineSegment{ start, end, color });
            return true;
        }

        bool addPointMarker(const Vec3& center, float size, const Rgba& color, std::size_t maxVertices)
        {
            const std::size_t effectiveMax = (std::min)(maxVertices, _preparedMaxVertices);
            if (vertexCount() + 6 > effectiveMax) {
                _rejectedLines += 3;
                return false;
            }

            const bool xAdded = addLine(Vec3{ center.x - size, center.y, center.z }, Vec3{ center.x + size, center.y, center.z }, color, maxVertices);
            const bool yAdded = addLine(Vec3{ center.x, center.y - size, center.z }, Vec3{ center.x, center.y + size, center.z }, color, maxVertices);
            const bool zAdded = addLine(Vec3{ center.x, center.y, center.z - size }, Vec3{ center.x, center.y, center.z + size }, color, maxVertices);
            return xAdded || yAdded || zAdded;
        }

        bool addPointMarker(const Vec3& center, float size, const Rgba& color)
        {
            return addPointMarker(center, size, color, _activeMaxVertices);
        }

        [[nodiscard]] std::size_t vertexCount() const { return _segments.size() * 2; }
        [[nodiscard]] std::size_t lineCount() const { return _segments.size(); }
        [[nodiscard]] std::size_t rejectedLineCount() const { return _rejectedLines; }
        [[nodiscard]] bool empty() const { return _segments.empty(); }
        [[nodiscard]] const std::vector<LineSegment>& segments() const { return _segments; }
        [[nodiscard]] std::size_t preparedMaxVertices() const { return _preparedMaxVertices; }

    private:
        struct KeySlot
        {
            LineKey key{};
            std::uint32_t generation{ 0 };
        };

        static bool finite(const Vec3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        static bool finite(const Rgba& value)
        {
            return std::isfinite(value.r) && std::isfinite(value.g) && std::isfinite(value.b) && std::isfinite(value.a);
        }

        bool containsOrInsert(const LineKey& key)
        {
            if (_slots.empty()) {
                return true;
            }

            const std::size_t mask = _slots.size() - 1;
            std::size_t index = LineKeyHash{}(key) & mask;
            for (std::size_t probe = 0; probe < _slots.size(); ++probe) {
                auto& slot = _slots[index];
                if (slot.generation != _generation) {
                    slot.key = key;
                    slot.generation = _generation;
                    return false;
                }
                if (slot.key == key) {
                    return true;
                }
                index = (index + 1) & mask;
            }
            return true;
        }

        [[nodiscard]] bool canAppendLine(std::size_t maxVertices) const
        {
            const std::size_t effectiveMax = (std::min)(maxVertices, _preparedMaxVertices);
            return vertexCount() + 2 <= effectiveMax;
        }

        std::vector<LineSegment> _segments{};
        std::vector<KeySlot> _slots{};
        std::size_t _preparedMaxVertices{ 0 };
        std::size_t _activeMaxVertices{ 0 };
        std::size_t _rejectedLines = 0;
        std::uint32_t _generation{ 0 };
    };
}
