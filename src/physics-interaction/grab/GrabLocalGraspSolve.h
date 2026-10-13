#pragma once

/*
 * Local grasp solve for close grabs (close-grab seat mode 1).
 *
 * The current close-grab seat keeps the object's rotation and slides the mesh
 * point nearest the palm onto the palm centre. It never reads what kind of
 * geometry is in front of the hand, so a mug handle held in front of the palm
 * is seated like any other surface point and the fingers close on the mug
 * body behind it.
 *
 * This policy reads only the mesh inside a hand-sized window around that
 * nearest point, in the palm's own frame, and classifies that piece:
 *
 *   Bar       thin in width and depth with free space around it (handle,
 *             neck, strap, plank). The centreline seats on the palm with the
 *             bar turned onto the hand's grip line. Mesh behind a
 *             finger-sized gap is reported so finger closure can thread it.
 *   Edge      thin in width but deep (plate rim, cup rim, book spine). Same
 *             turn; the rim crest seats on the palm.
 *   Cylinder  compact but too wide to close the hand around (bottle or mug
 *             body). Same turn about the nearest point, so the fingers wrap
 *             its girth.
 *   None      nothing recognised; the caller keeps the current seat.
 *
 * A recognised part does not seat on the flat palm centre, where closed
 * fingers never reach it. It seats against the finger bases, as far back
 * toward the palm centre as its own width needs (fingerBaseSeat).
 *
 * A broad flat face is deliberately left to the current seat. Swinging a face
 * flat onto the palm was tried for arrivals and removed: verified holds of
 * flat objects sit on an edge, well off the palm normal (the plate note in
 * HandGrab.cpp's seat alignment). The Edge class above is that hold.
 *
 * Whole-mesh shape classes cannot do this: a mug is compact as a whole and its
 * handle is a bar only locally. The direction of the part comes from the
 * surface patch nearest the palm that is connected to the nearest point; its
 * outline comes from an exact slice of the mesh through that point, across
 * that direction. Pure value math in the pocket frame - x along
 * the fingers, y across the palm, z out of the palm - with no engine types,
 * configuration, logging or time source. Directions are returned as vectors
 * rather than a rotation so a mirrored hand frame cannot flip the turn. One
 * pass at capture; nothing here runs while an object is held.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace rock::grab_local_grasp
{
    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    enum class Kind : std::uint8_t
    {
        None,
        Bar,
        Edge,
        Cylinder,
    };

    inline const char* kindName(Kind kind)
    {
        switch (kind) {
        case Kind::Bar:
            return "bar";
        case Kind::Edge:
            return "edge";
        case Kind::Cylinder:
            return "cylinder";
        default:
            return "none";
        }
    }

    // Game units. The hand dimensions are the closing span of the FO4VR hand,
    // not tuning: a feature wider or thicker than the wrap limits cannot sit
    // inside closed fingers.
    struct Config
    {
        float windowHalfExtent = 6.0f;
        float windowFrontDepth = 1.5f;
        float windowBackDepth = 10.0f;
        float sampleSpacing = 0.4f;
        std::size_t maxSamples = 48000;
        std::size_t maxWindowTriangles = 32768;
        std::size_t maxSectionPoints = 16384;
        float nearLayerDepth = 1.25f;
        float nearLayerRadius = 5.0f;
        float minElongation = 1.6f;
        // A round body is turned only when it is clearly longer than wide
        // under the hand. A body about as tall as it is wide (a mug) has no
        // wrap direction worth a turn and keeps the current seat.
        float minCylinderElongation = 1.9f;
        float minAxisRmsLength = 0.8f;
        float maxAxisDepthComponent = 0.75f;
        float maxWrapWidth = 3.6f;
        float maxWrapThickness = 3.6f;
        float maxEdgeWidth = 2.5f;
        // Widest compact part an open hand still cups (about 9 cm).
        float maxCupWidth = 6.5f;
        float edgeNearDepth = 2.0f;
        // A feature's crest is the nearest mesh. A steep single face instead
        // keeps running in front of the nearest point until the window clips
        // it, and its clipped side must not read as free space.
        float frontClipMargin = 0.3f;
        float minBarLength = 2.0f;
        float handHalfWidth = 2.0f;
        float maxStationShift = 2.5f;
        float minThreadGap = 1.0f;
        float maxThreadGapCredit = 2.4f;
        // Added to the bar's thickness to keep the arms that close a loop at
        // its ends in finger closure.
        float threadEndMargin = 0.3f;
        float minAlignDegrees = 1.0f;
    };

    struct Result
    {
        Kind kind = Kind::None;
        const char* reason = "notEvaluated";
        // Object point, pocket frame, that must seat on the palm centre.
        Vec3 pivot{};
        // Unit object direction and the unit hand direction it must reach.
        Vec3 alignFrom{};
        Vec3 alignTo{};
        float alignDegrees = 0.0f;
        float elongation = 0.0f;
        float widthGameUnits = 0.0f;
        float thicknessGameUnits = 0.0f;
        float lengthGameUnits = 0.0f;
        // Negative when nothing lies behind the feature inside the window.
        float gapBehindGameUnits = -1.0f;
        float stationShiftGameUnits = 0.0f;
        // Threaded bar, pocket frame at capture: mesh deeper than the limit
        // past the pivot along the depth axis lies behind the gap the fingers
        // pass through. Only the stretch of the bar between its ends is
        // opened, measured from the pivot along the bar.
        Vec3 threadDepthAxis{};
        Vec3 threadAxis{};
        float threadDepthLimitGameUnits = 0.0f;
        float threadStationMin = 0.0f;
        float threadStationMax = 0.0f;
        std::uint32_t sampleCount = 0;
        std::uint32_t nearSampleCount = 0;
        bool hasAlignment = false;
        bool threadFingers = false;
        // The window was analysed. False keeps the caller on the current seat.
        bool valid = false;
    };

    namespace detail
    {
        inline constexpr int kNearCells = 24;
        inline constexpr float kNearCell = 0.5f;
        inline constexpr int kSectionCells = 32;
        inline constexpr float kSectionCell = 0.4f;

        [[nodiscard]] inline Vec3 add(const Vec3& lhs, const Vec3& rhs) { return Vec3{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z }; }
        [[nodiscard]] inline Vec3 sub(const Vec3& lhs, const Vec3& rhs) { return Vec3{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z }; }
        [[nodiscard]] inline Vec3 mul(const Vec3& value, float scalar) { return Vec3{ value.x * scalar, value.y * scalar, value.z * scalar }; }
        [[nodiscard]] inline float dot(const Vec3& lhs, const Vec3& rhs) { return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z; }
        [[nodiscard]] inline Vec3 cross(const Vec3& lhs, const Vec3& rhs)
        {
            return Vec3{ lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x };
        }
        [[nodiscard]] inline float lengthSquared(const Vec3& value) { return dot(value, value); }
        [[nodiscard]] inline bool finite(const Vec3& value) { return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z); }
        [[nodiscard]] inline Vec3 normalizeOrZero(const Vec3& value)
        {
            const float squared = lengthSquared(value);
            if (!std::isfinite(squared) || squared <= 0.000001f) {
                return Vec3{};
            }
            return mul(value, 1.0f / std::sqrt(squared));
        }
        [[nodiscard]] inline float component(const Vec3& value, int axis) { return axis == 0 ? value.x : (axis == 1 ? value.y : value.z); }

        struct Eigen3
        {
            // Descending eigenvalues with their unit eigenvectors.
            std::array<double, 3> values{};
            std::array<Vec3, 3> vectors{};
        };

        // Cyclic Jacobi on a symmetric 3x3. Fixed sweep bound, no allocation.
        [[nodiscard]] inline Eigen3 symmetricEigen(double xx, double xy, double xz, double yy, double yz, double zz)
        {
            double a[3][3] = { { xx, xy, xz }, { xy, yy, yz }, { xz, yz, zz } };
            double v[3][3] = { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } };
            for (int sweep = 0; sweep < 32; ++sweep) {
                const double offDiagonal = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
                if (!(offDiagonal > 1e-20)) {
                    break;
                }
                for (int p = 0; p < 2; ++p) {
                    for (int q = p + 1; q < 3; ++q) {
                        if (!(std::abs(a[p][q]) > 1e-24)) {
                            continue;
                        }
                        const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                        const double t = (theta >= 0.0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
                        const double c = 1.0 / std::sqrt(t * t + 1.0);
                        const double s = t * c;
                        for (int k = 0; k < 3; ++k) {
                            const double akp = a[k][p];
                            const double akq = a[k][q];
                            a[k][p] = c * akp - s * akq;
                            a[k][q] = s * akp + c * akq;
                        }
                        for (int k = 0; k < 3; ++k) {
                            const double apk = a[p][k];
                            const double aqk = a[q][k];
                            a[p][k] = c * apk - s * aqk;
                            a[q][k] = s * apk + c * aqk;
                        }
                        for (int k = 0; k < 3; ++k) {
                            const double vkp = v[k][p];
                            const double vkq = v[k][q];
                            v[k][p] = c * vkp - s * vkq;
                            v[k][q] = s * vkp + c * vkq;
                        }
                    }
                }
            }
            std::array<int, 3> order{ 0, 1, 2 };
            std::sort(order.begin(), order.end(), [&](int lhs, int rhs) { return a[lhs][lhs] > a[rhs][rhs]; });
            Eigen3 result{};
            for (int i = 0; i < 3; ++i) {
                const int column = order[static_cast<std::size_t>(i)];
                result.values[static_cast<std::size_t>(i)] = a[column][column];
                result.vectors[static_cast<std::size_t>(i)] = normalizeOrZero(Vec3{
                    static_cast<float>(v[0][column]),
                    static_cast<float>(v[1][column]),
                    static_cast<float>(v[2][column]),
                });
            }
            return result;
        }

        // Area-weighted first and second moments of a sample set.
        struct Moments
        {
            double weight = 0.0;
            double sx = 0.0, sy = 0.0, sz = 0.0;
            double xx = 0.0, xy = 0.0, xz = 0.0, yy = 0.0, yz = 0.0, zz = 0.0;
            std::uint32_t count = 0;

            void add(const Vec3& point, float sampleWeight)
            {
                const double w = sampleWeight;
                weight += w;
                sx += w * point.x;
                sy += w * point.y;
                sz += w * point.z;
                xx += w * point.x * point.x;
                xy += w * point.x * point.y;
                xz += w * point.x * point.z;
                yy += w * point.y * point.y;
                yz += w * point.y * point.z;
                zz += w * point.z * point.z;
                ++count;
            }

            [[nodiscard]] bool usable(std::uint32_t minimumCount) const { return count >= minimumCount && weight > 1e-9; }

            [[nodiscard]] Vec3 mean() const
            {
                const double inverse = 1.0 / weight;
                return Vec3{ static_cast<float>(sx * inverse), static_cast<float>(sy * inverse), static_cast<float>(sz * inverse) };
            }

            [[nodiscard]] Eigen3 eigen() const
            {
                const double inverse = 1.0 / weight;
                const double mx = sx * inverse;
                const double my = sy * inverse;
                const double mz = sz * inverse;
                return symmetricEigen(
                    xx * inverse - mx * mx, xy * inverse - mx * my, xz * inverse - mx * mz,
                    yy * inverse - my * my, yz * inverse - my * mz, zz * inverse - mz * mz);
            }
        };

        struct Sample
        {
            Vec3 point{};
            float weight = 0.0f;
        };

        // A triangle clipped by the six window planes has at most nine vertices.
        using Polygon = std::array<Vec3, 12>;

        [[nodiscard]] inline std::size_t clipAgainst(Polygon& polygon, std::size_t count, int axis, float limit, bool keepBelow)
        {
            if (count == 0) {
                return 0;
            }
            Polygon clipped{};
            std::size_t clippedCount = 0;
            auto insideDistance = [&](const Vec3& point) {
                const float coordinate = component(point, axis);
                return keepBelow ? limit - coordinate : coordinate - limit;
            };
            Vec3 previous = polygon[count - 1];
            float previousDistance = insideDistance(previous);
            for (std::size_t i = 0; i < count; ++i) {
                const Vec3 current = polygon[i];
                const float currentDistance = insideDistance(current);
                if ((previousDistance < 0.0f) != (currentDistance < 0.0f)) {
                    const float t = previousDistance / (previousDistance - currentDistance);
                    if (clippedCount < clipped.size()) {
                        clipped[clippedCount++] = add(previous, mul(sub(current, previous), t));
                    }
                }
                if (currentDistance >= 0.0f && clippedCount < clipped.size()) {
                    clipped[clippedCount++] = current;
                }
                previous = current;
                previousDistance = currentDistance;
            }
            polygon = clipped;
            return clippedCount;
        }

        [[nodiscard]] inline std::size_t clipToWindow(Polygon& polygon, std::size_t count, const Config& config)
        {
            count = clipAgainst(polygon, count, 0, config.windowHalfExtent, true);
            count = clipAgainst(polygon, count, 0, -config.windowHalfExtent, false);
            count = clipAgainst(polygon, count, 1, config.windowHalfExtent, true);
            count = clipAgainst(polygon, count, 1, -config.windowHalfExtent, false);
            count = clipAgainst(polygon, count, 2, config.windowBackDepth, true);
            count = clipAgainst(polygon, count, 2, -config.windowFrontDepth, false);
            return count;
        }

        // Rows parallel to the longest edge, each filled at the sample spacing.
        // Density follows area, not edge length, so the sliver triangles of a
        // thin tube cost no more than their surface. Each sample carries an
        // equal share of its row's area.
        template <class Emit>
        inline void sampleTriangle(Vec3 a, Vec3 b, Vec3 c, float spacing, Emit emit)
        {
            const float ab = lengthSquared(sub(b, a));
            const float bc = lengthSquared(sub(c, b));
            const float ca = lengthSquared(sub(a, c));
            if (bc >= ab && bc >= ca) {
                const Vec3 first = a;
                a = b;
                b = c;
                c = first;
            } else if (ca >= ab && ca >= bc) {
                const Vec3 first = a;
                a = c;
                c = b;
                b = first;
            }
            const Vec3 base = sub(b, a);
            const float baseLength = std::sqrt(lengthSquared(base));
            const float area = 0.5f * std::sqrt(lengthSquared(cross(base, sub(c, a))));
            if (!std::isfinite(area) || area <= 0.00001f || baseLength <= 0.0001f) {
                return;
            }
            const float height = 2.0f * area / baseLength;
            const int rows = std::clamp(static_cast<int>(std::ceil(height / spacing)), 1, 64);
            for (int row = 0; row < rows; ++row) {
                const float start = static_cast<float>(row) / static_cast<float>(rows);
                const float end = static_cast<float>(row + 1) / static_cast<float>(rows);
                const float middle = 0.5f * (start + end);
                const float rowArea = area * ((1.0f - start) * (1.0f - start) - (1.0f - end) * (1.0f - end));
                const Vec3 left = add(a, mul(sub(c, a), middle));
                const Vec3 right = add(b, mul(sub(c, b), middle));
                const int columns = std::clamp(static_cast<int>(std::ceil(baseLength * (1.0f - middle) / spacing)), 1, 64);
                const float weight = rowArea / static_cast<float>(columns);
                for (int column = 0; column < columns; ++column) {
                    const float along = (static_cast<float>(column) + 0.5f) / static_cast<float>(columns);
                    if (!emit(add(left, mul(sub(right, left), along)), weight)) {
                        return;
                    }
                }
            }
        }

        // Marks with 2 every occupied cell (1) that is 8-connected to the
        // occupied cell nearest (column, row), looking at most `reach` cells
        // away for that start. False when no occupied cell is that near.
        template <std::size_t N>
        [[nodiscard]] inline bool floodFromNearest(std::array<std::uint8_t, N>& grid, int cells, int column, int row, int reach)
        {
            int bestDistance = (std::numeric_limits<int>::max)();
            int start = -1;
            for (int r = (std::max)(0, row - reach); r <= (std::min)(cells - 1, row + reach); ++r) {
                for (int c = (std::max)(0, column - reach); c <= (std::min)(cells - 1, column + reach); ++c) {
                    if (grid[static_cast<std::size_t>(r * cells + c)] == 0) {
                        continue;
                    }
                    const int distance = (c - column) * (c - column) + (r - row) * (r - row);
                    if (distance < bestDistance) {
                        bestDistance = distance;
                        start = r * cells + c;
                    }
                }
            }
            if (start < 0) {
                return false;
            }
            // Each cell is pushed once, when it is marked.
            std::array<std::uint16_t, N> stack{};
            std::size_t stackCount = 0;
            grid[static_cast<std::size_t>(start)] = 2;
            stack[stackCount++] = static_cast<std::uint16_t>(start);
            while (stackCount > 0) {
                const int index = stack[--stackCount];
                const int r = index / cells;
                const int c = index % cells;
                for (int dr = -1; dr <= 1; ++dr) {
                    for (int dc = -1; dc <= 1; ++dc) {
                        const int nextRow = r + dr;
                        const int nextColumn = c + dc;
                        if (nextRow < 0 || nextColumn < 0 || nextRow >= cells || nextColumn >= cells) {
                            continue;
                        }
                        auto& cell = grid[static_cast<std::size_t>(nextRow * cells + nextColumn)];
                        if (cell == 1) {
                            cell = 2;
                            stack[stackCount++] = static_cast<std::uint16_t>(nextRow * cells + nextColumn);
                        }
                    }
                }
            }
            return true;
        }

        // Liang-Barsky clip of a 2D segment to a rectangle. False when
        // nothing of the segment is inside.
        [[nodiscard]] inline bool clipSegment(float& ax, float& ay, float& bx, float& by, float minX, float maxX, float minY, float maxY)
        {
            float enter = 0.0f;
            float leave = 1.0f;
            const float dx = bx - ax;
            const float dy = by - ay;
            auto clip = [&](float direction, float distance) {
                if (std::abs(direction) < 1e-12f) {
                    return distance >= 0.0f;
                }
                const float t = distance / direction;
                if (direction < 0.0f) {
                    if (t > leave) {
                        return false;
                    }
                    enter = (std::max)(enter, t);
                } else {
                    if (t < enter) {
                        return false;
                    }
                    leave = (std::min)(leave, t);
                }
                return true;
            };
            if (!clip(-dx, ax - minX) || !clip(dx, maxX - ax) || !clip(-dy, ay - minY) || !clip(dy, maxY - ay)) {
                return false;
            }
            const float startX = ax + dx * enter;
            const float startY = ay + dy * enter;
            bx = ax + dx * leave;
            by = ay + dy * leave;
            ax = startX;
            ay = startY;
            return true;
        }

        [[nodiscard]] inline Vec3 closestPointOnTriangle(const Vec3& point, const Vec3& a, const Vec3& b, const Vec3& c)
        {
            const Vec3 ab = sub(b, a);
            const Vec3 ac = sub(c, a);
            const Vec3 ap = sub(point, a);
            const float d1 = dot(ab, ap);
            const float d2 = dot(ac, ap);
            if (d1 <= 0.0f && d2 <= 0.0f) {
                return a;
            }
            const Vec3 bp = sub(point, b);
            const float d3 = dot(ab, bp);
            const float d4 = dot(ac, bp);
            if (d3 >= 0.0f && d4 <= d3) {
                return b;
            }
            const float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
                return add(a, mul(ab, d1 / (d1 - d3)));
            }
            const Vec3 cp = sub(point, c);
            const float d5 = dot(ab, cp);
            const float d6 = dot(ac, cp);
            if (d6 >= 0.0f && d5 <= d6) {
                return c;
            }
            const float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
                return add(a, mul(ac, d2 / (d2 - d6)));
            }
            const float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
                return add(b, mul(sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))));
            }
            const float denominator = va + vb + vc;
            if (!(std::abs(denominator) > 1e-20f)) {
                return a;
            }
            const float inverse = 1.0f / denominator;
            return add(a, add(mul(ab, vb * inverse), mul(ac, vc * inverse)));
        }

        [[nodiscard]] inline bool validConfig(const Config& config)
        {
            const float values[] = {
                config.windowHalfExtent, config.windowFrontDepth, config.windowBackDepth, config.sampleSpacing,
                config.nearLayerDepth, config.nearLayerRadius, config.minElongation, config.minCylinderElongation, config.minAxisRmsLength,
                config.maxAxisDepthComponent, config.maxWrapWidth, config.maxWrapThickness,
                config.maxEdgeWidth, config.edgeNearDepth, config.frontClipMargin, config.minBarLength, config.handHalfWidth,
                config.maxStationShift, config.minThreadGap, config.maxThreadGapCredit, config.threadEndMargin,
                config.maxCupWidth, config.minAlignDegrees,
            };
            for (const float value : values) {
                if (!std::isfinite(value) || value <= 0.0f) {
                    return false;
                }
            }
            // Near-layer connectivity assumes neighbouring samples land in
            // neighbouring cells, and that the layer fits its grid.
            return config.sampleSpacing <= 0.8f * kNearCell + 0.0001f &&
                   config.nearLayerRadius <= 0.5f * static_cast<float>(kNearCells) * kNearCell &&
                   config.maxSamples >= 64 && config.maxWindowTriangles >= 1 && config.maxSectionPoints >= 64 &&
                   config.maxAxisDepthComponent < 1.0f;
        }
    }

    struct NearestPoint
    {
        Vec3 point{};
        float distanceGameUnits = 0.0f;
        bool found = false;
    };

    // The mesh point nearest the palm centre (the pocket-frame origin): the
    // seed solve() classifies around. Taken from the same triangles solve()
    // reads, so the seed is always on the analysed surface.
    template <class Triangles, class Project>
    [[nodiscard]] NearestPoint nearestPointToOrigin(const Triangles& triangles, Project project)
    {
        using namespace detail;

        NearestPoint nearest{};
        float bestSquared = (std::numeric_limits<float>::max)();
        for (const auto& triangle : triangles) {
            const Vec3 a = project(triangle.v0);
            const Vec3 b = project(triangle.v1);
            const Vec3 c = project(triangle.v2);
            if (!finite(a) || !finite(b) || !finite(c)) {
                continue;
            }
            // Distance from the origin to the triangle's bounds is a lower
            // bound on its distance to the triangle.
            float boundSquared = 0.0f;
            for (int axis = 0; axis < 3; ++axis) {
                const float low = (std::min)({ component(a, axis), component(b, axis), component(c, axis) });
                const float high = (std::max)({ component(a, axis), component(b, axis), component(c, axis) });
                const float gap = low > 0.0f ? low : (high < 0.0f ? -high : 0.0f);
                boundSquared += gap * gap;
            }
            if (boundSquared >= bestSquared) {
                continue;
            }
            const Vec3 candidate = closestPointOnTriangle(Vec3{}, a, b, c);
            const float squared = lengthSquared(candidate);
            if (squared < bestSquared) {
                bestSquared = squared;
                nearest.point = candidate;
                nearest.found = true;
            }
        }
        if (nearest.found) {
            nearest.distanceGameUnits = std::sqrt(bestSquared);
        }
        return nearest;
    }

    template <class Triangles, class Project>
    [[nodiscard]] Result solve(const Triangles& triangles, const Vec3& seed, const Vec3& gripLine, const Config& config, Project project)
    {
        using namespace detail;

        Result result{};
        const Vec3 line = normalizeOrZero(gripLine);
        if (!finite(seed) || lengthSquared(line) <= 0.0f || !validConfig(config)) {
            result.reason = "invalidLocalGraspInput";
            return result;
        }

        // Window triangles and their surface samples, relative to the seed.
        std::vector<Sample> samples;
        samples.reserve(4096);
        std::vector<std::array<Vec3, 3>> windowTriangles;
        windowTriangles.reserve(1024);
        bool overflow = false;
        for (const auto& triangle : triangles) {
            Polygon polygon{};
            polygon[0] = sub(project(triangle.v0), seed);
            polygon[1] = sub(project(triangle.v1), seed);
            polygon[2] = sub(project(triangle.v2), seed);
            if (!finite(polygon[0]) || !finite(polygon[1]) || !finite(polygon[2])) {
                result.reason = "nonFiniteLocalMesh";
                return result;
            }
            bool outsideWindow = false;
            for (int axis = 0; axis < 3 && !outsideWindow; ++axis) {
                const float low = (std::min)({ component(polygon[0], axis), component(polygon[1], axis), component(polygon[2], axis) });
                const float high = (std::max)({ component(polygon[0], axis), component(polygon[1], axis), component(polygon[2], axis) });
                const float lowLimit = axis == 2 ? -config.windowFrontDepth : -config.windowHalfExtent;
                const float highLimit = axis == 2 ? config.windowBackDepth : config.windowHalfExtent;
                outsideWindow = low > highLimit || high < lowLimit;
            }
            if (outsideWindow) {
                continue;
            }
            if (windowTriangles.size() >= config.maxWindowTriangles) {
                overflow = true;
                break;
            }
            windowTriangles.push_back({ polygon[0], polygon[1], polygon[2] });
            const std::size_t count = clipToWindow(polygon, 3, config);
            for (std::size_t i = 1; i + 1 < count && !overflow; ++i) {
                sampleTriangle(polygon[0], polygon[i], polygon[i + 1], config.sampleSpacing, [&](const Vec3& point, float weight) {
                    if (samples.size() >= config.maxSamples) {
                        overflow = true;
                        return false;
                    }
                    samples.push_back(Sample{ point, weight });
                    return true;
                });
            }
            if (overflow) {
                break;
            }
        }
        result.sampleCount = static_cast<std::uint32_t>(samples.size());
        if (overflow) {
            result.reason = "localWindowBudgetExceeded";
            return result;
        }
        if (samples.size() < 12) {
            result.reason = "noMeshInLocalWindow";
            return result;
        }
        result.valid = true;
        result.pivot = seed;
        result.reason = "noLocalFeature";

        auto setAlignment = [&](const Vec3& from, const Vec3& to) {
            const float cosine = std::clamp(dot(from, to), -1.0f, 1.0f);
            result.alignFrom = from;
            result.alignTo = to;
            result.alignDegrees = std::acos(cosine) * 57.29577951308232f;
            result.hasAlignment = result.alignDegrees >= config.minAlignDegrees;
        };

        // Axis candidate: principal direction of the layer nearest the palm.
        // Only the patch connected to the seed counts, so another part of the
        // object that happens to be as near cannot skew the direction.
        Moments nearLayer{};
        {
            const float nearRadiusSquared = config.nearLayerRadius * config.nearLayerRadius;
            const float nearOrigin = -0.5f * static_cast<float>(kNearCells) * kNearCell;
            std::array<std::uint8_t, kNearCells * kNearCells> nearGrid{};
            auto nearCell = [&](const Vec3& point, std::size_t& index) {
                if (point.z > config.nearLayerDepth || point.x * point.x + point.y * point.y > nearRadiusSquared) {
                    return false;
                }
                const float fc = (point.x - nearOrigin) / kNearCell;
                const float fr = (point.y - nearOrigin) / kNearCell;
                if (!(fc >= 0.0f) || !(fr >= 0.0f) || fc >= static_cast<float>(kNearCells) || fr >= static_cast<float>(kNearCells)) {
                    return false;
                }
                index = static_cast<std::size_t>(static_cast<int>(fr) * kNearCells + static_cast<int>(fc));
                return true;
            };
            for (const auto& sample : samples) {
                std::size_t index = 0;
                if (nearCell(sample.point, index)) {
                    nearGrid[index] = 1;
                }
            }
            if (floodFromNearest(nearGrid, kNearCells, kNearCells / 2, kNearCells / 2, 2)) {
                for (const auto& sample : samples) {
                    std::size_t index = 0;
                    if (nearCell(sample.point, index) && nearGrid[index] == 2) {
                        nearLayer.add(sample.point, sample.weight);
                    }
                }
            }
        }
        result.nearSampleCount = nearLayer.count;
        Vec3 axis{};
        bool axisValid = false;
        if (nearLayer.usable(8)) {
            const Eigen3 eigen = nearLayer.eigen();
            const double major = (std::max)(eigen.values[0], 0.0);
            const double minor = (std::max)(eigen.values[1], 1e-6);
            result.elongation = static_cast<float>(std::sqrt(major / minor));
            axis = eigen.vectors[0];
            if (dot(axis, line) < 0.0f) {
                axis = mul(axis, -1.0f);
            }
            axisValid = lengthSquared(axis) > 0.0f &&
                        result.elongation >= config.minElongation &&
                        std::sqrt(major) >= config.minAxisRmsLength &&
                        std::abs(axis.z) <= config.maxAxisDepthComponent;
        }

        if (axisValid) {
            // Cross-section through the seed, perpendicular to the axis.
            const Vec3 depthAxis = normalizeOrZero(sub(Vec3{ 0.0f, 0.0f, 1.0f }, mul(axis, axis.z)));
            const Vec3 sideAxis = cross(depthAxis, axis);
            const float sideOrigin = -0.5f * static_cast<float>(kSectionCells) * kSectionCell;
            const float depthOrigin = -(config.windowFrontDepth + 0.5f);
            std::array<std::uint8_t, kSectionCells * kSectionCells> grid{};
            auto sectionCell = [&](float side, float depth, int& column, int& row) {
                const float fc = (side - sideOrigin) / kSectionCell;
                const float fr = (depth - depthOrigin) / kSectionCell;
                if (!(fc >= 0.0f) || !(fr >= 0.0f) || fc >= static_cast<float>(kSectionCells) || fr >= static_cast<float>(kSectionCells)) {
                    return false;
                }
                column = static_cast<int>(fc);
                row = static_cast<int>(fr);
                return true;
            };
            auto cellIndex = [](int column, int row) { return static_cast<std::size_t>(row * kSectionCells + column); };

            // Exact slice: every window triangle the plane cuts contributes
            // its cut segment, rasterised at half-cell steps. A slab of
            // surface samples would smear a curved handle into the body
            // behind it.
            struct SectionPoint
            {
                float side = 0.0f;
                float depth = 0.0f;
                float weight = 0.0f;
            };
            std::vector<SectionPoint> sectionPoints;
            sectionPoints.reserve(512);
            const float sideLimit = sideOrigin + static_cast<float>(kSectionCells) * kSectionCell;
            const float depthLimit = depthOrigin + static_cast<float>(kSectionCells) * kSectionCell;
            for (const auto& triangle : windowTriangles) {
                const float stations[3] = { dot(triangle[0], axis), dot(triangle[1], axis), dot(triangle[2], axis) };
                Vec3 cut[2]{};
                int cutCount = 0;
                for (int edge = 0; edge < 3 && cutCount < 2; ++edge) {
                    const int next = (edge + 1) % 3;
                    const float from = stations[edge];
                    const float to = stations[next];
                    if ((from < 0.0f) == (to < 0.0f)) {
                        continue;
                    }
                    cut[cutCount++] = add(triangle[static_cast<std::size_t>(edge)],
                        mul(sub(triangle[static_cast<std::size_t>(next)], triangle[static_cast<std::size_t>(edge)]), from / (from - to)));
                }
                if (cutCount != 2) {
                    continue;
                }
                float side0 = dot(cut[0], sideAxis);
                float depth0 = dot(cut[0], depthAxis);
                float side1 = dot(cut[1], sideAxis);
                float depth1 = dot(cut[1], depthAxis);
                if (!clipSegment(side0, depth0, side1, depth1, sideOrigin, sideLimit, depthOrigin, depthLimit)) {
                    continue;
                }
                const float cutLength = std::hypot(side1 - side0, depth1 - depth0);
                const int steps = std::clamp(static_cast<int>(std::ceil(cutLength / (0.5f * kSectionCell))), 1, 128);
                if (sectionPoints.size() + static_cast<std::size_t>(steps) > config.maxSectionPoints) {
                    result.valid = false;
                    result.reason = "localSectionBudgetExceeded";
                    return result;
                }
                const float weight = (std::max)(cutLength, 0.0001f) / static_cast<float>(steps);
                for (int step = 0; step < steps; ++step) {
                    const float t = (static_cast<float>(step) + 0.5f) / static_cast<float>(steps);
                    const float side = side0 + (side1 - side0) * t;
                    const float depth = depth0 + (depth1 - depth0) * t;
                    int column = 0;
                    int row = 0;
                    if (sectionCell(side, depth, column, row)) {
                        grid[cellIndex(column, row)] = 1;
                        sectionPoints.push_back(SectionPoint{ side, depth, weight });
                    }
                }
            }

            // The connected outline that contains the seed is the feature.
            int seedColumn = 0;
            int seedRow = 0;
            const bool componentFound = sectionCell(0.0f, 0.0f, seedColumn, seedRow) &&
                                        floodFromNearest(grid, kSectionCells, seedColumn, seedRow, 2);

            if (componentFound) {
                constexpr float kUnset = (std::numeric_limits<float>::max)();
                float sideMin = kUnset, sideMax = -kUnset, depthMin = kUnset, depthMax = -kUnset;
                double componentWeight = 0.0, componentSide = 0.0, componentDepth = 0.0;
                for (const auto& point : sectionPoints) {
                    const float side = point.side;
                    const float depth = point.depth;
                    int column = 0;
                    int row = 0;
                    if (!sectionCell(side, depth, column, row) || grid[cellIndex(column, row)] != 2) {
                        continue;
                    }
                    sideMin = (std::min)(sideMin, side);
                    sideMax = (std::max)(sideMax, side);
                    depthMin = (std::min)(depthMin, depth);
                    depthMax = (std::max)(depthMax, depth);
                    componentWeight += point.weight;
                    componentSide += static_cast<double>(point.weight) * side;
                    componentDepth += static_cast<double>(point.weight) * depth;
                }

                if (componentWeight > 1e-9) {
                    const float centreSide = static_cast<float>(componentSide / componentWeight);
                    const float centreDepthOfComponent = static_cast<float>(componentDepth / componentWeight);
                    float nearSideMin = kUnset, nearSideMax = -kUnset;
                    double nearWeight = 0.0, nearSide = 0.0;
                    float behindDepth = kUnset;
                    for (const auto& point : sectionPoints) {
                        const float side = point.side;
                        const float depth = point.depth;
                        int column = 0;
                        int row = 0;
                        const bool inGrid = sectionCell(side, depth, column, row);
                        const bool inComponent = inGrid && grid[cellIndex(column, row)] == 2;
                        if (inComponent) {
                            if (depth <= depthMin + config.edgeNearDepth) {
                                nearSideMin = (std::min)(nearSideMin, side);
                                nearSideMax = (std::max)(nearSideMax, side);
                                nearWeight += point.weight;
                                nearSide += static_cast<double>(point.weight) * side;
                            }
                        } else if (depth > depthMax && side >= sideMin - kSectionCell && side <= sideMax + kSectionCell) {
                            behindDepth = (std::min)(behindDepth, depth);
                        }
                    }

                    const float width = sideMax - sideMin;
                    const float thickness = depthMax - depthMin;
                    const float nearWidth = nearWeight > 1e-9 ? nearSideMax - nearSideMin : width;
                    const bool hasBehind = behindDepth < kUnset;
                    result.widthGameUnits = width;
                    result.thicknessGameUnits = thickness;
                    result.gapBehindGameUnits = hasBehind ? behindDepth - depthMax : -1.0f;

                    if (depthMin <= -config.windowFrontDepth + config.frontClipMargin) {
                        result.reason = "surfaceRunsInFrontOfNearestPoint";
                    } else if (width <= config.maxWrapWidth && thickness <= config.maxWrapThickness) {
                        // Bar. Re-fit the axis on the bar's own samples so
                        // neighbouring mesh at the same depth cannot skew it.
                        const float margin = 0.3f;
                        auto inBar = [&](const Vec3& point) {
                            const float side = dot(point, sideAxis);
                            const float depth = dot(point, depthAxis);
                            return side >= sideMin - margin && side <= sideMax + margin &&
                                   depth >= depthMin - margin && depth <= depthMax + margin;
                        };
                        Moments barMoments{};
                        for (const auto& sample : samples) {
                            if (inBar(sample.point)) {
                                barMoments.add(sample.point, sample.weight);
                            }
                        }
                        Vec3 barAxis = axis;
                        if (barMoments.usable(8)) {
                            Vec3 refined = barMoments.eigen().vectors[0];
                            if (dot(refined, axis) < 0.0f) {
                                refined = mul(refined, -1.0f);
                            }
                            // cos(30 degrees): a larger swing means the bar's own
                            // samples disagree with the layer that found it.
                            if (dot(refined, axis) >= 0.8660254f && std::abs(refined.z) <= config.maxAxisDepthComponent) {
                                barAxis = refined;
                            }
                        }
                        float stationMin = kUnset, stationMax = -kUnset;
                        for (const auto& sample : samples) {
                            if (inBar(sample.point)) {
                                const float station = dot(sample.point, barAxis);
                                stationMin = (std::min)(stationMin, station);
                                stationMax = (std::max)(stationMax, station);
                            }
                        }
                        const float length = stationMax - stationMin;
                        result.lengthGameUnits = length;
                        if (length >= config.minBarLength) {
                            // Keep the hand's width on the bar when the nearest
                            // point is close to one of its ends.
                            float station = 0.0f;
                            if (length >= 2.0f * config.handHalfWidth) {
                                station = std::clamp(0.0f, stationMin + config.handHalfWidth, stationMax - config.handHalfWidth);
                            } else {
                                station = 0.5f * (stationMin + stationMax);
                            }
                            station = std::clamp(station, -config.maxStationShift, config.maxStationShift);
                            result.kind = Kind::Bar;
                            result.reason = hasBehind ? "barWithMeshBehind" : "freeBar";
                            result.stationShiftGameUnits = station;
                            const Vec3 pivotFromSeed = add(mul(barAxis, station),
                                add(mul(sideAxis, centreSide), mul(depthAxis, centreDepthOfComponent)));
                            result.pivot = add(seed, pivotFromSeed);
                            // A loop is open between the arms that close it at
                            // the bar's ends; an opening shorter than a finger
                            // is not threaded.
                            const float endMargin = thickness + config.threadEndMargin;
                            const float openMin = stationMin - station + endMargin;
                            const float openMax = stationMax - station - endMargin;
                            if (hasBehind && result.gapBehindGameUnits >= config.minThreadGap &&
                                openMax - openMin >= config.minThreadGap) {
                                result.threadFingers = true;
                                result.threadDepthAxis = depthAxis;
                                result.threadAxis = barAxis;
                                result.threadDepthLimitGameUnits = depthMax - dot(pivotFromSeed, depthAxis) +
                                    0.5f * (std::min)(result.gapBehindGameUnits, config.maxThreadGapCredit);
                                result.threadStationMin = openMin;
                                result.threadStationMax = openMax;
                            }
                            setAlignment(barAxis, line);
                            return result;
                        }
                        result.reason = "barTooShortToWrap";
                    } else if (nearWidth <= config.maxEdgeWidth && thickness > config.maxWrapThickness) {
                        result.kind = Kind::Edge;
                        result.reason = "thinDeepEdge";
                        result.widthGameUnits = nearWidth;
                        const float crestSide = nearWeight > 1e-9 ? static_cast<float>(nearSide / nearWeight) : centreSide;
                        result.pivot = add(seed, add(mul(sideAxis, crestSide), mul(depthAxis, depthMin)));
                        setAlignment(axis, line);
                        return result;
                    } else if (width <= config.maxCupWidth && thickness <= config.maxCupWidth) {
                        if (result.elongation >= config.minCylinderElongation) {
                            result.kind = Kind::Cylinder;
                            result.reason = "roundBody";
                            setAlignment(axis, line);
                            return result;
                        }
                        result.reason = "roundBodyHasNoClearDirection";
                    } else {
                        result.reason = "partTooWideForHand";
                    }
                } else {
                    result.reason = "emptySeedSection";
                }
            } else {
                result.reason = "seedNotInSection";
            }
        }

        if (!axisValid) {
            result.reason = "nearSurfaceHasNoDirection";
        }
        return result;
    }

    // How far along the fingers from the palm centre the seated part's pivot
    // belongs, in game units. Closing fingers trap a part in the corner
    // between the palm and their first segments, so the part lies along the
    // grip line one finger radius plus its own half width short of the finger
    // bases. A part too wide for that corner ends up on the palm centre,
    // which is the current seat. `fingerBaseCentre` is the mean base joint of
    // the four fingers in the pocket frame.
    [[nodiscard]] inline float fingerBaseSeat(const Result& result, const Vec3& fingerBaseCentre, float fingerRadius, const Vec3& gripLine)
    {
        using namespace detail;

        if (result.kind == Kind::None || !finite(fingerBaseCentre) || !std::isfinite(fingerRadius) || fingerRadius < 0.0f ||
            !std::isfinite(result.widthGameUnits) || !(fingerBaseCentre.x > 0.0f)) {
            return 0.0f;
        }
        const Vec3 line = normalizeOrZero(Vec3{ gripLine.x, gripLine.y, 0.0f });
        // The grip line must run across the palm to pass the finger bases.
        if (std::abs(line.y) < 0.5f) {
            return 0.0f;
        }
        const float alongAtBases = fingerBaseCentre.x - fingerRadius - 0.5f * (std::max)(result.widthGameUnits, 0.0f);
        // The pivot sits where that line crosses the palm centre's
        // cross-palm coordinate.
        const float alongAtPalmCentre = alongAtBases - line.x * (fingerBaseCentre.y / line.y);
        return std::clamp(alongAtPalmCentre, 0.0f, fingerBaseCentre.x);
    }

    // True when a triangle, given in the pocket frame at capture, lies wholly
    // behind the gap of a threaded bar and across its opening. Finger closure
    // drops these so the fingers wrap the bar instead of stopping on the body
    // behind it. The arms that close the loop at the bar's ends, and anything
    // that crosses the gap, stay and still stop a finger.
    [[nodiscard]] inline bool behindThreadGap(const Result& result, const Vec3& a, const Vec3& b, const Vec3& c)
    {
        if (!result.threadFingers) {
            return false;
        }
        const Vec3 fromPivot[3] = { detail::sub(a, result.pivot), detail::sub(b, result.pivot), detail::sub(c, result.pivot) };
        float stationLow = (std::numeric_limits<float>::max)();
        float stationHigh = -(std::numeric_limits<float>::max)();
        for (const Vec3& point : fromPivot) {
            if (!(detail::dot(point, result.threadDepthAxis) > result.threadDepthLimitGameUnits)) {
                return false;
            }
            const float station = detail::dot(point, result.threadAxis);
            stationLow = (std::min)(stationLow, station);
            stationHigh = (std::max)(stationHigh, station);
        }
        return stationHigh > result.threadStationMin && stationLow < result.threadStationMax;
    }
}
