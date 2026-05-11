#pragma once

/*
 * Weapon geometry helpers are grouped here because collision hull construction and interaction probe math are one geometry policy surface.
 */


// ---- WeaponCollisionGeometryMath.h ----

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace rock::weapon_collision_geometry_math
{
    /*
     * Weapon mesh collision is generated in weapon-root local space so every
     * hull follows the firearm as one package. FO4VR firearms can have many
     * visible mod parts, but those parts still need one coherent package frame
     * rather than independent source-node frames.
     *
     * FO4VR's native transform compose helper applies child-local vectors
     * through the parent's column basis: world.x = local dot column0, not row0.
     * The hand collider path already uses this convention in PalmTransform.
     * Generated weapon hulls must use one coherent package frame for both
     * center placement and body orientation; mixing column-space centers with
     * row-space body rotation makes every hull spin around its own center
     * instead of moving as one weapon package.
     */
    template <class Matrix>
    inline Matrix transposeRotation(const Matrix& matrix)
    {
        Matrix result{};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                result.entry[row][col] = matrix.entry[col][row];
            }
            result.entry[row][3] = 0.0f;
        }
        return result;
    }

    template <class Matrix, class Vector>
    inline Vector rotateLocalPoint(const Matrix& matrix, const Vector& point)
    {
        return Vector{
            matrix.entry[0][0] * point.x + matrix.entry[1][0] * point.y + matrix.entry[2][0] * point.z,
            matrix.entry[0][1] * point.x + matrix.entry[1][1] * point.y + matrix.entry[2][1] * point.z,
            matrix.entry[0][2] * point.x + matrix.entry[1][2] * point.y + matrix.entry[2][2] * point.z,
        };
    }

    template <class Matrix, class Vector>
    inline Vector localPointToWorld(const Matrix& rootRotation, const Vector& rootTranslation, float rootScale, const Vector& localPoint)
    {
        const Vector scaled{ localPoint.x * rootScale, localPoint.y * rootScale, localPoint.z * rootScale };
        const Vector rotated = rotateLocalPoint(rootRotation, scaled);
        return Vector{ rootTranslation.x + rotated.x, rootTranslation.y + rotated.y, rootTranslation.z + rotated.z };
    }

    template <class Matrix, class Vector>
    inline Vector worldPointToLocal(const Matrix& rootRotation, const Vector& rootTranslation, float rootScale, const Vector& worldPoint)
    {
        const Vector delta{ worldPoint.x - rootTranslation.x, worldPoint.y - rootTranslation.y, worldPoint.z - rootTranslation.z };
        const float invScale = std::abs(rootScale) > 1.0e-6f ? 1.0f / rootScale : 1.0f;
        return Vector{
            (rootRotation.entry[0][0] * delta.x + rootRotation.entry[0][1] * delta.y + rootRotation.entry[0][2] * delta.z) * invScale,
            (rootRotation.entry[1][0] * delta.x + rootRotation.entry[1][1] * delta.y + rootRotation.entry[1][2] * delta.z) * invScale,
            (rootRotation.entry[2][0] * delta.x + rootRotation.entry[2][1] * delta.y + rootRotation.entry[2][2] * delta.z) * invScale,
        };
    }

    template <class Vector>
    inline Vector pointMin(const Vector& a, const Vector& b)
    {
        return Vector{ (std::min)(a.x, b.x), (std::min)(a.y, b.y), (std::min)(a.z, b.z) };
    }

    template <class Vector>
    inline Vector pointMax(const Vector& a, const Vector& b)
    {
        return Vector{ (std::max)(a.x, b.x), (std::max)(a.y, b.y), (std::max)(a.z, b.z) };
    }

    template <class Vector>
    inline Vector pointCenter(const std::vector<Vector>& points)
    {
        if (points.empty()) {
            return Vector{};
        }

        Vector minPoint = points.front();
        Vector maxPoint = points.front();
        for (const auto& point : points) {
            minPoint = pointMin(minPoint, point);
            maxPoint = pointMax(maxPoint, point);
        }

        return Vector{ (minPoint.x + maxPoint.x) * 0.5f, (minPoint.y + maxPoint.y) * 0.5f, (minPoint.z + maxPoint.z) * 0.5f };
    }

    template <class Vector>
    inline float pointAxisValue(const Vector& point, int axis)
    {
        switch (axis) {
        case 0:
            return point.x;
        case 1:
            return point.y;
        default:
            return point.z;
        }
    }

    template <class Vector>
    inline int longestAxis(const std::vector<Vector>& points)
    {
        if (points.empty()) {
            return 0;
        }

        Vector minPoint = points.front();
        Vector maxPoint = points.front();
        for (const auto& point : points) {
            minPoint = pointMin(minPoint, point);
            maxPoint = pointMax(maxPoint, point);
        }

        const float dx = maxPoint.x - minPoint.x;
        const float dy = maxPoint.y - minPoint.y;
        const float dz = maxPoint.z - minPoint.z;
        if (dy >= dx && dy >= dz) {
            return 1;
        }
        if (dz >= dx && dz >= dy) {
            return 2;
        }
        return 0;
    }

    template <class Vector>
    inline std::vector<Vector> limitPointCloud(std::vector<Vector> points, std::size_t maxPoints)
    {
        if (points.size() <= maxPoints || maxPoints == 0) {
            return points;
        }

        const int axis = longestAxis(points);
        std::stable_sort(points.begin(), points.end(), [axis](const Vector& a, const Vector& b) { return pointAxisValue(a, axis) < pointAxisValue(b, axis); });

        std::vector<Vector> result;
        result.reserve(maxPoints);
        for (std::size_t i = 0; i < maxPoints; ++i) {
            const std::size_t srcIndex = (i * (points.size() - 1)) / (maxPoints - 1);
            result.push_back(points[srcIndex]);
        }
        return result;
    }

    template <class Vector>
    inline void splitOversizedCluster(const std::vector<Vector>& points, std::size_t maxPoints, std::vector<std::vector<Vector>>& outClusters)
    {
        if (points.size() <= maxPoints || maxPoints < 4) {
            outClusters.push_back(points);
            return;
        }

        std::vector<Vector> sorted = points;
        const int axis = longestAxis(sorted);
        std::stable_sort(sorted.begin(), sorted.end(), [axis](const Vector& a, const Vector& b) { return pointAxisValue(a, axis) < pointAxisValue(b, axis); });

        const auto mid = sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2);
        std::vector<Vector> left(sorted.begin(), mid);
        std::vector<Vector> right(mid, sorted.end());
        splitOversizedCluster(left, maxPoints, outClusters);
        splitOversizedCluster(right, maxPoints, outClusters);
    }

    template <class Vector>
    struct PointBounds
    {
        Vector min{};
        Vector max{};
        bool valid{ false };
    };

    template <class Vector>
    inline PointBounds<Vector> pointBounds(const std::vector<Vector>& points)
    {
        PointBounds<Vector> bounds{};
        if (points.empty()) {
            return bounds;
        }

        bounds.min = points.front();
        bounds.max = points.front();
        bounds.valid = true;
        for (const auto& point : points) {
            bounds.min = pointMin(bounds.min, point);
            bounds.max = pointMax(bounds.max, point);
        }
        return bounds;
    }

    template <class Vector>
    inline float boundsAxisValue(const PointBounds<Vector>& bounds, int axis, bool maximum)
    {
        return pointAxisValue(maximum ? bounds.max : bounds.min, axis);
    }

    template <class Vector>
    inline float boundsAxisSpan(const PointBounds<Vector>& bounds, int axis)
    {
        if (!bounds.valid) {
            return 0.0f;
        }
        return boundsAxisValue(bounds, axis, true) - boundsAxisValue(bounds, axis, false);
    }

    template <class Vector>
    inline int longestAxisForBounds(const PointBounds<Vector>& bounds)
    {
        if (!bounds.valid) {
            return 0;
        }

        const float dx = boundsAxisSpan(bounds, 0);
        const float dy = boundsAxisSpan(bounds, 1);
        const float dz = boundsAxisSpan(bounds, 2);
        if (dy >= dx && dy >= dz) {
            return 1;
        }
        if (dz >= dx && dz >= dy) {
            return 2;
        }
        return 0;
    }

    template <class Vector>
    inline float largestCrossAxisSpan(const PointBounds<Vector>& bounds, int longAxis)
    {
        float largest = 0.0f;
        for (int axis = 0; axis < 3; ++axis) {
            if (axis == longAxis) {
                continue;
            }
            largest = (std::max)(largest, boundsAxisSpan(bounds, axis));
        }
        return largest;
    }

    template <class Vector>
    inline float pointCloudDiagonalSquaredGeneric(const std::vector<Vector>& points)
    {
        const auto bounds = pointBounds(points);
        if (!bounds.valid) {
            return 0.0f;
        }

        const float dx = bounds.max.x - bounds.min.x;
        const float dy = bounds.max.y - bounds.min.y;
        const float dz = bounds.max.z - bounds.min.z;
        return dx * dx + dy * dy + dz * dz;
    }

    struct SupportDirection
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    template <class Vector>
    struct ConvexSupportFitResult
    {
        std::vector<Vector> points;
        std::size_t inputPointCount = 0;
        std::size_t targetPointCount = 0;
        std::size_t maxPointCount = 0;
        std::size_t selectedPointCount = 0;
        std::size_t repairPointCount = 0;
        std::size_t validationDirectionCount = 0;
        float maxSupportError = 0.0f;
        bool attempted = false;
        bool accepted = false;
        bool usedReduction = false;
        bool reachedMaxPoints = false;
    };

    inline float supportDirectionLengthSquared(const SupportDirection& direction)
    {
        return direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
    }

    inline SupportDirection normalizeSupportDirection(SupportDirection direction)
    {
        const float lengthSquared = supportDirectionLengthSquared(direction);
        if (lengthSquared <= 1.0e-10f || !std::isfinite(lengthSquared)) {
            return {};
        }

        const float invLength = 1.0f / std::sqrt(lengthSquared);
        return SupportDirection{ direction.x * invLength, direction.y * invLength, direction.z * invLength };
    }

    inline float supportDirectionDot(const SupportDirection& a, const SupportDirection& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline SupportDirection supportDirectionCross(const SupportDirection& a, const SupportDirection& b)
    {
        return SupportDirection{
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x,
        };
    }

    inline SupportDirection supportDirectionScale(const SupportDirection& direction, float scale)
    {
        return SupportDirection{ direction.x * scale, direction.y * scale, direction.z * scale };
    }

    inline SupportDirection supportDirectionSub(const SupportDirection& a, const SupportDirection& b)
    {
        return SupportDirection{ a.x - b.x, a.y - b.y, a.z - b.z };
    }

    inline void appendSupportDirection(std::vector<SupportDirection>& directions, float x, float y, float z)
    {
        const auto normalized = normalizeSupportDirection(SupportDirection{ x, y, z });
        if (supportDirectionLengthSquared(normalized) <= 0.0f) {
            return;
        }

        for (const auto& existing : directions) {
            if (supportDirectionDot(existing, normalized) > 0.9995f) {
                return;
            }
        }

        directions.push_back(normalized);
    }

    inline void appendSignedSupportDirection(std::vector<SupportDirection>& directions, const SupportDirection& direction)
    {
        appendSupportDirection(directions, direction.x, direction.y, direction.z);
        appendSupportDirection(directions, -direction.x, -direction.y, -direction.z);
    }

    template <class Vector>
    inline float pointSupportDot(const Vector& point, const SupportDirection& direction)
    {
        return point.x * direction.x + point.y * direction.y + point.z * direction.z;
    }

    inline SupportDirection multiplySymmetric3(const std::array<std::array<float, 3>, 3>& matrix, const SupportDirection& direction)
    {
        return SupportDirection{
            matrix[0][0] * direction.x + matrix[0][1] * direction.y + matrix[0][2] * direction.z,
            matrix[1][0] * direction.x + matrix[1][1] * direction.y + matrix[1][2] * direction.z,
            matrix[2][0] * direction.x + matrix[2][1] * direction.y + matrix[2][2] * direction.z,
        };
    }

    inline SupportDirection principalAxisForSymmetric3(const std::array<std::array<float, 3>, 3>& matrix, SupportDirection seed)
    {
        SupportDirection axis = normalizeSupportDirection(seed);
        if (supportDirectionLengthSquared(axis) <= 0.0f) {
            axis = SupportDirection{ 1.0f, 0.0f, 0.0f };
        }

        for (int iteration = 0; iteration < 12; ++iteration) {
            const auto next = normalizeSupportDirection(multiplySymmetric3(matrix, axis));
            if (supportDirectionLengthSquared(next) <= 0.0f) {
                break;
            }
            axis = next;
        }

        return axis;
    }

    template <class Vector>
    inline std::array<SupportDirection, 3> makeSupportPrincipalBasis(const std::vector<Vector>& points)
    {
        std::array<SupportDirection, 3> basis{
            SupportDirection{ 1.0f, 0.0f, 0.0f },
            SupportDirection{ 0.0f, 1.0f, 0.0f },
            SupportDirection{ 0.0f, 0.0f, 1.0f },
        };
        if (points.size() < 3) {
            return basis;
        }

        SupportDirection centroid{};
        for (const auto& point : points) {
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
        }
        const float invCount = 1.0f / static_cast<float>(points.size());
        centroid.x *= invCount;
        centroid.y *= invCount;
        centroid.z *= invCount;

        std::array<std::array<float, 3>, 3> covariance{};
        for (const auto& point : points) {
            const float x = point.x - centroid.x;
            const float y = point.y - centroid.y;
            const float z = point.z - centroid.z;
            covariance[0][0] += x * x;
            covariance[0][1] += x * y;
            covariance[0][2] += x * z;
            covariance[1][1] += y * y;
            covariance[1][2] += y * z;
            covariance[2][2] += z * z;
        }
        covariance[1][0] = covariance[0][1];
        covariance[2][0] = covariance[0][2];
        covariance[2][1] = covariance[1][2];

        const auto bounds = pointBounds(points);
        SupportDirection seed{ 1.0f, 0.0f, 0.0f };
        if (bounds.valid) {
            const int axis = longestAxisForBounds(bounds);
            seed = SupportDirection{ axis == 0 ? 1.0f : 0.0f, axis == 1 ? 1.0f : 0.0f, axis == 2 ? 1.0f : 0.0f };
        }

        const auto first = principalAxisForSymmetric3(covariance, seed);
        const auto firstCov = multiplySymmetric3(covariance, first);
        const float firstLambda = supportDirectionDot(first, firstCov);
        std::array<std::array<float, 3>, 3> deflated = covariance;
        const std::array<float, 3> firstValues{ first.x, first.y, first.z };
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                deflated[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] -=
                    firstLambda * firstValues[static_cast<std::size_t>(row)] * firstValues[static_cast<std::size_t>(col)];
            }
        }

        std::array<SupportDirection, 3> fallbackAxes{
            SupportDirection{ 1.0f, 0.0f, 0.0f },
            SupportDirection{ 0.0f, 1.0f, 0.0f },
            SupportDirection{ 0.0f, 0.0f, 1.0f },
        };
        SupportDirection secondSeed = fallbackAxes[0];
        float leastAligned = std::abs(supportDirectionDot(first, secondSeed));
        for (const auto& candidate : fallbackAxes) {
            const float aligned = std::abs(supportDirectionDot(first, candidate));
            if (aligned < leastAligned) {
                leastAligned = aligned;
                secondSeed = candidate;
            }
        }

        auto second = principalAxisForSymmetric3(deflated, secondSeed);
        second = normalizeSupportDirection(supportDirectionSub(second, supportDirectionScale(first, supportDirectionDot(second, first))));
        if (supportDirectionLengthSquared(second) <= 0.0f) {
            second = normalizeSupportDirection(supportDirectionSub(secondSeed, supportDirectionScale(first, supportDirectionDot(secondSeed, first))));
        }

        auto third = normalizeSupportDirection(supportDirectionCross(first, second));
        if (supportDirectionLengthSquared(third) <= 0.0f) {
            return basis;
        }
        second = normalizeSupportDirection(supportDirectionCross(third, first));

        basis[0] = first;
        basis[1] = second;
        basis[2] = third;
        return basis;
    }

    inline void appendBaseSupportDirections(std::vector<SupportDirection>& directions, const std::array<SupportDirection, 3>& basis)
    {
        appendSignedSupportDirection(directions, SupportDirection{ 1.0f, 0.0f, 0.0f });
        appendSignedSupportDirection(directions, SupportDirection{ 0.0f, 1.0f, 0.0f });
        appendSignedSupportDirection(directions, SupportDirection{ 0.0f, 0.0f, 1.0f });
        for (const auto& axis : basis) {
            appendSignedSupportDirection(directions, axis);
        }

        for (int x = -1; x <= 1; x += 2) {
            for (int y = -1; y <= 1; y += 2) {
                for (int z = -1; z <= 1; z += 2) {
                    appendSupportDirection(directions, static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
                }
            }
        }

        for (int first = 0; first < 3; ++first) {
            for (int second = first + 1; second < 3; ++second) {
                for (int a = -1; a <= 1; a += 2) {
                    for (int b = -1; b <= 1; b += 2) {
                        std::array<float, 3> values{ 0.0f, 0.0f, 0.0f };
                        values[static_cast<std::size_t>(first)] = static_cast<float>(a);
                        values[static_cast<std::size_t>(second)] = static_cast<float>(b);
                        appendSupportDirection(directions, values[0], values[1], values[2]);
                    }
                }
            }
        }
    }

    inline std::vector<SupportDirection> makeValidationSupportDirections(const std::array<SupportDirection, 3>& basis)
    {
        std::vector<SupportDirection> directions;
        directions.reserve(160);
        appendBaseSupportDirections(directions, basis);

        for (int x = -2; x <= 2; ++x) {
            for (int y = -2; y <= 2; ++y) {
                for (int z = -2; z <= 2; ++z) {
                    if (x == 0 && y == 0 && z == 0) {
                        continue;
                    }
                    appendSupportDirection(directions, static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
                }
            }
        }

        return directions;
    }

    template <class Vector>
    inline std::size_t supportPointIndexForDirection(const std::vector<Vector>& points, const SupportDirection& direction)
    {
        std::size_t bestIndex = 0;
        float bestValue = -std::numeric_limits<float>::infinity();
        for (std::size_t i = 0; i < points.size(); ++i) {
            const float value = pointSupportDot(points[i], direction);
            if (value > bestValue) {
                bestValue = value;
                bestIndex = i;
            }
        }
        return bestIndex;
    }

    template <class Vector>
    inline bool selectSupportPointIndex(const std::vector<Vector>& points,
        const SupportDirection& direction,
        std::vector<std::uint8_t>& selected,
        std::size_t& selectedCount,
        std::size_t maxPoints)
    {
        if (points.empty() || selectedCount >= maxPoints) {
            return false;
        }

        const std::size_t index = supportPointIndexForDirection(points, direction);
        if (index >= selected.size() || selected[index]) {
            return false;
        }

        selected[index] = 1;
        ++selectedCount;
        return true;
    }

    template <class Vector>
    inline void selectSlicedSupportPoints(const std::vector<Vector>& points,
        std::vector<std::uint8_t>& selected,
        std::size_t& selectedCount,
        std::size_t maxPoints)
    {
        const auto bounds = pointBounds(points);
        if (!bounds.valid || selectedCount >= maxPoints) {
            return;
        }

        const int longAxis = longestAxisForBounds(bounds);
        const float minAxis = boundsAxisValue(bounds, longAxis, false);
        const float maxAxis = boundsAxisValue(bounds, longAxis, true);
        const float span = maxAxis - minAxis;
        if (span <= 1.0e-5f) {
            return;
        }

        const int crossA = (longAxis + 1) % 3;
        const int crossB = (longAxis + 2) % 3;
        std::vector<SupportDirection> crossDirections;
        crossDirections.reserve(8);
        for (int a = -1; a <= 1; a += 2) {
            for (int b = -1; b <= 1; b += 2) {
                std::array<float, 3> diagonal{ 0.0f, 0.0f, 0.0f };
                diagonal[static_cast<std::size_t>(crossA)] = static_cast<float>(a);
                diagonal[static_cast<std::size_t>(crossB)] = static_cast<float>(b);
                appendSupportDirection(crossDirections, diagonal[0], diagonal[1], diagonal[2]);
            }
        }
        std::array<float, 3> axisA{ 0.0f, 0.0f, 0.0f };
        std::array<float, 3> axisB{ 0.0f, 0.0f, 0.0f };
        axisA[static_cast<std::size_t>(crossA)] = 1.0f;
        axisB[static_cast<std::size_t>(crossB)] = 1.0f;
        appendSignedSupportDirection(crossDirections, SupportDirection{ axisA[0], axisA[1], axisA[2] });
        appendSignedSupportDirection(crossDirections, SupportDirection{ axisB[0], axisB[1], axisB[2] });

        constexpr std::size_t kSliceCount = 7;
        for (std::size_t slice = 0; slice < kSliceCount && selectedCount < maxPoints; ++slice) {
            const float start = minAxis + span * (static_cast<float>(slice) / static_cast<float>(kSliceCount));
            const float end = slice + 1 == kSliceCount ? maxAxis : minAxis + span * (static_cast<float>(slice + 1) / static_cast<float>(kSliceCount));
            for (const auto& direction : crossDirections) {
                if (selectedCount >= maxPoints) {
                    return;
                }

                std::size_t bestIndex = points.size();
                float bestValue = -std::numeric_limits<float>::infinity();
                for (std::size_t i = 0; i < points.size(); ++i) {
                    const float axisValue = pointAxisValue(points[i], longAxis);
                    if (axisValue < start || axisValue > end) {
                        continue;
                    }
                    const float value = pointSupportDot(points[i], direction);
                    if (value > bestValue) {
                        bestValue = value;
                        bestIndex = i;
                    }
                }

                if (bestIndex < selected.size() && !selected[bestIndex]) {
                    selected[bestIndex] = 1;
                    ++selectedCount;
                }
            }
        }
    }

    template <class Vector>
    inline float supportErrorForSelectedPoints(const std::vector<Vector>& points,
        const std::vector<std::uint8_t>& selected,
        const std::vector<SupportDirection>& validationDirections,
        SupportDirection* outWorstDirection)
    {
        float maxError = 0.0f;
        SupportDirection worstDirection{};
        for (const auto& direction : validationDirections) {
            float originalSupport = -std::numeric_limits<float>::infinity();
            float selectedSupport = -std::numeric_limits<float>::infinity();
            for (std::size_t i = 0; i < points.size(); ++i) {
                const float value = pointSupportDot(points[i], direction);
                originalSupport = (std::max)(originalSupport, value);
                if (i < selected.size() && selected[i]) {
                    selectedSupport = (std::max)(selectedSupport, value);
                }
            }

            const float error = (std::max)(0.0f, originalSupport - selectedSupport);
            if (error > maxError) {
                maxError = error;
                worstDirection = direction;
            }
        }

        if (outWorstDirection) {
            *outWorstDirection = worstDirection;
        }
        return maxError;
    }

    template <class Vector>
    inline std::vector<Vector> collectSelectedSupportPoints(const std::vector<Vector>& points, const std::vector<std::uint8_t>& selected)
    {
        std::vector<Vector> result;
        result.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (i < selected.size() && selected[i]) {
                result.push_back(points[i]);
            }
        }
        return result;
    }

    template <class Vector>
    inline ConvexSupportFitResult<Vector> fitConvexSupportPointCloud(const std::vector<Vector>& points,
        std::size_t targetPoints,
        std::size_t maxPoints,
        float maxSupportError)
    {
        /*
         * Weapon visual meshes often contain dense bevels, triangle duplicates,
         * and decorative surface loops that do not change the convex envelope.
         * This reducer keeps real mesh vertices only: it chooses directional
         * support points, measures support-function error against the original
         * cloud, then repairs with the worst missing support points. The result
         * is a bounded convex input that preserves collision silhouette without
         * splitting a single render source only because it had many triangles.
         */
        ConvexSupportFitResult<Vector> result{};
        result.inputPointCount = points.size();
        result.maxPointCount = maxPoints;
        result.targetPointCount = targetPoints;
        if (points.empty() || maxPoints < 4) {
            return result;
        }

        result.attempted = true;
        const std::size_t safeMaxPoints = (std::max)(std::size_t{ 4 }, maxPoints);
        const std::size_t safeTargetPoints = std::clamp(targetPoints, std::size_t{ 4 }, safeMaxPoints);
        const float safeMaxSupportError = (std::max)(0.0f, maxSupportError);
        result.maxPointCount = safeMaxPoints;
        result.targetPointCount = safeTargetPoints;

        if (points.size() <= safeTargetPoints) {
            result.points = points;
            result.selectedPointCount = points.size();
            result.accepted = true;
            result.maxSupportError = 0.0f;
            return result;
        }

        std::vector<std::uint8_t> selected(points.size(), 0);
        std::size_t selectedCount = 0;
        const auto basis = makeSupportPrincipalBasis(points);
        std::vector<SupportDirection> selectionDirections;
        selectionDirections.reserve(64);
        appendBaseSupportDirections(selectionDirections, basis);
        for (const auto& direction : selectionDirections) {
            selectSupportPointIndex(points, direction, selected, selectedCount, safeMaxPoints);
        }
        selectSlicedSupportPoints(points, selected, selectedCount, safeMaxPoints);

        for (std::size_t i = 0; selectedCount < 4 && i < points.size(); ++i) {
            if (!selected[i]) {
                selected[i] = 1;
                ++selectedCount;
            }
        }

        const auto validationDirections = makeValidationSupportDirections(basis);
        result.validationDirectionCount = validationDirections.size();
        SupportDirection worstDirection{};
        result.maxSupportError = supportErrorForSelectedPoints(points, selected, validationDirections, &worstDirection);

        while (result.maxSupportError > safeMaxSupportError && selectedCount < safeMaxPoints) {
            const bool added = selectSupportPointIndex(points, worstDirection, selected, selectedCount, safeMaxPoints);
            if (!added) {
                break;
            }
            ++result.repairPointCount;
            result.maxSupportError = supportErrorForSelectedPoints(points, selected, validationDirections, &worstDirection);
        }

        result.points = collectSelectedSupportPoints(points, selected);
        result.selectedPointCount = result.points.size();
        result.accepted = result.maxSupportError <= safeMaxSupportError;
        result.usedReduction = result.accepted && result.points.size() < points.size();
        result.reachedMaxPoints = result.points.size() >= safeMaxPoints && !result.accepted;
        return result;
    }

    struct HullSelectionInput
    {
        std::array<float, 3> center{};
        std::array<float, 3> min{};
        std::array<float, 3> max{};
        std::size_t pointCount = 0;
        int coverageClass = 0;
        int priority = 0;
        bool cosmetic = false;
    };

    inline float hullAxisValue(const std::array<float, 3>& value, int axis)
    {
        return value[static_cast<std::size_t>((std::max)(0, (std::min)(axis, 2)))];
    }

    inline float hullSelectionScore(const HullSelectionInput& input)
    {
        const float dx = input.max[0] - input.min[0];
        const float dy = input.max[1] - input.min[1];
        const float dz = input.max[2] - input.min[2];
        const float diagonalSquared = dx * dx + dy * dy + dz * dz;
        return diagonalSquared + static_cast<float>(input.pointCount) * 0.01f;
    }

    inline int longestAxisForHullSelection(const std::vector<HullSelectionInput>& inputs)
    {
        if (inputs.empty()) {
            return 0;
        }

        std::array<float, 3> minValue = inputs.front().min;
        std::array<float, 3> maxValue = inputs.front().max;
        for (const auto& input : inputs) {
            for (std::size_t axis = 0; axis < 3; ++axis) {
                minValue[axis] = (std::min)(minValue[axis], input.min[axis]);
                maxValue[axis] = (std::max)(maxValue[axis], input.max[axis]);
            }
        }

        const float dx = maxValue[0] - minValue[0];
        const float dy = maxValue[1] - minValue[1];
        const float dz = maxValue[2] - minValue[2];
        if (dy >= dx && dy >= dz) {
            return 1;
        }
        if (dz >= dx && dz >= dy) {
            return 2;
        }
        return 0;
    }

    inline std::vector<std::size_t> selectBalancedHullIndices(const std::vector<HullSelectionInput>& inputs, std::size_t budget)
    {
        std::vector<std::size_t> result;
        if (inputs.empty() || budget == 0) {
            return result;
        }
        if (inputs.size() <= budget) {
            result.reserve(inputs.size());
            for (std::size_t i = 0; i < inputs.size(); ++i) {
                result.push_back(i);
            }
            return result;
        }

        const int lengthAxis = longestAxisForHullSelection(inputs);
        std::vector<std::uint8_t> selected(inputs.size(), 0);
        result.reserve(budget);

        auto addIndex = [&](std::size_t index) {
            if (index >= inputs.size() || selected[index] || result.size() >= budget) {
                return false;
            }
            selected[index] = 1;
            result.push_back(index);
            return true;
        };

        auto findExtreme = [&](bool minimum, bool includeCosmetic) {
            std::size_t best = inputs.size();
            float bestAxis = 0.0f;
            float bestScore = 0.0f;
            for (std::size_t i = 0; i < inputs.size(); ++i) {
                if (selected[i] || (!includeCosmetic && inputs[i].cosmetic)) {
                    continue;
                }

                const float axisValue = hullAxisValue(inputs[i].center, lengthAxis);
                const float score = hullSelectionScore(inputs[i]);
                if (best == inputs.size() || (minimum ? axisValue < bestAxis : axisValue > bestAxis) ||
                    (std::abs(axisValue - bestAxis) < 0.001f && score > bestScore)) {
                    best = i;
                    bestAxis = axisValue;
                    bestScore = score;
                }
            }
            return best;
        };

        std::size_t rear = findExtreme(true, false);
        if (rear == inputs.size()) {
            rear = findExtreme(true, true);
        }
        addIndex(rear);

        std::size_t front = findExtreme(false, false);
        if (front == inputs.size()) {
            front = findExtreme(false, true);
        }
        addIndex(front);

        std::vector<int> coverageClasses;
        for (const auto& input : inputs) {
            if (input.cosmetic) {
                continue;
            }
            if (std::find(coverageClasses.begin(), coverageClasses.end(), input.coverageClass) == coverageClasses.end()) {
                coverageClasses.push_back(input.coverageClass);
            }
        }

        auto classBestPriority = [&](int coverageClass) {
            int best = std::numeric_limits<int>::max();
            for (const auto& input : inputs) {
                if (!input.cosmetic && input.coverageClass == coverageClass) {
                    best = (std::min)(best, input.priority);
                }
            }
            return best;
        };

        std::stable_sort(coverageClasses.begin(), coverageClasses.end(), [&](int lhs, int rhs) {
            const int lhsPriority = classBestPriority(lhs);
            const int rhsPriority = classBestPriority(rhs);
            if (lhsPriority != rhsPriority) {
                return lhsPriority < rhsPriority;
            }
            return lhs < rhs;
        });

        auto findBestForClass = [&](int coverageClass, bool includeCosmetic) {
            std::size_t best = inputs.size();
            int bestPriority = std::numeric_limits<int>::max();
            float bestScore = 0.0f;
            for (std::size_t i = 0; i < inputs.size(); ++i) {
                if (selected[i] || inputs[i].coverageClass != coverageClass || (!includeCosmetic && inputs[i].cosmetic)) {
                    continue;
                }

                const int priority = inputs[i].priority;
                const float score = hullSelectionScore(inputs[i]);
                if (best == inputs.size() || priority < bestPriority || (priority == bestPriority && score > bestScore)) {
                    best = i;
                    bestPriority = priority;
                    bestScore = score;
                }
            }
            return best;
        };

        for (const int coverageClass : coverageClasses) {
            if (result.size() >= budget) {
                break;
            }
            addIndex(findBestForClass(coverageClass, false));
        }

        bool madeProgress = true;
        while (result.size() < budget && madeProgress) {
            madeProgress = false;
            for (const int coverageClass : coverageClasses) {
                if (result.size() >= budget) {
                    break;
                }
                madeProgress = addIndex(findBestForClass(coverageClass, false)) || madeProgress;
            }
        }

        while (result.size() < budget) {
            std::size_t best = inputs.size();
            int bestPriority = std::numeric_limits<int>::max();
            float bestScore = 0.0f;
            for (std::size_t i = 0; i < inputs.size(); ++i) {
                if (selected[i]) {
                    continue;
                }

                const int priority = inputs[i].priority;
                const float score = hullSelectionScore(inputs[i]);
                if (best == inputs.size() || priority < bestPriority || (priority == bestPriority && score > bestScore)) {
                    best = i;
                    bestPriority = priority;
                    bestScore = score;
                }
            }
            if (!addIndex(best)) {
                break;
            }
        }

        std::stable_sort(result.begin(), result.end(), [&](std::size_t lhs, std::size_t rhs) {
            return hullAxisValue(inputs[lhs].center, lengthAxis) < hullAxisValue(inputs[rhs].center, lengthAxis);
        });
        return result;
    }
}

// ---- WeaponInteractionProbeMath.h ----

#include <algorithm>
#include <cmath>

namespace rock::weapon_interaction_probe_math
{
    /*
     * ROCK drives two-handed weapon selection from an active near-hand search
     * instead of relying only on solver contact events. Generated hand and weapon
     * bodies are both keyframed, so FO4VR may not report a contact even when the
     * left hand is visually touching the weapon. This helper keeps the proximity
     * test explicit and testable.
     */
    template <class Vector>
    inline float pointAabbDistanceSquared(const Vector& point, const Vector& boundsMin, const Vector& boundsMax)
    {
        const float clampedX = (std::max)(boundsMin.x, (std::min)(point.x, boundsMax.x));
        const float clampedY = (std::max)(boundsMin.y, (std::min)(point.y, boundsMax.y));
        const float clampedZ = (std::max)(boundsMin.z, (std::min)(point.z, boundsMax.z));

        const float dx = point.x - clampedX;
        const float dy = point.y - clampedY;
        const float dz = point.z - clampedZ;
        return dx * dx + dy * dy + dz * dz;
    }

    inline bool isWithinProbeRadiusSquared(float distanceSquared, float radius)
    {
        const float safeRadius = (std::max)(0.0f, radius);
        return distanceSquared <= safeRadius * safeRadius;
    }
}
