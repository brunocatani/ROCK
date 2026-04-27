#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace frik::rock::weapon_collision_geometry_math
{
    /*
     * Weapon mesh collision is generated in weapon-root local space so every
     * hull follows the firearm as one package. HIGGS solves weapon collision
     * by owning one keyframed weapon body instead of reinterpreting each source
     * node as an independent frame; for FO4VR firearms we keep that ownership
     * model but build several convex hulls from visible mesh parts because the
     * native collision bodies are often incomplete.
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
