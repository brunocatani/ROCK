#pragma once

#include <array>
#include <cmath>

namespace rock::skinned_surface_math
{
    using Transform = std::array<float, 16>;
    using Affine = std::array<float, 12>;

    // Native render palette: 141DB6CA0 transposes the padded Ni bases and
    // composes them after applying the optional instance+50 bone scale.
    // 141DB2BE0 and 141DB65D0 consume that palette for static AND dynamic skin.
    inline bool worldFromSkin(const Transform& boneWorld, const Transform& skinToBone,
        const std::array<float, 3>& extraScale, Affine& result) noexcept
    {
        for (float value : boneWorld) if (!std::isfinite(value)) return false;
        for (float value : skinToBone) if (!std::isfinite(value)) return false;
        for (float value : extraScale) if (!std::isfinite(value)) return false;
        if (boneWorld[15] <= 0.0f || skinToBone[15] <= 0.0f) return false;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 4; ++column) {
                double value = column == 3 ? double(boneWorld[12 + row]) * skinToBone[15] : 0.0;
                for (int axis = 0; axis < 3; ++axis) {
                    const double scale = boneWorld[15] * (extraScale[0] > 0.5f ? extraScale[axis] : 1.0f);
                    value += double(boneWorld[axis * 4 + row]) * scale * skinToBone[column * 4 + axis];
                }
                result[row * 4 + column] = static_cast<float>(value);
                if (!std::isfinite(result[row * 4 + column])) return false;
            }
        }
        return true;
    }

    template <class Point>
    inline Point transform(const Affine& matrix, const Point& point) noexcept
    {
        return Point{
            matrix[0] * point.x + matrix[1] * point.y + matrix[2] * point.z + matrix[3],
            matrix[4] * point.x + matrix[5] * point.y + matrix[6] * point.z + matrix[7],
            matrix[8] * point.x + matrix[9] * point.y + matrix[10] * point.z + matrix[11]};
    }

    template <class Point>
    inline bool blendVertex(const std::array<const Affine*,4>& matrices, const std::array<float,4>& weights,
        const Point& bind, const Point& origin, Point& result) noexcept
    {
        // 141DB2BE0 subtracts geometry world translation before blending.
        // Half-precision weights can sum slightly above one when weight 4 is
        // clamped to zero. Blending absolute world positions would amplify that
        // quantization error by the distance from the world origin.
        double x=0, y=0, z=0;
        bool any=false;
        for (std::size_t i=0;i<weights.size();++i) {
            const auto weight=weights[i];
            if (!std::isfinite(weight)) return false;
            if (weight<=0) continue;
            if (!matrices[i]) return false;
            const auto point=transform(*matrices[i],bind);
            x+=weight*(double(point.x)-origin.x);
            y+=weight*(double(point.y)-origin.y);
            z+=weight*(double(point.z)-origin.z);
            any=true;
        }
        result=Point{static_cast<float>(x+origin.x),static_cast<float>(y+origin.y),static_cast<float>(z+origin.z)};
        return any && std::isfinite(result.x) && std::isfinite(result.y) && std::isfinite(result.z);
    }
}
