#pragma once

namespace frik::rock::debug_pivot_math
{
    // Grab pivots live in hknp constraint atom local space, while the visualizer
    // draws game-space world coordinates. Keep this conversion isolated because
    // a row/column swap here would make the overlay disagree with the real
    // constraint and send pivot tuning in the wrong direction.
    template <class Vector>
    inline Vector bodyLocalPointToWorldGamePoint(const float* bodyFloats, const float* localPoint, float havokToGameScale)
    {
        return Vector{
            (bodyFloats[12] + bodyFloats[0] * localPoint[0] + bodyFloats[4] * localPoint[1] + bodyFloats[8] * localPoint[2]) * havokToGameScale,
            (bodyFloats[13] + bodyFloats[1] * localPoint[0] + bodyFloats[5] * localPoint[1] + bodyFloats[9] * localPoint[2]) * havokToGameScale,
            (bodyFloats[14] + bodyFloats[2] * localPoint[0] + bodyFloats[6] * localPoint[1] + bodyFloats[10] * localPoint[2]) * havokToGameScale,
        };
    }

    template <class Vector>
    inline Vector bodyOriginToWorldGamePoint(const float* bodyFloats, float havokToGameScale)
    {
        return Vector{
            bodyFloats[12] * havokToGameScale,
            bodyFloats[13] * havokToGameScale,
            bodyFloats[14] * havokToGameScale,
        };
    }
}
