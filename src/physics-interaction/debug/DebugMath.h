#pragma once

namespace rock::debug_axis_math
{
    // FO4VR's NiAVObject world update uses Bethesda's transform compose helper
    // at 0x1401A8D60. Local basis vectors become world vectors through the
    // stored matrix columns, which is equivalent to CommonLib's Transpose()*v
    // behavior. Keep the debug overlay on that convention so the axes expose
    // the same frame used by hand collision, palm checks, and grab inputs.
    template <class Matrix, class Vector>
    inline Vector rotateNiLocalToWorld(const Matrix& matrix, const Vector& vector)
    {
        return Vector{
            matrix.entry[0][0] * vector.x + matrix.entry[1][0] * vector.y + matrix.entry[2][0] * vector.z,
            matrix.entry[0][1] * vector.x + matrix.entry[1][1] * vector.y + matrix.entry[2][1] * vector.z,
            matrix.entry[0][2] * vector.x + matrix.entry[1][2] * vector.y + matrix.entry[2][2] * vector.z,
        };
    }
}

namespace rock::debug_pivot_math
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
