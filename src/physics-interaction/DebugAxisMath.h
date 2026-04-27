#pragma once

namespace frik::rock::debug_axis_math
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
