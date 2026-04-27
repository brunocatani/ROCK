#pragma once

namespace frik::rock::handspace_convention
{
    /*
     * The live hknp visualizer proved that the FRIK hand transforms already carry
     * per-hand orientation, while ROCK's old HIGGS-style left-hand mirror was being
     * applied on the wrong semantic axis. Keep hand mirroring tied to the selected
     * convention mode so legacy/HIGGS diagnostics remain available, and the active
     * FO4/FRIK convention can use the same effective finger offset for both hands.
     */
    template <class Vector>
    inline Vector makeVector(float x, float y, float z)
    {
        Vector result{};
        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    }

    template <class Vector>
    inline Vector mirrorAuthoredForHand(Vector value, bool isLeft, int mode)
    {
        if (!isLeft) {
            return value;
        }

        switch (mode) {
        case 0:
            value.x *= -1.0f;
            break;
        case 2:
            value.z *= -1.0f;
            break;
        default:
            break;
        }

        return value;
    }

    template <class Vector>
    inline Vector authoredToRaw(Vector value, int mode)
    {
        switch (mode) {
        case 1:
            return makeVector<Vector>(value.x, value.z, -value.y);
        case 2:
            return makeVector<Vector>(value.z, value.x, value.y);
        default:
            return value;
        }
    }

    template <class Vector>
    inline Vector authoredToRawForHand(Vector value, bool isLeft, int mode)
    {
        return authoredToRaw(mirrorAuthoredForHand(value, isLeft, mode), mode);
    }
}
