#pragma once

namespace frik::rock::handspace_convention
{
    /*
     * FO4/FRIK hand nodes use the active ROCK authored handspace convention:
     * authored X = fingers, authored Y = cross-palm, authored Z = palm thickness.
     * Runtime FRIK transforms already carry handedness, so left/right conversion
     * uses the same X/-Z/Y basis and does not apply an additional manual mirror.
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
    inline Vector authoredToRaw(Vector value)
    {
        return makeVector<Vector>(value.x, value.z, -value.y);
    }

    template <class Vector>
    inline Vector authoredToRawForHand(Vector value, bool isLeft)
    {
        (void)isLeft;
        return authoredToRaw(value);
    }
}
