#pragma once

namespace frik::rock::pointing_direction_math
{
    /*
     * HIGGS keeps close palm selection and far pointing selection as separate
     * handspace directions. ROCK mirrors that split so debugging the blue far
     * selection ray never changes the yellow palm-normal line or close-grab
     * direction. This helper keeps the optional far-ray reversal shared between
     * runtime selection and the visualizer.
     */
    template <class Vector>
    inline Vector applyFarGrabNormalReversal(Vector direction, bool reverse)
    {
        if (reverse) {
            direction.x *= -1.0f;
            direction.y *= -1.0f;
            direction.z *= -1.0f;
        }

        return direction;
    }
}
