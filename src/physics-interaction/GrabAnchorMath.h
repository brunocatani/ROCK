#pragma once

namespace frik::rock::grab_anchor_math
{
    /*
     * ROCK exposes pivot-A tuning because FO4VR/FRIK handspace is not identical to
     * HIGGS' authored Skyrim handspace. The visible pre-grab anchor, selection ray
     * origin, and constraint pivot A must all be derived from the same handspace
     * point. Live left-hand tuning showed the hand node already carries handedness
     * for this pivot, so the authored INI offset must not be mirrored per hand.
     */
    template <class Vector>
    inline Vector applyReversal(Vector value, bool reverse)
    {
        if (reverse) {
            value.x *= -1.0f;
            value.y *= -1.0f;
            value.z *= -1.0f;
        }

        return value;
    }

    template <class Vector>
    inline Vector makeGrabAnchorHandspace(Vector palmBase, Vector pivotAOffset, bool reverseOffset)
    {
        const Vector effectiveOffset = applyReversal(pivotAOffset, reverseOffset);
        palmBase.x += effectiveOffset.x;
        palmBase.y += effectiveOffset.y;
        palmBase.z += effectiveOffset.z;
        return palmBase;
    }

    template <class Vector>
    inline Vector makeGrabAnchorHandspaceForHand(Vector palmBase, Vector pivotAOffset, bool reverseOffset, bool isLeft)
    {
        (void)isLeft;
        return makeGrabAnchorHandspace(palmBase, pivotAOffset, reverseOffset);
    }
}
