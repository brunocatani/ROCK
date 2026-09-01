#pragma once

#include <cmath>
#include <cstddef>
#include <span>

namespace rock::hand_finger_mirror_math
{
    /*
     * Exact mirror of one finger-bone local transform between the two
     * hands. The active skeleton.nif places every left arm, hand, and
     * finger bone as the Z-reflection of its right counterpart in the
     * parent frame (exact to five decimals), so a pose authored on one
     * hand transfers to the other by conjugating each local with
     * diag(1,1,-1): the rotation entries with exactly one Z index change
     * sign and the local Z translation negates. The reflection is diagonal,
     * so the map is convention-free and an involution; it applies to either
     * hand. It replaces hFRIK's flex/splay projection, which re-synthesised
     * the pose along a generic open-to-closed curl and lost every off-arc
     * rotation: on a handguard wrap that was 40-60 degrees per proximal
     * joint and visibly spread the fingers.
     */
    template <class Transform>
    [[nodiscard]] inline Transform mirrorFingerLocalAcrossHands(const Transform& local)
    {
        Transform mirrored = local;
        mirrored.rotate.entry[0][2] = -local.rotate.entry[0][2];
        mirrored.rotate.entry[1][2] = -local.rotate.entry[1][2];
        mirrored.rotate.entry[2][0] = -local.rotate.entry[2][0];
        mirrored.rotate.entry[2][1] = -local.rotate.entry[2][1];
        mirrored.translate.z = -local.translate.z;
        return mirrored;
    }

    template <class Transform>
    [[nodiscard]] inline bool finiteFingerLocal(const Transform& local)
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(local.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        return std::isfinite(local.translate.x) &&
               std::isfinite(local.translate.y) &&
               std::isfinite(local.translate.z) &&
               std::isfinite(local.scale) &&
               std::abs(local.scale) > 0.0001f;
    }

    /*
     * Mirrors a complete finger pose bone by bone. Bone order is per finger
     * and joint, identical on both hands, so the enabled mask carries over
     * unchanged. Fails closed on a size mismatch or a non-finite source
     * local and clears the target.
     */
    template <class Transform>
    [[nodiscard]] inline bool mirrorFingerLocalsAcrossHands(
        std::span<const Transform> source,
        std::span<Transform> target)
    {
        if (source.size() != target.size()) {
            return false;
        }
        for (std::size_t index = 0; index < source.size(); ++index) {
            if (!finiteFingerLocal(source[index])) {
                for (auto& local : target) {
                    local = Transform{};
                }
                return false;
            }
            target[index] = mirrorFingerLocalAcrossHands(source[index]);
        }
        return true;
    }
}
