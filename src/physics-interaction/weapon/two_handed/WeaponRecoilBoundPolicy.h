#pragma once

#include <cmath>

namespace rock::weapon_recoil_bound_policy
{
    /*
     * Bounds the weapon kick by the recoil the solver was actually asked for.
     *
     * The applied weapon delta is derived from the firing hand: the pose
     * captured before the deferred solve, against the wrist that solve
     * presented. That difference is the kick only if the hand actually
     * tracked its target. If the solve fell back to the tracked hand instead,
     * the same subtraction yields the whole fallback jump, tens of game
     * units, and applying it kicks the weapon somewhere no shot could send
     * it.
     *
     * The ticket already carries the kick the controller published, so the
     * question has a direct answer: is the delta about the size of the kick
     * that produced it? A real kick passes, a fallback jump does not.
     *
     * The factor is generous rather than tight. The published kick is in the
     * hand's local frame and reaches the weapon through the arm chain, so a
     * faithful delivery is the same order of magnitude but not the same
     * number. The allowance covers small solver residual on top of a kick
     * that is itself near zero. Together they still reject a fallback jump by
     * an order of magnitude, which is the failure this bound exists for.
     */
    inline constexpr float kKickMagnitudeFactor = 2.0f;
    inline constexpr float kTranslationAllowanceGameUnits = 0.5f;
    inline constexpr float kRotationAllowanceDegrees = 1.5f;

    struct Bounds
    {
        float maxTranslationGameUnits = 0.0f;
        float maxRotationDegrees = 0.0f;
        bool valid = false;
    };

    [[nodiscard]] inline Bounds makeBounds(
        const float kickTranslationGameUnits,
        const float kickRotationDegrees,
        const float factor = kKickMagnitudeFactor,
        const float translationAllowanceGameUnits =
            kTranslationAllowanceGameUnits,
        const float rotationAllowanceDegrees = kRotationAllowanceDegrees)
    {
        if (!std::isfinite(kickTranslationGameUnits) ||
            !std::isfinite(kickRotationDegrees) ||
            kickTranslationGameUnits < 0.0f ||
            kickRotationDegrees < 0.0f ||
            !std::isfinite(factor) ||
            factor <= 0.0f) {
            return {};
        }
        return Bounds{
            .maxTranslationGameUnits =
                kickTranslationGameUnits * factor +
                translationAllowanceGameUnits,
            .maxRotationDegrees =
                kickRotationDegrees * factor + rotationAllowanceDegrees,
            .valid = true,
        };
    }

    [[nodiscard]] inline bool isAppliedDeltaWithinBounds(
        const float appliedTranslationGameUnits,
        const float appliedRotationDegrees,
        const Bounds& bounds)
    {
        if (!bounds.valid ||
            !std::isfinite(appliedTranslationGameUnits) ||
            !std::isfinite(appliedRotationDegrees)) {
            return false;
        }
        return appliedTranslationGameUnits <= bounds.maxTranslationGameUnits &&
            appliedRotationDegrees <= bounds.maxRotationDegrees;
    }
}
