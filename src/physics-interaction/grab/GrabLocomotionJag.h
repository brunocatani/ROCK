#pragma once

/*
 * Locomotion jag correction for the held dynamic body.
 *
 * MEASUREMENT (2026-07-25, camera-relative decomposition of the 07-24 paired
 * session, 4,954 frames both hands, 0-587 gu/s -- see
 * Docs\ROCK\docs\2026-07-25-stutter-mechanism-motor-lowpass-of-vfr-staircase.md):
 * the felt stick-locomotion stutter is NOT a command defect and NOT a mean
 * tracking failure. Decomposing (obj - cam) = (obj - tgt) + (tgt - cam):
 *
 *   at 400+ gu/s   perceived (obj-cam) 0.225/0.231 gu per frame
 *                  command   (tgt-cam) 0.031/0.034  <- LOWER than standing
 *                  motor     (obj-tgt) 0.215/0.232  <- 96-101% of perceived
 *
 * The absolute following error is flat with speed (0.52-0.73 gu everywhere,
 * implied lag 0.016 ms), so the motors keep up on average at every speed;
 * only the per-frame JITTER scales. The world path is a v*|d(dt)| staircase
 * (VFR frame pacing x locomotion speed, 0.42-0.47 gu/frame at sprint) which
 * the camera reproduces exactly -- it IS the room -- and which the motors
 * reproduce at a measured, speed-independent ratio of 0.50. The half they
 * drop is what the eye sees, and it rings (lag-1 autocorr -0.53 at sprint vs
 * +0.06 standing).
 *
 * THE CORRECTION delivers that dropped half as POSITION, once per flush:
 *
 *     correction = dP_room - v_room * dt_substep
 *
 * dP_room is a DIFFERENCED anchor position, never an integrated velocity:
 * integration is precisely what cannot express a per-frame discontinuity,
 * which is why the aa4d514 velocity-add transport measured as a no-op for
 * this symptom. v_room * dt is the displacement the body's own inertia
 * already provides for free (Havok bodies coast; steady locomotion never
 * demanded sustained linear motor force).
 *
 * It self-zeroes where it must:
 *   standing                     dP = 0, v = 0        -> 0
 *   locomotion, even pacing      dP = v*dt            -> ~0
 *   locomotion, dt mismatch      v*(dt_render-dt_sub) -> 0.4-0.65 gu  (the bug)
 *
 * so standing behaviour cannot regress: the term is identically zero there.
 *
 * NOT a double drive: the motors are a closed loop on (target - body).
 * Placing the body closer to correct shrinks the error, so their own
 * contribution shrinks in the same frame -- the two settle rather than stack.
 * The solver re-bakes its motor elements from LIVE body state during solving
 * (2026-07-24 Ghidra pass), so a pre-solve write is seen rather than fighting
 * a stale frame-start snapshot -- which is exactly how the removed Era-1
 * compensation died (phase-dead write, overwritten by the proxy solve).
 *
 * NOT the removed compensation class and NOT the vetoed position accumulator:
 * nothing accumulates across frames, no commanded target is constructed from
 * deltas, the command is not touched at all, and the motors remain the sole
 * position authority. This is a bounded, self-zeroing nudge to the BODY.
 *
 * NEVER filter, smooth, or low-pass anything in this path. The world path is
 * legitimately jagged and the eye rides the jags; smoothing the object makes
 * it disagree with the camera MORE. That inversion is what defeated the
 * resampler (4e383d8) and the velocity smoother (69a1246).
 */

#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "RE/NetImmerse/NiPoint.h"

#include <cmath>
#include <cstdint>

namespace rock::grab_locomotion_jag
{
    /*
     * One-flush anchor discontinuity gate. A cell change, load door, coc, or
     * script teleport must never be read as one frame of locomotion. Shares the
     * source clock's translation-jump rationale and threshold: sprint moves the
     * anchor ~4.8 gu per frame, so 35 gu is far above any real locomotion step
     * while still catching every teleport class.
     */
    constexpr float kMaxAnchorJumpGameUnits = 35.0f;

    /*
     * Below this the correction is not worth a deferred body write. Two orders
     * of magnitude under the measured standing motor error (0.039 gu), so it
     * can never mask a real correction.
     */
    constexpr float kMinCorrectionGameUnits = 0.0005f;

    // Speed gates mirror the room-velocity feed-forward: below the floor the
    // player is standing (controller noise, and the term is zero anyway);
    // above the cap the read is not locomotion (launch, script teleport,
    // corrupted pointer walk) and must not become a body displacement.
    constexpr float kMinSpeedGameUnitsPerSecond = 1.0f;
    constexpr float kMaxSpeedGameUnitsPerSecond = 2000.0f;

    enum class JagSkipReason : std::uint8_t
    {
        None,
        Disabled,
        NoPreviousAnchor,
        NonFiniteAnchor,
        NoRoomVelocity,
        BelowSpeedGate,
        AboveSpeedCap,
        UnusableDelta,
        AnchorDiscontinuity,
        NonFiniteCorrection,
        BelowEpsilon,
    };

    [[nodiscard]] inline const char* describe(JagSkipReason reason) noexcept
    {
        switch (reason) {
        case JagSkipReason::None:
            return "none";
        case JagSkipReason::Disabled:
            return "disabled";
        case JagSkipReason::NoPreviousAnchor:
            return "noPreviousAnchor";
        case JagSkipReason::NonFiniteAnchor:
            return "nonFiniteAnchor";
        case JagSkipReason::NoRoomVelocity:
            return "noRoomVelocity";
        case JagSkipReason::BelowSpeedGate:
            return "belowSpeedGate";
        case JagSkipReason::AboveSpeedCap:
            return "aboveSpeedCap";
        case JagSkipReason::UnusableDelta:
            return "unusableDelta";
        case JagSkipReason::AnchorDiscontinuity:
            return "anchorDiscontinuity";
        case JagSkipReason::NonFiniteCorrection:
            return "nonFiniteCorrection";
        case JagSkipReason::BelowEpsilon:
            return "belowEpsilon";
        }
        return "unknown";
    }

    struct JagInput
    {
        // Room anchor (character-controller world position) in GAME UNITS,
        // this flush and the previous one. Differenced, never integrated.
        RE::NiPoint3 anchorGameUnits{};
        RE::NiPoint3 previousAnchorGameUnits{};
        bool previousAnchorValid = false;

        // Live character-controller velocity, GAME UNITS per second.
        RE::NiPoint3 roomVelocityGameUnitsPerSecond{};
        bool roomVelocityValid = false;

        // The substep delta the body will actually be integrated with.
        float substepDeltaSeconds = 0.0f;

        // 0 disables; 1 delivers the whole dropped jag. A/B only -- this is a
        // scale on a bounded correction, never a smoothing filter.
        float gain = 1.0f;

        // Hard clamp. Exceeding it is a bug signal, not a normal path.
        float maxCorrectionGameUnits = 2.0f;
    };

    struct JagCorrection
    {
        RE::NiPoint3 deltaGameUnits{};
        bool apply = false;
        bool clamped = false;
        JagSkipReason skipReason = JagSkipReason::None;
    };

    [[nodiscard]] inline bool isFiniteVector(const RE::NiPoint3& value) noexcept
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    [[nodiscard]] inline float lengthSquared(const RE::NiPoint3& value) noexcept
    {
        return value.x * value.x + value.y * value.y + value.z * value.z;
    }

    /*
     * Evaluate one flush's correction. Pure: no engine state, no history --
     * the caller owns the previous anchor so there is nothing to accumulate
     * and nothing to drift.
     */
    [[nodiscard]] inline JagCorrection evaluate(const JagInput& input) noexcept
    {
        JagCorrection result{};

        if (!(input.gain > 0.0f) || !std::isfinite(input.gain)) {
            result.skipReason = JagSkipReason::Disabled;
            return result;
        }
        if (!input.previousAnchorValid) {
            result.skipReason = JagSkipReason::NoPreviousAnchor;
            return result;
        }
        if (!isFiniteVector(input.anchorGameUnits) || !isFiniteVector(input.previousAnchorGameUnits)) {
            result.skipReason = JagSkipReason::NonFiniteAnchor;
            return result;
        }
        if (!input.roomVelocityValid || !isFiniteVector(input.roomVelocityGameUnitsPerSecond)) {
            result.skipReason = JagSkipReason::NoRoomVelocity;
            return result;
        }

        const float speedSquared = lengthSquared(input.roomVelocityGameUnitsPerSecond);
        if (!(speedSquared >= kMinSpeedGameUnitsPerSecond * kMinSpeedGameUnitsPerSecond)) {
            result.skipReason = JagSkipReason::BelowSpeedGate;
            return result;
        }
        if (speedSquared > kMaxSpeedGameUnitsPerSecond * kMaxSpeedGameUnitsPerSecond) {
            result.skipReason = JagSkipReason::AboveSpeedCap;
            return result;
        }
        if (!havok_physics_timing::isUsableDelta(input.substepDeltaSeconds)) {
            result.skipReason = JagSkipReason::UnusableDelta;
            return result;
        }

        const RE::NiPoint3 anchorDelta{
            input.anchorGameUnits.x - input.previousAnchorGameUnits.x,
            input.anchorGameUnits.y - input.previousAnchorGameUnits.y,
            input.anchorGameUnits.z - input.previousAnchorGameUnits.z,
        };
        if (lengthSquared(anchorDelta) > kMaxAnchorJumpGameUnits * kMaxAnchorJumpGameUnits) {
            result.skipReason = JagSkipReason::AnchorDiscontinuity;
            return result;
        }

        // The displacement inertia already delivers this substep. Subtracting
        // it leaves exactly the jag the motors drop.
        const RE::NiPoint3 correction{
            (anchorDelta.x - input.roomVelocityGameUnitsPerSecond.x * input.substepDeltaSeconds) * input.gain,
            (anchorDelta.y - input.roomVelocityGameUnitsPerSecond.y * input.substepDeltaSeconds) * input.gain,
            (anchorDelta.z - input.roomVelocityGameUnitsPerSecond.z * input.substepDeltaSeconds) * input.gain,
        };
        if (!isFiniteVector(correction)) {
            result.skipReason = JagSkipReason::NonFiniteCorrection;
            return result;
        }

        const float magnitudeSquared = lengthSquared(correction);
        if (magnitudeSquared < kMinCorrectionGameUnits * kMinCorrectionGameUnits) {
            result.skipReason = JagSkipReason::BelowEpsilon;
            return result;
        }

        const float maxCorrection =
            (std::isfinite(input.maxCorrectionGameUnits) && input.maxCorrectionGameUnits > 0.0f)
            ? input.maxCorrectionGameUnits
            : 0.0f;
        if (maxCorrection <= 0.0f) {
            result.skipReason = JagSkipReason::Disabled;
            return result;
        }

        result.deltaGameUnits = correction;
        if (magnitudeSquared > maxCorrection * maxCorrection) {
            const float magnitude = std::sqrt(magnitudeSquared);
            const float scale = maxCorrection / magnitude;
            result.deltaGameUnits.x = correction.x * scale;
            result.deltaGameUnits.y = correction.y * scale;
            result.deltaGameUnits.z = correction.z * scale;
            result.clamped = true;
        }
        result.apply = true;
        return result;
    }
}
