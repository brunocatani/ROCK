#pragma once

/*
 * Game-clock phase lock for the grab-authority proxy target.
 *
 * The held-object target is sampled on the game/source clock (precise wall
 * intervals, ~11ms) but consumed on the physics-substep clock (quantized
 * 10/11/12ms). The previous design here -- a source-to-physics-clock
 * trajectory resampler -- played the sampled trajectory back on the PHYSICS
 * clock: the commanded velocity was smooth, but the commanded POSITION
 * deviated from the wand's game-time path by v x (clock mismatch) on every
 * rendered frame. The 2026-07-13 OVERLAY_POINT probe measured that deviation
 * as the dominant visible-stutter link (tgt-wand jitter 2x every other link;
 * frame-gap-binned drift +0.28 gu on short frames / -0.19 gu on long frames
 * at a 400 gu/s walk), while the hand collider -- keyframed straight onto the
 * game-clock wand sample -- was the visibly smooth reference in the same
 * view. The eye compares the held object against the wand, hands, and camera,
 * all of which advance on the game clock; game-clock path fidelity wins over
 * physics-clock velocity smoothness.
 *
 * Contract: the raw phase-lock output on the LAST physics substep of every
 * frame lands EXACTLY on the newest queued game-frame sample (segment fraction
 * (index+1)/count reaches 1). RootMotionFeedForward then adds the player-root
 * displacement that the game will apply AFTER this physics update: the
 * 2026-08-16 phase-bracket captures proved FO4VR applies joystick locomotion
 * post-physics in game code (pre-collide, ROCK's +0x30, and post-solve all
 * see ZERO movement in every root domain -- roomNode world/local,
 * playerWorldNode, actor, and the bhkCharProxyController root, which never
 * moves at all during stick locomotion -- while post-solve-to-producer
 * carries the full step). The current frame's displacement therefore exists
 * NOWHERE during physics, so no measured rebase at any flush boundary can
 * remove the one-frame basis lag (appliedRaw(t) == raw(t-1) bit-exact over
 * 359 locomotion frames; downstream target->proxy error 0.003 gu). The only
 * consumption-side correction is the engine's own last applied root step,
 * scaled by the exact frame-dt ratio: the dt-jitter component of the stutter
 * (dominant; corr 0.80-0.85 with speed x |d(dt)|) cancels exactly, leaving
 * only true acceleration (~0.07-0.11 gu at steady walk/sprint). This is
 * deliberately restricted to the PLAYER ROOT step measured by the game's own
 * tracker -- the general no-feed-forward rule for hand/wand motion remains:
 * hand velocity is noisy, root locomotion is not.
 * The commanded velocity absorbs the
 * substep-dt quantization (~+-10%); the constraint motors low-pass velocity
 * noise (measured 2026-07-13 against the far larger v1 feed-forward spikes),
 * and no session ever correlated commanded-velocity smoothness with what the
 * player sees. Discontinuity gates (teleport, snap turn, source hitch, proxy
 * rebuild) snap to the new sample instead of interpolating across the jump.
 * Rotation is deliberately not interpolated; it stays on the sampled path and
 * enters only the discontinuity check.
 */

#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "RE/NetImmerse/NiMatrix3.h"
#include "RE/NetImmerse/NiPoint.h"

#include <cmath>
#include <cstdint>
#include <limits>

namespace rock::grab_authority_source_clock
{
    enum class ResampleAction : std::uint8_t
    {
        Hold,
        Interpolate,
        Lock,
        Rebase,
    };

    constexpr const char* resampleActionName(ResampleAction action) noexcept
    {
        switch (action) {
        case ResampleAction::Hold:
            return "hold";
        case ResampleAction::Interpolate:
            return "interpolate";
        case ResampleAction::Lock:
            return "lock";
        case ResampleAction::Rebase:
            return "rebase";
        }
        return "unknown";
    }

    // A source sample farther than one game frame at 10 FPS is a hitch, not motion.
    constexpr float kMaxSourceIntervalSeconds = 0.1f;
    // One-sample discontinuity gates; beyond these the sample is a snap turn,
    // teleport, or proxy rebuild and must not become interpolated motion.
    constexpr float kMaxTranslationJumpGameUnits = 35.0f;
    constexpr float kMaxRotationJumpDegrees = 15.0f;
    // angle(a^T * b) via trace(a^T * b) = element-wise dot product; identical for
    // row-major and column-major storage because both operands share it.
    inline float rotationDeltaDegrees(const RE::NiMatrix3& a, const RE::NiMatrix3& b) noexcept
    {
        float trace = 0.0f;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                trace += a.entry[row][column] * b.entry[row][column];
            }
        }
        float cosAngle = (trace - 1.0f) * 0.5f;
        cosAngle = cosAngle > 1.0f ? 1.0f : (cosAngle < -1.0f ? -1.0f : cosAngle);
        return std::acos(cosAngle) * 57.29577951308232f;
    }

    inline bool isFiniteVector(const RE::NiPoint3& value) noexcept
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    enum class RootMotionFeedForwardStatus : std::uint8_t
    {
        Unavailable,
        // Player not in locomotion (tracker moving=false or negligible step):
        // no shift, room-scale hand motion is never touched.
        Idle,
        Applied,
        InvalidSample,
        // Source or physics delta unusable, or their ratio outside the sane
        // band (frame hitch): fail closed for this frame.
        DtOutOfRange,
        // Step or resulting shift beyond the teleport bound: fail closed.
        Discontinuity,
    };

    constexpr const char* rootMotionFeedForwardStatusName(RootMotionFeedForwardStatus status) noexcept
    {
        switch (status) {
        case RootMotionFeedForwardStatus::Unavailable:
            return "unavailable";
        case RootMotionFeedForwardStatus::Idle:
            return "idle";
        case RootMotionFeedForwardStatus::Applied:
            return "applied";
        case RootMotionFeedForwardStatus::InvalidSample:
            return "invalid-sample";
        case RootMotionFeedForwardStatus::DtOutOfRange:
            return "dt-out-of-range";
        case RootMotionFeedForwardStatus::Discontinuity:
            return "discontinuity";
        }
        return "unknown";
    }

    struct RootMotionFeedForwardResult
    {
        RE::NiPoint3 shiftGame{};
        RootMotionFeedForwardStatus status = RootMotionFeedForwardStatus::Unavailable;
        bool valid = false;
    };

    // The engine applies at most one root step per game frame; a ratio far
    // outside unity means a hitch or timing anomaly, not locomotion.
    constexpr float kMinFeedForwardDtRatio = 0.25f;
    constexpr float kMaxFeedForwardDtRatio = 4.0f;
    // Below this the "step" is tracker noise at rest, not locomotion.
    constexpr float kMinFeedForwardStepGameUnits = 0.001f;

    /*
     * Predict the player-root displacement the game will apply after this
     * physics update:
     *   shift = lastAppliedRootStep x (currentFrameDt / previousFrameDt).
     *
     * lastAppliedRootStep is the game's own per-frame roomNode-world delta as
     * measured by the runtime-state player-space tracker at the producer that
     * queued the sample. BOTH frame durations come from the engine's own
     * continuous frame-delta global (PhysicsInteraction's per-update series):
     * the mover advances the player by speed x that continuous dt. The
     * 2026-08-16 offline predictor comparison ruled out the alternatives --
     * the ms-quantized bhkWorld delta made the ratio no better than none, and
     * a wall clock sampled at the physics phase carried 9.9-15.5 ms of phase
     * noise on a steady 11.1 ms frame and visibly over/under-shot. With the
     * engine's dt on both sides the dominant stutter term -- speed x frame-dt
     * jitter -- cancels, leaving real speed change only. Stateless and
     * deterministic per sample, so multi-substep re-flushes of one pending
     * target recompute the identical shift. Fails closed to a zero shift on
     * any anomaly; a zero shift is precisely the pre-fix behavior.
     */
    inline RootMotionFeedForwardResult evaluateRootMotionFeedForward(
        const RE::NiPoint3& sourceRootStepGame,
        bool sourceMoving,
        float previousFrameDeltaSeconds,
        float currentFrameDeltaSeconds) noexcept
    {
        if (!isFiniteVector(sourceRootStepGame) ||
            !havok_physics_timing::isUsableDelta(previousFrameDeltaSeconds) ||
            !havok_physics_timing::isUsableDelta(currentFrameDeltaSeconds)) {
            return RootMotionFeedForwardResult{ .status = RootMotionFeedForwardStatus::InvalidSample };
        }

        const float stepSquared =
            sourceRootStepGame.x * sourceRootStepGame.x +
            sourceRootStepGame.y * sourceRootStepGame.y +
            sourceRootStepGame.z * sourceRootStepGame.z;
        if (!sourceMoving || stepSquared < kMinFeedForwardStepGameUnits * kMinFeedForwardStepGameUnits) {
            return RootMotionFeedForwardResult{ .status = RootMotionFeedForwardStatus::Idle, .valid = true };
        }
        if (stepSquared > kMaxTranslationJumpGameUnits * kMaxTranslationJumpGameUnits) {
            return RootMotionFeedForwardResult{ .status = RootMotionFeedForwardStatus::Discontinuity };
        }

        const float ratio = currentFrameDeltaSeconds / previousFrameDeltaSeconds;
        if (!std::isfinite(ratio) || ratio < kMinFeedForwardDtRatio || ratio > kMaxFeedForwardDtRatio) {
            return RootMotionFeedForwardResult{ .status = RootMotionFeedForwardStatus::DtOutOfRange };
        }

        RootMotionFeedForwardResult result{};
        result.shiftGame = RE::NiPoint3{
            sourceRootStepGame.x * ratio,
            sourceRootStepGame.y * ratio,
            sourceRootStepGame.z * ratio,
        };
        const float shiftSquared =
            result.shiftGame.x * result.shiftGame.x +
            result.shiftGame.y * result.shiftGame.y +
            result.shiftGame.z * result.shiftGame.z;
        if (!isFiniteVector(result.shiftGame) ||
            shiftSquared > kMaxTranslationJumpGameUnits * kMaxTranslationJumpGameUnits) {
            return RootMotionFeedForwardResult{ .status = RootMotionFeedForwardStatus::Discontinuity };
        }
        result.status = RootMotionFeedForwardStatus::Applied;
        result.valid = true;
        return result;
    }


    struct GameClockPhaseLock
    {
        bool initialized = false;
        RE::NiPoint3 previousTranslation{};
        RE::NiPoint3 currentTranslation{};
        RE::NiMatrix3 currentRotation{};
        std::uint64_t lastSourceSequence = 0;
        // Largest segment fraction already commanded for the current segment.
        // A re-flush of a stale segment (physics stepping without a new game
        // sample) can only hold at this fraction, never step backward along
        // the segment; a fresh sample resets it to 0.
        float playedFraction = 1.0f;
        std::uint32_t rebaseCount = 0;
        std::uint32_t duplicateSourceCount = 0;
        std::uint32_t invalidSourceCount = 0;

        void reset() noexcept
        {
            *this = GameClockPhaseLock{};
        }

        // Snap: adopt the sample as a degenerate (fully played) segment so the
        // next evaluate holds exactly on it instead of interpolating across a
        // discontinuity.
        void rebaseTo(const RE::NiPoint3& translation, const RE::NiMatrix3& rotation) noexcept
        {
            previousTranslation = translation;
            currentTranslation = translation;
            currentRotation = rotation;
            playedFraction = 1.0f;
            initialized = true;
            ++rebaseCount;
        }

        // Accept one game-frame source sample. Sequence identity keeps
        // multi-substep re-flushes of the same pending target from advancing
        // the source segment twice.
        void advanceSource(const RE::NiPoint3& translation, const RE::NiMatrix3& rotation, float sourceDeltaSeconds, std::uint64_t sourceSequence) noexcept
        {
            if (initialized && sourceSequence == lastSourceSequence) {
                ++duplicateSourceCount;
                return;
            }
            lastSourceSequence = sourceSequence;

            if (!isFiniteVector(translation)) {
                // Fail closed: never adopt a poisoned sample; the drive layer's
                // own guards own the rest of the failure path.
                ++invalidSourceCount;
                return;
            }

            if (!initialized) {
                rebaseTo(translation, rotation);
                return;
            }

            if (!havok_physics_timing::isUsableDelta(sourceDeltaSeconds) || sourceDeltaSeconds > kMaxSourceIntervalSeconds) {
                rebaseTo(translation, rotation);
                return;
            }

            const float dx = translation.x - currentTranslation.x;
            const float dy = translation.y - currentTranslation.y;
            const float dz = translation.z - currentTranslation.z;
            const float jumpSquared = dx * dx + dy * dy + dz * dz;
            if (jumpSquared > kMaxTranslationJumpGameUnits * kMaxTranslationJumpGameUnits ||
                rotationDeltaDegrees(rotation, currentRotation) > kMaxRotationJumpDegrees) {
                rebaseTo(translation, rotation);
                return;
            }

            previousTranslation = currentTranslation;
            currentTranslation = translation;
            currentRotation = rotation;
            playedFraction = 0.0f;
        }

        // Command the segment point for physics substep (substepIndex + 1) /
        // substepCount of the current frame. The frame's last substep reaches
        // fraction 1 and lands EXACTLY on the newest game-frame sample -- the
        // game-clock lock that keeps frame-end proxy positions on the sampled
        // wand path. Always returns a finite, previously accepted (or
        // segment-interpolated) translation.
        RE::NiPoint3 evaluate(std::uint32_t substepIndex, std::uint32_t substepCount, ResampleAction& outAction) noexcept
        {
            if (!initialized) {
                outAction = ResampleAction::Hold;
                return currentTranslation;
            }

            const float count = substepCount > 0 ? static_cast<float>(substepCount) : 1.0f;
            float fraction = (static_cast<float>(substepIndex) + 1.0f) / count;
            fraction = fraction < 0.0f ? 0.0f : (fraction > 1.0f ? 1.0f : fraction);
            if (fraction <= playedFraction) {
                fraction = playedFraction;
                outAction = ResampleAction::Hold;
            } else {
                playedFraction = fraction;
                outAction = fraction >= 1.0f ? ResampleAction::Lock : ResampleAction::Interpolate;
            }
            return RE::NiPoint3{
                previousTranslation.x + (currentTranslation.x - previousTranslation.x) * fraction,
                previousTranslation.y + (currentTranslation.y - previousTranslation.y) * fraction,
                previousTranslation.z + (currentTranslation.z - previousTranslation.z) * fraction,
            };
        }
    };
}
