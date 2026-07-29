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
 * Contract: the LAST physics substep of every frame commands EXACTLY the
 * newest queued game-frame sample (segment fraction (index+1)/count reaches
 * 1), so frame-end proxy positions lie on the sampled wand path exactly like
 * the hand collider's. Intra-frame substeps command the linear interpolation
 * between the previous and newest sample. The commanded velocity absorbs the
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
    // Room-velocity feed-forward speed gates. Below the floor the player is
    // standing (controller noise); above the cap the velocity is not
    // locomotion (launch, script teleport, corrupted read) and must not be
    // predicted into the target.
    constexpr float kFeedForwardMinSpeedGameUnitsPerSecond = 1.0f;
    constexpr float kFeedForwardMaxSpeedGameUnitsPerSecond = 2000.0f;
    /*
     * CONSTANT prediction lead. Predicting each substep's end with that
     * substep's own quantized dt puts the dt-DIFFERENCE into consecutive
     * target displacements (dTgt = vSrc*dt_n + vCC*(dt_n - dt_prev)), which
     * reads as +-vCC*2ms velocity spikes on every 10/11/12ms transition --
     * measured 2026-07-13: target speed 467 on dt-up vs 349 on dt-down at a
     * 412 gu/s walk. A constant lead cancels out of the difference, so the
     * commanded velocity stays exactly the source velocity; the cost is a
     * sub-2ms constant phase error, invisible next to the removed noise.
     */
    constexpr float kFeedForwardLeadSeconds = 1.0f / 90.0f;

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

    /*
     * Room-velocity feed-forward: one-substep prediction of the room-origin
     * component of the grab target.
     *
     * The room origin advances on the physics clock (the engine integrates the
     * player character controller inside the world step), but the grab target
     * is sampled on the game clock one frame earlier. The resampler makes the
     * commanded velocity smooth, yet the commanded POSITION still replays the
     * sampled room trajectory one substep late; with quantized substep deltas
     * the replay lag oscillates and the held object shimmers against the world
     * in proportion to locomotion speed (2026-07-13 telemetry: ~0.7 gu at
     * 400 gu/s).
     *
     * Adding liveCharControllerVelocity * leadSeconds to the resampled target
     * closes the one-frame sampling lag of the room component. leadSeconds
     * MUST be a constant (kFeedForwardLeadSeconds), never the varying substep
     * dt -- see the constant's comment for the measured dt-difference noise a
     * varying lead injects. This is NOT the removed compensation class: there
     * is no position accumulator (the base is the resampled actual trajectory
     * every substep, so error cannot build up), no commanded-vs-actual
     * comparison, and no second behavior path -- standing still the velocity
     * is zero and the result is byte-identical to the unpredicted target.
     */
    inline RE::NiPoint3 applyRoomVelocityFeedForward(const RE::NiPoint3& resampledTranslation,
        const RE::NiPoint3& liveVelocityGameUnitsPerSecond,
        float leadSeconds,
        bool& outApplied) noexcept
    {
        outApplied = false;
        if (!isFiniteVector(resampledTranslation)) {
            return resampledTranslation;
        }
        if (!isFiniteVector(liveVelocityGameUnitsPerSecond) ||
            !havok_physics_timing::isUsableDelta(leadSeconds)) {
            return resampledTranslation;
        }
        const float speedSquared =
            liveVelocityGameUnitsPerSecond.x * liveVelocityGameUnitsPerSecond.x +
            liveVelocityGameUnitsPerSecond.y * liveVelocityGameUnitsPerSecond.y +
            liveVelocityGameUnitsPerSecond.z * liveVelocityGameUnitsPerSecond.z;
        if (!(speedSquared >= kFeedForwardMinSpeedGameUnitsPerSecond * kFeedForwardMinSpeedGameUnitsPerSecond) ||
            speedSquared > kFeedForwardMaxSpeedGameUnitsPerSecond * kFeedForwardMaxSpeedGameUnitsPerSecond) {
            return resampledTranslation;
        }
        outApplied = true;
        return RE::NiPoint3{
            resampledTranslation.x + liveVelocityGameUnitsPerSecond.x * leadSeconds,
            resampledTranslation.y + liveVelocityGameUnitsPerSecond.y * leadSeconds,
            resampledTranslation.z + liveVelocityGameUnitsPerSecond.z * leadSeconds,
        };
    }

    /*
     * Bounded predictor-corrector for the commanded target (opt-in).
     *
     * The game-clock phase lock puts the target POSITION exactly on the newest
     * game sample every frame, but the VELOCITY it hands the motors is the
     * game-frame position delta divided by the physics substep dt -- which
     * quantizes on the engine's 11/11/12 ms substep cycle. That was the lock's
     * own documented, accepted tradeoff ("commanded velocity absorbs the
     * substep-dt quantization; the motors low-pass it"). Cross-session
     * OVERLAY_POINT data (2026-07-14, all resample=lock sessions) showed the
     * motors do NOT low-pass it at speed: the held object's ALONG-track motion
     * carried the quantization as a +ahead-on-short / -behind-on-long-frame
     * drift growing to +-0.6-1.0 gu at a run (cross-track stayed flat) -- the
     * measured stick-locomotion stutter.
     *
     * This advances a persistent commanded target by the SMOOTH game-clock
     * segment velocity each substep (constant velocity => no dt quantization),
     * then corrects a fraction `correctorGain` of the way back to the
     * phase-locked sample so the POSITION cannot drift off the game path the
     * way the removed resampler did. It is NOT that resampler: there is no
     * physics-clock trajectory playback of the sampled path and no unbounded
     * accumulator -- every substep re-anchors toward the live lock.
     *   correctorGain == 1 -> returns the locked target exactly (raw phase lock)
     *   correctorGain in (0,1) -> smoother velocity, bounded constant position lag
     * Fails closed to the raw locked target on first use, non-finite state,
     * unusable dt, or any discontinuity (teleport / rebase / snap) so jumps are
     * never smeared into interpolated motion.
     */
    inline RE::NiPoint3 applyBoundedVelocitySmoothing(const RE::NiPoint3& commandedTranslation,
        const RE::NiPoint3& lockedTarget,
        const RE::NiPoint3& segmentVelocityGameUnitsPerSecond,
        float substepDeltaSeconds,
        float correctorGain,
        bool commandedInitialized) noexcept
    {
        if (!commandedInitialized || !isFiniteVector(commandedTranslation)) {
            return lockedTarget;
        }
        if (!isFiniteVector(lockedTarget)) {
            return commandedTranslation;
        }
        const float dx = lockedTarget.x - commandedTranslation.x;
        const float dy = lockedTarget.y - commandedTranslation.y;
        const float dz = lockedTarget.z - commandedTranslation.z;
        if (dx * dx + dy * dy + dz * dz > kMaxTranslationJumpGameUnits * kMaxTranslationJumpGameUnits) {
            // Discontinuity: snap onto the locked sample, never smear.
            return lockedTarget;
        }
        if (!isFiniteVector(segmentVelocityGameUnitsPerSecond) ||
            !havok_physics_timing::isUsableDelta(substepDeltaSeconds)) {
            return lockedTarget;
        }
        const float gain = correctorGain < 0.0f ? 0.0f : (correctorGain > 1.0f ? 1.0f : correctorGain);
        const float px = commandedTranslation.x + segmentVelocityGameUnitsPerSecond.x * substepDeltaSeconds;
        const float py = commandedTranslation.y + segmentVelocityGameUnitsPerSecond.y * substepDeltaSeconds;
        const float pz = commandedTranslation.z + segmentVelocityGameUnitsPerSecond.z * substepDeltaSeconds;
        return RE::NiPoint3{
            px + (lockedTarget.x - px) * gain,
            py + (lockedTarget.y - py) * gain,
            pz + (lockedTarget.z - pz) * gain,
        };
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
        // Game-clock interval of the current segment (previous->current sample),
        // used only to derive the smooth game-clock segment velocity for the
        // opt-in bounded velocity smoother. Not part of the phase-lock playback.
        float segmentSourceDeltaSeconds = 0.0f;
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
            segmentSourceDeltaSeconds = sourceDeltaSeconds;
        }

        // Smooth game-clock velocity of the current segment (game units/second):
        // (current - previous) sample displacement over the game-frame interval.
        // Zero across a rebase (previous == current) or before a real segment,
        // so a teleport never predicts forward. Used only by the opt-in bounded
        // velocity smoother; independent of the per-substep phase-lock playback.
        RE::NiPoint3 segmentVelocity() const noexcept
        {
            if (!initialized || !havok_physics_timing::isUsableDelta(segmentSourceDeltaSeconds)) {
                return RE::NiPoint3{};
            }
            const float inverseDelta = 1.0f / segmentSourceDeltaSeconds;
            return RE::NiPoint3{
                (currentTranslation.x - previousTranslation.x) * inverseDelta,
                (currentTranslation.y - previousTranslation.y) * inverseDelta,
                (currentTranslation.z - previousTranslation.z) * inverseDelta,
            };
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
