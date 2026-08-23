#pragma once

/*
 * Game-clock phase lock for the grab-authority proxy target.
 *
 * The held-object target is sampled on the game/source clock and consumed on
 * the physics-substep clock. The global coherent timing schedule makes the
 * coming world's total simulated duration equal to the source duration,
 * scaled only by the native global simulation multiplier. The previous design
 * here -- a source-to-physics-clock
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
 * between the previous and newest sample. Because every substep divides the
 * same complete source duration, this fraction no longer compresses a full
 * source segment into a shorter residual solve. Discontinuity gates (teleport, snap turn,
 * source hitch, proxy rebuild) snap to the new sample instead of interpolating
 * across the jump.
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
    // Exponential filter weight per accepted source sample (~10-sample
    // smoothing): fast enough to track a real rate change within a fraction
    // of a second, slow enough that per-frame pacing jitter moves the lead by
    // well under 0.1 ms.
    constexpr float kSourceIntervalFilterAlpha = 0.1f;

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
        /*
         * Jitter-filtered measured game-source interval (exponential average
         * of validated segment intervals). Zero until the first validated
         * segment. Survives rebases: a teleport or snap turn interrupts the
         * position path, not the source cadence.
         */
        float filteredSourceIntervalSeconds = 0.0f;
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
            // Only validated ordinary intervals feed the cadence filter;
            // hitches and invalid samples took the rebase paths above.
            filteredSourceIntervalSeconds = filteredSourceIntervalSeconds > 0.0f ?
                filteredSourceIntervalSeconds +
                    kSourceIntervalFilterAlpha * (sourceDeltaSeconds - filteredSourceIntervalSeconds) :
                sourceDeltaSeconds;
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
