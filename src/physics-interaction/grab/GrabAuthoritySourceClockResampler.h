#pragma once

/*
 * Source-clock to physics-clock trajectory resampling for the grab-authority
 * proxy target.
 *
 * The held-object target is sampled on the game/source clock (precise wall
 * intervals, ~11ms) but consumed on the physics-substep clock (quantized
 * 10/11/12ms). The native keyframe drive computes velocity = error / delta and
 * the solver integrates it over the actual substep delta, so feeding a
 * source-clock displacement through a physics-clock delta modulates the proxy
 * velocity every frame (and multi-substep frames stall on a re-flushed,
 * already-reached target). Both cadences read as held-object stutter under
 * sustained stick locomotion.
 *
 * This resampler keeps both clocks and maps the sampled translation trajectory
 * onto physics time: one segment (previous -> current sample) is retained, and
 * each physics substep evaluates the segment at that substep's end time. It is
 * not locomotion compensation: it never inspects stick input, player velocity,
 * or room deltas, and for constant-speed source motion the commanded velocity
 * becomes exactly the source segment velocity regardless of substep
 * quantization.
 *
 * Rebase anchoring: the game loop samples the frame, then physics integrates
 * up to that sample time, so steady state has substep end times landing inside
 * the latest segment (alpha near 1). Rebases therefore anchor phase at
 * -interval (not 0); a zero anchor would converge to the extrapolation
 * boundary and oscillate. Rotation is deliberately not resampled; it stays on
 * the existing exact/substep path. Rotation enters only the discontinuity
 * check.
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
        Extrapolate,
        Rebase,
    };

    constexpr const char* resampleActionName(ResampleAction action) noexcept
    {
        switch (action) {
        case ResampleAction::Hold:
            return "hold";
        case ResampleAction::Interpolate:
            return "interpolate";
        case ResampleAction::Extrapolate:
            return "extrapolate";
        case ResampleAction::Rebase:
            return "rebase";
        }
        return "unknown";
    }

    // A source sample farther than one game frame at 10 FPS is a hitch, not motion.
    constexpr float kMaxSourceIntervalSeconds = 0.1f;
    // One-sample discontinuity gates; beyond these the sample is a snap turn,
    // teleport, or proxy rebuild and must not become extrapolated motion.
    constexpr float kMaxTranslationJumpGameUnits = 35.0f;
    constexpr float kMaxRotationJumpDegrees = 15.0f;
    // Physics may run at most one source interval past the newest sample on the
    // measured segment velocity before the resampler rebases.
    constexpr float kMaxExtrapolationSourceIntervals = 1.0f;
    // Anchor interval used before any usable source delta has been seen.
    constexpr float kFallbackSourceIntervalSeconds = 1.0f / 90.0f;

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

    struct Resampler
    {
        bool initialized = false;
        RE::NiPoint3 previousTranslation{};
        RE::NiPoint3 currentTranslation{};
        RE::NiMatrix3 currentRotation{};
        float sourceIntervalSeconds = 0.0f;
        // Cumulative physics time minus cumulative source time, bounded by the
        // alpha-window rebases below; both clocks accumulate real time so this
        // oscillates near zero at steady state.
        float phaseSeconds = 0.0f;
        std::uint64_t lastSourceSequence = 0;
        std::uint32_t rebaseCount = 0;
        std::uint32_t duplicateSourceCount = 0;
        std::uint32_t invalidSourceCount = 0;

        void reset() noexcept
        {
            *this = Resampler{};
        }

        void rebaseTo(const RE::NiPoint3& translation, const RE::NiMatrix3& rotation, float anchorIntervalSeconds) noexcept
        {
            const bool usableAnchor = havok_physics_timing::isUsableDelta(anchorIntervalSeconds) && anchorIntervalSeconds <= kMaxSourceIntervalSeconds;
            const float anchor = usableAnchor ? anchorIntervalSeconds : kFallbackSourceIntervalSeconds;
            previousTranslation = translation;
            currentTranslation = translation;
            currentRotation = rotation;
            sourceIntervalSeconds = anchor;
            phaseSeconds = -anchor;
            initialized = true;
            ++rebaseCount;
        }

        // Accept one game-frame source sample. Sequence identity keeps
        // multi-substep re-flushes of the same pending target from advancing
        // the source timeline twice.
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
                rebaseTo(translation, rotation, sourceDeltaSeconds);
                return;
            }

            if (!havok_physics_timing::isUsableDelta(sourceDeltaSeconds) || sourceDeltaSeconds > kMaxSourceIntervalSeconds) {
                rebaseTo(translation, rotation, 0.0f);
                return;
            }

            const float dx = translation.x - currentTranslation.x;
            const float dy = translation.y - currentTranslation.y;
            const float dz = translation.z - currentTranslation.z;
            const float jumpSquared = dx * dx + dy * dy + dz * dz;
            if (jumpSquared > kMaxTranslationJumpGameUnits * kMaxTranslationJumpGameUnits ||
                rotationDeltaDegrees(rotation, currentRotation) > kMaxRotationJumpDegrees) {
                rebaseTo(translation, rotation, sourceDeltaSeconds);
                return;
            }

            previousTranslation = currentTranslation;
            currentTranslation = translation;
            currentRotation = rotation;
            sourceIntervalSeconds = sourceDeltaSeconds;
            phaseSeconds -= sourceDeltaSeconds;
        }

        // Evaluate the drive translation for the END of the upcoming physics
        // substep and advance the physics clock. Always returns a finite,
        // previously accepted (or segment-interpolated) translation.
        RE::NiPoint3 evaluate(float physicsDeltaSeconds, ResampleAction& outAction) noexcept
        {
            if (!initialized) {
                outAction = ResampleAction::Hold;
                return currentTranslation;
            }
            if (havok_physics_timing::isUsableDelta(physicsDeltaSeconds)) {
                phaseSeconds += physicsDeltaSeconds;
            }
            if (sourceIntervalSeconds <= 0.0f) {
                outAction = ResampleAction::Hold;
                return currentTranslation;
            }

            const float alpha = 1.0f + phaseSeconds / sourceIntervalSeconds;
            if (alpha < 0.0f || alpha > 1.0f + kMaxExtrapolationSourceIntervals) {
                // Clock drift beyond the retained safe window in either
                // direction: hold the exact current target and re-anchor.
                rebaseTo(currentTranslation, currentRotation, sourceIntervalSeconds);
                outAction = ResampleAction::Rebase;
                return currentTranslation;
            }

            outAction = alpha <= 1.0f ? ResampleAction::Interpolate : ResampleAction::Extrapolate;
            return RE::NiPoint3{
                previousTranslation.x + (currentTranslation.x - previousTranslation.x) * alpha,
                previousTranslation.y + (currentTranslation.y - previousTranslation.y) * alpha,
                previousTranslation.z + (currentTranslation.z - previousTranslation.z) * alpha,
            };
        }
    };
}
