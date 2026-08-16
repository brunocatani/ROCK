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
 * (index+1)/count reaches 1). ConsumptionFrameRebase then re-expresses that
 * point in the live character-root frame measured after controller movement;
 * at rest this is zero, while stick locomotion removes the source-frame age.
 * Intra-frame substeps interpolate both the raw sample and its associated
 * source root. The commanded velocity absorbs the
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

    struct ControllerRootFrameSample
    {
        RE::NiPoint3 positionHavok{};
        std::uintptr_t controllerIdentity = 0;
        std::uintptr_t controllerVtable = 0;
        std::uint32_t physicsScaleRevision = 0;
        bool valid = false;
    };

    enum class ConsumptionFrameRebaseStatus : std::uint8_t
    {
        Unavailable,
        Stationary,
        Applied,
        InvalidSample,
        ControllerChanged,
        ScaleChanged,
        Discontinuity,
    };

    constexpr const char* consumptionFrameRebaseStatusName(ConsumptionFrameRebaseStatus status) noexcept
    {
        switch (status) {
        case ConsumptionFrameRebaseStatus::Unavailable:
            return "unavailable";
        case ConsumptionFrameRebaseStatus::Stationary:
            return "stationary";
        case ConsumptionFrameRebaseStatus::Applied:
            return "applied";
        case ConsumptionFrameRebaseStatus::InvalidSample:
            return "invalid-sample";
        case ConsumptionFrameRebaseStatus::ControllerChanged:
            return "controller-changed";
        case ConsumptionFrameRebaseStatus::ScaleChanged:
            return "scale-changed";
        case ConsumptionFrameRebaseStatus::Discontinuity:
            return "discontinuity";
        }
        return "unknown";
    }

    struct ConsumptionFrameRebaseResult
    {
        RE::NiPoint3 shiftGame{};
        RE::NiPoint3 currentEndpointShiftGame{};
        ConsumptionFrameRebaseStatus status = ConsumptionFrameRebaseStatus::Unavailable;
        bool valid = false;
    };

    /*
     * Re-express a queued world-space hand sample in the character controller's
     * frame at the exact physics boundary that consumes it.
     *
     * This is deliberately position-only measurement, not velocity prediction:
     * source and consumption positions come from the same live controller. The
     * raw hand/proxy sample already contains the controller root at source time,
     * so adding (consumptionRoot - sourceRoot) removes only the common root-frame
     * age. Room-scale hand motion remains untouched because it does not move the
     * character controller. For multi-substep source interpolation, both source
     * endpoint roots are retained and interpolated with the same fraction as the
     * target, while the consumption root is refreshed after every character move.
     */
    struct ConsumptionFrameRebase
    {
        bool initialized = false;
        RE::NiPoint3 previousSourceRootHavok{};
        RE::NiPoint3 currentSourceRootHavok{};
        std::uintptr_t controllerIdentity = 0;
        std::uint32_t physicsScaleRevision = 0;
        std::uint64_t lastSourceSequence = 0;
        std::uint64_t blockedSourceSequence = 0;
        ConsumptionFrameRebaseStatus blockedStatus = ConsumptionFrameRebaseStatus::Unavailable;
        std::uint32_t appliedCount = 0;
        std::uint32_t rejectedCount = 0;

        void reset() noexcept
        {
            *this = ConsumptionFrameRebase{};
        }

        static float vectorLength(const RE::NiPoint3& value) noexcept
        {
            const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
            return std::isfinite(lengthSquared) && lengthSquared >= 0.0f ?
                       std::sqrt(lengthSquared) :
                       (std::numeric_limits<float>::infinity)();
        }

        static RE::NiPoint3 rootDeltaGame(
            const RE::NiPoint3& consumptionRootHavok,
            const RE::NiPoint3& sourceRootHavok,
            float havokToGameScale) noexcept
        {
            return RE::NiPoint3{
                (consumptionRootHavok.x - sourceRootHavok.x) * havokToGameScale,
                (consumptionRootHavok.y - sourceRootHavok.y) * havokToGameScale,
                (consumptionRootHavok.z - sourceRootHavok.z) * havokToGameScale,
            };
        }

        void adoptSource(const ControllerRootFrameSample& source, std::uint64_t sourceSequence) noexcept
        {
            previousSourceRootHavok = source.positionHavok;
            currentSourceRootHavok = source.positionHavok;
            controllerIdentity = source.controllerIdentity;
            physicsScaleRevision = source.physicsScaleRevision;
            lastSourceSequence = sourceSequence;
            initialized = true;
        }

        ConsumptionFrameRebaseResult reject(
            ConsumptionFrameRebaseStatus status,
            const ControllerRootFrameSample* source,
            std::uint64_t sourceSequence) noexcept
        {
            ++rejectedCount;
            blockedSourceSequence = sourceSequence;
            blockedStatus = status;
            if (source && source->valid && source->controllerIdentity != 0 && sourceSequence != 0) {
                adoptSource(*source, sourceSequence);
            } else {
                initialized = false;
                lastSourceSequence = sourceSequence;
            }
            return ConsumptionFrameRebaseResult{ .status = status };
        }

        ConsumptionFrameRebaseResult evaluate(
            const ControllerRootFrameSample& source,
            const ControllerRootFrameSample& consumption,
            float havokToGameScale,
            std::uint64_t sourceSequence,
            float segmentFraction) noexcept
        {
            if (sourceSequence != 0 && sourceSequence == blockedSourceSequence) {
                return ConsumptionFrameRebaseResult{ .status = blockedStatus };
            }

            if (!source.valid || !consumption.valid || sourceSequence == 0 ||
                source.controllerIdentity == 0 || consumption.controllerIdentity == 0 ||
                !isFiniteVector(source.positionHavok) || !isFiniteVector(consumption.positionHavok) ||
                !std::isfinite(segmentFraction)) {
                return reject(ConsumptionFrameRebaseStatus::InvalidSample, &source, sourceSequence);
            }
            if (source.controllerIdentity != consumption.controllerIdentity) {
                return reject(ConsumptionFrameRebaseStatus::ControllerChanged, &source, sourceSequence);
            }
            if (source.physicsScaleRevision != consumption.physicsScaleRevision ||
                !std::isfinite(havokToGameScale) || havokToGameScale <= 0.0f || havokToGameScale >= 10'000.0f) {
                return reject(ConsumptionFrameRebaseStatus::ScaleChanged, &source, sourceSequence);
            }

            if (!initialized) {
                adoptSource(source, sourceSequence);
            } else {
                if (controllerIdentity != source.controllerIdentity) {
                    return reject(ConsumptionFrameRebaseStatus::ControllerChanged, &source, sourceSequence);
                }
                if (physicsScaleRevision != source.physicsScaleRevision) {
                    return reject(ConsumptionFrameRebaseStatus::ScaleChanged, &source, sourceSequence);
                }

                if (sourceSequence != lastSourceSequence) {
                    const RE::NiPoint3 sourceStepGame = rootDeltaGame(source.positionHavok, currentSourceRootHavok, havokToGameScale);
                    if (sourceSequence < lastSourceSequence ||
                        vectorLength(sourceStepGame) > kMaxTranslationJumpGameUnits) {
                        return reject(ConsumptionFrameRebaseStatus::Discontinuity, &source, sourceSequence);
                    }
                    previousSourceRootHavok = currentSourceRootHavok;
                    currentSourceRootHavok = source.positionHavok;
                    lastSourceSequence = sourceSequence;
                } else {
                    const RE::NiPoint3 duplicateSourceDeltaGame =
                        rootDeltaGame(source.positionHavok, currentSourceRootHavok, havokToGameScale);
                    if (vectorLength(duplicateSourceDeltaGame) > 0.001f) {
                        return reject(ConsumptionFrameRebaseStatus::Discontinuity, &source, sourceSequence);
                    }
                }
            }

            const float fraction = segmentFraction < 0.0f ? 0.0f : (segmentFraction > 1.0f ? 1.0f : segmentFraction);
            const RE::NiPoint3 previousShiftGame =
                rootDeltaGame(consumption.positionHavok, previousSourceRootHavok, havokToGameScale);
            const RE::NiPoint3 currentShiftGame =
                rootDeltaGame(consumption.positionHavok, currentSourceRootHavok, havokToGameScale);
            const RE::NiPoint3 shiftGame{
                previousShiftGame.x + (currentShiftGame.x - previousShiftGame.x) * fraction,
                previousShiftGame.y + (currentShiftGame.y - previousShiftGame.y) * fraction,
                previousShiftGame.z + (currentShiftGame.z - previousShiftGame.z) * fraction,
            };
            if (!isFiniteVector(shiftGame) ||
                vectorLength(currentShiftGame) > kMaxTranslationJumpGameUnits ||
                vectorLength(shiftGame) > kMaxTranslationJumpGameUnits) {
                return reject(ConsumptionFrameRebaseStatus::Discontinuity, &source, sourceSequence);
            }

            ConsumptionFrameRebaseResult result{};
            result.shiftGame = shiftGame;
            result.currentEndpointShiftGame = currentShiftGame;
            result.status = vectorLength(shiftGame) <= 0.0001f ?
                                ConsumptionFrameRebaseStatus::Stationary :
                                ConsumptionFrameRebaseStatus::Applied;
            result.valid = true;
            ++appliedCount;
            return result;
        }
    };

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
