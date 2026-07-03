#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "physics-interaction/weapon/WeaponPartMotionPathPolicy.h"

/*
 * Pure policy that turns hand displacement into a position along a learned
 * weapon-part motion path ("scrubbing"). The hand never gets direct authority:
 * the caller computes a desired point (part translate at grip start plus hand
 * displacement, weapon-root-local), this policy projects it onto the path near
 * the current scrub position, and the resulting keyed pose becomes the drive
 * target — so the part always moves exactly along the animation's own stroke,
 * including its rotation.
 */
namespace rock::weapon_part_motion_scrub
{
    using weapon_part_motion_path::MotionPath;
    using weapon_part_motion_path::PoseSample;
    using weapon_part_motion_path::Vec3;
    using weapon_part_motion_path::kResampledKeyCount;

    // Segments searched on each side of the current position per frame; keeps
    // folded paths (bolt lift + pull share space) from snapping across folds.
    inline constexpr std::uint32_t kScrubSearchWindowSegments = 3;
    // Per-frame scrub travel clamp in arc units; generous for a 90 fps hand
    // pull but blocks single-frame teleports from tracking spikes.
    inline constexpr float kMaxScrubAdvancePerFrame = 2.0f;
    // Segments whose translation span is below this cannot be projected onto
    // and are skipped (pure-rotation stretches of the stroke).
    inline constexpr float kDegenerateSegmentLengthGameUnits = 1.0e-3f;

    struct ScrubResult
    {
        bool valid{ false };
        float arcPosition{ 0.0f };
        PoseSample target{};
    };

    inline float keyArcPosition(const MotionPath& path, std::uint32_t key)
    {
        return path.totalArcLength * static_cast<float>(key) / static_cast<float>(kResampledKeyCount - 1);
    }

    inline PoseSample poseAtArcPosition(const MotionPath& path, float arcPosition)
    {
        const float keySpacing = path.totalArcLength / static_cast<float>(kResampledKeyCount - 1);
        if (!(keySpacing > 0.0f)) {
            return path.keys[0];
        }
        const float clamped = (std::min)(path.totalArcLength, (std::max)(0.0f, arcPosition));
        const float keyFloat = clamped / keySpacing;
        const auto keyIndex = static_cast<std::uint32_t>(keyFloat);
        if (keyIndex + 1 >= kResampledKeyCount) {
            return path.keys[kResampledKeyCount - 1];
        }
        return weapon_part_motion_path::lerpPose(
            path.keys[keyIndex],
            path.keys[keyIndex + 1],
            keyFloat - static_cast<float>(keyIndex));
    }

    namespace detail
    {
        struct SegmentProjection
        {
            bool valid{ false };
            float distance{ 0.0f };
            float arcPosition{ 0.0f };
        };

        inline SegmentProjection projectOntoSegment(
            const MotionPath& path,
            std::uint32_t segment,
            const Vec3& desiredTranslate)
        {
            using weapon_part_motion_path::dot;
            using weapon_part_motion_path::length;
            using weapon_part_motion_path::lerp;
            using weapon_part_motion_path::sub;

            const Vec3& a = path.keys[segment].translate;
            const Vec3& b = path.keys[segment + 1].translate;
            const Vec3 ab = sub(b, a);
            const float abLenSq = dot(ab, ab);
            if (abLenSq < kDegenerateSegmentLengthGameUnits * kDegenerateSegmentLengthGameUnits) {
                return {};
            }
            const float t = (std::min)(1.0f, (std::max)(0.0f, dot(sub(desiredTranslate, a), ab) / abLenSq));
            const Vec3 closest = lerp(a, b, t);
            const float keySpacing = path.totalArcLength / static_cast<float>(kResampledKeyCount - 1);
            return SegmentProjection{
                .valid = true,
                .distance = length(sub(desiredTranslate, closest)),
                .arcPosition = keySpacing * (static_cast<float>(segment) + t),
            };
        }

        inline SegmentProjection bestProjectionInRange(
            const MotionPath& path,
            std::uint32_t firstSegment,
            std::uint32_t lastSegment,
            const Vec3& desiredTranslate)
        {
            SegmentProjection best{};
            for (std::uint32_t segment = firstSegment; segment <= lastSegment; ++segment) {
                const auto candidate = projectOntoSegment(path, segment, desiredTranslate);
                if (candidate.valid && (!best.valid || candidate.distance < best.distance)) {
                    best = candidate;
                }
            }
            return best;
        }
    }

    /*
     * Global nearest projection over the whole path; used once at grip start
     * to seed the scrub position from wherever the part currently sits.
     */
    inline ScrubResult initialScrubPosition(const MotionPath& path, const Vec3& partTranslate)
    {
        if (!path.valid || !(path.totalArcLength > 0.0f)) {
            return {};
        }
        const auto best = detail::bestProjectionInRange(path, 0, kResampledKeyCount - 2, partTranslate);
        if (!best.valid) {
            return {};
        }
        return ScrubResult{
            .valid = true,
            .arcPosition = best.arcPosition,
            .target = poseAtArcPosition(path, best.arcPosition),
        };
    }

    /*
     * Per-frame scrub: window-limited projection around the current arc
     * position with a per-frame travel clamp. When every windowed segment is
     * degenerate the position holds (result stays valid at currentArc), so a
     * pure-rotation stretch parks the scrub instead of snapping it.
     */
    inline ScrubResult scrub(const MotionPath& path, float currentArcPosition, const Vec3& desiredTranslate)
    {
        if (!path.valid || !(path.totalArcLength > 0.0f)) {
            return {};
        }
        const float keySpacing = path.totalArcLength / static_cast<float>(kResampledKeyCount - 1);
        const float clampedCurrent = (std::min)(path.totalArcLength, (std::max)(0.0f, currentArcPosition));
        const std::uint32_t currentSegment = (std::min)(
            kResampledKeyCount - 2,
            keySpacing > 0.0f ? static_cast<std::uint32_t>(clampedCurrent / keySpacing) : 0u);
        const std::uint32_t firstSegment = currentSegment >= kScrubSearchWindowSegments
            ? currentSegment - kScrubSearchWindowSegments
            : 0u;
        const std::uint32_t lastSegment = (std::min)(
            kResampledKeyCount - 2,
            currentSegment + kScrubSearchWindowSegments);

        const auto best = detail::bestProjectionInRange(path, firstSegment, lastSegment, desiredTranslate);
        float newArc = clampedCurrent;
        if (best.valid) {
            const float advance = best.arcPosition - clampedCurrent;
            const float clampedAdvance = (std::min)(kMaxScrubAdvancePerFrame, (std::max)(-kMaxScrubAdvancePerFrame, advance));
            newArc = (std::min)(path.totalArcLength, (std::max)(0.0f, clampedCurrent + clampedAdvance));
        }
        return ScrubResult{
            .valid = true,
            .arcPosition = newArc,
            .target = poseAtArcPosition(path, newArc),
        };
    }
}
