#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

/*
 * Pure policy for learning a weapon part's animation-driven motion path from
 * passive per-frame pose observation. No engine types: poses arrive as plain
 * float translation + unit quaternion in weapon-root-local space, sampled only
 * on frames where the engine (not a provider drive) owns the node.
 *
 * Model: a part rests at a stable pose; when an engine animation moves it, the
 * recorder captures the pose sequence until the part is still again, then the
 * path builder keeps the rest→peak-excursion half of that cycle (a bolt cycle
 * goes rest→open→rest; scrubbing wants the opening stroke). The largest
 * completed stroke seen so far wins, so firing and reload animations both
 * contribute and the fullest one is kept. No authored per-weapon data.
 */
namespace rock::weapon_part_motion_path
{
    struct Vec3
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    struct Quat
    {
        float w{ 1.0f };
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    struct PoseSample
    {
        Vec3 translate{};
        Quat rotate{};
    };

    // Motion-start / stillness thresholds in game units (1 unit ≈ 1.43 cm) and
    // radians. Bolt strokes are 4-8 units, so these reject jitter while
    // catching any real stroke.
    inline constexpr float kTranslationEpsilonGameUnits = 0.10f;
    inline constexpr float kRotationEpsilonRadians = 0.02f;
    inline constexpr std::uint32_t kRestStableFramesToArm = 8;
    inline constexpr std::uint32_t kRestReturnFramesToComplete = 6;
    inline constexpr std::uint32_t kMaxRecordingSamples = 720;
    inline constexpr std::uint32_t kResampledKeyCount = 24;
    // Lever-arm scale that converts part rotation to arc length so rotating
    // strokes (bolt-handle lift) contribute to excursion and resampling.
    inline constexpr float kRotationArcRadiusGameUnits = 3.0f;
    // Completed strokes shorter than this are animation noise, not a stroke.
    inline constexpr float kMinPathExcursionGameUnits = 0.35f;
    // A new stroke must beat the stored one by this ratio to replace it.
    inline constexpr float kReplaceExcursionRatio = 1.02f;

    inline Vec3 sub(const Vec3& a, const Vec3& b) { return Vec3{ a.x - b.x, a.y - b.y, a.z - b.z }; }
    inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline float length(const Vec3& a) { return std::sqrt(dot(a, a)); }

    inline float quatDot(const Quat& a, const Quat& b)
    {
        return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline float quatAngleRadians(const Quat& a, const Quat& b)
    {
        const float d = (std::min)(1.0f, std::abs(quatDot(a, b)));
        return 2.0f * std::acos(d);
    }

    inline Quat quatNormalizeOrIdentity(const Quat& q)
    {
        const float lenSq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
        if (!(lenSq > 1.0e-12f) || !std::isfinite(lenSq)) {
            return Quat{};
        }
        const float inv = 1.0f / std::sqrt(lenSq);
        return Quat{ q.w * inv, q.x * inv, q.y * inv, q.z * inv };
    }

    // Normalized linear interpolation along the shorter arc; adequate for the
    // small inter-key rotations left after resampling.
    inline Quat quatNlerp(const Quat& a, const Quat& b, float t)
    {
        const float sign = quatDot(a, b) < 0.0f ? -1.0f : 1.0f;
        return quatNormalizeOrIdentity(Quat{
            a.w + (sign * b.w - a.w) * t,
            a.x + (sign * b.x - a.x) * t,
            a.y + (sign * b.y - a.y) * t,
            a.z + (sign * b.z - a.z) * t });
    }

    inline Vec3 lerp(const Vec3& a, const Vec3& b, float t)
    {
        return Vec3{ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t };
    }

    inline PoseSample lerpPose(const PoseSample& a, const PoseSample& b, float t)
    {
        return PoseSample{ lerp(a.translate, b.translate, t), quatNlerp(a.rotate, b.rotate, t) };
    }

    // Combined translation+rotation distance used for stillness detection,
    // excursion ranking, and arc-length parameterization.
    inline float poseDistance(const PoseSample& a, const PoseSample& b)
    {
        return length(sub(a.translate, b.translate)) +
               quatAngleRadians(a.rotate, b.rotate) * kRotationArcRadiusGameUnits;
    }

    inline bool posesAreStill(const PoseSample& a, const PoseSample& b)
    {
        return length(sub(a.translate, b.translate)) <= kTranslationEpsilonGameUnits &&
               quatAngleRadians(a.rotate, b.rotate) <= kRotationEpsilonRadians;
    }

    enum class RecorderPhase : std::uint8_t
    {
        WaitingForRest,
        Armed,
        Recording,
    };

    enum class StepResult : std::uint8_t
    {
        Idle,
        Armed,
        RecordingActive,
        RecordingComplete,
        RecordingDiscarded,
    };

    struct RecorderState
    {
        RecorderPhase phase{ RecorderPhase::WaitingForRest };
        PoseSample restPose{};
        PoseSample lastSample{};
        bool hasLastSample{ false };
        std::uint32_t stableFrames{ 0 };
        std::uint32_t sampleCount{ 0 };
    };

    /*
     * Advance the recorder one observed frame. `buffer` is caller-owned with
     * capacity kMaxRecordingSamples; on RecordingComplete it holds
     * state.sampleCount samples starting at the rest pose. sampleTrusted must
     * be false on frames where a provider drive (ours or a consumer's) owned
     * the node — an in-flight recording is discarded and the recorder re-seeks
     * rest, because both the motion and the restored pose are not animation
     * evidence.
     */
    inline StepResult step(
        RecorderState& state,
        PoseSample* buffer,
        const PoseSample& sample,
        bool sampleTrusted)
    {
        if (!sampleTrusted || !buffer) {
            const bool discarded = state.phase == RecorderPhase::Recording;
            state = {};
            return discarded ? StepResult::RecordingDiscarded : StepResult::Idle;
        }

        switch (state.phase) {
        case RecorderPhase::WaitingForRest: {
            if (state.hasLastSample && posesAreStill(sample, state.lastSample)) {
                ++state.stableFrames;
            } else {
                state.stableFrames = 0;
            }
            state.lastSample = sample;
            state.hasLastSample = true;
            if (state.stableFrames >= kRestStableFramesToArm) {
                state.phase = RecorderPhase::Armed;
                state.restPose = sample;
                return StepResult::Armed;
            }
            return StepResult::Idle;
        }
        case RecorderPhase::Armed: {
            if (!posesAreStill(sample, state.restPose)) {
                state.phase = RecorderPhase::Recording;
                buffer[0] = state.restPose;
                buffer[1] = sample;
                state.sampleCount = 2;
                state.stableFrames = 0;
                state.lastSample = sample;
                return StepResult::RecordingActive;
            }
            // Track slow idle drift so the stroke baseline stays honest.
            state.restPose = sample;
            state.lastSample = sample;
            return StepResult::Armed;
        }
        case RecorderPhase::Recording: {
            if (state.sampleCount >= kMaxRecordingSamples) {
                state = {};
                return StepResult::RecordingDiscarded;
            }
            buffer[state.sampleCount++] = sample;
            if (posesAreStill(sample, state.lastSample)) {
                ++state.stableFrames;
            } else {
                state.stableFrames = 0;
            }
            state.lastSample = sample;
            if (state.stableFrames >= kRestReturnFramesToComplete) {
                state.phase = RecorderPhase::WaitingForRest;
                const auto completedCount = state.sampleCount;
                state.stableFrames = 0;
                state.sampleCount = completedCount;
                return StepResult::RecordingComplete;
            }
            return StepResult::RecordingActive;
        }
        }
        return StepResult::Idle;
    }

    struct MotionPath
    {
        bool valid{ false };
        std::array<PoseSample, kResampledKeyCount> keys{};
        float totalArcLength{ 0.0f };
    };

    /*
     * Reduce a completed recording to the rest→peak stroke resampled to
     * kResampledKeyCount keys uniform in arc length. Returns false (and leaves
     * outPath invalid) when the stroke is below the noise floor.
     * `outKeySamplePositions` (optional, kResampledKeyCount floats) receives
     * each key's fractional source-sample position so a co-recorded part can
     * be resampled at the SAME frames (time-aligned follower extraction).
     */
    inline bool buildPathFromRecording(
        const PoseSample* samples,
        std::uint32_t sampleCount,
        MotionPath& outPath,
        float* outKeySamplePositions = nullptr)
    {
        outPath = MotionPath{};
        if (!samples || sampleCount < 2) {
            return false;
        }

        std::uint32_t peakIndex = 0;
        float peakExcursion = 0.0f;
        for (std::uint32_t i = 1; i < sampleCount; ++i) {
            const float excursion = poseDistance(samples[i], samples[0]);
            if (excursion > peakExcursion) {
                peakExcursion = excursion;
                peakIndex = i;
            }
        }
        if (peakIndex == 0 || peakExcursion < kMinPathExcursionGameUnits) {
            return false;
        }

        // Cumulative arc length over the opening stroke only.
        // Variable-length scratch is avoided: arc positions are recomputed on
        // the fly during resampling instead of stored.
        float totalArc = 0.0f;
        for (std::uint32_t i = 1; i <= peakIndex; ++i) {
            totalArc += poseDistance(samples[i], samples[i - 1]);
        }
        if (!(totalArc > 0.0f) || !std::isfinite(totalArc)) {
            return false;
        }

        outPath.keys[0] = samples[0];
        if (outKeySamplePositions) {
            outKeySamplePositions[0] = 0.0f;
        }
        std::uint32_t segment = 1;
        float arcAtSegmentStart = 0.0f;
        float segmentLength = poseDistance(samples[1], samples[0]);
        for (std::uint32_t key = 1; key < kResampledKeyCount; ++key) {
            const float targetArc = totalArc * static_cast<float>(key) / static_cast<float>(kResampledKeyCount - 1);
            while (segment < peakIndex && arcAtSegmentStart + segmentLength < targetArc) {
                arcAtSegmentStart += segmentLength;
                ++segment;
                segmentLength = poseDistance(samples[segment], samples[segment - 1]);
            }
            const float t = segmentLength > 1.0e-6f
                ? (std::min)(1.0f, (std::max)(0.0f, (targetArc - arcAtSegmentStart) / segmentLength))
                : 1.0f;
            outPath.keys[key] = lerpPose(samples[segment - 1], samples[segment], t);
            if (outKeySamplePositions) {
                outKeySamplePositions[key] = static_cast<float>(segment - 1) + t;
            }
        }
        outPath.totalArcLength = totalArc;
        outPath.valid = true;
        return true;
    }

    inline bool shouldReplacePath(const MotionPath& existing, const MotionPath& candidate)
    {
        if (!candidate.valid) {
            return false;
        }
        if (!existing.valid) {
            return true;
        }
        return candidate.totalArcLength >= existing.totalArcLength * kReplaceExcursionRatio;
    }
}
