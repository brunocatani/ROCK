#pragma once

#include <array>
#include <cstdint>
#include <cstring>

#include "physics-interaction/weapon/WeaponPartMotionPathPolicy.h"

/*
 * Pure policy that turns baked animation-clip track samples into authored
 * stroke groups. Input is engine-free: per-track pose arrays sampled at
 * uniform clip times (the harvest hook produces them by calling the engine's
 * own track sampler). Output is one stroke group per moving "leader" track:
 * the leader's rest→peak stroke resampled exactly like learner paths (so the
 * existing scrub consumes it unchanged), plus follower tracks time-aligned to
 * the leader's keys — the whole assembly the animation moves together, driven
 * off one scrub parameter. This is what runtime observation could not give us:
 * per-bone isolated tracks and group coherence.
 */
namespace rock::weapon_clip_stroke
{
    using weapon_part_motion_path::MotionPath;
    using weapon_part_motion_path::PoseSample;
    using weapon_part_motion_path::kMinPathExcursionGameUnits;
    using weapon_part_motion_path::kResampledKeyCount;
    using weapon_part_motion_path::lerpPose;
    using weapon_part_motion_path::poseDistance;

    inline constexpr std::uint32_t kClipSampleCount = 64;
    // Sized so one reload clip can map every moving weapon part: weapon rigs
    // carry bones for mag, bolt, slide, charging handle, hammer and more, and
    // every moving track becomes its own group leader.
    inline constexpr std::uint32_t kMaxTracksPerClip = 16;
    // Sized for real assemblies: a magazine leads its bullet stack, a slide
    // carries its sights and decorative riders.
    inline constexpr std::uint32_t kMaxFollowers = 6;
    inline constexpr std::uint32_t kMaxGroupsPerClip = 8;
    inline constexpr std::size_t kMaxBoneName = 64;
    // Followers move less than leaders (an ejector nudge vs the slide stroke);
    // anything below this is sampling noise and stays undriven.
    inline constexpr float kFollowerMinExcursionGameUnits = 0.15f;

    struct TrackSamples
    {
        std::array<char, kMaxBoneName> boneName{};
        std::uint32_t sampleCount{ 0 };
        std::array<PoseSample, kClipSampleCount> samples{};
    };

    struct AuthoredFollower
    {
        std::array<char, kMaxBoneName> boneName{};
        std::array<PoseSample, kResampledKeyCount> keys{};
        // Weapon-root-local rest scale of the follower node, captured at
        // attribution time; strokes do not animate scale, drives need one.
        float restScale{ 1.0f };
    };

    struct AuthoredStrokeGroup
    {
        std::array<char, kMaxBoneName> leaderBoneName{};
        MotionPath leaderPath{};
        std::uint32_t followerCount{ 0 };
        std::array<AuthoredFollower, kMaxFollowers> followers{};
    };

    inline float trackExcursion(const TrackSamples& track)
    {
        float excursion = 0.0f;
        for (std::uint32_t i = 1; i < track.sampleCount; ++i) {
            const float candidate = poseDistance(track.samples[i], track.samples[0]);
            if (candidate > excursion) {
                excursion = candidate;
            }
        }
        return excursion;
    }

    inline PoseSample poseAtSamplePosition(const TrackSamples& track, float samplePosition)
    {
        if (track.sampleCount == 0) {
            return {};
        }
        if (samplePosition <= 0.0f) {
            return track.samples[0];
        }
        const auto lastIndex = track.sampleCount - 1;
        if (samplePosition >= static_cast<float>(lastIndex)) {
            return track.samples[lastIndex];
        }
        const auto index = static_cast<std::uint32_t>(samplePosition);
        return lerpPose(track.samples[index], track.samples[index + 1], samplePosition - static_cast<float>(index));
    }

    /*
     * Leader path build: identical stroke semantics to the learner's
     * buildPathFromRecording (rest→peak, arc-uniform keys) but also reports
     * each key's fractional source-sample position so followers can be
     * sampled at the same clip times.
     */
    inline bool buildLeaderPath(
        const TrackSamples& track,
        MotionPath& outPath,
        std::array<float, kResampledKeyCount>& outKeySamplePositions)
    {
        outPath = MotionPath{};
        outKeySamplePositions = {};
        if (track.sampleCount < 2) {
            return false;
        }

        std::uint32_t peakIndex = 0;
        float peakExcursion = 0.0f;
        for (std::uint32_t i = 1; i < track.sampleCount; ++i) {
            const float excursion = poseDistance(track.samples[i], track.samples[0]);
            if (excursion > peakExcursion) {
                peakExcursion = excursion;
                peakIndex = i;
            }
        }
        if (peakIndex == 0 || peakExcursion < kMinPathExcursionGameUnits) {
            return false;
        }

        float totalArc = 0.0f;
        for (std::uint32_t i = 1; i <= peakIndex; ++i) {
            totalArc += poseDistance(track.samples[i], track.samples[i - 1]);
        }
        if (!(totalArc > 0.0f)) {
            return false;
        }

        outPath.keys[0] = track.samples[0];
        outKeySamplePositions[0] = 0.0f;
        std::uint32_t segment = 1;
        float arcAtSegmentStart = 0.0f;
        float segmentLength = poseDistance(track.samples[1], track.samples[0]);
        for (std::uint32_t key = 1; key < kResampledKeyCount; ++key) {
            const float targetArc = totalArc * static_cast<float>(key) / static_cast<float>(kResampledKeyCount - 1);
            while (segment < peakIndex && arcAtSegmentStart + segmentLength < targetArc) {
                arcAtSegmentStart += segmentLength;
                ++segment;
                segmentLength = poseDistance(track.samples[segment], track.samples[segment - 1]);
            }
            const float t = segmentLength > 1.0e-6f
                ? (std::min)(1.0f, (std::max)(0.0f, (targetArc - arcAtSegmentStart) / segmentLength))
                : 1.0f;
            outPath.keys[key] = lerpPose(track.samples[segment - 1], track.samples[segment], t);
            outKeySamplePositions[key] = static_cast<float>(segment - 1) + t;
        }
        outPath.totalArcLength = totalArc;
        outPath.valid = true;
        return true;
    }

    /*
     * Build one stroke group per moving track (leaders), with every other
     * moving track attached as a time-aligned follower. Both the bolt and the
     * magazine of a reload clip become leaders of their own groups, each
     * carrying the rest of the assembly as followers.
     */
    inline std::uint32_t buildAuthoredGroups(
        const TrackSamples* tracks,
        std::uint32_t trackCount,
        AuthoredStrokeGroup* outGroups,
        std::uint32_t maxGroups)
    {
        if (!tracks || !outGroups || maxGroups == 0) {
            return 0;
        }

        std::array<float, kMaxTracksPerClip> excursions{};
        const auto boundedTrackCount = (std::min)(trackCount, kMaxTracksPerClip);
        for (std::uint32_t i = 0; i < boundedTrackCount; ++i) {
            excursions[i] = trackExcursion(tracks[i]);
        }

        std::uint32_t groupCount = 0;
        for (std::uint32_t leader = 0; leader < boundedTrackCount && groupCount < maxGroups && groupCount < kMaxGroupsPerClip; ++leader) {
            if (excursions[leader] < kMinPathExcursionGameUnits) {
                continue;
            }
            AuthoredStrokeGroup group{};
            std::array<float, kResampledKeyCount> keyPositions{};
            if (!buildLeaderPath(tracks[leader], group.leaderPath, keyPositions)) {
                continue;
            }
            group.leaderBoneName = tracks[leader].boneName;

            for (std::uint32_t follower = 0; follower < boundedTrackCount && group.followerCount < kMaxFollowers; ++follower) {
                if (follower == leader || excursions[follower] < kFollowerMinExcursionGameUnits) {
                    continue;
                }
                auto& slot = group.followers[group.followerCount];
                slot.boneName = tracks[follower].boneName;
                bool followerMoves = false;
                for (std::uint32_t key = 0; key < kResampledKeyCount; ++key) {
                    slot.keys[key] = poseAtSamplePosition(tracks[follower], keyPositions[key]);
                    if (key > 0 && !followerMoves &&
                        poseDistance(slot.keys[key], slot.keys[0]) >= kFollowerMinExcursionGameUnits) {
                        followerMoves = true;
                    }
                }
                // A track can move in the clip but be still during the
                // leader's stroke window (e.g. hammer only moves at fire);
                // such a follower would just pin its node — drop it.
                if (followerMoves) {
                    ++group.followerCount;
                }
            }

            outGroups[groupCount++] = group;
        }
        return groupCount;
    }

    // Fractional key index for a scrub arc position; mirrors the scrub
    // policy's uniform-arc key spacing so follower poses interpolate at the
    // same place the leader pose was produced.
    inline float keyPositionForArc(const MotionPath& path, float arcPosition)
    {
        const float keySpacing = path.totalArcLength / static_cast<float>(kResampledKeyCount - 1);
        if (!(keySpacing > 0.0f)) {
            return 0.0f;
        }
        const float clamped = (std::min)(path.totalArcLength, (std::max)(0.0f, arcPosition));
        return clamped / keySpacing;
    }

    inline PoseSample followerPoseAtKeyPosition(const AuthoredFollower& follower, float keyPosition)
    {
        if (keyPosition <= 0.0f) {
            return follower.keys[0];
        }
        if (keyPosition >= static_cast<float>(kResampledKeyCount - 1)) {
            return follower.keys[kResampledKeyCount - 1];
        }
        const auto index = static_cast<std::uint32_t>(keyPosition);
        return lerpPose(follower.keys[index], follower.keys[index + 1], keyPosition - static_cast<float>(index));
    }
}
