#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "physics-interaction/weapon/WeaponClipStrokePolicy.h"
#include "physics-interaction/weapon/WeaponPartMotionPathPolicy.h"

namespace rock
{
    /*
     * Passive runtime that learns weapon-part motion paths from engine
     * animation. PhysicsInteraction feeds it one observation per action part
     * per frame, sampled in weapon-root-local space BEFORE provider drives
     * apply; observations on frames where a drive owned the node arrive with
     * trusted=false so the recorder never learns our own authority back.
     *
     * Learned paths are keyed by (weapon form ID, part source name) so they
     * survive re-equips and weapon switches within the session. Storage is
     * fixed-capacity with oldest-use eviction; no allocation after
     * construction and no I/O. Main-thread only (PhysicsInteraction update).
     */
    class WeaponPartMotionLearner
    {
    public:
        static constexpr std::size_t kMaxStoredPaths = 64;
        // One recorder per evidence part — the learner observes EVERY part,
        // unfiltered by grab eligibility, so every mover of a reload records
        // simultaneously: co-movement grouping (mag pulls its bullets, a
        // slide carries its sights) needs concurrent recordings to compare,
        // and later-phase motions (mag insertion, lever-action feeding) need
        // parts that are still at grab time to be watched too.
        static constexpr std::size_t kMaxActiveRecorders = 48;
        static constexpr std::size_t kMaxSourceName = 64;
        // A recorder whose part was not observed for this many observations is
        // stale (weapon switched / part removed) and may be reclaimed.
        static constexpr std::uint64_t kRecorderStaleObservationAge = 300;

        struct Observation
        {
            std::uint32_t weaponFormId{ 0 };
            std::string_view sourceName{};
            weapon_part_motion_path::PoseSample pose{};
            // False when a provider drive owned the node this frame.
            bool trusted{ true };
        };

        // Call once per frame before the frame's observe() calls: recorder
        // samples are frame-aligned through this counter so concurrent
        // recordings can be compared sample-for-sample.
        void beginObservationFrame();

        void observe(const Observation& observation);

        [[nodiscard]] const weapon_part_motion_path::MotionPath* findPath(
            std::uint32_t weaponFormId,
            std::string_view sourceName) const;

        /*
         * Full stroke-group view for a part: the leader path plus any
         * followers — authored (assembly parts the clip moves with it) or
         * learned (parts observed moving rigidly with the leader during the
         * same stroke). Learned data outranks authored for the same key.
         */
        struct GroupView
        {
            const weapon_part_motion_path::MotionPath* leaderPath{ nullptr };
            const weapon_clip_stroke::AuthoredFollower* followers{ nullptr };
            std::uint32_t followerCount{ 0 };
            bool authored{ false };
        };
        [[nodiscard]] GroupView findGroup(std::uint32_t weaponFormId, std::string_view sourceName) const;

        /*
         * Store a clip-harvested stroke group (already converted to
         * weapon-root-local and mapped to the evidence source name). Authored
         * groups only bootstrap parts without learned data. `fallbackSource`
         * marks strokes from clips merely found LOADED on the graph (shared/
         * template data): the data must be what THIS weapon actually plays,
         * so a stroke from a clip the weapon ACTIVATED always beats a
         * fallback stroke for the same part, and fallback data is used only
         * when it is all that exists. Within the same tier the largest
         * leader stroke wins.
         */
        void storeAuthoredGroup(
            std::uint32_t weaponFormId,
            std::string_view sourceName,
            const weapon_clip_stroke::AuthoredStrokeGroup& group,
            bool fallbackSource);

        void reset();

    private:
        struct PathSlot
        {
            bool used{ false };
            bool authored{ false };
            // Authored-only: stroke from a merely-loaded (fallback) clip;
            // outranked by the weapon's own activated-clip strokes.
            bool fallbackSource{ false };
            std::uint32_t weaponFormId{ 0 };
            std::array<char, kMaxSourceName> sourceName{};
            std::uint64_t lastUseCounter{ 0 };
            weapon_part_motion_path::MotionPath path{};
            std::uint32_t followerCount{ 0 };
            std::array<weapon_clip_stroke::AuthoredFollower, weapon_clip_stroke::kMaxFollowers> followers{};
        };

        struct RecorderSlot
        {
            bool used{ false };
            std::uint32_t weaponFormId{ 0 };
            std::array<char, kMaxSourceName> sourceName{};
            std::uint64_t lastSeenCounter{ 0 };
            // Frame index of buffer[1] (buffer[0] is the pre-motion rest
            // pose); aligns concurrent recordings for co-movement checks.
            std::uint64_t startFrame{ 0 };
            weapon_part_motion_path::RecorderState state{};
            std::array<weapon_part_motion_path::PoseSample, weapon_part_motion_path::kMaxRecordingSamples> buffer{};
        };

        RecorderSlot* acquireRecorderSlot(std::uint32_t weaponFormId, std::string_view sourceName);
        void storeCompletedPath(const RecorderSlot& recorder);

        std::array<PathSlot, kMaxStoredPaths> _paths{};
        std::array<RecorderSlot, kMaxActiveRecorders> _recorders{};
        std::uint64_t _observationCounter{ 0 };
        std::uint64_t _frameCounter{ 0 };
    };
}
