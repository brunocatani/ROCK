#pragma once

#include <array>
#include <cstdint>
#include <string_view>

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
        static constexpr std::size_t kMaxStoredPaths = 16;
        static constexpr std::size_t kMaxActiveRecorders = 2;
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

        void observe(const Observation& observation);

        [[nodiscard]] const weapon_part_motion_path::MotionPath* findPath(
            std::uint32_t weaponFormId,
            std::string_view sourceName) const;

        void reset();

    private:
        struct PathSlot
        {
            bool used{ false };
            std::uint32_t weaponFormId{ 0 };
            std::array<char, kMaxSourceName> sourceName{};
            std::uint64_t lastUseCounter{ 0 };
            weapon_part_motion_path::MotionPath path{};
        };

        struct RecorderSlot
        {
            bool used{ false };
            std::uint32_t weaponFormId{ 0 };
            std::array<char, kMaxSourceName> sourceName{};
            std::uint64_t lastSeenCounter{ 0 };
            weapon_part_motion_path::RecorderState state{};
            std::array<weapon_part_motion_path::PoseSample, weapon_part_motion_path::kMaxRecordingSamples> buffer{};
        };

        RecorderSlot* acquireRecorderSlot(std::uint32_t weaponFormId, std::string_view sourceName);
        void storeCompletedPath(const RecorderSlot& recorder);

        std::array<PathSlot, kMaxStoredPaths> _paths{};
        std::array<RecorderSlot, kMaxActiveRecorders> _recorders{};
        std::uint64_t _observationCounter{ 0 };
    };
}
