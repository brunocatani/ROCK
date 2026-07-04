#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "physics-interaction/weapon/WeaponClipStrokePolicy.h"
#include "physics-interaction/weapon/WeaponPartMotionPathPolicy.h"

namespace rock
{
    class WeaponPartMotionLearner;

    /*
     * Config-gated reference consumer that lets a hand scrub an animated
     * weapon part along its authored animation stroke. It exercises the
     * public provider API end-to-end from inside ROCK: it registers a real
     * consumer, and per equipped weapon installs NonExclusive AttachOnly
     * whitelist targets ONLY for parts with a stored motion path — authored
     * clip strokes mapped at equip, runtime-learned paths as the fallback
     * (MatchBodyId, generation-scoped) — every part without motion data
     * keeps its normal authority/support grip. Gripped parts
     * are driven with setWeaponPartDriveTargetsV1 — the exact loop an
     * external reload consumer will run. Engine access stays in
     * PhysicsInteraction: this class receives plain weapon-root-local data
     * plus the movable-part set per frame.
     *
     * Ownership/lifetime: registration is lazy on the first enabled update and
     * torn down by shutdown() (drive + whitelist cleared, consumer
     * unregistered). Main-thread only. Drive leases are 2 frames, so a lost
     * frame fails closed into ROCK's baseline-restore path.
     */
    class WeaponPartDriveSandbox
    {
    public:
        static constexpr std::size_t kMaxSourceName = 64;
        // Upper bound of whitelisted movable parts per weapon; matches the
        // caller's drive-part cache capacity.
        static constexpr std::size_t kMaxMovableParts = 24;

        struct HandInput
        {
            // Attach-only grip on a whitelisted movable part owned by this
            // sandbox; false ends any session for the hand.
            bool gripActive{ false };
            std::uint64_t gripSequence{ 0 };
            std::uint32_t bodyId{ 0x7FFF'FFFFu };
            std::string_view sourceName{};
            // All positions weapon-root-local, valid only when resolvable.
            bool transformsValid{ false };
            weapon_part_motion_path::Vec3 partTranslate{};
            float partScale{ 1.0f };
            weapon_part_motion_path::Vec3 handTranslate{};
        };

        struct FrameInput
        {
            std::uint32_t weaponFormId{ 0 };
            std::uint64_t weaponGenerationKey{ 0 };
            /*
             * Evidence parts of the current weapon that own a stored motion
             * path (authored clip stroke or runtime-learned fallback). Only
             * these body IDs are whitelisted for AttachOnly grabs; the set is
             * compared against the installed targets each frame and
             * reinstalled only when it changes.
             */
            std::uint32_t movablePartCount{ 0 };
            std::array<std::uint32_t, kMaxMovableParts> movableBodyIds{};
            // Indexed [0]=right, [1]=left to match hand-state conventions.
            std::array<HandInput, 2> hands{};
        };

        void update(const FrameInput& input, const WeaponPartMotionLearner& learner);
        void shutdown();

        [[nodiscard]] std::uint64_t ownerToken() const { return _ownerToken; }

    private:
        struct HandSession
        {
            bool active{ false };
            std::uint64_t gripSequence{ 0 };
            std::uint32_t bodyId{ 0x7FFF'FFFFu };
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint32_t weaponFormId{ 0 };
            std::array<char, kMaxSourceName> sourceName{};
            float arcPosition{ 0.0f };
            weapon_part_motion_path::Vec3 handStartTranslate{};
            // On-path point nearest the part at grip start; hand displacement
            // is applied relative to it so the desired point stays anchored to
            // the path's own geometry.
            weapon_part_motion_path::Vec3 pathAnchorTranslate{};
            float partScale{ 1.0f };
            /*
             * Authored assembly followers copied at grip start (the learner
             * slot can be replaced mid-session by a fresh harvest, so the
             * session owns its data). Each follower node is driven at the
             * same stroke progress as the leader.
             */
            std::uint32_t followerCount{ 0 };
            std::array<weapon_clip_stroke::AuthoredFollower, weapon_clip_stroke::kMaxFollowers> followers{};
        };

        bool ensureRegistered();
        void refreshMovableTargets(const FrameInput& input);
        void endSession(HandSession& session);

        std::uint64_t _ownerToken{ 0 };
        // Movable-part whitelist currently installed in the provider store;
        // compared against FrameInput to skip redundant setWeaponPartTargets
        // calls. _installedGenerationKey 0 = nothing installed.
        std::uint64_t _installedGenerationKey{ 0 };
        std::uint32_t _installedMovableCount{ 0 };
        std::array<std::uint32_t, kMaxMovableParts> _installedMovableBodyIds{};
        bool _sentDrivesLastUpdate{ false };
        std::uint32_t _registrationRetryCooldownFrames{ 0 };
        bool _registrationWarned{ false };
        std::array<HandSession, 2> _sessions{};
        // Rate-limits the "no learned path yet" hint to once per fresh grip.
        std::array<std::uint64_t, 2> _lastNoPathGripSequence{};
    };
}
