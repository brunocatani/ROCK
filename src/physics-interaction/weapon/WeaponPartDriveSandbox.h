#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "physics-interaction/weapon/WeaponPartMotionPathPolicy.h"

namespace rock
{
    class WeaponPartMotionLearner;

    /*
     * Config-gated reference consumer that lets a hand scrub a bolt-classified
     * weapon part along its learned animation path. It exercises the public
     * provider API end-to-end from inside ROCK: it registers a real consumer,
     * installs a NonExclusive AttachOnly whitelist target for the Bolt action
     * role (so normal support grips stay untouched), and drives the gripped
     * part with setWeaponPartDriveTargetsV1 — the exact loop an external
     * reload consumer will run. Engine access stays in PhysicsInteraction:
     * this class receives plain weapon-root-local data per frame.
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

        struct HandInput
        {
            // Attach-only grip on a Bolt-classified part owned by this
            // sandbox's whitelist; false ends any session for the hand.
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
        };

        bool ensureRegistered();
        void endSession(HandSession& session);

        std::uint64_t _ownerToken{ 0 };
        bool _whitelistInstalled{ false };
        bool _sentDrivesLastUpdate{ false };
        std::uint32_t _registrationRetryCooldownFrames{ 0 };
        bool _registrationWarned{ false };
        std::array<HandSession, 2> _sessions{};
        // Rate-limits the "no learned path yet" hint to once per fresh grip.
        std::array<std::uint64_t, 2> _lastNoPathGripSequence{};
    };
}
