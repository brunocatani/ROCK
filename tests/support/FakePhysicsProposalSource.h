#pragma once

#include <cstdint>

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

namespace rock::test_support
{
    /*
     * A physics correction is produced on the physics thread and consumed one
     * or more game frames later. Between those two points the world can be
     * torn down, the weapon can be re-equipped, or the body can be rebuilt,
     * and a snapshot from before any of those describes an object that no
     * longer exists.
     *
     * This model carries the same identity tuple the runtime stamps on its
     * snapshot, so a test can present a stale or teleported proposal and
     * check that the consumer refuses it instead of moving the weapon by a
     * correction that belongs to a dead body.
     */
    struct PhysicsProposalIdentity
    {
        std::uintptr_t world = 0;
        std::uint32_t bodyId = 0x7FFF'FFFFu;
        std::uint64_t generationKey = 0;

        [[nodiscard]] friend constexpr bool operator==(
            const PhysicsProposalIdentity&,
            const PhysicsProposalIdentity&) = default;
    };

    struct PhysicsProposalSnapshot
    {
        PhysicsProposalIdentity identity{};
        RE::NiTransform requestedProxyBodyWorld{};
        RE::NiTransform liveProxyBodyWorld{};
        RE::NiPoint3 centerWeaponLocal{};
        float weaponScale = 1.0f;
        std::uint64_t solveSequence = 0;
        bool contactActive = false;
        bool teleported = false;
        bool valid = false;
    };

    class FakePhysicsProposalSource
    {
    public:
        void publish(const PhysicsProposalSnapshot& snapshot)
        {
            _snapshot = snapshot;
            _snapshot.valid = true;
        }

        void clear() { _snapshot = {}; }

        [[nodiscard]] bool read(PhysicsProposalSnapshot& outSnapshot) const
        {
            outSnapshot = _snapshot;
            return _snapshot.valid;
        }

        /*
         * The same admission rule the runtime applies: the snapshot must be
         * valid, must name the current world, body, and weapon generation,
         * and must not describe a teleport. A teleport means the body was
         * moved rather than solved, so its live pose carries no correction.
         */
        [[nodiscard]] static bool isAdmissible(
            const PhysicsProposalSnapshot& snapshot,
            const PhysicsProposalIdentity& current)
        {
            return snapshot.valid &&
                snapshot.identity == current &&
                !snapshot.teleported;
        }

    private:
        PhysicsProposalSnapshot _snapshot{};
    };
}
