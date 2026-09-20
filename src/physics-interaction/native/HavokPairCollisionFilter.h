#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace RE
{
    class hknpWorld;
}

namespace rock
{
    /*
     * Fixed-capacity, serialized ownership of native counted
     * hknpPairCollisionFilter entries. The service never retains transient
     * body pointers; collision-object identities only guard body-ID reuse.
     * Reconcile on the physics owner, or on the game thread while that owner's
     * PhysicsCallbackQuiescenceGate is held for a structural mutation.
     */
    template <std::size_t MaximumPairs, std::size_t OwnerGroupCount>
    class BasicHavokPairCollisionLeaseSet
    {
    public:
        static_assert(MaximumPairs > 0);
        static_assert(OwnerGroupCount > 0);
        static constexpr std::size_t kMaximumPairs = MaximumPairs;

        struct DesiredPair
        {
            std::uint32_t bodyA{ 0x7FFF'FFFFu };
            std::uint32_t bodyB{ 0x7FFF'FFFFu };
            std::uint8_t ownerGroup{ 0 };
        };

        struct ReconcileResult
        {
            bool filterAvailable{ false };
            std::uint32_t activePairCount{ 0 };
            std::array<std::uint32_t, OwnerGroupCount> activePairsByOwnerGroup{};
        };

        [[nodiscard]] ReconcileResult reconcile(
            RE::hknpWorld* world,
            const DesiredPair* desiredPairs,
            std::size_t desiredPairCount) noexcept;

        // World teardown invalidates both the filter and its pair table. No
        // native call is legal once that world lifetime has ended.
        void abandonWorld() noexcept;

    private:
        struct PairIdentity
        {
            std::uint32_t bodyA{ 0x7FFF'FFFFu };
            std::uint32_t bodyB{ 0x7FFF'FFFFu };
            std::uintptr_t collisionObjectA{ 0 };
            std::uintptr_t collisionObjectB{ 0 };
            std::uint8_t ownerGroup{ 0 };
            bool valid{ false };
        };

        [[nodiscard]] bool resolveFilter(RE::hknpWorld* world) noexcept;
        [[nodiscard]] bool filterIdentityMatches(RE::hknpWorld* world) const noexcept;

        RE::hknpWorld* _world{ nullptr };
        void* _filter{ nullptr };
        std::array<PairIdentity, kMaximumPairs> _activePairs{};
        std::size_t _activePairCount{ 0 };
    };

    using HavokPairCollisionLeaseSet =
        BasicHavokPairCollisionLeaseSet<34, 2>;
}
