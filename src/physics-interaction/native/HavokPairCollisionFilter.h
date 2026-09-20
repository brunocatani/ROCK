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
    // Optional copied evidence for a single acquisition attempt. No native
    // pointers are retained for dereference and ordinary hand updates omit it.
    struct HavokPairCollisionDiagnostics
    {
        const char* stage{ "not-attempted" };
        std::uintptr_t filter{ 0 };
        std::uintptr_t ownerVtable{ 0 };
        std::uintptr_t expectedVtable{ 0 };
        std::uintptr_t ownerWorld{ 0 };
        std::uintptr_t addedHead{ 0 };
        std::uintptr_t removedHead{ 0 };
        std::uintptr_t lastAddedCallback{ 0 };
        std::uintptr_t lastRemovedCallback{ 0 };
        std::uintptr_t expectedAddedCallback{ 0 };
        std::uintptr_t expectedRemovedCallback{ 0 };
        std::uintptr_t lastAddedOwner{ 0 };
        std::uintptr_t lastRemovedOwner{ 0 };
        std::uint32_t addedSlots{ 0 };
        std::uint32_t removedSlots{ 0 };
        std::uint32_t addedCallbackMatches{ 0 };
        std::uint32_t removedCallbackMatches{ 0 };
        std::uint32_t bodyA{ 0x7FFF'FFFFu };
        std::uint32_t bodyB{ 0x7FFF'FFFFu };
        std::uintptr_t collisionObjectA{ 0 };
        std::uintptr_t collisionObjectB{ 0 };
        std::uint32_t nativeReferenceCount{ 0 };
        std::uint8_t ownerType{ 0xFF };
        bool bodyAValid{ false };
        bool bodyBValid{ false };
        bool nativeCallMade{ false };
        bool pairIdentityChecked{ false };
    };

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
            std::size_t desiredPairCount,
            HavokPairCollisionDiagnostics* diagnostics = nullptr) noexcept;

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

        [[nodiscard]] bool resolveFilter(RE::hknpWorld* world, HavokPairCollisionDiagnostics* diagnostics) noexcept;
        [[nodiscard]] bool filterIdentityMatches(RE::hknpWorld* world, HavokPairCollisionDiagnostics* diagnostics) const noexcept;

        RE::hknpWorld* _world{ nullptr };
        void* _filter{ nullptr };
        std::array<PairIdentity, kMaximumPairs> _activePairs{};
        std::size_t _activePairCount{ 0 };
    };

    using HavokPairCollisionLeaseSet =
        BasicHavokPairCollisionLeaseSet<34, 2>;
}
