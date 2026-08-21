#include "physics-interaction/visual/HandWorldAuthorityRegistryPolicy.h"

#include <array>
#include <cstdio>
#include <string_view>
#include <type_traits>

namespace
{
    enum class TestHand : std::uint8_t
    {
        Left,
        Right,
    };

    bool expect(const char* message, const bool condition)
    {
        if (!condition) {
            std::fprintf(stderr, "FAILED: %s\n", message);
        }
        return condition;
    }
}

int main()
{
    namespace policy =
        rock::hand_world_authority_registry_policy;
    constexpr std::size_t providerConsumerCount = 64;
    constexpr std::size_t internalClaimCount = 32;
    constexpr std::size_t exactCapacity =
        providerConsumerCount * 2 + internalClaimCount;
    using Registry = policy::Registry<exactCapacity, TestHand, int>;
    using Entry = Registry::Entry;

    static_assert(std::is_trivially_copyable_v<Entry>);
    static_assert(exactCapacity == 160);

    bool ok = true;
    Registry capacityRegistry{};
    std::array<char, policy::kTagCapacity> tag{};
    for (std::size_t index = 0; index < exactCapacity; ++index) {
        std::snprintf(tag.data(), tag.size(), "claim-%03zu", index);
        const TestHand hand =
            (index & 1u) != 0 ? TestHand::Left : TestHand::Right;
        auto* slot = capacityRegistry.findOrReserve(tag.data(), hand);
        ok &= expect("every admitted provider and internal slot must reserve",
            slot != nullptr);
        if (slot) {
            ok &= expect("every reserved slot must commit",
                capacityRegistry.commit(
                    *slot,
                    tag.data(),
                    hand,
                    policy::Role::Provider,
                    10,
                    static_cast<int>(index)));
        }
    }
    ok &= expect("the registry must admit its complete exact capacity",
        capacityRegistry.activeCount() == exactCapacity);
    ok &= expect("capacity exhaustion must reject before external publication",
        capacityRegistry.findOrReserve("claim-overflow", TestHand::Left) ==
            nullptr);
    ok &= expect("an individual clear must recover capacity",
        capacityRegistry.invalidate("claim-001", TestHand::Left));
    auto* recovered =
        capacityRegistry.findOrReserve("claim-recovered", TestHand::Left);
    ok &= expect("a cleared slot must be immediately reservable",
        recovered != nullptr);
    if (recovered) {
        ok &= expect("the recovered slot must commit exact metadata",
            capacityRegistry.commit(
                *recovered,
                "claim-recovered",
                TestHand::Left,
                policy::Role::SupportGrip,
                40,
                9001));
    }

    policy::Snapshot<TestHand, int> winner{};
    ok &= expect("the newest recovered support claim must be queryable",
        capacityRegistry.winner(TestHand::Left, winner));
    ok &= expect("winner metadata must retain its role and target",
        winner.valid &&
            winner.role == policy::Role::SupportGrip &&
            winner.target == 9001);

    policy::Registry<1, TestHand, int> reservation{};
    auto* pending = reservation.findOrReserve("pending", TestHand::Right);
    ok &= expect("a pending local reservation must occupy capacity",
        pending != nullptr &&
            reservation.findOrReserve("second", TestHand::Right) == nullptr);
    if (pending) {
        reservation.cancelReservation(*pending);
    }
    ok &= expect("a failed external publish must release its reservation",
        reservation.findOrReserve("recovered", TestHand::Right) != nullptr);

    policy::Registry<4, TestHand, int> arbitration{};
    auto publish = [&arbitration](
                       const char* claimTag,
                       const int priority,
                       const int target,
                       const policy::Role role) {
        auto* slot = arbitration.findOrReserve(claimTag, TestHand::Right);
        return slot && arbitration.commit(
                           *slot,
                           claimTag,
                           TestHand::Right,
                           role,
                           priority,
                           target);
    };
    ok &= expect("low priority arbitration claim must publish",
        publish("low", 10, 1, policy::Role::GrabHeld));
    ok &= expect("first high priority arbitration claim must publish",
        publish("high-old", 50, 2, policy::Role::PrimaryGrip));
    ok &= expect("newest equal-priority arbitration claim must publish",
        publish("high-new", 50, 3, policy::Role::Gunstock));
    ok &= expect("arbitration must return a winner",
        arbitration.winner(TestHand::Right, winner));
    ok &= expect("priority ties must select newest sequence with typed metadata",
        winner.target == 3 &&
            winner.role == policy::Role::Gunstock &&
            std::string_view(winner.tag.data(), winner.tagLength) ==
                "high-new");

    arbitration.reset();
    ok &= expect("scheduler or lifecycle cleanup must reset every local claim",
        arbitration.activeCount() == 0 &&
            !arbitration.hasAny(TestHand::Right) &&
            !arbitration.winner(TestHand::Right, winner));

    ok &= expect("weapon roles must explicitly compose with weapon presentation",
        policy::weaponPresentationFollowsRole(policy::Role::PrimaryGrip) &&
            policy::weaponPresentationFollowsRole(policy::Role::Gunstock) &&
            !policy::weaponPresentationFollowsRole(policy::Role::Provider) &&
            !policy::weaponPresentationFollowsRole(policy::Role::GrabHeld));

    return ok ? 0 : 1;
}
