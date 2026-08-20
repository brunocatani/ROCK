#include "api/ProviderLeasePolicy.h"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace
{
    using namespace rock::provider;

    struct TestSlot
    {
        bool active{ false };
        bool generationChanged{ false };
        std::uint64_t expiresAfterFrame{ 0 };
    };
}

int main()
{
    constexpr std::uint64_t frameIndex = 100;
    std::array<TestSlot, 6> slots{
        TestSlot{ .active = false, .expiresAfterFrame = 50 },
        TestSlot{ .active = true, .expiresAfterFrame = 101 },
        TestSlot{ .active = true, .expiresAfterFrame = 100 },
        TestSlot{
            .active = true,
            .generationChanged = true,
            .expiresAfterFrame = 101,
        },
        TestSlot{
            .active = true,
            .generationChanged = true,
            .expiresAfterFrame = 100,
        },
        TestSlot{ .active = true, .expiresAfterFrame = 0 },
    };
    std::array<RockProviderSuppressionInvalidationReasonV1, 6> reasons{};
    std::array<bool, 6> revoked{};

    const bool changed = rock::provider_lease_policy::pruneExpiredSlots(
        slots.size(),
        frameIndex,
        [&slots](const std::size_t index) { return slots[index].active; },
        [&slots](const std::size_t index) {
            return slots[index].generationChanged;
        },
        [&slots](const std::size_t index) {
            return slots[index].expiresAfterFrame;
        },
        [&revoked, &reasons](
            const std::size_t index,
            const RockProviderSuppressionInvalidationReasonV1 reason) {
            revoked[index] = true;
            reasons[index] = reason;
        });

    assert(changed);
    assert(!revoked[0]);
    assert(!revoked[1]);
    assert(revoked[2]);
    assert(reasons[2] ==
           RockProviderSuppressionInvalidationReasonV1::Expired);
    assert(revoked[3]);
    assert(reasons[3] ==
           RockProviderSuppressionInvalidationReasonV1::GenerationChanged);
    assert(revoked[4]);
    assert(reasons[4] ==
           RockProviderSuppressionInvalidationReasonV1::GenerationChanged);
    assert(revoked[5]);
    assert(reasons[5] ==
           RockProviderSuppressionInvalidationReasonV1::Expired);

    const bool unchanged = rock::provider_lease_policy::pruneExpiredSlots(
        1,
        frameIndex,
        [](const std::size_t) { return true; },
        [](const std::size_t) { return false; },
        [](const std::size_t) { return frameIndex + 1; },
        [](const std::size_t, auto) { assert(false); });
    assert(!unchanged);

    return 0;
}
