#include "api/ProviderLeasePolicy.h"
#include "api/ProviderFrameClock.h"

#include <cassert>
#include <cstdint>
#include <limits>

int main()
{
    using namespace rock::provider_lease_policy;

    assert(clampLeaseFrames(1, 120) == 1);
    assert(clampLeaseFrames(120, 120) == 120);
    assert(clampLeaseFrames(121, 120) == 120);

    const auto oneFrameExpiry = exclusiveExpiryFrame(100, 1);
    assert(oneFrameExpiry == 101);
    assert(isActive(100, oneFrameExpiry));
    assert(!isActive(101, oneFrameExpiry));
    assert(remainingFrames(100, oneFrameExpiry) == 1);
    assert(remainingFrames(101, oneFrameExpiry) == 0);

    const auto maximumExpiry = exclusiveExpiryFrame(100, 120);
    assert(isActive(219, maximumExpiry));
    assert(!isActive(220, maximumExpiry));
    assert(remainingFrames(100, maximumExpiry) == 120);

    constexpr auto maximumFrame =
        (std::numeric_limits<std::uint64_t>::max)();
    assert(exclusiveExpiryFrame(maximumFrame - 1, 120) == maximumFrame);
    assert(isActive(maximumFrame - 1, maximumFrame));
    assert(!isActive(maximumFrame, maximumFrame));
    assert(!isActive(0, 0));

    // A default one-frame drive published by the end-of-frame callback must
    // reach the following update. Public timestamps already name that update,
    // but its lease retires only at the following provider publication.
    rock::provider::ProviderFrameClock clock;
    clock.beginFrame(100);
    assert(clock.publishFrame() == 100);
    const auto driveExpiry = exclusiveExpiryFrame(clock.leaseBoundary(), 1);
    clock.beginFrame(101);
    assert(clock.current() == 101);
    assert(isActive(clock.leaseBoundary(), driveExpiry));
    assert(clock.publishFrame() == 101);
    assert(!isActive(clock.leaseBoundary(), driveExpiry));

    // Missing provider frames and provider recreation never invent a second
    // counter or rewind timestamps; a second lifecycle publication can reuse
    // this frame identity without consuming another lease frame.
    clock.beginFrame(120);
    assert(clock.current() == 120 && clock.leaseBoundary() == 101);
    assert(clock.publishFrame() == 120);
    assert(clock.publishFrame() == 120);

    return 0;
}
