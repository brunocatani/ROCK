#include "api/ProviderLeasePolicy.h"

#ifdef NDEBUG
#undef NDEBUG
#endif
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

    return 0;
}
