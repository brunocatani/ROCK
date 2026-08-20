#include "physics-interaction/debug/DebugConfigPolicy.h"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>

int main()
{
    using namespace rock::debug_config_policy;

    assert(!subsystemEnabled(false, false));
    assert(!subsystemEnabled(false, true));
    assert(!subsystemEnabled(true, false));
    assert(subsystemEnabled(true, true));

    assert(!childEnabled(false, false));
    assert(!childEnabled(false, true));
    assert(!childEnabled(true, false));
    assert(childEnabled(true, true));
    return 0;
}
