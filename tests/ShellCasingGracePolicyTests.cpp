#include "physics-interaction/native/ShellCasingGracePolicy.h"

#include <cstdio>
#include <limits>

int main()
{
    using namespace rock::shell_casing_grace;
    bool ok = true;
    const auto expect = [&](bool passed, const char* message) {
        if (!passed) { std::printf("FAIL: %s\n", message); ok = false; }
    };

    const auto tiny = makeWindow(10.0, 42, 1.0f);
    expect(!tiny.expired(10.020, 42), "elapsed time without a completed solve must not remove first-step protection");
    expect(tiny.expired(10.020, 43), "short grace ends after its protected solve");

    const auto first = makeWindow(10.0, 42, 30.0f);
    const auto second = makeWindow(10.025, 44, 30.0f);
    expect(!first.expired(10.020, 44), "multiple short solves must retain the configured duration");
    expect(first.expired(10.040, 45), "the earlier casing expires independently during automatic fire");
    expect(!second.expired(10.040, 45), "the later casing keeps its own deadline");
    expect(!first.expired(10.0, 100), "paused simulation time must not consume the duration");

    const BodyIdentity original{ 25, 80, 0x10000 };
    expect(original.matches(original), "live body identity matches");
    expect(!original.matches({ 25, 81, 0x10000 }), "reused motion slot must not inherit suppression");
    expect(!original.matches({ 25, 80, 0x20000 }), "replacement collision object must not inherit suppression");
    expect(!original.matches({ 26, 80, 0x10000 }), "another casing body must not inherit suppression");
    expect(!BodyIdentity{}.matches(BodyIdentity{}), "missing bodies fail closed");

    expect(sanitizeMilliseconds(0.0f) == 0.0f, "zero disables protection for new casings");
    expect(sanitizeMilliseconds(2.0f) == 2.0f, "millisecond tuning is preserved");
    expect(sanitizeMilliseconds(kMaximumMilliseconds) == kMaximumMilliseconds, "upper limit remains usable");
    expect(sanitizeMilliseconds(-1.0f) == kDefaultMilliseconds, "negative durations use the compiled default");
    expect(sanitizeMilliseconds(501.0f) == kDefaultMilliseconds, "out-of-range durations use the compiled default");
    expect(sanitizeMilliseconds(std::numeric_limits<float>::quiet_NaN()) == kDefaultMilliseconds, "NaN cannot create permanent suppression");
    expect(sanitizeMilliseconds(std::numeric_limits<float>::infinity()) == kDefaultMilliseconds, "infinity cannot create permanent suppression");
    return ok ? 0 : 1;
}
