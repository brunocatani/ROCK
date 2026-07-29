#include "physics-interaction/debug/DebugOverlayFrameAdmission.h"

#include <iostream>
#include <utility>

namespace
{
    bool expect(bool condition, const char* message)
    {
        if (!condition) {
            std::cerr << "FAIL: " << message << '\n';
            return false;
        }
        return true;
    }
}

int main()
{
    using rock::debug_overlay_frame_admission::FrameAdmission;

    bool ok = true;
    FrameAdmission admission;

    {
        auto lease = admission.tryAcquire();
        ok &= expect(!lease, "a frame cannot be acquired before publication");
    }

    ok &= expect(admission.publish() == 1, "the first publication serial must be one");
    {
        auto first = admission.tryAcquire();
        ok &= expect(static_cast<bool>(first), "the first published frame must be acquired");

        auto reentrant = admission.tryAcquire();
        ok &= expect(!reentrant, "reentrant drawing must be rejected");

        ok &= expect(admission.publish() == 2, "publication must continue while a draw is active");
        auto concurrent = admission.tryAcquire();
        ok &= expect(!concurrent, "a newer frame cannot draw concurrently with the active frame");
    }

    {
        auto second = admission.tryAcquire();
        ok &= expect(static_cast<bool>(second), "the newer published frame must remain claimable after the active draw completes");
    }

    {
        auto duplicate = admission.tryAcquire();
        ok &= expect(!duplicate, "a published frame may render only once");
    }

    ok &= expect(admission.publish() == 3, "publication serials must be monotonic");
    {
        auto third = admission.tryAcquire();
        ok &= expect(static_cast<bool>(third), "the next published frame must be claimable");
        auto moved = std::move(third);
        ok &= expect(!third && static_cast<bool>(moved), "lease ownership must transfer without releasing the frame early");
    }

    const auto stats = admission.stats();
    ok &= expect(stats.publishedSerial == 3 && stats.acquiredFrames == 3,
        "admission publication/acquisition counters are inaccurate");
    ok &= expect(stats.activeSkips == 2 && stats.noPublicationSkips == 1 && stats.duplicateSkips == 1,
        "admission rejection counters do not distinguish active, unpublished, and duplicate skips");
    ok &= expect(stats.serialRaceSkips == 0, "single-threaded admission reported a serial race");

    return ok ? 0 : 1;
}
