#include "physics-interaction/collision/ContactActivityTracker.h"

#include <cstdio>

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }

        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }

        std::printf("%s expected false\n", label);
        return false;
    }
}

int main()
{
    using namespace rock::contact_activity_tracker;

    bool ok = true;

    // Continuity is an elapsed-time contract: the same real duration expires
    // the active window at every frame rate.
    for (const float frameDelta : { 1.0f / 45.0f, 1.0f / 90.0f, 1.0f / 120.0f }) {
        ContactActivityTracker tracker{};
        const auto first = tracker.registerHandContact(false, 11, 22);
        ok &= expectTrue("first contact tracked", first.tracked);
        ok &= expectTrue("first contact newly active", first.newlyActive);
        ok &= expectTrue("registered contact reads active", tracker.isHandContactActive(false, 11, 22));

        // Age just under the active window: continuing contact.
        float elapsed = 0.0f;
        while (elapsed + frameDelta < static_cast<float>(kActiveContactSeconds)) {
            (void)tracker.advanceFrame(frameDelta);
            elapsed += frameDelta;
        }
        ok &= expectTrue("contact still active within window", tracker.isHandContactActive(false, 11, 22));
        const auto continuing = tracker.registerHandContact(false, 11, 22);
        ok &= expectFalse("within-window re-contact continues", continuing.newlyActive);

        // Age past the active window: the next contact is newly active again.
        elapsed = 0.0f;
        while (elapsed <= static_cast<float>(kActiveContactSeconds)) {
            (void)tracker.advanceFrame(frameDelta);
            elapsed += frameDelta;
        }
        ok &= expectFalse("contact inactive past window", tracker.isHandContactActive(false, 11, 22));
        const auto reentry = tracker.registerHandContact(false, 11, 22);
        ok &= expectTrue("past-window re-contact is newly active", reentry.newlyActive);
    }

    // Invalid frames advance no measured time: activity holds instead of
    // expiring on fabricated progress.
    {
        ContactActivityTracker tracker{};
        (void)tracker.registerHandContact(true, 33, 44);
        for (int i = 0; i < 1000; ++i) {
            (void)tracker.advanceFrame(0.0f);
        }
        ok &= expectTrue("invalid frames hold contact activity", tracker.isHandContactActive(true, 33, 44));
    }

    // Cleanup recycles slots only after the cleanup duration.
    {
        ContactActivityTracker tracker{};
        (void)tracker.registerHandContact(false, 55, 66);
        float elapsed = 0.0f;
        while (elapsed <= static_cast<float>(kCleanupContactSeconds)) {
            (void)tracker.advanceFrame(1.0f / 90.0f);
            elapsed += 1.0f / 90.0f;
        }
        ok &= expectFalse("cleanup window prunes the slot", tracker.isHandContactTracked(false, 55, 66));
    }

    // The frame counter stays a publication identity.
    {
        ContactActivityTracker tracker{};
        (void)tracker.advanceFrame(1.0f / 90.0f);
        (void)tracker.advanceFrame(0.0f);
        ok &= expectTrue("frame identity counts every publication", tracker.currentFrame() == 2);
    }

    return ok ? 0 : 1;
}
