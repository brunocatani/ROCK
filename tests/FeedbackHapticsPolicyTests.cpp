#include "physics-interaction/feedback/FeedbackHaptics.h"

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

    bool expectEqual(const char* label, std::size_t actual, std::size_t expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %zu got %zu\n", label, expected, actual);
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        const float delta = actual > expected ? actual - expected : expected - actual;
        if (delta <= epsilon) {
            return true;
        }
        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace rock::feedback_haptics;

    bool ok = true;
    FeedbackHaptics haptics{};
    HapticOutput outputs[2]{};

    ok &= expectTrue("first queue succeeds", haptics.queue(FeedbackHand::Right, 0.10f, 0.10f));
    ok &= expectTrue("second queue succeeds", haptics.queue(FeedbackHand::Right, 0.10f, 0.80f));
    std::size_t count = haptics.update(0.01f, outputs, 2);
    ok &= expectEqual("one hand emits one latest pulse", count, 1);
    ok &= expectNear("latest event wins", outputs[0].intensity, 0.80f, 0.001f);

    haptics.reset();
    ok &= expectTrue("fade queue succeeds", haptics.queue(FeedbackHand::Left, 0.10f, 1.0f, 0.0f));
    count = haptics.update(0.05f, outputs, 2);
    ok &= expectEqual("fade starts immediately", count, 1);
    ok &= expectNear("fade starts at start intensity", outputs[0].intensity, 1.0f, 0.001f);
    count = haptics.update(0.025f, outputs, 2);
    ok &= expectEqual("fade continues", count, 1);
    ok &= expectNear("fade interpolates", outputs[0].intensity, 0.50f, 0.001f);
    (void)haptics.update(0.10f, outputs, 2);
    ok &= expectEqual("expired fade clears", haptics.activeEventCount(FeedbackHand::Left), 0);

    haptics.reset();
    ok &= expectTrue("long candidate queue succeeds", haptics.queue(FeedbackHand::Right, 0.10f, 0.20f));
    ok &= expectTrue("short confirmation queue succeeds", haptics.queue(FeedbackHand::Right, 0.02f, 0.90f));
    count = haptics.update(0.01f, outputs, 2);
    ok &= expectEqual("confirmation suppresses older candidate", count, 1);
    ok &= expectNear("confirmation intensity wins", outputs[0].intensity, 0.90f, 0.001f);
    (void)haptics.update(0.02f, outputs, 2);
    count = haptics.update(0.01f, outputs, 2);
    ok &= expectEqual("older candidate does not resume", count, 0);

    haptics.reset();
    for (int i = 0; i < 12; ++i) {
        ok &= expectTrue("capacity queue succeeds", haptics.queue(FeedbackHand::Right, 0.10f, 0.20f));
    }
    ok &= expectEqual("queue stays fixed capacity", haptics.activeEventCount(FeedbackHand::Right), FeedbackHaptics::kEventsPerHand);

    return ok ? 0 : 1;
}
