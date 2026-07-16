#include "physics-interaction/weapon/PipboyEquipPolicy.h"

#include <cstdio>

namespace
{
    bool expectResolution(
        const char* label,
        const rock::pipboy_equip_policy::TriggerResolution actual,
        const rock::pipboy_equip_policy::Hand hand,
        const rock::pipboy_equip_policy::TriggerSource source)
    {
        if (actual.hand == hand && actual.source == source) {
            return true;
        }
        std::printf("%s returned hand=%u source=%u\n", label, static_cast<unsigned>(actual.hand), static_cast<unsigned>(actual.source));
        return false;
    }
}

int main()
{
    using namespace rock::pipboy_equip_policy;

    constexpr std::uint32_t generation = 7;
    constexpr std::uint32_t now = 1000;
    constexpr std::uint32_t maximumAge = 500;
    const TransitionToken freshLeft{ .menuGeneration = generation, .tickMilliseconds = 800, .present = true };
    const TransitionToken freshRight{ .menuGeneration = generation, .tickMilliseconds = 900, .present = true };
    const TransitionToken staleLeft{ .menuGeneration = generation, .tickMilliseconds = 400, .present = true };
    const TransitionToken wrongGeneration{ .menuGeneration = generation - 1, .tickMilliseconds = 900, .present = true };
    const TransitionToken none{};

    bool ok = true;
    ok &= expectResolution("left physical level",
        resolveTriggerHand(true, false, none, none, generation, now, maximumAge),
        Hand::Left,
        TriggerSource::PhysicalLevel);
    ok &= expectResolution("right physical level",
        resolveTriggerHand(false, true, none, none, generation, now, maximumAge),
        Hand::Right,
        TriggerSource::PhysicalLevel);
    ok &= expectResolution("both physical levels are ambiguous",
        resolveTriggerHand(true, true, freshLeft, none, generation, now, maximumAge),
        Hand::Right,
        TriggerSource::FallbackRight);
    ok &= expectResolution("fresh left release transition",
        resolveTriggerHand(false, false, freshLeft, none, generation, now, maximumAge),
        Hand::Left,
        TriggerSource::FreshTransition);
    ok &= expectResolution("fresh right release transition",
        resolveTriggerHand(false, false, none, freshRight, generation, now, maximumAge),
        Hand::Right,
        TriggerSource::FreshTransition);
    ok &= expectResolution("two fresh transitions are ambiguous",
        resolveTriggerHand(false, false, freshLeft, freshRight, generation, now, maximumAge),
        Hand::Right,
        TriggerSource::FallbackRight);
    ok &= expectResolution("stale transition",
        resolveTriggerHand(false, false, staleLeft, none, generation, now, maximumAge),
        Hand::Right,
        TriggerSource::FallbackRight);
    ok &= expectResolution("prior menu generation transition",
        resolveTriggerHand(false, false, wrongGeneration, none, generation, now, maximumAge),
        Hand::Right,
        TriggerSource::FallbackRight);
    ok &= expectResolution("no evidence",
        resolveTriggerHand(false, false, none, none, generation, now, maximumAge),
        Hand::Right,
        TriggerSource::FallbackRight);

    if (handTag(Hand::Left) != " [Left]" || handTag(Hand::Right) != " [Right]") {
        std::printf("side tags are incorrect\n");
        ok = false;
    }

    std::uint8_t matchingOffsetFrames = 1;
    if (advanceNativeOffsetReadiness(false, true, matchingOffsetFrames) || matchingOffsetFrames != 0) {
        std::printf("an invalid offset sample did not fail closed\n");
        ok = false;
    }
    if (advanceNativeOffsetReadiness(true, false, matchingOffsetFrames) || matchingOffsetFrames != 0) {
        std::printf("an offset mismatch did not reset readiness\n");
        ok = false;
    }
    if (advanceNativeOffsetReadiness(true, true, matchingOffsetFrames) || matchingOffsetFrames != 1) {
        std::printf("the first offset match did not reserve a canonical refresh frame\n");
        ok = false;
    }
    if (!advanceNativeOffsetReadiness(true, true, matchingOffsetFrames) || matchingOffsetFrames != 2) {
        std::printf("the second consecutive offset match did not become ready\n");
        ok = false;
    }
    if (advanceNativeOffsetReadiness(true, false, matchingOffsetFrames) || matchingOffsetFrames != 0 ||
        advanceNativeOffsetReadiness(true, true, matchingOffsetFrames)) {
        std::printf("readiness did not require two new matches after a mismatch\n");
        ok = false;
    }
    return ok ? 0 : 1;
}
