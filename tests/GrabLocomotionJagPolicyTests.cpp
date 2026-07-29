#include "physics-interaction/grab/GrabLocomotionJag.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
    using rock::grab_locomotion_jag::JagInput;
    using rock::grab_locomotion_jag::JagSkipReason;

    bool expectTrue(const char* label, bool condition)
    {
        if (!condition) {
            std::printf("%s expected true\n", label);
        }
        return condition;
    }

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        if (std::fabs(actual - expected) <= epsilon) {
            return true;
        }
        std::printf("%s expected %.6f got %.6f\n", label, expected, actual);
        return false;
    }

    bool expectSkip(const char* label, const rock::grab_locomotion_jag::JagCorrection& result, JagSkipReason expected)
    {
        bool ok = true;
        if (result.apply) {
            std::printf("%s expected no correction, got apply\n", label);
            ok = false;
        }
        if (result.skipReason != expected) {
            std::printf("%s expected reason %s got %s\n",
                label,
                rock::grab_locomotion_jag::describe(expected),
                rock::grab_locomotion_jag::describe(result.skipReason));
            ok = false;
        }
        if (result.deltaGameUnits.x != 0.0f || result.deltaGameUnits.y != 0.0f || result.deltaGameUnits.z != 0.0f) {
            std::printf("%s expected zero delta on skip\n", label);
            ok = false;
        }
        return ok;
    }

    // Running straight along +X at `speed`, where the room actually advanced
    // over `renderDeltaSeconds` while the solver will integrate the body with
    // `substepDeltaSeconds`. This is the measured stutter condition.
    JagInput makeRunning(float speed, float renderDeltaSeconds, float substepDeltaSeconds)
    {
        JagInput input{};
        input.previousAnchorValid = true;
        input.previousAnchorGameUnits = RE::NiPoint3{ 1000.0f, 2000.0f, 300.0f };
        input.anchorGameUnits = RE::NiPoint3{ 1000.0f + speed * renderDeltaSeconds, 2000.0f, 300.0f };
        input.roomVelocityValid = true;
        input.roomVelocityGameUnitsPerSecond = RE::NiPoint3{ speed, 0.0f, 0.0f };
        input.substepDeltaSeconds = substepDeltaSeconds;
        input.gain = 1.0f;
        input.maxCorrectionGameUnits = 2.0f;
        return input;
    }
}

int main()
{
    bool ok = true;

    // Standing still: both terms are zero, so the correction must be too. This
    // is the regression that matters most -- standing behaviour is already good
    // and the term must never touch it.
    {
        JagInput input{};
        input.previousAnchorValid = true;
        input.previousAnchorGameUnits = RE::NiPoint3{ 10.0f, 20.0f, 30.0f };
        input.anchorGameUnits = RE::NiPoint3{ 10.0f, 20.0f, 30.0f };
        input.roomVelocityValid = true;
        input.substepDeltaSeconds = 1.0f / 90.0f;
        ok = expectSkip("standing", rock::grab_locomotion_jag::evaluate(input), JagSkipReason::BelowSpeedGate) && ok;
    }

    // Locomotion with even pacing: the room advanced exactly as far as inertia
    // will carry the body, so there is nothing for the motors to have dropped.
    {
        const auto input = makeRunning(430.0f, 1.0f / 90.0f, 1.0f / 90.0f);
        ok = expectSkip("evenPacing", rock::grab_locomotion_jag::evaluate(input), JagSkipReason::BelowEpsilon) && ok;
    }

    // THE BUG: the room advanced over a 12 ms render frame while the solver
    // integrates a 10 ms substep. The correction must be exactly the missing
    // v * (dt_render - dt_substep) = 430 * 0.002 = 0.86 gu.
    {
        const auto input = makeRunning(430.0f, 0.012f, 0.010f);
        const auto result = rock::grab_locomotion_jag::evaluate(input);
        ok = expectTrue("dtMismatch applies", result.apply) && ok;
        ok = expectTrue("dtMismatch not clamped", !result.clamped) && ok;
        ok = expectNear("dtMismatch x", result.deltaGameUnits.x, 0.86f, 0.0005f) && ok;
        ok = expectNear("dtMismatch y", result.deltaGameUnits.y, 0.0f, 1e-6f) && ok;
        ok = expectNear("dtMismatch z", result.deltaGameUnits.z, 0.0f, 1e-6f) && ok;
    }

    // Opposite sign: the solver integrates a LONGER substep than the room
    // actually advanced, so the body must be pulled back.
    {
        const auto input = makeRunning(430.0f, 0.010f, 0.012f);
        const auto result = rock::grab_locomotion_jag::evaluate(input);
        ok = expectTrue("dtMismatchNegative applies", result.apply) && ok;
        ok = expectNear("dtMismatchNegative x", result.deltaGameUnits.x, -0.86f, 0.0005f) && ok;
    }

    // Gain scales a bounded correction; it is not a filter.
    {
        auto input = makeRunning(430.0f, 0.012f, 0.010f);
        input.gain = 0.5f;
        const auto result = rock::grab_locomotion_jag::evaluate(input);
        ok = expectTrue("gainHalf applies", result.apply) && ok;
        ok = expectNear("gainHalf x", result.deltaGameUnits.x, 0.43f, 0.0005f) && ok;

        input.gain = 0.0f;
        ok = expectSkip("gainZero", rock::grab_locomotion_jag::evaluate(input), JagSkipReason::Disabled) && ok;
    }

    // Clamp: an implausible-but-sub-teleport anchor step must be bounded, and
    // must report that it was bounded so the backstop can log loudly.
    {
        auto input = makeRunning(430.0f, 0.012f, 0.010f);
        input.anchorGameUnits.x = input.previousAnchorGameUnits.x + 10.0f;
        const auto result = rock::grab_locomotion_jag::evaluate(input);
        ok = expectTrue("clamp applies", result.apply) && ok;
        ok = expectTrue("clamp reports clamped", result.clamped) && ok;
        const float magnitude = std::sqrt(rock::grab_locomotion_jag::lengthSquared(result.deltaGameUnits));
        ok = expectNear("clamp magnitude", magnitude, 2.0f, 0.0005f) && ok;
    }

    // A teleport, load door, or cell change must never read as locomotion.
    {
        auto input = makeRunning(430.0f, 0.012f, 0.010f);
        input.anchorGameUnits.x = input.previousAnchorGameUnits.x + 500.0f;
        ok = expectSkip("teleport", rock::grab_locomotion_jag::evaluate(input), JagSkipReason::AnchorDiscontinuity) && ok;
    }

    // First flush of a hold has no previous anchor; it must produce nothing
    // rather than a large catch-up.
    {
        auto input = makeRunning(430.0f, 0.012f, 0.010f);
        input.previousAnchorValid = false;
        ok = expectSkip("noPrevious", rock::grab_locomotion_jag::evaluate(input), JagSkipReason::NoPreviousAnchor) && ok;
    }

    // Fail closed on every unusable input.
    {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const float inf = std::numeric_limits<float>::infinity();

        auto nonFiniteAnchor = makeRunning(430.0f, 0.012f, 0.010f);
        nonFiniteAnchor.anchorGameUnits.y = nan;
        ok = expectSkip("nonFiniteAnchor", rock::grab_locomotion_jag::evaluate(nonFiniteAnchor), JagSkipReason::NonFiniteAnchor) && ok;

        auto nonFinitePrevious = makeRunning(430.0f, 0.012f, 0.010f);
        nonFinitePrevious.previousAnchorGameUnits.z = inf;
        ok = expectSkip("nonFinitePrevious", rock::grab_locomotion_jag::evaluate(nonFinitePrevious), JagSkipReason::NonFiniteAnchor) && ok;

        auto noVelocity = makeRunning(430.0f, 0.012f, 0.010f);
        noVelocity.roomVelocityValid = false;
        ok = expectSkip("noVelocity", rock::grab_locomotion_jag::evaluate(noVelocity), JagSkipReason::NoRoomVelocity) && ok;

        auto nonFiniteVelocity = makeRunning(430.0f, 0.012f, 0.010f);
        nonFiniteVelocity.roomVelocityGameUnitsPerSecond.x = nan;
        ok = expectSkip("nonFiniteVelocity", rock::grab_locomotion_jag::evaluate(nonFiniteVelocity), JagSkipReason::NoRoomVelocity) && ok;

        auto overspeed = makeRunning(430.0f, 0.012f, 0.010f);
        overspeed.roomVelocityGameUnitsPerSecond.x = 5000.0f;
        ok = expectSkip("overspeed", rock::grab_locomotion_jag::evaluate(overspeed), JagSkipReason::AboveSpeedCap) && ok;

        auto zeroDelta = makeRunning(430.0f, 0.012f, 0.010f);
        zeroDelta.substepDeltaSeconds = 0.0f;
        ok = expectSkip("zeroDelta", rock::grab_locomotion_jag::evaluate(zeroDelta), JagSkipReason::UnusableDelta) && ok;

        auto hugeDelta = makeRunning(430.0f, 0.012f, 0.010f);
        hugeDelta.substepDeltaSeconds = 5.0f;
        ok = expectSkip("hugeDelta", rock::grab_locomotion_jag::evaluate(hugeDelta), JagSkipReason::UnusableDelta) && ok;

        auto noClamp = makeRunning(430.0f, 0.012f, 0.010f);
        noClamp.maxCorrectionGameUnits = 0.0f;
        ok = expectSkip("noClampBudget", rock::grab_locomotion_jag::evaluate(noClamp), JagSkipReason::Disabled) && ok;

        auto nanGain = makeRunning(430.0f, 0.012f, 0.010f);
        nanGain.gain = nan;
        ok = expectSkip("nanGain", rock::grab_locomotion_jag::evaluate(nanGain), JagSkipReason::Disabled) && ok;
    }

    // The correction is a pure function of its input: evaluating twice with the
    // same input must not drift, because nothing accumulates across frames.
    {
        const auto input = makeRunning(430.0f, 0.012f, 0.010f);
        const auto first = rock::grab_locomotion_jag::evaluate(input);
        const auto second = rock::grab_locomotion_jag::evaluate(input);
        ok = expectNear("pureX", first.deltaGameUnits.x, second.deltaGameUnits.x, 0.0f) && ok;
        ok = expectTrue("pureApply", first.apply == second.apply) && ok;
    }

    // Sanity on the real measured magnitudes: at the observed sprint speed and
    // the observed 1.5 ms pacing spread, the correction must land in the
    // 0.4-0.7 gu band the mechanism doc predicts -- not near the 4.8 gu/frame
    // a full rigid transport would have moved.
    {
        const auto input = makeRunning(430.0f, 0.0111f + 0.0015f, 0.0111f);
        const auto result = rock::grab_locomotion_jag::evaluate(input);
        ok = expectTrue("measuredBand applies", result.apply) && ok;
        const float magnitude = std::sqrt(rock::grab_locomotion_jag::lengthSquared(result.deltaGameUnits));
        ok = expectTrue("measuredBand lower", magnitude > 0.4f) && ok;
        ok = expectTrue("measuredBand upper", magnitude < 0.7f) && ok;
    }

    if (!ok) {
        std::printf("GrabLocomotionJagPolicyTests FAILED\n");
        return 1;
    }
    std::printf("GrabLocomotionJagPolicyTests passed\n");
    return 0;
}
