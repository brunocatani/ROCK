#include "physics-interaction/grab/GrabAuthoritySourceClockResampler.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

namespace
{
    using rock::grab_authority_source_clock::GameClockPhaseLock;
    using rock::grab_authority_source_clock::ResampleAction;
    using rock::grab_authority_source_clock::RootMotionFeedForwardStatus;
    using rock::grab_authority_source_clock::evaluateRootMotionFeedForward;

    constexpr float kPi = 3.14159265358979323846f;

    bool expectTrue(const char* label, bool condition)
    {
        if (!condition) {
            std::printf("%s expected true\n", label);
        }
        return condition;
    }

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        const float delta = std::fabs(actual - expected);
        if (delta <= epsilon) {
            return true;
        }
        std::printf("%s expected %.6f got %.6f\n", label, expected, actual);
        return false;
    }

    RE::NiMatrix3 identityRotation()
    {
        RE::NiMatrix3 matrix{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                matrix.entry[row][column] = row == column ? 1.0f : 0.0f;
            }
        }
        return matrix;
    }

    RE::NiMatrix3 rotationAroundZ(float radians)
    {
        RE::NiMatrix3 matrix = identityRotation();
        const float c = std::cos(radians);
        const float s = std::sin(radians);
        matrix.entry[0][0] = c;
        matrix.entry[0][1] = -s;
        matrix.entry[1][0] = s;
        matrix.entry[1][1] = c;
        return matrix;
    }

}

int main()
{
    bool ok = true;
    const RE::NiMatrix3 identity = identityRotation();

    // THE game-clock lock: with one substep per frame, every frame-end
    // evaluate lands EXACTLY on the newest game-frame sample, so consecutive
    // frame-end positions reproduce the sampled wand path with zero
    // physics-clock deviation (the 2026-07-13 OVERLAY_POINT stutter link).
    {
        GameClockPhaseLock phaseLock;
        const float speed = 400.0f;
        const float sourceDelta = 0.01114f;
        float sourceX = 0.0f;
        std::uint64_t sequence = 0;
        for (int frame = 0; frame < 2000; ++frame) {
            sourceX += speed * sourceDelta;
            phaseLock.advanceSource(RE::NiPoint3{ sourceX, 0.0f, 0.0f }, identity, sourceDelta, ++sequence);
            ResampleAction action = ResampleAction::Hold;
            const RE::NiPoint3 evaluated = phaseLock.evaluate(0, 1, action);
            ok &= expectNear("frame-end evaluate locks on the sample", evaluated.x, sourceX, 0.0f);
            if (frame > 0) {
                // Frame 0 adopts the sample as a degenerate segment (init snap)
                // and reports hold; the position is exact either way.
                ok &= expectTrue("fresh single-substep frame reports lock", action == ResampleAction::Lock);
            }
        }
        ok &= expectTrue("steady state never rebases after initialization", phaseLock.rebaseCount == 1);
    }

    // Multi-substep frames interpolate the sample segment evenly and still
    // lock the frame's last substep on the newest sample.
    {
        GameClockPhaseLock phaseLock;
        std::uint64_t sequence = 0;
        phaseLock.advanceSource(RE::NiPoint3{ 100.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        ResampleAction action = ResampleAction::Hold;
        phaseLock.evaluate(0, 1, action);
        phaseLock.advanceSource(RE::NiPoint3{ 130.0f, 0.0f, 0.0f }, identity, 0.022f, ++sequence);

        RE::NiPoint3 evaluated = phaseLock.evaluate(0, 2, action);
        ok &= expectNear("substep 0/2 commands the segment midpoint", evaluated.x, 115.0f, 1e-3f);
        ok &= expectTrue("substep 0/2 reports interpolate", action == ResampleAction::Interpolate);
        evaluated = phaseLock.evaluate(1, 2, action);
        ok &= expectNear("substep 1/2 locks on the sample", evaluated.x, 130.0f, 0.0f);
        ok &= expectTrue("substep 1/2 reports lock", action == ResampleAction::Lock);

        phaseLock.advanceSource(RE::NiPoint3{ 160.0f, 0.0f, 0.0f }, identity, 0.033f, ++sequence);
        evaluated = phaseLock.evaluate(0, 3, action);
        ok &= expectNear("substep 0/3 commands one third", evaluated.x, 140.0f, 1e-3f);
        evaluated = phaseLock.evaluate(1, 3, action);
        ok &= expectNear("substep 1/3 commands two thirds", evaluated.x, 150.0f, 1e-3f);
        evaluated = phaseLock.evaluate(2, 3, action);
        ok &= expectNear("substep 2/3 locks on the sample", evaluated.x, 160.0f, 0.0f);
        ok &= expectTrue("multi-substep frames never rebase", phaseLock.rebaseCount == 1);
    }

    // A stale segment (physics stepping without a new game sample) holds the
    // newest sample and never steps backward along the segment, even when the
    // stale frame reports multiple substeps.
    {
        GameClockPhaseLock phaseLock;
        std::uint64_t sequence = 0;
        phaseLock.advanceSource(RE::NiPoint3{ 10.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        phaseLock.advanceSource(RE::NiPoint3{ 20.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        ResampleAction action = ResampleAction::Hold;
        RE::NiPoint3 evaluated = phaseLock.evaluate(0, 1, action);
        ok &= expectNear("fresh segment locks", evaluated.x, 20.0f, 0.0f);

        for (int starvedStep = 0; starvedStep < 12; ++starvedStep) {
            evaluated = phaseLock.evaluate(0, 2, action);
            ok &= expectNear("stale multi-substep never steps backward", evaluated.x, 20.0f, 0.0f);
            ok &= expectTrue("stale segment reports hold", action == ResampleAction::Hold);
            evaluated = phaseLock.evaluate(1, 2, action);
            ok &= expectNear("stale segment holds the newest sample", evaluated.x, 20.0f, 0.0f);
        }
        ok &= expectTrue("starved source never rebases", phaseLock.rebaseCount == 1);
    }

    // Duplicate physics flushes of one pending target must not advance the
    // source segment twice.
    {
        GameClockPhaseLock phaseLock;
        std::uint64_t sequence = 0;
        phaseLock.advanceSource(RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        phaseLock.advanceSource(RE::NiPoint3{ 2.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        const float previousBefore = phaseLock.previousTranslation.x;
        const float currentBefore = phaseLock.currentTranslation.x;
        phaseLock.advanceSource(RE::NiPoint3{ 99.0f, 0.0f, 0.0f }, identity, 0.011f, sequence);
        ok &= expectTrue("duplicate sequence is counted", phaseLock.duplicateSourceCount == 1);
        ok &= expectNear("duplicate sequence leaves the segment start alone", phaseLock.previousTranslation.x, previousBefore, 0.0f);
        ok &= expectNear("duplicate sequence leaves the segment end alone", phaseLock.currentTranslation.x, currentBefore, 0.0f);
    }

    // Stationary target: evaluated position holds exactly.
    {
        GameClockPhaseLock phaseLock;
        std::uint64_t sequence = 0;
        const RE::NiPoint3 target{ 5.0f, -3.0f, 8.0f };
        for (int frame = 0; frame < 50; ++frame) {
            phaseLock.advanceSource(target, identity, 0.011f, ++sequence);
            ResampleAction action = ResampleAction::Hold;
            const RE::NiPoint3 evaluated = phaseLock.evaluate(0, 1, action);
            ok &= expectNear("stationary x holds", evaluated.x, target.x, 0.0f);
            ok &= expectNear("stationary y holds", evaluated.y, target.y, 0.0f);
            ok &= expectNear("stationary z holds", evaluated.z, target.z, 0.0f);
        }
    }

    // Invalid, zero, and non-finite source deltas snap to the exact target.
    {
        GameClockPhaseLock phaseLock;
        std::uint64_t sequence = 0;
        phaseLock.advanceSource(RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        phaseLock.advanceSource(RE::NiPoint3{ 2.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);

        phaseLock.advanceSource(RE::NiPoint3{ 3.0f, 0.0f, 0.0f }, identity, 0.0f, ++sequence);
        ok &= expectTrue("zero source delta rebases", phaseLock.rebaseCount == 2);
        ResampleAction action = ResampleAction::Hold;
        RE::NiPoint3 evaluated = phaseLock.evaluate(0, 1, action);
        ok &= expectNear("zero source delta holds the exact target", evaluated.x, 3.0f, 0.0f);

        const float nan = std::nanf("");
        phaseLock.advanceSource(RE::NiPoint3{ 4.0f, 0.0f, 0.0f }, identity, nan, ++sequence);
        ok &= expectTrue("non-finite source delta rebases", phaseLock.rebaseCount == 3);
        evaluated = phaseLock.evaluate(0, 1, action);
        ok &= expectNear("non-finite source delta holds the exact target", evaluated.x, 4.0f, 0.0f);

        phaseLock.advanceSource(RE::NiPoint3{ 5.0f, 0.0f, 0.0f }, identity, 0.2f, ++sequence);
        ok &= expectTrue("hitch-length source delta rebases", phaseLock.rebaseCount == 4);

        phaseLock.advanceSource(RE::NiPoint3{ nan, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        ok &= expectTrue("non-finite target is refused", phaseLock.invalidSourceCount == 1);
        evaluated = phaseLock.evaluate(0, 1, action);
        ok &= expectTrue("non-finite target never reaches the output", std::isfinite(evaluated.x));
    }

    // Degenerate substep metadata fails closed onto the newest sample.
    {
        GameClockPhaseLock phaseLock;
        std::uint64_t sequence = 0;
        phaseLock.advanceSource(RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        phaseLock.advanceSource(RE::NiPoint3{ 2.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        ResampleAction action = ResampleAction::Hold;
        RE::NiPoint3 evaluated = phaseLock.evaluate(0, 0, action);
        ok &= expectNear("zero substep count locks on the sample", evaluated.x, 2.0f, 0.0f);
        phaseLock.advanceSource(RE::NiPoint3{ 3.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        evaluated = phaseLock.evaluate(7, 2, action);
        ok &= expectNear("out-of-range substep index clamps to the sample", evaluated.x, 3.0f, 0.0f);
    }

    // Translation and rotation discontinuities snap to the exact new target
    // instead of interpolating across the jump.
    {
        GameClockPhaseLock phaseLock;
        std::uint64_t sequence = 0;
        phaseLock.advanceSource(RE::NiPoint3{ 0.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        phaseLock.advanceSource(RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);

        phaseLock.advanceSource(RE::NiPoint3{ 51.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        ok &= expectTrue("translation jump rebases", phaseLock.rebaseCount == 2);
        ResampleAction action = ResampleAction::Hold;
        RE::NiPoint3 evaluated = phaseLock.evaluate(0, 2, action);
        ok &= expectNear("translation jump lands exactly on the new target", evaluated.x, 51.0f, 0.0f);

        phaseLock.advanceSource(RE::NiPoint3{ 51.5f, 0.0f, 0.0f }, rotationAroundZ(kPi * 20.0f / 180.0f), 0.011f, ++sequence);
        ok &= expectTrue("rotation jump rebases", phaseLock.rebaseCount == 3);
        evaluated = phaseLock.evaluate(0, 1, action);
        ok &= expectNear("rotation jump lands exactly on the new target", evaluated.x, 51.5f, 0.0f);

        phaseLock.advanceSource(RE::NiPoint3{ 51.6f, 0.0f, 0.0f }, rotationAroundZ(kPi * 22.0f / 180.0f), 0.011f, ++sequence);
        ok &= expectTrue("small rotation step after a jump does not rebase", phaseLock.rebaseCount == 3);
    }

    // Proxy rebuild / world change resets restore the uninitialized state.
    {
        GameClockPhaseLock phaseLock;
        std::uint64_t sequence = 0;
        phaseLock.advanceSource(RE::NiPoint3{ 7.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        phaseLock.reset();
        ok &= expectTrue("reset drops initialization", !phaseLock.initialized);
        ok &= expectTrue("reset clears the sequence", phaseLock.lastSourceSequence == 0);
        ResampleAction action = ResampleAction::Interpolate;
        const RE::NiPoint3 evaluated = phaseLock.evaluate(0, 1, action);
        ok &= expectTrue("uninitialized evaluate holds", action == ResampleAction::Hold);
        ok &= expectNear("uninitialized evaluate returns the reset origin", evaluated.x, 0.0f, 0.0f);
    }

    // Long mixed-cadence run: frame-end positions reproduce the source path
    // exactly, output stays finite, and only the initialization rebase occurs.
    {
        GameClockPhaseLock phaseLock;
        std::uint64_t sequence = 0;
        float sourceX = 0.0f;
        float maxFrameEndError = 0.0f;
        const std::uint32_t substepCounts[] = { 1, 1, 1, 2, 1, 1, 3, 1 };
        for (int frame = 0; frame < 20000; ++frame) {
            const float sourceDelta = 0.010f + 0.003f * ((frame * 7919) % 100) / 100.0f;
            const float speed = 150.0f + 80.0f * std::sin(static_cast<float>(frame) * 0.01f);
            sourceX += speed * sourceDelta;
            phaseLock.advanceSource(RE::NiPoint3{ sourceX, 0.0f, 0.0f }, identity, sourceDelta, ++sequence);
            const std::uint32_t substeps = substepCounts[frame % 8];
            RE::NiPoint3 evaluated{};
            for (std::uint32_t substep = 0; substep < substeps; ++substep) {
                ResampleAction action = ResampleAction::Hold;
                evaluated = phaseLock.evaluate(substep, substeps, action);
                ok &= expectTrue("long run output stays finite", std::isfinite(evaluated.x));
            }
            maxFrameEndError = (std::max)(maxFrameEndError, std::fabs(evaluated.x - sourceX));
        }
        ok &= expectNear("long run frame-end positions reproduce the source path", maxFrameEndError, 0.0f, 1e-3f);
        ok &= expectTrue("long run keeps a single initialization rebase", phaseLock.rebaseCount == 1);
    }

    // Root-motion feed-forward: FO4VR applies joystick locomotion AFTER the
    // physics update (2026-08-16 phase-bracket captures), so the flush
    // predicts its frame's root step as lastAppliedStep x (physicsDt/sourceDt).
    // At rest (tracker moving=false) the shift is exactly zero and cannot
    // alter room-scale hand motion.
    {
        const auto result = evaluateRootMotionFeedForward(RE::NiPoint3{ 0.021f, 0.008f, 0.001f }, false, 0.0111f, 0.0111f);
        ok &= expectTrue("rest sample is valid", result.valid);
        ok &= expectTrue("rest sample reports idle", result.status == RootMotionFeedForwardStatus::Idle);
        ok &= expectNear("rest sample adds no x shift", result.shiftGame.x, 0.0f, 0.0f);
        ok &= expectNear("rest sample adds no y shift", result.shiftGame.y, 0.0f, 0.0f);

        const RE::NiPoint3 roomScaleHandMotion{ 18.0f, -7.0f, 3.0f };
        ok &= expectNear("room-scale hand x is not synthesized", roomScaleHandMotion.x + result.shiftGame.x, roomScaleHandMotion.x, 0.0f);
    }

    // Equal frame durations reproduce the last applied step exactly -- the two
    // measured tire-run locomotion steps pass through unchanged.
    for (const float measuredStepGame : { 1.845f, 3.427f }) {
        const auto result = evaluateRootMotionFeedForward(RE::NiPoint3{ measuredStepGame, 0.0f, 0.0f }, true, 0.0111f, 0.0111f);
        ok &= expectTrue("steady locomotion shift is valid", result.valid);
        ok &= expectTrue("steady locomotion shift is applied", result.status == RootMotionFeedForwardStatus::Applied);
        ok &= expectNear("steady locomotion shift equals the step", result.shiftGame.x, measuredStepGame, 1e-5f);
    }

    // The dominant stutter term was speed x frame-dt jitter: the dt ratio
    // cancels it exactly. A 3.427 gu step over 10.6 ms consumed by a 12.2 ms
    // physics frame predicts the proportionally longer step.
    {
        const auto result = evaluateRootMotionFeedForward(RE::NiPoint3{ 3.427f, 0.0f, 0.0f }, true, 0.0106f, 0.0122f);
        ok &= expectTrue("dt-scaled shift is applied", result.valid && result.status == RootMotionFeedForwardStatus::Applied);
        ok &= expectNear("dt-scaled shift is exact", result.shiftGame.x, 3.427f * 0.0122f / 0.0106f, 1e-4f);
    }

    // Anomalies fail closed with a zero shift (the pre-fix behavior).
    {
        const auto nan = std::numeric_limits<float>::quiet_NaN();
        auto result = evaluateRootMotionFeedForward(RE::NiPoint3{ nan, 0.0f, 0.0f }, true, 0.0111f, 0.0111f);
        ok &= expectTrue("non-finite step is rejected", !result.valid && result.status == RootMotionFeedForwardStatus::InvalidSample);

        result = evaluateRootMotionFeedForward(RE::NiPoint3{ 1.8f, 0.0f, 0.0f }, true, 0.0f, 0.0111f);
        ok &= expectTrue("unusable source delta is rejected", !result.valid && result.status == RootMotionFeedForwardStatus::InvalidSample);

        result = evaluateRootMotionFeedForward(RE::NiPoint3{ 1.8f, 0.0f, 0.0f }, true, 0.0111f, nan);
        ok &= expectTrue("unusable physics delta is rejected", !result.valid && result.status == RootMotionFeedForwardStatus::InvalidSample);

        result = evaluateRootMotionFeedForward(RE::NiPoint3{ 1.8f, 0.0f, 0.0f }, true, 0.011f, 0.06f);
        ok &= expectTrue("hitch-scale dt ratio is rejected", !result.valid && result.status == RootMotionFeedForwardStatus::DtOutOfRange);
        ok &= expectNear("hitch-scale dt ratio adds no shift", result.shiftGame.x, 0.0f, 0.0f);

        result = evaluateRootMotionFeedForward(RE::NiPoint3{ 42.0f, 0.0f, 0.0f }, true, 0.0111f, 0.0111f);
        ok &= expectTrue("teleport-scale step is rejected", !result.valid && result.status == RootMotionFeedForwardStatus::Discontinuity);
        ok &= expectNear("teleport-scale step adds no shift", result.shiftGame.x, 0.0f, 0.0f);

        result = evaluateRootMotionFeedForward(RE::NiPoint3{ 12.0f, 0.0f, 0.0f }, true, 0.011f, 0.04f);
        ok &= expectTrue("shift beyond the teleport bound is rejected",
            !result.valid && result.status == RootMotionFeedForwardStatus::Discontinuity);

        result = evaluateRootMotionFeedForward(RE::NiPoint3{ 0.0005f, 0.0f, 0.0f }, true, 0.0111f, 0.0111f);
        ok &= expectTrue("sub-floor step reports idle", result.valid && result.status == RootMotionFeedForwardStatus::Idle);
    }

    return ok ? 0 : 1;
}
