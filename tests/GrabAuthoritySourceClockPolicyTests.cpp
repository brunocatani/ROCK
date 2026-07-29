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

    // Room-velocity feed-forward: constant-lead prediction of the room component.
    {
        using rock::grab_authority_source_clock::applyRoomVelocityFeedForward;
        using rock::grab_authority_source_clock::kFeedForwardLeadSeconds;
        const RE::NiPoint3 base{ 100.0f, -50.0f, 25.0f };

        // Walking: target advances by exactly v * constant lead.
        bool applied = false;
        const RE::NiPoint3 walking =
            applyRoomVelocityFeedForward(base, RE::NiPoint3{ 400.0f, 0.0f, 0.0f }, kFeedForwardLeadSeconds, applied);
        ok &= expectTrue("walking feed-forward applies", applied);
        ok &= expectNear("walking feed-forward x", walking.x, 100.0f + 400.0f * kFeedForwardLeadSeconds, 1e-4f);
        ok &= expectNear("walking feed-forward y untouched", walking.y, -50.0f, 1e-6f);

        // The constant lead cancels out of consecutive-substep target
        // differences: two bases one source step apart, predicted with the
        // SAME lead, differ by exactly the source displacement regardless of
        // how the physics dt was quantized (no vCC*(dt_n - dt_prev) noise).
        bool appliedA = false;
        bool appliedB = false;
        const RE::NiPoint3 stepA =
            applyRoomVelocityFeedForward(RE::NiPoint3{ 0.0f, 0.0f, 0.0f }, RE::NiPoint3{ 400.0f, 0.0f, 0.0f }, kFeedForwardLeadSeconds, appliedA);
        const RE::NiPoint3 stepB =
            applyRoomVelocityFeedForward(RE::NiPoint3{ 4.4f, 0.0f, 0.0f }, RE::NiPoint3{ 400.0f, 0.0f, 0.0f }, kFeedForwardLeadSeconds, appliedB);
        ok &= expectTrue("constant-lead pair applies", appliedA && appliedB);
        ok &= expectNear("constant lead cancels in the difference", stepB.x - stepA.x, 4.4f, 1e-5f);

        // Standing: sub-floor speed is controller noise and must not perturb the target.
        applied = true;
        const RE::NiPoint3 standing = applyRoomVelocityFeedForward(base, RE::NiPoint3{ 0.2f, 0.1f, 0.0f }, 0.011f, applied);
        ok &= expectTrue("standing feed-forward is a no-op", !applied);
        ok &= expectNear("standing target byte-identical", standing.x, base.x, 0.0f);

        // Implausible speed (launch/teleport/corrupt read) fails closed.
        applied = true;
        const RE::NiPoint3 launched = applyRoomVelocityFeedForward(base, RE::NiPoint3{ 5000.0f, 0.0f, 0.0f }, 0.011f, applied);
        ok &= expectTrue("over-cap speed fails closed", !applied);
        ok &= expectNear("over-cap target unchanged", launched.x, base.x, 0.0f);

        // Non-finite velocity fails closed.
        applied = true;
        const RE::NiPoint3 poisoned = applyRoomVelocityFeedForward(
            base, RE::NiPoint3{ std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f }, 0.011f, applied);
        ok &= expectTrue("non-finite velocity fails closed", !applied);
        ok &= expectNear("non-finite target unchanged", poisoned.x, base.x, 0.0f);

        // Unusable physics delta fails closed.
        applied = true;
        const RE::NiPoint3 badDelta = applyRoomVelocityFeedForward(base, RE::NiPoint3{ 400.0f, 0.0f, 0.0f }, 0.0f, applied);
        ok &= expectTrue("unusable delta fails closed", !applied);
        ok &= expectNear("unusable delta target unchanged", badDelta.x, base.x, 0.0f);

        // The prediction is per-substep and stateless: repeated application from
        // the same base cannot accumulate (no compensation-class drift).
        applied = false;
        RE::NiPoint3 repeated = base;
        for (int i = 0; i < 100; ++i) {
            repeated = applyRoomVelocityFeedForward(base, RE::NiPoint3{ 400.0f, 0.0f, 0.0f }, 0.011f, applied);
        }
        ok &= expectNear("feed-forward is stateless (no accumulation)", repeated.x, 100.0f + 400.0f * 0.011f, 1e-4f);
    }

    // Segment velocity: the smooth game-clock velocity of the current segment.
    {
        GameClockPhaseLock phaseLock;
        std::uint64_t sequence = 0;
        phaseLock.advanceSource(RE::NiPoint3{ 0.0f, 0.0f, 0.0f }, identity, 0.0111f, ++sequence);
        // First sample is a degenerate init segment (previous == current) -> zero.
        ok &= expectNear("segment velocity is zero on init", phaseLock.segmentVelocity().x, 0.0f, 0.0f);
        phaseLock.advanceSource(RE::NiPoint3{ 4.44f, 0.0f, 0.0f }, identity, 0.0111f, ++sequence);
        ok &= expectNear("segment velocity = displacement / interval", phaseLock.segmentVelocity().x, 4.44f / 0.0111f, 1e-2f);
        // A rebase (teleport) leaves previous == current -> zero, so a teleport
        // never predicts forward.
        phaseLock.advanceSource(RE::NiPoint3{ 500.0f, 0.0f, 0.0f }, identity, 0.0111f, ++sequence);
        ok &= expectNear("segment velocity is zero across a rebase", phaseLock.segmentVelocity().x, 0.0f, 0.0f);
    }

    // Bounded velocity smoother: identity, guards, and the de-quantization claim.
    {
        using rock::grab_authority_source_clock::applyBoundedVelocitySmoothing;
        const RE::NiPoint3 commanded{ 10.0f, 0.0f, 0.0f };
        const RE::NiPoint3 locked{ 14.0f, 0.0f, 0.0f };
        const RE::NiPoint3 segVel{ 400.0f, 0.0f, 0.0f };

        // gain == 1 reproduces the raw phase lock exactly.
        ok &= expectNear("gain 1 == raw phase lock",
            applyBoundedVelocitySmoothing(commanded, locked, segVel, 0.011f, 1.0f, true).x, locked.x, 1e-4f);
        // Uninitialized state adopts the locked target.
        ok &= expectNear("uninitialized adopts locked",
            applyBoundedVelocitySmoothing(commanded, locked, segVel, 0.011f, 0.2f, false).x, locked.x, 0.0f);
        // Discontinuity (locked far from commanded) snaps to locked.
        ok &= expectNear("discontinuity snaps to locked",
            applyBoundedVelocitySmoothing(commanded, RE::NiPoint3{ 100.0f, 0.0f, 0.0f }, segVel, 0.011f, 0.2f, true).x, 100.0f, 0.0f);
        // Unusable dt / non-finite segment velocity fail closed onto locked.
        ok &= expectNear("unusable dt fails closed",
            applyBoundedVelocitySmoothing(commanded, locked, segVel, 0.0f, 0.2f, true).x, locked.x, 0.0f);
        ok &= expectNear("non-finite segment velocity fails closed",
            applyBoundedVelocitySmoothing(commanded, locked,
                RE::NiPoint3{ std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f }, 0.011f, 0.2f, true).x, locked.x, 0.0f);

        // THE claim: driving a locked target that advances by a fixed game-frame
        // delta over a QUANTIZED substep dt, the smoother's commanded VELOCITY
        // (delta / substep dt) has strictly lower variance than the raw lock's --
        // i.e. it removes the +ahead-short/-behind-long dt quantization that is
        // the measured stutter -- and it reports the constant position lag traded
        // for it. Simulated steady straight run.
        const float v = 400.0f;
        const float gameInterval = 0.0111f;
        const float gain = 0.2f;
        RE::NiPoint3 smoothCommanded{ 0.0f, 0.0f, 0.0f };
        float lockedX = 0.0f;
        float prevSmoothX = 0.0f;
        float prevLockedX = 0.0f;
        // Welford accumulators for per-frame velocity (delta / dt).
        int n = 0;
        double rawMean = 0.0, rawM2 = 0.0, smMean = 0.0, smM2 = 0.0;
        float maxLag = 0.0f;
        for (int frame = 0; frame < 400; ++frame) {
            const float substepDt = (frame & 1) ? 0.0120f : 0.0102f; // 11/12ms-style quantization
            lockedX += v * gameInterval; // game-clock sample advances by a fixed delta
            const RE::NiPoint3 lockedTarget{ lockedX, 0.0f, 0.0f };
            const RE::NiPoint3 out = applyBoundedVelocitySmoothing(
                smoothCommanded, lockedTarget, RE::NiPoint3{ v, 0.0f, 0.0f }, substepDt, gain, frame != 0);
            smoothCommanded = out;
            if (frame > 100) { // let both settle
                const float rawVel = (lockedX - prevLockedX) / substepDt;
                const float smVel = (out.x - prevSmoothX) / substepDt;
                ++n;
                const double rd = rawVel - rawMean; rawMean += rd / n; rawM2 += rd * (rawVel - rawMean);
                const double sd = smVel - smMean; smMean += sd / n; smM2 += sd * (smVel - smMean);
                maxLag = (std::max)(maxLag, std::fabs(lockedX - out.x));
            }
            prevSmoothX = out.x;
            prevLockedX = lockedX;
        }
        const double rawStd = std::sqrt(rawM2 / n);
        const double smStd = std::sqrt(smM2 / n);
        std::printf("smoother: raw vel std=%.2f gu/s, smoothed vel std=%.2f gu/s (%.0f%% of raw), steady lag=%.2f gu\n",
            rawStd, smStd, 100.0 * smStd / rawStd, maxLag);
        ok &= expectTrue("smoother reduces commanded-velocity quantization", smStd < rawStd * 0.6);
    }

    return ok ? 0 : 1;
}
