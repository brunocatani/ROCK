#include "physics-interaction/grab/GrabAuthoritySourceClockResampler.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace
{
    using rock::grab_authority_source_clock::ResampleAction;
    using rock::grab_authority_source_clock::Resampler;

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

    // Engine-shaped physics quantizer: whole-millisecond substep deltas with a
    // carried remainder, mimicking the observed 10/11/12ms quantization of the
    // FO4VR physics clock against precise source frame intervals.
    struct MillisecondQuantizer
    {
        float carrySeconds = 0.0f;

        float quantize(float sourceDeltaSeconds)
        {
            const float total = sourceDeltaSeconds + carrySeconds;
            float quantized = std::round(total * 1000.0f) / 1000.0f;
            if (quantized < 0.001f) {
                quantized = 0.001f;
            }
            carrySeconds = total - quantized;
            return quantized;
        }
    };
}

int main()
{
    bool ok = true;
    const RE::NiMatrix3 identity = identityRotation();

    // Constant-speed source motion, precise source deltas, quantized physics
    // deltas: per-substep commanded velocity must be the source velocity, not
    // the physics-clock-modulated one.
    {
        Resampler resampler;
        MillisecondQuantizer quantizer;
        const float speed = 200.0f;
        const float sourceDelta = 0.01114f;
        float sourceX = 0.0f;
        std::uint64_t sequence = 0;
        float previousEvaluatedX = 0.0f;
        bool warm = false;
        float maxVelocityError = 0.0f;

        for (int frame = 0; frame < 2000; ++frame) {
            sourceX += speed * sourceDelta;
            resampler.advanceSource(RE::NiPoint3{ sourceX, 0.0f, 0.0f }, identity, sourceDelta, ++sequence);
            const float physicsDelta = quantizer.quantize(sourceDelta);
            ResampleAction action = ResampleAction::Hold;
            const RE::NiPoint3 evaluated = resampler.evaluate(physicsDelta, action);
            if (warm && frame > 2) {
                const float velocity = (evaluated.x - previousEvaluatedX) / physicsDelta;
                maxVelocityError = (std::max)(maxVelocityError, std::fabs(velocity - speed));
            }
            previousEvaluatedX = evaluated.x;
            warm = true;
        }

        ok &= expectNear("steady-state velocity stays on the source clock", maxVelocityError, 0.0f, 0.5f);
        ok &= expectTrue("steady state never rebases after initialization", resampler.rebaseCount == 1);
        ok &= expectNear("phase stays bounded at steady state", resampler.phaseSeconds, 0.0f, 2.0f * sourceDelta);
    }

    // Exact target agreement whenever cumulative source and physics time coincide.
    {
        Resampler resampler;
        const float sourceDelta = 0.01f;
        float sourceX = 0.0f;
        std::uint64_t sequence = 0;
        for (int frame = 0; frame < 200; ++frame) {
            sourceX += 150.0f * sourceDelta;
            resampler.advanceSource(RE::NiPoint3{ sourceX, 0.0f, 0.0f }, identity, sourceDelta, ++sequence);
            ResampleAction action = ResampleAction::Hold;
            const RE::NiPoint3 evaluated = resampler.evaluate(sourceDelta, action);
            if (frame > 1) {
                ok &= expectNear("coinciding clocks return the exact sample", evaluated.x, sourceX, 0.001f);
            }
        }
    }

    // One, two, and three physics substeps per source frame: evaluated motion
    // marches evenly, including bounded extrapolation past the newest sample.
    {
        Resampler resampler;
        const float speed = 90.0f;
        float sourceX = 0.0f;
        std::uint64_t sequence = 0;
        float previousEvaluatedX = 0.0f;
        bool warm = false;

        const int substepCounts[] = { 1, 2, 3, 2, 1, 3, 1, 2 };
        for (int frame = 0; frame < 400; ++frame) {
            const int substeps = substepCounts[frame % 8];
            const float sourceDelta = 0.011f * static_cast<float>(substeps);
            sourceX += speed * sourceDelta;
            resampler.advanceSource(RE::NiPoint3{ sourceX, 0.0f, 0.0f }, identity, sourceDelta, ++sequence);
            for (int substep = 0; substep < substeps; ++substep) {
                ResampleAction action = ResampleAction::Hold;
                const RE::NiPoint3 evaluated = resampler.evaluate(0.011f, action);
                if (warm && frame > 1) {
                    const float velocity = (evaluated.x - previousEvaluatedX) / 0.011f;
                    ok &= expectNear("multi-substep velocity stays on the source clock", velocity, speed, 1.0f);
                    ok &= expectTrue("multi-substep never rebases", action != ResampleAction::Rebase);
                }
                previousEvaluatedX = evaluated.x;
                warm = true;
            }
        }
        ok &= expectTrue("mixed substep cadence keeps a single initialization rebase", resampler.rebaseCount == 1);
    }

    // Duplicate physics flushes of one pending target must not advance the
    // source timeline twice.
    {
        Resampler resampler;
        std::uint64_t sequence = 0;
        resampler.advanceSource(RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        resampler.advanceSource(RE::NiPoint3{ 2.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        const float phaseBefore = resampler.phaseSeconds;
        const float currentBefore = resampler.currentTranslation.x;
        resampler.advanceSource(RE::NiPoint3{ 99.0f, 0.0f, 0.0f }, identity, 0.011f, sequence);
        ok &= expectTrue("duplicate sequence is counted", resampler.duplicateSourceCount == 1);
        ok &= expectNear("duplicate sequence leaves the phase alone", resampler.phaseSeconds, phaseBefore, 1e-9f);
        ok &= expectNear("duplicate sequence leaves the segment alone", resampler.currentTranslation.x, currentBefore, 1e-9f);
    }

    // Stationary target: evaluated position holds exactly with zero velocity.
    {
        Resampler resampler;
        std::uint64_t sequence = 0;
        const RE::NiPoint3 target{ 5.0f, -3.0f, 8.0f };
        for (int frame = 0; frame < 50; ++frame) {
            resampler.advanceSource(target, identity, 0.011f, ++sequence);
            ResampleAction action = ResampleAction::Hold;
            const RE::NiPoint3 evaluated = resampler.evaluate(0.011f, action);
            ok &= expectNear("stationary x holds", evaluated.x, target.x, 1e-4f);
            ok &= expectNear("stationary y holds", evaluated.y, target.y, 1e-4f);
            ok &= expectNear("stationary z holds", evaluated.z, target.z, 1e-4f);
        }
    }

    // Acceleration, deceleration, and reversal below the discontinuity gates
    // stay on the continuous path with no rebase.
    {
        Resampler resampler;
        MillisecondQuantizer quantizer;
        const float sourceDelta = 0.0111f;
        float sourceX = 0.0f;
        std::uint64_t sequence = 0;
        for (int frame = 0; frame < 900; ++frame) {
            const float speed = 300.0f * std::sin(static_cast<float>(frame) * 0.02f);
            sourceX += speed * sourceDelta;
            resampler.advanceSource(RE::NiPoint3{ sourceX, 0.0f, 0.0f }, identity, sourceDelta, ++sequence);
            ResampleAction action = ResampleAction::Hold;
            const RE::NiPoint3 evaluated = resampler.evaluate(quantizer.quantize(sourceDelta), action);
            ok &= expectTrue("reversal profile output stays finite", std::isfinite(evaluated.x));
        }
        ok &= expectTrue("reversal profile never rebases", resampler.rebaseCount == 1);
    }

    // Invalid, zero, and non-finite source deltas rebase to the exact target.
    {
        Resampler resampler;
        std::uint64_t sequence = 0;
        resampler.advanceSource(RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        resampler.advanceSource(RE::NiPoint3{ 2.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);

        resampler.advanceSource(RE::NiPoint3{ 3.0f, 0.0f, 0.0f }, identity, 0.0f, ++sequence);
        ok &= expectTrue("zero source delta rebases", resampler.rebaseCount == 2);
        ResampleAction action = ResampleAction::Hold;
        RE::NiPoint3 evaluated = resampler.evaluate(0.011f, action);
        ok &= expectNear("zero source delta holds the exact target", evaluated.x, 3.0f, 1e-4f);

        const float nan = std::nanf("");
        resampler.advanceSource(RE::NiPoint3{ 4.0f, 0.0f, 0.0f }, identity, nan, ++sequence);
        ok &= expectTrue("non-finite source delta rebases", resampler.rebaseCount == 3);
        evaluated = resampler.evaluate(0.011f, action);
        ok &= expectNear("non-finite source delta holds the exact target", evaluated.x, 4.0f, 1e-4f);

        resampler.advanceSource(RE::NiPoint3{ 5.0f, 0.0f, 0.0f }, identity, 0.2f, ++sequence);
        ok &= expectTrue("hitch-length source delta rebases", resampler.rebaseCount == 4);

        resampler.advanceSource(RE::NiPoint3{ nan, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        ok &= expectTrue("non-finite target is refused", resampler.invalidSourceCount == 1);
        evaluated = resampler.evaluate(0.011f, action);
        ok &= expectTrue("non-finite target never reaches the output", std::isfinite(evaluated.x));
    }

    // Invalid physics deltas neither corrupt the phase nor the output.
    {
        Resampler resampler;
        std::uint64_t sequence = 0;
        resampler.advanceSource(RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        resampler.advanceSource(RE::NiPoint3{ 2.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        ResampleAction action = ResampleAction::Hold;
        const RE::NiPoint3 reference = resampler.evaluate(0.011f, action);
        const float phaseBefore = resampler.phaseSeconds;
        const RE::NiPoint3 nanEvaluated = resampler.evaluate(std::nanf(""), action);
        ok &= expectTrue("non-finite physics delta output stays finite", std::isfinite(nanEvaluated.x));
        ok &= expectNear("non-finite physics delta does not advance the clock", resampler.phaseSeconds, phaseBefore, 1e-9f);
        ok &= expectNear("non-finite physics delta repeats the reference", nanEvaluated.x, reference.x, 1e-4f);
    }

    // Stale source: extrapolation is bounded to one source interval, then the
    // resampler rebases and holds the exact last sample.
    {
        Resampler resampler;
        std::uint64_t sequence = 0;
        const float speed = 200.0f;
        const float sourceDelta = 0.011f;
        float sourceX = 0.0f;
        for (int frame = 0; frame < 10; ++frame) {
            sourceX += speed * sourceDelta;
            resampler.advanceSource(RE::NiPoint3{ sourceX, 0.0f, 0.0f }, identity, sourceDelta, ++sequence);
            ResampleAction action = ResampleAction::Hold;
            resampler.evaluate(sourceDelta, action);
        }

        const float lastSample = sourceX;
        const float extrapolationCap = lastSample + speed * sourceDelta * 1.0f + 0.01f;
        bool sawRebase = false;
        float maxX = 0.0f;
        float heldX = 0.0f;
        for (int starvedStep = 0; starvedStep < 12; ++starvedStep) {
            ResampleAction action = ResampleAction::Hold;
            const RE::NiPoint3 evaluated = resampler.evaluate(sourceDelta, action);
            maxX = (std::max)(maxX, evaluated.x);
            sawRebase = sawRebase || action == ResampleAction::Rebase;
            heldX = evaluated.x;
        }
        ok &= expectTrue("starved source triggers a rebase", sawRebase);
        ok &= expectTrue("starved source never extrapolates past one interval", maxX <= extrapolationCap);
        ok &= expectNear("starved source holds the exact last sample", heldX, lastSample, 0.01f);
    }

    // Translation and rotation discontinuities rebase to the exact new target
    // instead of becoming extrapolated motion.
    {
        Resampler resampler;
        std::uint64_t sequence = 0;
        resampler.advanceSource(RE::NiPoint3{ 0.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        resampler.advanceSource(RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);

        resampler.advanceSource(RE::NiPoint3{ 51.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        ok &= expectTrue("translation jump rebases", resampler.rebaseCount == 2);
        ResampleAction action = ResampleAction::Hold;
        RE::NiPoint3 evaluated = resampler.evaluate(0.011f, action);
        ok &= expectNear("translation jump lands exactly on the new target", evaluated.x, 51.0f, 1e-4f);

        resampler.advanceSource(RE::NiPoint3{ 51.5f, 0.0f, 0.0f }, rotationAroundZ(kPi * 20.0f / 180.0f), 0.011f, ++sequence);
        ok &= expectTrue("rotation jump rebases", resampler.rebaseCount == 3);
        evaluated = resampler.evaluate(0.011f, action);
        ok &= expectNear("rotation jump lands exactly on the new target", evaluated.x, 51.5f, 1e-4f);

        resampler.advanceSource(RE::NiPoint3{ 51.6f, 0.0f, 0.0f }, rotationAroundZ(kPi * 22.0f / 180.0f), 0.011f, ++sequence);
        ok &= expectTrue("small rotation step after a jump does not rebase", resampler.rebaseCount == 3);
    }

    // Proxy rebuild / world change resets restore the uninitialized state.
    {
        Resampler resampler;
        std::uint64_t sequence = 0;
        resampler.advanceSource(RE::NiPoint3{ 7.0f, 0.0f, 0.0f }, identity, 0.011f, ++sequence);
        resampler.reset();
        ok &= expectTrue("reset drops initialization", !resampler.initialized);
        ok &= expectTrue("reset clears the sequence", resampler.lastSourceSequence == 0);
        ResampleAction action = ResampleAction::Interpolate;
        const RE::NiPoint3 evaluated = resampler.evaluate(0.011f, action);
        ok &= expectTrue("uninitialized evaluate holds", action == ResampleAction::Hold);
        ok &= expectNear("uninitialized evaluate returns the reset origin", evaluated.x, 0.0f, 1e-6f);
    }

    // Long mixed-cadence run: no accumulated drift, bounded phase, finite output.
    {
        Resampler resampler;
        MillisecondQuantizer quantizer;
        std::uint64_t sequence = 0;
        float sourceX = 0.0f;
        float maxTrackingError = 0.0f;
        for (int frame = 0; frame < 20000; ++frame) {
            const float sourceDelta = 0.010f + 0.003f * ((frame * 7919) % 100) / 100.0f;
            const float speed = 150.0f + 80.0f * std::sin(static_cast<float>(frame) * 0.01f);
            sourceX += speed * sourceDelta;
            resampler.advanceSource(RE::NiPoint3{ sourceX, 0.0f, 0.0f }, identity, sourceDelta, ++sequence);
            ResampleAction action = ResampleAction::Hold;
            const RE::NiPoint3 evaluated = resampler.evaluate(quantizer.quantize(sourceDelta), action);
            maxTrackingError = (std::max)(maxTrackingError, std::fabs(evaluated.x - sourceX));
        }
        ok &= expectTrue("long run phase stays bounded", std::fabs(resampler.phaseSeconds) < 0.05f);
        ok &= expectTrue("long run tracks the source trajectory", maxTrackingError < 10.0f);
        ok &= expectTrue("long run output stays finite", std::isfinite(resampler.currentTranslation.x));
    }

    return ok ? 0 : 1;
}
