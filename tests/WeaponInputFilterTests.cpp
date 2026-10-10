#include "physics-interaction/weapon/WeaponInputFilter.h"

#include <cassert>
#include <cstdio>
#include <limits>

using namespace rock;
namespace filter = weapon_input_filter;

static RE::NiTransform identity() { return transform_math::makeIdentityTransform<RE::NiTransform>(); }
static bool near(float a, float b, float epsilon = 0.0002f) { return std::abs(a - b) < epsilon; }
static RE::NiTransform rotated(float radians)
{
    auto result = identity();
    result.rotate.entry[0][0] = std::cos(radians);
    result.rotate.entry[0][1] = -std::sin(radians);
    result.rotate.entry[1][0] = std::sin(radians);
    result.rotate.entry[1][1] = std::cos(radians);
    return result;
}

int main()
{
    const auto room = identity();
    constexpr filter::Tuning config{ 0.045f, 0.022f };
    RE::NiTransform output{};
    for (const int rate : {72, 90, 120}) {
        filter::State state{};
        const float dt = 1.0f / rate;
        assert(filter::update(state, identity(), room, {}, 1, dt, false, config, output));
        auto target = rotated(0.8f);
        target.translate.x = 10.0f;
        float previous = 0.0f;
        for (int frame = 1; frame <= rate; ++frame) {
            assert(filter::update(state, target, room, {}, frame + 1, dt, false, config, output));
            const float expected = 10.0f * (1.0f - std::exp(-(frame * dt) / config.translationSeconds));
            assert(near(output.translate.x, expected));
            assert(output.translate.x >= previous && output.translate.x <= 10.0001f);
            previous = output.translate.x;
            const float angle = 0.8f * (1.0f - std::exp(-(frame * dt) / config.rotationSeconds));
            assert(near(output.rotate.entry[0][0], std::cos(angle), 0.001f));
        }
    }

    // Uneven frame durations must follow elapsed time, not sample count.
    filter::State variable{};
    assert(filter::update(variable, identity(), room, {}, 1, 0.01f, false, config, output));
    auto translated = identity();
    translated.translate.x = 7.0f;
    float elapsed = 0.0f;
    std::uint64_t sequence = 1;
    for (float dt : {0.004f, 0.021f, 0.008f, 0.0f, 0.016f, 0.011f}) {
        elapsed += dt;
        assert(filter::update(variable, translated, room, {}, ++sequence, dt, false, config, output));
        assert(near(output.translate.x, 7.0f * (1.0f - std::exp(-elapsed / config.translationSeconds))));
    }

    // Filtering around an offset grip must keep that grip fixed under rotation.
    filter::State state{};
    const RE::NiPoint3 pivot{ 0.0f, 5.0f, 0.0f };
    assert(filter::update(state, identity(), room, pivot, 1, 0.01f, false, config, output));
    auto target = rotated(1.0f);
    target.translate = pivot - transform_math::localPointToWorld(target, pivot);
    assert(filter::update(state, target, room, pivot, 2, 0.01f, false, config, output));
    auto actualPivot = transform_math::localPointToWorld(output, pivot);
    assert((actualPivot - pivot).Length() < 0.001f);
    const auto once = output;
    assert(filter::update(state, identity(), room, pivot, 2, 0.01f, false, config, output));
    assert((output.translate - once.translate).Length() < 0.00001f);

    // A room-space turn and translation transport the current output exactly,
    // even when no measurable controller timestep has elapsed.
    auto movedRoom = rotated(1.3f);
    movedRoom.translate = {100.0f, 30.0f, 0.0f};
    auto movedTarget = transform_math::composeTransforms(movedRoom, target);
    assert(filter::update(state, movedTarget, movedRoom, pivot, 3, 0.0f, false, config, output));
    const auto transported = transform_math::composeTransforms(movedRoom, once);
    assert((output.translate - transported.translate).Length() < 0.001f);

    // Discontinuity, missed frames and invalid tracking cannot retain stale history.
    assert(filter::update(state, movedTarget, movedRoom, pivot, 4, 0.1f, true, config, output));
    assert((output.translate - movedTarget.translate).Length() < 0.001f);
    assert(filter::update(state, identity(), room, pivot, 9, 0.01f, false, config, output));
    assert(output.translate.Length() < 0.001f);
    auto invalid = identity();
    invalid.translate.x = std::numeric_limits<float>::quiet_NaN();
    assert(!filter::update(state, invalid, room, pivot, 10, 0.01f, false, config, output));
    assert(!state.valid);
    assert(filter::update(state, target, room, pivot, 11, 0.01f, false, {}, output));
    assert((output.translate - target.translate).Length() < 0.00001f);
    assert(!state.valid);

    // Measure shake after settling using the same input at each sample rate.
    for (const int rate : {72, 90, 120}) {
        state = {};
        double inputEnergy = 0.0, outputEnergy = 0.0;
        for (int frame = 0; frame < rate * 4; ++frame) {
            const float angle = 0.1f * std::sin(2.0f * 3.14159265f * 8.0f * frame / rate);
            assert(filter::update(state, rotated(angle), room, {}, frame + 1, 1.0f / rate, false, config, output));
            if (frame >= rate) {
                const float actual = std::atan2(output.rotate.entry[1][0], output.rotate.entry[0][0]);
                inputEnergy += angle * angle;
                outputEnergy += actual * actual;
            }
        }
        const double ratio = std::sqrt(outputEnergy / inputEnergy);
        std::printf("%d Hz: 8 Hz shake amplitude ratio %.4f\n", rate, ratio);
        assert(ratio < 0.5);
    }
    // Shortest rotation across the wrap must stay near 180 degrees, not zero.
    state = {};
    assert(filter::update(state, rotated(3.12f), room, {}, 1, 0.01f, false, config, output));
    assert(filter::update(state, rotated(-3.12f), room, {}, 2, 0.01f, false, config, output));
    assert(output.rotate.entry[0][0] < -0.99f);
    std::puts("Weapon input filtering checks passed");
}
