#include "physics-interaction/visual/DampenedDriverPredictionPolicy.h"

#include <cmath>
#include <cstdio>

namespace
{
    using namespace rock::dampened_driver_prediction_policy;

    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        if (std::fabs(actual - expected) <= epsilon) {
            return true;
        }
        std::printf("%s expected %.5f got %.5f\n", label, expected, actual);
        return false;
    }

    RE::NiTransform identity()
    {
        return rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    }

    RE::NiTransform translated(float x, float y, float z)
    {
        RE::NiTransform t = identity();
        t.translate = RE::NiPoint3{ x, y, z };
        return t;
    }

    RE::NiTransform yawed(float degrees)
    {
        const float radians = degrees * 0.017453292519943295f;
        const float c = std::cos(radians);
        const float s = std::sin(radians);
        RE::NiTransform t = identity();
        t.rotate.entry[0][0] = c;
        t.rotate.entry[0][1] = -s;
        t.rotate.entry[1][0] = s;
        t.rotate.entry[1][1] = c;
        return t;
    }
}

int main()
{
    bool ok = true;
    const DampenFactors frikDefault{ .enabled = true, .translation = 0.6f, .rotation = 0.6f };

    // Translation: FRIK keeps 60% of the previous value, after removing the camera step.
    {
        const RE::NiTransform predicted = predictDampened(translated(10.0f, 0.0f, 0.0f), translated(0.0f, 0.0f, 0.0f), RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, frikDefault);
        // raw - 0.6 * (raw - previous - camera) = 10 - 0.6 * 9
        ok &= expectNear("translation dampened", predicted.translate.x, 4.6f, 0.0001f);
        ok &= expectNear("translation y untouched", predicted.translate.y, 0.0f, 0.0001f);
        const RE::NiTransform riding = predictDampened(translated(10.0f, 0.0f, 0.0f), translated(0.0f, 0.0f, 0.0f), RE::NiPoint3{ 10.0f, 0.0f, 0.0f }, frikDefault);
        ok &= expectNear("player movement is not dampened", riding.translate.x, 10.0f, 0.0001f);
    }

    // Rotation: slerp from the previous dampened world toward the raw world by 1 - factor.
    {
        const RE::NiTransform predicted = predictDampened(yawed(90.0f), identity(), RE::NiPoint3{}, frikDefault);
        ok &= expectNear("rotation dampened to 40% of the step", rock::hand_visual_lerp_math::rotationDistanceDegrees(identity(), predicted), 36.0f, 0.05f);
        ok &= expectNear("rotation stays short of the raw", rock::hand_visual_lerp_math::rotationDistanceDegrees(predicted, yawed(90.0f)), 54.0f, 0.05f);
        ok &= expectNear("prediction is a rotation", static_cast<float>(rock::transform_math::storedRotationOrthonormalityError(predicted.rotate)), 0.0f, 0.001f);
        const DampenFactors translationOnly{ .enabled = true, .translation = 0.6f, .rotation = 0.0f };
        ok &= expectNear("zero rotation factor keeps the raw rotation", rock::hand_visual_lerp_math::rotationDistanceDegrees(predictDampened(yawed(90.0f), identity(), RE::NiPoint3{}, translationOnly), yawed(90.0f)), 0.0f, 0.001f);
    }

    // Disabled factors return the raw world; configuration selection follows the vanilla scope menu.
    {
        const DampenFactors off{};
        const RE::NiTransform raw = translated(10.0f, 2.0f, 3.0f);
        const RE::NiTransform predicted = predictDampened(raw, translated(0.0f, 0.0f, 0.0f), RE::NiPoint3{}, off);
        ok &= expectNear("disabled keeps raw x", predicted.translate.x, 10.0f, 0.0001f);
        ok &= expectNear("disabled keeps raw y", predicted.translate.y, 2.0f, 0.0001f);

        FrikDampenConfig config{};
        config.normal = frikDefault;
        config.vanillaScope = DampenFactors{ .enabled = true, .translation = 0.2f, .rotation = 0.2f };
        ok &= expectTrue("invalid config selects nothing", !selectFactors(config, false).enabled);
        config.valid = true;
        ok &= expectNear("normal factors outside the scope", selectFactors(config, false).translation, 0.6f, 0.0001f);
        ok &= expectNear("scope factors in the scope", selectFactors(config, true).translation, 0.2f, 0.0001f);
        ok &= expectTrue("factor bounds", isUsableFactor(0.0f) && isUsableFactor(0.95f) && !isUsableFactor(1.0f) && !isUsableFactor(-0.1f) && !isUsableFactor(std::nanf("")));
    }

    if (!ok) {
        std::printf("DampenedDriverPredictionPolicyTests FAILED\n");
        return 1;
    }
    std::printf("DampenedDriverPredictionPolicyTests passed\n");
    return 0;
}
