#include "physics-interaction/grab/HeldScenePresentationPolicy.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
    bool expect(const char* label, bool condition)
    {
        if (condition) {
            return true;
        }
        std::printf("%s\n", label);
        return false;
    }

    bool near(float lhs, float rhs, float tolerance = 0.0001f)
    {
        return std::fabs(lhs - rhs) <= tolerance;
    }

    void makeIdentity(float* transform)
    {
        for (std::size_t index = 0;
             index < rock::held_scene_presentation_policy::kPredictionFloatCount;
             ++index) {
            transform[index] = 0.0f;
        }
        transform[0] = 1.0f;
        transform[5] = 1.0f;
        transform[10] = 1.0f;
    }
}

int main()
{
    using namespace rock::held_scene_presentation_policy;

    bool ok = true;

    const auto normalTiming = evaluateTiming(true, 0.011f, 0.002f);
    ok &= expect(
        "the main scene writer must use raw frame time plus native remainder",
        normalTiming.apply &&
            normalTiming.reason == RejectReason::None &&
            near(normalTiming.rawFrameSeconds, 0.011f) &&
            near(normalTiming.nativeRemainderSeconds, 0.002f) &&
            near(normalTiming.predictionSeconds, 0.013f));
    ok &= expect(
        "the proxy and all unknown callers must retain the native transform",
        evaluateTiming(false, 0.011f, 0.0f).reason ==
            RejectReason::UnsupportedCallsite);
    ok &= expect(
        "a zero raw frame must fail closed",
        evaluateTiming(true, 0.0f, 0.0f).reason ==
            RejectReason::InvalidRawFrame);
    ok &= expect(
        "an excessive raw frame must fail closed",
        evaluateTiming(true, 0.1f, 0.0f).reason ==
            RejectReason::InvalidRawFrame);
    ok &= expect(
        "a non-finite raw frame must fail closed",
        evaluateTiming(
            true,
            (std::numeric_limits<float>::quiet_NaN)(),
            0.0f).reason == RejectReason::InvalidRawFrame);
    ok &= expect(
        "a small negative native remainder must retain the engine time sum",
        evaluateTiming(true, 0.011f, -0.001f).apply &&
            near(
                evaluateTiming(true, 0.011f, -0.001f).predictionSeconds,
                0.010f));
    ok &= expect(
        "an excessive negative native remainder must fail closed",
        evaluateTiming(true, 0.011f, -0.1f).reason ==
            RejectReason::InvalidRemainder);
    ok &= expect(
        "a non-positive total prediction must fail closed",
        evaluateTiming(true, 0.011f, -0.020f).reason ==
            RejectReason::ExcessivePredictionTime);
    ok &= expect(
        "an excessive total prediction must fail closed",
        evaluateTiming(true, 0.033f, 0.020f).reason ==
            RejectReason::ExcessivePredictionTime);

    float original[kPredictionFloatCount]{};
    float predicted[kPredictionFloatCount]{};
    float corrected[kPredictionFloatCount]{};
    makeIdentity(original);
    makeIdentity(predicted);
    original[3] = 0.25f;
    original[7] = 0.50f;
    original[11] = 0.75f;
    original[12] = 69.0f;
    original[13] = 139.0f;
    original[14] = 209.0f;
    predicted[12] = 1.0f;
    predicted[13] = 2.0f;
    predicted[14] = 3.0f;

    const auto normalTransform = buildWriterTransform(
        original,
        predicted,
        70.0f,
        corrected);
    ok &= expect(
        "a bounded native prediction must replace the scene-writer pose",
        normalTransform.apply &&
            near(normalTransform.translationDeltaGameUnits, std::sqrt(3.0f)) &&
            near(normalTransform.rotationDeltaDegrees, 0.0f) &&
            near(corrected[12], 70.0f) &&
            near(corrected[13], 140.0f) &&
            near(corrected[14], 210.0f));
    ok &= expect(
        "writer padding must remain native",
        near(corrected[3], original[3]) &&
            near(corrected[7], original[7]) &&
            near(corrected[11], original[11]));

    float nonFinitePrediction[kPredictionFloatCount]{};
    makeIdentity(nonFinitePrediction);
    nonFinitePrediction[12] =
        (std::numeric_limits<float>::infinity)();
    ok &= expect(
        "a non-finite native prediction must fail closed",
        buildWriterTransform(
            original,
            nonFinitePrediction,
            70.0f,
            corrected).reason == RejectReason::InvalidPredictedTransform);

    float distantPrediction[kPredictionFloatCount]{};
    makeIdentity(distantPrediction);
    distantPrediction[12] = 2.0f;
    distantPrediction[13] = 2.0f;
    distantPrediction[14] = 3.0f;
    ok &= expect(
        "an excessive translation correction must fail closed",
        buildWriterTransform(
            original,
            distantPrediction,
            70.0f,
            corrected).reason == RejectReason::ExcessiveTranslationDelta);

    float rotatedPrediction[kPredictionFloatCount]{};
    makeIdentity(rotatedPrediction);
    rotatedPrediction[0] = 0.0f;
    rotatedPrediction[1] = -1.0f;
    rotatedPrediction[4] = 1.0f;
    rotatedPrediction[5] = 0.0f;
    rotatedPrediction[12] = 1.0f;
    rotatedPrediction[13] = 2.0f;
    rotatedPrediction[14] = 3.0f;
    ok &= expect(
        "an excessive rotation correction must fail closed",
        buildWriterTransform(
            original,
            rotatedPrediction,
            70.0f,
            corrected).reason == RejectReason::ExcessiveRotationDelta);

    ok &= expect(
        "an invalid Havok scale must fail closed",
        buildWriterTransform(
            original,
            predicted,
            0.0f,
            corrected).reason == RejectReason::InvalidScale);

    return ok ? 0 : 1;
}
