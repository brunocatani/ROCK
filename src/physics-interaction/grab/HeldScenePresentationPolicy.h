#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rock::held_scene_presentation_policy
{
    inline constexpr float kMinRawFrameSeconds = 0.001f;
    inline constexpr float kMaxRawFrameSeconds = 1.0f / 30.0f;
    inline constexpr float kMaxNativeRemainderSeconds = 0.050f;
    inline constexpr float kMinPredictionSeconds = 0.001f;
    inline constexpr float kMaxPredictionSeconds = 0.050f;
    inline constexpr float kMaxTranslationDeltaGameUnits = 8.0f;
    inline constexpr float kMaxRotationDeltaDegrees = 45.0f;
    inline constexpr std::size_t kWriterInputFloatCount = 15;
    inline constexpr std::size_t kPredictionFloatCount = 16;

    inline constexpr bool shouldPublishSolvedPose(
        std::uint32_t substepIndex,
        std::uint32_t substepCount) noexcept
    {
        return substepCount > 0 &&
               substepIndex < substepCount &&
               substepIndex + 1u == substepCount;
    }

    enum class RejectReason
    {
        None,
        UnsupportedCallsite,
        InvalidRawFrame,
        InvalidRemainder,
        ExcessivePredictionTime,
        InvalidScale,
        InvalidInputTransform,
        InvalidPredictedTransform,
        ExcessiveTranslationDelta,
        ExcessiveRotationDelta,
    };

    inline const char* rejectReasonName(RejectReason reason)
    {
        switch (reason) {
        case RejectReason::None:
            return "none";
        case RejectReason::UnsupportedCallsite:
            return "unsupportedCallsite";
        case RejectReason::InvalidRawFrame:
            return "invalidRawFrame";
        case RejectReason::InvalidRemainder:
            return "invalidRemainder";
        case RejectReason::ExcessivePredictionTime:
            return "excessivePredictionTime";
        case RejectReason::InvalidScale:
            return "invalidScale";
        case RejectReason::InvalidInputTransform:
            return "invalidInputTransform";
        case RejectReason::InvalidPredictedTransform:
            return "invalidPredictedTransform";
        case RejectReason::ExcessiveTranslationDelta:
            return "excessiveTranslationDelta";
        case RejectReason::ExcessiveRotationDelta:
            return "excessiveRotationDelta";
        }
        return "unknown";
    }

    struct TimingDecision
    {
        bool apply = false;
        float rawFrameSeconds = 0.0f;
        float nativeRemainderSeconds = 0.0f;
        float predictionSeconds = 0.0f;
        RejectReason reason = RejectReason::InvalidRawFrame;
    };

    inline TimingDecision evaluateTiming(
        bool mainWriterCallsite,
        float rawFrameSeconds,
        float nativeRemainderSeconds)
    {
        TimingDecision decision{
            .rawFrameSeconds = rawFrameSeconds,
            .nativeRemainderSeconds = nativeRemainderSeconds,
            .predictionSeconds = rawFrameSeconds + nativeRemainderSeconds,
        };
        if (!mainWriterCallsite) {
            decision.reason = RejectReason::UnsupportedCallsite;
            return decision;
        }
        if (!std::isfinite(rawFrameSeconds) ||
            rawFrameSeconds < kMinRawFrameSeconds ||
            rawFrameSeconds > kMaxRawFrameSeconds) {
            decision.reason = RejectReason::InvalidRawFrame;
            return decision;
        }
        if (!std::isfinite(nativeRemainderSeconds) ||
            std::fabs(nativeRemainderSeconds) >
                kMaxNativeRemainderSeconds) {
            decision.reason = RejectReason::InvalidRemainder;
            return decision;
        }

        const float predictionSeconds =
            rawFrameSeconds + nativeRemainderSeconds;
        if (!std::isfinite(predictionSeconds) ||
            predictionSeconds < kMinPredictionSeconds ||
            predictionSeconds > kMaxPredictionSeconds) {
            decision.reason = RejectReason::ExcessivePredictionTime;
            return decision;
        }

        decision.apply = true;
        decision.predictionSeconds = predictionSeconds;
        decision.reason = RejectReason::None;
        return decision;
    }

    struct TransformDecision
    {
        bool apply = false;
        float translationDeltaGameUnits = 0.0f;
        float rotationDeltaDegrees = 0.0f;
        RejectReason reason = RejectReason::InvalidInputTransform;
    };

    inline bool finiteWriterInput(const float* transform)
    {
        if (!transform) {
            return false;
        }
        for (std::size_t index = 0; index < kWriterInputFloatCount; ++index) {
            if (!std::isfinite(transform[index])) {
                return false;
            }
        }
        return true;
    }

    inline bool finiteNativePrediction(const float* transform)
    {
        if (!transform) {
            return false;
        }
        constexpr std::size_t usedIndices[]{
            0, 1, 2,
            4, 5, 6,
            8, 9, 10,
            12, 13, 14,
        };
        for (const std::size_t index : usedIndices) {
            if (!std::isfinite(transform[index])) {
                return false;
            }
        }
        return true;
    }

    inline float rotationDeltaDegrees(
        const float* first,
        const float* second)
    {
        constexpr std::size_t rotationIndices[]{
            0, 1, 2,
            4, 5, 6,
            8, 9, 10,
        };
        float frobeniusDot = 0.0f;
        for (const std::size_t index : rotationIndices) {
            frobeniusDot += first[index] * second[index];
        }
        const float cosine = std::clamp(
            (frobeniusDot - 1.0f) * 0.5f,
            -1.0f,
            1.0f);
        return std::acos(cosine) * 57.2957795131f;
    }

    inline TransformDecision buildWriterTransform(
        const float* originalWriterInput,
        const float* nativePredictedTransformHavok,
        float havokToGameScale,
        float* correctedWriterInput)
    {
        if (!std::isfinite(havokToGameScale) ||
            havokToGameScale <= 0.000001f ||
            havokToGameScale >= 10000.0f) {
            return TransformDecision{ .reason = RejectReason::InvalidScale };
        }
        if (!finiteWriterInput(originalWriterInput) || !correctedWriterInput) {
            return TransformDecision{ .reason = RejectReason::InvalidInputTransform };
        }
        if (!finiteNativePrediction(nativePredictedTransformHavok)) {
            return TransformDecision{ .reason = RejectReason::InvalidPredictedTransform };
        }

        for (std::size_t index = 0; index < kWriterInputFloatCount; ++index) {
            correctedWriterInput[index] = originalWriterInput[index];
        }
        correctedWriterInput[15] = 0.0f;

        constexpr std::size_t rotationIndices[]{
            0, 1, 2,
            4, 5, 6,
            8, 9, 10,
        };
        for (const std::size_t index : rotationIndices) {
            correctedWriterInput[index] =
                nativePredictedTransformHavok[index];
        }
        correctedWriterInput[12] =
            nativePredictedTransformHavok[12] * havokToGameScale;
        correctedWriterInput[13] =
            nativePredictedTransformHavok[13] * havokToGameScale;
        correctedWriterInput[14] =
            nativePredictedTransformHavok[14] * havokToGameScale;

        const float deltaX =
            correctedWriterInput[12] - originalWriterInput[12];
        const float deltaY =
            correctedWriterInput[13] - originalWriterInput[13];
        const float deltaZ =
            correctedWriterInput[14] - originalWriterInput[14];
        const float translationDelta = std::sqrt(
            deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
        if (!std::isfinite(translationDelta) ||
            translationDelta > kMaxTranslationDeltaGameUnits) {
            return TransformDecision{
                .translationDeltaGameUnits = translationDelta,
                .reason = RejectReason::ExcessiveTranslationDelta,
            };
        }

        const float rotationDelta = rotationDeltaDegrees(
            originalWriterInput,
            correctedWriterInput);
        if (!std::isfinite(rotationDelta) ||
            rotationDelta > kMaxRotationDeltaDegrees) {
            return TransformDecision{
                .translationDeltaGameUnits = translationDelta,
                .rotationDeltaDegrees = rotationDelta,
                .reason = RejectReason::ExcessiveRotationDelta,
            };
        }

        return TransformDecision{
            .apply = true,
            .translationDeltaGameUnits = translationDelta,
            .rotationDeltaDegrees = rotationDelta,
            .reason = RejectReason::None,
        };
    }
}
