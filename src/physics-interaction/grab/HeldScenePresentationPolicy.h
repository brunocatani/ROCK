#pragma once

#include "physics-interaction/TransformMath.h"

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
    inline constexpr float kMaxTargetTranslationStepGameUnits = 25.0f;
    inline constexpr float kMaxTargetRotationStepDegrees = 75.0f;
    inline constexpr float kMaxTargetResidualTranslationGameUnits = 100.0f;
    inline constexpr float kMaxTargetTransportAdvanceGameUnits = 50.0f;
    inline constexpr std::size_t kWriterInputFloatCount = 15;
    inline constexpr std::size_t kPredictionFloatCount = 16;

    inline bool preferEarlierTrace(std::uint64_t candidate, std::uint64_t current) noexcept
    {
        return candidate != 0 && (current == 0 || candidate < current);
    }

    enum class TargetTransportRejectReason
    {
        None,
        InvalidTransform,
        ExcessiveTargetTranslationStep,
        ExcessiveTargetRotationStep,
        ExcessivePhysicalResidual,
        ExcessiveTransportAdvance,
    };

    inline const char* targetTransportRejectReasonName(
        TargetTransportRejectReason reason) noexcept
    {
        switch (reason) {
        case TargetTransportRejectReason::None:
            return "none";
        case TargetTransportRejectReason::InvalidTransform:
            return "invalidTransform";
        case TargetTransportRejectReason::ExcessiveTargetTranslationStep:
            return "excessiveTargetTranslationStep";
        case TargetTransportRejectReason::ExcessiveTargetRotationStep:
            return "excessiveTargetRotationStep";
        case TargetTransportRejectReason::ExcessivePhysicalResidual:
            return "excessivePhysicalResidual";
        case TargetTransportRejectReason::ExcessiveTransportAdvance:
            return "excessiveTransportAdvance";
        }
        return "unknown";
    }

    template <class Transform>
    inline bool finiteTransform(const Transform& transform) noexcept
    {
        if (!std::isfinite(transform.translate.x) ||
            !std::isfinite(transform.translate.y) ||
            !std::isfinite(transform.translate.z) ||
            !std::isfinite(transform.scale) ||
            transform.scale <= 0.000001f ||
            transform.scale >= 10000.0f) {
            return false;
        }
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(
                        transform.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    template <class Point>
    inline float pointDistance(const Point& first, const Point& second) noexcept
    {
        const float deltaX = first.x - second.x;
        const float deltaY = first.y - second.y;
        const float deltaZ = first.z - second.z;
        return std::sqrt(
            deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
    }

    template <class Matrix>
    inline float matrixRotationDeltaDegrees(
        const Matrix& first,
        const Matrix& second) noexcept
    {
        float frobeniusDot = 0.0f;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                frobeniusDot += first.entry[row][column] *
                                second.entry[row][column];
            }
        }
        const float cosine = std::clamp(
            (frobeniusDot - 1.0f) * 0.5f,
            -1.0f,
            1.0f);
        return std::acos(cosine) * 57.2957795131f;
    }

    template <class Transform>
    struct TargetTransportDecision
    {
        bool apply = false;
        Transform presentedWorld{};
        float targetTranslationStepGameUnits = 0.0f;
        float targetRotationStepDegrees = 0.0f;
        float physicalResidualGameUnits = 0.0f;
        float transportAdvanceGameUnits = 0.0f;
        TargetTransportRejectReason reason =
            TargetTransportRejectReason::InvalidTransform;
    };

    template <class Transform>
    inline TargetTransportDecision<Transform> buildTargetTransport(
        const Transform& previousTargetWorld,
        const Transform& currentTargetWorld,
        const Transform& previousSolvedBodyWorld) noexcept
    {
        TargetTransportDecision<Transform> decision{};
        if (!finiteTransform(previousTargetWorld) ||
            !finiteTransform(currentTargetWorld) ||
            !finiteTransform(previousSolvedBodyWorld)) {
            return decision;
        }

        decision.targetTranslationStepGameUnits = pointDistance(
            previousTargetWorld.translate,
            currentTargetWorld.translate);
        if (!std::isfinite(decision.targetTranslationStepGameUnits) ||
            decision.targetTranslationStepGameUnits >
                kMaxTargetTranslationStepGameUnits) {
            decision.reason = TargetTransportRejectReason::
                ExcessiveTargetTranslationStep;
            return decision;
        }

        decision.targetRotationStepDegrees = matrixRotationDeltaDegrees(
            previousTargetWorld.rotate,
            currentTargetWorld.rotate);
        if (!std::isfinite(decision.targetRotationStepDegrees) ||
            decision.targetRotationStepDegrees >
                kMaxTargetRotationStepDegrees) {
            decision.reason =
                TargetTransportRejectReason::ExcessiveTargetRotationStep;
            return decision;
        }

        const Transform physicalResidual =
            transform_math::composeTransforms(
                transform_math::invertTransform(previousTargetWorld),
                previousSolvedBodyWorld);
        if (!finiteTransform(physicalResidual)) {
            return decision;
        }
        const decltype(physicalResidual.translate) residualOrigin{};
        decision.physicalResidualGameUnits = pointDistance(
            physicalResidual.translate,
            residualOrigin);
        if (!std::isfinite(decision.physicalResidualGameUnits) ||
            decision.physicalResidualGameUnits >
                kMaxTargetResidualTranslationGameUnits) {
            decision.reason =
                TargetTransportRejectReason::ExcessivePhysicalResidual;
            return decision;
        }

        decision.presentedWorld = transform_math::composeTransforms(
            currentTargetWorld,
            physicalResidual);
        if (!finiteTransform(decision.presentedWorld)) {
            return decision;
        }
        decision.transportAdvanceGameUnits = pointDistance(
            previousSolvedBodyWorld.translate,
            decision.presentedWorld.translate);
        const float transportRotationDegrees = matrixRotationDeltaDegrees(
            previousSolvedBodyWorld.rotate,
            decision.presentedWorld.rotate);
        if (!std::isfinite(decision.transportAdvanceGameUnits) ||
            !std::isfinite(transportRotationDegrees) ||
            decision.transportAdvanceGameUnits >
                kMaxTargetTransportAdvanceGameUnits ||
            transportRotationDegrees > kMaxTargetRotationStepDegrees) {
            decision.reason =
                TargetTransportRejectReason::ExcessiveTransportAdvance;
            return decision;
        }

        decision.apply = true;
        decision.reason = TargetTransportRejectReason::None;
        return decision;
    }

    // Every body keeps its own solved pose. Only the primary target's clock
    // advance is shared; freezing offsets here would erase articulation.
    template <class Transform>
    inline bool transportAssemblyBody(
        const Transform& solvedPrimary,
        const Transform& presentedPrimary,
        const Transform& solvedBody,
        Transform& presentedBody) noexcept
    {
        if (!finiteTransform(solvedPrimary) || !finiteTransform(presentedPrimary) ||
            !finiteTransform(solvedBody)) {
            return false;
        }
        const auto delta = transform_math::composeTransforms(
            presentedPrimary, transform_math::invertTransform(solvedPrimary));
        presentedBody = transform_math::composeTransforms(delta, solvedBody);
        presentedBody.scale = solvedBody.scale;
        return finiteTransform(presentedBody) &&
               pointDistance(solvedBody.translate, presentedBody.translate) <= kMaxTargetTransportAdvanceGameUnits &&
               matrixRotationDeltaDegrees(solvedBody.rotate, presentedBody.rotate) <= kMaxTargetRotationStepDegrees;
    }

    // Order independent owner subtrees before their descendants. Bound the walk
    // and reject cycles before the first scene write. Duplicate owners may only
    // describe the same pose; one scene node cannot represent two body frames.
    template <class Node, class Transform>
    struct ScenePose
    {
        Node* node = nullptr;
        Transform world{};
        std::size_t depth = 0;
        bool duplicate = false;
    };

    // Loose references can have mesh-only branches beside their collision owners.
    // Present the reference root first, then retain each body's independent
    // solved pose. The captured root/body relation never uses last frame's
    // presented node as a new physics input.
    template <class Node, class Transform>
    inline bool appendAssemblyRootPose(
        ScenePose<Node, Transform>* poses, std::size_t& count, std::size_t capacity,
        Node* root, const Transform& presentedBody, const Transform& bodyInRoot) noexcept
    {
        if (!root || count == 0 || count >= capacity || !finiteTransform(root->world) ||
            !finiteTransform(presentedBody) || !finiteTransform(bodyInRoot)) {
            return false;
        }
        bool rootIsBodyOwner = false;
        for (std::size_t index = 0; index < count; ++index) {
            auto* node = poses[index].node;
            std::size_t depth = 0;
            while (node && node != root && depth++ < 128) node = node->parent;
            if (node != root) return false;
            rootIsBodyOwner |= poses[index].node == root;
        }
        if (rootIsBodyOwner) return true;
        auto rootWorld = transform_math::composeTransforms(
            presentedBody, transform_math::invertTransform(bodyInRoot));
        rootWorld.scale = root->world.scale;
        if (!finiteTransform(rootWorld)) return false;
        poses[count++] = {root, rootWorld};
        return true;
    }

    template <class Node, class Transform>
    inline bool prepareScenePoses(ScenePose<Node, Transform>* poses, std::size_t count) noexcept
    {
        constexpr std::size_t kMaxParentDepth = 128;
        for (std::size_t index = 0; index < count; ++index) {
            auto& pose = poses[index];
            pose.depth = 0;
            pose.duplicate = false;
            if (!pose.node || !finiteTransform(pose.world)) {
                return false;
            }
            for (auto* parent = pose.node->parent; parent; parent = parent->parent) {
                if (++pose.depth > kMaxParentDepth || !finiteTransform(parent->world) ||
                    !finiteTransform(parent->local)) {
                    return false;
                }
            }
            for (std::size_t prior = 0; prior < index; ++prior) {
                if (poses[prior].node == pose.node) {
                    if (pointDistance(poses[prior].world.translate, pose.world.translate) > 0.001f ||
                        matrixRotationDeltaDegrees(poses[prior].world.rotate, pose.world.rotate) > 0.1f ||
                        std::fabs(poses[prior].world.scale - pose.world.scale) > 0.0001f) {
                        return false;
                    }
                    pose.duplicate = true;
                    break;
                }
            }
        }
        std::sort(poses, poses + count, [](const auto& first, const auto& second) {
            return first.depth < second.depth;
        });
        return true;
    }

    template <class Node, class Transform, class RefreshSubtree>
    inline void applyScenePoses(
        const ScenePose<Node, Transform>* poses, std::size_t count, RefreshSubtree refreshSubtree) noexcept
    {
        for (std::size_t index = 0; index < count; ++index) {
            const auto& pose = poses[index];
            if (pose.duplicate) {
                continue;
            }
            auto* node = pose.node;
            node->world = pose.world;
            node->local = node->parent ? transform_math::composeTransforms(
                transform_math::invertTransform(node->parent->world), pose.world) : pose.world;
            refreshSubtree(node);
        }
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
        float* correctedWriterInput,
        float maxTranslationDeltaGameUnits =
            kMaxTranslationDeltaGameUnits,
        float maxRotationDeltaDegrees = kMaxRotationDeltaDegrees)
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
            translationDelta > maxTranslationDeltaGameUnits) {
            return TransformDecision{
                .translationDeltaGameUnits = translationDelta,
                .reason = RejectReason::ExcessiveTranslationDelta,
            };
        }

        const float rotationDelta = rotationDeltaDegrees(
            originalWriterInput,
            correctedWriterInput);
        if (!std::isfinite(rotationDelta) ||
            rotationDelta > maxRotationDeltaDegrees) {
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
