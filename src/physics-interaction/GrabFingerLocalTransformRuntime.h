#pragma once

/*
 * ROCK publishes full-hand local transforms only after FRIK has generated the
 * authored local pose for the same 15 joint values. That keeps FRIK as the hand
 * table owner, while ROCK contributes a bounded mesh-contact aim correction
 * derived from the root flattened finger bones and the current object surface
 * probes. The live transform source intentionally matches generated hand
 * colliders; FRIK remains only the pose publication API here.
 */

#include "DirectSkeletonBoneReader.h"
#include "GrabFingerLocalTransformMath.h"
#include "GrabFingerPoseRuntime.h"
#include "HandVisualLerpMath.h"
#include "PhysicsLog.h"
#include "TransformMath.h"
#include "api/FRIKApi.h"

#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>

namespace frik::rock::grab_finger_local_transform_runtime
{
    struct Options
    {
        bool enabled = true;
        float smoothingSpeed = grab_finger_local_transform_math::kDefaultLocalTransformSmoothingSpeed;
        float maxCorrectionDegrees = grab_finger_local_transform_math::kDefaultMaxCorrectionDegrees;
        float surfaceAimStrength = grab_finger_local_transform_math::kDefaultSurfaceAimStrength;
        float thumbOppositionStrength = grab_finger_local_transform_math::kDefaultThumbOppositionStrength;
        float thumbAlternateCurveStrength = grab_finger_local_transform_math::kDefaultThumbAlternateCurveStrength;
    };

    struct State
    {
        std::array<RE::NiTransform, 15> currentTransforms{};
        std::uint16_t currentMask = 0;
        bool hasCurrentTransforms = false;
    };

    struct LiveFingerTransform
    {
        RE::NiTransform world{};
        RE::NiTransform parentWorld{};
        bool valid = false;
    };

    [[nodiscard]] inline Options sanitizeOptions(Options options)
    {
        options.smoothingSpeed = grab_finger_local_transform_math::sanitizeSmoothingSpeed(
            options.smoothingSpeed, grab_finger_local_transform_math::kDefaultLocalTransformSmoothingSpeed);
        options.maxCorrectionDegrees = grab_finger_local_transform_math::sanitizeMaxCorrectionDegrees(
            options.maxCorrectionDegrees, grab_finger_local_transform_math::kDefaultMaxCorrectionDegrees);
        options.surfaceAimStrength = grab_finger_local_transform_math::sanitizeUnitStrength(
            options.surfaceAimStrength, grab_finger_local_transform_math::kDefaultSurfaceAimStrength);
        options.thumbOppositionStrength = grab_finger_local_transform_math::sanitizeUnitStrength(
            options.thumbOppositionStrength, grab_finger_local_transform_math::kDefaultThumbOppositionStrength);
        options.thumbAlternateCurveStrength = grab_finger_local_transform_math::sanitizeUnitStrength(
            options.thumbAlternateCurveStrength, grab_finger_local_transform_math::kDefaultThumbAlternateCurveStrength);
        return options;
    }

    [[nodiscard]] inline bool isFiniteRotation(const RE::NiMatrix3& rotation)
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(rotation.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    [[nodiscard]] inline bool isFiniteTransform(const RE::NiTransform& transform)
    {
        return grab_finger_local_transform_math::sceneTransformHasUsableBasis(transform);
    }

    [[nodiscard]] inline float dot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    [[nodiscard]] inline RE::NiPoint3 cross(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3{
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x,
        };
    }

    [[nodiscard]] inline float lengthSquared(const RE::NiPoint3& value)
    {
        return dot(value, value);
    }

    [[nodiscard]] inline RE::NiPoint3 normalizeOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float valueLengthSquared = lengthSquared(value);
        if (std::isfinite(valueLengthSquared) && valueLengthSquared > 0.000001f) {
            const float invLength = 1.0f / std::sqrt(valueLengthSquared);
            return RE::NiPoint3{ value.x * invLength, value.y * invLength, value.z * invLength };
        }

        const float fallbackLengthSquared = lengthSquared(fallback);
        if (std::isfinite(fallbackLengthSquared) && fallbackLengthSquared > 0.000001f) {
            const float invLength = 1.0f / std::sqrt(fallbackLengthSquared);
            return RE::NiPoint3{ fallback.x * invLength, fallback.y * invLength, fallback.z * invLength };
        }

        return RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
    }

    [[nodiscard]] inline RE::NiPoint3 orthogonalAxis(const RE::NiPoint3& value)
    {
        const RE::NiPoint3 axis = std::abs(value.x) < 0.7f ? RE::NiPoint3{ 1.0f, 0.0f, 0.0f } : RE::NiPoint3{ 0.0f, 1.0f, 0.0f };
        return normalizeOrFallback(cross(value, axis), RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
    }

    [[nodiscard]] inline RE::NiMatrix3 storedRotationFromConventionalRows(const RE::NiPoint3 rows[3])
    {
        RE::NiMatrix3 result{};
        result.entry[0][0] = rows[0].x;
        result.entry[0][1] = rows[1].x;
        result.entry[0][2] = rows[2].x;
        result.entry[1][0] = rows[0].y;
        result.entry[1][1] = rows[1].y;
        result.entry[1][2] = rows[2].y;
        result.entry[2][0] = rows[0].z;
        result.entry[2][1] = rows[1].z;
        result.entry[2][2] = rows[2].z;
        return result;
    }

    [[nodiscard]] inline RE::NiMatrix3 axisAngleStored(const RE::NiPoint3& axisRaw, float angle)
    {
        const RE::NiPoint3 axis = normalizeOrFallback(axisRaw, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        const float x = axis.x;
        const float y = axis.y;
        const float z = axis.z;
        const float cosTheta = std::cos(angle);
        const float sinTheta = std::sin(angle);
        const float oneMinusCos = 1.0f - cosTheta;

        const RE::NiPoint3 conventionalRows[3]{
            RE::NiPoint3{ cosTheta + x * x * oneMinusCos, x * y * oneMinusCos - z * sinTheta, x * z * oneMinusCos + y * sinTheta },
            RE::NiPoint3{ y * x * oneMinusCos + z * sinTheta, cosTheta + y * y * oneMinusCos, y * z * oneMinusCos - x * sinTheta },
            RE::NiPoint3{ z * x * oneMinusCos - y * sinTheta, z * y * oneMinusCos + x * sinTheta, cosTheta + z * z * oneMinusCos },
        };
        return storedRotationFromConventionalRows(conventionalRows);
    }

    [[nodiscard]] inline RE::NiMatrix3 applyWorldRotationToStoredBasis(const RE::NiMatrix3& worldRotationStored, const RE::NiMatrix3& baseRotation)
    {
        RE::NiMatrix3 result{};
        const RE::NiPoint3 basis[3]{
            RE::NiPoint3{ baseRotation.entry[0][0], baseRotation.entry[0][1], baseRotation.entry[0][2] },
            RE::NiPoint3{ baseRotation.entry[1][0], baseRotation.entry[1][1], baseRotation.entry[1][2] },
            RE::NiPoint3{ baseRotation.entry[2][0], baseRotation.entry[2][1], baseRotation.entry[2][2] },
        };

        for (int axis = 0; axis < 3; ++axis) {
            const RE::NiPoint3 rotated = transform_math::rotateLocalVectorToWorld(worldRotationStored, basis[axis]);
            result.entry[axis][0] = rotated.x;
            result.entry[axis][1] = rotated.y;
            result.entry[axis][2] = rotated.z;
        }
        return result;
    }

    [[nodiscard]] inline bool applyAlternateThumbPlaneCorrection(
        const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
        const std::array<LiveFingerTransform, 15>& liveNodes,
        float maxCorrectionRadians,
        float strength,
        frik::api::FRIKApi::FingerLocalTransformOverride& transforms)
    {
        const float curveStrength = grab_finger_local_transform_math::sanitizeUnitStrength(
            strength, grab_finger_local_transform_math::kDefaultThumbAlternateCurveStrength);
        if (!fingerPose.usedAlternateThumbCurve || !fingerPose.hasThumbAlternateCurveFrame ||
            curveStrength <= 0.0f || !std::isfinite(maxCorrectionRadians) || maxCorrectionRadians <= 0.0f) {
            return false;
        }

        const RE::NiPoint3 targetAxisWorld = grab_finger_local_transform_math::alternateThumbPlaneCurlDirection(
            fingerPose.thumbAlternateCurveOpenDirectionWorld,
            fingerPose.thumbAlternateCurveNormalWorld,
            fingerPose.values[0],
            fingerPose.thumbAlternateCurveMaxCurlAngleRadians);
        bool applied = false;
        bool hadUsableThumbSegment = false;
        for (std::size_t segment = 0; segment < 3; ++segment) {
            const std::uint16_t bit = static_cast<std::uint16_t>(1U << segment);
            const auto& node = liveNodes[segment];
            if ((transforms.enabledMask & bit) == 0 || !node.valid) {
                continue;
            }

            const float segmentStrength = grab_finger_local_transform_math::alternateThumbSegmentCorrectionStrength(segment, curveStrength);
            if (segmentStrength <= 0.0f) {
                continue;
            }
            hadUsableThumbSegment = true;

            const RE::NiPoint3 currentAxisWorld = normalizeOrFallback(
                transform_math::rotateLocalVectorToWorld(node.world.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }),
                fingerPose.thumbAlternateCurveOpenDirectionWorld);
            const float dotToTarget = std::clamp(dot(currentAxisWorld, targetAxisWorld), -1.0f, 1.0f);
            float angle = std::acos(dotToTarget);
            if (!std::isfinite(angle) || angle <= 0.0001f) {
                continue;
            }
            angle = std::min(angle, maxCorrectionRadians) * segmentStrength;

            RE::NiPoint3 axis = cross(currentAxisWorld, targetAxisWorld);
            if (lengthSquared(axis) <= 0.000001f) {
                axis = orthogonalAxis(currentAxisWorld);
            }

            const RE::NiMatrix3 rotationDelta = axisAngleStored(axis, angle);
            const RE::NiMatrix3 targetWorldRotation = applyWorldRotationToStoredBasis(rotationDelta, node.world.rotate);
            RE::NiTransform localTransform = transforms.localTransforms[segment];
            localTransform.rotate = transform_math::multiplyStoredRotations(targetWorldRotation, transform_math::transposeRotation(node.parentWorld.rotate));
            if (!isFiniteTransform(localTransform)) {
                continue;
            }

            transforms.localTransforms[segment] = localTransform;
            applied = true;
        }

        return applied || hadUsableThumbSegment;
    }

    [[nodiscard]] inline DirectSkeletonBoneReader& rootFlattenedFingerReader()
    {
        static DirectSkeletonBoneReader reader;
        return reader;
    }

    [[nodiscard]] inline const DirectSkeletonBoneEntry* findSnapshotBone(const DirectSkeletonBoneSnapshot& snapshot, std::string_view name)
    {
        for (const auto& bone : snapshot.bones) {
            if (bone.name == name) {
                return &bone;
            }
        }
        return nullptr;
    }

    [[nodiscard]] inline const DirectSkeletonBoneEntry* findSnapshotBoneByTreeIndex(const DirectSkeletonBoneSnapshot& snapshot, int treeIndex)
    {
        if (treeIndex < 0) {
            return nullptr;
        }

        for (const auto& bone : snapshot.bones) {
            if (bone.treeIndex == treeIndex) {
                return &bone;
            }
        }
        return nullptr;
    }

    [[nodiscard]] inline bool resolveLiveFingerTransforms(bool isLeft, std::array<LiveFingerTransform, 15>& outNodes)
    {
        outNodes = {};

        DirectSkeletonBoneSnapshot snapshot{};
        if (!rootFlattenedFingerReader().capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                snapshot)) {
            return false;
        }

        for (std::size_t finger = 0; finger < 5; ++finger) {
            for (std::size_t segment = 0; segment < 3; ++segment) {
                const std::size_t index = finger * 3 + segment;
                const char* boneName = root_flattened_finger_skeleton_runtime::fingerBoneName(isLeft, finger, segment);
                const auto* node = boneName ? findSnapshotBone(snapshot, boneName) : nullptr;
                const auto* parent = node ? findSnapshotBoneByTreeIndex(snapshot, node->parentTreeIndex) : nullptr;
                if (!node || !parent || !isFiniteTransform(node->world) || !isFiniteTransform(parent->world)) {
                    return false;
                }
                outNodes[index] = LiveFingerTransform{
                    .world = node->world,
                    .parentWorld = parent->world,
                    .valid = true,
                };
            }
        }
        return true;
    }

    [[nodiscard]] inline bool buildSurfaceCorrectedLocalTransforms(
        bool isLeft,
        const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
        const frik::api::FRIKApi::FingerLocalTransformOverride& baseline,
        Options options,
        frik::api::FRIKApi::FingerLocalTransformOverride& outTransforms,
        const char** outFailureReason = nullptr)
    {
        if (outFailureReason) {
            *outFailureReason = "none";
        }
        outTransforms = baseline;
        outTransforms.enabledMask = grab_finger_local_transform_math::sanitizeFingerLocalTransformMask(baseline.enabledMask);
        if (outTransforms.enabledMask != grab_finger_local_transform_math::kFullFingerLocalTransformMask) {
            if (outFailureReason) {
                *outFailureReason = "baseline-mask";
            }
            return false;
        }

        options = sanitizeOptions(options);
        if (!options.enabled) {
            return true;
        }

        constexpr float kDegreesToRadians = 0.01745329251994329577f;
        const float maxCorrectionRadians = options.maxCorrectionDegrees * kDegreesToRadians;
        bool anyCorrected = false;

        const bool wantsSurfaceCorrection =
            options.maxCorrectionDegrees > 0.0f &&
            (options.surfaceAimStrength > 0.0f || options.thumbOppositionStrength > 0.0f);
        const bool wantsAlternateThumbPlaneCorrection =
            fingerPose.usedAlternateThumbCurve &&
            options.maxCorrectionDegrees > 0.0f &&
            options.thumbAlternateCurveStrength > 0.0f;
        if (wantsAlternateThumbPlaneCorrection && !fingerPose.hasThumbAlternateCurveFrame) {
            if (outFailureReason) {
                *outFailureReason = "alternate-thumb-frame";
            }
            return false;
        }

        std::array<LiveFingerTransform, 15> liveNodes{};
        const bool needsLiveNodes = wantsSurfaceCorrection || wantsAlternateThumbPlaneCorrection;
        if (needsLiveNodes && !resolveLiveFingerTransforms(isLeft, liveNodes)) {
            if (outFailureReason) {
                *outFailureReason = "live-root-finger-transforms";
            }
            return false;
        }

        if (wantsSurfaceCorrection) {
            for (std::size_t index = 0; index < liveNodes.size(); ++index) {
                const auto& node = liveNodes[index];
                if (!node.valid) {
                    continue;
                }

                const std::size_t finger = index / 3;
                if (!grab_finger_local_transform_math::shouldApplySurfaceAimCorrection(finger, wantsAlternateThumbPlaneCorrection)) {
                    continue;
                }

                const float strength = grab_finger_local_transform_math::correctionStrengthForFinger(
                    finger, options.surfaceAimStrength, options.thumbOppositionStrength);
                if (strength <= 0.0f) {
                    continue;
                }

                const RE::NiPoint3 currentAxisWorld = normalizeOrFallback(
                    transform_math::rotateLocalVectorToWorld(node.world.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }),
                    RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                const RE::NiPoint3 toSurface = fingerPose.probeEnd[finger] - node.world.translate;
                if (lengthSquared(toSurface) <= 0.000001f) {
                    continue;
                }

                const RE::NiPoint3 targetAxisWorld = normalizeOrFallback(toSurface, currentAxisWorld);
                const float dotToTarget = std::clamp(dot(currentAxisWorld, targetAxisWorld), -1.0f, 1.0f);
                float angle = std::acos(dotToTarget);
                if (!std::isfinite(angle) || angle <= 0.0001f) {
                    continue;
                }
                angle = std::min(angle * strength, maxCorrectionRadians);

                RE::NiPoint3 axis = cross(currentAxisWorld, targetAxisWorld);
                if (lengthSquared(axis) <= 0.000001f) {
                    axis = orthogonalAxis(currentAxisWorld);
                }

                const RE::NiMatrix3 rotationDelta = axisAngleStored(axis, angle);
                const RE::NiMatrix3 targetWorldRotation = applyWorldRotationToStoredBasis(rotationDelta, node.world.rotate);
                RE::NiTransform localTransform = baseline.localTransforms[index];
                localTransform.rotate = transform_math::multiplyStoredRotations(targetWorldRotation, transform_math::transposeRotation(node.parentWorld.rotate));
                if (!isFiniteTransform(localTransform)) {
                    continue;
                }

                outTransforms.localTransforms[index] = localTransform;
                anyCorrected = true;
            }
        }

        const bool appliedAlternateThumbCurve = wantsAlternateThumbPlaneCorrection && applyAlternateThumbPlaneCorrection(
            fingerPose,
            liveNodes,
            maxCorrectionRadians,
            options.thumbAlternateCurveStrength,
            outTransforms);

        if (wantsAlternateThumbPlaneCorrection && !appliedAlternateThumbCurve) {
            if (outFailureReason) {
                *outFailureReason = "alternate-thumb-plane";
            }
            return false;
        }

        if (!anyCorrected && !appliedAlternateThumbCurve && (wantsSurfaceCorrection || wantsAlternateThumbPlaneCorrection) && outFailureReason) {
            *outFailureReason = "no-surface-correction";
        }
        return anyCorrected || appliedAlternateThumbCurve || (!wantsSurfaceCorrection && !wantsAlternateThumbPlaneCorrection);
    }

    [[nodiscard]] inline frik::api::FRIKApi::FingerLocalTransformOverride smoothLocalTransforms(
        const frik::api::FRIKApi::FingerLocalTransformOverride& target,
        State& state,
        float smoothingSpeed,
        float deltaTime)
    {
        frik::api::FRIKApi::FingerLocalTransformOverride smoothed = target;
        const std::uint16_t targetMask = grab_finger_local_transform_math::sanitizeFingerLocalTransformMask(target.enabledMask);
        if (!state.hasCurrentTransforms || state.currentMask != targetMask) {
            for (std::size_t i = 0; i < state.currentTransforms.size(); ++i) {
                state.currentTransforms[i] = target.localTransforms[i];
            }
            state.currentMask = targetMask;
            state.hasCurrentTransforms = true;
            return smoothed;
        }

        const float alpha = grab_finger_local_transform_math::exponentialSmoothingAlpha(smoothingSpeed, deltaTime);
        for (std::size_t i = 0; i < state.currentTransforms.size(); ++i) {
            if ((targetMask & (1U << i)) == 0) {
                continue;
            }

            RE::NiTransform next = target.localTransforms[i];
            next.translate.x = state.currentTransforms[i].translate.x + (target.localTransforms[i].translate.x - state.currentTransforms[i].translate.x) * alpha;
            next.translate.y = state.currentTransforms[i].translate.y + (target.localTransforms[i].translate.y - state.currentTransforms[i].translate.y) * alpha;
            next.translate.z = state.currentTransforms[i].translate.z + (target.localTransforms[i].translate.z - state.currentTransforms[i].translate.z) * alpha;
            next.scale = state.currentTransforms[i].scale + (target.localTransforms[i].scale - state.currentTransforms[i].scale) * alpha;

            const auto currentRotation = hand_visual_lerp_math::matrixToQuaternion(state.currentTransforms[i].rotate);
            const auto targetRotation = hand_visual_lerp_math::matrixToQuaternion(target.localTransforms[i].rotate);
            next.rotate = hand_visual_lerp_math::quaternionToMatrix<RE::NiMatrix3>(
                hand_visual_lerp_math::slerp(currentRotation, targetRotation, alpha));

            state.currentTransforms[i] = next;
            smoothed.localTransforms[i] = next;
        }
        smoothed.enabledMask = targetMask;
        return smoothed;
    }

    inline void clearLocalTransformOverride(const char* tag, frik::api::FRIKApi::Hand hand, int priority, State& state)
    {
        auto* api = frik::api::FRIKApi::inst;
        if (!api || !api->setHandPoseCustomLocalTransformsWithPriority || !state.hasCurrentTransforms) {
            state = {};
            return;
        }

        frik::api::FRIKApi::FingerLocalTransformOverride clearData{};
        api->setHandPoseCustomLocalTransformsWithPriority(tag, hand, &clearData, priority);
        state = {};
    }

    [[nodiscard]] inline bool publishLocalTransformPose(
        const char* tag,
        frik::api::FRIKApi::Hand hand,
        bool isLeft,
        const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
        const std::array<float, 15>& jointValues,
        Options options,
        float deltaTime,
        int priority,
        State& state)
    {
        auto* api = frik::api::FRIKApi::inst;
        const bool canPublish = api &&
            grab_finger_local_transform_math::shouldPublishLocalTransformPose(
                options.enabled,
                fingerPose.solved,
                fingerPose.hasJointValues,
                api->getHandPoseLocalTransformsForJointPositions != nullptr,
                api->setHandPoseCustomLocalTransformsWithPriority != nullptr);
        if (!canPublish) {
            clearLocalTransformOverride(tag, hand, priority, state);
            return false;
        }

        frik::api::FRIKApi::FingerLocalTransformOverride baseline{};
        if (!api->getHandPoseLocalTransformsForJointPositions(hand, jointValues.data(), &baseline)) {
            clearLocalTransformOverride(tag, hand, priority, state);
            return false;
        }

        frik::api::FRIKApi::FingerLocalTransformOverride corrected{};
        if (!buildSurfaceCorrectedLocalTransforms(isLeft, fingerPose, baseline, options, corrected)) {
            clearLocalTransformOverride(tag, hand, priority, state);
            return false;
        }

        const Options sanitizedOptions = sanitizeOptions(options);
        const auto smoothed = smoothLocalTransforms(corrected, state, sanitizedOptions.smoothingSpeed, deltaTime);
        api->setHandPoseCustomLocalTransformsWithPriority(tag, hand, &smoothed, priority);
        return true;
    }
}
