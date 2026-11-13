#pragma once

#include <algorithm>
#include <array>
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>
#include <vector>
#include "RE/NetImmerse/NiTransform.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/VectorMath.h"
#include "physics-interaction/grab/GeneratedGrabFingerCalibration.h"
#include "physics-interaction/grab/GrabFingerLocalTransformMath.h"
#include "physics-interaction/grab/GrabFingerPoseRuntime.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

/*
 * ROCK publishes full-hand local transforms only after FRIK has generated the
 * authored local pose for the same 15 joint values. ROCK reconstructs those
 * joints under the captured controller hand and adds bounded mesh-contact aim
 * corrections. The flattened tree supplies hand placement and chain identity;
 * displayed finger rotations never become a new pose baseline.
 */

namespace rock::grab_finger_local_transform_runtime
{
    struct Options
    {
        bool enabled = true;
        float smoothingSpeed = grab_finger_local_transform_math::kDefaultLocalTransformSmoothingSpeed;
        float maxCorrectionDegrees = grab_finger_local_transform_math::kDefaultMaxCorrectionDegrees;
        float surfaceAimStrength = grab_finger_local_transform_math::kDefaultSurfaceAimStrength;
        float thumbOppositionStrength = grab_finger_local_transform_math::kDefaultThumbOppositionStrength;
        float thumbAlternateCurveStrength = grab_finger_local_transform_math::kDefaultThumbAlternateCurveStrength;
        bool thumbSurfaceSafetyEnabled = true;
        float thumbSurfaceSafetyMarginGameUnits = 1.0f;
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

    // Rebuild each joint under the already corrected parent. Rendered finger
    // rotations belong to the previous provider pose and must never seed this
    // solve or replace the requested curl/splay baseline.
    inline void updatePoseFingerTransform(std::size_t index,
        const frik_visual_authority::FingerLocalTransformOverride& transforms,
        std::array<LiveFingerTransform, 15>& nodes)
    {
        auto& node = nodes[index];
        if (index % 3 != 0) {
            node.parentWorld = nodes[index - 1].world;
        }
        node.world = transform_math::composeTransforms(node.parentWorld, transforms.localTransforms[index]);
        node.world.rotate = transform_math::orthonormalizeStoredRotation(node.world.rotate);
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

    inline constexpr std::size_t kInvalidFingerLocalTransformIndex = 15;

    inline void logRejectedFingerTransform(bool isLeft, std::size_t index, const char* stage,
        const RE::NiTransform& transform, grab_finger_local_transform_math::FingerLocalTransformSafetyFailure failure)
    {
        const std::size_t finger = index / 3;
        const std::size_t segment = index % 3;
        const char* boneName = index < kInvalidFingerLocalTransformIndex
            ? root_flattened_finger_skeleton_runtime::fingerBoneName(isLeft, finger, segment)
            : nullptr;
        const float translationLengthSquared = transform.translate.x * transform.translate.x +
            transform.translate.y * transform.translate.y + transform.translate.z * transform.translate.z;
        const float translationLength = std::isfinite(translationLengthSquared) && translationLengthSquared >= 0.0f
            ? std::sqrt(translationLengthSquared)
            : translationLengthSquared;

        ROCK_LOG_SAMPLE_WARN(Hand, 1000,
            "Dynamic grab finger transform rejected hand={} stage={} bone={} index={} detail={} scale={:.6f} "
            "translation={:.3f} rowNormSq=({:.4f},{:.4f},{:.4f}) rowDot=({:.4f},{:.4f},{:.4f}) determinant={:.4f}",
            isLeft ? "left" : "right", stage ? stage : "unknown", boneName ? boneName : "unknown", index,
            grab_finger_local_transform_math::fingerLocalTransformSafetyFailureName(failure), transform.scale,
            translationLength, grab_finger_local_transform_math::storedRotationRowDot(transform, 0, 0),
            grab_finger_local_transform_math::storedRotationRowDot(transform, 1, 1),
            grab_finger_local_transform_math::storedRotationRowDot(transform, 2, 2),
            grab_finger_local_transform_math::storedRotationRowDot(transform, 0, 1),
            grab_finger_local_transform_math::storedRotationRowDot(transform, 0, 2),
            grab_finger_local_transform_math::storedRotationRowDot(transform, 1, 2),
            grab_finger_local_transform_math::storedRotationDeterminant(transform));
    }

    inline void setTransformFailure(const char** outFailureReason, std::size_t* outFailureIndex, const char* reason,
        std::size_t index)
    {
        if (outFailureReason) {
            *outFailureReason = reason;
        }
        if (outFailureIndex) {
            *outFailureIndex = index;
        }
    }

    [[nodiscard]] inline bool fingerLocalTransformOverrideIsSafeForPublication(
        const frik_visual_authority::FingerLocalTransformOverride& transforms, std::size_t* outFailureIndex = nullptr,
        grab_finger_local_transform_math::FingerLocalTransformSafetyFailure* outFailure = nullptr)
    {
        if (outFailureIndex) {
            *outFailureIndex = kInvalidFingerLocalTransformIndex;
        }
        if (outFailure) {
            *outFailure = grab_finger_local_transform_math::FingerLocalTransformSafetyFailure::None;
        }
        if (grab_finger_local_transform_math::sanitizeFingerLocalTransformMask(transforms.enabledMask) !=
            grab_finger_local_transform_math::kFullFingerLocalTransformMask) {
            return false;
        }

        for (std::size_t index = 0; index < std::size(transforms.localTransforms); ++index) {
            const auto failure = grab_finger_local_transform_math::inspectFingerLocalTransformForPublication(
                transforms.localTransforms[index]);
            if (failure == grab_finger_local_transform_math::FingerLocalTransformSafetyFailure::None) {
                continue;
            }
            if (outFailureIndex) {
                *outFailureIndex = index;
            }
            if (outFailure) {
                *outFailure = failure;
            }
            return false;
        }
        return true;
    }

    [[nodiscard]] inline float dot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return vector_math::dot(lhs, rhs);
    }

    [[nodiscard]] inline RE::NiPoint3 cross(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return vector_math::cross(lhs, rhs);
    }

    [[nodiscard]] inline float lengthSquared(const RE::NiPoint3& value) { return vector_math::lengthSquared(value); }

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
        const RE::NiPoint3 axis =
            std::abs(value.x) < 0.7f ? RE::NiPoint3{ 1.0f, 0.0f, 0.0f } : RE::NiPoint3{ 0.0f, 1.0f, 0.0f };
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
            RE::NiPoint3{ cosTheta + x * x * oneMinusCos, x * y * oneMinusCos - z * sinTheta,
                x * z * oneMinusCos + y * sinTheta },
            RE::NiPoint3{ y * x * oneMinusCos + z * sinTheta, cosTheta + y * y * oneMinusCos,
                y * z * oneMinusCos - x * sinTheta },
            RE::NiPoint3{ z * x * oneMinusCos - y * sinTheta, z * y * oneMinusCos + x * sinTheta,
                cosTheta + z * z * oneMinusCos },
        };
        return storedRotationFromConventionalRows(conventionalRows);
    }

    [[nodiscard]] inline RE::NiMatrix3 applyWorldRotationToStoredBasis(const RE::NiMatrix3& worldRotationStored,
        const RE::NiMatrix3& baseRotation)
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
        std::array<LiveFingerTransform, 15>& liveNodes, float maxCorrectionRadians, float strength,
        bool surfaceSafetyEnabled, float surfaceSafetyMarginGameUnits,
        frik_visual_authority::FingerLocalTransformOverride& transforms)
    {
        const float configuredStrength = strength;
        const float laneStrength = fingerPose.selectedThumbLaneLocalCorrectionStrength > 0.0f
            ? fingerPose.selectedThumbLaneLocalCorrectionStrength
            : 1.0f;
        const float curveStrength = configuredStrength * laneStrength;
        if (!grab_finger_local_transform_math::shouldApplyAlternateThumbLocalCorrection(
                fingerPose.usedAlternateThumbCurve, fingerPose.usedAlternateThumbSurfaceHit) ||
            !fingerPose.hasThumbAlternateCurveFrame || curveStrength <= 0.0f || !std::isfinite(maxCorrectionRadians) ||
            maxCorrectionRadians <= 0.0f) {
            return false;
        }

        const RE::NiPoint3 targetAxisWorld = grab_finger_local_transform_math::alternateThumbPlaneCurlDirection(
            fingerPose.thumbAlternateCurveOpenDirectionWorld, fingerPose.thumbAlternateCurveNormalWorld,
            fingerPose.values[0], fingerPose.thumbAlternateCurveMaxCurlAngleRadians);
        const bool hasThumbSurface =
            fingerPose.surfaceAimTargetValid[0] != 0 && fingerPose.surfaceAimNormalValid[0] != 0 && liveNodes[2].valid;
        if (!grab_finger_local_transform_math::thumbAlternateSurfaceGuardAllows(liveNodes[2].world.translate,
                targetAxisWorld, fingerPose.surfaceAimTarget[0], fingerPose.surfaceAimNormal[0],
                surfaceSafetyMarginGameUnits, surfaceSafetyEnabled, hasThumbSurface)) {
            return false;
        }

        bool applied = false;
        for (std::size_t segment = 0; segment < 3; ++segment) {
            updatePoseFingerTransform(segment, transforms, liveNodes);
            const std::uint16_t bit = static_cast<std::uint16_t>(1U << segment);
            const auto& node = liveNodes[segment];
            if ((transforms.enabledMask & bit) == 0 || !node.valid) {
                continue;
            }

            const float segmentStrength =
                grab_finger_local_transform_math::alternateThumbSegmentCorrectionStrength(segment, curveStrength);
            if (segmentStrength <= 0.0f) {
                continue;
            }

            const RE::NiPoint3 currentAxisWorld = normalizeOrFallback(
                transform_math::rotateLocalVectorToWorld(node.world.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }),
                fingerPose.thumbAlternateCurveOpenDirectionWorld);
            const float dotToTarget = std::clamp(dot(currentAxisWorld, targetAxisWorld), -1.0f, 1.0f);
            float angle = std::acos(dotToTarget);
            if (!std::isfinite(angle) || angle <= 0.0001f) {
                // Already on the selected lane is a successful correction.
                applied = applied || std::isfinite(angle);
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
            localTransform.rotate = transform_math::orthonormalizeStoredRotation(
                transform_math::multiplyStoredRotations(targetWorldRotation,
                    transform_math::transposeRotation(node.parentWorld.rotate)));
            transforms.localTransforms[segment] = localTransform;
            updatePoseFingerTransform(segment, transforms, liveNodes);
            applied = true;
        }

        return applied;
    }

    [[nodiscard]] inline DirectSkeletonBoneReader& rootFlattenedFingerReader()
    {
        static DirectSkeletonBoneReader reader;
        return reader;
    }

    [[nodiscard]] inline const DirectSkeletonBoneEntry* findSnapshotBone(const DirectSkeletonBoneSnapshot& snapshot,
        std::string_view name)
    {
        for (const auto& bone : snapshot.bones) {
            if (bone.name == name) {
                return &bone;
            }
        }
        return nullptr;
    }

    [[nodiscard]] inline const DirectSkeletonBoneEntry* findSnapshotBoneByTreeIndex(
        const DirectSkeletonBoneSnapshot& snapshot, int treeIndex)
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

    [[nodiscard]] inline bool resolveFingerTransforms(const DirectSkeletonBoneSnapshot& snapshot, bool isLeft,
        const frik_visual_authority::FingerLocalTransformOverride& baseline,
        std::array<LiveFingerTransform, 15>& outNodes, const char** outFailureReason = nullptr,
        std::size_t* outFailureIndex = nullptr)
    {
        outNodes = {};
        if (!snapshot.valid) {
            setTransformFailure(outFailureReason, outFailureIndex, "live-snapshot-invalid",
                kInvalidFingerLocalTransformIndex);
            return false;
        }

        const auto* hand = findSnapshotBone(snapshot, isLeft ? "LArm_Hand" : "RArm_Hand");
        if (!hand || !isFiniteTransform(hand->world) || hand->world.scale <= 0.000001f ||
            grab_finger_local_transform_math::inspectStoredRotationBasis(hand->world) !=
                grab_finger_local_transform_math::FingerLocalTransformSafetyFailure::None) {
            setTransformFailure(outFailureReason, outFailureIndex, "pose-hand-transform",
                kInvalidFingerLocalTransformIndex);
            return false;
        }
        const RE::NiTransform handWorld = transform_math::orthonormalizedTransform(hand->world);

        for (std::size_t finger = 0; finger < 5; ++finger) {
            for (std::size_t segment = 0; segment < 3; ++segment) {
                const std::size_t index = finger * 3 + segment;
                const char* boneName = root_flattened_finger_skeleton_runtime::fingerBoneName(isLeft, finger, segment);
                const auto* node = boneName ? findSnapshotBone(snapshot, boneName) : nullptr;
                const auto* parent = node ? findSnapshotBoneByTreeIndex(snapshot, node->parentTreeIndex) : nullptr;
                const char* expectedParentName = segment == 0
                    ? (isLeft ? "LArm_Hand" : "RArm_Hand")
                    : root_flattened_finger_skeleton_runtime::fingerBoneName(isLeft, finger, segment - 1);
                if (!node || !parent || parent->name != expectedParentName) {
                    ROCK_LOG_SAMPLE_WARN(Hand, 1000,
                        "Dynamic grab finger transform rejected hand={} stage=live-node-resolution bone={} index={} "
                        "node={} parent={}",
                        isLeft ? "left" : "right", boneName ? boneName : "unknown", index, node ? "present" : "missing",
                        parent ? "present" : "missing");
                    setTransformFailure(outFailureReason, outFailureIndex, "finger-parent-chain", index);
                    return false;
                }

                const auto nodeFailure = !isFiniteTransform(node->world)
                    ? grab_finger_local_transform_math::FingerLocalTransformSafetyFailure::NonFinite
                    : grab_finger_local_transform_math::inspectStoredRotationBasis(node->world);
                if (nodeFailure != grab_finger_local_transform_math::FingerLocalTransformSafetyFailure::None) {
                    // Diagnostic only: the commanded pose is rebuilt below,
                    // so a bad displayed finger cannot contaminate its target.
                    logRejectedFingerTransform(isLeft, index, "rendered-input-excluded", node->world, nodeFailure);
                }
                outNodes[index] = LiveFingerTransform{
                    .parentWorld = handWorld,
                    .valid = true,
                };
                updatePoseFingerTransform(index, baseline, outNodes);
            }
        }
        return true;
    }

    [[nodiscard]] inline bool resolveLiveFingerTransforms(bool isLeft,
        const frik_visual_authority::FingerLocalTransformOverride& baseline,
        std::array<LiveFingerTransform, 15>& outNodes, const char** outFailureReason = nullptr,
        std::size_t* outFailureIndex = nullptr)
    {
        DirectSkeletonBoneSnapshot snapshot{};
        if (!rootFlattenedFingerReader().capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                SkeletonBoneCaptureSpace::Controller, snapshot)) {
            outNodes = {};
            setTransformFailure(outFailureReason, outFailureIndex, "live-snapshot-capture",
                kInvalidFingerLocalTransformIndex);
            return false;
        }
        return resolveFingerTransforms(snapshot, isLeft, baseline, outNodes, outFailureReason, outFailureIndex);
    }

    [[nodiscard]] inline bool buildSurfaceCorrectedLocalTransforms(bool isLeft,
        const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
        const frik_visual_authority::FingerLocalTransformOverride& baseline, Options options,
        frik_visual_authority::FingerLocalTransformOverride& outTransforms, const char** outFailureReason = nullptr,
        const DirectSkeletonBoneSnapshot* capturedFingerSnapshot = nullptr, std::size_t* outFailureIndex = nullptr)
    {
        if (outFailureReason) {
            *outFailureReason = "none";
        }
        if (outFailureIndex) {
            *outFailureIndex = kInvalidFingerLocalTransformIndex;
        }
        outTransforms = baseline;
        outTransforms.enabledMask =
            grab_finger_local_transform_math::sanitizeFingerLocalTransformMask(baseline.enabledMask);
        if (outTransforms.enabledMask != grab_finger_local_transform_math::kFullFingerLocalTransformMask) {
            setTransformFailure(outFailureReason, outFailureIndex, "baseline-mask", kInvalidFingerLocalTransformIndex);
            return false;
        }

        for (auto& local : outTransforms.localTransforms) {
            local.rotate = transform_math::orthonormalizeStoredRotation(local.rotate);
        }

        if (!options.enabled) {
            return true;
        }

        constexpr float kDegreesToRadians = 0.01745329251994329577f;
        const float maxCorrectionRadians = options.maxCorrectionDegrees * kDegreesToRadians;
        bool anyCorrected = false;

        const bool wantsSurfaceCorrection = options.maxCorrectionDegrees > 0.0f &&
            (options.surfaceAimStrength > 0.0f ||
                (fingerPose.thumbSurfaceFollowAllowed && options.thumbOppositionStrength > 0.0f));
        const bool wantsAlternateThumbPlaneCorrection = fingerPose.thumbSurfaceFollowAllowed &&
            grab_finger_local_transform_math::shouldApplyAlternateThumbLocalCorrection(
                fingerPose.usedAlternateThumbCurve, fingerPose.usedAlternateThumbSurfaceHit) &&
            options.maxCorrectionDegrees > 0.0f && options.thumbAlternateCurveStrength > 0.0f;
        if (wantsAlternateThumbPlaneCorrection && !fingerPose.hasThumbAlternateCurveFrame) {
            if (outFailureReason) {
                *outFailureReason = "alternate-thumb-frame";
            }
            return false;
        }

        std::array<LiveFingerTransform, 15> liveNodes{};
        const bool needsLiveNodes = wantsSurfaceCorrection || wantsAlternateThumbPlaneCorrection;
        bool liveNodesResolved = true;
        if (needsLiveNodes) {
            liveNodesResolved = capturedFingerSnapshot && capturedFingerSnapshot->valid
                ? resolveFingerTransforms(*capturedFingerSnapshot, isLeft, outTransforms, liveNodes, outFailureReason,
                      outFailureIndex)
                : resolveLiveFingerTransforms(isLeft, outTransforms, liveNodes, outFailureReason, outFailureIndex);
        }
        if (!liveNodesResolved) {
            return false;
        }

        if (wantsSurfaceCorrection) {
            for (std::size_t index = 0; index < liveNodes.size(); ++index) {
                updatePoseFingerTransform(index, outTransforms, liveNodes);
                const auto& node = liveNodes[index];
                if (!node.valid) {
                    continue;
                }

                const std::size_t finger = index / 3;
                if (!fingerPose.surfaceAimTargetValid[finger]) {
                    continue;
                }
                const bool allowSurfaceAim = grab_finger_local_transform_math::shouldApplySurfaceAimCorrection(finger,
                    wantsAlternateThumbPlaneCorrection, fingerPose.thumbSurfaceFollowAllowed);
                if (!allowSurfaceAim) {
                    continue;
                }

                const float strength = grab_finger_local_transform_math::correctionStrengthForFinger(finger,
                    options.surfaceAimStrength, options.thumbOppositionStrength);
                if (strength <= 0.0f) {
                    continue;
                }

                const std::size_t segment = index % 3;

                const RE::NiPoint3 currentAxisWorld = normalizeOrFallback(
                    transform_math::rotateLocalVectorToWorld(node.world.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }),
                    RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                const RE::NiPoint3 toSurface = fingerPose.surfaceAimTarget[finger] - node.world.translate;
                if (lengthSquared(toSurface) <= 0.000001f) {
                    continue;
                }

                const RE::NiPoint3 targetAxisWorld = normalizeOrFallback(toSurface, currentAxisWorld);
                const float dotToTarget = std::clamp(dot(currentAxisWorld, targetAxisWorld), -1.0f, 1.0f);
                float angle = std::acos(dotToTarget);
                if (!std::isfinite(angle) || angle <= 0.0001f) {
                    continue;
                }
                angle = grab_finger_local_transform_math::boundedSurfaceAimCorrectionRadians(angle, strength,
                    maxCorrectionRadians, finger, segment);
                if (angle <= 0.0001f) {
                    continue;
                }

                RE::NiPoint3 axis = cross(currentAxisWorld, targetAxisWorld);
                if (lengthSquared(axis) <= 0.000001f) {
                    axis = orthogonalAxis(currentAxisWorld);
                }

                const RE::NiMatrix3 rotationDelta = axisAngleStored(axis, angle);
                const RE::NiMatrix3 targetWorldRotation =
                    applyWorldRotationToStoredBasis(rotationDelta, node.world.rotate);
                RE::NiTransform localTransform = outTransforms.localTransforms[index];
                localTransform.rotate = transform_math::orthonormalizeStoredRotation(
                    transform_math::multiplyStoredRotations(targetWorldRotation,
                        transform_math::transposeRotation(node.parentWorld.rotate)));
                outTransforms.localTransforms[index] = localTransform;
                updatePoseFingerTransform(index, outTransforms, liveNodes);
                anyCorrected = true;
            }
        }

        const bool appliedAlternateThumbCurve = wantsAlternateThumbPlaneCorrection &&
            applyAlternateThumbPlaneCorrection(fingerPose, liveNodes, maxCorrectionRadians,
                options.thumbAlternateCurveStrength, options.thumbSurfaceSafetyEnabled,
                options.thumbSurfaceSafetyMarginGameUnits, outTransforms);

        if (wantsAlternateThumbPlaneCorrection && !appliedAlternateThumbCurve) {
            setTransformFailure(outFailureReason, outFailureIndex, "alternate-thumb-plane",
                kInvalidFingerLocalTransformIndex);
            return false;
        }

        if (!anyCorrected && !appliedAlternateThumbCurve &&
            (wantsSurfaceCorrection || wantsAlternateThumbPlaneCorrection) && outFailureReason) {
            *outFailureReason = "surface-correction-not-needed";
        }
        return anyCorrected || appliedAlternateThumbCurve || !wantsAlternateThumbPlaneCorrection;
    }

    [[nodiscard]] inline frik_visual_authority::FingerLocalTransformOverride smoothLocalTransforms(
        const frik_visual_authority::FingerLocalTransformOverride& target, State& state, float smoothingSpeed,
        float deltaTime)
    {
        frik_visual_authority::FingerLocalTransformOverride smoothed = target;
        const std::uint16_t targetMask =
            grab_finger_local_transform_math::sanitizeFingerLocalTransformMask(target.enabledMask);
        smoothed.enabledMask = targetMask;
        const auto resetStateToTarget = [&]() {
            for (std::size_t i = 0; i < state.currentTransforms.size(); ++i) {
                state.currentTransforms[i] = target.localTransforms[i];
            }
            state.currentMask = targetMask;
            state.hasCurrentTransforms = true;
        };
        if (!state.hasCurrentTransforms || state.currentMask != targetMask) {
            resetStateToTarget();
            return smoothed;
        }

        const float alpha = grab_finger_local_transform_math::exponentialSmoothingAlpha(smoothingSpeed, deltaTime);
        for (std::size_t i = 0; i < state.currentTransforms.size(); ++i) {
            if ((targetMask & (1U << i)) == 0) {
                continue;
            }

            RE::NiTransform next = target.localTransforms[i];
            next.translate.x = state.currentTransforms[i].translate.x +
                (target.localTransforms[i].translate.x - state.currentTransforms[i].translate.x) * alpha;
            next.translate.y = state.currentTransforms[i].translate.y +
                (target.localTransforms[i].translate.y - state.currentTransforms[i].translate.y) * alpha;
            next.translate.z = state.currentTransforms[i].translate.z +
                (target.localTransforms[i].translate.z - state.currentTransforms[i].translate.z) * alpha;
            next.scale = state.currentTransforms[i].scale +
                (target.localTransforms[i].scale - state.currentTransforms[i].scale) * alpha;

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

    inline void clearLocalTransformOverride(const char* tag, frik_visual_authority::Hand hand, int priority,
        State& state)
    {
        if (!state.hasCurrentTransforms) {
            state = {};
            return;
        }

        frik_visual_authority::FingerLocalTransformOverride clearData{};
        (void)frik_visual_authority::setHandPoseCustomLocalTransforms(tag, hand, &clearData, priority);
        state = {};
    }

    [[nodiscard]] inline bool publishLocalTransformPose(const char* tag, frik_visual_authority::Hand hand, bool isLeft,
        const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
        const frik_visual_authority::HandPoseData& handPose, Options options, float deltaTime, int priority,
        State& state)
    {
        const auto* api = frik_visual_authority::api();
        const bool canPublish = grab_finger_local_transform_math::shouldPublishLocalTransformPose(options.enabled,
            fingerPose.solved, true, api && api->getHandPoseLocalTransformsForPose != nullptr,
            api && api->setHandPoseCustomLocalTransforms != nullptr);
        if (!canPublish) {
            clearLocalTransformOverride(tag, hand, priority, state);
            return false;
        }

        frik_visual_authority::FingerLocalTransformOverride baseline{};
        if (!frik_visual_authority::getHandPoseLocalTransformsForPose(hand, handPose, &baseline)) {
            clearLocalTransformOverride(tag, hand, priority, state);
            return false;
        }

        frik_visual_authority::FingerLocalTransformOverride corrected{};
        const char* failureReason = "unknown";
        std::size_t failureIndex = kInvalidFingerLocalTransformIndex;
        if (!buildSurfaceCorrectedLocalTransforms(isLeft, fingerPose, baseline, options, corrected, &failureReason,
                nullptr, &failureIndex)) {
            clearLocalTransformOverride(tag, hand, priority, state);
            return false;
        }

        const auto smoothed = smoothLocalTransforms(corrected, state, options.smoothingSpeed, deltaTime);
        // The one safety check: an unsafe result clears the override and
        // the smoothing state, so the next publish starts from the target.
        std::size_t unsafeIndex = kInvalidFingerLocalTransformIndex;
        grab_finger_local_transform_math::FingerLocalTransformSafetyFailure unsafeFailure{};
        if (!fingerLocalTransformOverrideIsSafeForPublication(smoothed, &unsafeIndex, &unsafeFailure)) {
            if (unsafeIndex < std::size(smoothed.localTransforms)) {
                logRejectedFingerTransform(isLeft, unsafeIndex, "publication-boundary",
                    smoothed.localTransforms[unsafeIndex], unsafeFailure);
            }
            clearLocalTransformOverride(tag, hand, priority, state);
            return false;
        }

        if (!frik_visual_authority::setHandPoseCustomLocalTransforms(tag, hand, &smoothed, priority)) {
            clearLocalTransformOverride(tag, hand, priority, state);
            return false;
        }
        return true;
    }
}
