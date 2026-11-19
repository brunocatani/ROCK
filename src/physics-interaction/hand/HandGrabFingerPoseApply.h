// Applies a solved grab finger pose to the hand: the standard pose, the swept-arc
// pose, and the pose shown while the object is still being acquired.
#pragma once

#include "physics-interaction/hand/HandGrabInternal.h"

namespace rock::hand_grab_internal
{
    /*
         * Publishes a mesh finger pose as FRIK joint values. Mesh poses carry no
         * per-bone local transforms, so any exact-local pose still held under
         * this tag (a saved offset) is cleared first.
         */
    inline void applyRockGrabHandPose(bool isLeft, const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
        std::array<float, 15>& currentJointPose, bool& hasCurrentJointPose,
        std::array<RE::NiTransform, 15>& currentLocalTransforms, std::uint16_t& currentLocalTransformMask,
        bool& hasCurrentLocalTransforms, float deltaTime, bool snapJointPoseToTarget = false)
    {
        if (!frik_visual_authority::isAvailable()) {
            return;
        }

        const auto hand = handFromBool(isLeft);
        grab_finger_local_transform_runtime::State localTransformState{
            .currentTransforms = currentLocalTransforms,
            .currentMask = currentLocalTransformMask,
            .hasCurrentTransforms = hasCurrentLocalTransforms,
        };
        grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
        currentLocalTransforms = localTransformState.currentTransforms;
        currentLocalTransformMask = localTransformState.currentMask;
        hasCurrentLocalTransforms = localTransformState.hasCurrentTransforms;

        if (g_rockConfig.rockGrabMeshJointPoseEnabled && fingerPose.solved) {
            const auto targetJointPose = fingerPose.hasJointValues
                ? fingerPose.jointValues
                : grab_finger_pose_math::expandFingerCurlsToJointValues(fingerPose.values);
            if (!hasCurrentJointPose || snapJointPoseToTarget) {
                currentJointPose = targetJointPose;
                hasCurrentJointPose = true;
            } else {
                currentJointPose = grab_finger_pose_math::advanceJointValues(currentJointPose, targetJointPose,
                    g_rockConfig.rockGrabFingerPoseSmoothingSpeed, deltaTime);
            }
            if (!frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand,
                    frik_visual_authority::makeHandPoseDataFromJointValues(currentJointPose), 100)) {
                hasCurrentJointPose = false;
            }
            return;
        }

        hasCurrentJointPose = false;
        if (fingerPose.solved) {
            const auto handPose = frik_visual_authority::makeUniformHandPoseData(fingerPose.values[0],
                fingerPose.values[1], fingerPose.values[2], fingerPose.values[3], fingerPose.values[4]);
            (void)frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand, handPose, 100);
            return;
        }

        const float closedValue =
            std::clamp(g_rockConfig.rockSelectedCloseFingerAnimValue, g_rockConfig.rockGrabFingerMinValue, 1.0f);
        const auto closedPose = frik_visual_authority::makeUniformHandPoseData(closedValue, closedValue, closedValue,
            closedValue, closedValue);
        (void)frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand, closedPose, 100);
    }

    /*
         * FingerPoses.LooseGrabFingerPoseMode = 1: the pre-capsule publication for a swept-arc
         * mesh pose (contact splay, then the surface-aim local-transform layer when
         * publishLocalTransforms is set), unchanged.
         */
    inline void applySweptArcGrabHandPose(bool isLeft, const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
        std::array<float, 15>& currentJointPose, bool& hasCurrentJointPose,
        std::array<RE::NiTransform, 15>& currentLocalTransforms, std::uint16_t& currentLocalTransformMask,
        bool& hasCurrentLocalTransforms, float deltaTime, bool publishLocalTransforms = true,
        bool snapJointPoseToTarget = false)
    {
        if (!frik_visual_authority::isAvailable()) {
            return;
        }

        const auto hand = handFromBool(isLeft);
        grab_finger_local_transform_runtime::State localTransformState{
            .currentTransforms = currentLocalTransforms,
            .currentMask = currentLocalTransformMask,
            .hasCurrentTransforms = hasCurrentLocalTransforms,
        };
        auto syncLocalTransformState = [&]() {
            currentLocalTransforms = localTransformState.currentTransforms;
            currentLocalTransformMask = localTransformState.currentMask;
            hasCurrentLocalTransforms = localTransformState.hasCurrentTransforms;
        };

        if (g_rockConfig.rockGrabMeshJointPoseEnabled && fingerPose.solved) {
            const auto targetJointPose = fingerPose.hasJointValues
                ? fingerPose.jointValues
                : grab_finger_pose_math::expandFingerCurlsToJointValues(fingerPose.values);
            if (!hasCurrentJointPose || snapJointPoseToTarget) {
                currentJointPose = targetJointPose;
                hasCurrentJointPose = true;
            } else {
                currentJointPose = grab_finger_pose_math::advanceJointValues(currentJointPose, targetJointPose,
                    g_rockConfig.rockGrabFingerPoseSmoothingSpeed, deltaTime);
            }

            std::array<float, 5> currentSplayRadians{};
            (void)grab_finger_pose_runtime::resolveSurfaceContactSplayValues(isLeft, fingerPose, currentSplayRadians);
            const auto currentHandPose =
                frik_visual_authority::makeHandPoseDataFromJointValues(currentJointPose, currentSplayRadians);
            if (!frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand, currentHandPose, 100)) {
                hasCurrentJointPose = false;
                grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100,
                    localTransformState);
                syncLocalTransformState();
                return;
            }
            (void)grab_finger_local_transform_runtime::publishLocalTransformPose("ROCK_Grab", hand, isLeft, fingerPose,
                currentHandPose,
                grab_finger_local_transform_runtime::Options{
                    .enabled = publishLocalTransforms && g_rockConfig.rockGrabMeshLocalTransformPoseEnabled,
                    .smoothingSpeed = g_rockConfig.rockGrabFingerLocalTransformSmoothingSpeed,
                    .maxCorrectionDegrees = g_rockConfig.rockGrabFingerLocalTransformMaxCorrectionDegrees,
                    .surfaceAimStrength = g_rockConfig.rockGrabFingerSurfaceAimStrength,
                    .thumbOppositionStrength = g_rockConfig.rockGrabThumbOppositionStrength,
                    .thumbAlternateCurveStrength = g_rockConfig.rockGrabThumbAlternateCurveStrength,
                    .thumbSurfaceSafetyMarginGameUnits = g_rockConfig.rockGrabThumbSurfaceSafetyMarginGameUnits,
                },
                deltaTime, 100, localTransformState);
            syncLocalTransformState();
            return;
        }

        if (fingerPose.solved) {
            hasCurrentJointPose = false;
            grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100,
                localTransformState);
            syncLocalTransformState();
            std::array<float, 5> currentSplayRadians{};
            (void)grab_finger_pose_runtime::resolveSurfaceContactSplayValues(isLeft, fingerPose, currentSplayRadians);
            const auto handPose =
                frik_visual_authority::makeUniformHandPoseData(fingerPose.values[0], fingerPose.values[1],
                    fingerPose.values[2], fingerPose.values[3], fingerPose.values[4], currentSplayRadians);
            (void)frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand, handPose, 100);
            return;
        }

        hasCurrentJointPose = false;
        grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
        syncLocalTransformState();
        const float closedValue =
            std::clamp(g_rockConfig.rockSelectedCloseFingerAnimValue, g_rockConfig.rockGrabFingerMinValue, 1.0f);
        const auto closedPose = frik_visual_authority::makeUniformHandPoseData(closedValue, closedValue, closedValue,
            closedValue, closedValue);
        (void)frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand, closedPose, 100);
    }

    inline grab_finger_pose_runtime::SolvedGrabFingerPose buildAcquisitionFingerPose(
        const grab_finger_pose_runtime::SolvedGrabFingerPose& targetPose, float progress)
    {
        auto pose = targetPose;
        const float t = std::clamp(std::isfinite(progress) ? progress : 0.0f, 0.0f, 1.0f);
        const float precloseValue = g_rockConfig.rockSelectedCloseFingerAnimValue;

        for (std::size_t finger = 0; finger < pose.values.size(); ++finger) {
            const float solvedValue =
                targetPose.solved ? std::clamp(targetPose.values[finger], 0.0f, 1.0f) : precloseValue;
            const float closingTarget = (std::min)(precloseValue, solvedValue);
            pose.values[finger] = precloseValue + (closingTarget - precloseValue) * t;
        }

        if (targetPose.solved && targetPose.hasJointValues) {
            std::array<float, 5> precloseValues{ precloseValue, precloseValue, precloseValue, precloseValue,
                precloseValue };
            const auto precloseJoints = grab_finger_pose_math::expandFingerCurlsToJointValues(precloseValues);
            for (std::size_t joint = 0; joint < pose.jointValues.size(); ++joint) {
                const float targetJoint = std::clamp(targetPose.jointValues[joint], 0.0f, 1.0f);
                pose.jointValues[joint] = precloseJoints[joint] + (targetJoint - precloseJoints[joint]) * t;
            }
        } else {
            pose.jointValues = grab_finger_pose_math::expandFingerCurlsToJointValues(pose.values);
        }
        pose.solved = true;
        pose.hasJointValues = true;
        return pose;
    }
}
