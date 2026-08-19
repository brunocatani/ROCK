#pragma once

#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandGrabContactEvidence.h"

#include <array>
#include <vector>

namespace rock::hand_grab_detail
{
    struct FingerPadPublishDebug
    {
        std::array<RE::NiPoint3, 5> padProbeStart{};
        std::array<RE::NiPoint3, 5> padProbeEnd{};
        std::array<RE::NiPoint3, 5> padProbeHit{};
        std::array<std::uint8_t, 5> padProbeHitValid{};
        bool hasPadProbeDebug = false;
        std::array<RE::NiPoint3, 5> surfaceTarget{};
        std::array<std::uint8_t, 5> surfaceTargetValid{};
        bool hasSurfaceTargetDebug = false;
    };

    [[nodiscard]] FingerPadPublishDebug makeFingerPadPublishDebug(
        const grab_finger_pose_runtime::SolvedGrabFingerPose& pose,
        const std::array<grab_finger_pose_runtime::FingerPadSurfaceEvidence, 5>& padEvidence);
    [[nodiscard]] grab_finger_pose_runtime::GrabFingerPoseTargetSet buildRuntimeFingerPoseTargets(
        const RE::NiPoint3& seatPointWorld,
        const RE::NiPoint3& seatNormalWorld);
    [[nodiscard]] grab_finger_pose_runtime::GrabFingerPoseTargetSet buildRuntimePinchFingerPoseTargets(
        const RuntimePinchPocketCandidate& candidate);
    void applyPinchFingerPosePolicy(
        grab_finger_pose_runtime::SolvedGrabFingerPose& pose,
        const CanonicalGrabFrame& frame,
        float minFingerValue);
    void storeFingerPoseTargetsInGrabFrame(
        CanonicalGrabFrame& frame,
        const grab_finger_pose_runtime::GrabFingerPoseTargetSet& targets,
        const RE::NiTransform& objectWorldTransform);
    void storeGripSourceEvidence(
        CanonicalGrabFrame& frame,
        RE::NiAVObject* sourceNode,
        const RE::NiTransform& fallbackObjectWorld,
        const RE::NiPoint3& gripPointWorld,
        const RE::NiPoint3& gripNormalWorld,
        bool normalValid);
    [[nodiscard]] RE::NiTransform gripEvidenceWorldFrame(
        const CanonicalGrabFrame& frame,
        const RE::NiTransform& fallbackWorld);
    [[nodiscard]] RE::NiTransform gripEvidenceWorldFrame(
        const ImmutableGrabCaptureTelemetry& capture,
        const RE::NiTransform& fallbackWorld);
    [[nodiscard]] RE::NiPoint3 gripEvidencePointWorld(
        const CanonicalGrabFrame& frame,
        const RE::NiTransform& fallbackWorld);
    [[nodiscard]] RE::NiPoint3 gripEvidenceNormalWorld(
        const CanonicalGrabFrame& frame,
        const RE::NiTransform& fallbackWorld);
    [[nodiscard]] grab_finger_pose_runtime::GrabFingerPoseTargetSet rebuildFingerPoseTargetsFromGrabFrame(
        const CanonicalGrabFrame& frame,
        const RE::NiTransform& currentNodeWorld);
    [[nodiscard]] std::vector<TriangleData> rebuildFingerPoseWorldTrianglesFromGrabFrame(
        const CanonicalGrabFrame& frame,
        const RE::NiTransform& currentNodeWorld);
    void applyRockGrabHandPose(
        bool isLeft,
        const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
        std::array<float, 15>& currentJointPose,
        bool& hasCurrentJointPose,
        std::array<RE::NiTransform, 15>& currentLocalTransforms,
        std::uint16_t& currentLocalTransformMask,
        bool& hasCurrentLocalTransforms,
        float deltaTime,
        bool publishLocalTransforms = true,
        bool snapJointPoseToTarget = false);
    [[nodiscard]] grab_finger_pose_runtime::SolvedGrabFingerPose buildAcquisitionFingerPose(
        const grab_finger_pose_runtime::SolvedGrabFingerPose& targetPose,
        float acquisitionProgress);
}
