#include "physics-interaction/hand/grab/HandGrabFingerPose.h"

#include "physics-interaction/hand/grab/HandGrabMath.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/TransformMath.h"
#include "RockConfig.h"
#include "RockUtils.h"

#include <algorithm>
#include <cmath>

namespace rock::hand_grab_detail
{
    FingerPadPublishDebug makeFingerPadPublishDebug(
        const grab_finger_pose_runtime::SolvedGrabFingerPose& pose,
        const std::array<grab_finger_pose_runtime::FingerPadSurfaceEvidence, 5>& padEvidence)
    {
        FingerPadPublishDebug debug{};
        for (std::size_t finger = 0; finger < padEvidence.size(); ++finger) {
            const auto& evidence = padEvidence[finger];
            debug.padProbeStart[finger] = evidence.startWorld;
            debug.padProbeEnd[finger] = evidence.endWorld;
            debug.padProbeHit[finger] = evidence.hitPointWorld;
            debug.padProbeHitValid[finger] = evidence.hit ? 1 : 0;
            debug.hasPadProbeDebug = debug.hasPadProbeDebug || grab_finger_pose_runtime::hasFingerPadProbeLine(evidence);

            if (pose.surfaceAimTargetValid[finger]) {
                debug.surfaceTarget[finger] = pose.surfaceAimTarget[finger];
                debug.surfaceTargetValid[finger] = 1;
                debug.hasSurfaceTargetDebug = true;
            }
        }
        return debug;
    }

    void applyRockGrabHandPose(bool isLeft, const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose, std::array<float, 15>& currentJointPose,
        bool& hasCurrentJointPose, std::array<RE::NiTransform, 15>& currentLocalTransforms, std::uint16_t& currentLocalTransformMask, bool& hasCurrentLocalTransforms,
        float deltaTime, bool publishLocalTransforms, bool snapJointPoseToTarget)
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

        if (!g_rockConfig.rockGrabMeshFingerPoseEnabled) {
            hasCurrentJointPose = false;
            grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
            syncLocalTransformState();
            (void)frik_visual_authority::clearHandPose("ROCK_Grab", hand);
            return;
        }

        if (g_rockConfig.rockGrabMeshJointPoseEnabled && fingerPose.solved) {
            const auto targetJointPose = fingerPose.hasJointValues ? fingerPose.jointValues : grab_finger_pose_math::expandFingerCurlsToJointValues(fingerPose.values);
            if (!hasCurrentJointPose || snapJointPoseToTarget) {
                currentJointPose = targetJointPose;
                hasCurrentJointPose = true;
            } else {
                currentJointPose = grab_finger_pose_math::advanceJointValues(currentJointPose, targetJointPose, g_rockConfig.rockGrabFingerPoseSmoothingSpeed, deltaTime);
            }

            std::array<float, 5> currentSplayRadians{};
            (void)grab_finger_pose_runtime::resolveSurfaceContactSplayValues(isLeft, fingerPose, currentSplayRadians);
            const auto currentHandPose = frik_visual_authority::makeHandPoseDataFromJointValues(currentJointPose, currentSplayRadians);
            if (!frik_visual_authority::setHandPoseCustomWithPriority("ROCK_Grab", hand, currentHandPose, 100)) {
                hasCurrentJointPose = false;
                grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
                syncLocalTransformState();
                return;
            }
            const bool publishedLocalTransforms = grab_finger_local_transform_runtime::publishLocalTransformPose("ROCK_Grab",
                hand,
                isLeft,
                fingerPose,
                currentHandPose,
                grab_finger_local_transform_runtime::Options{
                    .enabled = publishLocalTransforms && g_rockConfig.rockGrabMeshLocalTransformPoseEnabled,
                    .smoothingSpeed = g_rockConfig.rockGrabFingerLocalTransformSmoothingSpeed,
                    .maxCorrectionDegrees = g_rockConfig.rockGrabFingerLocalTransformMaxCorrectionDegrees,
                    .surfaceAimStrength = g_rockConfig.rockGrabFingerSurfaceAimStrength,
                    .thumbOppositionStrength = g_rockConfig.rockGrabThumbOppositionStrength,
                    .thumbAlternateCurveStrength = g_rockConfig.rockGrabThumbAlternateCurveStrength,
                    .thumbSurfaceSafetyEnabled = g_rockConfig.rockGrabThumbSurfaceSafetyEnabled,
                    .thumbSurfaceSafetyMarginGameUnits = g_rockConfig.rockGrabThumbSurfaceSafetyMarginGameUnits,
                },
                deltaTime,
                100,
                localTransformState);
            syncLocalTransformState();
            if (g_rockConfig.rockDebugGrabFrameLogging) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand FINGER JOINT POSE: thumb=({:.2f},{:.2f},{:.2f}) index=({:.2f},{:.2f},{:.2f}) hits={} candidateTris={} altThumb={} thumbLane={} localTransforms={} mask=0x{:04X}",
                    isLeft ? "Left" : "Right", currentJointPose[0], currentJointPose[1], currentJointPose[2], currentJointPose[3], currentJointPose[4],
                    currentJointPose[5], fingerPose.hitCount, fingerPose.candidateTriangleCount, fingerPose.usedAlternateThumbCurve ? "yes" : "no",
                    grab_finger_pose_math::thumbLaneName(fingerPose.selectedThumbLane),
                    publishedLocalTransforms ? "yes" : "no", currentLocalTransformMask);
            }
            return;
        }

        if (fingerPose.solved) {
            hasCurrentJointPose = false;
            grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
            syncLocalTransformState();
            std::array<float, 5> currentSplayRadians{};
            (void)grab_finger_pose_runtime::resolveSurfaceContactSplayValues(isLeft, fingerPose, currentSplayRadians);
            const auto handPose = frik_visual_authority::makeUniformHandPoseData(
                fingerPose.values[0],
                fingerPose.values[1],
                fingerPose.values[2],
                fingerPose.values[3],
                fingerPose.values[4],
                currentSplayRadians);
            const bool published = frik_visual_authority::setHandPoseCustomWithPriority("ROCK_Grab", hand, handPose, 100);
            if (published && g_rockConfig.rockDebugGrabFrameLogging) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand FINGER POSE: mesh values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} candidateTris={} altThumb={} thumbLane={}",
                    isLeft ? "Left" : "Right", fingerPose.values[0], fingerPose.values[1], fingerPose.values[2], fingerPose.values[3], fingerPose.values[4],
                    fingerPose.hitCount, fingerPose.candidateTriangleCount, fingerPose.usedAlternateThumbCurve ? "yes" : "no",
                    grab_finger_pose_math::thumbLaneName(fingerPose.selectedThumbLane));
            }
            return;
        }

        hasCurrentJointPose = false;
        grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
        syncLocalTransformState();
        const float fallbackMin =
            std::clamp(std::isfinite(g_rockConfig.rockGrabFingerMinValue) ? g_rockConfig.rockGrabFingerMinValue : 0.2f, 0.0f, 1.0f);
        const float configuredFallback =
            std::isfinite(g_rockConfig.rockSelectedCloseFingerAnimValue) ? g_rockConfig.rockSelectedCloseFingerAnimValue : 0.9f;
        const float fallbackValue = std::clamp(configuredFallback, fallbackMin, 1.0f);
        const auto fallbackPose = frik_visual_authority::makeUniformHandPoseData(
            fallbackValue,
            fallbackValue,
            fallbackValue,
            fallbackValue,
            fallbackValue);
        (void)frik_visual_authority::setHandPoseCustomWithPriority("ROCK_Grab", hand, fallbackPose, 100);
        if (g_rockConfig.rockDebugGrabFrameLogging) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand FINGER POSE: using selected-close fallback value={:.2f} solved={} hits={} candidateTris={}",
                isLeft ? "Left" : "Right",
                fallbackValue,
                fingerPose.solved ? "yes" : "no",
                fingerPose.hitCount,
                fingerPose.candidateTriangleCount);
        }
    }

    grab_finger_pose_runtime::SolvedGrabFingerPose buildAcquisitionFingerPose(
        const grab_finger_pose_runtime::SolvedGrabFingerPose& targetPose,
        float progress)
    {
        auto pose = targetPose;
        const float t = std::clamp(std::isfinite(progress) ? progress : 0.0f, 0.0f, 1.0f);
        const float precloseValue =
            std::clamp(std::isfinite(g_rockConfig.rockSelectedCloseFingerAnimValue) ? g_rockConfig.rockSelectedCloseFingerAnimValue : 0.9f, 0.0f, 1.0f);

        for (std::size_t finger = 0; finger < pose.values.size(); ++finger) {
            const float solvedValue =
                targetPose.solved ? std::clamp(targetPose.values[finger], 0.0f, 1.0f) : precloseValue;
            const float closingTarget = (std::min)(precloseValue, solvedValue);
            pose.values[finger] = precloseValue + (closingTarget - precloseValue) * t;
        }

        if (targetPose.solved && targetPose.hasJointValues) {
            std::array<float, 5> precloseValues{ precloseValue, precloseValue, precloseValue, precloseValue, precloseValue };
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
    grab_finger_pose_runtime::GrabFingerPoseTargetSet buildRuntimeFingerPoseTargets(
        const RE::NiPoint3& seatPointWorld,
        const RE::NiPoint3& seatNormalWorld)
    {
        auto targets = grab_finger_pose_runtime::makeSharedGripPoseTarget(seatPointWorld, seatNormalWorld);
        targets.useSeatPointForMissingTargets = false;
        targets.useWholeMeshForMissingTargets = true;
        return targets;
    }
    
    grab_finger_pose_runtime::GrabFingerPoseTargetSet buildRuntimePinchFingerPoseTargets(
        const RuntimePinchPocketCandidate& candidate)
    {
        auto targets = grab_finger_pose_runtime::makeSharedGripPoseTarget(candidate.surfaceHit.position, candidate.surfaceHit.normal);
        targets.useSeatPointForMissingTargets = false;
        targets.useWholeMeshForMissingTargets = false;
    
        const auto config = currentPinchPocketConfig();
        const float halfWidth =
            grab_pinch_pocket_policy::oppositionHalfWidthGameUnits(candidate.meshExtents, config.surfaceInsetGameUnits);
        const RE::NiPoint3 thumbNormal{
            -candidate.pinchAxisWorld.x,
            -candidate.pinchAxisWorld.y,
            -candidate.pinchAxisWorld.z,
        };
        const RE::NiPoint3 indexNormal = candidate.pinchAxisWorld;
    
        targets.targets[0] = candidate.surfaceHit.position + thumbNormal * halfWidth;
        targets.targetNormals[0] = thumbNormal;
        targets.targetValid[0] = 1;
        targets.targetNormalValid[0] = 1;
        targets.targets[1] = candidate.surfaceHit.position + indexNormal * halfWidth;
        targets.targetNormals[1] = indexNormal;
        targets.targetValid[1] = 1;
        targets.targetNormalValid[1] = 1;
        targets.targetCount = 2;
        return targets;
    }
    
    void applyPinchFingerPosePolicy(
        grab_finger_pose_runtime::SolvedGrabFingerPose& pose,
        const CanonicalGrabFrame& frame,
        float minFingerValue)
    {
        const auto config = currentPinchPocketConfig();
        const auto stablePose = grab_pinch_pocket_policy::buildStablePinchFingerPose(config, minFingerValue);
    
        pose.values = stablePose.values;
        pose.usedAlternateThumbCurve = false;
        pose.usedAlternateThumbSurfaceHit = false;
        pose.selectedThumbLane = grab_finger_calibration_data::BakedGrabThumbLane::Wrap;
        pose.selectedThumbLaneNormalBlend = 0.0f;
        pose.selectedThumbLaneLocalCorrectionStrength = 0.0f;
        pose.hasThumbAlternateCurveFrame = false;
        pose.thumbAlternateCurveBaseWorld = {};
        pose.thumbAlternateCurveOpenDirectionWorld = {};
        pose.thumbAlternateCurveNormalWorld = {};
        pose.thumbAlternateCurveMaxCurlAngleRadians = 0.0f;
        pose.hasThumbCurveDiagnostics = false;
        pose.thumbPrimaryCurve = {};
        pose.thumbAlternateCurve = {};
        pose.thumbSidePadCurve = {};
        pose.poseTargetCount = (std::max)(pose.poseTargetCount, static_cast<int>(frame.fingerPoseTargetCount));
    
        for (std::size_t finger = 0; finger < 2 && finger < pose.surfaceAimTargetValid.size(); ++finger) {
            pose.surfaceAimTargetValid[finger] = 0;
            pose.surfaceAimNormalValid[finger] = 0;
        }
    
        pose.solved = true;
        pose.hasJointValues = true;
        pose.jointValues = stablePose.jointValues;
    }
    
    void storeFingerPoseTargetsInGrabFrame(CanonicalGrabFrame& frame,
        const grab_finger_pose_runtime::GrabFingerPoseTargetSet& targets,
        const RE::NiTransform& objectWorldTransform)
    {
        frame.fingerPoseTargetLocal = {};
        frame.fingerPoseTargetNormalLocal = {};
        frame.fingerPoseTargetValid = {};
        frame.fingerPoseTargetNormalValid = {};
        frame.fingerPoseTargetCount = 0;
        for (std::size_t finger = 0; finger < targets.targets.size(); ++finger) {
            if (!targets.targetValid[finger]) {
                continue;
            }
            frame.fingerPoseTargetLocal[finger] = transform_math::worldPointToLocal(objectWorldTransform, targets.targets[finger]);
            frame.fingerPoseTargetValid[finger] = 1;
            if (targets.targetNormalValid[finger]) {
                frame.fingerPoseTargetNormalLocal[finger] = transform_math::worldVectorToLocal(objectWorldTransform, targets.targetNormals[finger]);
                frame.fingerPoseTargetNormalValid[finger] = 1;
            }
            ++frame.fingerPoseTargetCount;
        }
    }
    
    void storeGripSourceEvidence(CanonicalGrabFrame& frame,
        RE::NiAVObject* sourceNode,
        const RE::NiTransform& fallbackObjectWorld,
        const RE::NiPoint3& gripPointWorld,
        const RE::NiPoint3& gripNormalWorld,
        bool normalValid)
    {
        /*
         * Mesh triangles choose a world-space position only. A rendered
         * triangle can live under a child node whose native X/Y/Z does not
         * match the collidable node or the hknp BODY, so its local point is
         * stored only as source-node evidence. Solver authority is the
         * separate BODY-local pivot B captured from the same world point.
         */
        const RE::NiTransform& evidenceWorld = sourceNode ? sourceNode->world : fallbackObjectWorld;
        frame.gripSourceNode = sourceNode;
        frame.gripSourceNodeWorldAtGrab = evidenceWorld;
        frame.gripPointSourceNodeLocal = transform_math::worldPointToLocal(evidenceWorld, gripPointWorld);
        frame.hasGripSourceNodePoint = true;
        if (normalValid) {
            frame.gripNormalSourceNodeLocal = transform_math::worldVectorToLocal(evidenceWorld, gripNormalWorld);
            frame.hasGripSourceNodeNormal = true;
        } else {
            frame.gripNormalSourceNodeLocal = {};
            frame.hasGripSourceNodeNormal = false;
        }
    }
    
    RE::NiTransform gripEvidenceWorldFrame(const CanonicalGrabFrame& frame, const RE::NiTransform& fallbackWorld)
    {
        if (frame.gripSourceNode) {
            return frame.gripSourceNode->world;
        }
        if (frame.hasGripSourceNodePoint) {
            return frame.gripSourceNodeWorldAtGrab;
        }
        return fallbackWorld;
    }
    
    RE::NiTransform gripEvidenceWorldFrame(const ImmutableGrabCaptureTelemetry& capture, const RE::NiTransform& fallbackWorld)
    {
        if (capture.gripSourceNode) {
            return capture.gripSourceNode->world;
        }
        if (capture.hasGripSourceNodePoint) {
            return capture.gripSourceNodeWorld;
        }
        return fallbackWorld;
    }
    
    RE::NiPoint3 gripEvidencePointWorld(const CanonicalGrabFrame& frame, const RE::NiTransform& fallbackWorld)
    {
        if (frame.hasGripSourceNodePoint) {
            return transform_math::localPointToWorld(gripEvidenceWorldFrame(frame, fallbackWorld), frame.gripPointSourceNodeLocal);
        }
        return transform_math::localPointToWorld(fallbackWorld, frame.gripPointLocal);
    }
    
    RE::NiPoint3 gripEvidenceNormalWorld(const CanonicalGrabFrame& frame, const RE::NiTransform& fallbackWorld)
    {
        if (frame.hasGripSourceNodeNormal) {
            return normalizeOrZero(transform_math::localVectorToWorld(gripEvidenceWorldFrame(frame, fallbackWorld), frame.gripNormalSourceNodeLocal));
        }
        return normalizeOrZero(transform_math::localVectorToWorld(fallbackWorld, frame.gripNormalLocal));
    }
    
    grab_finger_pose_runtime::GrabFingerPoseTargetSet rebuildFingerPoseTargetsFromGrabFrame(
        const CanonicalGrabFrame& frame,
        const RE::NiTransform& currentNodeWorld)
    {
        const RE::NiPoint3 seatNormalWorld = frame.hasGripPoint ? gripEvidenceNormalWorld(frame, currentNodeWorld) : RE::NiPoint3{};
        auto targets = grab_finger_pose_runtime::makeSharedGripPoseTarget(gripEvidencePointWorld(frame, currentNodeWorld), seatNormalWorld);
        targets.useSeatPointForMissingTargets = false;
        targets.useWholeMeshForMissingTargets = true;
        for (std::size_t finger = 0; finger < frame.fingerPoseTargetLocal.size(); ++finger) {
            if (!frame.fingerPoseTargetValid[finger]) {
                continue;
            }
            targets.targets[finger] = transform_math::localPointToWorld(currentNodeWorld, frame.fingerPoseTargetLocal[finger]);
            targets.targetValid[finger] = 1;
            if (frame.fingerPoseTargetNormalValid[finger]) {
                targets.targetNormals[finger] = normalizeOrZero(transform_math::localVectorToWorld(currentNodeWorld, frame.fingerPoseTargetNormalLocal[finger]));
                targets.targetNormalValid[finger] = 1;
            }
            ++targets.targetCount;
        }
        return targets;
    }
    
    std::vector<TriangleData> rebuildFingerPoseWorldTrianglesFromGrabFrame(
        const CanonicalGrabFrame& frame,
        const RE::NiTransform& currentNodeWorld)
    {
        std::vector<TriangleData> worldTriangles;
        const auto& localPoseTriangles = !frame.fingerPoseLocalMeshTriangles.empty() ?
            frame.fingerPoseLocalMeshTriangles :
            frame.localMeshTriangles;
        worldTriangles.reserve(localPoseTriangles.size());
        for (const auto& localTriangle : localPoseTriangles) {
            worldTriangles.push_back(TriangleData{
                transform_math::localPointToWorld(currentNodeWorld, localTriangle.v0),
                transform_math::localPointToWorld(currentNodeWorld, localTriangle.v1),
                transform_math::localPointToWorld(currentNodeWorld, localTriangle.v2),
            });
        }
        return worldTriangles;
    }
}
