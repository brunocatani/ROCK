#pragma once

#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/TransformMath.h"

#include <ROCK/FingerSweep.h>

#include <array>
#include <cmath>

/*
 * ROCK's adapter for the capsule finger sweep: ROCK's only mesh finger solve,
 * for loose-object grabs and weapon support grips.
 *
 * The game-independent math (FRIK forward model, capsule sweep, curl
 * expansion, sweep reach) is include/ROCK/FingerSweep.h, shared with the
 * offline tools so they show the pose ROCK computes. This file adds what
 * needs the game: FRIK rig capture, live skeleton snapshots, ROCK's mesh
 * indices and its finger pose record.
 *
 * Why the sweep: the baked-curve sweep it replaced reported pad contact while
 * the fingers FRIK drew for the chosen values sat 1-1.3 gu inside the mesh
 * (bench replays of typewriter, basketball and coffee cup grabs). Replays
 * (tests/GrabPoseReplayTests.cpp) left no index-pinky bone inside the mesh, at
 * 2-8x lower cost per solve.
 */
namespace rock::grab_finger_capsule_sweep
{
    using FrikFingerRig = finger_sweep::FingerRig<RE::NiTransform>;
    using FingerChainPose = finger_sweep::FingerChainPose<RE::NiPoint3>;
    using SweepReach = finger_sweep::SweepReach<RE::NiPoint3>;
    using finger_sweep::CapsuleRadii;
    using finger_sweep::FingerResult;
    using finger_sweep::jointValuesForCurl;
    using finger_sweep::Options;
    using finger_sweep::poseFinger;
    using finger_sweep::sweepReach;

    struct Result : finger_sweep::Result
    {
        grab_finger_pose_runtime::FingerPoseSpatialQueryStats stats{};
    };

    inline FrikFingerRig makeFrikFingerRig(const std::array<RE::NiTransform, 15>& closedLocal, const std::array<RE::NiTransform, 15>& openLocal)
    {
        return finger_sweep::makeFingerRig(closedLocal, openLocal);
    }

    // FRIK's authored closed (0) and open (1) finger local transforms, read from FRIK itself.
    inline bool captureFrikRigLocals(bool isLeft, std::array<RE::NiTransform, 15>& closedLocal, std::array<RE::NiTransform, 15>& openLocal)
    {
        frik_visual_authority::FingerLocalTransformOverride closed{};
        frik_visual_authority::FingerLocalTransformOverride open{};
        const auto hand = frik_visual_authority::handFromBool(isLeft);
        if (!frik_visual_authority::getHandPoseLocalTransformsForPose(hand, frik_visual_authority::makeUniformHandPoseData(0.0f, 0.0f, 0.0f, 0.0f, 0.0f), &closed) ||
            !frik_visual_authority::getHandPoseLocalTransformsForPose(hand, frik_visual_authority::makeUniformHandPoseData(1.0f, 1.0f, 1.0f, 1.0f, 1.0f), &open)) {
            return false;
        }
        const auto finite = [](const RE::NiTransform& value) {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(value.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return std::isfinite(value.translate.x) && std::isfinite(value.translate.y) && std::isfinite(value.translate.z) && std::isfinite(value.scale) &&
                value.scale > 0.0001f;
        };
        for (std::size_t bone = 0; bone < 15; ++bone) {
            if (!finite(closed.localTransforms[bone]) || !finite(open.localTransforms[bone])) {
                return false;
            }
            closedLocal[bone] = closed.localTransforms[bone];
            openLocal[bone] = open.localTransforms[bone];
        }
        return true;
    }

    inline bool resolveFrikFingerRig(bool isLeft, FrikFingerRig& rig)
    {
        std::array<RE::NiTransform, 15> closedLocal{};
        std::array<RE::NiTransform, 15> openLocal{};
        if (!captureFrikRigLocals(isLeft, closedLocal, openLocal)) {
            return false;
        }
        rig = makeFrikFingerRig(closedLocal, openLocal);
        return rig.valid;
    }

    /*
     * Moves a finger skeleton captured relative to sourceHandWorld onto
     * targetHandWorld. The skeleton is read relative to its own wrist, so this
     * places it at the hand pose being solved for (or at the drawn hand).
     */
    inline bool rebaseSkeleton(root_flattened_finger_skeleton_runtime::Snapshot& snapshot, const RE::NiTransform& sourceHandWorld,
        const RE::NiTransform& targetHandWorld)
    {
        if (!snapshot.valid || !std::isfinite(sourceHandWorld.scale) || std::abs(sourceHandWorld.scale) <= 0.000001f || !std::isfinite(targetHandWorld.scale) ||
            std::abs(targetHandWorld.scale) <= 0.000001f) {
            return false;
        }
        const auto moveVector = [&](const RE::NiPoint3& world) {
            const RE::NiPoint3 moved = transform_math::localVectorToWorld(targetHandWorld, transform_math::worldVectorToLocal(sourceHandWorld, world));
            const float length = std::sqrt(moved.x * moved.x + moved.y * moved.y + moved.z * moved.z);
            return length > 0.000001f ? moved * (1.0f / length) : RE::NiPoint3{};
        };
        for (auto& finger : snapshot.fingers) {
            if (!finger.valid) {
                return false;
            }
            for (auto& point : finger.points) {
                point = transform_math::localPointToWorld(targetHandWorld, transform_math::worldPointToLocal(sourceHandWorld, point));
            }
            if (finger.tipGeometryValid) {
                finger.tipSegmentCenterWorld =
                    transform_math::localPointToWorld(targetHandWorld, transform_math::worldPointToLocal(sourceHandWorld, finger.tipSegmentCenterWorld));
                finger.tipDirectionWorld = moveVector(finger.tipDirectionWorld);
            }
        }
        if (snapshot.palmNormalValid) {
            snapshot.palmNormalWorld = moveVector(snapshot.palmNormalWorld);
            if (snapshot.palmNormalWorld.x == 0.0f && snapshot.palmNormalWorld.y == 0.0f && snapshot.palmNormalWorld.z == 0.0f) {
                return false;
            }
        }
        return true;
    }

    /*
     * The hand bone frame the fingers hang from. Its rotation is the solve
     * hand's (bench replay: residual 0.01-0.04 gu with rotation unchanged);
     * its position is recovered from the live finger bases and FRIK's rest
     * offsets, so the chains hang exactly where the captured skeleton puts
     * the knuckles.
     */
    inline RE::NiTransform resolveHandBoneWorld(const RE::NiTransform& solveHandWorld, const root_flattened_finger_skeleton_runtime::Snapshot& skeleton,
        const FrikFingerRig& rig)
    {
        std::array<RE::NiPoint3, 5> knuckles{};
        std::array<bool, 5> valid{};
        for (std::size_t finger = 0; finger < 5; ++finger) {
            valid[finger] = skeleton.fingers[finger].valid;
            knuckles[finger] = skeleton.fingers[finger].points[0];
        }
        return finger_sweep::handBoneFromKnuckles(solveHandWorld, rig, knuckles, valid);
    }

    template <class Index>
    Result solve(const FrikFingerRig& rig, const RE::NiTransform& handBoneWorld, const std::array<float, 5>& tipLengths, const Index& index,
        const RE::NiTransform& objectWorld, const Options& options)
    {
        Result result{};
        static_cast<finger_sweep::Result&>(result) = finger_sweep::solve(rig, handBoneWorld, tipLengths,
            [&](const RE::NiPoint3& center, float radius) { return index.querySphereWorld(objectWorld, center, radius, nullptr, nullptr, &result.stats); }, options);
        return result;
    }

    // Distal tip length per finger from a live skeleton: twice the tip-segment centre offset.
    inline std::array<float, 5> tipLengths(const root_flattened_finger_skeleton_runtime::Snapshot& skeleton)
    {
        std::array<float, 5> out{};
        for (std::size_t finger = 0; finger < 5; ++finger) {
            const auto& chain = skeleton.fingers[finger];
            if (chain.valid && chain.tipGeometryValid) {
                const RE::NiPoint3 offset = chain.tipSegmentCenterWorld - chain.points[2];
                out[finger] = 2.0f * std::sqrt(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z);
            }
        }
        return out;
    }

    /*
     * The grab-time solve: hand bone recovered from the live skeleton placed at the
     * solve hand, capsules swept against the object's mesh index in its frozen
     * relation to that hand.
     */
    template <class Index>
    Result solveForHand(const FrikFingerRig& rig, const RE::NiTransform& solveHandWorld, const root_flattened_finger_skeleton_runtime::Snapshot& skeleton,
        const Index& index, const RE::NiTransform& objectWorld, const Options& options)
    {
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::GrabFingerSolve);
        return solve(rig, resolveHandBoneWorld(solveHandWorld, skeleton, rig), tipLengths(skeleton), index, objectWorld, options);
    }

    // ROCK's shared finger pose record for a capsule result.
    inline grab_finger_pose_runtime::SolvedGrabFingerPose toSolvedPose(const Result& result, std::uint32_t triangleCount)
    {
        grab_finger_pose_runtime::SolvedGrabFingerPose pose{};
        pose.values = result.values;
        pose.jointValues = result.jointValues;
        pose.hasJointValues = true;
        pose.solved = true;
        pose.usedSpatialIndex = true;
        pose.usedLiveRootFlattenedFingerBones = true;
        pose.candidateTriangleCount = static_cast<int>(triangleCount);
        pose.spatialNodeVisitCount = result.stats.nodeVisits;
        pose.spatialTriangleTestCount = result.stats.triangleTests;
        for (std::size_t finger = 0; finger < 5; ++finger) {
            const bool contact = result.fingers[finger].contact;
            if (contact) {
                ++pose.hitCount;
                pose.contactValidMask |= static_cast<std::uint8_t>(1u << finger);
            }
        }
        return pose;
    }
}
