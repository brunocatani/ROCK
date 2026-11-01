#pragma once

#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <array>
#include <cmath>

/*
 * Capsule finger sweep on FRIK's own finger kinematics: ROCK's only mesh
 * finger solve, for loose-object grabs and weapon support grips.
 *
 * Why: the baked-curve sweep it replaced reported pad contact while the
 * fingers FRIK drew for the chosen values sat 1-1.3 gu inside the mesh
 * (bench replays of typewriter, basketball and coffee cup grabs). The baked
 * rows approximated the finger; FRIK's open/closed local transforms are the
 * finger. Replays (tests/GrabPoseReplayTests.cpp) left no index-pinky bone
 * inside the mesh, at 2-8x lower cost per solve.
 *
 * Model. Each finger's three bones are posed exactly as FRIK poses them:
 * local rotation = slerp(closed -> open, jointValue) with the interpolant
 * unclamped (values above 1 bend back), local translation = authored rest,
 * world = parent * local from the hand bone. Joint values come from the curl
 * through expandFingerCurlsToJointValues, the same expansion ROCK publishes.
 * Each bone is a capsule (sampled as overlapping spheres) with the hand
 * collider radius for its segment, so solve and physical hand agree on
 * thickness.
 *
 * Decision per finger. From the authored open pose (curl 1): if it is clear,
 * close in fixed steps and keep the last clear curl before any capsule
 * touches the mesh (closing fully to minValue on a miss). If the open pose
 * already intersects, open past 1 only as far as needed to clear, up to the
 * cap; a finger that cannot clear stays at the cap and is reported.
 */
namespace rock::grab_finger_capsule_sweep
{
    struct Quat
    {
        float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
    };

    inline Quat quatFromMatrix(const RE::NiMatrix3& m)
    {
        const auto& e = m.entry;
        const float trace = e[0][0] + e[1][1] + e[2][2];
        if (trace > 0.0f) {
            const float s = std::sqrt(trace + 1.0f) * 2.0f;
            return { 0.25f * s, (e[2][1] - e[1][2]) / s, (e[0][2] - e[2][0]) / s, (e[1][0] - e[0][1]) / s };
        }
        if (e[0][0] > e[1][1] && e[0][0] > e[2][2]) {
            const float s = std::sqrt(1.0f + e[0][0] - e[1][1] - e[2][2]) * 2.0f;
            return { (e[2][1] - e[1][2]) / s, 0.25f * s, (e[0][1] + e[1][0]) / s, (e[0][2] + e[2][0]) / s };
        }
        if (e[1][1] > e[2][2]) {
            const float s = std::sqrt(1.0f + e[1][1] - e[0][0] - e[2][2]) * 2.0f;
            return { (e[0][2] - e[2][0]) / s, (e[0][1] + e[1][0]) / s, 0.25f * s, (e[1][2] + e[2][1]) / s };
        }
        const float s = std::sqrt(1.0f + e[2][2] - e[0][0] - e[1][1]) * 2.0f;
        return { (e[1][0] - e[0][1]) / s, (e[0][2] + e[2][0]) / s, (e[1][2] + e[2][1]) / s, 0.25f * s };
    }

    inline RE::NiMatrix3 matrixFromQuat(const Quat& q)
    {
        RE::NiMatrix3 m{};
        const float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z, wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
        m.entry[0][0] = 1 - 2 * (yy + zz);
        m.entry[0][1] = 2 * (xy - wz);
        m.entry[0][2] = 2 * (xz + wy);
        m.entry[1][0] = 2 * (xy + wz);
        m.entry[1][1] = 1 - 2 * (xx + zz);
        m.entry[1][2] = 2 * (yz - wx);
        m.entry[2][0] = 2 * (xz - wy);
        m.entry[2][1] = 2 * (yz + wx);
        m.entry[2][2] = 1 - 2 * (xx + yy);
        return m;
    }

    // FRIK common Quaternion::slerp: shortest arc, no clamp on t (extrapolates past the target).
    inline Quat slerpUnclamped(Quat from, const Quat& to, float t)
    {
        float d = from.w * to.w + from.x * to.x + from.y * to.y + from.z * to.z;
        if (d < 0.0f) {
            from = { -from.w, -from.x, -from.y, -from.z };
            d = -d;
        }
        if (d > 0.999995f) {
            return from;
        }
        const float theta0 = std::acos(d);
        const float theta = theta0 * t;
        const float s0 = std::cos(theta) - d * std::sin(theta) / std::sin(theta0);
        const float s1 = std::sin(theta) / std::sin(theta0);
        return { s0 * from.w + s1 * to.w, s0 * from.x + s1 * to.x, s0 * from.y + s1 * to.y, s0 * from.z + s1 * to.z };
    }

    // FRIK's per-bone pose for a joint value, from its authored closed and open local transforms.
    struct FrikFingerRig
    {
        std::array<Quat, 15> closed{};
        std::array<Quat, 15> open{};
        std::array<RE::NiPoint3, 15> restTranslate{};
        std::array<float, 15> restScale{};
        bool valid = false;

        [[nodiscard]] RE::NiTransform boneLocal(std::size_t bone, float jointValue) const
        {
            RE::NiTransform out{};
            out.rotate = matrixFromQuat(slerpUnclamped(closed[bone], open[bone], std::clamp(jointValue, -1.0f, 2.0f)));
            out.translate = restTranslate[bone];
            out.scale = restScale[bone];
            return out;
        }
    };

    inline FrikFingerRig makeFrikFingerRig(const std::array<RE::NiTransform, 15>& closedLocal, const std::array<RE::NiTransform, 15>& openLocal)
    {
        FrikFingerRig rig{};
        for (std::size_t bone = 0; bone < 15; ++bone) {
            rig.closed[bone] = quatFromMatrix(closedLocal[bone].rotate);
            rig.open[bone] = quatFromMatrix(openLocal[bone].rotate);
            rig.restTranslate[bone] = openLocal[bone].translate;
            rig.restScale[bone] = openLocal[bone].scale;
        }
        rig.valid = true;
        return rig;
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

    struct FingerChainPose
    {
        std::array<RE::NiPoint3, 4> joints{}; // three bone origins, then the tip end
    };

    inline FingerChainPose poseFinger(const FrikFingerRig& rig, const RE::NiTransform& handBoneWorld, std::size_t finger, const std::array<float, 3>& jointValues,
        float tipLengthGameUnits)
    {
        FingerChainPose out{};
        RE::NiTransform parent = handBoneWorld;
        for (std::size_t segment = 0; segment < 3; ++segment) {
            parent = transform_math::composeTransforms(parent, rig.boneLocal(finger * 3 + segment, jointValues[segment]));
            out.joints[segment] = parent.translate;
        }
        out.joints[3] = parent.translate + transform_math::localVectorToWorld(parent, RE::NiPoint3{ tipLengthGameUnits, 0.0f, 0.0f });
        return out;
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
        RE::NiTransform out = solveHandWorld;
        RE::NiPoint3 sum{};
        int count = 0;
        for (std::size_t finger = 0; finger < 5; ++finger) {
            if (!skeleton.fingers[finger].valid) {
                continue;
            }
            const RE::NiPoint3 restWorldOffset = transform_math::localVectorToWorld(solveHandWorld, rig.restTranslate[finger * 3]);
            sum = sum + (skeleton.fingers[finger].points[0] - restWorldOffset);
            ++count;
        }
        if (count > 0) {
            out.translate = sum * (1.0f / static_cast<float>(count));
        }
        return out;
    }

    struct CapsuleRadii
    {
        // Hand collider radius + convex radius per segment (HandBoneColliderSet roleRadius/roleConvexRadius).
        std::array<float, 3> segment{ 0.65f, 0.56f, 0.48f };
    };

    struct Options
    {
        float minValue = 0.2f;
        float fingerMaxOpenValue = 2.0f;
        float thumbMaxOpenValue = 1.5f;
        float step = 0.05f;
        CapsuleRadii radii{};
    };

    struct FingerResult
    {
        float value = 1.0f;
        bool contact = false;     // closed onto the surface (or opened to clear it)
        bool openedToClear = false;
        bool blocked = false;      // could not clear even at the cap
    };

    struct Result
    {
        std::array<FingerResult, 5> fingers{};
        std::array<float, 5> values{};
        std::array<float, 15> jointValues{};
        grab_finger_pose_runtime::FingerPoseSpatialQueryStats stats{};
        std::uint32_t sphereQueries = 0;
    };

    inline std::array<float, 3> jointValuesForCurl(std::size_t finger, float curl)
    {
        std::array<float, 5> curls{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
        curls[finger] = curl;
        const auto joints = grab_finger_pose_math::expandFingerCurlsToJointValues(curls);
        return { joints[finger * 3], joints[finger * 3 + 1], joints[finger * 3 + 2] };
    }

    template <class Index>
    bool fingerIntersects(const FingerChainPose& pose, const Index& index, const RE::NiTransform& objectWorld, const CapsuleRadii& radii,
        grab_finger_pose_runtime::FingerPoseSpatialQueryStats& stats, std::uint32_t& sphereQueries)
    {
        for (std::size_t segment = 0; segment < 3; ++segment) {
            const RE::NiPoint3 a = pose.joints[segment];
            const RE::NiPoint3 b = pose.joints[segment + 1];
            const float radius = radii.segment[segment];
            const RE::NiPoint3 ab = b - a;
            const float length = std::sqrt(ab.x * ab.x + ab.y * ab.y + ab.z * ab.z);
            // Spheres spaced at most one radius apart cover the capsule to within ~13% of the radius.
            const int samples = (std::max)(2, static_cast<int>(std::ceil(length / radius)) + 1);
            for (int i = 0; i < samples; ++i) {
                const RE::NiPoint3 center = a + ab * (static_cast<float>(i) / static_cast<float>(samples - 1));
                ++sphereQueries;
                if (index.querySphereWorld(objectWorld, center, radius, nullptr, nullptr, &stats)) {
                    return true;
                }
            }
        }
        return false;
    }

    template <class Index>
    Result solve(const FrikFingerRig& rig, const RE::NiTransform& handBoneWorld, const std::array<float, 5>& tipLengths, const Index& index,
        const RE::NiTransform& objectWorld, const Options& options)
    {
        Result result{};
        const float step = (std::max)(0.01f, options.step);
        for (std::size_t finger = 0; finger < 5; ++finger) {
            auto& out = result.fingers[finger];
            const float cap = finger == 0 ? options.thumbMaxOpenValue : options.fingerMaxOpenValue;
            const auto intersects = [&](float curl) {
                return fingerIntersects(poseFinger(rig, handBoneWorld, finger, jointValuesForCurl(finger, curl), tipLengths[finger]), index, objectWorld, options.radii,
                    result.stats, result.sphereQueries);
            };
            if (!intersects(1.0f)) {
                out.value = options.minValue;
                for (float curl = 1.0f - step; curl >= options.minValue - 0.0001f; curl -= step) {
                    if (intersects(curl)) {
                        out.value = curl + step;
                        out.contact = true;
                        break;
                    }
                }
            } else {
                out.value = cap;
                out.blocked = true;
                for (float curl = 1.0f + step; curl <= cap + 0.0001f; curl += step) {
                    if (!intersects(curl)) {
                        out.value = curl;
                        out.contact = true;
                        out.openedToClear = true;
                        out.blocked = false;
                        break;
                    }
                }
            }
            result.values[finger] = out.value;
        }
        result.jointValues = grab_finger_pose_math::expandFingerCurlsToJointValues(result.values);
        return result;
    }

    /*
     * World points the finger can occupy across its sweep (base, then the
     * three joint ends at open, half and fully closed). Callers use them to
     * pick the mesh triangles worth indexing before the solve.
     */
    template <std::size_t Capacity>
    inline std::size_t sweepReferencePoints(const FrikFingerRig& rig, const RE::NiTransform& handBoneWorld, std::size_t finger, float tipLengthGameUnits, float minValue,
        std::array<RE::NiPoint3, Capacity>& out)
    {
        static_assert(Capacity >= 10);
        std::size_t count = 0;
        for (const float curl : { 1.0f, 0.6f, minValue }) {
            const auto pose = poseFinger(rig, handBoneWorld, finger, jointValuesForCurl(finger, curl), tipLengthGameUnits);
            if (count == 0) {
                out[count++] = pose.joints[0];
            }
            for (std::size_t joint = 1; joint < 4; ++joint) {
                out[count++] = pose.joints[joint];
            }
        }
        return count;
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
