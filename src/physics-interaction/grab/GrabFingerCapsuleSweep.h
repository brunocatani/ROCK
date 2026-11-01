#pragma once

#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/TransformMath.h"

#include <algorithm>
#include <array>
#include <cmath>

/*
 * Capsule finger sweep on FRIK's own finger kinematics (prototype under
 * offline evaluation, see tests/GrabPoseReplayTests.cpp).
 *
 * Problem it addresses: the baked-curve sweep reports pad contact while the
 * fingers FRIK draws for the chosen values sit 1-1.3 gu inside the mesh
 * (bench replay of typewriter and basketball pulls). The baked rows are an
 * approximation of the finger; FRIK's open/closed local transforms are the
 * finger.
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
     * offsets, which absorbs the fixed offset between ROCK's hand frame and
     * FRIK's hand bone (4.2 gu measured).
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
}
