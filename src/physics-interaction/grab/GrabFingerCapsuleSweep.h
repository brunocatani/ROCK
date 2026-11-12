#pragma once

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/GrabFinger.h"

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
 * Decision per finger. A finger never bends past its authored open pose
 * (curl 1): a hand bent backwards reads as broken, a finger clipping the
 * surface in its natural pose reads as holding (bench sessions: a quarter of
 * the grabs had a finger opened past 1 to clear, a tenth past 1.3). From the
 * open pose: if it is clear, close in fixed steps and keep the last clear
 * curl before any capsule touches the mesh (closing fully to minValue on a
 * miss). If the open pose already intersects, the object is across the open
 * finger (a handle the hand reached into, a bar along the finger backs):
 * close inward until the finger clears what it was inside, then on until it
 * touches again, and keep that last clear curl, the finger closed onto the
 * inner surface. A finger that never clears inward, or clears and then closes
 * to minValue without touching anything, keeps the open pose and is reported
 * blocked: FRIK moves a finger only along its closed-open arc, and when the
 * object fills that arc (a palm on a large or deep object) there is no clear
 * pose on it.
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

    inline FrikFingerRig makeFrikFingerRig(const std::array<RE::NiTransform, 15>& closedLocal,
        const std::array<RE::NiTransform, 15>& openLocal)
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
    inline bool captureFrikRigLocals(bool isLeft, std::array<RE::NiTransform, 15>& closedLocal,
        std::array<RE::NiTransform, 15>& openLocal)
    {
        frik_visual_authority::FingerLocalTransformOverride closed{};
        frik_visual_authority::FingerLocalTransformOverride open{};
        const auto hand = frik_visual_authority::handFromBool(isLeft);
        if (!frik_visual_authority::getHandPoseLocalTransformsForPose(hand,
                frik_visual_authority::makeUniformHandPoseData(0.0f, 0.0f, 0.0f, 0.0f, 0.0f), &closed) ||
            !frik_visual_authority::getHandPoseLocalTransformsForPose(hand,
                frik_visual_authority::makeUniformHandPoseData(1.0f, 1.0f, 1.0f, 1.0f, 1.0f), &open)) {
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
            return std::isfinite(value.translate.x) && std::isfinite(value.translate.y) &&
                std::isfinite(value.translate.z) && std::isfinite(value.scale) && value.scale > 0.0001f;
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
    inline bool rebaseSkeleton(root_flattened_finger_skeleton_runtime::Snapshot& snapshot,
        const RE::NiTransform& sourceHandWorld, const RE::NiTransform& targetHandWorld)
    {
        if (!snapshot.valid || !std::isfinite(sourceHandWorld.scale) || std::abs(sourceHandWorld.scale) <= 0.000001f ||
            !std::isfinite(targetHandWorld.scale) || std::abs(targetHandWorld.scale) <= 0.000001f) {
            return false;
        }
        const auto moveVector = [&](const RE::NiPoint3& world) {
            const RE::NiPoint3 moved = transform_math::localVectorToWorld(targetHandWorld,
                transform_math::worldVectorToLocal(sourceHandWorld, world));
            const float length = std::sqrt(moved.x * moved.x + moved.y * moved.y + moved.z * moved.z);
            return length > 0.000001f ? moved * (1.0f / length) : RE::NiPoint3{};
        };
        for (auto& finger : snapshot.fingers) {
            if (!finger.valid) {
                return false;
            }
            for (auto& point : finger.points) {
                point = transform_math::localPointToWorld(targetHandWorld,
                    transform_math::worldPointToLocal(sourceHandWorld, point));
            }
            if (finger.tipGeometryValid) {
                finger.tipSegmentCenterWorld = transform_math::localPointToWorld(targetHandWorld,
                    transform_math::worldPointToLocal(sourceHandWorld, finger.tipSegmentCenterWorld));
                finger.tipDirectionWorld = moveVector(finger.tipDirectionWorld);
            }
        }
        if (snapshot.palmNormalValid) {
            snapshot.palmNormalWorld = moveVector(snapshot.palmNormalWorld);
            if (snapshot.palmNormalWorld.x == 0.0f && snapshot.palmNormalWorld.y == 0.0f &&
                snapshot.palmNormalWorld.z == 0.0f) {
                return false;
            }
        }
        return true;
    }

    struct FingerChainPose
    {
        std::array<RE::NiPoint3, 4> joints{}; // three bone origins, then the tip end
    };

    inline FingerChainPose poseFinger(const FrikFingerRig& rig, const RE::NiTransform& handBoneWorld,
        std::size_t finger, const std::array<float, 3>& jointValues, float tipLengthGameUnits)
    {
        FingerChainPose out{};
        RE::NiTransform parent = handBoneWorld;
        for (std::size_t segment = 0; segment < 3; ++segment) {
            parent =
                transform_math::composeTransforms(parent, rig.boneLocal(finger * 3 + segment, jointValues[segment]));
            out.joints[segment] = parent.translate;
        }
        out.joints[3] = parent.translate +
            transform_math::localVectorToWorld(parent, RE::NiPoint3{ tipLengthGameUnits, 0.0f, 0.0f });
        return out;
    }

    /*
     * The hand bone frame the fingers hang from. Its rotation is the solve
     * hand's (bench replay: residual 0.01-0.04 gu with rotation unchanged);
     * its position is recovered from the live finger bases and FRIK's rest
     * offsets, so the chains hang exactly where the captured skeleton puts
     * the knuckles.
     */
    inline RE::NiTransform resolveHandBoneWorld(const RE::NiTransform& solveHandWorld,
        const root_flattened_finger_skeleton_runtime::Snapshot& skeleton, const FrikFingerRig& rig)
    {
        RE::NiTransform out = solveHandWorld;
        RE::NiPoint3 sum{};
        int count = 0;
        for (std::size_t finger = 0; finger < 5; ++finger) {
            if (!skeleton.fingers[finger].valid) {
                continue;
            }
            const RE::NiPoint3 restWorldOffset =
                transform_math::localVectorToWorld(solveHandWorld, rig.restTranslate[finger * 3]);
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
        float step = 0.05f;
        CapsuleRadii radii{};
    };

    struct FingerResult
    {
        float value = 1.0f;
        bool contact = false; // closed onto the surface
        bool closedPastIntersection = false; // the open pose intersected; closed inward past it onto the surface
        bool blocked = false; // the open pose intersected and no inward pose clears and touches; kept open
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

    /*
     * The spheres that stand for one finger's capsules. The solve and the probe
     * visualizer both walk this, so what is drawn is exactly what is tested.
     * visit(center, radius, segment) returns true to stop; the result says
     * whether a visit stopped the walk.
     */
    template <class Visit>
    bool forEachCapsuleSphere(const FingerChainPose& pose, const CapsuleRadii& radii, Visit&& visit)
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
                if (visit(center, radius, segment)) {
                    return true;
                }
            }
        }
        return false;
    }

    template <class Index>
    bool fingerIntersects(const FingerChainPose& pose, const Index& index, const RE::NiTransform& objectWorld,
        const CapsuleRadii& radii, grab_finger_pose_runtime::FingerPoseSpatialQueryStats& stats,
        std::uint32_t& sphereQueries)
    {
        return forEachCapsuleSphere(pose, radii, [&](const RE::NiPoint3& center, float radius, std::size_t) {
            ++sphereQueries;
            return index.querySphereWorld(objectWorld, center, radius, nullptr, nullptr, &stats);
        });
    }

    template <class Index>
    Result solve(const FrikFingerRig& rig, const RE::NiTransform& handBoneWorld, const std::array<float, 5>& tipLengths,
        const Index& index, const RE::NiTransform& objectWorld, const Options& options)
    {
        Result result{};
        const float step = (std::max)(0.01f, options.step);
        for (std::size_t finger = 0; finger < 5; ++finger) {
            auto& out = result.fingers[finger];
            const auto intersects = [&](float curl) {
                return fingerIntersects(
                    poseFinger(rig, handBoneWorld, finger, jointValuesForCurl(finger, curl), tipLengths[finger]), index,
                    objectWorld, options.radii, result.stats, result.sphereQueries);
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
                out.value = 1.0f;
                out.blocked = true;
                bool cleared = false;
                for (float curl = 1.0f - step; curl >= options.minValue - 0.0001f; curl -= step) {
                    const bool hit = intersects(curl);
                    if (!cleared) {
                        cleared = !hit;
                        continue;
                    }
                    if (hit) {
                        out.value = curl + step;
                        out.contact = true;
                        out.closedPastIntersection = true;
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
    Result solveForHand(const FrikFingerRig& rig, const RE::NiTransform& solveHandWorld,
        const root_flattened_finger_skeleton_runtime::Snapshot& skeleton, const Index& index,
        const RE::NiTransform& objectWorld, const Options& options)
    {
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::GrabFingerSolve);
        return solve(rig, resolveHandBoneWorld(solveHandWorld, skeleton, rig), tipLengths(skeleton), index, objectWorld,
            options);
    }

    /*
     * Pinch reach. A pinch leaves the object where the hand found it, so
     * the fingers have to come to the object. The sweep and the pinch
     * closure both move a finger along one arc (all three joints from one
     * curl value); an object smaller than the fingers, such as a single
     * cartridge, rarely lies on both arcs, and thumb and index closed on
     * empty space beside it while it hung off the fingertip (log of a
     * .38 round: thumb 1.6 gu and index 1.0 gu from the mesh).
     *
     * Thumb and index here bend their three joints separately: the pose
     * whose tip end is nearest the mesh while no capsule of the finger
     * enters it. The spread term keeps the joints near one another so the
     * finger still reads as one curl. The result replaces the arc pose only
     * when the tip ends up measurably nearer the mesh, or when the arc pose
     * was inside it. Capture-time only.
     */
    struct PinchReach
    {
        float beforeGameUnits = -1.0f;
        float afterGameUnits = -1.0f;
        bool applied = false;
    };

    inline PinchReach reachPinchFinger(grab_finger_pose_runtime::SolvedGrabFingerPose& pose, std::size_t finger,
        const FrikFingerRig& rig, const RE::NiTransform& handBone, float tipLength,
        const grab_finger_pose_runtime::FingerPoseTriangleSpatialIndex& index, const RE::NiTransform& objectWorld,
        const Options& options, float minimumJointValue)
    {
        // A tip farther than this from the mesh is not pinching this object.
        constexpr float kReachGameUnits = 8.0f;
        constexpr float kSpreadCostGameUnits = 0.3f;
        constexpr float kMinimumGainGameUnits = 0.15f;
        constexpr int kSteps = 8;

        PinchReach reach{};
        grab_finger_pose_runtime::FingerPoseSpatialQueryStats stats{};
        std::uint32_t sphereQueries = 0;
        const float minimum = std::clamp(minimumJointValue, 0.0f, 1.0f);

        const auto tipDistance = [&](const FingerChainPose& chain, float within, float& distance) {
            RE::NiPoint3 nearest{};
            if (!(within > 0.0f) ||
                !index.querySphereWorld(objectWorld, chain.joints[3], within, &nearest, nullptr, &stats)) {
                return false;
            }
            const RE::NiPoint3 delta = nearest - chain.joints[3];
            distance = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
            return std::isfinite(distance);
        };
        const auto spread = [](const std::array<float, 3>& joints) {
            return (std::max)({ joints[0], joints[1], joints[2] }) - (std::min)({ joints[0], joints[1], joints[2] });
        };
        const auto intersects = [&](const FingerChainPose& chain) {
            return fingerIntersects(chain, index, objectWorld, options.radii, stats, sphereQueries);
        };

        std::array<float, 3> best = pose.hasJointValues
            ? std::array<float, 3>{ pose.jointValues[finger * 3], pose.jointValues[finger * 3 + 1],
                  pose.jointValues[finger * 3 + 2] }
            : jointValuesForCurl(finger, pose.values[finger]);
        const auto current = poseFinger(rig, handBone, finger, best, tipLength);
        if (!tipDistance(current, kReachGameUnits, reach.beforeGameUnits)) {
            return reach;
        }
        const bool currentInside = intersects(current);
        float bestDistance = reach.beforeGameUnits;
        float bestCost = currentInside ? kReachGameUnits * 2.0f
                                       : reach.beforeGameUnits + kSpreadCostGameUnits * spread(best);
        bool found = false;

        const auto consider = [&](const std::array<float, 3>& joints) {
            const float spreadCost = kSpreadCostGameUnits * spread(joints);
            const auto chain = poseFinger(rig, handBone, finger, joints, tipLength);
            float distance = 0.0f;
            if (!tipDistance(chain, bestCost - spreadCost, distance) || distance + spreadCost >= bestCost ||
                intersects(chain)) {
                return;
            }
            best = joints;
            bestDistance = distance;
            bestCost = distance + spreadCost;
            found = true;
        };
        const auto search = [&](const std::array<float, 3>& center, float halfRange) {
            std::array<float, 3> joints{};
            for (int a = 0; a <= kSteps; ++a) {
                joints[0] = std::clamp(center[0] + halfRange * (2.0f * a / kSteps - 1.0f), minimum, 1.0f);
                for (int b = 0; b <= kSteps; ++b) {
                    joints[1] = std::clamp(center[1] + halfRange * (2.0f * b / kSteps - 1.0f), minimum, 1.0f);
                    for (int c = 0; c <= kSteps; ++c) {
                        joints[2] = std::clamp(center[2] + halfRange * (2.0f * c / kSteps - 1.0f), minimum, 1.0f);
                        consider(joints);
                    }
                }
            }
        };
        const float middle = (1.0f + minimum) * 0.5f;
        const float half = (1.0f - minimum) * 0.5f;
        search({ middle, middle, middle }, half);
        if (found) {
            search(best, half / kSteps);
        }

        reach.afterGameUnits = bestDistance;
        if (!found || (!currentInside && bestDistance > reach.beforeGameUnits - kMinimumGainGameUnits)) {
            reach.afterGameUnits = reach.beforeGameUnits;
            return reach;
        }
        for (std::size_t segment = 0; segment < 3; ++segment) {
            pose.jointValues[finger * 3 + segment] = best[segment];
        }
        pose.hasJointValues = true;
        pose.values[finger] = (best[0] + best[1] + best[2]) / 3.0f;
        // On the surface: the tip capsule's own radius plus a sweep step's slack.
        if (bestDistance <= options.radii.segment[2] + 0.35f && (pose.contactValidMask & (1u << finger)) == 0) {
            pose.contactValidMask |= static_cast<std::uint8_t>(1u << finger);
            ++pose.hitCount;
        }
        reach.applied = true;
        return reach;
    }

    // ROCK's shared finger pose record for a capsule result.
    inline grab_finger_pose_runtime::SolvedGrabFingerPose toSolvedPose(const Result& result,
        std::uint32_t triangleCount)
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
