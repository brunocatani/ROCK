#pragma once

#include <ROCK/TransformMath.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

/*
 * Capsule finger sweep on FRIK's finger kinematics: the game-independent
 * math of ROCK's mesh finger solve, shared with the offline tools (Reload
 * Maker, Offset Maker) so a pose they show is the pose ROCK computes.
 *
 * Not a provider interface: no table, version or ABI. Types are the caller's,
 * read the way rock::transform_math reads them (Transform{rotate, translate,
 * scale}, Matrix{entry[3][3]} with Ni's stored basis, Vector{x, y, z}), so
 * RE::NiTransform and any plain struct of that shape both work. Arithmetic is
 * float in a fixed order; ROCK's bench replays reproduce their recorded
 * solves exactly.
 *
 * Model. Each finger's three bones are posed exactly as FRIK poses them:
 * local rotation = slerp(closed -> open, jointValue), interpolant clamped to
 * [-1, 2] (values above 1 bend back); local translation = authored rest;
 * world = parent * local from the hand bone. Joint values come from the curl
 * through expandCurlsToJointValues, the same expansion ROCK publishes. Each
 * bone is a capsule, sampled as overlapping spheres, with the hand collider
 * radius for its segment.
 *
 * Decision per finger. From the authored open pose (curl 1): if it is clear,
 * close in fixed steps and keep the last clear curl before any capsule
 * touches the mesh (closing fully to minValue on a miss). If the open pose
 * already intersects, open past 1 only as far as needed to clear, up to the
 * cap; a finger that cannot clear stays at the cap and is reported blocked.
 */
namespace rock::finger_sweep
{
    inline constexpr std::size_t kFingerCount = 5;
    inline constexpr std::size_t kBoneCount = 15;
    inline constexpr float kMaxFingerOpenValue = 1.0f;
    inline constexpr float kMaxOverOpenValue = 2.0f;

    namespace detail
    {
        template <class Vector>
        inline Vector add(const Vector& a, const Vector& b)
        {
            Vector out{};
            out.x = a.x + b.x;
            out.y = a.y + b.y;
            out.z = a.z + b.z;
            return out;
        }

        template <class Vector>
        inline Vector subtract(const Vector& a, const Vector& b)
        {
            Vector out{};
            out.x = a.x - b.x;
            out.y = a.y - b.y;
            out.z = a.z - b.z;
            return out;
        }

        template <class Vector>
        inline Vector scale(const Vector& a, float s)
        {
            Vector out{};
            out.x = a.x * s;
            out.y = a.y * s;
            out.z = a.z * s;
            return out;
        }

        template <class Vector>
        inline float length(const Vector& a)
        {
            return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        }
    }

    struct Quat
    {
        float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
    };

    template <class Matrix>
    inline Quat quatFromMatrix(const Matrix& m)
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

    template <class Matrix>
    inline Matrix matrixFromQuat(const Quat& q)
    {
        Matrix m{};
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

    [[nodiscard]] inline float clampOpenValue(float value)
    {
        return std::clamp(std::isfinite(value) ? value : kMaxFingerOpenValue, 0.0f, kMaxOverOpenValue);
    }

    /*
     * Curl per finger (thumb first) to joint values per bone. The proximal and
     * distal biases scale with how closed the finger is, so they vanish past 1
     * and over-open joints hyper-extend uniformly.
     */
    inline std::array<float, kBoneCount> expandCurlsToJointValues(const std::array<float, kFingerCount>& values)
    {
        std::array<float, kBoneCount> joints{};
        for (std::size_t finger = 0; finger < values.size(); ++finger) {
            const float value = clampOpenValue(values[finger]);
            const float closed = kMaxFingerOpenValue - (std::min)(value, kMaxFingerOpenValue);
            const float proximalOpenBias = (finger == 0) ? 0.15f : 0.25f;
            const float distalCloseBias = (finger == 0) ? 0.10f : 0.15f;
            const std::size_t base = finger * 3;
            joints[base + 0] = std::clamp(value + closed * proximalOpenBias, 0.0f, kMaxOverOpenValue);
            joints[base + 1] = value;
            joints[base + 2] = std::clamp(value - closed * distalCloseBias, 0.0f, kMaxOverOpenValue);
        }
        return joints;
    }

    inline std::array<float, 3> jointValuesForCurl(std::size_t finger, float curl)
    {
        std::array<float, kFingerCount> curls{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
        curls[finger] = curl;
        const auto joints = expandCurlsToJointValues(curls);
        return { joints[finger * 3], joints[finger * 3 + 1], joints[finger * 3 + 2] };
    }

    // FRIK's per-bone pose for a joint value, from its authored closed (0) and open (1) local transforms.
    template <class Transform>
    struct FingerRig
    {
        using Matrix = decltype(Transform{}.rotate);
        using Vector = decltype(Transform{}.translate);

        std::array<Quat, kBoneCount> closed{};
        std::array<Quat, kBoneCount> open{};
        std::array<Vector, kBoneCount> restTranslate{};
        std::array<float, kBoneCount> restScale{};
        bool valid = false;

        [[nodiscard]] Transform boneLocal(std::size_t bone, float jointValue) const
        {
            Transform out{};
            out.rotate = matrixFromQuat<Matrix>(slerpUnclamped(closed[bone], open[bone], std::clamp(jointValue, -1.0f, 2.0f)));
            out.translate = restTranslate[bone];
            out.scale = restScale[bone];
            return out;
        }
    };

    template <class Transform>
    inline FingerRig<Transform> makeFingerRig(const std::array<Transform, kBoneCount>& closedLocal, const std::array<Transform, kBoneCount>& openLocal)
    {
        FingerRig<Transform> rig{};
        for (std::size_t bone = 0; bone < kBoneCount; ++bone) {
            rig.closed[bone] = quatFromMatrix(closedLocal[bone].rotate);
            rig.open[bone] = quatFromMatrix(openLocal[bone].rotate);
            rig.restTranslate[bone] = openLocal[bone].translate;
            rig.restScale[bone] = openLocal[bone].scale;
        }
        rig.valid = true;
        return rig;
    }

    template <class Vector>
    struct FingerChainPose
    {
        std::array<Vector, 4> joints{}; // three bone origins, then the tip end
    };

    template <class Transform>
    inline auto poseFinger(const FingerRig<Transform>& rig, const Transform& handBoneWorld, std::size_t finger, const std::array<float, 3>& jointValues,
        float tipLengthGameUnits)
    {
        using Vector = typename FingerRig<Transform>::Vector;
        FingerChainPose<Vector> out{};
        Transform parent = handBoneWorld;
        for (std::size_t segment = 0; segment < 3; ++segment) {
            parent = transform_math::composeTransforms(parent, rig.boneLocal(finger * 3 + segment, jointValues[segment]));
            out.joints[segment] = parent.translate;
        }
        Vector tip{};
        tip.x = tipLengthGameUnits;
        out.joints[3] = detail::add(parent.translate, transform_math::localVectorToWorld(parent, tip));
        return out;
    }

    /*
     * The hand bone frame the fingers hang from: the solve hand's rotation, at
     * the position the knuckles (each finger's first bone origin) imply through
     * FRIK's rest offsets. Fingers with valid[finger] false are skipped; with
     * none valid the solve hand is returned unchanged.
     */
    template <class Transform, class Vector>
    inline Transform handBoneFromKnuckles(const Transform& solveHandWorld, const FingerRig<Transform>& rig, const std::array<Vector, kFingerCount>& knuckles,
        const std::array<bool, kFingerCount>& valid)
    {
        Transform out = solveHandWorld;
        Vector sum{};
        int count = 0;
        for (std::size_t finger = 0; finger < kFingerCount; ++finger) {
            if (!valid[finger]) {
                continue;
            }
            const Vector restWorldOffset = transform_math::localVectorToWorld(solveHandWorld, rig.restTranslate[finger * 3]);
            sum = detail::add(sum, detail::subtract(knuckles[finger], restWorldOffset));
            ++count;
        }
        if (count > 0) {
            out.translate = detail::scale(sum, 1.0f / static_cast<float>(count));
        }
        return out;
    }

    struct CapsuleRadii
    {
        // Hand collider radius + convex radius per segment (ROCK HandBoneColliderSet roleRadius/roleConvexRadius).
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
        bool contact = false; // closed onto the surface (or opened to clear it)
        bool openedToClear = false;
        bool blocked = false; // could not clear even at the cap
    };

    struct Result
    {
        std::array<FingerResult, kFingerCount> fingers{};
        std::array<float, kFingerCount> values{};
        std::array<float, kBoneCount> jointValues{};
        std::uint32_t sphereQueries = 0;
    };

    /*
     * sphereHits(center, radius) -> bool answers whether any mesh triangle lies
     * within radius of center (world space). It is the only mesh access.
     */
    template <class Vector, class SphereHits>
    bool fingerIntersects(const FingerChainPose<Vector>& pose, SphereHits&& sphereHits, const CapsuleRadii& radii, std::uint32_t& sphereQueries)
    {
        for (std::size_t segment = 0; segment < 3; ++segment) {
            const Vector a = pose.joints[segment];
            const Vector b = pose.joints[segment + 1];
            const float radius = radii.segment[segment];
            const Vector ab = detail::subtract(b, a);
            const float length = detail::length(ab);
            // Spheres spaced at most one radius apart cover the capsule to within ~13% of the radius.
            const int samples = (std::max)(2, static_cast<int>(std::ceil(length / radius)) + 1);
            for (int i = 0; i < samples; ++i) {
                const Vector center = detail::add(a, detail::scale(ab, static_cast<float>(i) / static_cast<float>(samples - 1)));
                ++sphereQueries;
                if (sphereHits(center, radius)) {
                    return true;
                }
            }
        }
        return false;
    }

    template <class Transform, class SphereHits>
    Result solve(const FingerRig<Transform>& rig, const Transform& handBoneWorld, const std::array<float, kFingerCount>& tipLengths, SphereHits&& sphereHits,
        const Options& options)
    {
        Result result{};
        const float step = (std::max)(0.01f, options.step);
        for (std::size_t finger = 0; finger < kFingerCount; ++finger) {
            auto& out = result.fingers[finger];
            const float cap = finger == 0 ? options.thumbMaxOpenValue : options.fingerMaxOpenValue;
            const auto intersects = [&](float curl) {
                return fingerIntersects(poseFinger(rig, handBoneWorld, finger, jointValuesForCurl(finger, curl), tipLengths[finger]), sphereHits, options.radii,
                    result.sphereQueries);
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
        result.jointValues = expandCurlsToJointValues(result.values);
        return result;
    }

    template <class Vector>
    struct SweepReach
    {
        Vector center{};
        float radius = 0.0f;
    };

    /*
     * Sphere that holds every capsule of the sweep, centred on the grasp
     * pocket (the centroid of the finger joints at half curl, where a held
     * surface sits). Its radius covers the joints at the closed, middle, open
     * and bent-back limits plus the widest capsule and a margin for the arcs
     * between those samples. Mesh outside it can never touch a finger, so
     * callers can skip mesh whose bounds miss it.
     */
    template <class Transform>
    inline auto sweepReach(const FingerRig<Transform>& rig, const Transform& handBoneWorld, const std::array<float, kFingerCount>& tipLengthsGameUnits,
        const Options& options, const CapsuleRadii& radii = {})
    {
        using Vector = typename FingerRig<Transform>::Vector;
        SweepReach<Vector> out{};
        int count = 0;
        for (std::size_t finger = 0; finger < kFingerCount; ++finger) {
            for (const auto& joint : poseFinger(rig, handBoneWorld, finger, jointValuesForCurl(finger, 0.5f), tipLengthsGameUnits[finger]).joints) {
                out.center = detail::add(out.center, joint);
                ++count;
            }
        }
        out.center = detail::scale(out.center, 1.0f / static_cast<float>(count));
        for (std::size_t finger = 0; finger < kFingerCount; ++finger) {
            const float cap = finger == 0 ? options.thumbMaxOpenValue : options.fingerMaxOpenValue;
            for (const float curl : { options.minValue, 0.5f, 1.0f, cap }) {
                for (const auto& joint : poseFinger(rig, handBoneWorld, finger, jointValuesForCurl(finger, curl), tipLengthsGameUnits[finger]).joints) {
                    out.radius = (std::max)(out.radius, detail::length(detail::subtract(joint, out.center)));
                }
            }
        }
        out.radius += (std::max)({ radii.segment[0], radii.segment[1], radii.segment[2] }) + 1.0f;
        return out;
    }
}
