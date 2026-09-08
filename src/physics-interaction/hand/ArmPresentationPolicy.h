#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string_view>

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/hand/RenderedBoneTransportPolicy.h"
#include "physics-interaction/hand/TrackedHandIsolationPolicy.h"

#include "RE/NetImmerse/NiTransform.h"

/*
 * Arm carry for the end-of-frame hand presentation under FRIK API v2.
 *
 * FRIK solved the arm to the claim it consumed; ROCK then moves the hand by
 * the change it made to that claim. Carrying only the hand chain would leave
 * the elbow where FRIK put it, with the forearm's elbow end off by the
 * change and FRIK's upper arm wobbling with the rebase residual. Instead the
 * arm is re-solved from FRIK's result: the shoulder stays, the elbow is
 * placed by the two bone lengths in FRIK's bend plane (law of cosines with
 * FRIK's elbow as the pole hint), the upper arm turns about the shoulder to
 * the new elbow, the forearm turns about the new elbow to the carried wrist,
 * and the hand and fingers take the hand delta exactly. Out of reach the arm
 * straightens and the wrist joint absorbs the rest.
 */
namespace rock::arm_presentation_policy
{
    using HandChainSide = rendered_bone_transport_policy::HandChainSide;

    enum class ArmSegment : std::uint8_t
    {
        None,
        // UpperArm and its twist bones: turned about the shoulder.
        UpperArm,
        // ForeArm1..3: turned about the elbow.
        Forearm,
        // Hand and fingers: the hand delta itself.
        Hand,
    };

    // A bone shorter than this is not an arm segment to solve with.
    inline constexpr double kMinBoneLengthGameUnits = 0.5;

    [[nodiscard]] inline HandChainSide armSideForBone(const std::string_view name) noexcept
    {
        if (name.starts_with("RArm_")) {
            return HandChainSide::Right;
        }
        if (name.starts_with("LArm_")) {
            return HandChainSide::Left;
        }
        return HandChainSide::None;
    }

    [[nodiscard]] inline ArmSegment armSegmentForBone(const std::string_view name) noexcept
    {
        if (armSideForBone(name) == HandChainSide::None) {
            return ArmSegment::None;
        }
        const std::string_view tail = name.substr(5);
        if (tail == "UpperArm" || tail == "UpperTwist1" || tail == "UpperTwist2") {
            return ArmSegment::UpperArm;
        }
        if (tail == "ForeArm1" || tail == "ForeArm2" || tail == "ForeArm3") {
            return ArmSegment::Forearm;
        }
        if (tail == "Hand" || tail.starts_with("Finger")) {
            return ArmSegment::Hand;
        }
        return ArmSegment::None;
    }

    struct ArmCarry
    {
        // World rigid transforms, applied as composeTransforms(transform, boneWorld).
        RE::NiTransform upperArm{};
        RE::NiTransform forearm{};
        RE::NiTransform hand{};
        float elbowMoveGameUnits = 0.0f;
        // How far the wrist lies beyond the straight arm (the wrist joint absorbs it).
        float reachDeficitGameUnits = 0.0f;
        bool valid = false;
    };

    namespace detail
    {
        struct Vec
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        struct Mat
        {
            // World rotation: v' = m * v (row-major, m[row][column]).
            double m[3][3]{ { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } };
        };

        [[nodiscard]] inline Vec toVec(const RE::NiPoint3& point) noexcept
        {
            return Vec{ point.x, point.y, point.z };
        }

        [[nodiscard]] inline Vec add(const Vec& a, const Vec& b) noexcept { return Vec{ a.x + b.x, a.y + b.y, a.z + b.z }; }
        [[nodiscard]] inline Vec sub(const Vec& a, const Vec& b) noexcept { return Vec{ a.x - b.x, a.y - b.y, a.z - b.z }; }
        [[nodiscard]] inline Vec scale(const Vec& a, const double s) noexcept { return Vec{ a.x * s, a.y * s, a.z * s }; }
        [[nodiscard]] inline double dot(const Vec& a, const Vec& b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }
        [[nodiscard]] inline Vec cross(const Vec& a, const Vec& b) noexcept
        {
            return Vec{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
        }
        [[nodiscard]] inline double length(const Vec& a) noexcept { return std::sqrt(dot(a, a)); }
        [[nodiscard]] inline bool finite(const Vec& a) noexcept { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }

        [[nodiscard]] inline Vec apply(const Mat& m, const Vec& v) noexcept
        {
            return Vec{
                m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z,
                m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z,
                m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z,
            };
        }

        // Rodrigues rotation about a unit axis.
        [[nodiscard]] inline Mat rotationAboutAxis(const Vec& k, const double cosine, const double sine) noexcept
        {
            const double t = 1.0 - cosine;
            Mat r{};
            r.m[0][0] = cosine + k.x * k.x * t;
            r.m[0][1] = k.x * k.y * t - k.z * sine;
            r.m[0][2] = k.x * k.z * t + k.y * sine;
            r.m[1][0] = k.y * k.x * t + k.z * sine;
            r.m[1][1] = cosine + k.y * k.y * t;
            r.m[1][2] = k.y * k.z * t - k.x * sine;
            r.m[2][0] = k.z * k.x * t - k.y * sine;
            r.m[2][1] = k.z * k.y * t + k.x * sine;
            r.m[2][2] = cosine + k.z * k.z * t;
            return r;
        }

        // The smallest rotation that turns direction a onto direction b.
        [[nodiscard]] inline Mat rotationFromTo(const Vec& a, const Vec& b) noexcept
        {
            const double la = length(a);
            const double lb = length(b);
            if (!(la > 1e-9) || !(lb > 1e-9)) {
                return Mat{};
            }
            const Vec an = scale(a, 1.0 / la);
            const Vec bn = scale(b, 1.0 / lb);
            const double cosine = std::clamp(dot(an, bn), -1.0, 1.0);
            const Vec axis = cross(an, bn);
            const double sine = length(axis);
            if (sine > 1e-9) {
                return rotationAboutAxis(scale(axis, 1.0 / sine), cosine, sine);
            }
            if (cosine > 0.0) {
                return Mat{};
            }
            // Antiparallel: half a turn about any axis perpendicular to a.
            const Vec helper = std::abs(an.x) < 0.9 ? Vec{ 1.0, 0.0, 0.0 } : Vec{ 0.0, 1.0, 0.0 };
            Vec perpendicular = cross(an, helper);
            perpendicular = scale(perpendicular, 1.0 / length(perpendicular));
            return rotationAboutAxis(perpendicular, -1.0, 0.0);
        }

        /*
         * T(x) = to + m * (x - from), in the engine's stored form: the stored
         * rows are the local axes in world, so row k is column k of m.
         */
        [[nodiscard]] inline RE::NiTransform rigidAboutPivot(const Mat& m, const Vec& from, const Vec& to) noexcept
        {
            RE::NiTransform transform = transform_math::makeIdentityTransform<RE::NiTransform>();
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    transform.rotate.entry[row][column] = static_cast<float>(m.m[column][row]);
                }
            }
            const Vec rotatedFrom = apply(m, from);
            transform.translate = RE::NiPoint3(
                static_cast<float>(to.x - rotatedFrom.x),
                static_cast<float>(to.y - rotatedFrom.y),
                static_cast<float>(to.z - rotatedFrom.z));
            return transform;
        }
    }

    /*
     * shoulder, elbow, wrist: the origins FRIK rendered (upper arm, forearm 1,
     * hand). handDelta: the world rigid transform the hand takes.
     */
    [[nodiscard]] inline ArmCarry planArmCarry(
        const RE::NiPoint3& shoulder,
        const RE::NiPoint3& elbow,
        const RE::NiPoint3& wrist,
        const RE::NiTransform& handDelta) noexcept
    {
        using namespace detail;
        ArmCarry carry{};
        if (!tracked_hand_isolation_policy::isFiniteTransform(handDelta)) {
            return carry;
        }
        const Vec s = toVec(shoulder);
        const Vec e = toVec(elbow);
        const Vec w = toVec(wrist);
        const Vec wn = toVec(transform_math::localPointToWorld(handDelta, wrist));
        if (!finite(s) || !finite(e) || !finite(w) || !finite(wn)) {
            return carry;
        }
        const Vec upper = sub(e, s);
        const Vec fore = sub(w, e);
        const Vec reach = sub(w, s);
        const Vec reachNew = sub(wn, s);
        const double a = length(upper);
        const double b = length(fore);
        const double c = length(reachNew);
        if (!(a > kMinBoneLengthGameUnits) || !(b > kMinBoneLengthGameUnits) ||
            !(length(reach) > kMinBoneLengthGameUnits) || !(c > kMinBoneLengthGameUnits)) {
            return carry;
        }

        // FRIK's bend plane: the old arm re-aimed onto the new reach axis.
        const Vec axis = scale(reachNew, 1.0 / c);
        const Vec elbowReaimed = apply(rotationFromTo(reach, reachNew), upper);
        const Vec pole = sub(elbowReaimed, scale(axis, dot(elbowReaimed, axis)));
        const double poleLength = length(pole);

        const double cosine = std::clamp((a * a + c * c - b * b) / (2.0 * a * c), -1.0, 1.0);
        const double sine = std::sqrt((std::max)(0.0, 1.0 - cosine * cosine));
        Vec elbowNew = add(s, scale(axis, a * cosine));
        if (poleLength > kMinBoneLengthGameUnits * 0.01) {
            elbowNew = add(elbowNew, scale(pole, a * sine / poleLength));
        }

        carry.upperArm = rigidAboutPivot(rotationFromTo(upper, sub(elbowNew, s)), s, s);
        carry.forearm = rigidAboutPivot(rotationFromTo(fore, sub(wn, elbowNew)), e, elbowNew);
        carry.hand = transform_math::orthonormalizedTransform(handDelta);
        carry.elbowMoveGameUnits = static_cast<float>(length(sub(elbowNew, e)));
        carry.reachDeficitGameUnits = static_cast<float>((std::max)(0.0, c - (a + b)));
        carry.valid =
            tracked_hand_isolation_policy::isFiniteTransform(carry.upperArm) &&
            tracked_hand_isolation_policy::isFiniteTransform(carry.forearm) &&
            tracked_hand_isolation_policy::isFiniteTransform(carry.hand) &&
            std::isfinite(carry.elbowMoveGameUnits) &&
            std::isfinite(carry.reachDeficitGameUnits);
        return carry;
    }

    [[nodiscard]] inline const RE::NiTransform& carryForSegment(const ArmCarry& carry, const ArmSegment segment) noexcept
    {
        switch (segment) {
        case ArmSegment::UpperArm:
            return carry.upperArm;
        case ArmSegment::Forearm:
            return carry.forearm;
        default:
            return carry.hand;
        }
    }
}
