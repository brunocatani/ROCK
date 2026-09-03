#include "physics-interaction/hand/ArmPresentationPolicy.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
    using namespace rock::arm_presentation_policy;
    namespace isolation = rock::tracked_hand_isolation_policy;

    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        if (std::fabs(actual - expected) <= epsilon) {
            return true;
        }
        std::printf("%s expected %.5f got %.5f\n", label, expected, actual);
        return false;
    }

    template <class Enum>
    bool expectEnum(const char* label, Enum actual, Enum expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %d got %d\n", label, static_cast<int>(expected), static_cast<int>(actual));
        return false;
    }

    RE::NiTransform identity()
    {
        return rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    }

    RE::NiTransform translated(float x, float y, float z)
    {
        RE::NiTransform t = identity();
        t.translate = RE::NiPoint3(x, y, z);
        return t;
    }

    RE::NiTransform yawed(float degrees, float x, float y, float z)
    {
        const float radians = degrees * 0.017453292519943295f;
        const float c = std::cos(radians);
        const float s = std::sin(radians);
        RE::NiTransform t = identity();
        t.rotate.entry[0][0] = c;
        t.rotate.entry[0][1] = -s;
        t.rotate.entry[1][0] = s;
        t.rotate.entry[1][1] = c;
        t.translate = RE::NiPoint3(x, y, z);
        return t;
    }

    RE::NiPoint3 moved(const RE::NiTransform& transform, const RE::NiPoint3& point)
    {
        return rock::transform_math::localPointToWorld(transform, point);
    }

    float distance(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        const float x = a.x - b.x;
        const float y = a.y - b.y;
        const float z = a.z - b.z;
        return std::sqrt(x * x + y * y + z * z);
    }

    bool orthonormal(const RE::NiTransform& transform)
    {
        return rock::transform_math::storedRotationOrthonormalityError(transform.rotate) < 1e-5;
    }
}

int main()
{
    bool ok = true;

    // Segment and side by name.
    ok &= expectEnum("upper arm", armSegmentForBone("RArm_UpperArm"), ArmSegment::UpperArm);
    ok &= expectEnum("upper twist", armSegmentForBone("LArm_UpperTwist2"), ArmSegment::UpperArm);
    ok &= expectEnum("forearm", armSegmentForBone("RArm_ForeArm2"), ArmSegment::Forearm);
    ok &= expectEnum("hand", armSegmentForBone("LArm_Hand"), ArmSegment::Hand);
    ok &= expectEnum("finger", armSegmentForBone("RArm_Finger31"), ArmSegment::Hand);
    ok &= expectEnum("collarbone stays", armSegmentForBone("RArm_Collarbone"), ArmSegment::None);
    ok &= expectEnum("spine", armSegmentForBone("SPINE2"), ArmSegment::None);
    ok &= expectEnum("right side", armSideForBone("RArm_UpperTwist1"), HandChainSide::Right);
    ok &= expectEnum("left side", armSideForBone("LArm_Finger11"), HandChainSide::Left);
    ok &= expectEnum("no side", armSideForBone("Pelvis"), HandChainSide::None);

    // A bent arm: shoulder at the origin, 25 gu upper arm, 25 gu forearm.
    const RE::NiPoint3 shoulder(0.0f, 0.0f, 0.0f);
    const RE::NiPoint3 elbow(15.0f, 0.0f, 20.0f);
    const RE::NiPoint3 wrist(30.0f, 0.0f, 0.0f);

    // No change: everything stays.
    {
        const ArmCarry carry = planArmCarry(shoulder, elbow, wrist, identity());
        ok &= expectTrue("identity valid", carry.valid);
        ok &= expectNear("identity elbow stays", carry.elbowMoveGameUnits, 0.0f, 1e-4f);
        ok &= expectNear("identity upper arm", distance(moved(carry.upperArm, elbow), elbow), 0.0f, 1e-3f);
        ok &= expectNear("identity forearm", distance(moved(carry.forearm, wrist), wrist), 0.0f, 1e-3f);
    }

    // The wrist moves 2 gu outward and turns 5 deg: bone lengths hold, the
    // elbow stays in the bend plane on the same side, the wrist lands exactly.
    {
        const RE::NiTransform delta = yawed(5.0f, 2.0f, 0.0f, 0.0f);
        const RE::NiPoint3 wristNew = moved(delta, wrist);
        const ArmCarry carry = planArmCarry(shoulder, elbow, wrist, delta);
        ok &= expectTrue("carry valid", carry.valid);
        const RE::NiPoint3 elbowNew = moved(carry.upperArm, elbow);
        ok &= expectNear("upper arm length", distance(elbowNew, shoulder), 25.0f, 0.01f);
        ok &= expectNear("forearm length", distance(moved(carry.forearm, wrist), elbowNew), 25.0f, 0.01f);
        ok &= expectNear("forearm reaches the wrist", distance(moved(carry.forearm, wrist), wristNew), 0.0f, 0.01f);
        ok &= expectNear("forearm elbow end", distance(moved(carry.forearm, elbow), elbowNew), 0.0f, 0.01f);
        // The bend plane holds the new reach axis and the old bend direction (+Z).
        const RE::NiPoint3 planeNormal(-wristNew.y, wristNew.x, 0.0f);
        const float normalLength = std::sqrt(planeNormal.x * planeNormal.x + planeNormal.y * planeNormal.y);
        ok &= expectNear("bend plane kept", (elbowNew.x * planeNormal.x + elbowNew.y * planeNormal.y) / normalLength, 0.0f, 0.02f);
        ok &= expectTrue("bend side kept", elbowNew.z > 10.0f);
        ok &= expectNear("elbow move reported", carry.elbowMoveGameUnits, distance(elbowNew, elbow), 0.01f);
        ok &= expectTrue("elbow moves less than the wrist", carry.elbowMoveGameUnits < 2.0f);
        ok &= expectTrue("upper orthonormal", orthonormal(carry.upperArm));
        ok &= expectTrue("forearm orthonormal", orthonormal(carry.forearm));
        ok &= expectTrue("hand orthonormal", orthonormal(carry.hand));
        ok &= expectNear("shoulder stays", distance(moved(carry.upperArm, shoulder), shoulder), 0.0f, 1e-3f);
    }

    // A pure wrist turn: nothing but the hand changes.
    {
        const RE::NiTransform turn = yawed(8.0f, 0.0f, 0.0f, 0.0f);
        RE::NiTransform aboutWrist = turn;
        // Turn about the wrist itself: T(x) = wrist + R (x - wrist).
        const RE::NiPoint3 rotatedWrist = moved(turn, wrist);
        aboutWrist.translate = RE::NiPoint3(wrist.x - rotatedWrist.x, wrist.y - rotatedWrist.y, wrist.z - rotatedWrist.z);
        const ArmCarry carry = planArmCarry(shoulder, elbow, wrist, aboutWrist);
        ok &= expectTrue("turn valid", carry.valid);
        ok &= expectNear("turn keeps the elbow", carry.elbowMoveGameUnits, 0.0f, 0.01f);
        ok &= expectNear("turn keeps the wrist", distance(moved(carry.forearm, wrist), wrist), 0.0f, 0.01f);
    }

    // Out of reach: the arm straightens toward the wrist, lengths still hold.
    {
        const RE::NiTransform far = translated(30.0f, 0.0f, 0.0f);
        const ArmCarry carry = planArmCarry(shoulder, elbow, wrist, far);
        ok &= expectTrue("far valid", carry.valid);
        const RE::NiPoint3 elbowNew = moved(carry.upperArm, elbow);
        ok &= expectNear("far upper arm length", distance(elbowNew, shoulder), 25.0f, 0.01f);
        ok &= expectNear("far straight", elbowNew.z, 0.0f, 0.05f);
        ok &= expectNear("far forearm length", distance(moved(carry.forearm, wrist), elbowNew), 25.0f, 0.01f);
    }

    // Degenerate input is refused.
    {
        ok &= expectTrue("zero upper arm", !planArmCarry(shoulder, shoulder, wrist, translated(1.0f, 0.0f, 0.0f)).valid);
        RE::NiTransform broken = identity();
        broken.translate.x = std::numeric_limits<float>::quiet_NaN();
        ok &= expectTrue("non-finite delta", !planArmCarry(shoulder, elbow, wrist, broken).valid);
    }

    if (!ok) {
        std::printf("ArmPresentationPolicyTests FAILED\n");
        return 1;
    }
    std::printf("ArmPresentationPolicyTests passed\n");
    return 0;
}
