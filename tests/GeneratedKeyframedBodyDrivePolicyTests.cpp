#include "RE/Havok/hknpCollisionQueryCollector.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiPoint.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace
{
    constexpr float kPi = 3.14159265358979323846f;

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        const float delta = std::fabs(actual - expected);
        if (delta <= epsilon) {
            return true;
        }

        std::printf("%s expected %.5f got %.5f\n", label, expected, actual);
        return false;
    }

    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }

        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }

        std::printf("%s expected false\n", label);
        return false;
    }

    RE::NiTransform identityTransform()
    {
        return rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    }

    RE::NiMatrix3 rotationAroundZ(float radians)
    {
        const float halfAngle = radians * 0.5f;
        const float quaternion[4]{ 0.0f, 0.0f, std::sin(halfAngle), std::cos(halfAngle) };
        return rock::transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(quaternion);
    }

    float pointDistance(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        const float dx = rhs.x - lhs.x;
        const float dy = rhs.y - lhs.y;
        const float dz = rhs.z - lhs.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    float rotationDistanceRadians(const RE::NiMatrix3& lhs, const RE::NiMatrix3& rhs)
    {
        return rock::generated_keyframed_body_drive_math::rotationAngleRadians(lhs, rhs);
    }
}

int main()
{
    using namespace rock::generated_keyframed_body_drive_math;

    bool ok = true;
    constexpr float gameToHavokScale = 0.1f;
    constexpr float driveDeltaSeconds = 0.1f;

    {
        RE::NiTransform from = identityTransform();
        RE::NiTransform requested = identityTransform();
        requested.translate = RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        requested.rotate = rotationAroundZ(0.1f);

        const auto limited = limitGeneratedDriveTarget(from, requested, driveDeltaSeconds, gameToHavokScale, 2.0f, 2.0f);

        ok &= expectFalse("below-cap target has no linear cap", limited.limit.linearLimitExceeded);
        ok &= expectFalse("below-cap target has no angular cap", limited.limit.angularLimitExceeded);
        ok &= expectNear("below-cap alpha", limited.limit.alpha, 1.0f, 0.001f);
        ok &= expectNear("below-cap translation unchanged", pointDistance(limited.target.translate, requested.translate), 0.0f, 0.001f);
        ok &= expectNear("below-cap rotation unchanged", rotationDistanceRadians(limited.target.rotate, requested.rotate), 0.0f, 0.001f);
    }

    {
        RE::NiTransform from = identityTransform();
        RE::NiTransform requested = identityTransform();
        requested.translate = RE::NiPoint3{ 10.0f, 0.0f, 0.0f };

        const auto limited = limitGeneratedDriveTarget(from, requested, driveDeltaSeconds, gameToHavokScale, 5.0f, 0.0f);

        ok &= expectTrue("linear cap exceeded", limited.limit.linearLimitExceeded);
        ok &= expectFalse("linear cap does not report angular cap", limited.limit.angularLimitExceeded);
        ok &= expectNear("linear cap alpha", limited.limit.alpha, 0.5f, 0.001f);
        ok &= expectNear("linear cap target x", limited.target.translate.x, 5.0f, 0.001f);
    }

    {
        RE::NiTransform from = identityTransform();
        RE::NiTransform requested = identityTransform();
        requested.rotate = rotationAroundZ(kPi * 0.5f);

        const auto limited = limitGeneratedDriveTarget(from, requested, driveDeltaSeconds, gameToHavokScale, 0.0f, kPi);

        ok &= expectFalse("angular cap does not report linear cap", limited.limit.linearLimitExceeded);
        ok &= expectTrue("angular cap exceeded", limited.limit.angularLimitExceeded);
        ok &= expectNear("angular cap alpha", limited.limit.alpha, 0.2f, 0.001f);
        ok &= expectNear("angular cap target radians", rotationDistanceRadians(from.rotate, limited.target.rotate), kPi * 0.1f, 0.001f);
    }

    {
        RE::NiTransform from = identityTransform();
        RE::NiTransform requested = identityTransform();
        requested.translate = RE::NiPoint3{ 10.0f, 0.0f, 0.0f };
        requested.rotate = rotationAroundZ(kPi * 0.5f);

        const auto limited = limitGeneratedDriveTarget(from, requested, driveDeltaSeconds, gameToHavokScale, 8.0f, kPi);

        ok &= expectTrue("combined cap reports linear", limited.limit.linearLimitExceeded);
        ok &= expectTrue("combined cap reports angular", limited.limit.angularLimitExceeded);
        ok &= expectNear("combined cap uses stricter alpha", limited.limit.alpha, 0.2f, 0.001f);
        ok &= expectNear("combined cap translation follows stricter alpha", limited.target.translate.x, 2.0f, 0.001f);
        ok &= expectNear("combined cap rotation follows stricter alpha", rotationDistanceRadians(from.rotate, limited.target.rotate), kPi * 0.1f, 0.001f);
    }

    {
        RE::NiTransform from = identityTransform();
        RE::NiTransform requested = identityTransform();
        requested.translate = RE::NiPoint3{ 100.0f, 0.0f, 0.0f };
        requested.rotate = rotationAroundZ(kPi);

        const auto limited = limitGeneratedDriveTarget(from, requested, driveDeltaSeconds, gameToHavokScale, 0.0f, -1.0f);

        ok &= expectFalse("non-positive linear cap stays unlimited", limited.limit.linearLimitExceeded);
        ok &= expectFalse("non-positive angular cap stays unlimited", limited.limit.angularLimitExceeded);
        ok &= expectNear("non-positive caps alpha", limited.limit.alpha, 1.0f, 0.001f);
        ok &= expectNear("non-positive caps translation unchanged", pointDistance(limited.target.translate, requested.translate), 0.0f, 0.001f);
        ok &= expectNear("non-positive caps rotation unchanged", rotationDistanceRadians(limited.target.rotate, requested.rotate), 0.0f, 0.001f);
    }

    // Fail-closed timing: an unmeasured drive delta commands zero motion
    // instead of a velocity computed against a fabricated rate.
    {
        RE::NiTransform from = identityTransform();
        RE::NiTransform requested = identityTransform();
        requested.translate = RE::NiPoint3{ 10.0f, 0.0f, 0.0f };

        const auto limited = limitGeneratedDriveTarget(from, requested, 0.0f, gameToHavokScale, 2.0f, kPi);

        ok &= expectTrue("unmeasured dt reports linear clamp", limited.limit.linearLimitExceeded);
        ok &= expectTrue("unmeasured dt reports angular clamp", limited.limit.angularLimitExceeded);
        ok &= expectNear("unmeasured dt commands zero motion", limited.limit.alpha, 0.0f, 0.0f);
        ok &= expectNear("unmeasured dt holds the body", pointDistance(limited.target.translate, from.translate), 0.0f, 0.001f);
    }

    // Fail-closed source interval: no measured source delta means no sampled
    // velocity.
    {
        RE::NiPoint3 velocity{};
        const bool computed = tryComputeSampledLinearVelocityHavok(
            RE::NiPoint3{ 0.0f, 0.0f, 0.0f },
            RE::NiPoint3{ 1.0f, 0.0f, 0.0f },
            0.0f,
            gameToHavokScale,
            velocity);
        ok &= expectFalse("unmeasured source delta yields no velocity sample", computed);
    }

    // Contact pressure is relative to shared transport, not world speed. Both
    // directions of walking must leave the same bounded relative pressure.
    for (const float walkingSpeed : { -4.0f, 0.0f, 4.0f }) {
        for (const float pressSign : { -1.0f, 1.0f }) {
            RE::NiPoint3 velocity{ walkingSpeed + pressSign * 6.0f, 2.0f, 0.0f };
            ok &= expectTrue("moving contact press is limited", clampContactPressVelocity(
                velocity, RE::NiPoint3{ pressSign, 0.0f, 0.0f },
                RE::NiPoint3{ walkingSpeed, 0.0f, 0.0f }, 1.0f, 15.0f));
            ok &= expectNear("walking does not consume press allowance",
                velocity.x, walkingSpeed + pressSign, 0.0001f);
            ok &= expectNear("contact tangent keeps full velocity", velocity.y, 2.0f, 0.0001f);
        }
    }
    {
        RE::NiPoint3 worldPress{ 6.0f, 2.0f, 0.0f };
        ok &= expectTrue("stationary world contact still limits pressure", clampContactPressVelocity(
            worldPress, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, RE::NiPoint3{}, 1.0f, 15.0f));
        ok &= expectNear("world press remains one", worldPress.x, 1.0f, 0.0001f);

        RE::NiPoint3 retreat{ 2.0f, 0.0f, 0.0f };
        ok &= expectFalse("retreat relative to contact is unrestricted", clampContactPressVelocity(
            retreat, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, RE::NiPoint3{ 4.0f, 0.0f, 0.0f }, 1.0f, 15.0f));
        ok &= expectNear("retreat velocity preserved", retreat.x, 2.0f, 0.0001f);

        RE::NiPoint3 fastRetreat{ 6.0f, 2.0f, 0.0f };
        ok &= expectTrue("fast reference still clamps", clampContactPressVelocity(
            fastRetreat, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, RE::NiPoint3{ -30.0f, 0.0f, 0.0f }, 1.0f, 15.0f));
        ok &= expectNear("transport preserves absolute speed guard", pointDistance(fastRetreat, RE::NiPoint3{}), 15.0f, 0.0001f);
    }

    // Reproduce the failure over sustained contact: the contact plane and
    // controller translate together, with constant blocked depth. The solver
    // rejects penetration but lets the collider follow a receding plane.
    // The old world cap accumulates a recovery-sized gap; the relative cap
    // preserves the initial depth across ordinary frame rates/substeps.
    for (const float delta : { 1.0f / 45.0f, 1.0f / 90.0f, 1.0f / 180.0f }) {
        for (const float direction : { -1.0f, 1.0f }) {
            float plane = 0.0f;
            float oldBody = 0.0f;
            float body = 0.0f;
            constexpr float depth = 0.05f;
            const float walk = 4.0f * direction;
            for (int step = 0; step < static_cast<int>(2.0f / delta); ++step) {
                plane += walk * delta;
                const float target = plane + direction * depth;
                RE::NiPoint3 oldVelocity{ (target - oldBody) / delta, 0.0f, 0.0f };
                RE::NiPoint3 velocity{ (target - body) / delta, 0.0f, 0.0f };
                const RE::NiPoint3 press{ direction, 0.0f, 0.0f };
                (void)clampContactPressVelocity(oldVelocity, press, RE::NiPoint3{}, 1.0f, 15.0f);
                (void)clampContactPressVelocity(velocity, press, RE::NiPoint3{ walk, 0.0f, 0.0f }, 1.0f, 15.0f);
                oldBody += oldVelocity.x * delta;
                body += velocity.x * delta;
                if ((body - plane) * direction > 0.0f) body = plane;
                if ((oldBody - plane) * direction > 0.0f) oldBody = plane;
            }
            ok &= expectNear("moving contact preserves blocked depth", std::fabs(plane - body), 0.0f, 0.0001f);
            ok &= expectTrue("old world cap reproduces sustained locomotion lag", std::fabs(plane - oldBody) > 5.0f);
        }
    }

    return ok ? 0 : 1;
}
