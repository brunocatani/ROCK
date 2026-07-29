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

    return ok ? 0 : 1;
}
