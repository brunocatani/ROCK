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

    RE::NiMatrix3 rotationAroundAxis(const RE::NiPoint3& axis, float radians)
    {
        const float axisLength = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
        const float halfAngle = radians * 0.5f;
        const float vectorScale = std::sin(halfAngle) / axisLength;
        const float quaternion[4]{
            axis.x * vectorScale,
            axis.y * vectorScale,
            axis.z * vectorScale,
            std::cos(halfAngle),
        };
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

    {
        constexpr float sampledDeltaSeconds = 0.1f;
        RE::NiPoint3 positiveAngular{};
        RE::NiPoint3 negativeAngular{};
        const bool positiveValid = tryComputeSampledAngularVelocityRadians(
            rotationAroundZ(0.0f),
            rotationAroundZ(kPi * 0.5f),
            sampledDeltaSeconds,
            positiveAngular);
        const bool negativeValid = tryComputeSampledAngularVelocityRadians(
            rotationAroundZ(0.0f),
            rotationAroundZ(-kPi * 0.5f),
            sampledDeltaSeconds,
            negativeAngular);

        ok &= expectTrue("positive angular sample valid", positiveValid);
        ok &= expectTrue("negative angular sample valid", negativeValid);
        ok &= expectNear("positive swing angular x", positiveAngular.x, 0.0f, 0.001f);
        ok &= expectNear("positive swing angular y", positiveAngular.y, 0.0f, 0.001f);
        ok &= expectNear("positive swing angular z", positiveAngular.z, (kPi * 0.5f) / sampledDeltaSeconds, 0.001f);
        ok &= expectNear("negative swing angular x", negativeAngular.x, 0.0f, 0.001f);
        ok &= expectNear("negative swing angular y", negativeAngular.y, 0.0f, 0.001f);
        ok &= expectNear("negative swing angular z", negativeAngular.z, -(kPi * 0.5f) / sampledDeltaSeconds, 0.001f);
        ok &= expectNear("opposite swings preserve angular speed", std::fabs(positiveAngular.z), std::fabs(negativeAngular.z), 0.001f);

        const RE::NiPoint3 leverHavok{ 1.0f, 0.0f, 0.0f };
        const RE::NiPoint3 positivePointVelocity{
            positiveAngular.y * leverHavok.z - positiveAngular.z * leverHavok.y,
            positiveAngular.z * leverHavok.x - positiveAngular.x * leverHavok.z,
            positiveAngular.x * leverHavok.y - positiveAngular.y * leverHavok.x,
        };
        const RE::NiPoint3 negativePointVelocity{
            negativeAngular.y * leverHavok.z - negativeAngular.z * leverHavok.y,
            negativeAngular.z * leverHavok.x - negativeAngular.x * leverHavok.z,
            negativeAngular.x * leverHavok.y - negativeAngular.y * leverHavok.x,
        };
        ok &= expectNear("positive swing point closing speed", positivePointVelocity.y, (kPi * 0.5f) / sampledDeltaSeconds, 0.001f);
        ok &= expectNear("negative swing point closing speed", -negativePointVelocity.y, (kPi * 0.5f) / sampledDeltaSeconds, 0.001f);
    }

    {
        constexpr float sampledDeltaSeconds = 0.1f;
        constexpr float shortestArcRadians = 20.0f * kPi / 180.0f;
        RE::NiPoint3 forwardAcrossPi{};
        RE::NiPoint3 reverseAcrossPi{};
        const bool forwardValid = tryComputeSampledAngularVelocityRadians(
            rotationAroundZ(170.0f * kPi / 180.0f),
            rotationAroundZ(-170.0f * kPi / 180.0f),
            sampledDeltaSeconds,
            forwardAcrossPi);
        const bool reverseValid = tryComputeSampledAngularVelocityRadians(
            rotationAroundZ(-170.0f * kPi / 180.0f),
            rotationAroundZ(170.0f * kPi / 180.0f),
            sampledDeltaSeconds,
            reverseAcrossPi);

        ok &= expectTrue("forward shortest-arc sample valid", forwardValid);
        ok &= expectTrue("reverse shortest-arc sample valid", reverseValid);
        ok &= expectNear("forward shortest arc is positive", forwardAcrossPi.z, shortestArcRadians / sampledDeltaSeconds, 0.001f);
        ok &= expectNear("reverse shortest arc is negative", reverseAcrossPi.z, -shortestArcRadians / sampledDeltaSeconds, 0.001f);
    }

    {
        constexpr float sampledDeltaSeconds = 0.1f;
        constexpr float angleRadians = kPi * 0.25f;
        const RE::NiPoint3 axis{ 1.0f, 2.0f, 3.0f };
        const float axisLength = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
        RE::NiPoint3 obliqueAngular{};
        const bool obliqueValid = tryComputeSampledAngularVelocityRadians(
            rotationAroundAxis(axis, 0.0f),
            rotationAroundAxis(axis, angleRadians),
            sampledDeltaSeconds,
            obliqueAngular);

        ok &= expectTrue("oblique angular sample valid", obliqueValid);
        ok &= expectNear("oblique angular x", obliqueAngular.x, axis.x * angleRadians / (axisLength * sampledDeltaSeconds), 0.001f);
        ok &= expectNear("oblique angular y", obliqueAngular.y, axis.y * angleRadians / (axisLength * sampledDeltaSeconds), 0.001f);
        ok &= expectNear("oblique angular z", obliqueAngular.z, axis.z * angleRadians / (axisLength * sampledDeltaSeconds), 0.001f);
    }

    return ok ? 0 : 1;
}
