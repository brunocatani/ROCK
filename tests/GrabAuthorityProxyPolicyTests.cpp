#include "physics-interaction/grab/GrabAuthorityProxyMotion.h"
#include "physics-interaction/TransformMath.h"

#include <cmath>
#include <cstdio>

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

    RE::NiMatrix3 rotationAroundX(float radians)
    {
        const float halfAngle = radians * 0.5f;
        const float quaternion[4]{ std::sin(halfAngle), 0.0f, 0.0f, std::cos(halfAngle) };
        return rock::transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(quaternion);
    }
}

int main()
{
    bool ok = true;

    {
        RE::NiTransform previous = identityTransform();
        RE::NiTransform current = identityTransform();
        current.rotate = rotationAroundZ(kPi * 0.5f);

        float angularVelocity[4]{};
        rock::grab_authority_proxy_motion::computeAngularVelocityRadiansPerSecond(previous, current, 0.1f, angularVelocity);

        ok &= expectNear("zero x for z rotation", angularVelocity[0], 0.0f, 0.001f);
        ok &= expectNear("zero y for z rotation", angularVelocity[1], 0.0f, 0.001f);
        ok &= expectNear("positive z angular velocity", angularVelocity[2], 5.0f * kPi, 0.001f);
        ok &= expectNear("zero w angular velocity", angularVelocity[3], 0.0f, 0.001f);
    }

    {
        RE::NiTransform previous = identityTransform();
        RE::NiTransform current = identityTransform();
        current.rotate = rotationAroundX(-kPi * 0.5f);

        float angularVelocity[4]{};
        rock::grab_authority_proxy_motion::computeAngularVelocityRadiansPerSecond(previous, current, 0.1f, angularVelocity);

        ok &= expectNear("negative x angular velocity", angularVelocity[0], -5.0f * kPi, 0.001f);
        ok &= expectNear("zero y for x rotation", angularVelocity[1], 0.0f, 0.001f);
        ok &= expectNear("zero z for x rotation", angularVelocity[2], 0.0f, 0.001f);
    }

    {
        RE::NiTransform previous = identityTransform();
        RE::NiTransform current = previous;

        float angularVelocity[4]{ 9.0f, 9.0f, 9.0f, 9.0f };
        rock::grab_authority_proxy_motion::computeAngularVelocityRadiansPerSecond(previous, current, 0.0f, angularVelocity);

        ok &= expectNear("invalid dt clears x", angularVelocity[0], 0.0f, 0.001f);
        ok &= expectNear("invalid dt clears y", angularVelocity[1], 0.0f, 0.001f);
        ok &= expectNear("invalid dt clears z", angularVelocity[2], 0.0f, 0.001f);
        ok &= expectNear("invalid dt clears w", angularVelocity[3], 0.0f, 0.001f);
    }

    return ok ? 0 : 1;
}
