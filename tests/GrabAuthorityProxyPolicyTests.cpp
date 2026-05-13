#include "physics-interaction/grab/GrabAuthorityProxyMotion.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/GrabConstraintMath.h"

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

    float pointDistance(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        const float dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    RE::NiPoint3 matrixColumn(const RE::NiMatrix3& matrix, int column)
    {
        return RE::NiPoint3{ matrix.entry[0][column], matrix.entry[1][column], matrix.entry[2][column] };
    }

    float axisDeltaDegrees(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        const float aLength = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        const float bLength = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
        if (aLength <= 0.000001f || bLength <= 0.000001f) {
            return 0.0f;
        }

        const float dot = (a.x * b.x + a.y * b.y + a.z * b.z) / (aLength * bLength);
        return std::acos(std::clamp(dot, -1.0f, 1.0f)) * (180.0f / kPi);
    }

    float rotationDeltaDegrees(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
    {
        const float x = axisDeltaDegrees(matrixColumn(a, 0), matrixColumn(b, 0));
        const float y = axisDeltaDegrees(matrixColumn(a, 1), matrixColumn(b, 1));
        const float z = axisDeltaDegrees(matrixColumn(a, 2), matrixColumn(b, 2));
        return std::max(x, std::max(y, z));
    }

    RE::NiMatrix3 matrixFromRawRows(const float* raw)
    {
        RE::NiMatrix3 result{};
        result.entry[0][0] = raw[0];
        result.entry[0][1] = raw[1];
        result.entry[0][2] = raw[2];
        result.entry[1][0] = raw[4];
        result.entry[1][1] = raw[5];
        result.entry[1][2] = raw[6];
        result.entry[2][0] = raw[8];
        result.entry[2][1] = raw[9];
        result.entry[2][2] = raw[10];
        return result;
    }

    RE::NiMatrix3 matrixFromRawColumns(const float* raw)
    {
        RE::NiMatrix3 result{};
        result.entry[0][0] = raw[0];
        result.entry[1][0] = raw[1];
        result.entry[2][0] = raw[2];
        result.entry[0][1] = raw[4];
        result.entry[1][1] = raw[5];
        result.entry[2][1] = raw[6];
        result.entry[0][2] = raw[8];
        result.entry[1][2] = raw[9];
        result.entry[2][2] = raw[10];
        return result;
    }

    bool expectTransformClose(const char* label, const RE::NiTransform& actual, const RE::NiTransform& expected)
    {
        bool ok = true;
        ok &= expectNear((std::string(label) + " position").c_str(), pointDistance(actual.translate, expected.translate), 0.0f, 0.001f);
        ok &= expectNear((std::string(label) + " rotation").c_str(), rotationDeltaDegrees(actual.rotate, expected.rotate), 0.0f, 0.01f);
        return ok;
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

    {
        RE::NiTransform desiredBodyTransformHandSpace = identityTransform();
        desiredBodyTransformHandSpace.rotate = rotationAroundZ(kPi * 0.5f);
        const RE::NiMatrix3 expectedBodyToHand =
            rock::grab_constraint_math::desiredBodyToHandRotation(desiredBodyTransformHandSpace.rotate);

        float transformBRotation[12]{};
        float targetBRca[12]{};
        rock::grab_constraint_math::writeInitialGrabAngularFrame(
            transformBRotation,
            targetBRca,
            desiredBodyTransformHandSpace);

        ok &= expectNear(
            "transformB rows carry body-to-hand rotation",
            rotationDeltaDegrees(matrixFromRawRows(transformBRotation), expectedBodyToHand),
            0.0f,
            0.01f);
        ok &= expectNear(
            "target rows carry body-to-hand rotation",
            rotationDeltaDegrees(matrixFromRawRows(targetBRca), expectedBodyToHand),
            0.0f,
            0.01f);
        if (rotationDeltaDegrees(matrixFromRawColumns(targetBRca), expectedBodyToHand) <= 1.0f) {
            std::printf("target_bRca column interpretation unexpectedly matches the solver row convention\n");
            ok = false;
        }
    }

    {
        RE::NiTransform objectWorld = identityTransform();
        objectWorld.translate = RE::NiPoint3{ 10.0f, -4.0f, 3.0f };
        objectWorld.rotate = rotationAroundZ(kPi * 0.5f);

        RE::NiTransform objectToBody = identityTransform();
        objectToBody.translate = RE::NiPoint3{ 1.25f, -2.5f, 3.75f };
        objectToBody.rotate = rotationAroundX(kPi * 0.5f);

        const RE::NiTransform bodyWorld = rock::transform_math::composeTransforms(objectWorld, objectToBody);
        const RE::NiTransform capturedObjectToBody = rock::transform_math::composeTransforms(rock::transform_math::invertTransform(objectWorld), bodyWorld);

        RE::NiTransform desiredObjectWorld = identityTransform();
        desiredObjectWorld.translate = RE::NiPoint3{ -7.0f, 2.0f, 19.0f };
        desiredObjectWorld.rotate = rotationAroundZ(-kPi * 0.5f);

        const RE::NiTransform desiredBodyWorld = rock::transform_math::composeTransforms(desiredObjectWorld, capturedObjectToBody);
        const RE::NiTransform recoveredObjectWorld =
            rock::transform_math::composeTransforms(desiredBodyWorld, rock::transform_math::invertTransform(capturedObjectToBody));

        ok &= expectTransformClose("visual object recovered from body target", recoveredObjectWorld, desiredObjectWorld);

        const RE::NiTransform identityBodyTarget = rock::transform_math::composeTransforms(desiredObjectWorld, identityTransform());
        const RE::NiTransform identityRecoveredObject =
            rock::transform_math::composeTransforms(identityBodyTarget, rock::transform_math::invertTransform(capturedObjectToBody));
        if (pointDistance(identityRecoveredObject.translate, desiredObjectWorld.translate) <= 0.1f &&
            rotationDeltaDegrees(identityRecoveredObject.rotate, desiredObjectWorld.rotate) <= 0.1f) {
            std::printf("identity bodyLocal unexpectedly preserved visual object relation\n");
            ok = false;
        }
    }

    return ok ? 0 : 1;
}
