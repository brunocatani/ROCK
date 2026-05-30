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

    RE::NiMatrix3 rotationAroundY(float radians)
    {
        const float halfAngle = radians * 0.5f;
        const float quaternion[4]{ 0.0f, std::sin(halfAngle), 0.0f, std::cos(halfAngle) };
        return rock::transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(quaternion);
    }

    RE::NiMatrix3 composeRotations(const RE::NiMatrix3& parent, const RE::NiMatrix3& child)
    {
        RE::NiTransform parentTransform = identityTransform();
        parentTransform.rotate = parent;
        RE::NiTransform childTransform = identityTransform();
        childTransform.rotate = child;
        return rock::transform_math::composeTransforms(parentTransform, childTransform).rotate;
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

    RE::NiMatrix3 reconstructSolverEffectiveBodyInProxyRotation(
        const RE::NiMatrix3& transformBRotation,
        const RE::NiMatrix3& targetBRcaRotation)
    {
        return rock::transform_math::multiplyStoredRotations(
            rock::transform_math::transposeRotation(transformBRotation),
            rock::transform_math::transposeRotation(targetBRcaRotation));
    }
}

int main()
{
    bool ok = true;

    {
        RE::NiTransform previous = identityTransform();
        RE::NiTransform current = identityTransform();
        current.translate = RE::NiPoint3{ 7.0f, -14.0f, 21.0f };

        float linearVelocity[4]{};
        rock::grab_authority_proxy_motion::computeLinearVelocityHavok(previous, current, 0.1f, 0.1f, linearVelocity);

        ok &= expectNear("linear x uses game-to-havok scale", linearVelocity[0], 7.0f, 0.001f);
        ok &= expectNear("linear y uses game-to-havok scale", linearVelocity[1], -14.0f, 0.001f);
        ok &= expectNear("linear z uses game-to-havok scale", linearVelocity[2], 21.0f, 0.001f);
        ok &= expectNear("linear w remains zero", linearVelocity[3], 0.0f, 0.001f);
    }

    {
        RE::NiTransform previous = identityTransform();
        RE::NiTransform current = identityTransform();
        current.translate = RE::NiPoint3{ 7.0f, -14.0f, 21.0f };

        float linearVelocity[4]{ 9.0f, 9.0f, 9.0f, 9.0f };
        rock::grab_authority_proxy_motion::computeLinearVelocityHavok(previous, current, 0.0f, 0.1f, linearVelocity);

        ok &= expectNear("invalid dt clears linear x", linearVelocity[0], 0.0f, 0.001f);
        ok &= expectNear("invalid dt clears linear y", linearVelocity[1], 0.0f, 0.001f);
        ok &= expectNear("invalid dt clears linear z", linearVelocity[2], 0.0f, 0.001f);
        ok &= expectNear("invalid dt clears linear w", linearVelocity[3], 0.0f, 0.001f);
    }

    {
        const RE::NiMatrix3 compositeRotation =
            composeRotations(rotationAroundZ(kPi * 0.37f), composeRotations(rotationAroundX(-kPi * 0.23f), rotationAroundY(kPi * 0.41f)));
        const RE::NiMatrix3 cases[2]{
            rotationAroundZ(kPi * 0.5f),
            compositeRotation,
        };

        for (const auto& bodyInHandRotation : cases) {
            RE::NiTransform bodyInProxyAtCreation = identityTransform();
            bodyInProxyAtCreation.translate = RE::NiPoint3{ 3.0f, -5.0f, 2.0f };
            bodyInProxyAtCreation.rotate = bodyInHandRotation;
            RE::NiTransform bodyInProxyHeld = identityTransform();
            bodyInProxyHeld.translate = RE::NiPoint3{ -4.0f, 1.5f, 7.0f };
            bodyInProxyHeld.rotate = composeRotations(bodyInHandRotation, rotationAroundX(kPi * 0.17f));

            const RE::NiPoint3 frozenPivotAProxyLocal{ 0.75f, -1.25f, 2.5f };
            constexpr float gameToHavokScale = 2.0f;
            const RE::NiTransform expectedInitialProxyInBody =
                rock::grab_constraint_math::proxyInBodyFromBodyInProxy(bodyInProxyAtCreation);
            const RE::NiTransform expectedHeldProxyInBody =
                rock::grab_constraint_math::proxyInBodyFromBodyInProxy(bodyInProxyHeld);
            const RE::NiMatrix3 expectedInitialTargetResidual =
                rock::grab_constraint_math::computeRagdollTargetBRcaResidualFromFrozenTransformB(
                    expectedInitialProxyInBody.rotate,
                    expectedInitialProxyInBody.rotate);
            const RE::NiMatrix3 expectedHeldTargetResidual =
                rock::grab_constraint_math::computeRagdollTargetBRcaResidualFromFrozenTransformB(
                    expectedInitialProxyInBody.rotate,
                    expectedHeldProxyInBody.rotate);
            const RE::NiPoint3 expectedInitialPivotB =
                rock::grab_constraint_math::computeHiggsTransformBTranslationGame(bodyInProxyAtCreation, frozenPivotAProxyLocal);
            const RE::NiPoint3 expectedHeldPivotB =
                rock::grab_constraint_math::computeHiggsTransformBTranslationGame(bodyInProxyHeld, frozenPivotAProxyLocal);

            float transformBRotation[12]{};
            float transformBTranslation[4]{};
            float targetBRca[12]{};
            rock::grab_constraint_math::writeGrabConstraintCreationAtoms(
                transformBRotation,
                transformBTranslation,
                targetBRca,
                bodyInProxyAtCreation,
                frozenPivotAProxyLocal,
                gameToHavokScale);

            ok &= expectNear(
                "creation transformB columns freeze initial proxy-in-BODY rotation",
                rotationDeltaDegrees(matrixFromRawColumns(transformBRotation), expectedInitialProxyInBody.rotate),
                0.0f,
                0.01f);
            ok &= expectNear(
                "creation target rows carry neutral residual",
                rotationDeltaDegrees(matrixFromRawRows(targetBRca), expectedInitialTargetResidual),
                0.0f,
                0.01f);
            ok &= expectNear(
                "creation solver-effective relation preserves initial body-in-proxy rotation",
                rotationDeltaDegrees(
                    reconstructSolverEffectiveBodyInProxyRotation(matrixFromRawColumns(transformBRotation), matrixFromRawRows(targetBRca)),
                    bodyInProxyAtCreation.rotate),
                0.0f,
                0.01f);
            ok &= expectNear("creation transformB relation pivot x", transformBTranslation[0], expectedInitialPivotB.x * gameToHavokScale, 0.001f);
            ok &= expectNear("creation transformB relation pivot y", transformBTranslation[1], expectedInitialPivotB.y * gameToHavokScale, 0.001f);
            ok &= expectNear("creation transformB relation pivot z", transformBTranslation[2], expectedInitialPivotB.z * gameToHavokScale, 0.001f);

            const RE::NiMatrix3 frozenTransformBRotation = matrixFromRawColumns(transformBRotation);
            rock::grab_constraint_math::writeGrabConstraintHeldTargetAtoms(
                transformBTranslation,
                targetBRca,
                frozenTransformBRotation,
                bodyInProxyHeld,
                frozenPivotAProxyLocal,
                gameToHavokScale);

            ok &= expectNear(
                "held update leaves transformB rotation frozen",
                rotationDeltaDegrees(matrixFromRawColumns(transformBRotation), frozenTransformBRotation),
                0.0f,
                0.01f);
            ok &= expectNear(
                "held target rows carry residual relative to frozen transformB",
                rotationDeltaDegrees(matrixFromRawRows(targetBRca), expectedHeldTargetResidual),
                0.0f,
                0.01f);
            ok &= expectNear(
                "held solver-effective relation preserves current body-in-proxy rotation",
                rotationDeltaDegrees(
                    reconstructSolverEffectiveBodyInProxyRotation(frozenTransformBRotation, matrixFromRawRows(targetBRca)),
                    bodyInProxyHeld.rotate),
                0.0f,
                0.01f);
            if (rotationDeltaDegrees(matrixFromRawRows(targetBRca), expectedHeldProxyInBody.rotate) <= 1.0f) {
                std::printf("held target_bRca unexpectedly duplicates the full current proxy-in-BODY relation\n");
                ok = false;
            }
            ok &= expectNear("held transformB relation pivot x", transformBTranslation[0], expectedHeldPivotB.x * gameToHavokScale, 0.001f);
            ok &= expectNear("held transformB relation pivot y", transformBTranslation[1], expectedHeldPivotB.y * gameToHavokScale, 0.001f);
            ok &= expectNear("held transformB relation pivot z", transformBTranslation[2], expectedHeldPivotB.z * gameToHavokScale, 0.001f);
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
