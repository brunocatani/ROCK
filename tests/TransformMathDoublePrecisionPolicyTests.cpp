#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/WeaponGeometry.h"

#include <cmath>
#include <cstdio>

namespace
{
    struct TestVector
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct TestMatrix
    {
        float entry[3][3]{};
    };

    struct TestTransform
    {
        TestMatrix rotate{};
        TestVector translate{};
        float scale = 1.0f;
    };

    bool expectFloat(const char* label, float actual, float expected)
    {
        if (std::fabs(actual - expected) <= 0.0001f) {
            return true;
        }

        std::printf("%s expected %.8f got %.8f\n", label, expected, actual);
        return false;
    }

    bool expectVector(const char* label, const TestVector& actual, const TestVector& expected)
    {
        bool ok = true;
        ok &= expectFloat(label, actual.x, expected.x);
        ok &= expectFloat(label, actual.y, expected.y);
        ok &= expectFloat(label, actual.z, expected.z);
        return ok;
    }

    TestTransform identityTransform()
    {
        return rock::transform_math::makeIdentityTransform<TestTransform>();
    }
}

int main()
{
    bool ok = true;

    {
        TestMatrix rotation{};
        rotation.entry[0][0] = -7312.71533203125f;
        rotation.entry[1][0] = 6948.6748046875f;
        rotation.entry[2][0] = 5275.4921875f;
        const TestVector vector{ -4898.61962890625f, -91.29825592041016f, -1010.1787109375f };

        const TestVector result = rock::transform_math::rotateLocalVectorToWorld(rotation, vector);
        ok &= expectVector("local vector rotation uses double intermediates", result, TestVector{ 29858620.0f, 0.0f, 0.0f });
    }

    {
        TestMatrix lhs{};
        lhs.entry[0][0] = -7312.71533203125f;
        lhs.entry[0][1] = 6948.6748046875f;
        lhs.entry[0][2] = 5275.4921875f;

        TestMatrix rhs{};
        rhs.entry[0][0] = -4898.61962890625f;
        rhs.entry[1][0] = -91.29825592041016f;
        rhs.entry[2][0] = -1010.1787109375f;

        const TestMatrix result = rock::transform_math::multiplyStoredRotations(lhs, rhs);
        ok &= expectFloat("stored rotation multiply uses double intermediates", result.entry[0][0], 29858620.0f);
    }

    {
        TestTransform parent = identityTransform();
        parent.rotate.entry[0][0] = -7312.71533203125f;
        parent.rotate.entry[1][0] = 6948.6748046875f;
        parent.rotate.entry[2][0] = 5275.4921875f;

        TestTransform child = identityTransform();
        child.translate = TestVector{ -4898.61962890625f, -91.29825592041016f, -1010.1787109375f };

        const TestTransform result = rock::transform_math::composeTransforms(parent, child);
        ok &= expectVector("composed transform translation uses double intermediates",
            result.translate,
            TestVector{ 29858620.0f, -91.29825592041016f, -1010.1787109375f });
    }

    {
        TestTransform parent = identityTransform();
        parent.rotate.entry[0][0] = -7312.71533203125f;
        parent.rotate.entry[0][1] = 6948.6748046875f;
        parent.rotate.entry[0][2] = 5275.4921875f;
        const TestVector point{ -4898.61962890625f, -91.29825592041016f, -1010.1787109375f };

        const TestVector result = rock::transform_math::worldPointToLocal(parent, point);
        ok &= expectVector("world point inverse path uses double intermediates",
            result,
            TestVector{ 29858620.0f, -91.29825592041016f, -1010.1787109375f });
    }

    {
        TestMatrix rootRotation{};
        rootRotation.entry[0][0] = -7312.71533203125f;
        rootRotation.entry[1][0] = 6948.6748046875f;
        rootRotation.entry[2][0] = 5275.4921875f;
        rootRotation.entry[1][1] = 1.0f;
        rootRotation.entry[2][2] = 1.0f;
        const TestVector rootTranslation{};
        const TestVector localPoint{ -4898.61962890625f, -91.29825592041016f, -1010.1787109375f };

        const TestVector result = rock::weapon_collision_geometry_math::localPointToWorld(rootRotation, rootTranslation, 1.0f, localPoint);
        ok &= expectVector("weapon geometry local point uses double intermediates",
            result,
            TestVector{ 29858620.0f, -91.29825592041016f, -1010.1787109375f });
    }

    {
        TestMatrix rootRotation{};
        rootRotation.entry[0][0] = -7312.71533203125f;
        rootRotation.entry[0][1] = 6948.6748046875f;
        rootRotation.entry[0][2] = 5275.4921875f;
        rootRotation.entry[1][1] = 1.0f;
        rootRotation.entry[2][2] = 1.0f;
        const TestVector rootTranslation{};
        const TestVector worldPoint{ -4898.61962890625f, -91.29825592041016f, -1010.1787109375f };

        const TestVector result = rock::weapon_collision_geometry_math::worldPointToLocal(rootRotation, rootTranslation, 1.0f, worldPoint);
        ok &= expectVector("weapon geometry world point uses double intermediates",
            result,
            TestVector{ 29858620.0f, -91.29825592041016f, -1010.1787109375f });
    }

    return ok ? 0 : 1;
}
