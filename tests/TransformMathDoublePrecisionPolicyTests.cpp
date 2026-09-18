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

    // Independent forward reconstruction, calibrated below against the actual
    // native loose-root matrices from the recorded drop session.
    TestMatrix nativeReferenceMatrix(const TestVector& angles)
    {
        const double cx = std::cos(static_cast<double>(angles.x));
        const double sx = std::sin(static_cast<double>(angles.x));
        const double cy = std::cos(static_cast<double>(angles.y));
        const double sy = std::sin(static_cast<double>(angles.y));
        const double cz = std::cos(static_cast<double>(angles.z));
        const double sz = std::sin(static_cast<double>(angles.z));
        const double entries[3][3]{
            { cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz },
            { cy * sz, sx * sy * sz + cx * cz, cx * sy * sz - sx * cz },
            { -sy, sx * cy, cx * cy },
        };
        TestMatrix result{};
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column)
                result.entry[row][column] = static_cast<float>(entries[row][column]);
        return result;
    }

    bool expectRotation(const char* label, const TestMatrix& actual, const TestMatrix& expected)
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(actual.entry[row][column]) ||
                    std::abs(actual.entry[row][column] - expected.entry[row][column]) > 0.000002f) {
                    std::printf("%s [%d,%d] expected %.9f got %.9f\n", label, row, column,
                        expected.entry[row][column], actual.entry[row][column]);
                    return false;
                }
            }
        }
        return true;
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


    {
        struct RecordedDrop
        {
            TestMatrix release;
            TestVector nativeRequest;
            TestMatrix spawnedRoot;
        };
        const RecordedDrop drops[]{
            // 2026-09-17 20:38:08.106, right hand, ref FF001197.
            { { { { 0.842013700f, 0.083444900f, 0.532962700f },
                  { -0.211039600f, 0.960178600f, 0.183082700f },
                  { -0.496462400f, -0.266634500f, 0.826093900f } } },
                { -0.218099340f, 0.562098155f, -0.098779063f },
                { { { 0.842014000f, -0.018475100f, 0.539139200f },
                  { -0.083444900f, 0.982923900f, 0.164004700f },
                  { -0.532962700f, -0.183082700f, 0.826094100f } } } },
            // 2026-09-17 20:39:57.544, left hand, ref FF0017EA.
            { { { { -0.550497900f, 0.728385300f, -0.407930100f },
                  { -0.533430800f, 0.068979000f, 0.843026700f },
                  { 0.642186800f, 0.681686800f, 0.350570200f } } },
                { -1.176703748f, -0.420185802f, -2.217985386f },
                { { { -0.550497800f, 0.079218700f, 0.831069500f },
                  { -0.728385300f, -0.532006300f, -0.431768500f },
                  { 0.407930100f, -0.843026500f, 0.350570100f } } } },
            // 2026-09-17 20:40:22.470, right hand, ref FF001197.
            { { { { -0.056600300f, -0.236650900f, 0.969944800f },
                  { -0.593466200f, 0.789213400f, 0.157924200f },
                  { -0.802866400f, -0.566690800f, -0.185114100f } } },
                { -2.435291121f, 1.325003850f, 1.805558380f },
                { { { -0.056600300f, 0.886332500f, -0.459577000f },
                  { 0.236650600f, -0.435286900f, -0.868632100f },
                  { -0.969944800f, -0.157924000f, -0.185113800f } } } },
        };
        for (const auto& drop : drops) {
            ok &= expectRotation("recorded native reference reconstruction",
                nativeReferenceMatrix(drop.nativeRequest), drop.spawnedRoot);
            const auto corrected = rock::transform_math::matrixToReferenceEulerRadians<TestMatrix, TestVector>(drop.release);
            ok &= expectRotation("native drop preserves recorded equipped rotation",
                nativeReferenceMatrix(corrected), drop.release);
        }

        // Identity, individual axes, mixed axes, and both sides of each vertical
        // pole. Compare orientation, since Euler triples are nonunique at a pole.
        constexpr float halfPi = 1.5707963267948966f;
        for (const float roll : { -3.0f, -0.8f, 0.0f, 0.9f, 3.0f }) {
            for (const float yaw : { -3.0f, -1.2f, 0.0f, 1.1f, 3.0f }) {
                for (const float pitch : { -halfPi - 0.00001f, -halfPi, -halfPi + 0.0000005f,
                         -halfPi + 0.00001f, -0.7f, 0.0f, 0.6f,
                         halfPi - 0.00001f, halfPi - 0.0000005f, halfPi, halfPi + 0.00001f }) {
                    const auto expected = nativeReferenceMatrix({ roll, pitch, yaw });
                    const auto corrected = rock::transform_math::matrixToReferenceEulerRadians<TestMatrix, TestVector>(expected);
                    ok &= expectRotation("native reference rotation round trip",
                        nativeReferenceMatrix(corrected), expected);
                }
            }
        }
    }


    return ok ? 0 : 1;
}
