#include "physics-interaction/hand/HandFingerMirrorMath.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <span>

namespace
{
    struct TestVector3
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    struct TestMatrix3
    {
        float entry[3][4]{};
    };

    struct TestTransform
    {
        TestMatrix3 rotate{};
        TestVector3 translate{};
        float scale{ 1.0f };
    };

    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float tolerance = 0.0001f)
    {
        if (std::fabs(actual - expected) <= tolerance) {
            return true;
        }
        std::printf("%s expected %.6f got %.6f\n", label, expected, actual);
        return false;
    }

    bool expectTransformNear(const char* label, const TestTransform& actual, const TestTransform& expected, float tolerance = 0.0001f)
    {
        bool ok = true;
        ok &= expectNear(label, actual.translate.x, expected.translate.x, tolerance);
        ok &= expectNear(label, actual.translate.y, expected.translate.y, tolerance);
        ok &= expectNear(label, actual.translate.z, expected.translate.z, tolerance);
        ok &= expectNear(label, actual.scale, expected.scale, tolerance);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                ok &= expectNear(label, actual.rotate.entry[row][column], expected.rotate.entry[row][column], tolerance);
            }
        }
        return ok;
    }

    TestTransform makeTransform(const float (&rows)[3][3], TestVector3 translate, float scale = 1.0f)
    {
        TestTransform transform{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                transform.rotate.entry[row][column] = rows[row][column];
            }
        }
        transform.translate = translate;
        transform.scale = scale;
        return transform;
    }

    TestTransform makeAxisAngle(TestVector3 axis, float degrees, TestVector3 translate)
    {
        const float length = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
        const float x = axis.x / length;
        const float y = axis.y / length;
        const float z = axis.z / length;
        const float radians = degrees * 0.01745329251994329577f;
        const float c = std::cos(radians);
        const float s = std::sin(radians);
        const float t = 1.0f - c;
        const float rows[3][3] = {
            { t * x * x + c, t * x * y - s * z, t * x * z + s * y },
            { t * x * y + s * z, t * y * y + c, t * y * z - s * x },
            { t * x * z - s * y, t * y * z + s * x, t * z * z + c },
        };
        return makeTransform(rows, translate);
    }

    float determinant(const TestMatrix3& m)
    {
        return m.entry[0][0] * (m.entry[1][1] * m.entry[2][2] - m.entry[1][2] * m.entry[2][1]) -
               m.entry[0][1] * (m.entry[1][0] * m.entry[2][2] - m.entry[1][2] * m.entry[2][0]) +
               m.entry[0][2] * (m.entry[1][0] * m.entry[2][1] - m.entry[1][1] * m.entry[2][0]);
    }
}

int main()
{
    using namespace rock::hand_finger_mirror_math;
    bool ok = true;

    {
        // Identity keeps its rotation; only the local Z translation flips.
        const float identityRows[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
        const TestTransform right = makeTransform(identityRows, { 7.5f, 0.43f, -2.28f });
        const TestTransform left = mirrorFingerLocalAcrossHands(right);
        ok &= expectTransformNear(
            "identity finger local mirrors to identity with negated Z",
            left,
            makeTransform(identityRows, { 7.5f, 0.43f, 2.28f }));
    }

    {
        // hFRIK's authored thumb-base closed poses are an authored left/right
        // pair on the same skeleton; the mirror must reproduce one from the
        // other exactly, translation included.
        const float rightClosedRows[3][3] = {
            { 0.8494f, -0.2706f, -0.4531f },
            { -0.3826f, 0.2755f, -0.8819f },
            { 0.3635f, 0.9224f, 0.1305f },
        };
        const float leftClosedRows[3][3] = {
            { 0.8494f, -0.2706f, 0.4531f },
            { -0.3826f, 0.2755f, 0.8819f },
            { -0.3635f, -0.9224f, 0.1305f },
        };
        const TestTransform rightClosed = makeTransform(rightClosedRows, { 1.583f, -1.263f, -1.853f });
        const TestTransform leftClosed = makeTransform(leftClosedRows, { 1.583f, -1.263f, 1.853f });
        ok &= expectTransformNear("right thumb base mirrors onto authored left thumb base", mirrorFingerLocalAcrossHands(rightClosed), leftClosed);
        ok &= expectTransformNear("left thumb base mirrors onto authored right thumb base", mirrorFingerLocalAcrossHands(leftClosed), rightClosed);
    }

    {
        // General rotation: stays proper, keeps scale, and the mirror is an
        // involution.
        const TestTransform source = makeAxisAngle({ 0.3f, -0.7f, 0.5f }, 47.0f, { 3.1f, -0.4f, 1.9f });
        TestTransform scaledSource = source;
        scaledSource.scale = 1.25f;
        const TestTransform mirrored = mirrorFingerLocalAcrossHands(scaledSource);
        ok &= expectNear("finger mirror remains a proper rotation", determinant(mirrored.rotate), 1.0f);
        ok &= expectNear("finger mirror keeps scale", mirrored.scale, 1.25f);
        ok &= expectTransformNear("finger mirror is an involution", mirrorFingerLocalAcrossHands(mirrored), scaledSource);
        ok &= expectNear("finger mirror negates entry [0][2]", mirrored.rotate.entry[0][2], -source.rotate.entry[0][2]);
        ok &= expectNear("finger mirror negates entry [2][1]", mirrored.rotate.entry[2][1], -source.rotate.entry[2][1]);
        ok &= expectNear("finger mirror keeps entry [2][2]", mirrored.rotate.entry[2][2], source.rotate.entry[2][2]);
        ok &= expectNear("finger mirror keeps entry [0][1]", mirrored.rotate.entry[0][1], source.rotate.entry[0][1]);
    }

    {
        // Whole pose: fifteen bones, fail closed on size mismatch and on a
        // non-finite local, clearing the target.
        std::array<TestTransform, 15> source{};
        for (std::size_t index = 0; index < source.size(); ++index) {
            source[index] = makeAxisAngle(
                { 0.2f + 0.05f * static_cast<float>(index), -0.6f, 0.4f },
                10.0f + 4.0f * static_cast<float>(index),
                { 1.0f + static_cast<float>(index), -0.5f, 0.25f * static_cast<float>(index) });
        }
        std::array<TestTransform, 15> target{};
        ok &= expectTrue("complete finger pose mirrors", mirrorFingerLocalsAcrossHands<TestTransform>(source, target));
        for (std::size_t index = 0; index < source.size(); ++index) {
            ok &= expectTransformNear("pose mirror matches per-bone mirror", target[index], mirrorFingerLocalAcrossHands(source[index]));
        }

        std::array<TestTransform, 14> shortTarget{};
        ok &= expectTrue("size mismatch fails closed", !mirrorFingerLocalsAcrossHands<TestTransform>(source, shortTarget));

        std::array<TestTransform, 15> poisoned = source;
        poisoned[7].translate.y = std::numeric_limits<float>::quiet_NaN();
        std::array<TestTransform, 15> poisonedTarget = target;
        ok &= expectTrue("non-finite source fails closed", !mirrorFingerLocalsAcrossHands<TestTransform>(poisoned, poisonedTarget));
        ok &= expectNear("failed mirror clears target", poisonedTarget[0].translate.x, 0.0f);
        ok &= expectNear("failed mirror clears target rotation", poisonedTarget[0].rotate.entry[0][0], 0.0f);

        std::array<TestTransform, 15> zeroScale = source;
        zeroScale[3].scale = 0.0f;
        ok &= expectTrue("zero scale fails closed", !mirrorFingerLocalsAcrossHands<TestTransform>(zeroScale, poisonedTarget));
    }

    return ok ? 0 : 1;
}
