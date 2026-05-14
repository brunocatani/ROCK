#include "physics-interaction/hand/HandColliderTypes.h"
#include "physics-interaction/TransformMath.h"

#include "RE/NetImmerse/NiMatrix3.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

namespace
{
    constexpr float kEpsilon = 0.001f;

    bool expectNear(const char* label, float actual, float expected, float epsilon = kEpsilon)
    {
        const float delta = std::fabs(actual - expected);
        if (delta <= epsilon) {
            return true;
        }

        std::printf("%s expected %.5f got %.5f\n", label, expected, actual);
        return false;
    }

    bool expectVectorNear(const char* label, const RE::NiPoint3& actual, const RE::NiPoint3& expected)
    {
        bool ok = true;
        ok &= expectNear((std::string(label) + ".x").c_str(), actual.x, expected.x);
        ok &= expectNear((std::string(label) + ".y").c_str(), actual.y, expected.y);
        ok &= expectNear((std::string(label) + ".z").c_str(), actual.z, expected.z);
        return ok;
    }

    bool expectMatrixNear(const char* label, const RE::NiMatrix3& actual, const RE::NiMatrix3& expected)
    {
        bool ok = true;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                ok &= expectNear(
                    (std::string(label) + "[" + std::to_string(row) + "][" + std::to_string(column) + "]").c_str(),
                    actual.entry[row][column],
                    expected.entry[row][column]);
            }
        }
        return ok;
    }

    bool expectTransformNear(const char* label, const RE::NiTransform& actual, const RE::NiTransform& expected)
    {
        bool ok = true;
        ok &= expectVectorNear((std::string(label) + ".translate").c_str(), actual.translate, expected.translate);
        ok &= expectMatrixNear((std::string(label) + ".rotate").c_str(), actual.rotate, expected.rotate);
        return ok;
    }

    float dot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    RE::NiPoint3 cross(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3{
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x,
        };
    }

    float length(const RE::NiPoint3& value)
    {
        return std::sqrt(dot(value, value));
    }

    RE::NiPoint3 normalize(const RE::NiPoint3& value)
    {
        const float len = length(value);
        if (len <= 0.000001f) {
            return RE::NiPoint3{};
        }
        return RE::NiPoint3{ value.x / len, value.y / len, value.z / len };
    }

    bool expectUnitAxes(const char* label, const RE::NiPoint3& xAxis, const RE::NiPoint3& yAxis, const RE::NiPoint3& zAxis)
    {
        bool ok = true;
        ok &= expectNear((std::string(label) + " x length").c_str(), length(xAxis), 1.0f);
        ok &= expectNear((std::string(label) + " y length").c_str(), length(yAxis), 1.0f);
        ok &= expectNear((std::string(label) + " z length").c_str(), length(zAxis), 1.0f);
        ok &= expectNear((std::string(label) + " x dot y").c_str(), dot(xAxis, yAxis), 0.0f);
        ok &= expectNear((std::string(label) + " y dot z").c_str(), dot(yAxis, zAxis), 0.0f);
        ok &= expectNear((std::string(label) + " z dot x").c_str(), dot(zAxis, xAxis), 0.0f);
        return ok;
    }

    RE::NiTransform identityTransform()
    {
        return rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    }

    RE::NiPoint3 rotateLocal(const RE::NiMatrix3& matrix, const RE::NiPoint3& localAxis)
    {
        return rock::transform_math::rotateLocalVectorToWorld(matrix, localAxis);
    }

    RE::NiPoint3 matrixRow(const RE::NiMatrix3& matrix, int row)
    {
        return RE::NiPoint3{ matrix.entry[row][0], matrix.entry[row][1], matrix.entry[row][2] };
    }

    RE::NiPoint3 matrixColumn(const RE::NiMatrix3& matrix, int column)
    {
        return RE::NiPoint3{ matrix.entry[0][column], matrix.entry[1][column], matrix.entry[2][column] };
    }
}

int main()
{
    bool ok = true;

    {
        const RE::NiPoint3 xAxis = normalize(RE::NiPoint3{ 0.36f, -0.48f, 0.80f });
        const RE::NiPoint3 yAxis = normalize(RE::NiPoint3{ -0.80f, 0.36f, 0.48f });
        const RE::NiPoint3 zAxis = normalize(cross(xAxis, yAxis));
        const RE::NiMatrix3 matrix =
            rock::hand_bone_collider_geometry_math::matrixFromAxes<RE::NiMatrix3>(xAxis, yAxis, zAxis);

        ok &= expectVectorNear("collider matrix column X", matrixColumn(matrix, 0), xAxis);
        ok &= expectVectorNear("collider matrix column Y", matrixColumn(matrix, 1), yAxis);
        ok &= expectVectorNear("collider matrix column Z", matrixColumn(matrix, 2), zAxis);

        RE::NiTransform colliderFrame = identityTransform();
        colliderFrame.rotate = matrix;
        const RE::NiTransform grabFrame =
            rock::hand_bone_collider_geometry_math::generatedColliderFrameToGrabAuthorityFrame(colliderFrame);
        ok &= expectVectorNear("grab frame row X", matrixRow(grabFrame.rotate, 0), xAxis);
        ok &= expectVectorNear("grab frame row Y", matrixRow(grabFrame.rotate, 1), yAxis);
        ok &= expectVectorNear("grab frame row Z", matrixRow(grabFrame.rotate, 2), zAxis);
        ok &= expectVectorNear("grab frame local X", rotateLocal(grabFrame.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }), xAxis);
        ok &= expectVectorNear("grab frame local Y", rotateLocal(grabFrame.rotate, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }), yAxis);
        ok &= expectVectorNear("grab frame local Z", rotateLocal(grabFrame.rotate, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }), zAxis);
    }

    {
        RE::NiTransform hand = identityTransform();
        hand.translate = RE::NiPoint3{ 10.0f, -5.0f, 2.0f };

        std::array<RE::NiPoint3, 5> fingerBases{
            RE::NiPoint3{ 18.0f, -7.0f, 2.0f },
            RE::NiPoint3{ 20.0f, -5.0f, 2.0f },
            RE::NiPoint3{ 20.0f, -3.0f, 2.0f },
            RE::NiPoint3{ 18.0f, -1.0f, 2.0f },
            RE::NiPoint3{ 16.0f, 1.0f, 2.0f },
        };
        const RE::NiPoint3 backDirection{ 0.0f, 0.0f, 1.0f };

        const auto palm =
            rock::hand_bone_collider_geometry_math::buildPalmAnchorFrame(hand, fingerBases, backDirection, 0.75f);
        if (!palm.valid) {
            std::printf("palm anchor frame was not valid\n");
            ok = false;
        } else {
            ok &= expectUnitAxes("palm anchor", palm.xAxis, palm.yAxis, palm.zAxis);
            ok &= expectVectorNear("palm collider column X", matrixColumn(palm.transform.rotate, 0), palm.xAxis);
            ok &= expectVectorNear("palm collider column Y", matrixColumn(palm.transform.rotate, 1), palm.yAxis);
            ok &= expectVectorNear("palm collider column Z", matrixColumn(palm.transform.rotate, 2), palm.zAxis);

            const RE::NiTransform grabPalm =
                rock::hand_bone_collider_geometry_math::generatedColliderFrameToGrabAuthorityFrame(palm.transform);
            ok &= expectVectorNear("palm grab row X", matrixRow(grabPalm.rotate, 0), palm.xAxis);
            ok &= expectVectorNear("palm grab row Y", matrixRow(grabPalm.rotate, 1), palm.yAxis);
            ok &= expectVectorNear("palm grab row Z", matrixRow(grabPalm.rotate, 2), palm.zAxis);
            ok &= expectVectorNear("palm grab local X", rotateLocal(grabPalm.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }), palm.xAxis);
            ok &= expectVectorNear("palm grab local Y", rotateLocal(grabPalm.rotate, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }), palm.yAxis);
            ok &= expectVectorNear("palm grab local Z", rotateLocal(grabPalm.rotate, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }), palm.zAxis);
        }
    }

    {
        RE::NiTransform start = identityTransform();
        RE::NiTransform end = identityTransform();
        start.translate = RE::NiPoint3{ -1.0f, 2.0f, 3.0f };
        end.translate = RE::NiPoint3{ 7.0f, 2.0f, 3.0f };

        rock::hand_bone_collider_geometry_math::BoneColliderFrameInput<RE::NiTransform, RE::NiPoint3> input{};
        input.start = start;
        input.end = end;
        input.radius = 0.5f;
        input.convexRadius = 0.1f;

        const auto segment = rock::hand_bone_collider_geometry_math::buildSegmentColliderFrame(input);
        if (!segment.valid) {
            std::printf("segment collider frame was not valid\n");
            ok = false;
        } else {
            ok &= expectVectorNear("segment collider column X", matrixColumn(segment.transform.rotate, 0), segment.xAxis);
            ok &= expectVectorNear("segment collider column Y", matrixColumn(segment.transform.rotate, 1), segment.yAxis);
            ok &= expectVectorNear("segment collider column Z", matrixColumn(segment.transform.rotate, 2), segment.zAxis);

            const RE::NiTransform grabSegment =
                rock::hand_bone_collider_geometry_math::generatedColliderFrameToGrabAuthorityFrame(segment.transform);
            ok &= expectVectorNear("segment grab row X", matrixRow(grabSegment.rotate, 0), segment.xAxis);
            ok &= expectVectorNear("segment grab row Y", matrixRow(grabSegment.rotate, 1), segment.yAxis);
            ok &= expectVectorNear("segment grab row Z", matrixRow(grabSegment.rotate, 2), segment.zAxis);
            ok &= expectVectorNear("segment grab local X", rotateLocal(grabSegment.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }), segment.xAxis);
            ok &= expectVectorNear("segment grab local Y", rotateLocal(grabSegment.rotate, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }), segment.yAxis);
            ok &= expectVectorNear("segment grab local Z", rotateLocal(grabSegment.rotate, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }), segment.zAxis);
        }
    }

    return ok ? 0 : 1;
}
