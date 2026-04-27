#pragma once

#include "TransformMath.h"

#include <algorithm>
#include <cmath>

namespace frik::rock
{
    /*
     * FRIK's suppressed offhand grip works because it solves the weapon
     * transform from two points: the fixed primary grip and the offhand support
     * point. ROCK uses generated mesh collision, so this solver returns one
     * coherent weapon-root transform that can be applied to both the visible
     * weapon node and every generated collision body.
     */

    template <class Transform, class Vector>
    struct WeaponTwoHandedSolverInput
    {
        Transform weaponWorldTransform{};
        Vector primaryGripLocal{};
        Vector supportGripLocal{};
        Vector primaryTargetWorld{};
        Vector supportTargetWorld{};
        Vector supportNormalLocal{};
        Vector supportNormalTargetWorld{};
        float minimumSeparation{ 0.001f };
        float supportNormalTwistFactor{ 0.0f };
        bool useSupportNormalTwist{ false };
    };

    template <class Transform>
    struct WeaponTwoHandedSolverResult
    {
        Transform weaponWorldTransform{};
        bool solved{ false };
        float primaryError{ 0.0f };
        float supportError{ 0.0f };
    };

    template <class Vector>
    inline Vector weaponSolverSub(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
    }

    template <class Vector>
    inline Vector weaponSolverAdd(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
    }

    template <class Vector>
    inline Vector weaponSolverScale(const Vector& vector, float scale)
    {
        return Vector{ vector.x * scale, vector.y * scale, vector.z * scale };
    }

    template <class Vector>
    inline float weaponSolverDot(const Vector& lhs, const Vector& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vector>
    inline Vector weaponSolverCross(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x };
    }

    template <class Vector>
    inline float weaponSolverLength(const Vector& vector)
    {
        return std::sqrt(weaponSolverDot(vector, vector));
    }

    template <class Vector>
    inline Vector weaponSolverNormalize(const Vector& vector)
    {
        const float length = weaponSolverLength(vector);
        if (length <= 0.000001f) {
            return Vector{};
        }
        return weaponSolverScale(vector, 1.0f / length);
    }

    template <class Matrix, class Vector>
    inline Matrix weaponSolverStoredRotationFromConventionalRows(const Vector rows[3])
    {
        Matrix result{};
        for (int row = 0; row < 3; ++row) {
            result.entry[row][0] = rows[0].x;
            result.entry[row][1] = rows[1].x;
            result.entry[row][2] = rows[2].x;
        }
        result.entry[0][0] = rows[0].x;
        result.entry[0][1] = rows[1].x;
        result.entry[0][2] = rows[2].x;
        result.entry[1][0] = rows[0].y;
        result.entry[1][1] = rows[1].y;
        result.entry[1][2] = rows[2].y;
        result.entry[2][0] = rows[0].z;
        result.entry[2][1] = rows[1].z;
        result.entry[2][2] = rows[2].z;
        return result;
    }

    template <class Vector>
    inline Vector weaponSolverOrthogonalAxis(const Vector& value)
    {
        Vector axis = std::abs(value.x) < 0.7f ? Vector{ 1.0f, 0.0f, 0.0f } : Vector{ 0.0f, 1.0f, 0.0f };
        return weaponSolverNormalize(weaponSolverCross(value, axis));
    }

    template <class Matrix, class Vector>
    inline Matrix weaponSolverRotationBetweenStored(const Vector& fromRaw, const Vector& toRaw)
    {
        const Vector from = weaponSolverNormalize(fromRaw);
        const Vector to = weaponSolverNormalize(toRaw);
        float cosTheta = (std::max)(-1.0f, (std::min)(1.0f, weaponSolverDot(from, to)));

        if (cosTheta > 0.9999f) {
            return transform_math::makeIdentityRotation<Matrix>();
        }

        Vector axis{};
        float sinTheta = 0.0f;
        if (cosTheta < -0.9999f) {
            axis = weaponSolverOrthogonalAxis(from);
            sinTheta = 0.0f;
            cosTheta = -1.0f;
        } else {
            axis = weaponSolverNormalize(weaponSolverCross(from, to));
            sinTheta = std::sqrt((std::max)(0.0f, 1.0f - cosTheta * cosTheta));
        }

        const float x = axis.x;
        const float y = axis.y;
        const float z = axis.z;
        const float oneMinusCos = 1.0f - cosTheta;

        Vector conventionalRows[3]{
            Vector{ cosTheta + x * x * oneMinusCos, x * y * oneMinusCos - z * sinTheta, x * z * oneMinusCos + y * sinTheta },
            Vector{ y * x * oneMinusCos + z * sinTheta, cosTheta + y * y * oneMinusCos, y * z * oneMinusCos - x * sinTheta },
            Vector{ z * x * oneMinusCos - y * sinTheta, z * y * oneMinusCos + x * sinTheta, cosTheta + z * z * oneMinusCos },
        };

        return weaponSolverStoredRotationFromConventionalRows<Matrix, Vector>(conventionalRows);
    }

    template <class Matrix, class Vector>
    inline Matrix weaponSolverAxisAngleStored(const Vector& axisRaw, float angle)
    {
        const Vector axis = weaponSolverNormalize(axisRaw);
        const float x = axis.x;
        const float y = axis.y;
        const float z = axis.z;
        const float cosTheta = std::cos(angle);
        const float sinTheta = std::sin(angle);
        const float oneMinusCos = 1.0f - cosTheta;

        Vector conventionalRows[3]{
            Vector{ cosTheta + x * x * oneMinusCos, x * y * oneMinusCos - z * sinTheta, x * z * oneMinusCos + y * sinTheta },
            Vector{ y * x * oneMinusCos + z * sinTheta, cosTheta + y * y * oneMinusCos, y * z * oneMinusCos - x * sinTheta },
            Vector{ z * x * oneMinusCos - y * sinTheta, z * y * oneMinusCos + x * sinTheta, cosTheta + z * z * oneMinusCos },
        };

        return weaponSolverStoredRotationFromConventionalRows<Matrix, Vector>(conventionalRows);
    }

    template <class Matrix, class Vector>
    inline Matrix weaponSolverApplyWorldRotationToStoredBasis(const Matrix& worldRotationStored, const Matrix& baseRotation)
    {
        Matrix result{};
        const Vector basis[3]{
            Vector{ baseRotation.entry[0][0], baseRotation.entry[0][1], baseRotation.entry[0][2] },
            Vector{ baseRotation.entry[1][0], baseRotation.entry[1][1], baseRotation.entry[1][2] },
            Vector{ baseRotation.entry[2][0], baseRotation.entry[2][1], baseRotation.entry[2][2] },
        };

        for (int axis = 0; axis < 3; ++axis) {
            const Vector rotated = transform_math::rotateLocalVectorToWorld(worldRotationStored, basis[axis]);
            result.entry[axis][0] = rotated.x;
            result.entry[axis][1] = rotated.y;
            result.entry[axis][2] = rotated.z;
        }
        return result;
    }

    template <class Matrix, class Vector>
    inline Vector weaponSolverApplyStoredWorldRotationToVector(const Matrix& worldRotationStored, const Vector& vector)
    {
        return transform_math::rotateLocalVectorToWorld(worldRotationStored, vector);
    }

    template <class Vector>
    inline Vector weaponSolverProjectOntoPlane(const Vector& vector, const Vector& planeNormal)
    {
        const float normalDot = weaponSolverDot(vector, planeNormal);
        return weaponSolverSub(vector, weaponSolverScale(planeNormal, normalDot));
    }

    template <class Transform, class Vector>
    inline WeaponTwoHandedSolverResult<Transform> solveTwoHandedWeaponTransform(const WeaponTwoHandedSolverInput<Transform, Vector>& input)
    {
        WeaponTwoHandedSolverResult<Transform> result{};
        result.weaponWorldTransform = input.weaponWorldTransform;

        const Vector localAxis = weaponSolverSub(input.supportGripLocal, input.primaryGripLocal);
        const Vector currentAxisWorld = transform_math::localVectorToWorld(input.weaponWorldTransform, localAxis);
        const Vector desiredAxisWorld = weaponSolverSub(input.supportTargetWorld, input.primaryTargetWorld);

        if (weaponSolverLength(currentAxisWorld) <= input.minimumSeparation || weaponSolverLength(desiredAxisWorld) <= input.minimumSeparation) {
            return result;
        }

        const auto rotationDelta = weaponSolverRotationBetweenStored<decltype(input.weaponWorldTransform.rotate), Vector>(currentAxisWorld, desiredAxisWorld);
        result.weaponWorldTransform.rotate =
            weaponSolverApplyWorldRotationToStoredBasis<decltype(input.weaponWorldTransform.rotate), Vector>(rotationDelta, input.weaponWorldTransform.rotate);

        const Vector primaryAfterRotation = transform_math::localPointToWorld(result.weaponWorldTransform, input.primaryGripLocal);
        const Vector primaryCorrection = weaponSolverSub(input.primaryTargetWorld, primaryAfterRotation);
        result.weaponWorldTransform.translate = weaponSolverAdd(result.weaponWorldTransform.translate, primaryCorrection);

        const Vector primaryWorld = transform_math::localPointToWorld(result.weaponWorldTransform, input.primaryGripLocal);
        const Vector supportWorld = transform_math::localPointToWorld(result.weaponWorldTransform, input.supportGripLocal);
        result.primaryError = weaponSolverLength(weaponSolverSub(primaryWorld, input.primaryTargetWorld));
        result.supportError = weaponSolverLength(weaponSolverSub(supportWorld, input.supportTargetWorld));

        if (input.useSupportNormalTwist && input.supportNormalTwistFactor > 0.0f) {
            const Vector twistAxis = weaponSolverNormalize(weaponSolverSub(input.supportTargetWorld, input.primaryTargetWorld));
            const Vector currentNormalWorld = transform_math::localVectorToWorld(result.weaponWorldTransform, input.supportNormalLocal);
            const Vector desiredNormalWorld = input.supportNormalTargetWorld;
            const Vector currentProjected = weaponSolverNormalize(weaponSolverProjectOntoPlane(currentNormalWorld, twistAxis));
            const Vector desiredProjected = weaponSolverNormalize(weaponSolverProjectOntoPlane(desiredNormalWorld, twistAxis));

            if (weaponSolverLength(currentProjected) > input.minimumSeparation && weaponSolverLength(desiredProjected) > input.minimumSeparation) {
                const float dotValue = (std::max)(-1.0f, (std::min)(1.0f, weaponSolverDot(currentProjected, desiredProjected)));
                const Vector crossValue = weaponSolverCross(currentProjected, desiredProjected);
                const float signedAngle = std::atan2(weaponSolverDot(twistAxis, crossValue), dotValue) * input.supportNormalTwistFactor;
                const auto twistRotation = weaponSolverAxisAngleStored<decltype(input.weaponWorldTransform.rotate), Vector>(twistAxis, signedAngle);

                const Vector supportPivot = input.supportTargetWorld;
                const Vector pivotToWeapon = weaponSolverSub(result.weaponWorldTransform.translate, supportPivot);
                const Vector rotatedPivotToWeapon = weaponSolverApplyStoredWorldRotationToVector<decltype(input.weaponWorldTransform.rotate), Vector>(twistRotation, pivotToWeapon);
                result.weaponWorldTransform.translate = weaponSolverAdd(supportPivot, rotatedPivotToWeapon);
                result.weaponWorldTransform.rotate =
                    weaponSolverApplyWorldRotationToStoredBasis<decltype(input.weaponWorldTransform.rotate), Vector>(twistRotation, result.weaponWorldTransform.rotate);

                const Vector primaryAfterTwist = transform_math::localPointToWorld(result.weaponWorldTransform, input.primaryGripLocal);
                const Vector primaryTwistCorrection = weaponSolverSub(input.primaryTargetWorld, primaryAfterTwist);
                result.weaponWorldTransform.translate = weaponSolverAdd(result.weaponWorldTransform.translate, primaryTwistCorrection);

                const Vector primaryFinal = transform_math::localPointToWorld(result.weaponWorldTransform, input.primaryGripLocal);
                const Vector supportFinal = transform_math::localPointToWorld(result.weaponWorldTransform, input.supportGripLocal);
                result.primaryError = weaponSolverLength(weaponSolverSub(primaryFinal, input.primaryTargetWorld));
                result.supportError = weaponSolverLength(weaponSolverSub(supportFinal, input.supportTargetWorld));
            }
        }

        result.solved = true;
        return result;
    }
}
