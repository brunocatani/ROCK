#pragma once

#include "physics-interaction/TransformMath.h"

#include <cstdint>
#include <cstring>

namespace rock::grab_constraint_math
{
    /*
     * Dynamic grab splits the authority frame deliberately: the generated
     * palm/proxy collider owns the linear pivot, while raw root-flattened hand
     * rotation owns angular intent. FO4VR's ragdoll target updater composes the
     * incoming body-to-authority target through transform-A rotation before
     * writing target_bRca, so ROCK keeps transform-A, transform-B, and target
     * writes separate. That prevents the hidden collider's geometry axes from
     * becoming object rotation authority while preserving the selected contact
     * pivot as the linear grip point.
     */

    /*
     * ROCK allocates raw bytes for a FO4VR custom atom chain, so constructor
     * defaults that Havok normally applies must be written explicitly. The setup
     * stabilization atom starts disabled but needs its impulse/angle clamps
     * initialized to HK_REAL_MAX/HK_REAL_HIGH.
     */
    inline constexpr std::uint32_t kHavokRealMaxBits = 0x7f7fffeeu;
    inline constexpr std::uint32_t kHavokRealHighBits = 0x5f7ffff0u;

    inline void writeSetupStabilizationDefaults(void* setupAtom)
    {
        if (!setupAtom) {
            return;
        }

        auto* bytes = static_cast<unsigned char*>(setupAtom);
        bytes[2] = 0;
        bytes[3] = 0;
        std::memcpy(bytes + 4, &kHavokRealMaxBits, sizeof(kHavokRealMaxBits));
        std::memcpy(bytes + 8, &kHavokRealMaxBits, sizeof(kHavokRealMaxBits));
        std::memcpy(bytes + 12, &kHavokRealHighBits, sizeof(kHavokRealHighBits));
    }

    template <class Matrix>
    inline Matrix desiredBodyToBodyARotation(const Matrix& desiredBodyTransformBodyASpaceRotation)
    {
        return transform_math::transposeRotation(desiredBodyTransformBodyASpaceRotation);
    }

    template <class Matrix>
    inline void writeHavokRotationColumns(float* target, const Matrix& rotation)
    {
        if (!target) {
            return;
        }

        target[0] = rotation.entry[0][0];
        target[1] = rotation.entry[1][0];
        target[2] = rotation.entry[2][0];
        target[3] = 0.0f;

        target[4] = rotation.entry[0][1];
        target[5] = rotation.entry[1][1];
        target[6] = rotation.entry[2][1];
        target[7] = 0.0f;

        target[8] = rotation.entry[0][2];
        target[9] = rotation.entry[1][2];
        target[10] = rotation.entry[2][2];
        target[11] = 0.0f;
    }

    template <class Matrix>
    inline void writeGrabTransformARotation(float* transformARotation, const Matrix& transformALocalRotation)
    {
        writeHavokRotationColumns(transformARotation, transformALocalRotation);
    }

    template <class Transform>
    inline void writeInitialGrabTransformBRotation(float* transformBRotation, const Transform& desiredBodyTransformBodyASpace)
    {
        const auto bodyToBodyARotation = desiredBodyToBodyARotation(desiredBodyTransformBodyASpace.rotate);
        writeHavokRotationColumns(transformBRotation, bodyToBodyARotation);
    }

    template <class Matrix>
    inline Matrix composeRagdollAngularTargetRotation(const Matrix& bodyToBodyARotation, const Matrix& transformALocalRotation)
    {
        return transform_math::multiplyStoredRotations(bodyToBodyARotation, transformALocalRotation);
    }

    template <class Transform, class Matrix>
    inline void writeGrabRagdollAngularTarget(float* targetBRca,
        const Transform& desiredBodyTransformBodyASpace,
        const Matrix& transformALocalRotation)
    {
        if (!targetBRca) {
            return;
        }

        const auto bodyToBodyARotation = desiredBodyToBodyARotation(desiredBodyTransformBodyASpace.rotate);
        const auto targetRotation = composeRagdollAngularTargetRotation(bodyToBodyARotation, transformALocalRotation);
        writeHavokRotationColumns(targetBRca, targetRotation);
    }

    template <class Transform, class Vector>
    inline Vector computeConstraintPivotLocalGame(const Transform& bodyWorld, const Vector& pivotWorld)
    {
        return transform_math::worldPointToLocal(bodyWorld, pivotWorld);
    }

    template <class Transform, class Vector>
    inline void writeConstraintPivotLocalTranslation(float* constraintTranslation,
        const Transform& bodyWorld,
        const Vector& pivotWorld,
        float gameToHavokScale)
    {
        if (!constraintTranslation) {
            return;
        }

        const Vector pivotLocalGame = computeConstraintPivotLocalGame(bodyWorld, pivotWorld);
        constraintTranslation[0] = pivotLocalGame.x * gameToHavokScale;
        constraintTranslation[1] = pivotLocalGame.y * gameToHavokScale;
        constraintTranslation[2] = pivotLocalGame.z * gameToHavokScale;
        constraintTranslation[3] = 0.0f;
    }

    template <class Transform, class Vector>
    inline Vector computeDynamicTransformBTranslationGame(const Transform& desiredBodyTransformHandSpace, const Vector& pivotAHandBodyLocalGame)
    {
        return transform_math::localPointToWorld(transform_math::invertTransform(desiredBodyTransformHandSpace), pivotAHandBodyLocalGame);
    }

    template <class Transform, class Vector>
    inline void writeDynamicTransformBTranslation(float* transformBTranslation,
        const Transform& desiredBodyTransformHandSpace,
        const Vector& pivotAHandBodyLocalGame,
        float gameToHavokScale)
    {
        if (!transformBTranslation) {
            return;
        }

        const Vector transformBTranslationGame = computeDynamicTransformBTranslationGame(desiredBodyTransformHandSpace, pivotAHandBodyLocalGame);
        transformBTranslation[0] = transformBTranslationGame.x * gameToHavokScale;
        transformBTranslation[1] = transformBTranslationGame.y * gameToHavokScale;
        transformBTranslation[2] = transformBTranslationGame.z * gameToHavokScale;
        transformBTranslation[3] = 0.0f;
    }
}
