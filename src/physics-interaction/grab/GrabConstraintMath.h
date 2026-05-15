#pragma once

#include "physics-interaction/TransformMath.h"

#include <cstdint>
#include <cstring>

namespace rock::grab_constraint_math
{
    /*
     * Dynamic grab uses the proxy/palm collider as body-A linear anchor, but
     * raw controller / flattened-root hand rotation as angular authority.
     * FO4VR's native ragdoll target updater composes target_bRca with
     * transform-A rotation, so the transform-A, transform-B, and target writes
     * stay split here. That prevents the held object from being driven by the
     * proxy collider's geometry axes while preserving the selected contact
     * pivot as the linear grip authority.
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
    inline Matrix desiredBodyToHandRotation(const Matrix& desiredBodyTransformHandSpaceRotation)
    {
        return transform_math::transposeRotation(desiredBodyTransformHandSpaceRotation);
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
    inline void writeInitialGrabTransformBRotation(float* transformBRotation, const Transform& desiredBodyTransformAuthoritySpace)
    {
        const auto bodyToAuthorityRotation = desiredBodyToHandRotation(desiredBodyTransformAuthoritySpace.rotate);
        writeHavokRotationColumns(transformBRotation, bodyToAuthorityRotation);
    }

    template <class Matrix>
    inline Matrix composeRagdollAngularTargetRotation(const Matrix& bodyToAuthorityRotation, const Matrix& transformALocalRotation)
    {
        /*
         * Ghidra-confirmed FO4VR updater:
         *   0x1419B26C0 -> 0x1417CF420(target_bRca, bRa, transformA.rotation)
         *
         * This mirrors HIGGS' symbolic hkp call:
         *   target_bRca = bRa * transformA.rotation
         *
         * ROCK keeps this as a matrix helper instead of rewriting the raw bytes
         * by hand at each call site.
         */
        return transform_math::multiplyStoredRotations(bodyToAuthorityRotation, transformALocalRotation);
    }

    template <class Transform, class Matrix>
    inline void writeGrabRagdollAngularTarget(float* targetBRca,
        const Transform& desiredBodyTransformAuthoritySpace,
        const Matrix& transformALocalRotation)
    {
        if (!targetBRca) {
            return;
        }

        const auto bodyToAuthorityRotation = desiredBodyToHandRotation(desiredBodyTransformAuthoritySpace.rotate);
        const auto targetRotation = composeRagdollAngularTargetRotation(bodyToAuthorityRotation, transformALocalRotation);
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
