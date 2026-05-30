#pragma once

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/hand/HandColliderTypes.h"

#include <cstdint>
#include <cstring>

namespace rock::grab_constraint_math
{
    /*
     * ROCK creates grab angular constraints in the object-to-hand frame and
     * refreshes the object-space transform-B pivot from that same frame during
     * every held update. Transform A comes from the live physics hand, so a
     * frozen transform-B pivot makes the linear and angular goals disagree: the
     * pivot can be visually locked while the held body cannot rotate into the
     * desired frame. Keeping transform-B rotation, transform-B translation,
     * and the ragdoll target initialization here makes constraint creation and
     * per-frame writes use one verified convention.
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
    inline void writeHavokRotationRows(float* target, const Matrix& rotation)
    {
        if (!target) {
            return;
        }

        target[0] = rotation.entry[0][0];
        target[1] = rotation.entry[0][1];
        target[2] = rotation.entry[0][2];
        target[3] = 0.0f;

        target[4] = rotation.entry[1][0];
        target[5] = rotation.entry[1][1];
        target[6] = rotation.entry[1][2];
        target[7] = 0.0f;

        target[8] = rotation.entry[2][0];
        target[9] = rotation.entry[2][1];
        target[10] = rotation.entry[2][2];
        target[11] = 0.0f;
    }

    template <class Transform, class Matrix>
    inline Transform rotationOnlyTransform(const Matrix& rotation)
    {
        Transform result = transform_math::makeIdentityTransform<Transform>();
        result.rotate = rotation;
        return result;
    }

    template <class Transform, class Matrix>
    inline Matrix frameToFrameRotation(const Matrix& fromWorldRotation, const Matrix& toWorldRotation)
    {
        const Transform fromWorld = rotationOnlyTransform<Transform>(fromWorldRotation);
        const Transform toWorld = rotationOnlyTransform<Transform>(toWorldRotation);
        return transform_math::composeTransforms(transform_math::invertTransform(fromWorld), toWorld).rotate;
    }

    template <class Transform, class Matrix>
    inline Matrix computeDesiredRagdollTargetBRca(const Transform& bodyAWorld,
        const Transform& desiredBodyWorld,
        const Matrix& transformARotation,
        const Matrix& transformBRotation)
    {
        const Matrix constraintAWorldRotation =
            transform_math::composeTransforms(bodyAWorld, rotationOnlyTransform<Transform>(transformARotation)).rotate;
        const Matrix constraintBWorldRotation =
            transform_math::composeTransforms(desiredBodyWorld, rotationOnlyTransform<Transform>(transformBRotation)).rotate;
        return frameToFrameRotation<Transform>(constraintBWorldRotation, constraintAWorldRotation);
    }

    template <class Transform>
    inline void writeInitialGrabAngularFrame(float* transformBRotation,
        float* targetBRca,
        const Transform& bodyAWorld,
        const Transform& desiredBodyWorld,
        const Transform& desiredBodyTransformHandSpace)
    {
        /*
         * Transform-B still defines body B's local frame relative to the
         * generated proxy, but target_bRca is not that same body-to-proxy
         * relation. FO4VR's ragdoll motor consumes the target in the effective
         * constraint A/B frame after transform-A and transform-B are applied.
         * Feed the residual BRca that makes the desired BODY pose satisfy those
         * frames; identity is wrong whenever the transform-B storage convention
         * leaves a nonzero residual.
         */
        auto bodyToHandRotation = desiredBodyToHandRotation(desiredBodyTransformHandSpace.rotate);
        const auto transformARotation = transform_math::makeIdentityRotation<decltype(bodyToHandRotation)>();
        const auto targetBRcaRotation =
            computeDesiredRagdollTargetBRca(bodyAWorld, desiredBodyWorld, transformARotation, bodyToHandRotation);
        writeHavokRotationColumns(transformBRotation, bodyToHandRotation);
        writeHavokRotationRows(targetBRca, targetBRcaRotation);
    }

    template <class Transform, class Vector>
    inline Vector computeGeneratedProxyConstraintPivotLocalGame(const Transform& bodyWorld, const Vector& pivotWorld)
    {
        return hand_bone_collider_geometry_math::generatedColliderWorldPointToLocal(bodyWorld, pivotWorld);
    }

    template <class Transform, class Vector>
    inline void writeGeneratedProxyConstraintPivotLocalTranslation(float* constraintTranslation,
        const Transform& bodyWorld,
        const Vector& pivotWorld,
        float gameToHavokScale)
    {
        if (!constraintTranslation) {
            return;
        }

        const Vector pivotLocalGame = computeGeneratedProxyConstraintPivotLocalGame(bodyWorld, pivotWorld);
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
