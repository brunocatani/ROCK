#pragma once

#include "TransformMath.h"

#include <cstdint>
#include <cstring>

namespace frik::rock::grab_constraint_math
{
    /*
     * HIGGS creates grab angular constraints in the object-to-hand frame and
     * refreshes the object-space transform-B pivot from that same frame during
     * every held update. ROCK updates transform A from the live physics hand, so
     * a frozen transform-B pivot makes the linear and angular goals disagree:
     * the pivot can be visually locked while the held body cannot rotate into
     * the desired frame. Keeping both target_bRca and transformB.translation
     * conversions here makes constraint creation and per-frame writes use one
     * verified convention.
     */

    /*
     * HIGGS constructs hkpSetupStabilizationAtom through Havok's constructor,
     * which leaves the atom disabled but initializes its impulse/angle clamps
     * to HK_REAL_MAX/HK_REAL_HIGH. ROCK allocates raw bytes for a FO4VR custom
     * atom chain, so those constructor defaults must be written explicitly.
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

    template <class Transform>
    inline void writeInitialGrabAngularFrame(float* transformBRotation, float* targetBRca, const Transform& desiredBodyTransformHandSpace)
    {
        const auto bodyToHandRotation = desiredBodyToHandRotation(desiredBodyTransformHandSpace.rotate);
        writeHavokRotationColumns(transformBRotation, bodyToHandRotation);
        writeHavokRotationColumns(targetBRca, bodyToHandRotation);
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
