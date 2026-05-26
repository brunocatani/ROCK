#pragma once

#include "physics-interaction/TransformMath.h"

#include <cstdint>
#include <cstring>

namespace rock::grab_constraint_math
{
    /*
     * ROCK writes the raw hand authority frame into both constraint local frames:
     * transform A is proxyBody^-1 * rawAuthority and transform B is
     * desiredBody^-1 * rawAuthority. FO4VR's ragdoll motor builder transforms
     * target_bRca rows from the base body-B frame, not from transform B's
     * already-localized frame, so the target rows must carry the same B-local
     * raw-authority rotation that transform B carries as hk column blocks.
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

    template <class Transform>
    inline void writeRagdollTargetForConstraintBFrame(float* targetBRca, const Transform& constraintBInBodyBSpace)
    {
        writeHavokRotationRows(targetBRca, constraintBInBodyBSpace.rotate);
    }

    template <class Transform>
    inline Transform computeConstraintFrameInBodySpace(const Transform& bodyWorld, const Transform& authorityFrameWorld)
    {
        return transform_math::composeTransforms(transform_math::invertTransform(bodyWorld), authorityFrameWorld);
    }

    template <class Transform>
    inline Transform computeDesiredBodyWorld(const Transform& authorityFrameWorld, const Transform& bodyInAuthorityFrame)
    {
        return transform_math::composeTransforms(authorityFrameWorld, bodyInAuthorityFrame);
    }

    template <class Transform>
    inline Transform computeDesiredObjectWorld(const Transform& authorityFrameWorld, const Transform& objectInAuthorityFrame)
    {
        return transform_math::composeTransforms(authorityFrameWorld, objectInAuthorityFrame);
    }

    template <class Transform>
    inline void writeConstraintLocalTransform(float* transformRotation, float* transformTranslation, const Transform& frameInBodySpace, float gameToHavokScale)
    {
        writeHavokRotationColumns(transformRotation, frameInBodySpace.rotate);
        if (!transformTranslation) {
            return;
        }

        transformTranslation[0] = frameInBodySpace.translate.x * gameToHavokScale;
        transformTranslation[1] = frameInBodySpace.translate.y * gameToHavokScale;
        transformTranslation[2] = frameInBodySpace.translate.z * gameToHavokScale;
        transformTranslation[3] = 0.0f;
    }

}
