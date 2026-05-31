#pragma once

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/hand/HandColliderTypes.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace rock::grab_constraint_math
{
    /*
     * ROCK's proxy grab keeps the HIGGS constraint relationship while preserving
     * the FO4VR byte-storage convention that was proven locally. Transform A is
     * the frozen proxy-local palm pivot with identity rotation. The transform-B
     * rotation is intentionally mode-selectable while investigating FO4VR's
     * ragdoll atom composition. Runtime testing showed two stable classes:
     * low-COL grabs need the relation in both transform-B and target_bRca,
     * while high-COL grabs need neutral transform-B with target_bRca carrying
     * the relation. Auto mode resolves that split once from the frozen
     * proxy-in-BODY column delta and keeps the concrete mode until release.
     */

    /*
     * ROCK allocates raw bytes for a FO4VR custom atom chain, so constructor
     * defaults that Havok normally applies must be written explicitly. The setup
     * stabilization atom starts disabled but needs its impulse/angle clamps
     * initialized to HK_REAL_MAX/HK_REAL_HIGH.
     */
    inline constexpr std::uint32_t kHavokRealMaxBits = 0x7f7fffeeu;
    inline constexpr std::uint32_t kHavokRealHighBits = 0x5f7ffff0u;

    inline constexpr int kGrabRagdollDecompositionModeAuto = -1;
    inline constexpr int kGrabRagdollDecompositionModeRelationTransformB = 0;
    inline constexpr int kGrabRagdollDecompositionModeNeutralTransformB = 1;
    inline constexpr int kDefaultGrabRagdollDecompositionConfigMode = kGrabRagdollDecompositionModeAuto;
    inline constexpr int kDefaultGrabRagdollDecompositionResolvedMode = kGrabRagdollDecompositionModeNeutralTransformB;

    /*
     * This threshold is an empirical guardrail, not a permanent model of Havok's
     * ragdoll motor math. The in-game failure is born at capture and is
     * hand-independent: the same object face swaps between the two working
     * decompositions around a 90 degree COL split. Keep this capture-time
     * selector until the missing FO4VR atom composition step is proven and can
     * replace both decomposition modes with one correct write path.
     */
    inline constexpr float kGrabRagdollDecompositionAutoThresholdDegrees = 90.0f;

    inline int sanitizeGrabRagdollDecompositionConfigMode(int mode) noexcept
    {
        switch (mode) {
        case kGrabRagdollDecompositionModeAuto:
        case kGrabRagdollDecompositionModeRelationTransformB:
        case kGrabRagdollDecompositionModeNeutralTransformB:
            return mode;
        default:
            return kDefaultGrabRagdollDecompositionConfigMode;
        }
    }

    inline int sanitizeGrabRagdollDecompositionMode(int mode) noexcept
    {
        switch (mode) {
        case kGrabRagdollDecompositionModeRelationTransformB:
        case kGrabRagdollDecompositionModeNeutralTransformB:
            return mode;
        default:
            return kDefaultGrabRagdollDecompositionResolvedMode;
        }
    }

    inline const char* grabRagdollDecompositionModeName(int mode) noexcept
    {
        switch (mode) {
        case kGrabRagdollDecompositionModeAuto:
            return "auto";
        case kGrabRagdollDecompositionModeRelationTransformB:
            return "relationTransformB";
        case kGrabRagdollDecompositionModeNeutralTransformB:
            return "neutralTransformB";
        default:
            return "invalid";
        }
    }

    template <class Matrix>
    inline float rotationDeltaDegrees(const Matrix& a, const Matrix& b)
    {
        const float trace =
            (a.entry[0][0] * b.entry[0][0] + a.entry[1][0] * b.entry[1][0] + a.entry[2][0] * b.entry[2][0]) +
            (a.entry[0][1] * b.entry[0][1] + a.entry[1][1] * b.entry[1][1] + a.entry[2][1] * b.entry[2][1]) +
            (a.entry[0][2] * b.entry[0][2] + a.entry[1][2] * b.entry[1][2] + a.entry[2][2] * b.entry[2][2]);
        const float cosTheta = std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f);
        return std::acos(cosTheta) * (180.0f / 3.14159265358979323846f);
    }

    template <class Matrix>
    inline float computeGrabRagdollDecompositionColumnDeltaDegrees(const Matrix& proxyInBodyRotation)
    {
        return rotationDeltaDegrees(proxyInBodyRotation, transform_math::transposeRotation(proxyInBodyRotation));
    }

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
    inline Matrix proxyInBodyRotationFromBodyInProxyRotation(const Matrix& bodyInProxyRotation)
    {
        return transform_math::transposeRotation(bodyInProxyRotation);
    }

    template <class Matrix>
    inline Matrix desiredBodyToHandRotation(const Matrix& desiredBodyTransformHandSpaceRotation)
    {
        return proxyInBodyRotationFromBodyInProxyRotation(desiredBodyTransformHandSpaceRotation);
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

    template <class Transform>
    inline Transform proxyInBodyFromBodyInProxy(const Transform& bodyInProxy)
    {
        return transform_math::invertTransform(bodyInProxy);
    }

    template <class Transform>
    inline int resolveGrabRagdollDecompositionMode(int configMode, const Transform& bodyInProxy, float* outColumnDeltaDegrees = nullptr)
    {
        const Transform proxyInBody = proxyInBodyFromBodyInProxy(bodyInProxy);
        const float columnDeltaDegrees = computeGrabRagdollDecompositionColumnDeltaDegrees(proxyInBody.rotate);
        if (outColumnDeltaDegrees) {
            *outColumnDeltaDegrees = columnDeltaDegrees;
        }

        const int sanitizedConfigMode = sanitizeGrabRagdollDecompositionConfigMode(configMode);
        if (sanitizedConfigMode != kGrabRagdollDecompositionModeAuto) {
            return sanitizeGrabRagdollDecompositionMode(sanitizedConfigMode);
        }

        // The mode must be chosen from the frozen grab relation, not from live
        // held-update telemetry. A good grab stays good and a bad grab starts bad,
        // so per-frame mode changes would just move the solver target while held.
        return columnDeltaDegrees > kGrabRagdollDecompositionAutoThresholdDegrees ?
                   kGrabRagdollDecompositionModeNeutralTransformB :
                   kGrabRagdollDecompositionModeRelationTransformB;
    }

    template <class Transform>
    inline auto proxyInBodyRotationFromBodyInProxy(const Transform& bodyInProxy)
    {
        return proxyInBodyRotationFromBodyInProxyRotation(bodyInProxy.rotate);
    }

    template <class Transform, class Vector>
    inline Vector computeHiggsTransformBTranslationGameFromProxyInBody(const Transform& proxyInBody, const Vector& frozenPivotAProxyLocalGame)
    {
        return transform_math::localPointToWorld(proxyInBody, frozenPivotAProxyLocalGame);
    }

    template <class Transform, class Vector>
    inline Vector computeHiggsTransformBTranslationGame(const Transform& bodyInProxy, const Vector& frozenPivotAProxyLocalGame)
    {
        return computeHiggsTransformBTranslationGameFromProxyInBody(proxyInBodyFromBodyInProxy(bodyInProxy), frozenPivotAProxyLocalGame);
    }

    template <class Transform>
    inline void writeGrabConstraintAngularDecomposition(float* transformBRotation,
        float* targetBRca,
        const Transform& proxyInBody,
        int decompositionMode)
    {
        const int mode = sanitizeGrabRagdollDecompositionMode(decompositionMode);
        const auto identityRotation = transform_math::makeIdentityRotation<decltype(proxyInBody.rotate)>();
        const auto& transformBRelation =
            mode == kGrabRagdollDecompositionModeRelationTransformB ? proxyInBody.rotate : identityRotation;

        writeHavokRotationColumns(transformBRotation, transformBRelation);
        writeHavokRotationRows(targetBRca, proxyInBody.rotate);
    }

    template <class Transform, class Vector>
    inline void writeGrabConstraintHeldTargetAtoms(float* transformBRotation,
        float* transformBTranslation,
        float* targetBRca,
        const Transform& bodyInProxy,
        const Vector& frozenPivotAProxyLocalGame,
        float gameToHavokScale,
        int decompositionMode)
    {
        const Transform proxyInBody = proxyInBodyFromBodyInProxy(bodyInProxy);
        writeGrabConstraintAngularDecomposition(transformBRotation, targetBRca, proxyInBody, decompositionMode);

        if (transformBTranslation) {
            const Vector transformBTranslationGame =
                computeHiggsTransformBTranslationGameFromProxyInBody(proxyInBody, frozenPivotAProxyLocalGame);
            transformBTranslation[0] = transformBTranslationGame.x * gameToHavokScale;
            transformBTranslation[1] = transformBTranslationGame.y * gameToHavokScale;
            transformBTranslation[2] = transformBTranslationGame.z * gameToHavokScale;
            transformBTranslation[3] = 0.0f;
        }
    }

    template <class Transform, class Vector>
    inline void writeGrabConstraintCreationAtoms(float* transformBRotation,
        float* transformBTranslation,
        float* targetBRca,
        const Transform& bodyInProxyAtCreation,
        const Vector& frozenPivotAProxyLocalGame,
        float gameToHavokScale,
        int decompositionMode)
    {
        const Transform proxyInBody = proxyInBodyFromBodyInProxy(bodyInProxyAtCreation);
        writeGrabConstraintAngularDecomposition(transformBRotation, targetBRca, proxyInBody, decompositionMode);

        if (transformBTranslation) {
            const Vector transformBTranslationGame =
                computeHiggsTransformBTranslationGameFromProxyInBody(proxyInBody, frozenPivotAProxyLocalGame);
            transformBTranslation[0] = transformBTranslationGame.x * gameToHavokScale;
            transformBTranslation[1] = transformBTranslationGame.y * gameToHavokScale;
            transformBTranslation[2] = transformBTranslationGame.z * gameToHavokScale;
            transformBTranslation[3] = 0.0f;
        }
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
        return computeHiggsTransformBTranslationGame(desiredBodyTransformHandSpace, pivotAHandBodyLocalGame);
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
