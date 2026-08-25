#pragma once

#include "physics-interaction/hand/Hand.h"

#include <cmath>

namespace rock::hand_grab_internal
{
    struct HeldBodyMassSummary
    {
        float primaryMass = 0.0f;
        float aggregateMass = 0.0f;
        std::uint32_t sampledBodies = 0;
        std::uint32_t uniqueMotions = 0;

        [[nodiscard]] float motorMass() const noexcept
        {
            if (std::isfinite(aggregateMass) && aggregateMass > 0.0f) {
                return aggregateMass;
            }
            return (std::isfinite(primaryMass) && primaryMass > 0.0f) ? primaryMass : 0.0f;
        }
    };

    [[nodiscard]] RE::NiPoint3 normalizeOrZero(const RE::NiPoint3& value);
    [[nodiscard]] float lengthSquared(const RE::NiPoint3& value);
    [[nodiscard]] RE::NiPoint3 crossProduct(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs);
    [[nodiscard]] float dotProduct(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs);
    [[nodiscard]] RE::NiPoint3 stablePerpendicularAxis(const RE::NiPoint3& normal);
    [[nodiscard]] float vectorMagnitude(const RE::NiPoint3& value);
    [[nodiscard]] float pointDistanceGameUnits(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs);
    [[nodiscard]] float translationDeltaGameUnits(const RE::NiTransform& lhs, const RE::NiTransform& rhs);
    [[nodiscard]] float rotationDeltaDegrees(const RE::NiMatrix3& lhs, const RE::NiMatrix3& rhs);
    [[nodiscard]] RE::NiPoint3 rotationCorrectionAxisWorld(const RE::NiMatrix3& current, const RE::NiMatrix3& target);

    [[nodiscard]] RE::NiMatrix3 matrixFromHkColumns(const float* hkMatrix);
    [[nodiscard]] RE::NiMatrix3 matrixFromHkRows(const float* hkMatrix);
    [[nodiscard]] RE::NiTransform makeIdentityTransform();
    [[nodiscard]] RE::NiTransform invertTransform(const RE::NiTransform& transform);
    [[nodiscard]] RE::NiTransform multiplyTransforms(const RE::NiTransform& parent, const RE::NiTransform& child);
    [[nodiscard]] RE::NiTransform deriveNodeWorldFromBodyWorld(
        const RE::NiTransform& bodyWorld,
        const RE::NiTransform& bodyLocalTransform);
    [[nodiscard]] RE::NiTransform reconstructBodyWorldFromProxyInBody(
        const RE::NiTransform& proxyWorld,
        const RE::NiMatrix3& proxyInBodyRotation,
        const RE::NiPoint3& transformBLocalGame,
        const RE::NiPoint3& pivotAProxyLocalGame);
    [[nodiscard]] RE::NiTransform reconstructSolverEffectiveBodyWorld(
        const RE::NiTransform& proxyWorld,
        const RE::NiMatrix3& transformARotation,
        const RE::NiMatrix3& transformBRotation,
        const RE::NiMatrix3& targetBRcaRotation,
        const RE::NiPoint3& transformBLocalGame,
        const RE::NiPoint3& anchorAWorld,
        float bodyScale);

    [[nodiscard]] bool tryGetGrabAuthorityBodyWorldTransform(
        RE::hknpWorld* world,
        RE::hknpBodyId bodyId,
        RE::NiTransform& outTransform);
    [[nodiscard]] RE::NiTransform gripEvidenceWorldFrame(
        const CanonicalGrabFrame& frame,
        const RE::NiTransform& fallbackWorld);
    [[nodiscard]] RE::NiTransform gripEvidenceWorldFrame(
        const ImmutableGrabCaptureTelemetry& capture,
        const RE::NiTransform& fallbackWorld);
    [[nodiscard]] RE::NiPoint3 gripEvidencePointWorld(
        const CanonicalGrabFrame& frame,
        const RE::NiTransform& fallbackWorld);
    [[nodiscard]] RE::NiPoint3 gripEvidenceNormalWorld(
        const CanonicalGrabFrame& frame,
        const RE::NiTransform& fallbackWorld);
    [[nodiscard]] HeldBodyMassSummary readHeldBodyMassSummary(
        RE::hknpWorld* world,
        RE::hknpBodyId primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds,
        bool includeConnectedBodies = true);
}
