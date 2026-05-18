#pragma once

/*
 * Three-phase grab is the new generic-object authority layer. The old path let
 * surface normals, contact patches, opposition contacts, raw ray hits, visible
 * hand reverse solving, and multiple pivots all decide part of the same grab. This
 * policy keeps those signals as evidence only: the root-flattened hand builds a
 * stable pocket, object evidence builds one grip area, and the dynamic native
 * spring receives a body-local grip point plus a pocket target.
 */

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/hand/HandFrame.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace rock::grab_three_phase
{
    enum class AcquisitionPhase : std::uint8_t
    {
        Idle,
        GravityPulling,
        NearConverging,
        SeatedPivotReacquire,
        TouchHeld,
    };

    inline const char* phaseName(AcquisitionPhase phase)
    {
        switch (phase) {
        case AcquisitionPhase::Idle:
            return "Idle";
        case AcquisitionPhase::GravityPulling:
            return "GravityPulling";
        case AcquisitionPhase::NearConverging:
            return "NearConverging";
        case AcquisitionPhase::SeatedPivotReacquire:
            return "SeatedPivotReacquire";
        case AcquisitionPhase::TouchHeld:
            return "TouchHeld";
        default:
            return "Unknown";
        }
    }

    inline float lengthSquared(const RE::NiPoint3& value)
    {
        return value.x * value.x + value.y * value.y + value.z * value.z;
    }

    inline float length(const RE::NiPoint3& value)
    {
        return std::sqrt((std::max)(0.0f, lengthSquared(value)));
    }

    inline float dot(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline bool isFinite(const RE::NiPoint3& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    inline bool isFinite(const RE::NiTransform& transform)
    {
        if (!isFinite(transform.translate) || !std::isfinite(transform.scale)) {
            return false;
        }
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(transform.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    inline RE::NiPoint3 normalizeOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float lenSq = lengthSquared(value);
        if (!std::isfinite(lenSq) || lenSq <= 0.000001f) {
            const float fallbackLenSq = lengthSquared(fallback);
            if (!std::isfinite(fallbackLenSq) || fallbackLenSq <= 0.000001f) {
                return RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
            }
            const float invFallbackLen = 1.0f / std::sqrt(fallbackLenSq);
            return RE::NiPoint3{ fallback.x * invFallbackLen, fallback.y * invFallbackLen, fallback.z * invFallbackLen };
        }

        const float invLen = 1.0f / std::sqrt(lenSq);
        return RE::NiPoint3{ value.x * invLen, value.y * invLen, value.z * invLen };
    }

    struct GrabPocketFrame
    {
        RE::NiTransform handWorld{};
        RE::NiPoint3 palmCenterWorld{};
        RE::NiPoint3 palmNormalWorld{};
        RE::NiPoint3 fingerForwardWorld{};
        RE::NiPoint3 thumbSideWorld{};
        RE::NiPoint3 fingerSideWorld{};
        RE::NiPoint3 pocketCenterWorld{};
        float pocketRadiusGameUnits = 0.0f;
        float pocketDepthGameUnits = 0.0f;
        bool valid = false;
    };

    inline GrabPocketFrame buildGrabPocketFrameWithPalmCenter(
        const RE::NiTransform& handWorld,
        bool isLeft,
        const RE::NiPoint3& palmCenterWorld,
        float pocketDepthGameUnits,
        float pocketRadiusGameUnits)
    {
        GrabPocketFrame frame{};
        frame.handWorld = handWorld;
        frame.palmCenterWorld = palmCenterWorld;
        frame.palmNormalWorld = normalizeOrFallback(computePalmNormalFromHandBasis(handWorld, isLeft), RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        frame.fingerForwardWorld = normalizeOrFallback(transformHandspaceDirection(handWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, isLeft), frame.palmNormalWorld);
        frame.thumbSideWorld = normalizeOrFallback(transformHandspaceDirection(handWorld, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }, isLeft), RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
        frame.fingerSideWorld = RE::NiPoint3{ -frame.thumbSideWorld.x, -frame.thumbSideWorld.y, -frame.thumbSideWorld.z };
        frame.pocketDepthGameUnits = (std::max)(0.0f, std::isfinite(pocketDepthGameUnits) ? pocketDepthGameUnits : 0.0f);
        frame.pocketRadiusGameUnits = (std::max)(0.1f, std::isfinite(pocketRadiusGameUnits) ? pocketRadiusGameUnits : 9.0f);
        frame.pocketCenterWorld = frame.palmCenterWorld + frame.palmNormalWorld * frame.pocketDepthGameUnits;
        frame.valid = isFinite(handWorld) && isFinite(frame.palmCenterWorld) && isFinite(frame.pocketCenterWorld) && lengthSquared(frame.palmNormalWorld) > 0.000001f;
        return frame;
    }

    struct ObjectGripArea
    {
        RE::NiTransform objectBodyWorldAtCapture{};
        RE::NiPoint3 contactSeedWorld{};
        RE::NiPoint3 contactSeedBodyLocal{};
        RE::NiPoint3 gripCenterWorld{};
        RE::NiPoint3 gripCenterBodyLocal{};
        RE::NiPoint3 centerOfMassWorld{};
        float seedInsetGameUnits = 0.0f;
        float confidence = 0.0f;
        const char* source = "none";
        const char* fallbackReason = "none";
        bool valid = false;
        bool centerOfMassValid = false;
    };

    struct GripAreaInput
    {
        RE::NiTransform objectBodyWorld{};
        RE::NiPoint3 contactSeedWorld{};
        RE::NiPoint3 centerOfMassWorld{};
        RE::NiPoint3 interiorDirectionWorld{};
        float preferredInsetGameUnits = 2.0f;
        float maxInsetGameUnits = 6.0f;
        bool centerOfMassValid = false;
        bool interiorDirectionValid = false;
        const char* source = "selection";
    };

    inline ObjectGripArea buildObjectGripArea(const GripAreaInput& input)
    {
        ObjectGripArea area{};
        area.objectBodyWorldAtCapture = input.objectBodyWorld;
        area.contactSeedWorld = input.contactSeedWorld;
        area.centerOfMassWorld = input.centerOfMassWorld;
        area.centerOfMassValid = input.centerOfMassValid;
        area.source = input.source ? input.source : "selection";

        if (!isFinite(input.objectBodyWorld) || !isFinite(input.contactSeedWorld)) {
            area.fallbackReason = "nonFiniteInput";
            return area;
        }

        RE::NiPoint3 seedToInterior{};
        bool hasInteriorDirection = false;
        if (input.centerOfMassValid && isFinite(input.centerOfMassWorld)) {
            seedToInterior = input.centerOfMassWorld - input.contactSeedWorld;
            hasInteriorDirection = true;
            area.fallbackReason = "centerOfMassInterior";
        } else if (input.interiorDirectionValid && isFinite(input.interiorDirectionWorld)) {
            seedToInterior = input.interiorDirectionWorld;
            hasInteriorDirection = true;
            area.fallbackReason = "providedInteriorDirection";
        } else {
            area.fallbackReason = "noTrustedInteriorDirection";
        }

        const float seedToInteriorLength = length(seedToInterior);
        const float requestedInset = std::isfinite(input.preferredInsetGameUnits) ? input.preferredInsetGameUnits : 2.0f;
        const float requestedMaxInset = std::isfinite(input.maxInsetGameUnits) ? input.maxInsetGameUnits : 6.0f;
        const float maxInset = (std::max)(0.0f, requestedMaxInset);

        RE::NiPoint3 gripCenterWorld = input.contactSeedWorld;
        if (hasInteriorDirection && seedToInteriorLength > 0.0001f && maxInset > 0.0f) {
            const float inset = (std::min)({ seedToInteriorLength * 0.35f, (std::max)(0.0f, requestedInset), maxInset });
            const RE::NiPoint3 inward = seedToInterior * (1.0f / seedToInteriorLength);
            gripCenterWorld = input.contactSeedWorld + inward * inset;
            area.seedInsetGameUnits = inset;
        }

        area.gripCenterWorld = gripCenterWorld;
        area.contactSeedBodyLocal = transform_math::worldPointToLocal(input.objectBodyWorld, input.contactSeedWorld);
        area.gripCenterBodyLocal = transform_math::worldPointToLocal(input.objectBodyWorld, gripCenterWorld);
        area.confidence = input.centerOfMassValid ? 0.9f : (hasInteriorDirection ? 0.8f : 0.55f);
        area.valid = isFinite(area.gripCenterWorld) && isFinite(area.gripCenterBodyLocal);
        return area;
    }

    struct PhaseClassificationInput
    {
        GrabPocketFrame pocket{};
        RE::NiPoint3 gripSeedWorld{};
        bool hasFreshTouchContact = false;
        bool isFarSelection = false;
        float touchAcquireDistanceGameUnits = 4.0f;
        float touchContactMaxDistanceGameUnits = 0.0f;
        float nearConvergeDistanceGameUnits = 28.0f;
        float behindPalmToleranceGameUnits = 1.5f;
    };

    struct PhaseClassificationResult
    {
        AcquisitionPhase phase = AcquisitionPhase::Idle;
        float gripToPocketDistanceGameUnits = 0.0f;
        float signedPalmDistanceGameUnits = 0.0f;
        const char* reason = "none";
        bool accepted = false;
        bool frontHemisphere = false;
    };

    inline PhaseClassificationResult classifyAcquisitionPhase(const PhaseClassificationInput& input)
    {
        PhaseClassificationResult result{};
        const float touchDistance =
            (std::max)(0.1f, std::isfinite(input.touchAcquireDistanceGameUnits) ? input.touchAcquireDistanceGameUnits : 4.0f);
        const float touchContactMaxDistance =
            (input.touchContactMaxDistanceGameUnits > 0.0f && std::isfinite(input.touchContactMaxDistanceGameUnits)) ?
                (std::max)(touchDistance, input.touchContactMaxDistanceGameUnits) :
                touchDistance;
        const float nearDistance = (std::max)(touchDistance, std::isfinite(input.nearConvergeDistanceGameUnits) ? input.nearConvergeDistanceGameUnits : 28.0f);
        const float behindTolerance =
            (std::max)(0.0f, std::isfinite(input.behindPalmToleranceGameUnits) ? input.behindPalmToleranceGameUnits : 1.5f);

        if (!input.pocket.valid || !isFinite(input.gripSeedWorld)) {
            result.reason = "invalidPocketOrGripSeed";
            return result;
        }

        const RE::NiPoint3 toGripFromPocket = input.gripSeedWorld - input.pocket.palmCenterWorld;
        result.gripToPocketDistanceGameUnits = length(toGripFromPocket);
        result.signedPalmDistanceGameUnits = dot(input.gripSeedWorld - input.pocket.palmCenterWorld, input.pocket.palmNormalWorld);
        result.frontHemisphere = result.signedPalmDistanceGameUnits >= -behindTolerance;

        if (!result.frontHemisphere) {
            result.reason = "behindPalm";
            return result;
        }

        result.accepted = true;
        const bool freshTouchWithinPocketEnvelope = input.hasFreshTouchContact && result.gripToPocketDistanceGameUnits <= touchContactMaxDistance;
        if (freshTouchWithinPocketEnvelope || result.gripToPocketDistanceGameUnits <= touchDistance) {
            result.phase = AcquisitionPhase::TouchHeld;
            result.reason = freshTouchWithinPocketEnvelope ? "freshTouchContactInPocketEnvelope" : "insideTouchEnvelope";
            return result;
        }

        if (result.gripToPocketDistanceGameUnits <= nearDistance && !input.isFarSelection) {
            result.phase = AcquisitionPhase::NearConverging;
            result.reason = "insideNearEnvelope";
            return result;
        }

        result.phase = AcquisitionPhase::GravityPulling;
        result.reason = input.isFarSelection ? "farSelection" : "outsideNearEnvelope";
        return result;
    }

    inline float computeAcquisitionVisualEnvelopeGameUnits(float touchDistanceGameUnits, float nearConvergeDistanceGameUnits, float configuredVisualStartDistanceGameUnits)
    {
        const float touchDistance = (std::max)(0.1f, std::isfinite(touchDistanceGameUnits) ? touchDistanceGameUnits : 4.0f);
        const float nearDistance = (std::max)(touchDistance, std::isfinite(nearConvergeDistanceGameUnits) ? nearConvergeDistanceGameUnits : touchDistance);
        const float configuredDistance =
            (std::isfinite(configuredVisualStartDistanceGameUnits) && configuredVisualStartDistanceGameUnits > 0.0f) ?
                configuredVisualStartDistanceGameUnits :
                touchDistance;
        return std::clamp((std::max)(touchDistance, configuredDistance), touchDistance, nearDistance);
    }

    struct ConvergencePromotionInput
    {
        bool hasGrabBody = false;
        bool heldBodyColliding = false;
        float gripErrorGameUnits = std::numeric_limits<float>::max();
        float previousGripErrorGameUnits = std::numeric_limits<float>::max();
        float deltaSeconds = 0.0f;
        float elapsedSeconds = 0.0f;
        float maxTimeSeconds = 0.0f;
        float touchDistanceGameUnits = 4.0f;
        float pocketRadiusGameUnits = 9.0f;
        int stableInsidePocketFrames = 0;
        int requiredStableInsidePocketFrames = 3;
        float maxSeparatingSpeedGameUnitsPerSecond = 40.0f;
    };

    struct ConvergencePromotionDecision
    {
        bool reachedTouchRange = false;
        bool insidePocket = false;
        bool stableThisFrame = false;
        bool timedOutInsidePocket = false;
        int nextStableInsidePocketFrames = 0;
        float separatingSpeedGameUnitsPerSecond = 0.0f;
        const char* timeoutBlockReason = "none";
    };

    inline ConvergencePromotionDecision evaluateConvergencePromotion(const ConvergencePromotionInput& input)
    {
        ConvergencePromotionDecision decision{};
        const float touchDistance = (std::max)(0.1f, std::isfinite(input.touchDistanceGameUnits) ? input.touchDistanceGameUnits : 4.0f);
        const float pocketRadius = (std::max)(touchDistance, std::isfinite(input.pocketRadiusGameUnits) ? input.pocketRadiusGameUnits : 9.0f);
        const float gripError =
            std::isfinite(input.gripErrorGameUnits) ? input.gripErrorGameUnits : std::numeric_limits<float>::max();

        decision.reachedTouchRange = input.hasGrabBody && gripError <= touchDistance;
        decision.insidePocket = input.hasGrabBody && gripError <= pocketRadius;

        const float deltaSeconds = (std::max)(0.0001f, std::isfinite(input.deltaSeconds) ? input.deltaSeconds : 0.0001f);
        if (std::isfinite(input.previousGripErrorGameUnits)) {
            decision.separatingSpeedGameUnitsPerSecond = (gripError - input.previousGripErrorGameUnits) / deltaSeconds;
        }

        const float maxSeparatingSpeed =
            (std::max)(0.0f, std::isfinite(input.maxSeparatingSpeedGameUnitsPerSecond) ? input.maxSeparatingSpeedGameUnitsPerSecond : 40.0f);
        decision.stableThisFrame =
            decision.insidePocket &&
            (input.heldBodyColliding ||
                decision.reachedTouchRange ||
                !std::isfinite(input.previousGripErrorGameUnits) ||
                decision.separatingSpeedGameUnitsPerSecond <= maxSeparatingSpeed);

        decision.nextStableInsidePocketFrames =
            decision.stableThisFrame ? (std::max)(0, input.stableInsidePocketFrames) + 1 : 0;

        const int requiredStableFrames = (std::max)(1, input.requiredStableInsidePocketFrames);
        const bool timeoutElapsed =
            input.maxTimeSeconds > 0.0f &&
            std::isfinite(input.maxTimeSeconds) &&
            std::isfinite(input.elapsedSeconds) &&
            input.elapsedSeconds >= input.maxTimeSeconds;
        decision.timedOutInsidePocket =
            timeoutElapsed &&
            decision.insidePocket &&
            decision.nextStableInsidePocketFrames >= requiredStableFrames;

        if (!timeoutElapsed) {
            decision.timeoutBlockReason = "waitingForTimeout";
        } else if (!decision.insidePocket) {
            decision.timeoutBlockReason = "outsidePocket";
        } else if (decision.nextStableInsidePocketFrames < requiredStableFrames) {
            decision.timeoutBlockReason = "waitingForStablePocket";
        } else {
            decision.timeoutBlockReason = "promote";
        }

        return decision;
    }

    inline RE::NiTransform makeBodyTargetWithLocalGripAtPocket(RE::NiTransform targetBodyWorld,
        const RE::NiPoint3& gripCenterBodyLocal,
        const RE::NiPoint3& pocketCenterWorld)
    {
        const RE::NiPoint3 gripOffsetWorld = transform_math::localVectorToWorld(targetBodyWorld, gripCenterBodyLocal);
        targetBodyWorld.translate = pocketCenterWorld - gripOffsetWorld;
        return targetBodyWorld;
    }

    inline RE::NiTransform deriveNodeTargetFromBodyTarget(const RE::NiTransform& targetBodyWorld, const RE::NiTransform& bodyLocal)
    {
        return transform_math::composeTransforms(targetBodyWorld, transform_math::invertTransform(bodyLocal));
    }
}
