#pragma once

/*
 * Hand selection helpers are grouped here so selection query, highlight, state, close-finger, and VATS highlight policy stay together.
 */


// ---- SelectionQueryPolicy.h ----

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "RE/NetImmerse/NiPoint.h"

namespace rock::selection_query_policy
{
    constexpr std::uint32_t kDefaultShapeCastFilterInfo = 0x000B002D;
    constexpr std::uint32_t kDefaultFarClipRayFilterInfo = 0x02420028;
    constexpr float kDefaultFarSelectionHmdConeHalfAngleDegrees = 50.0f;
    constexpr float kMinFarSelectionHmdConeHalfAngleDegrees = 1.0f;
    constexpr float kMaxFarSelectionHmdConeHalfAngleDegrees = 89.0f;
    constexpr float kDegreesToRadians = 0.017453292519943295769f;
    constexpr std::size_t kMaxShapeCastPrecisionCandidates = 4;
    constexpr float kShapeCastCandidateScoreTieEpsilon = 0.0001f;
    constexpr float kCloseSelectionCandidateSwitchScoreRatio = 0.70f;

    struct ShapeCastCandidateScoringInput
    {
        bool isFarSelection = false;
        float lateralDistance = (std::numeric_limits<float>::max)();
        float alongDistance = (std::numeric_limits<float>::max)();
        float lateralScale = 1.0f;
        float alongScale = 1.0f;
        float normalDotDirection = 0.0f;
        bool hasHitNormal = false;
    };

    struct ShapeCastCandidateScore
    {
        float score = (std::numeric_limits<float>::max)();
        float lateralComponent = 0.0f;
        float alongComponent = 0.0f;
        float surfaceComponent = 0.0f;
        bool valid = false;
    };

    inline std::uint32_t sanitizeFilterInfo(std::uint32_t configuredValue, std::uint32_t fallback)
    {
        return configuredValue != 0 ? configuredValue : fallback;
    }

    inline bool shouldReplaceSelectionForSameRef(bool currentIsFarSelection, bool nextIsFarSelection, std::uint32_t currentBodyId, std::uint32_t nextBodyId)
    {
        return currentIsFarSelection != nextIsFarSelection || currentBodyId != nextBodyId;
    }

    inline bool isBetterShapeCastCandidate(bool isFarSelection, float lateralDistance, float alongDistance, float bestLateralDistance, float bestAlongDistance)
    {
        constexpr float kTieEpsilon = 0.001f;
        if (!isFarSelection) {
            if (alongDistance < bestAlongDistance - kTieEpsilon) {
                return true;
            }
            if (alongDistance > bestAlongDistance + kTieEpsilon) {
                return false;
            }
        }
        if (lateralDistance < bestLateralDistance - kTieEpsilon) {
            return true;
        }
        if (lateralDistance > bestLateralDistance + kTieEpsilon) {
            return false;
        }
        return isFarSelection && alongDistance < bestAlongDistance;
    }

    inline float sanitizeSelectionScoreScale(float value, float fallback)
    {
        if (!std::isfinite(value) || value <= 0.0f) {
            return fallback;
        }
        return value;
    }

    inline float normalizedSquaredSelectionDistance(float value, float scale)
    {
        const float normalized = value / sanitizeSelectionScoreScale(scale, 1.0f);
        return normalized * normalized;
    }

    inline ShapeCastCandidateScore scoreShapeCastCandidate(const ShapeCastCandidateScoringInput& input)
    {
        ShapeCastCandidateScore result{};
        if (!std::isfinite(input.lateralDistance) || !std::isfinite(input.alongDistance) || input.lateralDistance < 0.0f || input.alongDistance < 0.0f) {
            return result;
        }

        const float lateralWeight = input.isFarSelection ? 0.80f : 0.70f;
        const float alongWeight = input.isFarSelection ? 0.12f : 0.22f;
        result.lateralComponent = normalizedSquaredSelectionDistance(input.lateralDistance, input.lateralScale) * lateralWeight;
        result.alongComponent = normalizedSquaredSelectionDistance(input.alongDistance, input.alongScale) * alongWeight;

        /*
         * Normal evidence is intentionally a small tie breaker. It prefers
         * front-facing surface evidence when several bodies are tightly packed,
         * but it cannot override clear palm/ray alignment.
         */
        if (input.hasHitNormal && std::isfinite(input.normalDotDirection)) {
            const float clampedDot = std::clamp(input.normalDotDirection, -1.0f, 1.0f);
            result.surfaceComponent = (clampedDot + 1.0f) * 0.04f;
        } else {
            result.surfaceComponent = 0.04f;
        }

        result.score = result.lateralComponent + result.alongComponent + result.surfaceComponent;
        result.valid = std::isfinite(result.score);
        return result;
    }

    inline bool isBetterShapeCastCandidateScore(const ShapeCastCandidateScore& candidate, const ShapeCastCandidateScore& best)
    {
        if (!candidate.valid) {
            return false;
        }
        if (!best.valid) {
            return true;
        }
        return candidate.score < best.score - kShapeCastCandidateScoreTieEpsilon;
    }

    inline bool isUsableShapeCastCandidateScore(float score)
    {
        return std::isfinite(score) && score >= 0.0f && score < (std::numeric_limits<float>::max)() * 0.5f;
    }

    inline bool shouldKeepCurrentCloseSelectionAgainstCandidate(float currentScore, float candidateScore, float currentDistance, float candidateDistance)
    {
        if (isUsableShapeCastCandidateScore(currentScore) && isUsableShapeCastCandidateScore(candidateScore)) {
            return candidateScore >= currentScore * kCloseSelectionCandidateSwitchScoreRatio;
        }

        if (!std::isfinite(currentDistance) || !std::isfinite(candidateDistance)) {
            return true;
        }

        return candidateDistance > currentDistance * kCloseSelectionCandidateSwitchScoreRatio;
    }

    inline float sanitizeBehindPalmTolerance(float toleranceGameUnits, float fallbackGameUnits)
    {
        if (!std::isfinite(toleranceGameUnits) || toleranceGameUnits < 0.0f) {
            return fallbackGameUnits;
        }
        return toleranceGameUnits;
    }

    inline bool shouldRejectBehindPalmHit(bool isFarSelection, float signedAlongDistance, float closeToleranceGameUnits)
    {
        if (!std::isfinite(signedAlongDistance)) {
            return true;
        }

        if (isFarSelection) {
            return signedAlongDistance < 0.0f;
        }

        const float tolerance = sanitizeBehindPalmTolerance(closeToleranceGameUnits, 2.0f);
        return signedAlongDistance < -tolerance;
    }

    inline bool shouldPromoteFarHitToClose(float hitDistanceFromHand, float nearReachDistance)
    {
        return nearReachDistance > 0.0f && hitDistanceFromHand <= nearReachDistance;
    }

    inline bool shouldGateFarCandidateWithHmdCone(bool isFarSelection, bool promotesToCloseSelection)
    {
        /*
         * ROCK's far sphere cast is also a recovery path for hand-reachable
         * objects that the narrow close cast missed. Those promoted hits are
         * close intent and must not be rejected by the far-only HMD view gate.
         */
        return isFarSelection && !promotesToCloseSelection;
    }

    inline float sanitizeFarSelectionHmdConeHalfAngleDegrees(float halfAngleDegrees)
    {
        if (!std::isfinite(halfAngleDegrees)) {
            return kDefaultFarSelectionHmdConeHalfAngleDegrees;
        }

        return std::clamp(halfAngleDegrees, kMinFarSelectionHmdConeHalfAngleDegrees, kMaxFarSelectionHmdConeHalfAngleDegrees);
    }

    inline float farSelectionHmdConeMinDot(float halfAngleDegrees)
    {
        return std::cos(sanitizeFarSelectionHmdConeHalfAngleDegrees(halfAngleDegrees) * kDegreesToRadians);
    }

    template <class Vector>
    inline bool tryNormalizeVectorForHmdCone(Vector value, Vector& out)
    {
        const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
        if (!std::isfinite(lengthSquared) || lengthSquared <= 1.0e-8f) {
            return false;
        }

        const float inverseLength = 1.0f / std::sqrt(lengthSquared);
        out.x = value.x * inverseLength;
        out.y = value.y * inverseLength;
        out.z = value.z * inverseLength;
        return std::isfinite(out.x) && std::isfinite(out.y) && std::isfinite(out.z);
    }

    /*
     * Far selection remains hand-ray driven so intentional side-pointing still
     * works. The HMD cone is a second intent gate: a far candidate must also be
     * in front of the headset before it can highlight, cache, or start a pull.
     */
    template <class Vector>
    inline bool isPointInsideFarHmdCone(bool gateEnabled,
        bool hasHmdFrame,
        const Vector& hmdPositionWorld,
        const Vector& hmdForwardWorld,
        const Vector& hitPointWorld,
        float minDot,
        float* outDot = nullptr)
    {
        if (outDot) {
            *outDot = -1.0f;
        }

        if (!gateEnabled) {
            if (outDot) {
                *outDot = 1.0f;
            }
            return true;
        }

        if (!hasHmdFrame) {
            return false;
        }

        Vector normalizedForward{};
        if (!tryNormalizeVectorForHmdCone(hmdForwardWorld, normalizedForward)) {
            return false;
        }

        Vector hmdToHit{};
        hmdToHit.x = hitPointWorld.x - hmdPositionWorld.x;
        hmdToHit.y = hitPointWorld.y - hmdPositionWorld.y;
        hmdToHit.z = hitPointWorld.z - hmdPositionWorld.z;

        Vector normalizedToHit{};
        if (!tryNormalizeVectorForHmdCone(hmdToHit, normalizedToHit)) {
            return false;
        }

        const float dot = normalizedForward.x * normalizedToHit.x + normalizedForward.y * normalizedToHit.y + normalizedForward.z * normalizedToHit.z;
        if (outDot) {
            *outDot = dot;
        }

        return std::isfinite(dot) && dot >= std::clamp(minDot, -1.0f, 1.0f);
    }

    inline bool shouldKeepSelectionAfterMiss(bool currentIsFarSelection, int heldFrames, int minimumHoldFrames, float currentDistance, float hysteresisRange)
    {
        if (heldFrames < minimumHoldFrames) {
            return true;
        }

        if (currentIsFarSelection) {
            return false;
        }

        return currentDistance <= hysteresisRange;
    }
}

namespace rock
{
    struct FarSelectionHmdConeGate
    {
        bool enabled = false;
        bool hasHmdFrame = false;
        RE::NiPoint3 hmdPositionWorld{};
        RE::NiPoint3 hmdForwardWorld{};
        float minDot = selection_query_policy::farSelectionHmdConeMinDot(selection_query_policy::kDefaultFarSelectionHmdConeHalfAngleDegrees);

        [[nodiscard]] bool acceptsHitPoint(const RE::NiPoint3& hitPointWorld, float* outDot = nullptr) const
        {
            return selection_query_policy::isPointInsideFarHmdCone(enabled, hasHmdFrame, hmdPositionWorld, hmdForwardWorld, hitPointWorld, minDot, outDot);
        }
    };
}

// ---- SelectionHighlightPolicy.h ----

#include <cstdint>

namespace RE
{
    class NiAVObject;
    class TESObjectREFR;
}

namespace rock::selection_highlight_policy
{
    /*
     * ROCK selection highlights whole grab targets, so it follows FO4VR's
     * pointed-object rollover path rather than the crafting/menu segment path.
     * Ghidra verifies the VR rollover caller enables VatsEffectControl with
     * (imageSpace=false, objectRolloverState=true), constructs a VatsEffectTarget
     * from object 3D, and adds it without applying segment-color data.
     */
    constexpr bool kVatsHighlightEnableImageSpaceEffect = false;
    constexpr bool kVatsHighlightUseObjectRolloverState = true;
    constexpr int kVatsHighlightRefreshIntervalFrames = 10;

    [[nodiscard]] constexpr bool shouldUseVatsHighlightForSelection(bool isFarSelection) noexcept
    {
        return isFarSelection;
    }

    enum class VatsHighlightSource : std::uint8_t
    {
        None,
        HitNode,
        VisualNode,
        ReferenceRoot
    };

    struct VatsHighlightTargetChoice
    {
        RE::NiAVObject* node = nullptr;
        VatsHighlightSource source = VatsHighlightSource::None;
    };

    /*
     * Vanilla pointed-object rollover builds the VatsEffectTarget from the
     * reference's object 3D. ROCK keeps visual/hit nodes as fallbacks for cases
     * where the selected physics body resolves to a child node that the root
     * target builder cannot enumerate.
     */
    [[nodiscard]] inline VatsHighlightTargetChoice chooseVatsHighlightTarget(RE::NiAVObject* hitNode, RE::NiAVObject* visualNode, RE::NiAVObject* referenceRoot) noexcept
    {
        if (referenceRoot) {
            return { referenceRoot, VatsHighlightSource::ReferenceRoot };
        }
        if (visualNode) {
            return { visualNode, VatsHighlightSource::VisualNode };
        }
        if (hitNode) {
            return { hitNode, VatsHighlightSource::HitNode };
        }
        return {};
    }

    [[nodiscard]] inline RE::NiAVObject* chooseVatsHighlightTargetNode(RE::NiAVObject* hitNode, RE::NiAVObject* visualNode, RE::NiAVObject* referenceRoot) noexcept
    {
        return chooseVatsHighlightTarget(hitNode, visualNode, referenceRoot).node;
    }

    [[nodiscard]] constexpr const char* vatsHighlightSourceName(VatsHighlightSource source) noexcept
    {
        switch (source) {
        case VatsHighlightSource::HitNode:
            return "hitNode";
        case VatsHighlightSource::VisualNode:
            return "visualNode";
        case VatsHighlightSource::ReferenceRoot:
            return "referenceRoot";
        default:
            return "none";
        }
    }

    [[nodiscard]] constexpr bool shouldStartVatsHighlight(bool enabled, bool hasReference, bool sameAsCurrent) noexcept
    {
        return enabled && hasReference && !sameAsCurrent;
    }

    [[nodiscard]] constexpr bool shouldRefreshVatsHighlightAttempt(bool enabled, bool hasReference, int framesSinceAttempt, int refreshIntervalFrames) noexcept
    {
        const int effectiveInterval = refreshIntervalFrames > 0 ? refreshIntervalFrames : 1;
        return enabled && hasReference && framesSinceAttempt >= effectiveInterval;
    }
}

// ---- SelectionStatePolicy.h ----

#include "physics-interaction/hand/HandInteractionStateMachine.h"

namespace rock::selection_state_policy
{
    inline HandState stateForSelection(bool isFarSelection) { return isFarSelection ? HandState::SelectedFar : HandState::SelectedClose; }

    inline bool canUpdateSelectionFromState(HandState state) { return rock::canUpdateSelectionFromState(state); }

    inline bool canProcessSelectedState(HandState state) { return rock::canProcessSelectedState(state); }

    inline bool hasExclusiveObjectSelection(HandState state) { return rock::hasExclusiveObjectSelection(state); }
}

namespace rock
{
    /*
     * ROCK treats the other hand's held dynamic object as a valid close-grab
     * candidate while still keeping pulls and locked selections exclusive. This
     * distinction is explicit because a single "other hand ref" block prevents
     * the second hand from ever reaching the shared dynamic-grab path.
     */
    struct OtherHandSelectionContext
    {
        RE::TESObjectREFR* exclusiveRef = nullptr;
        RE::TESObjectREFR* shareableHeldRef = nullptr;

        [[nodiscard]] bool allowsSharedHeldReference(RE::TESObjectREFR* ref) const noexcept
        {
            return ref != nullptr && shareableHeldRef == ref;
        }

        [[nodiscard]] bool blocksReference(RE::TESObjectREFR* ref) const noexcept
        {
            return ref != nullptr && exclusiveRef == ref && !allowsSharedHeldReference(ref);
        }
    };
}

// ---- SelectedCloseFingerPolicy.h ----

#include <algorithm>
#include <cmath>

namespace rock::selected_close_finger_policy
{
    /*
     * ROCK only pre-curls fingers around a close-selected object when the hand
     * is moving slowly enough. Keeping this as a policy helper prevents config
     * parsing, selection state, and FRIK hand-pose publishing from drifting
     * apart.
     */
    inline bool shouldApplyPreCurl(bool enabled, bool isSelectedClose, bool selectionValid, float handSpeedMetersPerSecond, float maxHandSpeedMetersPerSecond)
    {
        if (!enabled || !isSelectedClose || !selectionValid) {
            return false;
        }
        if (!std::isfinite(handSpeedMetersPerSecond)) {
            return false;
        }
        if (maxHandSpeedMetersPerSecond <= 0.0f) {
            return true;
        }
        return handSpeedMetersPerSecond <= maxHandSpeedMetersPerSecond;
    }

    inline float estimateHandSpeedMetersPerSecond(float distanceGameUnits, float deltaTimeSeconds, float gameUnitsPerMeter)
    {
        if (!std::isfinite(distanceGameUnits) || !std::isfinite(deltaTimeSeconds) || !std::isfinite(gameUnitsPerMeter) || deltaTimeSeconds <= 0.0f ||
            gameUnitsPerMeter <= 0.0f) {
            return 0.0f;
        }
        return (std::max)(0.0f, distanceGameUnits) / gameUnitsPerMeter / deltaTimeSeconds;
    }
}


// ---- VatsSelectionHighlight.h ----

namespace RE
{
    class NiAVObject;
    class TESObjectREFR;
}

namespace rock
{
    namespace detail
    {
        struct VatsEffectTarget;
    }

    class VatsSelectionHighlight
    {
    public:
        VatsSelectionHighlight() = default;
        ~VatsSelectionHighlight();

        VatsSelectionHighlight(const VatsSelectionHighlight&) = delete;
        VatsSelectionHighlight& operator=(const VatsSelectionHighlight&) = delete;

        VatsSelectionHighlight(VatsSelectionHighlight&&) = delete;
        VatsSelectionHighlight& operator=(VatsSelectionHighlight&&) = delete;

        bool play(RE::TESObjectREFR* refr, RE::NiAVObject* targetNode, bool enabled, const char* handName, const char* targetSourceName);
        bool refresh(RE::TESObjectREFR* refr, RE::NiAVObject* targetNode, bool enabled, const char* handName, const char* targetSourceName);
        bool refreshActive(bool enabled, const char* handName);
        void stop();

        [[nodiscard]] bool isActive() const noexcept { return _target != nullptr; }
        [[nodiscard]] bool isActiveForReference(RE::TESObjectREFR* refr) const noexcept { return _target != nullptr && _highlightedRef == refr; }
        [[nodiscard]] bool isActiveFor(RE::TESObjectREFR* refr, RE::NiAVObject* targetNode) const noexcept
        {
            return _target != nullptr && _highlightedRef == refr && _highlightedNode == targetNode;
        }

    private:
        detail::VatsEffectTarget* _target = nullptr;
        RE::TESObjectREFR* _highlightedRef = nullptr;
        RE::NiAVObject* _highlightedNode = nullptr;
    };
}
