#pragma once

/*
 * Hand selection helpers are grouped here so selection query, highlight, state, close-finger, and VATS highlight policy stay together.
 */


// ---- SelectionQueryPolicy.h ----

#include <cstdint>

namespace frik::rock::selection_query_policy
{
    constexpr std::uint32_t kDefaultShapeCastFilterInfo = 0x000B002D;
    constexpr std::uint32_t kDefaultFarClipRayFilterInfo = 0x02420028;

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

    inline bool shouldPromoteFarHitToClose(float hitDistanceFromHand, float nearReachDistance)
    {
        return nearReachDistance > 0.0f && hitDistanceFromHand <= nearReachDistance;
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

// ---- SelectionHighlightPolicy.h ----

#include <cstdint>

namespace RE
{
    class NiAVObject;
}

namespace frik::rock::selection_highlight_policy
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

namespace frik::rock::selection_state_policy
{
    inline HandState stateForSelection(bool isFarSelection) { return isFarSelection ? HandState::SelectedFar : HandState::SelectedClose; }

    inline bool canUpdateSelectionFromState(HandState state) { return frik::rock::canUpdateSelectionFromState(state); }

    inline bool canProcessSelectedState(HandState state) { return frik::rock::canProcessSelectedState(state); }

    inline bool hasExclusiveObjectSelection(HandState state) { return frik::rock::hasExclusiveObjectSelection(state); }
}

// ---- SelectedCloseFingerPolicy.h ----

#include <algorithm>
#include <cmath>

namespace frik::rock::selected_close_finger_policy
{
    /*
     * HIGGS only pre-curls fingers around a close-selected object when the hand
     * is moving slowly enough. ROCK keeps the same behavior as a policy helper
     * so config parsing, selection state, and FRIK hand-pose publishing cannot
     * drift apart.
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

namespace frik::rock
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
