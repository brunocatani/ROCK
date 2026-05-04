#pragma once

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
