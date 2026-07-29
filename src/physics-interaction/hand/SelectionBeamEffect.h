#pragma once

#include <array>

#include "physics-interaction/hand/SelectionBeamPolicy.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiSmartPointer.h"

namespace rock
{
    class SelectionBeamEffect
    {
    public:
        SelectionBeamEffect() = default;
        ~SelectionBeamEffect();

        SelectionBeamEffect(const SelectionBeamEffect&) = delete;
        SelectionBeamEffect& operator=(const SelectionBeamEffect&) = delete;

        SelectionBeamEffect(SelectionBeamEffect&&) = delete;
        SelectionBeamEffect& operator=(SelectionBeamEffect&&) = delete;

        bool preload(const char* handName);
        bool update(const selection_beam_policy::Frame& frame, const char* handName);
        void hide();
        void shutdown();
        void abandonSceneGraph();

        [[nodiscard]] bool isActive() const noexcept { return _active; }

    private:
        bool ensureSourceTemplate(const char* handName);
        RE::NiNode* cloneSegmentFromTemplate(const std::string& segmentName, const char* handName);
        bool ensureSegments(RE::NiNode* parent, const char* handName);
        void attachSegments(RE::NiNode* parent);
        void detachSegments();
        void setSegmentsVisible(bool visible, bool updateTransforms);
        void clearSegments(bool detachFromKnownValidParent);

        RE::NiPointer<RE::NiNode> _sourceTemplate;
        std::array<RE::NiPointer<RE::NiNode>, selection_beam_policy::kSegmentCount> _segments{};
        RE::NiNode* _parent = nullptr;
        bool _segmentsCreated = false;
        bool _active = false;
        bool _assetLoadFailed = false;
    };
}
