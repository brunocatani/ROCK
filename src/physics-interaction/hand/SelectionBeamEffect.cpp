#include "physics-interaction/hand/SelectionBeamEffect.h"

#include <cstdint>
#include <exception>
#include <string>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

namespace rock
{
    namespace
    {
        constexpr const char* kSelectionBeamSegmentNif = "Data/Meshes/ROCK/selection_beam_segment.nif";
        constexpr std::uint64_t kAppCulledFlag = 0x1ull;

        [[nodiscard]] RE::NiTransform makeSegmentLocalTransform(
            const RE::NiNode& parent,
            const RE::NiPoint3& positionWorld,
            float segmentSizeGameUnits)
        {
            auto result = transform_math::makeIdentityTransform<RE::NiTransform>();
            result.translate = transform_math::worldPointToLocal(parent.world, positionWorld);
            result.scale = segmentSizeGameUnits;
            return result;
        }

        void setNodeVisible(RE::NiAVObject* node, bool visible)
        {
            if (!node) {
                return;
            }

            if (visible) {
                node->flags.flags &= ~kAppCulledFlag;
            } else {
                node->flags.flags |= kAppCulledFlag;
            }
        }
    }

    SelectionBeamEffect::~SelectionBeamEffect()
    {
        abandonSceneGraph();
    }

    bool SelectionBeamEffect::ensureSourceTemplate(const char* handName)
    {
        if (_sourceTemplate) {
            return true;
        }

        if (_assetLoadFailed) {
            return false;
        }

        try {
            auto* sourceTemplate = f4vr::loadNifFromFile(kSelectionBeamSegmentNif);
            if (!sourceTemplate) {
                ROCK_LOG_WARN(Hand, "{} selection beam disabled: source template load returned null for '{}'", handName ? handName : "Unknown", kSelectionBeamSegmentNif);
                _assetLoadFailed = true;
                return false;
            }
            _sourceTemplate.reset(sourceTemplate);
        } catch (const std::exception& e) {
            ROCK_LOG_WARN(Hand, "{} selection beam disabled: failed to load '{}' ({})", handName ? handName : "Unknown", kSelectionBeamSegmentNif, e.what());
            _assetLoadFailed = true;
            return false;
        } catch (...) {
            ROCK_LOG_WARN(Hand, "{} selection beam disabled: failed to load '{}'", handName ? handName : "Unknown", kSelectionBeamSegmentNif);
            _assetLoadFailed = true;
            return false;
        }

        return true;
    }

    RE::NiNode* SelectionBeamEffect::cloneSegmentFromTemplate(const std::string& segmentName, const char* handName)
    {
        if (!ensureSourceTemplate(handName)) {
            return nullptr;
        }

        f4vr::NiCloneProcess proc;
        proc.unk18 = reinterpret_cast<std::uint64_t*>(f4vr::cloneAddr1.address());
        proc.unk48 = reinterpret_cast<std::uint64_t*>(f4vr::cloneAddr2.address());
        auto* segment = f4vr::cloneNode(_sourceTemplate.get(), &proc);
        if (!segment) {
            ROCK_LOG_WARN(Hand, "{} selection beam segment clone returned null for '{}'", handName ? handName : "Unknown", segmentName);
            return nullptr;
        }

        segment->name = RE::BSFixedString(segmentName.c_str());
        segment->fadeAmount = selection_beam_policy::kDefaultAlpha;
        setNodeVisible(segment, false);
        return segment;
    }

    bool SelectionBeamEffect::preload(const char* handName)
    {
        if (_segmentsCreated) {
            return true;
        }

        if (_assetLoadFailed) {
            return false;
        }

        if (!ensureSourceTemplate(handName)) {
            return false;
        }

        try {
            for (std::size_t i = 0; i < _segments.size(); ++i) {
                const std::string segmentName =
                    std::string("ROCK_SelectionBeam_") + (handName ? handName : "Hand") + "_" + std::to_string(i);
                auto* segment = cloneSegmentFromTemplate(segmentName, handName);
                if (!segment) {
                    clearSegments(false);
                    _assetLoadFailed = true;
                    return false;
                }
                _segments[i].reset(segment);
            }
        } catch (const std::exception& e) {
            ROCK_LOG_WARN(Hand, "{} selection beam disabled: failed to clone beam segment ({})", handName ? handName : "Unknown", e.what());
            clearSegments(false);
            _assetLoadFailed = true;
            return false;
        } catch (...) {
            ROCK_LOG_WARN(Hand, "{} selection beam disabled: failed to clone beam segment", handName ? handName : "Unknown");
            clearSegments(false);
            _assetLoadFailed = true;
            return false;
        }

        _segmentsCreated = true;
        return true;
    }

    bool SelectionBeamEffect::ensureSegments(RE::NiNode* parent, const char* handName)
    {
        if (!parent || _assetLoadFailed) {
            return false;
        }

        if (!preload(handName)) {
            return false;
        }

        if (_parent != parent) {
            detachSegments();
        }

        attachSegments(parent);
        return _parent == parent;
    }

    void SelectionBeamEffect::attachSegments(RE::NiNode* parent)
    {
        if (!parent) {
            return;
        }

        for (auto& segment : _segments) {
            if (!segment) {
                continue;
            }
            if (segment->parent != parent) {
                parent->AttachChild(segment.get(), true);
            }
        }
        _parent = parent;
    }

    void SelectionBeamEffect::detachSegments()
    {
        if (!_parent) {
            return;
        }

        for (auto& segment : _segments) {
            if (!segment || segment->parent != _parent) {
                continue;
            }
            RE::NiPointer<RE::NiAVObject> detached;
            _parent->DetachChild(segment.get(), detached);
        }
        _parent = nullptr;
    }

    void SelectionBeamEffect::setSegmentsVisible(bool visible, bool updateTransforms)
    {
        for (auto& segment : _segments) {
            if (!segment) {
                continue;
            }
            setNodeVisible(segment.get(), visible);
            segment->fadeAmount = visible ? segment->fadeAmount : 0.0f;
            if (updateTransforms) {
                f4vr::updateDown(segment.get(), true);
            }
        }
    }

    bool SelectionBeamEffect::update(const selection_beam_policy::Frame& rawFrame, const char* handName)
    {
        auto frame = rawFrame;
        frame.config = selection_beam_policy::sanitizeConfig(frame.config);
        if (!selection_beam_policy::shouldRender(frame)) {
            hide();
            return false;
        }

        auto* parent = f4vr::getWorldRootNode();
        if (!parent || !selection_beam_policy::isFinitePoint(parent->world.translate) || !ensureSegments(parent, handName)) {
            hide();
            return false;
        }

        const RE::NiPoint3 controlWorld =
            selection_beam_policy::makeControlPoint(frame.startWorld, frame.endWorld, frame.config.curveLiftGameUnits);

        for (std::size_t i = 0; i < _segments.size(); ++i) {
            auto& segment = _segments[i];
            if (!segment) {
                continue;
            }

            const float t = static_cast<float>(i + 1) / static_cast<float>(_segments.size() + 1);
            const RE::NiPoint3 positionWorld =
                selection_beam_policy::sampleQuadraticBezier(frame.startWorld, controlWorld, frame.endWorld, t);
            segment->local = makeSegmentLocalTransform(*parent, positionWorld, frame.config.segmentSizeGameUnits);
            segment->fadeAmount = frame.config.alpha;
            setNodeVisible(segment.get(), true);
            f4vr::updateDown(segment.get(), true);
        }

        _active = true;
        return true;
    }

    void SelectionBeamEffect::hide()
    {
        if (!_active) {
            return;
        }

        setSegmentsVisible(false, true);
        _active = false;
    }

    void SelectionBeamEffect::clearSegments(bool detachFromKnownValidParent)
    {
        if (detachFromKnownValidParent) {
            hide();
            detachSegments();
        } else {
            setSegmentsVisible(false, false);
            _parent = nullptr;
            _active = false;
        }

        for (auto& segment : _segments) {
            segment.reset();
        }
        _segmentsCreated = false;
    }

    void SelectionBeamEffect::shutdown()
    {
        clearSegments(true);
    }

    void SelectionBeamEffect::abandonSceneGraph()
    {
        clearSegments(false);
    }
}
