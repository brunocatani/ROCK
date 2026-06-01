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
        shutdown();
    }

    bool SelectionBeamEffect::ensureSegments(RE::NiNode* parent, const char* handName)
    {
        if (!parent || _assetLoadFailed) {
            return false;
        }

        if (_segmentsCreated) {
            if (_parent != parent) {
                detachSegments();
                attachSegments(parent);
            }
            return _parent == parent;
        }

        try {
            for (std::size_t i = 0; i < _segments.size(); ++i) {
                const std::string segmentName =
                    std::string("ROCK_SelectionBeam_") + (handName ? handName : "Hand") + "_" + std::to_string(i);
                auto* segment = f4vr::getClonedNiNodeForNifFileSetName(kSelectionBeamSegmentNif, segmentName);
                if (!segment) {
                    ROCK_LOG_WARN(Hand, "{} selection beam segment clone returned null at index {}", handName ? handName : "Unknown", i);
                    shutdown();
                    _assetLoadFailed = true;
                    return false;
                }
                segment->name = RE::BSFixedString(segmentName.c_str());
                segment->fadeAmount = selection_beam_policy::kDefaultAlpha;
                setNodeVisible(segment, false);
                _segments[i].reset(segment);
            }
        } catch (const std::exception& e) {
            ROCK_LOG_WARN(Hand, "{} selection beam disabled: failed to load '{}' ({})", handName ? handName : "Unknown", kSelectionBeamSegmentNif, e.what());
            shutdown();
            _assetLoadFailed = true;
            return false;
        } catch (...) {
            ROCK_LOG_WARN(Hand, "{} selection beam disabled: failed to load '{}'", handName ? handName : "Unknown", kSelectionBeamSegmentNif);
            shutdown();
            _assetLoadFailed = true;
            return false;
        }

        _segmentsCreated = true;
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

    void SelectionBeamEffect::setSegmentsVisible(bool visible)
    {
        for (auto& segment : _segments) {
            if (!segment) {
                continue;
            }
            setNodeVisible(segment.get(), visible);
            segment->fadeAmount = visible ? segment->fadeAmount : 0.0f;
            f4vr::updateDown(segment.get(), true);
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
        if (!_active && !_segmentsCreated) {
            return;
        }

        setSegmentsVisible(false);
        _active = false;
    }

    void SelectionBeamEffect::shutdown()
    {
        hide();
        detachSegments();
        for (auto& segment : _segments) {
            segment.reset();
        }
        _segmentsCreated = false;
        _active = false;
    }
}
