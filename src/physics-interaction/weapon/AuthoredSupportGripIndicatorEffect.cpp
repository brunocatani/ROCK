#include "physics-interaction/weapon/AuthoredSupportGripIndicatorEffect.h"

#include <cmath>
#include <cstdint>
#include <exception>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock
{
    namespace
    {
        constexpr const char* kIndicatorNif =
            "Data/Meshes/ROCK/authored_support_grip_indicator_bright.nif";
        constexpr std::uint64_t kAppCulledFlag = 0x1ull;
        constexpr float kIndicatorWorldScale = 1.0f;
        constexpr float kIndicatorAlpha = 1.0f;

        [[nodiscard]] bool isFiniteTransform(
            const RE::NiTransform& transform)
        {
            if (!std::isfinite(transform.translate.x) ||
                !std::isfinite(transform.translate.y) ||
                !std::isfinite(transform.translate.z) ||
                !std::isfinite(transform.scale)) {
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

        [[nodiscard]] bool isFinitePoint(const RE::NiPoint3& point)
        {
            return std::isfinite(point.x) &&
                   std::isfinite(point.y) &&
                   std::isfinite(point.z);
        }

        void setNodeVisible(RE::NiAVObject* node, const bool visible)
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

    AuthoredSupportGripIndicatorEffect::~AuthoredSupportGripIndicatorEffect()
    {
        abandonSceneGraph();
    }

    bool AuthoredSupportGripIndicatorEffect::ensureMarker()
    {
        if (_marker) {
            return true;
        }
        if (_assetLoadFailed) {
            return false;
        }
        if (!_nodeName) {
            ROCK_LOG_WARN(Weapon,
                "Grip zone indicator disabled: no scene node name");
            _assetLoadFailed = true;
            return false;
        }

        try {
            auto* marker = f4vr::loadNifObjectFromFile(kIndicatorNif);
            if (!marker) {
                ROCK_LOG_WARN(Weapon,
                    "Grip zone indicator '{}' disabled: source load returned null for '{}'",
                    _nodeName,
                    kIndicatorNif);
                _assetLoadFailed = true;
                return false;
            }
            marker->name = RE::BSFixedString(_nodeName);
            marker->fadeAmount = 0.0f;
            setNodeVisible(marker, false);
            _marker.reset(marker);
        } catch (const std::exception& e) {
            ROCK_LOG_WARN(Weapon,
                "Grip zone indicator '{}' disabled: failed to load '{}' ({})",
                _nodeName,
                kIndicatorNif,
                e.what());
            _assetLoadFailed = true;
            return false;
        } catch (...) {
            ROCK_LOG_WARN(Weapon,
                "Grip zone indicator '{}' disabled: failed to load '{}'",
                _nodeName,
                kIndicatorNif);
            _assetLoadFailed = true;
            return false;
        }

        return true;
    }

    bool AuthoredSupportGripIndicatorEffect::preload()
    {
        return ensureMarker();
    }

    bool AuthoredSupportGripIndicatorEffect::ensureAttached(RE::NiNode* parent)
    {
        if (!parent || !ensureMarker()) {
            return false;
        }
        if (_parent != parent) {
            detachMarker();
        }
        if (_marker->parent != parent) {
            parent->AttachChild(_marker.get(), true);
        }
        _parent = parent;
        return _marker->parent == parent;
    }

    void AuthoredSupportGripIndicatorEffect::detachMarker()
    {
        if (!_parent || !_marker || _marker->parent != _parent) {
            _parent = nullptr;
            return;
        }

        RE::NiPointer<RE::NiAVObject> detached;
        _parent->DetachChild(_marker.get(), detached);
        _parent = nullptr;
    }

    void AuthoredSupportGripIndicatorEffect::setVisible(
        const bool visible,
        const bool updateTransforms)
    {
        if (!_marker) {
            return;
        }
        setNodeVisible(_marker.get(), visible);
        _marker->fadeAmount = visible ? kIndicatorAlpha : 0.0f;
        if (updateTransforms) {
            f4vr::updateDown(_marker.get(), true);
        }
    }

    bool AuthoredSupportGripIndicatorEffect::update(
        const RE::NiPoint3& positionWorld)
    {
        auto* parent = f4vr::getWorldRootNode();
        if (!parent ||
            !isFinitePoint(positionWorld) ||
            !isFiniteTransform(parent->world) ||
            std::abs(parent->world.scale) <= 0.0001f ||
            !ensureAttached(parent)) {
            hide();
            return false;
        }

        auto local = transform_math::makeIdentityTransform<RE::NiTransform>();
        local.translate =
            transform_math::worldPointToLocal(parent->world, positionWorld);
        local.scale = kIndicatorWorldScale / std::abs(parent->world.scale);
        if (!isFiniteTransform(local) || !isFinitePoint(local.translate)) {
            hide();
            return false;
        }

        _marker->local = local;
        setVisible(true, true);
        _active = true;
        return true;
    }

    void AuthoredSupportGripIndicatorEffect::hide()
    {
        if (!_active) {
            return;
        }
        setVisible(false, true);
        _active = false;
    }

    void AuthoredSupportGripIndicatorEffect::clearMarker(
        const bool detachFromKnownValidParent)
    {
        if (detachFromKnownValidParent) {
            hide();
            detachMarker();
        } else {
            setVisible(false, false);
            _parent = nullptr;
            _active = false;
        }
        _marker.reset();
    }

    void AuthoredSupportGripIndicatorEffect::shutdown()
    {
        clearMarker(true);
    }

    void AuthoredSupportGripIndicatorEffect::abandonSceneGraph()
    {
        clearMarker(false);
    }
}
