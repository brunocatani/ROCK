#pragma once

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiSmartPointer.h"

namespace rock
{
    class AuthoredSupportGripIndicatorEffect
    {
    public:
        AuthoredSupportGripIndicatorEffect() = default;
        ~AuthoredSupportGripIndicatorEffect();

        AuthoredSupportGripIndicatorEffect(
            const AuthoredSupportGripIndicatorEffect&) = delete;
        AuthoredSupportGripIndicatorEffect& operator=(
            const AuthoredSupportGripIndicatorEffect&) = delete;
        AuthoredSupportGripIndicatorEffect(
            AuthoredSupportGripIndicatorEffect&&) = delete;
        AuthoredSupportGripIndicatorEffect& operator=(
            AuthoredSupportGripIndicatorEffect&&) = delete;

        bool preload();
        bool update(const RE::NiPoint3& positionWorld);
        void hide();
        void shutdown();
        void abandonSceneGraph();

    private:
        bool ensureMarker();
        bool ensureAttached(RE::NiNode* parent);
        void detachMarker();
        void setVisible(bool visible, bool updateTransforms);
        void clearMarker(bool detachFromKnownValidParent);

        RE::NiPointer<RE::NiNode> _marker;
        RE::NiNode* _parent{ nullptr };
        bool _active{ false };
        bool _assetLoadFailed{ false };
    };
}
