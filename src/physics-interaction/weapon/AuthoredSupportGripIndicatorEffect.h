#pragma once

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiSmartPointer.h"

namespace rock
{
    /*
     * World-root marker for a grip zone. The authored support seat zone and
     * the firing-grip reattach zone each own one instance under their own
     * scene node name; both show the same indicator mesh.
     */
    class AuthoredSupportGripIndicatorEffect
    {
    public:
        explicit AuthoredSupportGripIndicatorEffect(const char* nodeName) noexcept
            : _nodeName(nodeName)
        {}
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

        RE::NiPointer<RE::NiAVObject> _marker;
        RE::NiNode* _parent{ nullptr };
        const char* _nodeName{ nullptr };
        bool _active{ false };
        bool _assetLoadFailed{ false };
    };
}
