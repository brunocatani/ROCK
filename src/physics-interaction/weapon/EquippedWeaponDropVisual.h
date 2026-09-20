#pragma once

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiSmartPointer.h"
#include "physics-interaction/weapon/AuthoredWeaponGripPose.h"

namespace rock
{
    // Main-thread presentation owned only by a pending Toggle Drop. The exact
    // equipped instance is retained before RemoveItem; native code must detach
    // it before we may reuse it. No live loose-body transforms are overwritten.
    class EquippedWeaponDropVisual
    {
    public:
        EquippedWeaponDropVisual() = default;
        ~EquippedWeaponDropVisual() { release("destruction"); }
        EquippedWeaponDropVisual(const EquippedWeaponDropVisual&) = delete;
        EquippedWeaponDropVisual& operator=(const EquippedWeaponDropVisual&) = delete;
        void begin(RE::NiPointer<RE::NiAVObject> model,
            const RE::NiTransform& modelInWeapon,
            const AuthoredWeaponGripPose& grip, std::uint32_t referenceId);
        void update(const RE::NiTransform& physicalHandWorld, RE::NiAVObject* looseRoot);
        void tracePresentation(const char* phase) const;
        void prepareGrab();
        void release(const char* reason);
        void abandonSceneGraph();
        [[nodiscard]] bool active() const { return _model != nullptr; }

    private:
        void clear(const char* reason, bool sceneAvailable);
        void restoreLooseVisibility();
        void yieldHandPose();

        RE::NiPointer<RE::NiAVObject> _model;
        RE::NiPointer<RE::NiNode> _parent;
        RE::NiPointer<RE::NiAVObject> _hiddenLooseRoot;
        RE::NiTransform _modelInWeapon{};
        AuthoredWeaponGripPose _grip{};
        std::uint32_t _referenceId{ 0 };
        std::uint32_t _presentedFrames{ 0 };
        bool _looseRootWasVisible{ false };
        bool _handPoseOwned{ false };
    };
}
