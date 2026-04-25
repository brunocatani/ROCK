#pragma once

#include <atomic>

#include "MeshGrab.h"
#include "PalmTransform.h"
#include "PhysicsLog.h"
#include "PhysicsUtils.h"

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

namespace frik::rock
{
    enum class TwoHandedState
    {
        Inactive,
        Touching,
        Gripping,
    };

    class TwoHandedGrip
    {
    public:
        void update(RE::NiNode* weaponNode, bool offhandTouching, bool gripPressed, bool isLeftHanded, float dt);

        void reset();

        bool isGripping() const { return _state == TwoHandedState::Gripping; }

        bool isTouching() const { return _state == TwoHandedState::Touching; }

        TwoHandedState getState() const { return _state; }

    private:
        void transitionToTouching(RE::NiNode* weaponNode);
        void transitionToGripping(RE::NiNode* weaponNode, bool isLeftHanded);
        void transitionToInactive(bool isLeftHanded);

        void updateGripping(RE::NiNode* weaponNode, bool isLeftHanded, float dt);

        void setBarrelGripPose(bool isLeft);

        void clearBarrelGripPose(bool isLeft);

        static void killFrikOffhandGrip();

        static void restoreFrikOffhandGrip();

        static RE::NiPoint3 worldToWeaponLocal(const RE::NiPoint3& worldPos, const RE::NiNode* weaponNode);

        static RE::NiPoint3 weaponLocalToWorld(const RE::NiPoint3& localPos, const RE::NiNode* weaponNode);

        static void snapHandToWorldPos(RE::NiAVObject* handBone, const RE::NiPoint3& targetWorld, const RE::NiNode* weaponNode);

        TwoHandedState _state{ TwoHandedState::Inactive };

        RE::NiPoint3 _offhandGripLocal{};

        RE::NiPoint3 _primaryGripLocal{};

        RE::NiPoint3 _grabNormal{};

        int _touchFrames{ 0 };
        static constexpr int TOUCH_TIMEOUT_FRAMES = 5;

        float _rotationBlend{ 0.0f };
        static constexpr float ROTATION_BLEND_SPEED = 8.0f;

        int _gripLogCounter{ 0 };
    };

}
