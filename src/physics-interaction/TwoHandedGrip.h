#pragma once

#include <atomic>
#include <array>

#include "MeshGrab.h"
#include "PalmTransform.h"
#include "PhysicsLog.h"
#include "PhysicsUtils.h"
#include "WeaponInteractionRouter.h"

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
        void update(
            RE::NiNode* weaponNode,
            const WeaponInteractionContact& leftWeaponContact,
            bool leftGripPressed,
            float dt,
            const WeaponReloadRuntimeState& reloadState);

        void reset();

        bool isGripping() const { return _state == TwoHandedState::Gripping; }

        bool isTouching() const { return _state == TwoHandedState::Touching; }

        TwoHandedState getState() const { return _state; }

        bool getSolvedWeaponTransform(RE::NiTransform& outTransform) const;

    private:
        void transitionToTouching(RE::NiNode* weaponNode, const WeaponInteractionDecision& decision);
        void transitionToGripping(RE::NiNode* weaponNode, const WeaponInteractionDecision& decision);
        void transitionToInactive();

        void updateGripping(RE::NiNode* weaponNode, float dt);

        void setSupportGripPose(bool isLeft, WeaponGripPoseId poseId, const std::array<float, 5>* meshFingerPose);

        void clearSupportGripPose(bool isLeft);

        static void killFrikOffhandGrip();

        static void restoreFrikOffhandGrip();

        static RE::NiPoint3 worldToWeaponLocal(const RE::NiPoint3& worldPos, const RE::NiNode* weaponNode);

        static RE::NiPoint3 weaponLocalToWorld(const RE::NiPoint3& localPos, const RE::NiNode* weaponNode);

        static void snapHandToWorldPos(RE::NiAVObject* handBone, const RE::NiPoint3& targetWorld, const RE::NiNode* weaponNode);

        TwoHandedState _state{ TwoHandedState::Inactive };

        RE::NiPoint3 _offhandGripLocal{};

        RE::NiPoint3 _primaryGripLocal{};

        RE::NiPoint3 _grabNormal{};

        RE::NiPoint3 _supportNormalLocal{};

        WeaponGripPoseId _supportGripPose{ WeaponGripPoseId::BarrelWrap };

        WeaponPartKind _supportPartKind{ WeaponPartKind::Other };

        int _touchFrames{ 0 };
        static constexpr int TOUCH_TIMEOUT_FRAMES = 5;

        float _rotationBlend{ 0.0f };
        static constexpr float ROTATION_BLEND_SPEED = 8.0f;

        int _gripLogCounter{ 0 };

        RE::NiTransform _lastSolvedWeaponTransform{};

        bool _hasSolvedWeaponTransform{ false };

        RE::NiNode* _activeWeaponNode{ nullptr };
        RE::NiTransform _weaponNodeLocalBaseline{};
        bool _hasWeaponNodeLocalBaseline{ false };
    };

}
