#pragma once

#include "PhysicsLog.h"

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

namespace frik::rock
{
    class HandBoneCache
    {
    public:
        bool resolve()
        {
            auto* skeleton = f4vr::getFirstPersonSkeleton();
            if (!skeleton) {
                reset();
                return false;
            }

            auto* rightHand = static_cast<RE::NiNode*>(f4vr::findNode(skeleton, "RArm_Hand"));
            auto* leftHand = static_cast<RE::NiNode*>(f4vr::findNode(skeleton, "LArm_Hand"));
            if (!rightHand || !leftHand) {
                reset();
                return false;
            }

            const bool changed = !_ready || skeleton != _skeleton || rightHand != _rightHand || leftHand != _leftHand;
            _skeleton = skeleton;
            _rightHand = rightHand;
            _leftHand = leftHand;
            _ready = true;

            if (changed) {
                ROCK_LOG_DEBUG(Hand, "HandBoneCache resolved fpSkeleton={:p} RArm_Hand={:p} LArm_Hand={:p}", static_cast<void*>(_skeleton), static_cast<void*>(_rightHand),
                    static_cast<void*>(_leftHand));
            }

            return true;
        }

        void reset()
        {
            _skeleton = nullptr;
            _rightHand = nullptr;
            _leftHand = nullptr;
            _ready = false;
        }

        [[nodiscard]] bool isReady() const { return _ready && _skeleton && _rightHand && _leftHand; }

        [[nodiscard]] bool hasSkeletonChanged() const
        {
            auto* current = f4vr::getFirstPersonSkeleton();
            return !_ready || current != _skeleton;
        }

        [[nodiscard]] RE::NiTransform getWorldTransform(bool isLeft) const
        {
            auto* node = getNode(isLeft);
            return node ? node->world : RE::NiTransform();
        }

        [[nodiscard]] RE::NiNode* getNode(bool isLeft) const { return isLeft ? _leftHand : _rightHand; }

    private:
        RE::NiNode* _skeleton = nullptr;
        RE::NiNode* _rightHand = nullptr;
        RE::NiNode* _leftHand = nullptr;
        bool _ready = false;
    };
}
