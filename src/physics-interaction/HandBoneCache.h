#pragma once

#include "DirectSkeletonBoneReader.h"
#include "PhysicsLog.h"

#include "RE/NetImmerse/NiTransform.h"

#include <string_view>

namespace frik::rock
{
    class HandBoneCache
    {
    public:
        /*
         * The interaction hand frame is sampled from the same root flattened
         * bone tree that drives generated hand colliders. The cache stores
         * copied transforms, not scene-node pointers, so it must be refreshed
         * once per frame before grab, selection, and held-object math run.
         */
        bool resolve()
        {
            DirectSkeletonBoneSnapshot snapshot{};
            if (!_reader.capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                    skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                    snapshot)) {
                clearResolvedState();
                return false;
            }

            RE::NiTransform rightHand{};
            RE::NiTransform leftHand{};
            if (!findBone(snapshot, "RArm_Hand", rightHand) || !findBone(snapshot, "LArm_Hand", leftHand)) {
                clearResolvedState();
                return false;
            }

            const bool changed =
                !_ready ||
                snapshot.skeleton != _skeleton ||
                snapshot.boneTree != _boneTree ||
                snapshot.inPowerArmor != _inPowerArmor;

            _skeleton = snapshot.skeleton;
            _boneTree = snapshot.boneTree;
            _inPowerArmor = snapshot.inPowerArmor;
            _rightHandWorld = rightHand;
            _leftHandWorld = leftHand;
            _ready = true;

            if (changed) {
                ROCK_LOG_DEBUG(Hand,
                    "HandBoneCache resolved rootFlattenedTree skeleton={:p} tree={:p} powerArmor={}",
                    _skeleton,
                    _boneTree,
                    _inPowerArmor ? "yes" : "no");
            }

            return true;
        }

        void reset()
        {
            clearResolvedState();
            _reader.resetCache();
        }

        [[nodiscard]] bool isReady() const { return _ready && _skeleton && _boneTree; }

        [[nodiscard]] RE::NiTransform getWorldTransform(bool isLeft) const
        {
            return isLeft ? _leftHandWorld : _rightHandWorld;
        }

        [[nodiscard]] const void* getSkeleton() const { return _skeleton; }
        [[nodiscard]] const void* getBoneTree() const { return _boneTree; }
        [[nodiscard]] bool isInPowerArmor() const { return _inPowerArmor; }

    private:
        void clearResolvedState()
        {
            _skeleton = nullptr;
            _boneTree = nullptr;
            _inPowerArmor = false;
            _rightHandWorld = {};
            _leftHandWorld = {};
            _ready = false;
        }

        static bool findBone(const DirectSkeletonBoneSnapshot& snapshot, std::string_view name, RE::NiTransform& outTransform)
        {
            for (const auto& bone : snapshot.bones) {
                if (bone.name == name) {
                    outTransform = bone.world;
                    return true;
                }
            }
            return false;
        }

        DirectSkeletonBoneReader _reader;
        const void* _skeleton = nullptr;
        const void* _boneTree = nullptr;
        bool _inPowerArmor = false;
        RE::NiTransform _rightHandWorld{};
        RE::NiTransform _leftHandWorld{};
        bool _ready = false;
    };
}
