#pragma once

/*
 * Hand skeleton helpers are grouped here so direct skeleton reads, cached hand frames, root-flattened runtime, and frame resolution stay on one authority path.
 */


// ---- DirectSkeletonBoneReader.h ----

#include <cstdint>
#include <string>
#include <vector>

#include "physics-interaction/debug/SkeletonBoneDebugMath.h"

#include "RE/NetImmerse/NiTransform.h"

namespace rock::root_flattened_finger_skeleton_runtime
{
    struct SemanticHandFrame;
}

namespace rock
{
    struct DirectSkeletonBoneEntry
    {
        std::string name;
        int treeIndex = -1;
        int parentTreeIndex = -1;
        int drawableParentSnapshotIndex = -1;
        RE::NiTransform world{};
        bool included = false;
    };

    struct DirectSkeletonBoneSnapshot
    {
        bool valid = false;
        bool inPowerArmor = false;
        skeleton_bone_debug_math::DebugSkeletonBoneMode mode = skeleton_bone_debug_math::DebugSkeletonBoneMode::Off;
        skeleton_bone_debug_math::SkeletonBoneSnapshotSource source = skeleton_bone_debug_math::SkeletonBoneSnapshotSource::None;
        const void* skeleton = nullptr;
        const void* boneTree = nullptr;
        int totalBoneCount = 0;
        int requiredResolvedCount = 0;
        std::vector<DirectSkeletonBoneEntry> bones;
        std::vector<std::string> missingRequiredBones;
    };

    class DirectSkeletonBoneReader
    {
    public:
        bool capture(
            skeleton_bone_debug_math::DebugSkeletonBoneMode mode,
            skeleton_bone_debug_math::DebugSkeletonBoneSource source,
            DirectSkeletonBoneSnapshot& outSnapshot);
        void resetCache();

    private:
        struct CachedBone
        {
            std::string name;
            int treeIndex = -1;
            int parentTreeIndex = -1;
            int drawableParentSnapshotIndex = -1;
            bool included = false;
        };

        bool rebuildTreeCache(
            void* skeleton,
            void* boneTree,
            skeleton_bone_debug_math::SkeletonBoneSnapshotSource source,
            skeleton_bone_debug_math::DebugSkeletonBoneMode mode,
            bool inPowerArmor);

        bool captureFromCachedTree(DirectSkeletonBoneSnapshot& outSnapshot);

        const void* _cachedSkeleton = nullptr;
        void* _cachedBoneTree = nullptr;
        skeleton_bone_debug_math::SkeletonBoneSnapshotSource _cachedSource = skeleton_bone_debug_math::SkeletonBoneSnapshotSource::None;
        skeleton_bone_debug_math::DebugSkeletonBoneMode _cachedMode = skeleton_bone_debug_math::DebugSkeletonBoneMode::Off;
        int _cachedTotalBoneCount = 0;
        bool _cachedInPowerArmor = false;
        bool _missingSourceLogged = false;
        int _cachedRequiredResolvedCount = 0;
        std::vector<CachedBone> _cachedBones;
        std::vector<std::string> _cachedMissingRequiredBones;
    };
}

// ---- HandBoneCache.h ----

#include "RE/NetImmerse/NiTransform.h"

#include <string_view>

namespace rock
{
    void logHandBoneCacheResolved(const void* skeleton, const void* boneTree, bool inPowerArmor);

    struct CachedSemanticHandFrameData
    {
        RE::NiTransform rawHandWorld{};
        RE::NiTransform palmAnchorWorld{};
        RE::NiPoint3 fingerBaseCenterWorld{};
        RE::NiPoint3 fingerForwardWorld{ 1.0f, 0.0f, 0.0f };
        RE::NiPoint3 palmDepthWorld{ 0.0f, 1.0f, 0.0f };
        RE::NiPoint3 palmFaceWorld{ 0.0f, -1.0f, 0.0f };
        RE::NiPoint3 acrossPalmWorld{ 0.0f, 0.0f, 1.0f };
        float palmLength = 0.0f;
        bool valid = false;
    };

    class HandBoneCache
    {
    public:
        /*
         * The interaction hand frame is sampled from the same root flattened
         * bone tree that drives generated hand colliders. The cache stores
         * copied transforms, not scene-node pointers, so it must be refreshed
         * once per frame before grab, selection, and held-object math run.
         */
        bool resolve();

        void reset()
        {
            clearResolvedState();
            _reader.resetCache();
        }

        [[nodiscard]] bool isReady() const
        {
            return _ready && _skeleton && _boneTree && _rightSemanticHandFrame.valid && _leftSemanticHandFrame.valid;
        }

        [[nodiscard]] RE::NiTransform getWorldTransform(bool isLeft) const
        {
            return isLeft ? _leftHandWorld : _rightHandWorld;
        }

        [[nodiscard]] bool getSemanticHandFrame(bool isLeft, root_flattened_finger_skeleton_runtime::SemanticHandFrame& outFrame) const;

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
            _rightSemanticHandFrame = {};
            _leftSemanticHandFrame = {};
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
        CachedSemanticHandFrameData _rightSemanticHandFrame{};
        CachedSemanticHandFrameData _leftSemanticHandFrame{};
        bool _ready = false;
    };
}

// ---- RootFlattenedFingerSkeletonRuntime.h ----

/*
 * ROCK's runtime hand geometry is sourced from the live root-flattened FO4VR
 * skeleton snapshot, not authored INI landmark tables. This helper resolves the
 * rendered finger chains and palm-facing normal into a compact world-space
 * snapshot for collider, pose, and debug systems while leaving final pose
 * publication to the FRIK API.
 */

#include "RE/NetImmerse/NiPoint.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>

namespace rock::root_flattened_finger_skeleton_runtime
{
    struct FingerChain
    {
        std::array<RE::NiPoint3, 3> points{};
        bool valid = false;
    };

    struct Snapshot
    {
        std::array<FingerChain, 5> fingers{};
        RE::NiPoint3 palmNormalWorld{ 0.0f, -1.0f, 0.0f };
        bool palmNormalValid = false;
        bool valid = false;
    };

    struct FingerLandmark
    {
        RE::NiPoint3 base{};
        RE::NiPoint3 openDirection{ 1.0f, 0.0f, 0.0f };
        float length = 0.0f;
        bool valid = false;
    };

    struct LandmarkSet
    {
        std::array<FingerLandmark, 5> fingers{};
        RE::NiPoint3 palmNormalWorld{ 0.0f, -1.0f, 0.0f };
        bool valid = false;
    };

    struct SemanticHandFrame
    {
        RE::NiTransform rawHandWorld{};
        RE::NiTransform palmAnchorWorld{};
        RE::NiPoint3 fingerBaseCenterWorld{};
        RE::NiPoint3 fingerForwardWorld{ 1.0f, 0.0f, 0.0f };
        RE::NiPoint3 palmDepthWorld{ 0.0f, 1.0f, 0.0f };
        RE::NiPoint3 palmFaceWorld{ 0.0f, -1.0f, 0.0f };
        RE::NiPoint3 acrossPalmWorld{ 0.0f, 0.0f, 1.0f };
        float palmLength = 0.0f;
        bool valid = false;
    };

    inline const char* fingerBoneName(bool isLeft, std::size_t fingerIndex, std::size_t segmentIndex)
    {
        static constexpr std::array<const char*, 15> kRightNames{
            "RArm_Finger11",
            "RArm_Finger12",
            "RArm_Finger13",
            "RArm_Finger21",
            "RArm_Finger22",
            "RArm_Finger23",
            "RArm_Finger31",
            "RArm_Finger32",
            "RArm_Finger33",
            "RArm_Finger41",
            "RArm_Finger42",
            "RArm_Finger43",
            "RArm_Finger51",
            "RArm_Finger52",
            "RArm_Finger53"
        };
        static constexpr std::array<const char*, 15> kLeftNames{
            "LArm_Finger11",
            "LArm_Finger12",
            "LArm_Finger13",
            "LArm_Finger21",
            "LArm_Finger22",
            "LArm_Finger23",
            "LArm_Finger31",
            "LArm_Finger32",
            "LArm_Finger33",
            "LArm_Finger41",
            "LArm_Finger42",
            "LArm_Finger43",
            "LArm_Finger51",
            "LArm_Finger52",
            "LArm_Finger53"
        };

        if (fingerIndex >= 5 || segmentIndex >= 3) {
            return nullptr;
        }

        const std::size_t index = fingerIndex * 3 + segmentIndex;
        return isLeft ? kLeftNames[index] : kRightNames[index];
    }

    inline float distance(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        const float dx = lhs.x - rhs.x;
        const float dy = lhs.y - rhs.y;
        const float dz = lhs.z - rhs.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    inline RE::NiPoint3 normalizedOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
        if (!std::isfinite(lengthSquared) || lengthSquared <= 0.000001f) {
            const float fallbackLengthSquared = fallback.x * fallback.x + fallback.y * fallback.y + fallback.z * fallback.z;
            if (!std::isfinite(fallbackLengthSquared) || fallbackLengthSquared <= 0.000001f) {
                return RE::NiPoint3(1.0f, 0.0f, 0.0f);
            }
            const float fallbackInv = 1.0f / std::sqrt(fallbackLengthSquared);
            return RE::NiPoint3(fallback.x * fallbackInv, fallback.y * fallbackInv, fallback.z * fallbackInv);
        }

        const float inv = 1.0f / std::sqrt(lengthSquared);
        return RE::NiPoint3(value.x * inv, value.y * inv, value.z * inv);
    }

    inline float dot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    inline RE::NiPoint3 cross(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3{
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x,
        };
    }

    inline RE::NiPoint3 projectOntoPlane(const RE::NiPoint3& value, const RE::NiPoint3& normal)
    {
        return value - normal * dot(value, normal);
    }

    inline RE::NiPoint3 rotateLocalVectorToWorld(const RE::NiMatrix3& matrix, const RE::NiPoint3& localVector)
    {
        return RE::NiPoint3{
            matrix.entry[0][0] * localVector.x + matrix.entry[1][0] * localVector.y + matrix.entry[2][0] * localVector.z,
            matrix.entry[0][1] * localVector.x + matrix.entry[1][1] * localVector.y + matrix.entry[2][1] * localVector.z,
            matrix.entry[0][2] * localVector.x + matrix.entry[1][2] * localVector.y + matrix.entry[2][2] * localVector.z,
        };
    }

    inline RE::NiMatrix3 semanticPalmMatrixFromAxes(const RE::NiPoint3& fingerForwardWorld, const RE::NiPoint3& palmDepthWorld, const RE::NiPoint3& acrossPalmWorld)
    {
        /*
         * Match generated hand-collider storage: local X=fingers, local Y=palm
         * depth/back, local Z=across palm stored as matrix columns.
         */
        RE::NiMatrix3 matrix{};
        matrix.entry[0][0] = fingerForwardWorld.x;
        matrix.entry[1][0] = fingerForwardWorld.y;
        matrix.entry[2][0] = fingerForwardWorld.z;
        matrix.entry[0][1] = palmDepthWorld.x;
        matrix.entry[1][1] = palmDepthWorld.y;
        matrix.entry[2][1] = palmDepthWorld.z;
        matrix.entry[0][2] = acrossPalmWorld.x;
        matrix.entry[1][2] = acrossPalmWorld.y;
        matrix.entry[2][2] = acrossPalmWorld.z;
        return matrix;
    }

    inline SemanticHandFrame buildSemanticHandFrame(
        const RE::NiTransform& hand,
        const std::array<RE::NiPoint3, 5>& fingerBases,
        const RE::NiPoint3& crossPalmDirection,
        float palmDepth = 0.75f)
    {
        SemanticHandFrame frame{};
        frame.rawHandWorld = hand;

        RE::NiPoint3 fingerCenter{};
        RE::NiPoint3 palmCenter = hand.translate;
        for (const auto& point : fingerBases) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                return frame;
            }
            fingerCenter = fingerCenter + point;
            palmCenter = palmCenter + point;
        }
        fingerCenter = fingerCenter * (1.0f / static_cast<float>(fingerBases.size()));
        palmCenter = palmCenter * (1.0f / static_cast<float>(fingerBases.size() + 1));

        const RE::NiPoint3 fallbackX{ 1.0f, 0.0f, 0.0f };
        const RE::NiPoint3 fallbackY{ 0.0f, 1.0f, 0.0f };
        const RE::NiPoint3 fallbackZ{ 0.0f, 0.0f, 1.0f };
        frame.fingerForwardWorld = normalizedOrFallback(fingerCenter - hand.translate, fallbackX);
        frame.acrossPalmWorld = normalizedOrFallback(projectOntoPlane(crossPalmDirection, frame.fingerForwardWorld), fallbackZ);
        frame.palmDepthWorld = normalizedOrFallback(cross(frame.acrossPalmWorld, frame.fingerForwardWorld), fallbackY);
        frame.acrossPalmWorld = normalizedOrFallback(cross(frame.fingerForwardWorld, frame.palmDepthWorld), fallbackZ);
        frame.fingerForwardWorld = normalizedOrFallback(cross(frame.palmDepthWorld, frame.acrossPalmWorld), fallbackX);
        frame.palmFaceWorld = RE::NiPoint3{ -frame.palmDepthWorld.x, -frame.palmDepthWorld.y, -frame.palmDepthWorld.z };
        frame.fingerBaseCenterWorld = fingerCenter;
        frame.palmLength = distance(fingerCenter, hand.translate);
        if (!std::isfinite(frame.palmLength)) {
            frame = {};
            return frame;
        }

        const float currentPalmDepthOffset = dot(palmCenter - hand.translate, frame.palmDepthWorld);
        palmCenter = palmCenter - frame.palmDepthWorld * currentPalmDepthOffset;
        frame.palmAnchorWorld = hand;
        frame.palmAnchorWorld.translate = palmCenter + frame.palmDepthWorld * (-std::fabs(palmDepth) / 3.0f);
        frame.palmAnchorWorld.rotate =
            semanticPalmMatrixFromAxes(frame.fingerForwardWorld, frame.palmDepthWorld, frame.acrossPalmWorld);
        frame.palmAnchorWorld.scale = 1.0f;
        frame.valid = true;
        return frame;
    }

    inline RE::NiPoint3 transformSemanticHandFrameDirection(const SemanticHandFrame& frame, const RE::NiPoint3& localDirection)
    {
        if (!frame.valid) {
            return normalizedOrFallback(localDirection, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        }

        return normalizedOrFallback(
            frame.fingerForwardWorld * localDirection.x +
                frame.palmDepthWorld * localDirection.y +
                frame.acrossPalmWorld * localDirection.z,
            frame.fingerForwardWorld);
    }

    inline bool buildSemanticHandFrameFromSnapshot(
        const DirectSkeletonBoneSnapshot& snapshot,
        bool isLeft,
        SemanticHandFrame& outFrame,
        std::string* outMissingBoneName = nullptr)
    {
        outFrame = {};
        if (outMissingBoneName) {
            outMissingBoneName->clear();
        }

        const DirectSkeletonBoneEntry* handNode = nullptr;
        std::array<RE::NiPoint3, 5> fingerBases{};
        for (const auto& bone : snapshot.bones) {
            if (bone.name == (isLeft ? "LArm_Hand" : "RArm_Hand")) {
                handNode = &bone;
                break;
            }
        }
        if (!handNode) {
            if (outMissingBoneName) {
                *outMissingBoneName = isLeft ? "LArm_Hand" : "RArm_Hand";
            }
            return false;
        }

        for (std::size_t finger = 0; finger < fingerBases.size(); ++finger) {
            const char* name = fingerBoneName(isLeft, finger, 0);
            const DirectSkeletonBoneEntry* fingerNode = nullptr;
            for (const auto& bone : snapshot.bones) {
                if (name && bone.name == name) {
                    fingerNode = &bone;
                    break;
                }
            }
            if (!fingerNode) {
                if (outMissingBoneName) {
                    *outMissingBoneName = name ? name : "invalidFingerBone";
                }
                return false;
            }
            fingerBases[finger] = fingerNode->world.translate;
        }

        const RE::NiPoint3 crossPalmDirection = normalizedOrFallback(
            rotateLocalVectorToWorld(handNode->world.rotate, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }),
            RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        outFrame = buildSemanticHandFrame(handNode->world, fingerBases, crossPalmDirection);
        return outFrame.valid;
    }

    inline FingerLandmark buildFingerLandmark(const FingerChain& chain)
    {
        FingerLandmark landmark{};
        if (!chain.valid) {
            return landmark;
        }

        const float firstLength = distance(chain.points[0], chain.points[1]);
        const float secondLength = distance(chain.points[1], chain.points[2]);
        const float fullLength = firstLength + secondLength;
        if (!std::isfinite(fullLength) || fullLength <= 0.000001f) {
            return landmark;
        }

        landmark.base = chain.points[0];
        landmark.openDirection = normalizedOrFallback(chain.points[2] - chain.points[0], RE::NiPoint3(1.0f, 0.0f, 0.0f));
        landmark.length = fullLength;
        landmark.valid = true;
        return landmark;
    }

    inline LandmarkSet buildLandmarkSet(const Snapshot& snapshot)
    {
        LandmarkSet set{};
        bool allValid = snapshot.valid && snapshot.palmNormalValid;
        set.palmNormalWorld = normalizedOrFallback(snapshot.palmNormalWorld, RE::NiPoint3(0.0f, -1.0f, 0.0f));
        for (std::size_t finger = 0; finger < set.fingers.size(); ++finger) {
            set.fingers[finger] = buildFingerLandmark(snapshot.fingers[finger]);
            allValid = allValid && set.fingers[finger].valid;
        }
        set.valid = allValid;
        return set;
    }

    bool resolveLiveSemanticHandFrame(bool isLeft, SemanticHandFrame& outFrame, std::string* outMissingBoneName = nullptr);
    bool resolveLiveFingerSkeletonSnapshot(bool isLeft, Snapshot& outSnapshot, std::string* outMissingBoneName = nullptr);
}

namespace rock
{
    inline CachedSemanticHandFrameData cacheSemanticHandFrameData(const root_flattened_finger_skeleton_runtime::SemanticHandFrame& frame)
    {
        return CachedSemanticHandFrameData{
            .rawHandWorld = frame.rawHandWorld,
            .palmAnchorWorld = frame.palmAnchorWorld,
            .fingerBaseCenterWorld = frame.fingerBaseCenterWorld,
            .fingerForwardWorld = frame.fingerForwardWorld,
            .palmDepthWorld = frame.palmDepthWorld,
            .palmFaceWorld = frame.palmFaceWorld,
            .acrossPalmWorld = frame.acrossPalmWorld,
            .palmLength = frame.palmLength,
            .valid = frame.valid,
        };
    }

    inline bool HandBoneCache::resolve()
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

        root_flattened_finger_skeleton_runtime::SemanticHandFrame rightSemanticFrame{};
        root_flattened_finger_skeleton_runtime::SemanticHandFrame leftSemanticFrame{};
        if (!root_flattened_finger_skeleton_runtime::buildSemanticHandFrameFromSnapshot(snapshot, false, rightSemanticFrame) ||
            !root_flattened_finger_skeleton_runtime::buildSemanticHandFrameFromSnapshot(snapshot, true, leftSemanticFrame)) {
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
        _rightSemanticHandFrame = cacheSemanticHandFrameData(rightSemanticFrame);
        _leftSemanticHandFrame = cacheSemanticHandFrameData(leftSemanticFrame);
        _ready = true;

        if (changed) {
            logHandBoneCacheResolved(_skeleton, _boneTree, _inPowerArmor);
        }

        return true;
    }

    inline bool HandBoneCache::getSemanticHandFrame(bool isLeft, root_flattened_finger_skeleton_runtime::SemanticHandFrame& outFrame) const
    {
        outFrame = {};
        if (!isReady()) {
            return false;
        }

        const CachedSemanticHandFrameData& cached = isLeft ? _leftSemanticHandFrame : _rightSemanticHandFrame;
        if (!cached.valid) {
            return false;
        }

        outFrame.rawHandWorld = cached.rawHandWorld;
        outFrame.palmAnchorWorld = cached.palmAnchorWorld;
        outFrame.fingerBaseCenterWorld = cached.fingerBaseCenterWorld;
        outFrame.fingerForwardWorld = cached.fingerForwardWorld;
        outFrame.palmDepthWorld = cached.palmDepthWorld;
        outFrame.palmFaceWorld = cached.palmFaceWorld;
        outFrame.acrossPalmWorld = cached.acrossPalmWorld;
        outFrame.palmLength = cached.palmLength;
        outFrame.valid = true;
        return true;
    }
}

// ---- HandFrameResolver.h ----

#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

namespace rock
{
    struct HandFrame
    {
        RE::NiTransform transform{};
        RE::NiNode* node = nullptr;
        const char* label = "none";
        bool valid = false;
    };

    class HandFrameResolver
    {
    public:
        /*
         * ROCK's collision, palm selection, grab math, and debug axes use one
         * root flattened hand-frame convention. Scene nodes from another tree
         * are not returned as authority because mixing node conventions makes
         * grab frames disagree with the generated collider bodies.
         */
        HandFrame resolve(bool isLeft, bool hasRootFlattenedHand, const RE::NiTransform& rootFlattenedHandWorld) const
        {
            if (!hasRootFlattenedHand) {
                return {};
            }

            return HandFrame{
                rootFlattenedHandWorld,
                nullptr,
                isLeft ? "left-root-flattened-hand-bone" : "right-root-flattened-hand-bone",
                true
            };
        }
    };
}
