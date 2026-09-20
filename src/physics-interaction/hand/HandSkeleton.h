#pragma once

/*
 * Hand skeleton helpers are grouped here so direct skeleton reads, cached hand frames, root-flattened runtime, and frame resolution stay on one authority path.
 */


// ---- DirectSkeletonBoneReader.h ----

#include <cstdint>
#include <array>
#include <string>
#include <vector>

#include "physics-interaction/debug/SkeletonBoneDebugMath.h"
#include "physics-interaction/hand/RenderedBoneTransportPolicy.h"

#include "RE/NetImmerse/NiTransform.h"

namespace rock
{
    /*
     * Rendered: the live root flattened array, final only after WorldFinal.
     * Controller: each sampled hand chain (forearm, hand, fingers) carried
     * from that array's own wrist to the isolated controller hand.
     */
    enum class SkeletonBoneCaptureSpace : std::uint8_t
    {
        Rendered,
        Controller,
    };

    enum class SkeletonBoneCapturePayload : std::uint8_t
    {
        FlattenedTransforms,
        HandRootsWithSceneNodes,
    };

    struct DirectSkeletonBoneEntry
    {
        std::string name;
        int treeIndex = -1;
        int parentTreeIndex = -1;
        int drawableParentSnapshotIndex = -1;
        RE::NiTransform world{};
        // Independent live refNode world, always in scene space. Arm IK writes
        // it before the flattened array is final; controller transport must
        // not apply the array's delta to this already updated scene node.
        RE::NiTransform nodeWorld{};
        bool nodeWorldValid = false;
        bool included = false;
    };

    struct DirectSkeletonBoneSnapshot
    {
        const void* topologyOwner = nullptr; // Reader identity only; never dereferenced.
        std::uint64_t topologyRevision = 0;
        bool valid = false;
        bool inPowerArmor = false;
        skeleton_bone_debug_math::DebugSkeletonBoneMode mode = skeleton_bone_debug_math::DebugSkeletonBoneMode::Off;
        skeleton_bone_debug_math::SkeletonBoneSnapshotSource source = skeleton_bone_debug_math::SkeletonBoneSnapshotSource::None;
        SkeletonBoneCaptureSpace space = SkeletonBoneCaptureSpace::Rendered;
        SkeletonBoneCapturePayload payload = SkeletonBoneCapturePayload::FlattenedTransforms;
        std::array<bool, 2> controllerHandsValid{};
        const void* skeleton = nullptr;
        const void* boneTree = nullptr;
        int totalBoneCount = 0;
        int requiredResolvedCount = 0;
        std::vector<DirectSkeletonBoneEntry> bones;
        std::vector<std::string> missingRequiredBones;
    };

    class SkeletonBoneNameIndex;
    // Converts this copied array once, using its own wrists as the source.
    bool transportControllerHands(DirectSkeletonBoneSnapshot& snapshot);

    class DirectSkeletonBoneReader
    {
    public:
        bool capture(
            skeleton_bone_debug_math::DebugSkeletonBoneMode mode,
            skeleton_bone_debug_math::DebugSkeletonBoneSource source,
            SkeletonBoneCaptureSpace space,
            DirectSkeletonBoneSnapshot& outSnapshot,
            SkeletonBoneCapturePayload payload = SkeletonBoneCapturePayload::FlattenedTransforms);
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

        bool captureFromCachedTree(DirectSkeletonBoneSnapshot& outSnapshot, SkeletonBoneCaptureSpace space, SkeletonBoneCapturePayload payload);

        const void* _cachedSkeleton = nullptr;
        void* _cachedBoneTree = nullptr;
        skeleton_bone_debug_math::SkeletonBoneSnapshotSource _cachedSource = skeleton_bone_debug_math::SkeletonBoneSnapshotSource::None;
        skeleton_bone_debug_math::DebugSkeletonBoneMode _cachedMode = skeleton_bone_debug_math::DebugSkeletonBoneMode::Off;
        int _cachedTotalBoneCount = 0;
        bool _cachedInPowerArmor = false;
        bool _missingSourceLogged = false;
        int _cachedRequiredResolvedCount = 0;
        std::vector<CachedBone> _cachedBones;
        std::uint64_t _topologyRevision = 0;
        std::vector<std::string> _cachedMissingRequiredBones;
    };
}

// ---- HandBoneCache.h ----

#include "RE/NetImmerse/NiTransform.h"

#include <string_view>

namespace rock
{
    void logHandBoneCacheResolved(const void* skeleton, const void* boneTree, bool inPowerArmor);

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
            // Rendered on purpose: this cache is the controller-hand isolation's input.
            auto& snapshot = _snapshot;
            if (!_reader.capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                    skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                    SkeletonBoneCaptureSpace::Rendered,
                    snapshot, SkeletonBoneCapturePayload::HandRootsWithSceneNodes)) {
                clearResolvedState();
                return false;
            }

            const DirectSkeletonBoneEntry* rightHand = findBone(snapshot, "RArm_Hand");
            const DirectSkeletonBoneEntry* leftHand = findBone(snapshot, "LArm_Hand");
            if (!rightHand || !leftHand) {
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
            _rightHandWorld = rightHand->world;
            _leftHandWorld = leftHand->world;
            _rightHandNodeWorld = rightHand->nodeWorld;
            _leftHandNodeWorld = leftHand->nodeWorld;
            _rightHandNodeWorldValid = rightHand->nodeWorldValid;
            _leftHandNodeWorldValid = leftHand->nodeWorldValid;
            _ready = true;

            if (changed) {
                logHandBoneCacheResolved(_skeleton, _boneTree, _inPowerArmor);
            }

            return true;
        }

        void reset()
        {
            clearResolvedState();
            _reader.resetCache();
            _snapshot = {};
        }

        [[nodiscard]] bool isReady() const { return _ready && _skeleton && _boneTree; }

        [[nodiscard]] RE::NiTransform getWorldTransform(bool isLeft) const
        {
            return isLeft ? _leftHandWorld : _rightHandWorld;
        }

        /*
         * The hand bone's refNode world (FRIK's solved body hand, unblended).
         * False when the tree carries no readable node for the bone.
         */
        [[nodiscard]] bool tryGetNodeWorldTransform(bool isLeft, RE::NiTransform& outWorld) const
        {
            const bool valid = isLeft ? _leftHandNodeWorldValid : _rightHandNodeWorldValid;
            outWorld = valid ? (isLeft ? _leftHandNodeWorld : _rightHandNodeWorld) : RE::NiTransform{};
            return _ready && valid;
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
            _rightHandNodeWorld = {};
            _leftHandNodeWorld = {};
            _rightHandNodeWorldValid = false;
            _leftHandNodeWorldValid = false;
            _ready = false;
        }

        static const DirectSkeletonBoneEntry* findBone(const DirectSkeletonBoneSnapshot& snapshot, std::string_view name)
        {
            for (const auto& bone : snapshot.bones) {
                if (bone.name == name) {
                    return &bone;
                }
            }
            return nullptr;
        }

        DirectSkeletonBoneReader _reader;
        DirectSkeletonBoneSnapshot _snapshot;
        const void* _skeleton = nullptr;
        const void* _boneTree = nullptr;
        bool _inPowerArmor = false;
        RE::NiTransform _rightHandWorld{};
        RE::NiTransform _leftHandWorld{};
        RE::NiTransform _rightHandNodeWorld{};
        RE::NiTransform _leftHandNodeWorld{};
        bool _rightHandNodeWorldValid = false;
        bool _leftHandNodeWorldValid = false;
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
        RE::NiPoint3 tipSegmentCenterWorld{};
        RE::NiPoint3 tipDirectionWorld{};
        bool tipGeometryValid = false;
        bool valid = false;
    };

    struct Snapshot
    {
        std::array<FingerChain, 5> fingers{};
        RE::NiPoint3 palmNormalWorld{ 0.0f, 0.0f, -1.0f };
        bool palmNormalValid = false;
        bool inPowerArmor = false;
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
        RE::NiPoint3 palmNormalWorld{ 0.0f, 0.0f, -1.0f };
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
        set.palmNormalWorld = normalizedOrFallback(snapshot.palmNormalWorld, RE::NiPoint3(0.0f, 0.0f, -1.0f));
        for (std::size_t finger = 0; finger < set.fingers.size(); ++finger) {
            set.fingers[finger] = buildFingerLandmark(snapshot.fingers[finger]);
            allValid = allValid && set.fingers[finger].valid;
        }
        set.valid = allValid;
        return set;
    }

    bool buildFingerSkeletonSnapshot(
        const DirectSkeletonBoneSnapshot& boneSnapshot,
        bool isLeft,
        Snapshot& outSnapshot,
        std::string* outMissingBoneName = nullptr,
        SkeletonBoneNameIndex* nameIndex = nullptr);
    bool resolveLiveFingerSkeletonSnapshot(bool isLeft, Snapshot& outSnapshot, std::string* outMissingBoneName = nullptr,
        SkeletonBoneCaptureSpace space = SkeletonBoneCaptureSpace::Controller);
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
