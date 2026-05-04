#pragma once

#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"
#include "physics-interaction/hand/HandColliderTypes.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "RE/Havok/hknpShape.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <atomic>
#include <cstdint>

namespace frik::rock
{
    struct HandColliderBodyMetadata
    {
        bool valid = false;
        bool isLeft = false;
        bool primaryPalmAnchor = false;
        hand_collider_semantics::HandColliderRole role = hand_collider_semantics::HandColliderRole::PalmAnchor;
        hand_collider_semantics::HandFinger finger = hand_collider_semantics::HandFinger::None;
        hand_collider_semantics::HandFingerSegment segment = hand_collider_semantics::HandFingerSegment::None;
        std::uint32_t bodyId = hand_collider_semantics::kInvalidBodyId;
    };

    class HandBoneColliderSet
    {
    public:
        HandBoneColliderSet();

        bool create(RE::hknpWorld* world, void* bhkWorld, bool isLeft, BethesdaPhysicsBody& palmAnchorBody);
        void destroy(void* bhkWorld, BethesdaPhysicsBody& palmAnchorBody);
        void reset();
        void update(RE::hknpWorld* world, bool isLeft, BethesdaPhysicsBody& palmAnchorBody, float deltaTime);
        void flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing, BethesdaPhysicsBody& palmAnchorBody);

        bool hasBodies() const { return _created; }
        std::uint32_t getBodyCount() const { return _bodyCountAtomic.load(std::memory_order_acquire); }
        std::uint32_t getBodyIdAtomic(std::size_t index) const;
        bool isColliderBodyIdAtomic(std::uint32_t bodyId) const;
        bool tryGetBodyMetadataAtomic(std::uint32_t bodyId, HandColliderBodyMetadata& outMetadata) const;
        bool tryGetBodyRoleAtomic(std::uint32_t bodyId, hand_collider_semantics::HandColliderRole& outRole) const;

    private:
        static constexpr std::size_t MAX_SEGMENT_BODIES = hand_collider_semantics::kHandSegmentColliderBodyCountPerHand;

        struct BodyInstance
        {
            BethesdaPhysicsBody body;
            const RE::hknpShape* shape = nullptr;
            hand_collider_semantics::HandColliderRole role = hand_collider_semantics::HandColliderRole::PalmFace;
            bool ownsShapeRef = false;
            GeneratedKeyframedBodyDriveState driveState{};
        };

        struct BoneFrameLookup
        {
            bool valid = false;
            RE::NiTransform hand{};
            RE::NiTransform forearm3{};
            std::array<std::array<RE::NiTransform, 3>, hand_collider_semantics::kHandFingerCount> fingers{};
            std::array<bool, hand_collider_semantics::kHandFingerCount> fingerValid{};
            std::array<RE::NiPoint3, hand_collider_semantics::kHandFingerCount> fingerBases{};
            RE::NiPoint3 backDirection{};
            bool hasForearm3 = false;
        };

        struct RoleFrameResult
        {
            bool valid = false;
            RE::NiTransform transform{};
            float length = 1.0f;
            float radius = 0.5f;
            float convexRadius = 0.1f;
        };

        bool captureBoneLookup(bool isLeft, BoneFrameLookup& outLookup);
        bool makeRoleFrame(const BoneFrameLookup& lookup, bool isLeft, hand_collider_semantics::HandColliderRole role, RoleFrameResult& outFrame) const;
        RE::hknpShape* buildShapeForRole(const RoleFrameResult& frame, hand_collider_semantics::HandColliderRole role) const;
        bool createBodyForRole(RE::hknpWorld* world, void* bhkWorld, bool isLeft, hand_collider_semantics::HandColliderRole role, const RoleFrameResult& frame, BodyInstance& instance);
        void queueBodyTarget(BethesdaPhysicsBody& body, const RE::NiTransform& target, GeneratedKeyframedBodyDriveState& driveState);
        void handleGeneratedBodyDriveResult(const GeneratedKeyframedBodyDriveResult& result, const char* ownerName, std::uint32_t bodyIndex);
        void clearInstance(BodyInstance& instance, bool releaseShapeRef);
        void publishAtomicBodyIds(const BethesdaPhysicsBody& palmAnchorBody, bool isLeft);
        void clearAtomicBodyIds();

        DirectSkeletonBoneReader _reader;
        std::array<BodyInstance, MAX_SEGMENT_BODIES> _bodies{};
        RE::hknpWorld* _cachedWorld = nullptr;
        void* _cachedBhkWorld = nullptr;
        GeneratedKeyframedBodyDriveState _palmAnchorDriveState{};
        const void* _cachedSkeleton = nullptr;
        const void* _cachedBoneTree = nullptr;
        bool _cachedPowerArmor = false;
        const void* _lastCapturedSkeleton = nullptr;
        const void* _lastCapturedBoneTree = nullptr;
        bool _lastCapturedPowerArmor = false;
        std::atomic<std::uint32_t> _isLeftAtomic{ 0 };
        std::atomic<bool> _driveRebuildRequested{ false };
        std::atomic<std::uint32_t> _driveFailureCount{ 0 };
        bool _created = false;
        int _updateLogCounter = 0;

        std::array<std::atomic<std::uint32_t>, hand_collider_semantics::kHandColliderBodyCountPerHand> _bodyIdsAtomic{};
        std::array<std::atomic<std::uint32_t>, hand_collider_semantics::kHandColliderBodyCountPerHand> _rolesAtomic{};
        std::array<std::atomic<std::uint32_t>, hand_collider_semantics::kHandColliderBodyCountPerHand> _fingersAtomic{};
        std::array<std::atomic<std::uint32_t>, hand_collider_semantics::kHandColliderBodyCountPerHand> _segmentsAtomic{};
        std::array<std::atomic<std::uint32_t>, hand_collider_semantics::kHandColliderBodyCountPerHand> _primaryAnchorAtomic{};
        std::atomic<std::uint32_t> _bodyCountAtomic{ 0 };
    };
}
