#pragma once

#include "physics-interaction/debug/SkeletonBoneDebugMath.h"
#include "physics-interaction/hand/HandColliderTypes.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "RE/Havok/hknpShape.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace rock
{
    inline constexpr std::size_t kBodyBoneColliderBodyCount = skeleton_bone_debug_math::kStandardBodyColliderDescriptors.size();
    inline constexpr std::uint32_t kInvalidBodyBoneColliderBodyId = 0x7FFF'FFFFu;

    struct BodyBoneColliderMetadata
    {
        bool valid = false;
        bool inPowerArmor = false;
        skeleton_bone_debug_math::BoneColliderRole role = skeleton_bone_debug_math::BoneColliderRole::TorsoSegment;
        body_zone::BodyZoneKind zone = body_zone::BodyZoneKind::Unknown;
        body_zone::BodyZoneSide side = body_zone::BodyZoneSide::Center;
        std::uint32_t bodyId = kInvalidBodyBoneColliderBodyId;
        std::uint32_t descriptorIndex = 0;
        float lengthGameUnits = 0.0f;
        float radiusGameUnits = 0.0f;
    };

    class BodyBoneColliderSet
    {
    public:
        struct DescriptorFrameResult
        {
            bool valid = false;
            RE::NiTransform transform{};
            float length = 1.0f;
            float radius = 0.5f;
            float convexRadius = 0.1f;
        };

        BodyBoneColliderSet();

        bool create(RE::hknpWorld* world, void* bhkWorld);
        void destroy(void* bhkWorld);
        void reset();
        void update(RE::hknpWorld* world, float deltaTime);
        void flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);

        bool hasBodies() const { return _created; }
        std::uint32_t getBodyCount() const { return _bodyCountAtomic.load(std::memory_order_acquire); }
        std::uint32_t getBodyIdAtomic(std::size_t index) const;
        bool isColliderBodyIdAtomic(std::uint32_t bodyId) const;
        bool tryGetBodyMetadataAtomic(std::uint32_t bodyId, BodyBoneColliderMetadata& outMetadata) const;
        bool tryGetBodyRoleAtomic(std::uint32_t bodyId, skeleton_bone_debug_math::BoneColliderRole& outRole) const;
        bool isRebuildPendingAtomic() const { return _driveRebuildRequested.load(std::memory_order_acquire); }

    private:
        struct BodyInstance
        {
            BethesdaPhysicsBody body;
            const RE::hknpShape* shape = nullptr;
            skeleton_bone_debug_math::BoneColliderRole role = skeleton_bone_debug_math::BoneColliderRole::TorsoSegment;
            body_zone::BodyZoneKind zone = body_zone::BodyZoneKind::Unknown;
            body_zone::BodyZoneSide side = body_zone::BodyZoneSide::Center;
            std::uint32_t descriptorIndex = 0;
            float lengthGameUnits = 0.0f;
            float radiusGameUnits = 0.0f;
            bool ownsShapeRef = false;
            GeneratedKeyframedBodyDriveState driveState{};
        };

        bool captureBoneSnapshot(DirectSkeletonBoneSnapshot& outSnapshot);
        RE::hknpShape* buildShapeForFrame(const DescriptorFrameResult& frame) const;
        bool createBodyForDescriptor(
            RE::hknpWorld* world,
            void* bhkWorld,
            const skeleton_bone_debug_math::BoneColliderDescriptor& descriptor,
            std::uint32_t descriptorIndex,
            const DescriptorFrameResult& frame,
            BodyInstance& instance);
        void queueBodyTarget(BethesdaPhysicsBody& body, const RE::NiTransform& target, float sourceDeltaSeconds, GeneratedKeyframedBodyDriveState& driveState);
        void handleGeneratedBodyDriveResult(const GeneratedKeyframedBodyDriveResult& result, const char* ownerName, std::uint32_t bodyIndex);
        void clearInstance(BodyInstance& instance, bool releaseShapeRef);
        void publishAtomicBodyIds(bool inPowerArmor);
        void clearAtomicBodyIds();

        DirectSkeletonBoneReader _reader;
        std::array<BodyInstance, kBodyBoneColliderBodyCount> _bodies{};
        RE::hknpWorld* _cachedWorld = nullptr;
        void* _cachedBhkWorld = nullptr;
        const void* _cachedSkeleton = nullptr;
        const void* _cachedBoneTree = nullptr;
        bool _cachedPowerArmor = false;
        std::uint64_t _cachedTuningSignature = 0;
        const void* _lastCapturedSkeleton = nullptr;
        const void* _lastCapturedBoneTree = nullptr;
        bool _lastCapturedPowerArmor = false;
        std::atomic<bool> _driveRebuildRequested{ false };
        std::atomic<std::uint32_t> _driveFailureCount{ 0 };
        bool _created = false;
        int _updateLogCounter = 0;

        std::array<std::atomic<std::uint32_t>, kBodyBoneColliderBodyCount> _bodyIdsAtomic{};
        std::array<std::atomic<std::uint32_t>, kBodyBoneColliderBodyCount> _rolesAtomic{};
        std::array<std::atomic<std::uint32_t>, kBodyBoneColliderBodyCount> _zonesAtomic{};
        std::array<std::atomic<std::uint32_t>, kBodyBoneColliderBodyCount> _sidesAtomic{};
        std::array<std::atomic<std::uint32_t>, kBodyBoneColliderBodyCount> _descriptorIndicesAtomic{};
        std::array<std::atomic<std::uint32_t>, kBodyBoneColliderBodyCount> _powerArmorAtomic{};
        std::array<std::atomic<float>, kBodyBoneColliderBodyCount> _lengthsGameAtomic{};
        std::array<std::atomic<float>, kBodyBoneColliderBodyCount> _radiiGameAtomic{};
        std::atomic<std::uint32_t> _bodyCountAtomic{ 0 };
    };
}
