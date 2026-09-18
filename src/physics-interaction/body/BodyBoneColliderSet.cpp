#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/hand/DynamicHandCollisionKinematics.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/native/HavokConvexShapeBuilder.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <string_view>
#include <vector>

namespace rock
{
    namespace
    {
        using skeleton_bone_debug_math::BoneColliderDescriptor;
        using skeleton_bone_debug_math::BoneColliderRole;

        constexpr std::uint32_t kBodyFilterInfo = (0x000B << 16) | (collision_layer_policy::ROCK_LAYER_BODY & 0x7F);

        using BodyDescriptorArray = std::array<BoneColliderDescriptor, kBodyBoneColliderBodyCount>;
        using SnapshotBoneLookup = SkeletonBoneNameIndex::View;

        const BodyDescriptorArray& bodyDescriptorsForPowerArmor(bool inPowerArmor)
        {
            return inPowerArmor ? skeleton_bone_debug_math::kPowerArmorBodyColliderDescriptors :
                                  skeleton_bone_debug_math::kStandardBodyColliderDescriptors;
        }

        bool bodyColliderRoleEnabled(BoneColliderRole role)
        {
            if (role == BoneColliderRole::LegSegment || role == BoneColliderRole::FootSegment) {
                return g_rockConfig.rockBodyBoneLegAndFootCollidersEnabled;
            }
            return true;
        }

        const char* roleName(BoneColliderRole role)
        {
            switch (role) {
            case BoneColliderRole::UpperArmSegment:
                return "UpperArm";
            case BoneColliderRole::ForearmSegment:
                return "Forearm";
            case BoneColliderRole::HandSegment:
                return "Hand";
            case BoneColliderRole::FingerSegment:
                return "Finger";
            case BoneColliderRole::TorsoSegment:
                return "Torso";
            case BoneColliderRole::LegSegment:
                return "Leg";
            case BoneColliderRole::FootSegment:
                return "Foot";
            }
            return "Unknown";
        }

        hand_bone_collider_geometry_math::ColliderDimensionLimits bodyRoleDimensionLimits(BoneColliderRole role, bool inPowerArmor)
        {
            hand_bone_collider_geometry_math::ColliderDimensionLimits limits{};
            limits.maxConvexRadius = inPowerArmor ? 2.5f : 1.75f;

            switch (role) {
            case BoneColliderRole::TorsoSegment:
                limits.maxLength = inPowerArmor ? 70.0f : 50.0f;
                limits.maxRadius = inPowerArmor ? 24.0f : 16.0f;
                limits.maxLongAxisExtent = inPowerArmor ? 115.0f : 80.0f;
                return limits;
            case BoneColliderRole::UpperArmSegment:
            case BoneColliderRole::ForearmSegment:
            case BoneColliderRole::HandSegment:
            case BoneColliderRole::FingerSegment:
                limits.maxLength = inPowerArmor ? 68.0f : 46.0f;
                limits.maxRadius = inPowerArmor ? 18.0f : 11.0f;
                limits.maxLongAxisExtent = inPowerArmor ? 100.0f : 66.0f;
                return limits;
            case BoneColliderRole::LegSegment:
                limits.maxLength = inPowerArmor ? 88.0f : 62.0f;
                limits.maxRadius = inPowerArmor ? 20.0f : 13.0f;
                limits.maxLongAxisExtent = inPowerArmor ? 125.0f : 86.0f;
                return limits;
            case BoneColliderRole::FootSegment:
                limits.maxLength = inPowerArmor ? 58.0f : 38.0f;
                limits.maxRadius = inPowerArmor ? 15.0f : 9.0f;
                limits.maxLongAxisExtent = inPowerArmor ? 85.0f : 55.0f;
                return limits;
            }

            return limits;
        }

        bool descriptorFrameDimensionsValid(const BodyBoneColliderSet::DescriptorFrameResult& frame, BoneColliderRole role, bool inPowerArmor)
        {
            return frame.valid &&
                   hand_bone_collider_geometry_math::colliderDimensionsWithinLimits(
                       frame.length,
                       frame.radius,
                       frame.convexRadius,
                       bodyRoleDimensionLimits(role, inPowerArmor));
        }

        bool findSnapshotBone(const SnapshotBoneLookup& bonesByName, std::string_view name, RE::NiTransform& outTransform)
        {
            const auto* bone = bonesByName.find(name);
            if (!bone) return false;
            outTransform = bone->world;
            return true;
        }

        bool makeDescriptorFrame(
            const SnapshotBoneLookup& bonesByName,
            const BoneColliderDescriptor& descriptor,
            bool inPowerArmor,
            const collider_tuning::BodyDescriptor& tuning,
            BodyBoneColliderSet::DescriptorFrameResult& outFrame)
        {
            outFrame = {};
            if (!descriptor.enabled || descriptor.endpointMode != skeleton_bone_debug_math::BoneColliderEndpointMode::ChildBone) {
                return false;
            }

            hand_bone_collider_geometry_math::BoneColliderFrameInput<RE::NiTransform, RE::NiPoint3> input{};
            input.radius = tuning.radius;
            input.convexRadius = tuning.convexRadius;

            if (!findSnapshotBone(bonesByName, descriptor.startBone, input.start) ||
                !findSnapshotBone(bonesByName, descriptor.endBone, input.end)) {
                return false;
            }

            skeleton_bone_debug_math::applyColliderEndpointRange(descriptor, input.start.translate, input.end.translate);
            const auto frame = hand_bone_collider_geometry_math::buildSegmentColliderFrame(input);
            if (!frame.valid) {
                return false;
            }

            outFrame.valid = true;
            outFrame.transform = frame.transform;
            if (tuning.hasLocalOffset) {
                const auto offsetWorld =
                    hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(outFrame.transform, tuning.localOffsetGame);
                outFrame.transform.translate.x += offsetWorld.x;
                outFrame.transform.translate.y += offsetWorld.y;
                outFrame.transform.translate.z += offsetWorld.z;
            }
            outFrame.length = frame.length * tuning.lengthScale;
            outFrame.radius = frame.radius;
            outFrame.convexRadius = frame.convexRadius;
            if (!descriptorFrameDimensionsValid(outFrame, descriptor.role, inPowerArmor)) {
                ROCK_LOG_WARN(Body,
                    "Rejected implausible body collider frame role={} zone={} length={:.2f} radius={:.2f} convex={:.2f} bones={}->{} powerArmor={}",
                    roleName(descriptor.role),
                    body_zone::bodyZoneName(descriptor.zone),
                    outFrame.length,
                    outFrame.radius,
                    outFrame.convexRadius,
                    descriptor.startBone,
                    descriptor.endBone,
                    inPowerArmor ? "yes" : "no");
                outFrame = {};
                return false;
            }
            return true;
        }

        inline constexpr std::size_t kForearmUpperMergeSource = 0;
        inline constexpr std::size_t kForearmLowerMergeSource = 1;
        inline constexpr std::size_t kWristMergeSource = 2;
        inline constexpr std::size_t kForearmMergeSourceCount = 3;

        struct ForearmTwinMergeSources
        {
            std::array<BodyBoneColliderSet::DescriptorFrameResult, kForearmMergeSourceCount> right{};
            std::array<BodyBoneColliderSet::DescriptorFrameResult, kForearmMergeSourceCount> left{};
        };

        std::array<BodyBoneColliderSet::DescriptorFrameResult, kForearmMergeSourceCount>* forearmMergeSourcesForDescriptor(
            ForearmTwinMergeSources& sources,
            const BoneColliderDescriptor& descriptor,
            std::size_t& outSourceIndex)
        {
            auto* sideSources = descriptor.side == body_zone::BodyZoneSide::Left ? &sources.left :
                                descriptor.side == body_zone::BodyZoneSide::Right ? &sources.right : nullptr;
            if (!sideSources) {
                return nullptr;
            }

            if (descriptor.role == BoneColliderRole::ForearmSegment) {
                switch (descriptor.zone) {
                case body_zone::BodyZoneKind::LeftForearmUpper:
                case body_zone::BodyZoneKind::RightForearmUpper:
                    outSourceIndex = kForearmUpperMergeSource;
                    return sideSources;
                case body_zone::BodyZoneKind::LeftForearmLower:
                case body_zone::BodyZoneKind::RightForearmLower:
                    outSourceIndex = kForearmLowerMergeSource;
                    return sideSources;
                default:
                    return nullptr;
                }
            }

            if (descriptor.role == BoneColliderRole::HandSegment &&
                (descriptor.zone == body_zone::BodyZoneKind::LeftHand ||
                    descriptor.zone == body_zone::BodyZoneKind::RightHand)) {
                outSourceIndex = kWristMergeSource;
                return sideSources;
            }
            return nullptr;
        }

        void collectForearmTwinMergeSource(
            ForearmTwinMergeSources& sources,
            const BoneColliderDescriptor& descriptor,
            const BodyBoneColliderSet::DescriptorFrameResult& frame)
        {
            std::size_t sourceIndex = 0;
            auto* sideSources = forearmMergeSourcesForDescriptor(sources, descriptor, sourceIndex);
            if (sideSources && sourceIndex < sideSources->size() && frame.valid) {
                (*sideSources)[sourceIndex] = frame;
            }
        }

        void publishMergedForearmTwinSlot(
            dynamic_hand_twin::TwinSlotFrame& slot,
            const std::array<BodyBoneColliderSet::DescriptorFrameResult, kForearmMergeSourceCount>& sources,
            const SnapshotBoneLookup& bonesByName,
            bool inPowerArmor,
            bool isLeft)
        {
            if (!std::all_of(sources.begin(), sources.end(), [](const auto& source) { return source.valid; })) {
                return;
            }

            const std::string_view shoulderBone = isLeft ? "LArm_UpperArm" : "RArm_UpperArm";
            const std::string_view forearmStartBone = isLeft ? "LArm_ForeArm1" : "RArm_ForeArm1";
            const std::string_view handBone = isLeft ? "LArm_Hand" : "RArm_Hand";
            RE::NiTransform shoulder{};
            hand_bone_collider_geometry_math::BoneColliderFrameInput<RE::NiTransform, RE::NiPoint3> input{};
            input.radius = std::max({ sources[0].radius, sources[1].radius, sources[2].radius });
            input.convexRadius = std::max({ sources[0].convexRadius, sources[1].convexRadius, sources[2].convexRadius });
            if (!findSnapshotBone(bonesByName, shoulderBone, shoulder) ||
                !findSnapshotBone(bonesByName, forearmStartBone, input.start) ||
                !findSnapshotBone(bonesByName, handBone, input.end)) {
                return;
            }

            const auto mergedGeometry = hand_bone_collider_geometry_math::buildSegmentColliderFrame(input);
            if (!mergedGeometry.valid) {
                return;
            }

            BodyBoneColliderSet::DescriptorFrameResult mergedFrame{};
            mergedFrame.valid = true;
            mergedFrame.transform = mergedGeometry.transform;
            mergedFrame.length = sources[0].length + sources[1].length + sources[2].length;
            mergedFrame.radius = input.radius;
            mergedFrame.convexRadius = input.convexRadius;
            if (!descriptorFrameDimensionsValid(mergedFrame, BoneColliderRole::ForearmSegment, inPowerArmor)) {
                return;
            }

            // The twin belongs to the dynamic hand compound, which is built in
            // controller space: carry it with the hand chain so its shape and
            // its palm and finger twins share one frame under a ROCK claim.
            RE::NiTransform controllerRoot{};
            if (!frik_hand_world_authority::tryGetRawHandWorld(isLeft, controllerRoot) ||
                !tracked_hand_isolation_policy::isFiniteTransform(input.end)) {
                return;
            }
            const auto transport = rendered_bone_transport_policy::makeHandTransport(
                controllerRoot, true, input.end, true);
            slot.valid = true;
            slot.target = rendered_bone_transport_policy::transportWorld(transport, mergedFrame.transform);
            slot.length = mergedFrame.length;
            slot.radius = mergedFrame.radius;
            slot.convexRadius = mergedFrame.convexRadius;
            slot.handTargetResponseScale = dynamic_hand_collision_kinematics::forearmHandTargetResponseScale(
                shoulder.translate,
                input.end.translate,
                mergedFrame.transform.translate);
        }

        void publishMergedForearmTwinTargets(
            dynamic_hand_twin::ForearmTwinTargets& targets,
            const ForearmTwinMergeSources& sources,
            const SnapshotBoneLookup& bonesByName,
            bool inPowerArmor)
        {
            publishMergedForearmTwinSlot(targets.right[0], sources.right, bonesByName, inPowerArmor, false);
            publishMergedForearmTwinSlot(targets.left[0], sources.left, bonesByName, inPowerArmor, true);
        }

        void shapeRemoveRef(const RE::hknpShape* shape)
        {
            havok_ref_count::release(shape);
        }

        std::vector<RE::NiPoint3> toHavokPointCloud(const std::vector<RE::NiPoint3>& gamePoints)
        {
            std::vector<RE::NiPoint3> havokPoints;
            havokPoints.reserve(gamePoints.size());
            for (const auto& point : gamePoints) {
                havokPoints.emplace_back(point.x * gameToHavokScale(), point.y * gameToHavokScale(), point.z * gameToHavokScale());
            }
            return havokPoints;
        }
    }

    std::uint64_t BodyBoneColliderSet::refreshTuning(bool powerArmor)
    {
        const auto revision = g_rockConfig.configRevision();
        if (!_tuningReady || _tuningConfigRevision != revision || _tuningPowerArmor != powerArmor) {
            _tuning = collider_tuning::prepareBody(g_rockConfig, powerArmor);
            _tuningConfigRevision = revision;
            _tuningPowerArmor = powerArmor;
            _tuningReady = true;
        }
        // Keep the original effective signature: unrelated config edits must
        // not rebuild native colliders, and constrained hands still defer.
        return _tuning.signature;
    }

    BodyBoneColliderSet::BodyBoneColliderSet()
    {
        clearAtomicBodyIds();
    }

    bool BodyBoneColliderSet::captureBoneSnapshot(DirectSkeletonBoneSnapshot& outSnapshot)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::BodyBoneCapture);
        // Body colliders collide where the body draws, so the whole snapshot
        // stays rendered; the forearm twin handed to the dynamic hand compound
        // is carried to the controller hand on its own below.
        if (!_reader.capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::AllFlattenedBones,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                SkeletonBoneCaptureSpace::Rendered,
                outSnapshot)) {
            return false;
        }

        _lastCapturedSkeleton = outSnapshot.skeleton;
        _lastCapturedBoneTree = outSnapshot.boneTree;
        _lastCapturedPowerArmor = outSnapshot.inPowerArmor;
        return outSnapshot.valid;
    }

    RE::hknpShape* BodyBoneColliderSet::buildShapeForFrame(const DescriptorFrameResult& frame) const
    {
        if (!frame.valid) {
            return nullptr;
        }

        const auto gamePoints = hand_bone_collider_geometry_math::makeCapsuleLikeHullPoints<RE::NiPoint3>(frame.length, frame.radius);
        return havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(toHavokPointCloud(gamePoints), frame.convexRadius * gameToHavokScale());
    }

    RE::hknpShape* BodyBoneColliderSet::buildDynamicForearmTwinShape(const dynamic_hand_twin::TwinSlotFrame& slotFrame) const
    {
        DescriptorFrameResult frame{};
        frame.valid = slotFrame.valid;
        frame.length = slotFrame.length;
        frame.radius = slotFrame.radius;
        frame.convexRadius = slotFrame.convexRadius;
        return buildShapeForFrame(frame);
    }

    bool BodyBoneColliderSet::createBodyForDescriptor(
        RE::hknpWorld* world,
        void* bhkWorld,
        const BoneColliderDescriptor& descriptor,
        std::uint32_t descriptorIndex,
        const DescriptorFrameResult& frame,
        BodyInstance& instance)
    {
        auto* shape = buildShapeForFrame(frame);
        if (!shape) {
            ROCK_LOG_WARN(Body,
                "Body collider shape build failed index={} role={} {}->{}",
                descriptorIndex,
                roleName(descriptor.role),
                descriptor.startBone,
                descriptor.endBone);
            return false;
        }

        instance.shape = shape;
        instance.role = descriptor.role;
        instance.zone = descriptor.zone;
        instance.side = descriptor.side;
        instance.descriptorIndex = descriptorIndex;
        instance.lengthGameUnits = frame.length;
        instance.radiusGameUnits = frame.radius;
        instance.ownsShapeRef = true;
        clearGeneratedKeyframedBodyDriveState(instance.driveState);

        const std::string name = std::string("ROCK_Body_") + roleName(descriptor.role) + "_" + std::to_string(descriptorIndex);
        if (!instance.body.create(world, bhkWorld, shape, kBodyFilterInfo, havok_material_registry::registerGeneratedBodyMaterial(world), BethesdaMotionType::Keyframed, name.c_str())) {
            shapeRemoveRef(shape);
            clearInstance(instance, false);
            ROCK_LOG_ERROR(Body,
                "Body collider create failed index={} role={} {}->{}",
                descriptorIndex,
                roleName(descriptor.role),
                descriptor.startBone,
                descriptor.endBone);
            return false;
        }

        instance.body.createNiNode(name.c_str());
        if (!placeGeneratedKeyframedBodyImmediately(instance.body, frame.transform)) {
            ROCK_LOG_ERROR(Body,
                "Body collider initial placement failed index={} bodyId={}; destroying generated body",
                descriptorIndex,
                instance.body.getBodyId().value);
            instance.body.retireDeferred(bhkWorld);
            shapeRemoveRef(shape);
            clearInstance(instance, false);
            return false;
        }

        initializeGeneratedKeyframedBodyDriveState(instance.driveState, frame.transform);
        ROCK_LOG_DEBUG(Body,
            "Body collider created index={} role={} zone={} side={} bodyId={} length={:.2f} radius={:.2f} {}->{}",
            descriptorIndex,
            roleName(descriptor.role),
            body_zone::bodyZoneName(descriptor.zone),
            body_zone::bodyZoneSideName(descriptor.side),
            instance.body.getBodyId().value,
            frame.length,
            frame.radius,
            descriptor.startBone,
            descriptor.endBone);
        return true;
    }

    bool BodyBoneColliderSet::create(RE::hknpWorld* world, void* bhkWorld)
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        destroy(bhkWorld);
        if (!world || !bhkWorld) {
            return false;
        }

        auto& snapshot = _snapshot;
        if (!captureBoneSnapshot(snapshot)) {
            ROCK_LOG_WARN(Body, "Body bone colliders not created: root flattened skeleton snapshot unavailable");
            return false;
        }

        const auto& descriptors = bodyDescriptorsForPowerArmor(snapshot.inPowerArmor);
        const auto tuningSignature = refreshTuning(snapshot.inPowerArmor);
        const auto bonesByName = _boneNameIndex.bind(snapshot);
        dynamic_hand_twin::ForearmTwinTargets forearmTwinTargets{};
        ForearmTwinMergeSources forearmTwinMergeSources{};
        std::size_t createdCount = 0;
        for (std::uint32_t descriptorIndex = 0; descriptorIndex < descriptors.size(); ++descriptorIndex) {
            const auto& descriptor = descriptors[descriptorIndex];
            if (!bodyColliderRoleEnabled(descriptor.role)) {
                continue;
            }

            DescriptorFrameResult frame{};
            if (!makeDescriptorFrame(bonesByName, descriptor, snapshot.inPowerArmor, _tuning.descriptors[descriptorIndex], frame)) {
                ROCK_LOG_WARN(Body,
                    "Body collider frame missing index={} role={} {}->{}",
                    descriptorIndex,
                    roleName(descriptor.role),
                    descriptor.startBone,
                    descriptor.endBone);
                continue;
            }

            if (createdCount >= _bodies.size()) {
                break;
            }

            if (!createBodyForDescriptor(world, bhkWorld, descriptor, descriptorIndex, frame, _bodies[createdCount])) {
                ROCK_LOG_ERROR(Body, "Body collider set creation failed; destroying partial set");
                destroy(bhkWorld);
                return false;
            }
            collectForearmTwinMergeSource(forearmTwinMergeSources, descriptor, frame);
            ++createdCount;
        }

        publishMergedForearmTwinTargets(
            forearmTwinTargets,
            forearmTwinMergeSources,
            bonesByName,
            snapshot.inPowerArmor);

        if (createdCount == 0) {
            ROCK_LOG_ERROR(Body, "Body bone collider set creation produced zero bodies");
            destroy(bhkWorld);
            return false;
        }

        _cachedWorld = world;
        _cachedBhkWorld = bhkWorld;
        _cachedSkeleton = _lastCapturedSkeleton;
        _cachedBoneTree = _lastCapturedBoneTree;
        _cachedPowerArmor = _lastCapturedPowerArmor;
        _cachedTuningSignature = tuningSignature;
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _created = true;
        if (++_dynamicForearmGeometryGeneration == 0) {
            _dynamicForearmGeometryGeneration = 1;
        }
        forearmTwinTargets.geometryGeneration = _dynamicForearmGeometryGeneration;
        _canonicalForearmTwinDimensions = forearmTwinTargets;
        forearmTwinTargets.updateCounter = _dynamicForearmTwinTargets.updateCounter + 1;
        _dynamicForearmTwinTargets = forearmTwinTargets;
        publishAtomicBodyIds(snapshot.inPowerArmor);

        ROCK_LOG_INFO(Body,
            "Body bone colliders created: bodies={} descriptors={} legsAndFeet={} sourceSkeleton={} tree={} powerArmor={}",
            createdCount,
            descriptors.size(),
            g_rockConfig.rockBodyBoneLegAndFootCollidersEnabled ? "enabled" : "disabled",
            reinterpret_cast<std::uintptr_t>(_cachedSkeleton),
            reinterpret_cast<std::uintptr_t>(_cachedBoneTree),
            _cachedPowerArmor ? "yes" : "no");
        return true;
    }

    void BodyBoneColliderSet::destroy(void* bhkWorld)
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        clearAtomicBodyIds();
        for (auto& instance : _bodies) {
            if (instance.body.isValid()) {
                instance.body.retireDeferred(bhkWorld ? bhkWorld : _cachedBhkWorld);
            }
            clearInstance(instance, true);
        }

        _created = false;
        _cachedWorld = nullptr;
        _cachedBhkWorld = nullptr;
        _cachedSkeleton = nullptr;
        _cachedBoneTree = nullptr;
        _cachedPowerArmor = false;
        _cachedTuningSignature = 0;
        _dynamicForearmTwinTargets = {};
        _canonicalForearmTwinDimensions = {};
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _reader.resetCache();
        _snapshot = {};
    }

    void BodyBoneColliderSet::reset()
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        clearAtomicBodyIds();
        for (auto& instance : _bodies) {
            // reset is used when the Havok world is already gone. The body handle cannot be
            // destroyed through that world, but ROCK still owns the shape reference it created.
            clearInstance(instance, true);
        }

        _created = false;
        _cachedWorld = nullptr;
        _cachedBhkWorld = nullptr;
        _cachedSkeleton = nullptr;
        _cachedBoneTree = nullptr;
        _cachedPowerArmor = false;
        _cachedTuningSignature = 0;
        _dynamicForearmTwinTargets = {};
        _canonicalForearmTwinDimensions = {};
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _reader.resetCache();
        _snapshot = {};
    }

    void BodyBoneColliderSet::update(RE::hknpWorld* world, float deltaTime)
    {
        if (!world || !_created) {
            return;
        }

        if (_driveRebuildRequested.exchange(false, std::memory_order_acq_rel)) {
            ROCK_LOG_WARN(Body, "Body bone collider drive failure requested rebuild");
            create(world, _cachedBhkWorld);
            return;
        }

        auto& snapshot = _snapshot;
        if (!captureBoneSnapshot(snapshot)) {
            return;
        }

        const auto tuningSignature = refreshTuning(snapshot.inPowerArmor);
        if (_cachedWorld != world ||
            _cachedSkeleton != _lastCapturedSkeleton ||
            _cachedBoneTree != _lastCapturedBoneTree ||
            _cachedPowerArmor != _lastCapturedPowerArmor ||
            _cachedTuningSignature != tuningSignature) {
            if (++_updateLogCounter > 120) {
                _updateLogCounter = 0;
                ROCK_LOG_INFO(Body,
                    "Body bone collider source/tuning changed; rebuilding generated body set powerArmor={} tuning=0x{:016X}->0x{:016X}",
                    snapshot.inPowerArmor ? "yes" : "no",
                    _cachedTuningSignature,
                    tuningSignature);
            }
            create(world, _cachedBhkWorld);
            return;
        }

        const auto& descriptors = bodyDescriptorsForPowerArmor(snapshot.inPowerArmor);
        const auto bonesByName = _boneNameIndex.bind(snapshot);
        dynamic_hand_twin::ForearmTwinTargets forearmTwinTargets{};
        ForearmTwinMergeSources forearmTwinMergeSources{};
        for (auto& instance : _bodies) {
            if (!instance.body.isValid() || instance.descriptorIndex >= descriptors.size()) {
                continue;
            }

            const auto& descriptor = descriptors[instance.descriptorIndex];
            DescriptorFrameResult frame{};
            if (makeDescriptorFrame(bonesByName, descriptor, snapshot.inPowerArmor, _tuning.descriptors[instance.descriptorIndex], frame)) {
                collectForearmTwinMergeSource(forearmTwinMergeSources, descriptor, frame);
                queueBodyTarget(instance.body, frame.transform, deltaTime, instance.driveState);
            }
        }
        publishMergedForearmTwinTargets(
            forearmTwinTargets,
            forearmTwinMergeSources,
            bonesByName,
            snapshot.inPowerArmor);
        dynamic_hand_twin::applyCanonicalForearmDimensions(
            forearmTwinTargets,
            _canonicalForearmTwinDimensions);
        forearmTwinTargets.updateCounter = _dynamicForearmTwinTargets.updateCounter + 1;
        forearmTwinTargets.geometryGeneration = _dynamicForearmGeometryGeneration;
        _dynamicForearmTwinTargets = forearmTwinTargets;
    }

    void BodyBoneColliderSet::flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || !_created) {
            return;
        }

        for (std::size_t i = 0; i < _bodies.size(); ++i) {
            auto& instance = _bodies[i];
            if (!instance.body.isValid()) {
                continue;
            }

            handleGeneratedBodyDriveResult(
                driveGeneratedKeyframedBody(world,
                    instance.body,
                    instance.driveState,
                    timing,
                    "body-bone-collider",
                    static_cast<std::uint32_t>(i),
                    g_rockConfig.rockHandBoneColliderMaxLinearVelocity,
                    g_rockConfig.rockHandBoneColliderMaxAngularVelocity),
                "body-bone-collider",
                static_cast<std::uint32_t>(i));
        }
    }

    void BodyBoneColliderSet::queueBodyTarget(BethesdaPhysicsBody& body, const RE::NiTransform& target, float sourceDeltaSeconds, GeneratedKeyframedBodyDriveState& driveState)
    {
        if (!body.isValid()) {
            return;
        }

        queueGeneratedKeyframedBodyTarget(driveState, target, sourceDeltaSeconds, 1000.0f);
    }

    bool BodyBoneColliderSet::tryGetBodyTargetForDebug(
        const std::uint32_t bodyId,
        RE::NiTransform& outTarget) const
    {
        if (bodyId == kInvalidBodyBoneColliderBodyId) {
            return false;
        }

        for (const auto& instance : _bodies) {
            if (!instance.body.isValid() ||
                instance.body.getBodyId().value != bodyId) {
                continue;
            }

            std::unique_lock targetLock(
                instance.driveState.mutex,
                std::try_to_lock);
            if (!targetLock.owns_lock()) {
                return false;
            }
            if (instance.driveState.hasPendingTarget) {
                outTarget = instance.driveState.pendingTarget;
                return true;
            }
            if (instance.driveState.hasPreviousTarget) {
                outTarget = instance.driveState.previousTarget;
                return true;
            }
            return false;
        }
        return false;
    }

    void BodyBoneColliderSet::handleGeneratedBodyDriveResult(const GeneratedKeyframedBodyDriveResult& result, const char* ownerName, std::uint32_t bodyIndex)
    {
        if (!result.attempted || result.skippedStale) {
            return;
        }

        if (result.driven) {
            _driveFailureCount.store(0, std::memory_order_release);
            return;
        }

        if (!result.shouldRequestRebuild()) {
            return;
        }

        const auto failures = _driveFailureCount.fetch_add(1, std::memory_order_acq_rel) + 1;
        _driveRebuildRequested.store(true, std::memory_order_release);
        ROCK_LOG_SAMPLE_WARN(Body,
            g_rockConfig.rockLogSampleMilliseconds,
            "Body generated collider drive result requested rebuild owner={} bodyIndex={} failures={} missingBody={} ownerMismatch={} placementFailed={} nativeDriveFailed={} bodyDeltaGame={:.2f} bodyRotErr={:.2f}",
            ownerName ? ownerName : "unknown",
            bodyIndex,
            failures,
            result.missingBody ? "yes" : "no",
            result.bodyCollisionObjectMismatch ? "yes" : "no",
            result.placementFailed ? "yes" : "no",
            result.nativeDriveFailed ? "yes" : "no",
            result.hasLiveBodyTransform ? result.bodyDeltaGameUnits : -1.0f,
            result.hasLiveBodyTransform ? result.targetToBodyRotationDegrees : -1.0f);
    }

    void BodyBoneColliderSet::clearInstance(BodyInstance& instance, bool releaseShapeRef)
    {
        if (releaseShapeRef && instance.ownsShapeRef && instance.shape) {
            shapeRemoveRef(instance.shape);
        }

        instance.body.reset();
        instance.shape = nullptr;
        instance.role = BoneColliderRole::TorsoSegment;
        instance.zone = body_zone::BodyZoneKind::Unknown;
        instance.side = body_zone::BodyZoneSide::Center;
        instance.descriptorIndex = 0;
        instance.lengthGameUnits = 0.0f;
        instance.radiusGameUnits = 0.0f;
        instance.ownsShapeRef = false;
        clearGeneratedKeyframedBodyDriveState(instance.driveState);
    }

    void BodyBoneColliderSet::publishAtomicBodyIds(bool inPowerArmor)
    {
        clearAtomicBodyIds();
        std::uint32_t count = 0;
        for (const auto& instance : _bodies) {
            if (!instance.body.isValid() || count >= _bodyIdsAtomic.size()) {
                continue;
            }

            _rolesAtomic[count].store(static_cast<std::uint32_t>(instance.role), std::memory_order_release);
            _zonesAtomic[count].store(static_cast<std::uint32_t>(instance.zone), std::memory_order_release);
            _sidesAtomic[count].store(static_cast<std::uint32_t>(instance.side), std::memory_order_release);
            _descriptorIndicesAtomic[count].store(instance.descriptorIndex, std::memory_order_release);
            _powerArmorAtomic[count].store(inPowerArmor ? 1u : 0u, std::memory_order_release);
            _lengthsGameAtomic[count].store(instance.lengthGameUnits, std::memory_order_release);
            _radiiGameAtomic[count].store(instance.radiusGameUnits, std::memory_order_release);
            _bodyIdsAtomic[count].store(instance.body.getBodyId().value, std::memory_order_release);
            ++count;
        }
        _bodyCountAtomic.store(count, std::memory_order_release);
    }

    void BodyBoneColliderSet::clearAtomicBodyIds()
    {
        _bodyCountAtomic.store(0, std::memory_order_release);
        for (std::size_t i = 0; i < _bodyIdsAtomic.size(); ++i) {
            _bodyIdsAtomic[i].store(kInvalidBodyBoneColliderBodyId, std::memory_order_release);
            _rolesAtomic[i].store(static_cast<std::uint32_t>(BoneColliderRole::TorsoSegment), std::memory_order_release);
            _zonesAtomic[i].store(static_cast<std::uint32_t>(body_zone::BodyZoneKind::Unknown), std::memory_order_release);
            _sidesAtomic[i].store(static_cast<std::uint32_t>(body_zone::BodyZoneSide::Center), std::memory_order_release);
            _descriptorIndicesAtomic[i].store(0, std::memory_order_release);
            _powerArmorAtomic[i].store(0, std::memory_order_release);
            _lengthsGameAtomic[i].store(0.0f, std::memory_order_release);
            _radiiGameAtomic[i].store(0.0f, std::memory_order_release);
        }
    }

    std::uint32_t BodyBoneColliderSet::getBodyIdAtomic(std::size_t index) const
    {
        if (index >= _bodyIdsAtomic.size() || index >= _bodyCountAtomic.load(std::memory_order_acquire)) {
            return kInvalidBodyBoneColliderBodyId;
        }
        return _bodyIdsAtomic[index].load(std::memory_order_acquire);
    }

    std::uint32_t BodyBoneColliderSet::copyGrabSuppressionArmBodyIdsAtomic(bool isLeft, std::uint32_t* outBodyIds, std::size_t maxBodyIds) const
    {
        /*
         * A normal object grab owns the collision pocket around the driving hand.
         * The generated hand colliders cover palm/fingers; body colliders own the
         * adjacent root-flattened forearm and wrist/hand chain. Returning this
         * side-local arm chain lets held-object grabs suppress self-collision at
         * the wrist without changing the separate two-handed weapon suppression path.
         */
        if (!outBodyIds || maxBodyIds == 0) {
            return 0;
        }

        const auto wantedSide = isLeft ? body_zone::BodyZoneSide::Left : body_zone::BodyZoneSide::Right;
        std::uint32_t written = 0;
        const std::uint32_t count = _bodyCountAtomic.load(std::memory_order_acquire);
        for (std::uint32_t i = 0; i < count && i < _bodyIdsAtomic.size() && written < maxBodyIds; ++i) {
            const auto bodyId = _bodyIdsAtomic[i].load(std::memory_order_acquire);
            if (bodyId == kInvalidBodyBoneColliderBodyId) {
                continue;
            }

            const auto side = static_cast<body_zone::BodyZoneSide>(_sidesAtomic[i].load(std::memory_order_acquire));
            if (side != wantedSide) {
                continue;
            }

            const auto role = static_cast<BoneColliderRole>(_rolesAtomic[i].load(std::memory_order_acquire));
            if (role != BoneColliderRole::ForearmSegment && role != BoneColliderRole::HandSegment) {
                continue;
            }

            outBodyIds[written++] = bodyId;
        }
        return written;
    }

    bool BodyBoneColliderSet::isColliderBodyIdAtomic(std::uint32_t bodyId) const
    {
        BodyBoneColliderMetadata metadata{};
        return tryGetBodyMetadataAtomic(bodyId, metadata);
    }

    bool BodyBoneColliderSet::tryGetBodyRoleAtomic(std::uint32_t bodyId, BoneColliderRole& outRole) const
    {
        BodyBoneColliderMetadata metadata{};
        if (!tryGetBodyMetadataAtomic(bodyId, metadata)) {
            return false;
        }
        outRole = metadata.role;
        return true;
    }

    bool BodyBoneColliderSet::tryGetBodyMetadataAtomic(std::uint32_t bodyId, BodyBoneColliderMetadata& outMetadata) const
    {
        outMetadata = {};
        if (bodyId == kInvalidBodyBoneColliderBodyId) {
            return false;
        }

        const std::uint32_t count = _bodyCountAtomic.load(std::memory_order_acquire);
        for (std::uint32_t i = 0; i < count && i < _bodyIdsAtomic.size(); ++i) {
            if (_bodyIdsAtomic[i].load(std::memory_order_acquire) != bodyId) {
                continue;
            }

            return tryGetBodyMetadataAtIndexAtomic(i, bodyId, outMetadata);
        }
        return false;
    }

    bool BodyBoneColliderSet::tryGetBodyMetadataAtIndexAtomic(std::uint32_t i,
        std::uint32_t bodyId, BodyBoneColliderMetadata& outMetadata) const
    {
        outMetadata = {};
        if (i >= _bodyIdsAtomic.size() || i >= _bodyCountAtomic.load(std::memory_order_acquire) ||
            bodyId == kInvalidBodyBoneColliderBodyId || _bodyIdsAtomic[i].load(std::memory_order_acquire) != bodyId) {
            return false;
        }
        outMetadata.valid = true;
        outMetadata.inPowerArmor = _powerArmorAtomic[i].load(std::memory_order_acquire) != 0;
        outMetadata.bodyId = bodyId;
        outMetadata.role = static_cast<BoneColliderRole>(_rolesAtomic[i].load(std::memory_order_acquire));
        outMetadata.zone = static_cast<body_zone::BodyZoneKind>(_zonesAtomic[i].load(std::memory_order_acquire));
        outMetadata.side = static_cast<body_zone::BodyZoneSide>(_sidesAtomic[i].load(std::memory_order_acquire));
        outMetadata.descriptorIndex = _descriptorIndicesAtomic[i].load(std::memory_order_acquire);
        outMetadata.lengthGameUnits = _lengthsGameAtomic[i].load(std::memory_order_acquire);
        outMetadata.radiusGameUnits = _radiiGameAtomic[i].load(std::memory_order_acquire);
        return true;
    }
}
