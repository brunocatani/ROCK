#include "physics-interaction/hand/HandBoneColliderSet.h"

#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/debug/DebugAxisMath.h"
#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"
#include "physics-interaction/native/HavokConvexShapeBuilder.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "RockConfig.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <string_view>
#include <unordered_map>

namespace rock
{
    namespace
    {
        using hand_collider_semantics::HandColliderRole;
        using hand_collider_semantics::HandFinger;
        using hand_collider_semantics::HandFingerSegment;

        constexpr std::uint32_t kHandFilterInfo = (0x000B << 16) | (ROCK_HAND_LAYER & 0x7F);

        RE::NiTransform makeIdentityTransform()
        {
            return transform_math::makeIdentityTransform<RE::NiTransform>();
        }

        RE::NiPoint3 normalizeOr(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
        {
            const float lenSq = value.x * value.x + value.y * value.y + value.z * value.z;
            if (lenSq <= 1.0e-8f) {
                return fallback;
            }
            const float invLen = 1.0f / std::sqrt(lenSq);
            return RE::NiPoint3(value.x * invLen, value.y * invLen, value.z * invLen);
        }

        std::string fingerBoneName(bool isLeft, HandFinger finger, HandFingerSegment segment)
        {
            const int fingerIndex = static_cast<int>(finger) + 1;
            const int segmentIndex = static_cast<int>(segment) + 1;
            return std::string(isLeft ? "LArm_Finger" : "RArm_Finger") + std::to_string(fingerIndex) + std::to_string(segmentIndex);
        }

        using SnapshotBoneMap = std::unordered_map<std::string_view, const DirectSkeletonBoneEntry*>;

        SnapshotBoneMap makeSnapshotBoneMap(const DirectSkeletonBoneSnapshot& snapshot)
        {
            SnapshotBoneMap map;
            map.reserve(snapshot.bones.size());
            for (const auto& bone : snapshot.bones) {
                map.emplace(std::string_view{ bone.name }, &bone);
            }
            return map;
        }

        bool findSnapshotBone(const SnapshotBoneMap& bonesByName, const std::string& name, RE::NiTransform& outTransform)
        {
            const auto it = bonesByName.find(name);
            if (it == bonesByName.end() || !it->second) {
                return false;
            }
            outTransform = it->second->world;
            return true;
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

        float roleRadius(HandColliderRole role, bool powerArmor)
        {
            const float scale = powerArmor ? 1.55f : 1.0f;
            if (hand_collider_semantics::isPalmRole(role)) {
                return 1.35f * scale;
            }
            switch (hand_collider_semantics::segmentForRole(role)) {
            case HandFingerSegment::Base:
                return 0.55f * scale;
            case HandFingerSegment::Middle:
                return 0.46f * scale;
            case HandFingerSegment::Tip:
                return 0.38f * scale;
            default:
                return 0.5f * scale;
            }
        }

        float roleConvexRadius(HandColliderRole role, bool powerArmor)
        {
            (void)role;
            return powerArmor ? 0.15f : 0.10f;
        }
    }

    HandBoneColliderSet::HandBoneColliderSet()
    {
        clearAtomicBodyIds();
    }

    bool HandBoneColliderSet::captureBoneLookup(bool isLeft, BoneFrameLookup& outLookup)
    {
        outLookup = {};
        DirectSkeletonBoneSnapshot snapshot{};
        if (!_reader.capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                snapshot)) {
            return false;
        }

        _lastCapturedSkeleton = snapshot.skeleton;
        _lastCapturedBoneTree = snapshot.boneTree;
        _lastCapturedPowerArmor = snapshot.inPowerArmor;
        const auto bonesByName = makeSnapshotBoneMap(snapshot);

        if (!findSnapshotBone(bonesByName, isLeft ? "LArm_Hand" : "RArm_Hand", outLookup.hand)) {
            ROCK_LOG_WARN(Hand, "{} hand bone colliders disabled: missing hand bone", isLeft ? "Left" : "Right");
            return false;
        }

        outLookup.hasForearm3 = findSnapshotBone(bonesByName, isLeft ? "LArm_ForeArm3" : "RArm_ForeArm3", outLookup.forearm3);
        outLookup.backDirection = normalizeOr(
            debug_axis_math::rotateNiLocalToWorld(outLookup.hand.rotate, RE::NiPoint3(0.0f, 0.0f, 1.0f)),
            RE::NiPoint3(0.0f, 0.0f, 1.0f));

        bool allFingerBones = true;
        for (std::size_t fingerIndex = 0; fingerIndex < hand_collider_semantics::kHandFingerCount; ++fingerIndex) {
            bool fingerValid = true;
            for (std::size_t segmentIndex = 0; segmentIndex < hand_collider_semantics::kHandFingerSegmentCount; ++segmentIndex) {
                const auto name = fingerBoneName(
                    isLeft,
                    static_cast<HandFinger>(fingerIndex),
                    static_cast<HandFingerSegment>(segmentIndex));
                if (!findSnapshotBone(bonesByName, name, outLookup.fingers[fingerIndex][segmentIndex])) {
                    fingerValid = false;
                    allFingerBones = false;
                    ROCK_LOG_WARN(Hand, "{} hand bone collider missing {}", isLeft ? "Left" : "Right", name);
                    break;
                }
            }
            outLookup.fingerValid[fingerIndex] = fingerValid;
            outLookup.fingerBases[fingerIndex] = fingerValid ? outLookup.fingers[fingerIndex][0].translate : outLookup.hand.translate;
        }

        if (g_rockConfig.rockHandBoneCollidersRequireAllFingerBones && !allFingerBones) {
            return false;
        }

        outLookup.valid = true;
        return true;
    }

    bool HandBoneColliderSet::makeRoleFrame(const BoneFrameLookup& lookup, bool isLeft, HandColliderRole role, RoleFrameResult& outFrame) const
    {
        (void)isLeft;
        outFrame = {};
        if (!lookup.valid) {
            return false;
        }

        if (role == HandColliderRole::PalmAnchor || role == HandColliderRole::PalmFace || role == HandColliderRole::PalmBack) {
            const auto palm = hand_bone_collider_geometry_math::buildPalmAnchorFrame(lookup.hand, lookup.fingerBases, lookup.backDirection, 0.75f);
            if (!palm.valid) {
                return false;
            }
            outFrame.valid = true;
            outFrame.transform = palm.transform;
            outFrame.length = (std::max)(3.0f, palm.length * 1.15f);
            outFrame.radius = roleRadius(role, _lastCapturedPowerArmor);
            outFrame.convexRadius = roleConvexRadius(role, _lastCapturedPowerArmor);
            if (role == HandColliderRole::PalmFace) {
                outFrame.transform.translate = outFrame.transform.translate - lookup.backDirection * 0.55f;
                outFrame.radius *= 0.85f;
            } else if (role == HandColliderRole::PalmBack) {
                outFrame.transform.translate = outFrame.transform.translate + lookup.backDirection * 0.75f;
                outFrame.radius *= 0.95f;
            }
            return true;
        }

        hand_bone_collider_geometry_math::BoneColliderFrameInput<RE::NiTransform, RE::NiPoint3> input{};
        input.radius = roleRadius(role, _lastCapturedPowerArmor);
        input.convexRadius = roleConvexRadius(role, _lastCapturedPowerArmor);

        if (role == HandColliderRole::PalmHeel) {
            if (!lookup.hasForearm3) {
                return false;
            }
            input.start = lookup.forearm3;
            input.end = lookup.hand;
        } else if (role == HandColliderRole::ThumbPad) {
            if (!lookup.fingerValid[0]) {
                return false;
            }
            input.start = lookup.hand;
            input.end = lookup.fingers[0][0];
            input.radius *= 1.2f;
        } else if (hand_collider_semantics::isFingerRole(role)) {
            const auto finger = hand_collider_semantics::fingerForRole(role);
            const auto segment = hand_collider_semantics::segmentForRole(role);
            const std::size_t fingerIndex = static_cast<std::size_t>(finger);
            if (fingerIndex >= lookup.fingerValid.size() || !lookup.fingerValid[fingerIndex]) {
                return false;
            }

            if (segment == HandFingerSegment::Base) {
                input.start = lookup.fingers[fingerIndex][0];
                input.end = lookup.fingers[fingerIndex][1];
            } else if (segment == HandFingerSegment::Middle) {
                input.start = lookup.fingers[fingerIndex][1];
                input.end = lookup.fingers[fingerIndex][2];
            } else {
                input.previous = lookup.fingers[fingerIndex][1];
                input.start = lookup.fingers[fingerIndex][2];
                input.end = makeIdentityTransform();
                input.extrapolateFromPrevious = true;
                input.extrapolatedLengthScale = 0.65f;
            }
        } else {
            return false;
        }

        const auto frame = hand_bone_collider_geometry_math::buildSegmentColliderFrame(input);
        if (!frame.valid) {
            return false;
        }
        outFrame.valid = true;
        outFrame.transform = frame.transform;
        outFrame.length = frame.length;
        outFrame.radius = frame.radius;
        outFrame.convexRadius = frame.convexRadius;
        return true;
    }

    RE::hknpShape* HandBoneColliderSet::buildShapeForRole(const RoleFrameResult& frame, HandColliderRole role) const
    {
        if (!frame.valid) {
            return nullptr;
        }

        float length = frame.length;
        float radius = frame.radius;
        if (hand_collider_semantics::isPalmRole(role)) {
            length = (std::max)(2.5f, frame.length);
            radius = (std::max)(0.8f, frame.radius);
            const float width = radius * 2.25f;
            const float thickness = role == HandColliderRole::PalmFace ? radius * 0.55f :
                                    role == HandColliderRole::PalmBack ? radius * 0.70f :
                                    role == HandColliderRole::ThumbPad ? radius * 0.80f :
                                    radius * 0.95f;
            const auto gamePoints = hand_bone_collider_geometry_math::makeRoundedBoxHullPoints<RE::NiPoint3>(length, width, thickness);
            return havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(toHavokPointCloud(gamePoints), frame.convexRadius * gameToHavokScale());
        }

        const auto gamePoints = hand_bone_collider_geometry_math::makeCapsuleLikeHullPoints<RE::NiPoint3>(length, radius);
        return havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(toHavokPointCloud(gamePoints), frame.convexRadius * gameToHavokScale());
    }

    bool HandBoneColliderSet::createBodyForRole(RE::hknpWorld* world, void* bhkWorld, bool isLeft, HandColliderRole role, const RoleFrameResult& frame, BodyInstance& instance)
    {
        auto* shape = buildShapeForRole(frame, role);
        if (!shape) {
            ROCK_LOG_WARN(Hand, "{} {} collider shape build failed", isLeft ? "Left" : "Right", hand_collider_semantics::roleName(role));
            return false;
        }

        instance.shape = shape;
        instance.role = role;
        instance.ownsShapeRef = true;
        clearGeneratedKeyframedBodyDriveState(instance.driveState);

        const std::string name = std::string(isLeft ? "ROCK_Left" : "ROCK_Right") + hand_collider_semantics::roleName(role);
        if (!instance.body.create(world, bhkWorld, shape, kHandFilterInfo, registerHandMaterial(world), BethesdaMotionType::Keyframed, name.c_str())) {
            shapeRemoveRef(shape);
            clearInstance(instance, false);
            ROCK_LOG_ERROR(Hand, "{} {} collider body create failed", isLeft ? "Left" : "Right", hand_collider_semantics::roleName(role));
            return false;
        }

        instance.body.createNiNode(name.c_str());
        if (!placeGeneratedKeyframedBodyImmediately(instance.body, frame.transform)) {
            ROCK_LOG_ERROR(Hand,
                "{} {} collider initial placement failed; destroying generated body bodyId={}",
                isLeft ? "Left" : "Right",
                hand_collider_semantics::roleName(role),
                instance.body.getBodyId().value);
            instance.body.destroy(bhkWorld);
            shapeRemoveRef(shape);
            clearInstance(instance, false);
            return false;
        }
        instance.driveState.previousTargetPosition = frame.transform.translate;
        instance.driveState.hasPreviousTarget = true;
        ROCK_LOG_DEBUG(Hand,
            "{} {} collider created bodyId={} length={:.2f} radius={:.2f}",
            isLeft ? "Left" : "Right",
            hand_collider_semantics::roleName(role),
            instance.body.getBodyId().value,
            frame.length,
            frame.radius);
        return true;
    }

    bool HandBoneColliderSet::create(RE::hknpWorld* world, void* bhkWorld, bool isLeft, BethesdaPhysicsBody& palmAnchorBody)
    {
        destroy(bhkWorld, palmAnchorBody);
        if (!world || !bhkWorld) {
            return false;
        }

        BoneFrameLookup lookup{};
        if (!captureBoneLookup(isLeft, lookup)) {
            return false;
        }

        RoleFrameResult anchorFrame{};
        if (!makeRoleFrame(lookup, isLeft, HandColliderRole::PalmAnchor, anchorFrame)) {
            ROCK_LOG_ERROR(Hand, "{} palm anchor frame could not be derived; bone-derived hand creation cannot continue", isLeft ? "Left" : "Right");
            return false;
        }

        auto* anchorShape = buildShapeForRole(anchorFrame, HandColliderRole::PalmAnchor);
        if (!anchorShape) {
            ROCK_LOG_ERROR(Hand, "{} palm anchor shape build failed; bone-derived hand creation cannot continue", isLeft ? "Left" : "Right");
            return false;
        }

        const std::string anchorName = std::string(isLeft ? "ROCK_Left" : "ROCK_Right") + "PalmAnchor";
        if (!palmAnchorBody.create(world, bhkWorld, anchorShape, kHandFilterInfo, registerHandMaterial(world), BethesdaMotionType::Keyframed, anchorName.c_str())) {
            shapeRemoveRef(anchorShape);
            ROCK_LOG_ERROR(Hand, "{} palm anchor body create failed; bone-derived hand creation cannot continue", isLeft ? "Left" : "Right");
            return false;
        }
        shapeRemoveRef(anchorShape);
        palmAnchorBody.createNiNode(anchorName.c_str());
        if (!placeGeneratedKeyframedBodyImmediately(palmAnchorBody, anchorFrame.transform)) {
            ROCK_LOG_ERROR(Hand,
                "{} palm anchor initial placement failed; destroying generated anchor bodyId={}",
                isLeft ? "Left" : "Right",
                palmAnchorBody.getBodyId().value);
            palmAnchorBody.destroy(bhkWorld);
            return false;
        }
        _palmAnchorDriveState.previousTargetPosition = anchorFrame.transform.translate;
        _palmAnchorDriveState.hasPreviousTarget = true;

        std::size_t createdCount = 0;
        for (const auto role : hand_collider_semantics::kHandNonAnchorColliderRoles) {
            RoleFrameResult frame{};
            if (!makeRoleFrame(lookup, isLeft, role, frame)) {
                if (g_rockConfig.rockHandBoneCollidersRequireAllFingerBones) {
                    ROCK_LOG_ERROR(Hand, "{} {} collider frame missing; destroying bone-derived hand", isLeft ? "Left" : "Right", hand_collider_semantics::roleName(role));
                    destroy(bhkWorld, palmAnchorBody);
                    return false;
                }
                ROCK_LOG_WARN(Hand, "{} {} collider frame missing; continuing with partial bone-derived hand", isLeft ? "Left" : "Right", hand_collider_semantics::roleName(role));
                continue;
            }

            if (createdCount >= _bodies.size()) {
                break;
            }
            if (!createBodyForRole(world, bhkWorld, isLeft, role, frame, _bodies[createdCount])) {
                ROCK_LOG_ERROR(Hand, "{} {} collider body allocation failed; destroying bone-derived hand", isLeft ? "Left" : "Right", hand_collider_semantics::roleName(role));
                destroy(bhkWorld, palmAnchorBody);
                return false;
            }
            ++createdCount;
        }

        _cachedWorld = world;
        _cachedBhkWorld = bhkWorld;
        _cachedSkeleton = _lastCapturedSkeleton;
        _cachedBoneTree = _lastCapturedBoneTree;
        _cachedPowerArmor = _lastCapturedPowerArmor;
        _isLeftAtomic.store(isLeft ? 1u : 0u, std::memory_order_release);
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _created = true;
        publishAtomicBodyIds(palmAnchorBody, isLeft);
        ROCK_LOG_INFO(Hand,
            "{} bone-derived hand colliders created: anchor={} segments={} sourceSkeleton={} tree={} powerArmor={}",
            isLeft ? "Left" : "Right",
            palmAnchorBody.getBodyId().value,
            createdCount,
            reinterpret_cast<std::uintptr_t>(_cachedSkeleton),
            reinterpret_cast<std::uintptr_t>(_cachedBoneTree),
            _cachedPowerArmor ? "yes" : "no");
        return true;
    }

    void HandBoneColliderSet::destroy(void* bhkWorld, BethesdaPhysicsBody& palmAnchorBody)
    {
        clearAtomicBodyIds();
        for (auto& instance : _bodies) {
            if (instance.body.isValid()) {
                instance.body.destroy(bhkWorld ? bhkWorld : _cachedBhkWorld);
            }
            clearInstance(instance, true);
        }

        if (palmAnchorBody.isValid()) {
            palmAnchorBody.destroy(bhkWorld ? bhkWorld : _cachedBhkWorld);
        }

        _created = false;
        _cachedWorld = nullptr;
        _cachedBhkWorld = nullptr;
        clearGeneratedKeyframedBodyDriveState(_palmAnchorDriveState);
        _cachedSkeleton = nullptr;
        _cachedBoneTree = nullptr;
        _cachedPowerArmor = false;
        _isLeftAtomic.store(0, std::memory_order_release);
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _reader.resetCache();
    }

    void HandBoneColliderSet::reset()
    {
        clearAtomicBodyIds();
        for (auto& instance : _bodies) {
            clearInstance(instance, false);
        }
        _created = false;
        _cachedWorld = nullptr;
        _cachedBhkWorld = nullptr;
        clearGeneratedKeyframedBodyDriveState(_palmAnchorDriveState);
        _cachedSkeleton = nullptr;
        _cachedBoneTree = nullptr;
        _cachedPowerArmor = false;
        _isLeftAtomic.store(0, std::memory_order_release);
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _reader.resetCache();
    }

    void HandBoneColliderSet::update(RE::hknpWorld* world, bool isLeft, BethesdaPhysicsBody& palmAnchorBody, float deltaTime)
    {
        (void)deltaTime;
        if (!world || !_created || !palmAnchorBody.isValid()) {
            return;
        }

        if (_driveRebuildRequested.exchange(false, std::memory_order_acq_rel)) {
            if (palmAnchorBody.isConstrained()) {
                _driveRebuildRequested.store(true, std::memory_order_release);
                ROCK_LOG_SAMPLE_WARN(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} bone-derived hand collider rebuild deferred after drive failure while palm anchor is constrained",
                    isLeft ? "Left" : "Right");
                return;
            }

            ROCK_LOG_WARN(Hand, "{} bone-derived hand collider drive failure requested rebuild", isLeft ? "Left" : "Right");
            create(world, _cachedBhkWorld, isLeft, palmAnchorBody);
            return;
        }

        BoneFrameLookup lookup{};
        if (!captureBoneLookup(isLeft, lookup)) {
            return;
        }

        if (_cachedWorld != world || _cachedSkeleton != _lastCapturedSkeleton || _cachedBoneTree != _lastCapturedBoneTree || _cachedPowerArmor != _lastCapturedPowerArmor) {
            if (palmAnchorBody.isConstrained()) {
                if (++_updateLogCounter > 120) {
                    _updateLogCounter = 0;
                    ROCK_LOG_WARN(Hand, "{} bone-derived hand collider shape rebuild deferred while palm anchor is constrained; live transforms still update",
                        isLeft ? "Left" : "Right");
                }
            } else {
                ROCK_LOG_INFO(Hand, "{} bone-derived hand collider source changed; rebuilding", isLeft ? "Left" : "Right");
                create(world, _cachedBhkWorld, isLeft, palmAnchorBody);
                return;
            }
        }

        RoleFrameResult anchorFrame{};
        if (makeRoleFrame(lookup, isLeft, HandColliderRole::PalmAnchor, anchorFrame)) {
            queueBodyTarget(palmAnchorBody, anchorFrame.transform, _palmAnchorDriveState);
        }

        for (auto& instance : _bodies) {
            if (!instance.body.isValid()) {
                continue;
            }
            RoleFrameResult frame{};
            if (makeRoleFrame(lookup, isLeft, instance.role, frame)) {
                queueBodyTarget(instance.body, frame.transform, instance.driveState);
            }
        }
    }

    void HandBoneColliderSet::flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing, BethesdaPhysicsBody& palmAnchorBody)
    {
        if (!world || !_created || !palmAnchorBody.isValid()) {
            return;
        }

        handleGeneratedBodyDriveResult(
            driveGeneratedKeyframedBody(world, palmAnchorBody, _palmAnchorDriveState, timing, "hand-palm-anchor", 0),
            "hand-palm-anchor",
            0);

        for (std::size_t i = 0; i < _bodies.size(); ++i) {
            auto& instance = _bodies[i];
            if (!instance.body.isValid()) {
                continue;
            }
            const auto bodyIndex = static_cast<std::uint32_t>(i + 1);
            handleGeneratedBodyDriveResult(
                driveGeneratedKeyframedBody(world, instance.body, instance.driveState, timing, "hand-bone-collider", bodyIndex),
                "hand-bone-collider",
                bodyIndex);
        }
    }

    void HandBoneColliderSet::queueBodyTarget(BethesdaPhysicsBody& body, const RE::NiTransform& target, GeneratedKeyframedBodyDriveState& driveState)
    {
        if (!body.isValid()) {
            return;
        }

        queueGeneratedKeyframedBodyTarget(driveState, target, 1000.0f);
    }

    void HandBoneColliderSet::handleGeneratedBodyDriveResult(const GeneratedKeyframedBodyDriveResult& result, const char* ownerName, std::uint32_t bodyIndex)
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
        ROCK_LOG_SAMPLE_WARN(Hand,
            g_rockConfig.rockLogSampleMilliseconds,
            "Hand generated collider drive result requested rebuild owner={} bodyIndex={} failures={} missingBody={} placementFailed={} nativeDriveFailed={} bodyDeltaGame={:.2f}",
            ownerName ? ownerName : "unknown",
            bodyIndex,
            failures,
            result.missingBody ? "yes" : "no",
            result.placementFailed ? "yes" : "no",
            result.nativeDriveFailed ? "yes" : "no",
            result.hasLiveBodyTransform ? result.bodyDeltaGameUnits : -1.0f);
    }

    void HandBoneColliderSet::clearInstance(BodyInstance& instance, bool releaseShapeRef)
    {
        if (releaseShapeRef && instance.ownsShapeRef && instance.shape) {
            shapeRemoveRef(instance.shape);
        }
        instance.body.reset();
        instance.shape = nullptr;
        instance.role = HandColliderRole::PalmFace;
        instance.ownsShapeRef = false;
        clearGeneratedKeyframedBodyDriveState(instance.driveState);
    }

    void HandBoneColliderSet::clearAtomicBodyIds()
    {
        _bodyCountAtomic.store(0, std::memory_order_release);
        for (std::size_t i = 0; i < _bodyIdsAtomic.size(); ++i) {
            _bodyIdsAtomic[i].store(hand_collider_semantics::kInvalidBodyId, std::memory_order_release);
            _rolesAtomic[i].store(static_cast<std::uint32_t>(HandColliderRole::PalmAnchor), std::memory_order_release);
            _fingersAtomic[i].store(static_cast<std::uint32_t>(HandFinger::None), std::memory_order_release);
            _segmentsAtomic[i].store(static_cast<std::uint32_t>(HandFingerSegment::None), std::memory_order_release);
            _primaryAnchorAtomic[i].store(0, std::memory_order_release);
        }
    }

    void HandBoneColliderSet::publishAtomicBodyIds(const BethesdaPhysicsBody& palmAnchorBody, bool isLeft)
    {
        clearAtomicBodyIds();
        std::uint32_t count = 0;
        if (palmAnchorBody.isValid()) {
            _rolesAtomic[count].store(static_cast<std::uint32_t>(HandColliderRole::PalmAnchor), std::memory_order_release);
            _fingersAtomic[count].store(static_cast<std::uint32_t>(HandFinger::None), std::memory_order_release);
            _segmentsAtomic[count].store(static_cast<std::uint32_t>(HandFingerSegment::None), std::memory_order_release);
            _primaryAnchorAtomic[count].store(1, std::memory_order_release);
            _bodyIdsAtomic[count].store(palmAnchorBody.getBodyId().value, std::memory_order_release);
            ++count;
        }

        for (const auto& instance : _bodies) {
            if (!instance.body.isValid() || count >= _bodyIdsAtomic.size()) {
                continue;
            }
            const auto role = instance.role;
            _rolesAtomic[count].store(static_cast<std::uint32_t>(role), std::memory_order_release);
            _fingersAtomic[count].store(static_cast<std::uint32_t>(hand_collider_semantics::fingerForRole(role)), std::memory_order_release);
            _segmentsAtomic[count].store(static_cast<std::uint32_t>(hand_collider_semantics::segmentForRole(role)), std::memory_order_release);
            _primaryAnchorAtomic[count].store(0, std::memory_order_release);
            _bodyIdsAtomic[count].store(instance.body.getBodyId().value, std::memory_order_release);
            ++count;
        }
        (void)isLeft;
        _bodyCountAtomic.store(count, std::memory_order_release);
    }

    std::uint32_t HandBoneColliderSet::getBodyIdAtomic(std::size_t index) const
    {
        if (index >= _bodyIdsAtomic.size() || index >= _bodyCountAtomic.load(std::memory_order_acquire)) {
            return hand_collider_semantics::kInvalidBodyId;
        }
        return _bodyIdsAtomic[index].load(std::memory_order_acquire);
    }

    bool HandBoneColliderSet::isColliderBodyIdAtomic(std::uint32_t bodyId) const
    {
        HandColliderBodyMetadata metadata{};
        return tryGetBodyMetadataAtomic(bodyId, metadata);
    }

    bool HandBoneColliderSet::tryGetBodyRoleAtomic(std::uint32_t bodyId, HandColliderRole& outRole) const
    {
        HandColliderBodyMetadata metadata{};
        if (!tryGetBodyMetadataAtomic(bodyId, metadata)) {
            return false;
        }
        outRole = metadata.role;
        return true;
    }

    bool HandBoneColliderSet::tryGetBodyMetadataAtomic(std::uint32_t bodyId, HandColliderBodyMetadata& outMetadata) const
    {
        outMetadata = {};
        if (bodyId == hand_collider_semantics::kInvalidBodyId) {
            return false;
        }

        const std::uint32_t count = _bodyCountAtomic.load(std::memory_order_acquire);
        for (std::uint32_t i = 0; i < count && i < _bodyIdsAtomic.size(); ++i) {
            if (_bodyIdsAtomic[i].load(std::memory_order_acquire) != bodyId) {
                continue;
            }
            outMetadata.valid = true;
            outMetadata.isLeft = _isLeftAtomic.load(std::memory_order_acquire) != 0;
            outMetadata.bodyId = bodyId;
            outMetadata.role = static_cast<HandColliderRole>(_rolesAtomic[i].load(std::memory_order_acquire));
            outMetadata.finger = static_cast<HandFinger>(_fingersAtomic[i].load(std::memory_order_acquire));
            outMetadata.segment = static_cast<HandFingerSegment>(_segmentsAtomic[i].load(std::memory_order_acquire));
            outMetadata.primaryPalmAnchor = _primaryAnchorAtomic[i].load(std::memory_order_acquire) != 0;
            return true;
        }
        return false;
    }
}
