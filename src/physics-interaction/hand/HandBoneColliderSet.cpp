#include "physics-interaction/hand/HandBoneColliderSet.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/debug/DebugMath.h"
#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"
#include "physics-interaction/native/HavokConvexShapeBuilder.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "RockConfig.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>

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

        std::string_view fingerBoneName(bool isLeft, HandFinger finger, HandFingerSegment segment)
        {
            const auto index = (isLeft ? 0u : hand_collider_semantics::kHandFingerRoleCount) +
                static_cast<std::size_t>(finger) * hand_collider_semantics::kHandFingerSegmentCount +
                static_cast<std::size_t>(segment);
            return skeleton_bone_debug_math::kRequiredFingerBoneNames[index];
        }

        using SnapshotBoneLookup = SkeletonBoneNameIndex::View;

        bool findSnapshotBone(const SnapshotBoneLookup& bonesByName, std::string_view name, RE::NiTransform& outTransform)
        {
            const auto* bone = bonesByName.find(name);
            if (!bone) return false;
            outTransform = bone->world;
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

        hand_bone_collider_geometry_math::ColliderDimensionLimits handRoleDimensionLimits(HandColliderRole role, bool powerArmor)
        {
            hand_bone_collider_geometry_math::ColliderDimensionLimits limits{};
            if (hand_collider_semantics::isPalmRole(role)) {
                limits.maxLength = powerArmor ? 42.0f : 30.0f;
                limits.maxRadius = powerArmor ? 14.0f : 9.0f;
                limits.maxConvexRadius = powerArmor ? 1.0f : 0.75f;
                limits.maxLongAxisExtent = powerArmor ? 70.0f : 50.0f;
                return limits;
            }

            limits.maxLength = powerArmor ? 36.0f : 24.0f;
            limits.maxRadius = powerArmor ? 8.0f : 5.0f;
            limits.maxConvexRadius = powerArmor ? 0.75f : 0.5f;
            limits.maxLongAxisExtent = powerArmor ? 55.0f : 38.0f;
            return limits;
        }

        template <class Frame>
        bool handRoleFrameDimensionsValid(const Frame& frame, HandColliderRole role, bool powerArmor)
        {
            return frame.valid &&
                   hand_bone_collider_geometry_math::colliderDimensionsWithinLimits(
                       frame.length,
                       frame.radius,
                       frame.convexRadius,
                       handRoleDimensionLimits(role, powerArmor));
        }
    }

    std::uint64_t HandBoneColliderSet::refreshTuning(bool powerArmor)
    {
        const auto revision = g_rockConfig.configRevision();
        if (!_tuningReady || _tuningConfigRevision != revision || _tuningPowerArmor != powerArmor) {
            _tuning = collider_tuning::prepareHand(g_rockConfig, powerArmor);
            _tuningConfigRevision = revision;
            _tuningPowerArmor = powerArmor;
            _tuningReady = true;
        }
        // Keep the original effective signature: unrelated config edits must
        // not rebuild native colliders, and constrained hands still defer.
        return _tuning.signature;
    }

    HandBoneColliderSet::HandBoneColliderSet()
    {
        clearAtomicBodyIds();
    }

    bool HandBoneColliderSet::captureBoneLookup(
        bool isLeft,
        const RE::NiTransform& rollAuthorityWorld,
        BoneFrameLookup& outLookup)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::HandBoneCapture);
        outLookup = {};
        DirectSkeletonBoneSnapshot snapshot{};
        if (!_reader.capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                SkeletonBoneCaptureSpace::Controller,
                snapshot)) {
            return false;
        }

        return makeBoneLookup(snapshot, isLeft, rollAuthorityWorld, outLookup);
    }

    bool HandBoneColliderSet::makeBoneLookup(const DirectSkeletonBoneSnapshot& snapshot,
        bool isLeft, const RE::NiTransform& rollAuthorityWorld, BoneFrameLookup& outLookup)
    {
        outLookup = {};
        if (!snapshot.valid || snapshot.space != SkeletonBoneCaptureSpace::Controller ||
            snapshot.source != skeleton_bone_debug_math::SkeletonBoneSnapshotSource::GameRootFlattenedBoneTree) {
            return false;
        }
        _lastCapturedSkeleton = snapshot.skeleton;
        _lastCapturedBoneTree = snapshot.boneTree;
        _lastCapturedPowerArmor = snapshot.inPowerArmor;
        const auto bonesByName = _boneNameIndex.bind(snapshot);

        if (!findSnapshotBone(bonesByName, isLeft ? "LArm_Hand" : "RArm_Hand", outLookup.hand)) {
            ROCK_LOG_WARN(Hand, "{} hand bone colliders disabled: missing hand bone", isLeft ? "Left" : "Right");
            return false;
        }

        outLookup.rollAuthorityWorld = rollAuthorityWorld;

        outLookup.hasForearm3 = findSnapshotBone(bonesByName, isLeft ? "LArm_ForeArm3" : "RArm_ForeArm3", outLookup.forearm3);
        outLookup.crossPalmDirection = normalizeOr(
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

        if (!allFingerBones) {
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
            const auto palm = hand_bone_collider_geometry_math::buildPalmAnchorFrame(lookup.hand, lookup.fingerBases, lookup.crossPalmDirection, 0.75f);
            if (!palm.valid) {
                return false;
            }

            RE::NiPoint3 palmCenter = lookup.hand.translate;
            for (const auto& fingerBase : lookup.fingerBases) {
                palmCenter = hand_bone_collider_geometry_math::add(palmCenter, fingerBase);
            }
            palmCenter = hand_bone_collider_geometry_math::mul(palmCenter, 1.0f / static_cast<float>(lookup.fingerBases.size() + 1));

            const RE::NiPoint3 rawPalmDepthAxis = normalizeOr(
                debug_axis_math::rotateNiLocalToWorld(lookup.rollAuthorityWorld.rotate, RE::NiPoint3(0.0f, 1.0f, 0.0f)),
                RE::NiPoint3(0.0f, 1.0f, 0.0f));
            const float depthOffset = hand_bone_collider_geometry_math::dot(
                hand_bone_collider_geometry_math::sub(palmCenter, lookup.hand.translate),
                rawPalmDepthAxis);
            const RE::NiPoint3 palmPlaneCenter = hand_bone_collider_geometry_math::sub(
                palmCenter,
                hand_bone_collider_geometry_math::mul(rawPalmDepthAxis, depthOffset));

            outFrame.valid = true;
            outFrame.transform = lookup.hand;
            outFrame.transform.translate = hand_bone_collider_geometry_math::add(
                palmPlaneCenter,
                hand_bone_collider_geometry_math::mul(rawPalmDepthAxis, -0.25f));
            /*
             * Palm roles take roll from the normal Ni frame, but generated hand
             * collider bodies still consume the legacy column-authored rotation
             * convention before the shared Ni-to-Havok conversion. Convert only
             * this palm target; segment colliders already build their frames
             * through matrixFromAxes.
             */
            outFrame.transform.rotate =
                hand_bone_collider_geometry_math::transposeStoredRotation(lookup.rollAuthorityWorld.rotate);
            outFrame.transform.scale = 1.0f;
            outFrame.length = (std::max)(3.0f, palm.length * 1.15f);
            outFrame.radius = roleRadius(role, _lastCapturedPowerArmor);
            outFrame.convexRadius = roleConvexRadius(role, _lastCapturedPowerArmor);
            if (role == HandColliderRole::PalmFace) {
                outFrame.transform.translate = outFrame.transform.translate - rawPalmDepthAxis * 0.55f;
                outFrame.radius *= 0.85f;
            } else if (role == HandColliderRole::PalmBack) {
                outFrame.transform.translate = outFrame.transform.translate + rawPalmDepthAxis * 0.75f;
                outFrame.radius *= 0.95f;
            }
            if (!handRoleFrameDimensionsValid(outFrame, role, _lastCapturedPowerArmor)) {
                ROCK_LOG_WARN(Hand,
                    "{} {} collider frame rejected: implausible dimensions length={:.2f} radius={:.2f} convex={:.2f} powerArmor={}",
                    isLeft ? "Left" : "Right",
                    hand_collider_semantics::roleName(role),
                    outFrame.length,
                    outFrame.radius,
                    outFrame.convexRadius,
                    _lastCapturedPowerArmor ? "yes" : "no");
                outFrame = {};
                return false;
            }
            return true;
        }

        hand_bone_collider_geometry_math::BoneColliderFrameInput<RE::NiTransform, RE::NiPoint3> input{};
        input.radius = roleRadius(role, _lastCapturedPowerArmor) * _tuning.roles[static_cast<std::size_t>(role)].radiusScale;
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
                /*
                 * The distal phalanx has its own flexion angle (FRIK writes
                 * prox/mid/dist joint rotations); extrapolating straight along
                 * the middle→distal segment left the tip collider unbent while
                 * the rendered fingertip curled. Follow the distal bone's own
                 * long axis instead.
                 */
                input.extrapolateAlongStartBoneAxis = true;
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
        if (!handRoleFrameDimensionsValid(outFrame, role, _lastCapturedPowerArmor)) {
            ROCK_LOG_WARN(Hand,
                "{} {} collider frame rejected: implausible dimensions length={:.2f} radius={:.2f} convex={:.2f} powerArmor={}",
                isLeft ? "Left" : "Right",
                hand_collider_semantics::roleName(role),
                outFrame.length,
                outFrame.radius,
                outFrame.convexRadius,
                _lastCapturedPowerArmor ? "yes" : "no");
            outFrame = {};
            return false;
        }
        return true;
    }

    RE::hknpShape* HandBoneColliderSet::buildShapeForRole(const RoleFrameResult& frame, HandColliderRole role,
        RE::NiPoint3* outPalmHalfExtents) const
    {
        if (outPalmHalfExtents) {
            *outPalmHalfExtents = {};
        }
        if (!frame.valid) {
            return nullptr;
        }

        float length = frame.length;
        float radius = frame.radius;
        if (hand_collider_semantics::isPalmRole(role)) {
            length = (std::max)(2.5f, frame.length);
            radius = (std::max)(0.8f, frame.radius);
            const float crossPalmWidth = radius * 2.25f;
            const float palmDepth = role == HandColliderRole::PalmFace ? radius * 0.55f :
                                    role == HandColliderRole::PalmBack ? radius * 0.70f :
                                    role == HandColliderRole::ThumbPad ? radius * 0.80f :
                                    radius * 0.95f;
            const auto dimensionScale = _tuning.roles[static_cast<std::size_t>(role)].palmDimensionScale;
            length *= dimensionScale.x;
            const float scaledPalmDepth = palmDepth * dimensionScale.y;
            const float scaledCrossPalmWidth = crossPalmWidth * dimensionScale.z;
            const float radialEquivalent = (std::max)(scaledPalmDepth, scaledCrossPalmWidth) * 0.5f;
            if (!hand_bone_collider_geometry_math::colliderDimensionsWithinLimits(
                    length,
                    radialEquivalent,
                    frame.convexRadius,
                    handRoleDimensionLimits(role, _lastCapturedPowerArmor))) {
                ROCK_LOG_WARN(Hand,
                    "{} collider shape rejected: implausible palm dimensions x={:.2f} y={:.2f} z={:.2f} convex={:.2f} powerArmor={}",
                    hand_collider_semantics::roleName(role),
                    length,
                    scaledPalmDepth,
                    scaledCrossPalmWidth,
                    frame.convexRadius,
                    _lastCapturedPowerArmor ? "yes" : "no");
                return nullptr;
            }
            const auto gamePoints = hand_bone_collider_geometry_math::makePalmBoxHullPoints<RE::NiPoint3>(length, scaledPalmDepth, scaledCrossPalmWidth);
            if (outPalmHalfExtents) {
                for (const auto& point : gamePoints) {
                    outPalmHalfExtents->x = (std::max)(outPalmHalfExtents->x, std::abs(point.x));
                    outPalmHalfExtents->y = (std::max)(outPalmHalfExtents->y, std::abs(point.y));
                    outPalmHalfExtents->z = (std::max)(outPalmHalfExtents->z, std::abs(point.z));
                }
            }
            return havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(toHavokPointCloud(gamePoints), frame.convexRadius * gameToHavokScale());
        }

        const auto gamePoints = hand_bone_collider_geometry_math::makeCapsuleLikeHullPoints<RE::NiPoint3>(length, radius);
        return havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(toHavokPointCloud(gamePoints), frame.convexRadius * gameToHavokScale());
    }

    RE::hknpShape* HandBoneColliderSet::buildDynamicTwinShape(const dynamic_hand_twin::TwinSlotFrame& slotFrame, bool isPalm) const
    {
        if (!slotFrame.valid) {
            return nullptr;
        }

        /*
         * The dynamic twins reuse the exact hull construction of their
         * keyframed counterparts: the palm-anchor box hull or a finger-segment
         * capsule hull for the published dimensions. buildShapeForRole only
         * branches on palm-vs-segment, so any finger role selects the segment
         * path.
         */
        RoleFrameResult frame{};
        frame.valid = true;
        frame.transform = slotFrame.target;
        frame.length = slotFrame.length;
        frame.radius = slotFrame.radius;
        frame.convexRadius = slotFrame.convexRadius;
        return buildShapeForRole(frame, isPalm ? HandColliderRole::PalmAnchor : HandColliderRole::IndexTip);
    }

    bool HandBoneColliderSet::createBodyForRole(RE::hknpWorld* world, void* bhkWorld, bool isLeft, HandColliderRole role, const RoleFrameResult& frame, BodyInstance& instance)
    {
        auto* shape = buildShapeForRole(frame, role, &instance.palmHalfExtents);
        if (!shape) {
            ROCK_LOG_WARN(Hand, "{} {} collider shape build failed", isLeft ? "Left" : "Right", hand_collider_semantics::roleName(role));
            return false;
        }

        instance.shape = shape;
        instance.role = role;
        instance.ownsShapeRef = true;
        clearGeneratedKeyframedBodyDriveState(instance.driveState);

        const std::string name = std::string(isLeft ? "ROCK_Left" : "ROCK_Right") + hand_collider_semantics::roleName(role);
        if (!instance.body.create(world, bhkWorld, shape, kHandFilterInfo, havok_material_registry::registerGeneratedBodyMaterial(world), BethesdaMotionType::Keyframed, name.c_str())) {
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
            instance.body.retireDeferred(bhkWorld);
            shapeRemoveRef(shape);
            clearInstance(instance, false);
            return false;
        }
        initializeGeneratedKeyframedBodyDriveState(instance.driveState, frame.transform);
        ROCK_LOG_DEBUG(Hand,
            "{} {} collider created bodyId={} length={:.2f} radius={:.2f}",
            isLeft ? "Left" : "Right",
            hand_collider_semantics::roleName(role),
            instance.body.getBodyId().value,
            frame.length,
            frame.radius);
        return true;
    }

    bool HandBoneColliderSet::create(
        RE::hknpWorld* world,
        void* bhkWorld,
        bool isLeft,
        const RE::NiTransform& rollAuthorityWorld,
        BethesdaPhysicsBody& palmAnchorBody)
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        destroy(bhkWorld, palmAnchorBody);
        if (!world || !bhkWorld) {
            return false;
        }

        BoneFrameLookup lookup{};
        if (!captureBoneLookup(isLeft, rollAuthorityWorld, lookup)) {
            return false;
        }
        const auto tuningSignature = refreshTuning(_lastCapturedPowerArmor);
        dynamic_hand_twin::TwinTargets canonicalTwinTargets{};
        const auto publishCanonicalTwinSlot = [](dynamic_hand_twin::TwinSlotFrame& slot, const RoleFrameResult& frame) {
            slot.valid = true;
            slot.target = frame.transform;
            slot.length = frame.length;
            slot.radius = frame.radius;
            slot.convexRadius = frame.convexRadius;
        };

        RoleFrameResult anchorFrame{};
        if (!makeRoleFrame(lookup, isLeft, HandColliderRole::PalmAnchor, anchorFrame)) {
            ROCK_LOG_ERROR(Hand, "{} palm anchor frame could not be derived; bone-derived hand creation cannot continue", isLeft ? "Left" : "Right");
            return false;
        }
        publishCanonicalTwinSlot(canonicalTwinTargets.palm, anchorFrame);

        auto* anchorShape = buildShapeForRole(anchorFrame, HandColliderRole::PalmAnchor);
        if (!anchorShape) {
            ROCK_LOG_ERROR(Hand, "{} palm anchor shape build failed; bone-derived hand creation cannot continue", isLeft ? "Left" : "Right");
            return false;
        }

        const std::string anchorName = std::string(isLeft ? "ROCK_Left" : "ROCK_Right") + "PalmAnchor";
        if (!palmAnchorBody.create(world, bhkWorld, anchorShape, kHandFilterInfo, havok_material_registry::registerGeneratedBodyMaterial(world), BethesdaMotionType::Keyframed, anchorName.c_str())) {
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
            palmAnchorBody.retireDeferred(bhkWorld);
            return false;
        }
        initializeGeneratedKeyframedBodyDriveState(_palmAnchorDriveState, anchorFrame.transform);
        _latestPalmAnchorTarget = anchorFrame.transform;
        _hasLatestPalmAnchorTarget = true;

        std::size_t createdCount = 0;
        for (const auto role : hand_collider_semantics::kHandNonAnchorColliderRoles) {
            RoleFrameResult frame{};
            if (!makeRoleFrame(lookup, isLeft, role, frame)) {
                ROCK_LOG_ERROR(Hand, "{} {} collider frame missing; destroying bone-derived hand", isLeft ? "Left" : "Right", hand_collider_semantics::roleName(role));
                destroy(bhkWorld, palmAnchorBody);
                return false;
            }

            if (createdCount >= _bodies.size()) {
                break;
            }
            if (!createBodyForRole(world, bhkWorld, isLeft, role, frame, _bodies[createdCount])) {
                ROCK_LOG_ERROR(Hand, "{} {} collider body allocation failed; destroying bone-derived hand", isLeft ? "Left" : "Right", hand_collider_semantics::roleName(role));
                destroy(bhkWorld, palmAnchorBody);
                return false;
            }
            if (hand_collider_semantics::isFingerRole(role)) {
                const auto fingerIndex =
                    static_cast<std::size_t>(hand_collider_semantics::fingerForRole(role));
                const auto segmentIndex =
                    static_cast<std::size_t>(hand_collider_semantics::segmentForRole(role));
                if (fingerIndex < canonicalTwinTargets.fingers.size() &&
                    segmentIndex < canonicalTwinTargets.fingers[fingerIndex].size()) {
                    publishCanonicalTwinSlot(
                        canonicalTwinTargets.fingers[fingerIndex][segmentIndex],
                        frame);
                }
            }
            ++createdCount;
        }

        _cachedWorld = world;
        _cachedBhkWorld = bhkWorld;
        _cachedSkeleton = _lastCapturedSkeleton;
        _cachedBoneTree = _lastCapturedBoneTree;
        _cachedPowerArmor = _lastCapturedPowerArmor;
        _cachedTuningSignature = tuningSignature;
        _isLeftAtomic.store(isLeft ? 1u : 0u, std::memory_order_release);
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _created = true;
        if (++_dynamicTwinGeometryGeneration == 0) {
            _dynamicTwinGeometryGeneration = 1;
        }
        canonicalTwinTargets.geometryGeneration = _dynamicTwinGeometryGeneration;
        _canonicalDynamicTwinDimensions = canonicalTwinTargets;
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

        if (palmAnchorBody.isValid()) {
            palmAnchorBody.retireDeferred(bhkWorld ? bhkWorld : _cachedBhkWorld);
        }

        _created = false;
        _cachedWorld = nullptr;
        _cachedBhkWorld = nullptr;
        clearGeneratedKeyframedBodyDriveState(_palmAnchorDriveState);
        _latestPalmAnchorTarget = {};
        _hasLatestPalmAnchorTarget = false;
        _dynamicTwinTargets = {};
        _segmentFrames = {};
        _canonicalDynamicTwinDimensions = {};
        _cachedSkeleton = nullptr;
        _cachedBoneTree = nullptr;
        _cachedPowerArmor = false;
        _cachedTuningSignature = 0;
        _isLeftAtomic.store(0, std::memory_order_release);
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _reader.resetCache();
    }

    void HandBoneColliderSet::reset()
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        clearAtomicBodyIds();
        for (auto& instance : _bodies) {
            clearInstance(instance, false);
        }
        _created = false;
        _cachedWorld = nullptr;
        _cachedBhkWorld = nullptr;
        clearGeneratedKeyframedBodyDriveState(_palmAnchorDriveState);
        _latestPalmAnchorTarget = {};
        _hasLatestPalmAnchorTarget = false;
        _dynamicTwinTargets = {};
        _segmentFrames = {};
        _canonicalDynamicTwinDimensions = {};
        _cachedSkeleton = nullptr;
        _cachedBoneTree = nullptr;
        _cachedPowerArmor = false;
        _cachedTuningSignature = 0;
        _isLeftAtomic.store(0, std::memory_order_release);
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _reader.resetCache();
    }

    void HandBoneColliderSet::update(
        RE::hknpWorld* world,
        bool isLeft,
        const RE::NiTransform& rollAuthorityWorld,
        BethesdaPhysicsBody& palmAnchorBody,
        float deltaTime,
        const DirectSkeletonBoneSnapshot& colliderBones)
    {
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
            create(world, _cachedBhkWorld, isLeft, rollAuthorityWorld, palmAnchorBody);
            return;
        }

        BoneFrameLookup lookup{};
        if (!makeBoneLookup(colliderBones, isLeft, rollAuthorityWorld, lookup)) {
            return;
        }

        const auto tuningSignature = refreshTuning(_lastCapturedPowerArmor);
        if (_cachedWorld != world ||
            _cachedSkeleton != _lastCapturedSkeleton ||
            _cachedBoneTree != _lastCapturedBoneTree ||
            _cachedPowerArmor != _lastCapturedPowerArmor ||
            _cachedTuningSignature != tuningSignature) {
            if (palmAnchorBody.isConstrained()) {
                if (++_updateLogCounter > 120) {
                    _updateLogCounter = 0;
                    ROCK_LOG_WARN(Hand,
                        "{} bone-derived hand collider source/tuning rebuild deferred while palm anchor is constrained; live transforms still update tuning=0x{:016X}->0x{:016X}",
                        isLeft ? "Left" : "Right",
                        _cachedTuningSignature,
                        tuningSignature);
                }
            } else {
                ROCK_LOG_INFO(Hand,
                    "{} bone-derived hand collider source/tuning changed; rebuilding tuning=0x{:016X}->0x{:016X}",
                    isLeft ? "Left" : "Right",
                    _cachedTuningSignature,
                    tuningSignature);
                create(world, _cachedBhkWorld, isLeft, rollAuthorityWorld, palmAnchorBody);
                return;
            }
        }

        dynamic_hand_twin::TwinTargets twinTargets{};
        auto publishTwinSlot = [](dynamic_hand_twin::TwinSlotFrame& slot, const RoleFrameResult& frame) {
            slot.valid = true;
            slot.target = frame.transform;
            slot.length = frame.length;
            slot.radius = frame.radius;
            slot.convexRadius = frame.convexRadius;
        };

        RoleFrameResult anchorFrame{};
        if (makeRoleFrame(lookup, isLeft, HandColliderRole::PalmAnchor, anchorFrame)) {
            _latestPalmAnchorTarget = anchorFrame.transform;
            _hasLatestPalmAnchorTarget = true;
            publishTwinSlot(twinTargets.palm, anchorFrame);
            queueBodyTarget(palmAnchorBody, anchorFrame.transform, deltaTime, _palmAnchorDriveState, _palmAnchorPublicationIndex);
        }

        PublishedSegmentFrames segmentFrames{};
        std::size_t publishedSegmentCount = 0;
        for (auto& instance : _bodies) {
            if (!instance.body.isValid()) {
                continue;
            }
            RoleFrameResult frame{};
            if (makeRoleFrame(lookup, isLeft, instance.role, frame)) {
                if (hand_collider_semantics::isFingerRole(instance.role)) {
                    const auto fingerIndex = static_cast<std::size_t>(hand_collider_semantics::fingerForRole(instance.role));
                    const auto segmentIndex = static_cast<std::size_t>(hand_collider_semantics::segmentForRole(instance.role));
                    if (fingerIndex < twinTargets.fingers.size() &&
                        segmentIndex < twinTargets.fingers[fingerIndex].size()) {
                        publishTwinSlot(
                            twinTargets.fingers[fingerIndex][segmentIndex],
                            frame);
                    }
                }
                if (publishedSegmentCount < segmentFrames.size()) {
                    auto& published = segmentFrames[publishedSegmentCount++];
                    published.valid = true;
                    published.role = instance.role;
                    published.target = frame.transform;
                    published.length = frame.length;
                    published.radius = frame.radius;
                    published.convexRadius = frame.convexRadius;
                    published.palmHalfExtents = instance.palmHalfExtents;
                }
                queueBodyTarget(instance.body, frame.transform, deltaTime, instance.driveState, instance.publicationIndex);
            }
        }
        _segmentFrames = segmentFrames;

        dynamic_hand_twin::applyCanonicalHandDimensions(
            twinTargets,
            _canonicalDynamicTwinDimensions);
        twinTargets.updateCounter = _dynamicTwinTargets.updateCounter + 1;
        twinTargets.geometryGeneration = _dynamicTwinGeometryGeneration;
        _dynamicTwinTargets = twinTargets;
    }

    void HandBoneColliderSet::flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing, BethesdaPhysicsBody& palmAnchorBody)
    {
        if (!world || !_created || !palmAnchorBody.isValid()) {
            return;
        }

        handleGeneratedBodyDriveResult(
            driveGeneratedKeyframedBody(world,
                palmAnchorBody,
                _palmAnchorDriveState,
                timing,
                "hand-palm-anchor",
                0,
                g_rockConfig.rockHandBoneColliderMaxLinearVelocity,
                g_rockConfig.rockHandBoneColliderMaxAngularVelocity),
            "hand-palm-anchor",
            0);

        for (std::size_t i = 0; i < _bodies.size(); ++i) {
            auto& instance = _bodies[i];
            if (!instance.body.isValid()) {
                continue;
            }
            const auto bodyIndex = static_cast<std::uint32_t>(i + 1);
            handleGeneratedBodyDriveResult(
                driveGeneratedKeyframedBody(world,
                    instance.body,
                    instance.driveState,
                    timing,
                    "hand-bone-collider",
                    bodyIndex,
                    g_rockConfig.rockHandBoneColliderMaxLinearVelocity,
                    g_rockConfig.rockHandBoneColliderMaxAngularVelocity),
                "hand-bone-collider",
                bodyIndex);
        }
    }

    void HandBoneColliderSet::queueBodyTarget(BethesdaPhysicsBody& body, const RE::NiTransform& target, float sourceDeltaSeconds, GeneratedKeyframedBodyDriveState& driveState, std::uint32_t publicationIndex)
    {
        if (!body.isValid()) {
            return;
        }

        const auto queueResult = queueGeneratedKeyframedBodyTarget(driveState, target, sourceDeltaSeconds, 1000.0f);
        publishSampledVelocityAtomic(publicationIndex, queueResult);
    }

    bool HandBoneColliderSet::tryGetPalmAnchorTarget(RE::NiTransform& outTarget) const
    {
        if (!_hasLatestPalmAnchorTarget) {
            return false;
        }
        outTarget = _latestPalmAnchorTarget;
        return true;
    }

    bool HandBoneColliderSet::tryGetBodyTargetForDebug(
        const std::uint32_t bodyId,
        RE::NiTransform& outTarget) const
    {
        if (bodyId == hand_collider_semantics::kInvalidBodyId) {
            return false;
        }

        if (_hasLatestPalmAnchorTarget &&
            _palmAnchorPublicationIndex != kInvalidPublicationIndex &&
            getBodyIdAtomic(_palmAnchorPublicationIndex) == bodyId) {
            outTarget = _latestPalmAnchorTarget;
            return true;
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
            "Hand generated collider drive result requested rebuild owner={} bodyIndex={} failures={} missingBody={} ownerMismatch={} placementFailed={} nativeDriveFailed={} bodyDeltaGame={:.2f} bodyRotErr={:.2f}",
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

    void HandBoneColliderSet::clearInstance(BodyInstance& instance, bool releaseShapeRef)
    {
        if (releaseShapeRef && instance.ownsShapeRef && instance.shape) {
            shapeRemoveRef(instance.shape);
        }
        instance.body.reset();
        instance.shape = nullptr;
        instance.role = HandColliderRole::PalmFace;
        instance.ownsShapeRef = false;
        instance.palmHalfExtents = {};
        clearGeneratedKeyframedBodyDriveState(instance.driveState);
        instance.publicationIndex = kInvalidPublicationIndex;
    }

    void HandBoneColliderSet::clearAtomicBodyIds()
    {
        _palmAnchorPublicationIndex = kInvalidPublicationIndex;
        for (auto& instance : _bodies) {
            instance.publicationIndex = kInvalidPublicationIndex;
        }
        _bodyCountAtomic.store(0, std::memory_order_release);
        for (std::size_t i = 0; i < _bodyIdsAtomic.size(); ++i) {
            _bodyIdsAtomic[i].store(hand_collider_semantics::kInvalidBodyId, std::memory_order_release);
            _rolesAtomic[i].store(static_cast<std::uint32_t>(HandColliderRole::PalmAnchor), std::memory_order_release);
            _fingersAtomic[i].store(static_cast<std::uint32_t>(HandFinger::None), std::memory_order_release);
            _segmentsAtomic[i].store(static_cast<std::uint32_t>(HandFingerSegment::None), std::memory_order_release);
            _primaryAnchorAtomic[i].store(0, std::memory_order_release);
            _sampledVelocityValidAtomic[i].store(0, std::memory_order_release);
            _sampledVelocityHavokXAtomic[i].store(0.0f, std::memory_order_release);
            _sampledVelocityHavokYAtomic[i].store(0.0f, std::memory_order_release);
            _sampledVelocityHavokZAtomic[i].store(0.0f, std::memory_order_release);
        }
    }

    void HandBoneColliderSet::publishSampledVelocityAtomic(std::uint32_t publicationIndex, const GeneratedKeyframedBodyDriveQueueResult& queueResult)
    {
        if (publicationIndex >= _bodyIdsAtomic.size() || publicationIndex >= _bodyCountAtomic.load(std::memory_order_acquire)) {
            return;
        }

        if (!queueResult.sampledVelocityValid) {
            _sampledVelocityValidAtomic[publicationIndex].store(0, std::memory_order_release);
            _sampledVelocityHavokXAtomic[publicationIndex].store(0.0f, std::memory_order_release);
            _sampledVelocityHavokYAtomic[publicationIndex].store(0.0f, std::memory_order_release);
            _sampledVelocityHavokZAtomic[publicationIndex].store(0.0f, std::memory_order_release);
            return;
        }

        _sampledVelocityHavokXAtomic[publicationIndex].store(queueResult.sampledLinearVelocityHavok.x, std::memory_order_release);
        _sampledVelocityHavokYAtomic[publicationIndex].store(queueResult.sampledLinearVelocityHavok.y, std::memory_order_release);
        _sampledVelocityHavokZAtomic[publicationIndex].store(queueResult.sampledLinearVelocityHavok.z, std::memory_order_release);
        _sampledVelocityValidAtomic[publicationIndex].store(1, std::memory_order_release);
    }

    void HandBoneColliderSet::publishAtomicBodyIds(const BethesdaPhysicsBody& palmAnchorBody, bool isLeft)
    {
        clearAtomicBodyIds();
        std::uint32_t count = 0;
        if (palmAnchorBody.isValid()) {
            _palmAnchorPublicationIndex = count;
            _rolesAtomic[count].store(static_cast<std::uint32_t>(HandColliderRole::PalmAnchor), std::memory_order_release);
            _fingersAtomic[count].store(static_cast<std::uint32_t>(HandFinger::None), std::memory_order_release);
            _segmentsAtomic[count].store(static_cast<std::uint32_t>(HandFingerSegment::None), std::memory_order_release);
            _primaryAnchorAtomic[count].store(1, std::memory_order_release);
            _bodyIdsAtomic[count].store(palmAnchorBody.getBodyId().value, std::memory_order_release);
            ++count;
        }

        for (auto& instance : _bodies) {
            if (!instance.body.isValid() || count >= _bodyIdsAtomic.size()) {
                instance.publicationIndex = kInvalidPublicationIndex;
                continue;
            }
            instance.publicationIndex = count;
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
            return tryGetBodyMetadataAtIndexAtomic(i, bodyId, outMetadata);
        }
        return false;
    }

    bool HandBoneColliderSet::tryGetBodyMetadataAtIndexAtomic(std::uint32_t i,
        std::uint32_t bodyId, HandColliderBodyMetadata& outMetadata) const
    {
        outMetadata = {};
        if (i >= _bodyIdsAtomic.size() || i >= _bodyCountAtomic.load(std::memory_order_acquire) ||
            bodyId == hand_collider_semantics::kInvalidBodyId || _bodyIdsAtomic[i].load(std::memory_order_acquire) != bodyId) {
            return false;
        }
        outMetadata.valid = true;
        outMetadata.isLeft = _isLeftAtomic.load(std::memory_order_acquire) != 0;
        outMetadata.bodyId = bodyId;
        outMetadata.role = static_cast<HandColliderRole>(_rolesAtomic[i].load(std::memory_order_acquire));
        outMetadata.finger = static_cast<HandFinger>(_fingersAtomic[i].load(std::memory_order_acquire));
        outMetadata.segment = static_cast<HandFingerSegment>(_segmentsAtomic[i].load(std::memory_order_acquire));
        outMetadata.primaryPalmAnchor = _primaryAnchorAtomic[i].load(std::memory_order_acquire) != 0;
        if (_sampledVelocityValidAtomic[i].load(std::memory_order_acquire) != 0) {
            const float vx = _sampledVelocityHavokXAtomic[i].load(std::memory_order_acquire);
            const float vy = _sampledVelocityHavokYAtomic[i].load(std::memory_order_acquire);
            const float vz = _sampledVelocityHavokZAtomic[i].load(std::memory_order_acquire);
            if (std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz)) {
                outMetadata.hasSampledLinearVelocityHavok = true;
                outMetadata.sampledLinearVelocityHavok[0] = vx;
                outMetadata.sampledLinearVelocityHavok[1] = vy;
                outMetadata.sampledLinearVelocityHavok[2] = vz;
                outMetadata.sampledLinearVelocityHavok[3] = 0.0f;
            }
        }
        return true;
    }
}
