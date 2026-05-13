#include "physics-interaction/body/BodyBoneColliderSet.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/native/HavokConvexShapeBuilder.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/PhysicsUtils.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace rock
{
    namespace
    {
        using skeleton_bone_debug_math::BoneColliderDescriptor;
        using skeleton_bone_debug_math::BoneColliderRole;

        constexpr std::uint32_t kBodyFilterInfo = (0x000B << 16) | (collision_layer_policy::ROCK_LAYER_BODY & 0x7F);

        using BodyDescriptorArray = std::array<BoneColliderDescriptor, kBodyBoneColliderBodyCount>;
        using SnapshotBoneMap = std::unordered_map<std::string_view, const DirectSkeletonBoneEntry*>;

        const BodyDescriptorArray& bodyDescriptorsForPowerArmor(bool inPowerArmor)
        {
            return inPowerArmor ? skeleton_bone_debug_math::kPowerArmorBodyColliderDescriptors :
                                  skeleton_bone_debug_math::kStandardBodyColliderDescriptors;
        }

        float sanitizeBodyColliderScale(float value)
        {
            if (!std::isfinite(value)) {
                return 1.0f;
            }
            return std::clamp(value, 0.05f, 8.0f);
        }

        float roleRadiusScale(BoneColliderRole role)
        {
            switch (role) {
            case BoneColliderRole::TorsoSegment:
                return sanitizeBodyColliderScale(g_rockConfig.rockBodyBoneColliderTorsoRadiusScale);
            case BoneColliderRole::UpperArmSegment:
            case BoneColliderRole::ForearmSegment:
            case BoneColliderRole::HandSegment:
                return sanitizeBodyColliderScale(g_rockConfig.rockBodyBoneColliderArmRadiusScale);
            case BoneColliderRole::LegSegment:
                return sanitizeBodyColliderScale(g_rockConfig.rockBodyBoneColliderLegRadiusScale);
            case BoneColliderRole::FootSegment:
                return sanitizeBodyColliderScale(g_rockConfig.rockBodyBoneColliderFootRadiusScale);
            case BoneColliderRole::FingerSegment:
                return sanitizeBodyColliderScale(g_rockConfig.rockBodyBoneColliderArmRadiusScale);
            }
            return 1.0f;
        }

        float roleLengthScale(BoneColliderRole role)
        {
            switch (role) {
            case BoneColliderRole::TorsoSegment:
                return sanitizeBodyColliderScale(g_rockConfig.rockBodyBoneColliderTorsoLengthScale);
            case BoneColliderRole::UpperArmSegment:
            case BoneColliderRole::ForearmSegment:
            case BoneColliderRole::HandSegment:
                return sanitizeBodyColliderScale(g_rockConfig.rockBodyBoneColliderArmLengthScale);
            case BoneColliderRole::LegSegment:
                return sanitizeBodyColliderScale(g_rockConfig.rockBodyBoneColliderLegLengthScale);
            case BoneColliderRole::FootSegment:
                return sanitizeBodyColliderScale(g_rockConfig.rockBodyBoneColliderFootLengthScale);
            case BoneColliderRole::FingerSegment:
                return sanitizeBodyColliderScale(g_rockConfig.rockBodyBoneColliderArmLengthScale);
            }
            return 1.0f;
        }

        float profileRadiusScale(bool inPowerArmor)
        {
            return sanitizeBodyColliderScale(inPowerArmor ? g_rockConfig.rockBodyBoneColliderPowerArmorRadiusScale :
                                                            g_rockConfig.rockBodyBoneColliderStandardRadiusScale);
        }

        float profileLengthScale(bool inPowerArmor)
        {
            return sanitizeBodyColliderScale(inPowerArmor ? g_rockConfig.rockBodyBoneColliderPowerArmorLengthScale :
                                                            g_rockConfig.rockBodyBoneColliderStandardLengthScale);
        }

        float profileConvexRadiusScale(bool inPowerArmor)
        {
            return sanitizeBodyColliderScale(inPowerArmor ? g_rockConfig.rockBodyBoneColliderPowerArmorConvexRadiusScale :
                                                            g_rockConfig.rockBodyBoneColliderStandardConvexRadiusScale);
        }

        struct BodyZoneTuningOverride
        {
            bool valid = false;
            float radiusScale = 1.0f;
            float lengthScale = 1.0f;
            float convexRadiusScale = 1.0f;
            RE::NiPoint3 localOffsetGame{};
            bool hasLocalOffset = false;
        };

        std::string_view trimZoneOverrideToken(std::string_view value)
        {
            while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front()))) {
                value.remove_prefix(1);
            }
            while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) {
                value.remove_suffix(1);
            }
            return value;
        }

        char lowerAscii(char ch)
        {
            return static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        }

        bool equalsAsciiInsensitive(std::string_view lhs, std::string_view rhs)
        {
            if (lhs.size() != rhs.size()) {
                return false;
            }
            for (std::size_t i = 0; i < lhs.size(); ++i) {
                if (lowerAscii(lhs[i]) != lowerAscii(rhs[i])) {
                    return false;
                }
            }
            return true;
        }

        bool parseZoneOverrideFloat(std::string_view token, float& outValue)
        {
            token = trimZoneOverrideToken(token);
            if (token.empty()) {
                return false;
            }

            const std::string buffer{ token };
            char* end = nullptr;
            const float parsed = std::strtof(buffer.c_str(), &end);
            if (end == buffer.c_str() || !std::isfinite(parsed)) {
                return false;
            }
            while (end && *end != '\0') {
                if (!std::isspace(static_cast<unsigned char>(*end))) {
                    return false;
                }
                ++end;
            }
            outValue = parsed;
            return true;
        }

        bool parseZoneOverrideValue(std::string_view value, BodyZoneTuningOverride& outOverride)
        {
            outOverride = {};
            float parsed[6]{};
            std::uint32_t count = 0;
            value = trimZoneOverrideToken(value);
            while (!value.empty() && count < 6) {
                const auto comma = value.find(',');
                const auto token = trimZoneOverrideToken(value.substr(0, comma));
                if (!parseZoneOverrideFloat(token, parsed[count])) {
                    return false;
                }
                ++count;
                if (comma == std::string_view::npos) {
                    break;
                }
                if (count >= 6) {
                    return false;
                }
                value.remove_prefix(comma + 1);
            }

            if (count != 3 && count != 6) {
                return false;
            }
            if (value.find(',') != std::string_view::npos) {
                return false;
            }

            outOverride.valid = true;
            outOverride.radiusScale = sanitizeBodyColliderScale(parsed[0]);
            outOverride.lengthScale = sanitizeBodyColliderScale(parsed[1]);
            outOverride.convexRadiusScale = sanitizeBodyColliderScale(parsed[2]);
            if (count == 6) {
                outOverride.localOffsetGame = RE::NiPoint3{ parsed[3], parsed[4], parsed[5] };
                outOverride.hasLocalOffset = true;
            }
            return true;
        }

        bool zoneOverrideProfileMatches(std::string_view profile, bool inPowerArmor)
        {
            profile = trimZoneOverrideToken(profile);
            if (profile.empty()) {
                return true;
            }
            if (equalsAsciiInsensitive(profile, "PowerArmor")) {
                return inPowerArmor;
            }
            if (equalsAsciiInsensitive(profile, "Standard")) {
                return !inPowerArmor;
            }
            return false;
        }

        enum class BodyZoneOverrideMatch
        {
            None,
            Generic,
            ProfileSpecific,
        };

        BodyZoneOverrideMatch zoneOverrideKeyMatch(std::string_view key, body_zone::BodyZoneKind zone, bool inPowerArmor)
        {
            key = trimZoneOverrideToken(key);
            const auto dot = key.find('.');
            std::string_view zoneName = key;
            BodyZoneOverrideMatch matchType = BodyZoneOverrideMatch::Generic;
            if (dot != std::string_view::npos) {
                if (!zoneOverrideProfileMatches(key.substr(0, dot), inPowerArmor)) {
                    return BodyZoneOverrideMatch::None;
                }
                zoneName = trimZoneOverrideToken(key.substr(dot + 1));
                matchType = BodyZoneOverrideMatch::ProfileSpecific;
            }
            return equalsAsciiInsensitive(zoneName, body_zone::bodyZoneName(zone)) ? matchType : BodyZoneOverrideMatch::None;
        }

        BodyZoneTuningOverride bodyZoneTuningOverride(body_zone::BodyZoneKind zone, bool inPowerArmor)
        {
            if (zone == body_zone::BodyZoneKind::Unknown || g_rockConfig.rockBodyBoneColliderZoneScaleOverrides.empty()) {
                return {};
            }

            std::string_view overrides{ g_rockConfig.rockBodyBoneColliderZoneScaleOverrides };
            BodyZoneTuningOverride genericMatch{};
            while (!overrides.empty()) {
                const auto semicolon = overrides.find(';');
                const auto entry = trimZoneOverrideToken(overrides.substr(0, semicolon));
                if (!entry.empty()) {
                    const auto equals = entry.find('=');
                    const auto matchType = equals != std::string_view::npos ? zoneOverrideKeyMatch(entry.substr(0, equals), zone, inPowerArmor) :
                                                                              BodyZoneOverrideMatch::None;
                    if (matchType != BodyZoneOverrideMatch::None) {
                        BodyZoneTuningOverride parsed{};
                        if (parseZoneOverrideValue(entry.substr(equals + 1), parsed)) {
                            if (matchType == BodyZoneOverrideMatch::ProfileSpecific) {
                                return parsed;
                            }
                            genericMatch = parsed;
                        }
                    }
                }
                if (semicolon == std::string_view::npos) {
                    break;
                }
                overrides.remove_prefix(semicolon + 1);
            }
            return genericMatch;
        }

        std::uint32_t quantizeBodyColliderScale(float value)
        {
            return static_cast<std::uint32_t>(std::lround(sanitizeBodyColliderScale(value) * 10000.0f));
        }

        void mixBodyColliderSignature(std::uint64_t& signature, float value)
        {
            signature ^= static_cast<std::uint64_t>(quantizeBodyColliderScale(value)) + 0x9E37'79B9'7F4A'7C15ull + (signature << 6) + (signature >> 2);
        }

        void mixBodyColliderSignatureString(std::uint64_t& signature, const std::string& value)
        {
            for (unsigned char ch : value) {
                signature ^= static_cast<std::uint64_t>(ch) + 0x9E37'79B9'7F4A'7C15ull + (signature << 6) + (signature >> 2);
            }
        }

        std::uint64_t bodyColliderTuningSignature(bool inPowerArmor)
        {
            std::uint64_t signature = inPowerArmor ? 0x5041'524D'4F52ull : 0x5354'414E'4444ull;
            mixBodyColliderSignature(signature, profileRadiusScale(inPowerArmor));
            mixBodyColliderSignature(signature, profileLengthScale(inPowerArmor));
            mixBodyColliderSignature(signature, profileConvexRadiusScale(inPowerArmor));
            mixBodyColliderSignature(signature, g_rockConfig.rockBodyBoneColliderTorsoRadiusScale);
            mixBodyColliderSignature(signature, g_rockConfig.rockBodyBoneColliderArmRadiusScale);
            mixBodyColliderSignature(signature, g_rockConfig.rockBodyBoneColliderLegRadiusScale);
            mixBodyColliderSignature(signature, g_rockConfig.rockBodyBoneColliderFootRadiusScale);
            mixBodyColliderSignature(signature, g_rockConfig.rockBodyBoneColliderTorsoLengthScale);
            mixBodyColliderSignature(signature, g_rockConfig.rockBodyBoneColliderArmLengthScale);
            mixBodyColliderSignature(signature, g_rockConfig.rockBodyBoneColliderLegLengthScale);
            mixBodyColliderSignature(signature, g_rockConfig.rockBodyBoneColliderFootLengthScale);
            mixBodyColliderSignatureString(signature, g_rockConfig.rockBodyBoneColliderZoneScaleOverrides);
            return signature;
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

        SnapshotBoneMap makeSnapshotBoneMap(const DirectSkeletonBoneSnapshot& snapshot)
        {
            SnapshotBoneMap map;
            map.reserve(snapshot.bones.size());
            for (const auto& bone : snapshot.bones) {
                map.emplace(std::string_view{ bone.name }, &bone);
            }
            return map;
        }

        bool findSnapshotBone(const SnapshotBoneMap& bonesByName, std::string_view name, RE::NiTransform& outTransform)
        {
            const auto it = bonesByName.find(name);
            if (it == bonesByName.end() || !it->second) {
                return false;
            }

            outTransform = it->second->world;
            return true;
        }

        bool makeDescriptorFrame(
            const SnapshotBoneMap& bonesByName,
            const BoneColliderDescriptor& descriptor,
            bool inPowerArmor,
            BodyBoneColliderSet::DescriptorFrameResult& outFrame)
        {
            outFrame = {};
            if (!descriptor.enabled || descriptor.endpointMode != skeleton_bone_debug_math::BoneColliderEndpointMode::ChildBone) {
                return false;
            }

            hand_bone_collider_geometry_math::BoneColliderFrameInput<RE::NiTransform, RE::NiPoint3> input{};
            const auto zoneOverride = bodyZoneTuningOverride(descriptor.zone, inPowerArmor);
            const float radiusScale = profileRadiusScale(inPowerArmor) * roleRadiusScale(descriptor.role) *
                                      (zoneOverride.valid ? zoneOverride.radiusScale : 1.0f);
            const float lengthScale = profileLengthScale(inPowerArmor) * roleLengthScale(descriptor.role) *
                                      (zoneOverride.valid ? zoneOverride.lengthScale : 1.0f);
            const float convexRadiusScale = profileConvexRadiusScale(inPowerArmor) *
                                            (zoneOverride.valid ? zoneOverride.convexRadiusScale : 1.0f);
            input.radius = descriptor.radiusGameUnits * radiusScale;
            input.convexRadius = descriptor.convexRadiusGameUnits * convexRadiusScale;

            if (!findSnapshotBone(bonesByName, descriptor.startBone, input.start) ||
                !findSnapshotBone(bonesByName, descriptor.endBone, input.end)) {
                return false;
            }

            const auto frame = hand_bone_collider_geometry_math::buildSegmentColliderFrame(input);
            if (!frame.valid) {
                return false;
            }

            outFrame.valid = true;
            outFrame.transform = frame.transform;
            if (zoneOverride.valid && zoneOverride.hasLocalOffset) {
                const auto offsetWorld = transform_math::localVectorToWorld(outFrame.transform, zoneOverride.localOffsetGame);
                outFrame.transform.translate.x += offsetWorld.x;
                outFrame.transform.translate.y += offsetWorld.y;
                outFrame.transform.translate.z += offsetWorld.z;
            }
            outFrame.length = frame.length * lengthScale;
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

    BodyBoneColliderSet::BodyBoneColliderSet()
    {
        clearAtomicBodyIds();
    }

    bool BodyBoneColliderSet::captureBoneSnapshot(DirectSkeletonBoneSnapshot& outSnapshot)
    {
        if (!_reader.capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::AllFlattenedBones,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
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
            instance.body.destroy(bhkWorld);
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
        destroy(bhkWorld);
        if (!world || !bhkWorld) {
            return false;
        }

        DirectSkeletonBoneSnapshot snapshot{};
        if (!captureBoneSnapshot(snapshot)) {
            ROCK_LOG_WARN(Body, "Body bone colliders not created: root flattened skeleton snapshot unavailable");
            return false;
        }

        const auto& descriptors = bodyDescriptorsForPowerArmor(snapshot.inPowerArmor);
        const auto tuningSignature = bodyColliderTuningSignature(snapshot.inPowerArmor);
        const auto bonesByName = makeSnapshotBoneMap(snapshot);
        std::size_t createdCount = 0;
        for (std::uint32_t descriptorIndex = 0; descriptorIndex < descriptors.size(); ++descriptorIndex) {
            const auto& descriptor = descriptors[descriptorIndex];
            DescriptorFrameResult frame{};
            if (!makeDescriptorFrame(bonesByName, descriptor, snapshot.inPowerArmor, frame)) {
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
            ++createdCount;
        }

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
        publishAtomicBodyIds(snapshot.inPowerArmor);

        ROCK_LOG_INFO(Body,
            "Body bone colliders created: bodies={} descriptors={} sourceSkeleton={} tree={} powerArmor={}",
            createdCount,
            descriptors.size(),
            reinterpret_cast<std::uintptr_t>(_cachedSkeleton),
            reinterpret_cast<std::uintptr_t>(_cachedBoneTree),
            _cachedPowerArmor ? "yes" : "no");
        return true;
    }

    void BodyBoneColliderSet::destroy(void* bhkWorld)
    {
        clearAtomicBodyIds();
        for (auto& instance : _bodies) {
            if (instance.body.isValid()) {
                instance.body.destroy(bhkWorld ? bhkWorld : _cachedBhkWorld);
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
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _reader.resetCache();
    }

    void BodyBoneColliderSet::reset()
    {
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
        _driveRebuildRequested.store(false, std::memory_order_release);
        _driveFailureCount.store(0, std::memory_order_release);
        _reader.resetCache();
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

        DirectSkeletonBoneSnapshot snapshot{};
        if (!captureBoneSnapshot(snapshot)) {
            return;
        }

        const auto tuningSignature = bodyColliderTuningSignature(snapshot.inPowerArmor);
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
        const auto bonesByName = makeSnapshotBoneMap(snapshot);
        for (auto& instance : _bodies) {
            if (!instance.body.isValid() || instance.descriptorIndex >= descriptors.size()) {
                continue;
            }

            DescriptorFrameResult frame{};
            if (makeDescriptorFrame(bonesByName, descriptors[instance.descriptorIndex], snapshot.inPowerArmor, frame)) {
                queueBodyTarget(instance.body, frame.transform, deltaTime, instance.driveState);
            }
        }
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
                    static_cast<std::uint32_t>(i)),
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
            "Body generated collider drive result requested rebuild owner={} bodyIndex={} failures={} missingBody={} placementFailed={} nativeDriveFailed={} bodyDeltaGame={:.2f} bodyRotErr={:.2f}",
            ownerName ? ownerName : "unknown",
            bodyIndex,
            failures,
            result.missingBody ? "yes" : "no",
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
        return false;
    }
}
