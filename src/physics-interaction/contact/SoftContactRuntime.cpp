#include "physics-interaction/contact/SoftContactRuntime.h"

#include "RockConfig.h"
#include "RockUtils.h"
#include "api/FRIKApi.h"
#include "physics-interaction/contact/ContactTargetIdentity.h"
#include "physics-interaction/contact/SoftContactWorldPolicy.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/native/PhysicsShapeCast.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/weapon/WeaponCollision.h"

#include "RE/Havok/hknpAllHitsCollector.h"
#include "RE/Havok/hknpCollisionResult.h"
#include "RE/NetImmerse/NiAVObject.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <vrcf/VRControllersManager.h>

namespace rock
{
    namespace
    {
        using soft_contact_math::Capsule;
        using soft_contact_math::CapsuleContact;
        using soft_contact_math::ContactKind;
        using soft_contact_math::ContactState;

        constexpr const char* RIGHT_SOFT_CONTACT_TAG = "ROCK_SoftContact_Right";
        constexpr const char* LEFT_SOFT_CONTACT_TAG = "ROCK_SoftContact_Left";
        constexpr float kCorrectionClearDistance = 0.001f;
        constexpr float kWorldProbeMinSweepDistanceGameUnits = 0.05f;
        constexpr float kWorldProbeRestQueryDistanceGameUnits = 1.0f;
        constexpr float kWorldProbeRestQueryCooldownSeconds = 0.08f;
        constexpr float kDefaultWorldContactPaddingGameUnits = 0.35f;
        constexpr float kDefaultWorldPostReleaseReentryMinApproachDistanceGameUnits = 0.025f;
        constexpr float kDefaultWorldCachedPlaneMaxTangentDriftGameUnits = 10.0f;
        constexpr float kDefaultWeaponHandRadiusPaddingGameUnits = 0.25f;
        constexpr float kDefaultWeaponHandMaxCorrectionGameUnits = 3.0f;
        constexpr float kDefaultWeaponHandCorrectionScale = 0.35f;
        constexpr float kDefaultWeaponHandHardStopPenetrationGameUnits = 4.0f;
        constexpr std::uint32_t kNativeWorldContactMaxAgeFrames = 2;
        constexpr std::uint32_t kWorldProbeIdRightBase = 0x5000u;
        constexpr std::uint32_t kWorldProbeIdLeftBase = 0x6000u;

        enum class ShapeOwnerSide : std::uint8_t
        {
            Neutral = 0,
            Left,
            Right
        };

        struct RuntimeShape
        {
            Capsule capsule{};
            ContactKind kind = ContactKind::None;
            ShapeOwnerSide ownerSide = ShapeOwnerSide::Neutral;
            bool sameSideArm = false;
        };

        enum class CandidateSource : std::uint8_t
        {
            Shape = 0,
            QueryWorld,
            CachedWorldPlane,
            NativeWorld,
        };

        struct Candidate
        {
            bool valid = false;
            bool suppressed = false;
            ContactKind kind = ContactKind::None;
            CandidateSource source = CandidateSource::Shape;
            CapsuleContact contact{};
            float approachSpeedGameUnits = 0.0f;
            contact_target_identity::ContactTargetIdentity targetIdentity{};
        };

        struct CachedWorldPlaneResult
        {
            Candidate candidate{};
            bool hadCachedPlane = false;
            bool leftContact = false;
            RE::NiPoint3 normal{};
        };

        struct WorldContactProbe
        {
            bool valid = false;
            RE::NiPoint3 position{};
            float radius = 0.0f;
            std::uint32_t id = 0;
            std::size_t stateIndex = 0;
        };

        using BoneMap = std::unordered_map<std::string_view, const DirectSkeletonBoneEntry*>;

        contact_target_identity::ContactTargetResolutionOptions contactTargetResolutionOptions()
        {
            /*
             * Contact solving only needs cheap body/filter/motion identity.
             * Ref traversal and string copies are diagnostics for mapping world
             * contacts, so keep them behind the explicit identity log toggle.
             */
            const bool richDiagnostics = g_rockConfig.rockDebugContactTargetIdentityLogging;
            return contact_target_identity::ContactTargetResolutionOptions{
                .resolveReference = richDiagnostics,
                .includeRichText = richDiagnostics,
            };
        }

        const char* softContactTag(bool isLeft)
        {
            return isLeft ? LEFT_SOFT_CONTACT_TAG : RIGHT_SOFT_CONTACT_TAG;
        }

        std::size_t handIndex(bool isLeft)
        {
            return isLeft ? 1u : 0u;
        }

        bool startsWith(std::string_view value, std::string_view prefix)
        {
            return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
        }

        ShapeOwnerSide sideForBoneName(std::string_view name)
        {
            if (startsWith(name, "LArm_")) {
                return ShapeOwnerSide::Left;
            }
            if (startsWith(name, "RArm_")) {
                return ShapeOwnerSide::Right;
            }
            return ShapeOwnerSide::Neutral;
        }

        bool isSameSideArmDescriptor(const skeleton_bone_debug_math::BoneColliderDescriptor& descriptor, bool isLeft)
        {
            const auto desiredSide = isLeft ? ShapeOwnerSide::Left : ShapeOwnerSide::Right;
            return (sideForBoneName(descriptor.startBone) == desiredSide || sideForBoneName(descriptor.endBone) == desiredSide) &&
                   (startsWith(descriptor.startBone, isLeft ? "LArm_" : "RArm_") || startsWith(descriptor.endBone, isLeft ? "LArm_" : "RArm_"));
        }

        BoneMap makeBoneMap(const DirectSkeletonBoneSnapshot& snapshot)
        {
            BoneMap map;
            map.reserve(snapshot.bones.size());
            for (const auto& bone : snapshot.bones) {
                map.emplace(std::string_view{ bone.name }, &bone);
            }
            return map;
        }

        bool findBonePosition(const BoneMap& bones, std::string_view name, RE::NiPoint3& outPosition)
        {
            const auto it = bones.find(name);
            if (it == bones.end() || !it->second) {
                return false;
            }

            outPosition = it->second->world.translate;
            return true;
        }

        float fingerRadius(std::size_t segmentIndex)
        {
            switch (segmentIndex) {
            case 0:
                return 0.65f;
            case 1:
                return 0.55f;
            default:
                return 0.45f;
            }
        }

        std::uint32_t makeShapeId(bool isLeft, std::uint32_t ordinal)
        {
            return (isLeft ? 0x2000u : 0x1000u) + ordinal;
        }

        std::uint32_t makeWorldProbeId(bool isLeft, std::uint32_t ordinal)
        {
            return (isLeft ? kWorldProbeIdLeftBase : kWorldProbeIdRightBase) + ordinal;
        }

        void addCapsule(std::vector<RuntimeShape>& outShapes,
            const RE::NiPoint3& start,
            const RE::NiPoint3& end,
            float radius,
            ContactKind kind,
            std::uint32_t id,
            ShapeOwnerSide side = ShapeOwnerSide::Neutral,
            bool sameSideArm = false)
        {
            if (!soft_contact_math::isFinite(start) || !soft_contact_math::isFinite(end) || !std::isfinite(radius) || radius <= 0.0f) {
                return;
            }

            RuntimeShape shape{};
            shape.capsule = Capsule{
                .start = start,
                .end = end,
                .radius = radius,
                .id = id,
                .valid = true,
            };
            shape.kind = kind;
            shape.ownerSide = side;
            shape.sameSideArm = sameSideArm;
            outShapes.push_back(shape);
        }

        void buildHandShapes(bool isLeft, const HandFrameInput& handInput, std::vector<RuntimeShape>& outShapes)
        {
            outShapes.clear();
            if (handInput.disabled) {
                return;
            }

            std::uint32_t ordinal = 0;
            addCapsule(outShapes,
                handInput.rawHandWorld.translate,
                handInput.grabAnchorWorld,
                2.6f,
                ContactKind::HandHand,
                makeShapeId(isLeft, ordinal++),
                isLeft ? ShapeOwnerSide::Left : ShapeOwnerSide::Right);

            root_flattened_finger_skeleton_runtime::Snapshot snapshot{};
            if (!root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, snapshot)) {
                return;
            }

            for (std::size_t finger = 0; finger < snapshot.fingers.size(); ++finger) {
                const auto& chain = snapshot.fingers[finger];
                if (!chain.valid) {
                    continue;
                }

                addCapsule(outShapes,
                    chain.points[0],
                    chain.points[1],
                    fingerRadius(0),
                    ContactKind::HandHand,
                    makeShapeId(isLeft, ordinal++),
                    isLeft ? ShapeOwnerSide::Left : ShapeOwnerSide::Right);
                addCapsule(outShapes,
                    chain.points[1],
                    chain.points[2],
                    fingerRadius(1),
                    ContactKind::HandHand,
                    makeShapeId(isLeft, ordinal++),
                    isLeft ? ShapeOwnerSide::Left : ShapeOwnerSide::Right);

                const RE::NiPoint3 tipDirection = soft_contact_math::normalizeOr(
                    soft_contact_math::sub(chain.points[2], chain.points[1]),
                    soft_contact_math::sub(chain.points[2], chain.points[0]));
                const float tipLength = std::max(1.4f, soft_contact_math::length(soft_contact_math::sub(chain.points[2], chain.points[1])) * 0.55f);
                addCapsule(outShapes,
                    chain.points[2],
                    soft_contact_math::add(chain.points[2], soft_contact_math::mul(tipDirection, tipLength)),
                    fingerRadius(2),
                    ContactKind::HandHand,
                    makeShapeId(isLeft, ordinal++),
                    isLeft ? ShapeOwnerSide::Left : ShapeOwnerSide::Right);
            }
        }

        void addWorldContactProbe(
            std::array<WorldContactProbe, SoftContactRuntime::kMaxWorldContactProbesPerHand>& outProbes,
            std::uint32_t& outCount,
            const RE::NiPoint3& position,
            float radius,
            bool isLeft)
        {
            if (outCount >= outProbes.size() || !soft_contact_math::isFinite(position) || !std::isfinite(radius) || radius <= 0.0f) {
                return;
            }

            auto& probe = outProbes[outCount];
            probe.valid = true;
            probe.position = position;
            probe.radius = radius;
            probe.id = makeWorldProbeId(isLeft, outCount);
            probe.stateIndex = outCount;
            ++outCount;
        }

        void buildWorldContactProbes(
            bool isLeft,
            const HandFrameInput& handInput,
            std::array<WorldContactProbe, SoftContactRuntime::kMaxWorldContactProbesPerHand>& outProbes,
            std::uint32_t& outCount)
        {
            outProbes = {};
            outCount = 0;
            if (handInput.disabled) {
                return;
            }

            addWorldContactProbe(outProbes, outCount, handInput.rawHandWorld.translate, 3.2f, isLeft);
            addWorldContactProbe(outProbes, outCount, handInput.grabAnchorWorld, 2.4f, isLeft);

            root_flattened_finger_skeleton_runtime::Snapshot snapshot{};
            if (!root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, snapshot)) {
                return;
            }

            for (const auto& chain : snapshot.fingers) {
                if (!chain.valid) {
                    continue;
                }
                addWorldContactProbe(outProbes, outCount, chain.points[2], 1.15f, isLeft);
            }
        }

        void buildBodyShapes(const DirectSkeletonBoneSnapshot& snapshot, std::vector<RuntimeShape>& outShapes)
        {
            outShapes.clear();
            if (!snapshot.valid) {
                return;
            }

            const auto bones = makeBoneMap(snapshot);
            const auto descriptors = skeleton_bone_debug_math::colliderDescriptors(
                skeleton_bone_debug_math::colliderVariantForPowerArmor(snapshot.inPowerArmor));
            std::uint32_t ordinal = 0;
            for (const auto& descriptor : descriptors) {
                if (!descriptor.enabled ||
                    descriptor.role == skeleton_bone_debug_math::BoneColliderRole::FingerSegment ||
                    descriptor.role == skeleton_bone_debug_math::BoneColliderRole::HandSegment ||
                    descriptor.endpointMode != skeleton_bone_debug_math::BoneColliderEndpointMode::ChildBone) {
                    continue;
                }

                RE::NiPoint3 start{};
                RE::NiPoint3 end{};
                if (!findBonePosition(bones, descriptor.startBone, start) || !findBonePosition(bones, descriptor.endBone, end)) {
                    continue;
                }

                const bool leftArm = isSameSideArmDescriptor(descriptor, true);
                const bool rightArm = isSameSideArmDescriptor(descriptor, false);
                const auto side = leftArm ? ShapeOwnerSide::Left : rightArm ? ShapeOwnerSide::Right : ShapeOwnerSide::Neutral;
                addCapsule(outShapes,
                    start,
                    end,
                    descriptor.radiusGameUnits,
                    ContactKind::Body,
                    0x3000u + ordinal++,
                    side,
                    leftArm || rightArm);
            }
        }

        bool shouldSkipBodyTargetForHand(const RuntimeShape& bodyShape, bool handIsLeft)
        {
            if (!bodyShape.sameSideArm) {
                return false;
            }

            return bodyShape.ownerSide == (handIsLeft ? ShapeOwnerSide::Left : ShapeOwnerSide::Right);
        }

        int candidatePriority(ContactKind kind)
        {
            switch (kind) {
            case ContactKind::WorldStatic:
                return 20;
            case ContactKind::HandHand:
            case ContactKind::WeaponHand:
            case ContactKind::Body:
                return 10;
            default:
                return 0;
            }
        }

        int candidateSourcePriority(CandidateSource source)
        {
            switch (source) {
            case CandidateSource::NativeWorld:
                return 30;
            case CandidateSource::CachedWorldPlane:
                return 20;
            case CandidateSource::QueryWorld:
                return 10;
            case CandidateSource::Shape:
            default:
                return 0;
            }
        }

        float maxCorrectionForContactKind(ContactKind kind);
        float responseScaleForCandidate(const Candidate& candidate);
        RE::NiPoint3 correctionForCandidate(const Candidate& candidate);

        float candidateResponseStrength(const Candidate& candidate)
        {
            return soft_contact_math::length(correctionForCandidate(candidate));
        }

        bool candidateResponseWinsTie(const Candidate& best, const Candidate& candidate)
        {
            return soft_contact_math::preferStrongerContactResponse(
                candidateResponseStrength(candidate),
                candidate.contact.penetration,
                candidateResponseStrength(best),
                best.contact.penetration);
        }

        void keepStrongerCandidate(Candidate& best, const Candidate& candidate)
        {
            if (!candidate.valid) {
                return;
            }

            const int bestPriority = best.valid ? candidatePriority(best.kind) : -1;
            const int candidatePriorityValue = candidatePriority(candidate.kind);
            const int bestSourcePriority = best.valid ? candidateSourcePriority(best.source) : -1;
            const int candidateSourcePriorityValue = candidateSourcePriority(candidate.source);
            if (!best.valid ||
                candidatePriorityValue > bestPriority ||
                (candidatePriorityValue == bestPriority &&
                    (candidateSourcePriorityValue > bestSourcePriority ||
                        (candidateSourcePriorityValue == bestSourcePriority && candidateResponseWinsTie(best, candidate))))) {
                best = candidate;
            }
        }

        Candidate solveShapeAgainstShapes(const RuntimeShape& movableShape,
            const std::vector<RuntimeShape>& targets,
            const RE::NiPoint3& fallbackNormal,
            ContactKind kind,
            float radiusPadding,
            bool handIsLeft,
            bool skipSameSideBodyArm)
        {
            Candidate best{};
            for (const auto& target : targets) {
                if (!target.capsule.valid) {
                    continue;
                }
                if (skipSameSideBodyArm && shouldSkipBodyTargetForHand(target, handIsLeft)) {
                    continue;
                }

                const auto contact = soft_contact_math::solveCapsulePair(movableShape.capsule, target.capsule, fallbackNormal, radiusPadding);
                if (!contact.active) {
                    continue;
                }

                keepStrongerCandidate(best, Candidate{
                    .valid = true,
                    .kind = kind,
                    .contact = contact,
                });
            }
            return best;
        }

        Candidate solveShapeAgainstWeapon(const RuntimeShape& movableShape,
            const WeaponCollision& weaponCollision,
            RE::NiAVObject* weaponNode,
            const RE::NiPoint3& fallbackNormal,
            float radiusPadding)
        {
            Candidate best{};
            if (!weaponNode || !movableShape.capsule.valid || !weaponCollision.hasWeaponBody()) {
                return best;
            }

            WeaponSoftContactResult contact{};
            if (weaponCollision.tryFindSoftContactForCapsule(
                    weaponNode,
                    movableShape.capsule.start,
                    movableShape.capsule.end,
                    movableShape.capsule.radius,
                    radiusPadding,
                    fallbackNormal,
                    contact) &&
                contact.valid) {
                CapsuleContact softContact{};
                softContact.active = true;
                softContact.movablePoint = contact.movablePointWorld;
                softContact.targetPoint = contact.weaponPointWorld;
                softContact.normal = soft_contact_math::normalizeOr(contact.normalWorld, fallbackNormal);
                softContact.distance = contact.distanceGame;
                softContact.penetration = contact.penetrationGame;
                softContact.movableId = movableShape.capsule.id;
                softContact.targetId = contact.bodyId;
                keepStrongerCandidate(best, Candidate{
                    .valid = true,
                    .kind = ContactKind::WeaponHand,
                    .contact = softContact,
                });
            }

            return best;
        }

        void clearWorldContactState(auto& handState)
        {
            for (auto& probe : handState.worldProbes) {
                probe = {};
            }
            handState.cachedWorldPlane = {};
            handState.worldHaptic = {};
        }

        float worldContactPadding()
        {
            return soft_contact_math::sanitizeNonNegative(
                g_rockConfig.rockSoftContactWorldContactPaddingGameUnits,
                kDefaultWorldContactPaddingGameUnits);
        }

        float worldQueryRadiusPadding(float contactPadding)
        {
            return soft_contact_math::effectiveQueryPadding(
                g_rockConfig.rockSoftContactWorldRadiusPaddingGameUnits,
                contactPadding,
                1.5f,
                kDefaultWorldContactPaddingGameUnits);
        }

        float worldPostReleaseReentryMinApproachDistance()
        {
            return soft_contact_math::sanitizeNonNegative(
                g_rockConfig.rockSoftContactWorldPostReleaseReentryMinApproachDistanceGameUnits,
                kDefaultWorldPostReleaseReentryMinApproachDistanceGameUnits);
        }

        float worldCachedPlaneMaxTangentDrift()
        {
            return soft_contact_math::sanitizePositive(
                g_rockConfig.rockSoftContactWorldCachedPlaneMaxTangentDriftGameUnits,
                kDefaultWorldCachedPlaneMaxTangentDriftGameUnits);
        }

        float weaponHandRadiusPadding()
        {
            return soft_contact_math::sanitizeNonNegative(
                g_rockConfig.rockSoftContactWeaponHandRadiusPaddingGameUnits,
                kDefaultWeaponHandRadiusPaddingGameUnits);
        }

        float weaponHandCorrectionScale()
        {
            return std::clamp(
                std::isfinite(g_rockConfig.rockSoftContactWeaponHandCorrectionScale) ?
                    g_rockConfig.rockSoftContactWeaponHandCorrectionScale :
                    kDefaultWeaponHandCorrectionScale,
                0.0f,
                1.0f);
        }

        float weaponHandHardStopPenetration()
        {
            return soft_contact_math::sanitizePositive(
                g_rockConfig.rockSoftContactWeaponHandHardStopPenetrationGameUnits,
                kDefaultWeaponHandHardStopPenetrationGameUnits);
        }

        float maxCorrectionForContactKind(ContactKind kind)
        {
            switch (kind) {
            case ContactKind::WorldStatic:
                return soft_contact_math::sanitizePositive(g_rockConfig.rockSoftContactWorldMaxCorrectionGameUnits, 18.0f);
            case ContactKind::WeaponHand:
                return soft_contact_math::sanitizePositive(
                    g_rockConfig.rockSoftContactWeaponHandMaxCorrectionGameUnits,
                    kDefaultWeaponHandMaxCorrectionGameUnits);
            default:
                return soft_contact_math::sanitizePositive(g_rockConfig.rockSoftContactMaxCorrectionGameUnits, 7.0f);
            }
        }

        float responseScaleForCandidate(const Candidate& candidate)
        {
            if (candidate.kind != ContactKind::WeaponHand) {
                return candidate.contact.penetration > 0.0f ? 1.0f : 0.0f;
            }

            return soft_contact_math::compliantHardStopResponseScale(
                candidate.contact.penetration,
                weaponHandCorrectionScale(),
                weaponHandHardStopPenetration());
        }

        RE::NiPoint3 correctionForCandidate(const Candidate& candidate)
        {
            /*
             * Weapon contact is tuned separately because the weapon already has
             * visual/gameplay authority in the hand. ROCK keeps wall/body contact
             * as a hard current-frame projection, but weapon-hand overlap uses a
             * compliant current-frame response that ramps to a hard stop only for
             * deep penetration.
             */
            const float maxCorrection = maxCorrectionForContactKind(candidate.kind);
            if (candidate.kind == ContactKind::WeaponHand) {
                const RE::NiPoint3 hardStopCorrection = soft_contact_math::projectTrackedMagnetCorrection(
                    candidate.contact.normal,
                    candidate.contact.penetration,
                    maxCorrection);
                return soft_contact_math::mul(hardStopCorrection, responseScaleForCandidate(candidate));
            }

            return soft_contact_math::projectTrackedMagnetCorrection(
                candidate.contact.normal,
                candidate.contact.penetration,
                maxCorrection);
        }

        std::uint32_t resolveWorldHitFilterInfo(RE::hknpWorld* world, const RE::hknpCollisionResult& hit)
        {
            const std::uint32_t shapeFilterInfo = hit.hitBodyInfo.m_shapeCollisionFilterInfo.storage;
            if (soft_contact_world_policy::acceptsWorldSurfaceFilterInfo(shapeFilterInfo)) {
                return shapeFilterInfo;
            }

            const auto bodyId = hit.hitBodyInfo.m_bodyId;
            if (bodyId.value == soft_contact_world_policy::kInvalidWorldBodyId || !bodySlotLooksReadable(world, bodyId)) {
                return shapeFilterInfo;
            }

            auto* body = havok_runtime::getBody(world, bodyId);
            return body ? body->collisionFilterInfo : shapeFilterInfo;
        }

        const char* candidateSourceName(CandidateSource source)
        {
            switch (source) {
            case CandidateSource::QueryWorld:
                return "QueryWorld";
            case CandidateSource::CachedWorldPlane:
                return "CachedWorldPlane";
            case CandidateSource::NativeWorld:
                return "NativeWorld";
            default:
                return "Shape";
            }
        }

        Candidate makeWorldStaticCandidate(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const RE::hknpCollisionResult& hit,
            const WorldContactProbe& probe,
            const RE::NiPoint3& previousProbePosition,
            const RE::NiPoint3& fallbackNormal,
            float radiusPadding,
            float skin,
            float deltaSeconds)
        {
            Candidate candidate{};
            const auto hitBodyId = hit.hitBodyInfo.m_bodyId;
            if (hitBodyId.value == soft_contact_world_policy::kInvalidWorldBodyId) {
                return candidate;
            }

            const RE::NiPoint3 hitPoint = hkVectorToNiPoint(hit.position);
            const RE::NiPoint3 sweepDelta = soft_contact_math::sub(probe.position, previousProbePosition);
            const RE::NiPoint3 rawHitNormal(hit.normal.x, hit.normal.y, hit.normal.z);
            const RE::NiPoint3 castFallbackNormal = soft_contact_math::normalizeOr(soft_contact_math::negate(sweepDelta), fallbackNormal);
            const RE::NiPoint3 surfaceNormal =
                soft_contact_math::orientNormalTowardPoint(hitPoint, rawHitNormal, probe.position, castFallbackNormal);

            const float effectiveProbeRadius =
                probe.radius + soft_contact_math::sanitizeNonNegative(radiusPadding, 0.0f);
            auto contact = soft_contact_math::solvePointPlaneContact(
                probe.position,
                hitPoint,
                surfaceNormal,
                effectiveProbeRadius,
                skin,
                probe.id,
                hitBodyId.value);
            if (!contact.active) {
                return candidate;
            }

            const float dt = std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 1.0f / 240.0f, 0.1f);
            const RE::NiPoint3 velocity = soft_contact_math::mul(sweepDelta, 1.0f / dt);
            candidate.valid = true;
            candidate.kind = ContactKind::WorldStatic;
            candidate.source = CandidateSource::QueryWorld;
            candidate.contact = contact;
            candidate.approachSpeedGameUnits = std::max(0.0f, -soft_contact_math::dot(velocity, contact.normal));
            candidate.targetIdentity = contact_target_identity::resolveContactTarget(
                bhkWorld,
                hknpWorld,
                hitBodyId.value,
                contact_evidence::NativeContactEndpointKind::WorldSurface,
                &hitPoint,
                &surfaceNormal,
                contactTargetResolutionOptions());
            return candidate;
        }

        const WorldContactProbe* findClosestWorldProbe(
            const std::array<WorldContactProbe, SoftContactRuntime::kMaxWorldContactProbesPerHand>& probes,
            std::uint32_t probeCount,
            const RE::NiPoint3& point)
        {
            const WorldContactProbe* best = nullptr;
            float bestDistanceSquared = std::numeric_limits<float>::max();
            if (!soft_contact_math::isFinite(point)) {
                return nullptr;
            }

            for (std::uint32_t i = 0; i < probeCount && i < probes.size(); ++i) {
                const auto& probe = probes[i];
                if (!probe.valid) {
                    continue;
                }
                const float distanceSquared = soft_contact_math::lengthSquared(soft_contact_math::sub(probe.position, point));
                if (!std::isfinite(distanceSquared) || distanceSquared >= bestDistanceSquared) {
                    continue;
                }
                best = &probe;
                bestDistanceSquared = distanceSquared;
            }
            return best;
        }

        Candidate makeNativeWorldStaticCandidate(
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const contact_evidence::NativeContactEvidenceRecord& evidence,
            const WorldContactProbe& probe,
            auto& handState,
            const RE::NiPoint3& fallbackNormal,
            float contactPadding,
            float skin,
            float deltaSeconds)
        {
            Candidate candidate{};
            if (evidence.quality != contact_evidence::NativeContactQuality::RawPoint ||
                !soft_contact_math::isFinite(evidence.contactPointGame) ||
                !soft_contact_math::isFinite(evidence.contactNormalGame)) {
                return candidate;
            }

            const RE::NiPoint3 surfaceNormal = soft_contact_math::orientNormalTowardPoint(
                evidence.contactPointGame,
                evidence.contactNormalGame,
                probe.position,
                fallbackNormal);
            const float effectiveProbeRadius = probe.radius + soft_contact_math::sanitizeNonNegative(contactPadding, 0.0f);
            auto contact = soft_contact_math::solvePointPlaneContact(
                probe.position,
                evidence.contactPointGame,
                surfaceNormal,
                effectiveProbeRadius,
                skin,
                probe.id,
                evidence.targetBodyId);
            if (!contact.active) {
                return candidate;
            }

            float approachSpeed = 0.0f;
            if (soft_contact_math::isFinite(evidence.sourceVelocityGame)) {
                approachSpeed = std::max(0.0f, -soft_contact_math::dot(evidence.sourceVelocityGame, contact.normal));
            }
            if (approachSpeed <= 0.0f && probe.stateIndex < handState.worldProbes.size() && handState.worldProbes[probe.stateIndex].valid) {
                const float dt = std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 1.0f / 240.0f, 0.1f);
                const auto sweepDelta = soft_contact_math::sub(probe.position, handState.worldProbes[probe.stateIndex].previous);
                const auto velocity = soft_contact_math::mul(sweepDelta, 1.0f / dt);
                approachSpeed = std::max(0.0f, -soft_contact_math::dot(velocity, contact.normal));
            }

            candidate.valid = true;
            candidate.kind = ContactKind::WorldStatic;
            candidate.source = CandidateSource::NativeWorld;
            candidate.contact = contact;
            candidate.approachSpeedGameUnits = approachSpeed;
            candidate.targetIdentity = contact_target_identity::resolveContactTarget(
                bhkWorld,
                hknpWorld,
                evidence.targetBodyId,
                evidence.targetKind,
                &evidence.contactPointGame,
                &surfaceNormal,
                contactTargetResolutionOptions());
            if (candidate.targetIdentity.filterInfo == contact_target_identity::kUnknownFilterInfo &&
                evidence.targetFilterInfo != contact_evidence::kUnknownFilterInfo) {
                candidate.targetIdentity.filterInfo = evidence.targetFilterInfo;
                candidate.targetIdentity.layer = evidence.targetLayer;
            }
            return candidate;
        }

        Candidate solveNativeWorldStaticContact(
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const contact_evidence::NativeContactEvidenceSnapshot& evidenceSnapshot,
            auto& handState,
            bool isLeft,
            const std::array<WorldContactProbe, SoftContactRuntime::kMaxWorldContactProbesPerHand>& probes,
            std::uint32_t probeCount,
            const RE::NiPoint3& fallbackNormal,
            float contactPadding,
            float skin,
            float deltaSeconds)
        {
            Candidate best{};
            using contact_evidence::NativeContactEndpointKind;
            for (std::uint32_t i = 0; i < evidenceSnapshot.count && i < evidenceSnapshot.records.size(); ++i) {
                const auto& evidence = evidenceSnapshot.records[i];
                if (!evidence.valid ||
                    !contact_evidence::isFrameFresh(evidenceSnapshot.currentFrame, evidence.frame, kNativeWorldContactMaxAgeFrames) ||
                    evidence.targetKind != NativeContactEndpointKind::WorldSurface ||
                    evidence.quality != contact_evidence::NativeContactQuality::RawPoint ||
                    evidence.sourceIsLeft != isLeft) {
                    continue;
                }

                const bool sourceIsExpectedHand =
                    (!isLeft && evidence.sourceKind == NativeContactEndpointKind::RightHand) ||
                    (isLeft && evidence.sourceKind == NativeContactEndpointKind::LeftHand);
                if (!sourceIsExpectedHand) {
                    continue;
                }

                const auto* probe = findClosestWorldProbe(probes, probeCount, evidence.contactPointGame);
                if (!probe) {
                    continue;
                }

                keepStrongerCandidate(best,
                    makeNativeWorldStaticCandidate(bhkWorld, hknpWorld, evidence, *probe, handState, fallbackNormal, contactPadding, skin, deltaSeconds));
            }
            return best;
        }

        CachedWorldPlaneResult solveCachedWorldPlaneContact(auto& handState,
            const std::array<WorldContactProbe, SoftContactRuntime::kMaxWorldContactProbesPerHand>& probes,
            std::uint32_t probeCount,
            float contactPadding,
            float skin,
            float maxTangentDrift,
            float deltaSeconds)
        {
            CachedWorldPlaneResult result{};
            if (!handState.cachedWorldPlane.active) {
                return result;
            }
            result.hadCachedPlane = true;
            result.normal = handState.cachedWorldPlane.normal;

            const WorldContactProbe* probe = nullptr;
            for (std::uint32_t i = 0; i < probeCount && i < probes.size(); ++i) {
                if (probes[i].valid && probes[i].id == handState.cachedWorldPlane.probeId) {
                    probe = &probes[i];
                    break;
                }
            }

            if (!probe) {
                handState.cachedWorldPlane = {};
                result.leftContact = true;
                return result;
            }

            const float effectiveProbeRadius = probe->radius + soft_contact_math::sanitizeNonNegative(contactPadding, 0.0f);
            auto contact = soft_contact_math::solvePointPlaneContact(
                probe->position,
                handState.cachedWorldPlane.surfacePoint,
                handState.cachedWorldPlane.normal,
                effectiveProbeRadius,
                skin,
                probe->id,
                handState.cachedWorldPlane.bodyId);

            /*
             * This is cached evidence, not a lock. The current tracked probe
             * must still be penetrating the cached plane this frame; otherwise
             * the external visual authority is released immediately.
             */
            if (!contact.active) {
                handState.cachedWorldPlane = {};
                result.leftContact = true;
                return result;
            }

            if (!soft_contact_math::withinTangentDriftLimit(
                    contact.targetPoint,
                    handState.cachedWorldPlane.surfacePoint,
                    handState.cachedWorldPlane.normal,
                    maxTangentDrift)) {
                handState.cachedWorldPlane = {};
                result.leftContact = true;
                return result;
            }

            float approachSpeed = handState.cachedWorldPlane.approachSpeedGameUnits;
            if (probe->stateIndex < handState.worldProbes.size() && handState.worldProbes[probe->stateIndex].valid) {
                const float dt = std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 1.0f / 240.0f, 0.1f);
                const auto sweepDelta = soft_contact_math::sub(probe->position, handState.worldProbes[probe->stateIndex].previous);
                const auto velocity = soft_contact_math::mul(sweepDelta, 1.0f / dt);
                approachSpeed = std::max(0.0f, -soft_contact_math::dot(velocity, contact.normal));
            }

            result.candidate.valid = true;
            result.candidate.kind = ContactKind::WorldStatic;
            result.candidate.source = CandidateSource::CachedWorldPlane;
            result.candidate.contact = contact;
            result.candidate.approachSpeedGameUnits = approachSpeed;
            result.candidate.targetIdentity = handState.cachedWorldPlane.targetIdentity;
            return result;
        }

        Candidate runWorldProbeCast(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* world,
            const WorldContactProbe& probe,
            const RE::NiPoint3& start,
            const RE::NiPoint3& displacement,
            float distance,
            const RE::NiPoint3& previousPosition,
            const RE::NiPoint3& fallbackNormal,
            float queryRadiusPadding,
            float contactPadding,
            float skin,
            float deltaSeconds)
        {
            Candidate best{};
            if (!world || !probe.valid || !std::isfinite(distance) || distance <= 0.001f) {
                return best;
            }

            RE::hknpAllHitsCollector collector;
            if (!physics_shape_cast::castSelectionSphere(
                    world,
                    physics_shape_cast::SphereCastInput{
                        .startGame = start,
                        .directionGame = displacement,
                        .distanceGame = distance,
                        .radiusGame = probe.radius + queryRadiusPadding,
                        .collisionFilterInfo = g_rockConfig.rockSelectionShapeCastFilterInfo },
                    collector,
                    nullptr)) {
                return best;
            }

            auto* hits = collector.hits._data;
            const int hitCount = collector.hits._size;
            for (int hitIndex = 0; hitIndex < hitCount; ++hitIndex) {
                const auto& hit = hits[hitIndex];
                const std::uint32_t filterInfo = resolveWorldHitFilterInfo(world, hit);
                if (!soft_contact_world_policy::acceptsWorldSurfaceFilterInfo(filterInfo)) {
                    continue;
                }

                keepStrongerCandidate(best,
                    makeWorldStaticCandidate(bhkWorld, world, hit, probe, previousPosition, fallbackNormal, contactPadding, skin, deltaSeconds));
            }

            return best;
        }

        void storeWorldProbePositions(auto& handState,
            const std::array<WorldContactProbe, SoftContactRuntime::kMaxWorldContactProbesPerHand>& probes,
            std::uint32_t probeCount)
        {
            for (std::size_t i = 0; i < handState.worldProbes.size(); ++i) {
                if (i < probeCount && probes[i].valid) {
                    handState.worldProbes[i].valid = true;
                    handState.worldProbes[i].previous = probes[i].position;
                } else {
                    handState.worldProbes[i] = {};
                }
            }
        }

        Candidate solveWorldStaticContact(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* world,
            const contact_evidence::NativeContactEvidenceSnapshot& nativeContactEvidence,
            auto& handState,
            bool isLeft,
            const HandFrameInput& handInput,
            const RE::NiPoint3& fallbackNormal,
            float deltaSeconds)
        {
            Candidate best{};
            if (!g_rockConfig.rockSoftContactWorldEnabled || !world || handInput.disabled) {
                clearWorldContactState(handState);
                return best;
            }

            std::array<WorldContactProbe, SoftContactRuntime::kMaxWorldContactProbesPerHand> probes{};
            std::uint32_t probeCount = 0;
            buildWorldContactProbes(isLeft, handInput, probes, probeCount);
            if (probeCount == 0) {
                clearWorldContactState(handState);
                return best;
            }

            const float contactPadding = worldContactPadding();
            const float queryRadiusPadding = worldQueryRadiusPadding(contactPadding);
            const float skin =
                soft_contact_math::sanitizeNonNegative(g_rockConfig.rockSoftContactWorldSkinGameUnits, 0.5f);
            const float reentryMinApproachDistance = worldPostReleaseReentryMinApproachDistance();
            const float maxTangentDrift = worldCachedPlaneMaxTangentDrift();
            const float dt = std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 0.0f, 0.1f);

            const auto cachedPlaneContact = solveCachedWorldPlaneContact(
                handState,
                probes,
                probeCount,
                contactPadding,
                skin,
                maxTangentDrift,
                deltaSeconds);
            keepStrongerCandidate(best, cachedPlaneContact.candidate);

            keepStrongerCandidate(best,
                solveNativeWorldStaticContact(
                    bhkWorld,
                    world,
                    nativeContactEvidence,
                    handState,
                    isLeft,
                    probes,
                    probeCount,
                    fallbackNormal,
                    contactPadding,
                    skin,
                    deltaSeconds));

            const bool hasAuthoritativeWorldCandidate =
                best.valid &&
                best.kind == ContactKind::WorldStatic &&
                (best.source == CandidateSource::NativeWorld || best.source == CandidateSource::CachedWorldPlane);
            const bool releasedCachedPlaneThisFrame =
                cachedPlaneContact.hadCachedPlane && cachedPlaneContact.leftContact;

            if (!hasAuthoritativeWorldCandidate) {
                for (std::uint32_t i = 0; i < probeCount && i < probes.size(); ++i) {
                    const auto& probe = probes[i];
                    if (!probe.valid || probe.stateIndex >= handState.worldProbes.size()) {
                        continue;
                    }

                    auto& previousState = handState.worldProbes[probe.stateIndex];
                    previousState.restQueryCooldownSeconds =
                        std::max(0.0f, previousState.restQueryCooldownSeconds - dt);

                    const RE::NiPoint3 sweepDelta = previousState.valid ?
                                                      soft_contact_math::sub(probe.position, previousState.previous) :
                                                      RE::NiPoint3{};
                    const float sweepDistance = previousState.valid ? soft_contact_math::length(sweepDelta) : 0.0f;
                    const bool canSweep =
                        previousState.valid &&
                        std::isfinite(sweepDistance) &&
                        sweepDistance > kWorldProbeMinSweepDistanceGameUnits;
                    const bool allowPostReleaseSweep =
                        soft_contact_math::shouldAllowPostReleaseReentrySweep(
                            releasedCachedPlaneThisFrame,
                            sweepDelta,
                            cachedPlaneContact.normal,
                            reentryMinApproachDistance);

                    if (canSweep && allowPostReleaseSweep) {
                        keepStrongerCandidate(best,
                            runWorldProbeCast(bhkWorld,
                                world,
                                probe,
                                previousState.previous,
                                sweepDelta,
                                sweepDistance,
                                previousState.previous,
                                fallbackNormal,
                                queryRadiusPadding,
                                contactPadding,
                                skin,
                                deltaSeconds));
                    }

                    if (canSweep || releasedCachedPlaneThisFrame || previousState.restQueryCooldownSeconds > 0.0f) {
                        continue;
                    }

                    previousState.restQueryCooldownSeconds = kWorldProbeRestQueryCooldownSeconds;
                    const RE::NiPoint3 restDirection = soft_contact_math::normalizeOr(fallbackNormal, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
                    const RE::NiPoint3 previousPosition = previousState.valid ? previousState.previous : probe.position;
                    keepStrongerCandidate(best,
                        runWorldProbeCast(bhkWorld,
                            world,
                            probe,
                            probe.position,
                            restDirection,
                            kWorldProbeRestQueryDistanceGameUnits,
                            previousPosition,
                            fallbackNormal,
                            queryRadiusPadding,
                            contactPadding,
                            skin,
                            deltaSeconds));
                }
            }

            storeWorldProbePositions(handState, probes, probeCount);

            if (best.valid && best.kind == ContactKind::WorldStatic) {
                handState.cachedWorldPlane.active = true;
                handState.cachedWorldPlane.bodyId = best.contact.targetId;
                handState.cachedWorldPlane.probeId = best.contact.movableId;
                handState.cachedWorldPlane.surfacePoint = best.contact.targetPoint;
                handState.cachedWorldPlane.normal = best.contact.normal;
                handState.cachedWorldPlane.approachSpeedGameUnits = best.approachSpeedGameUnits;
                handState.cachedWorldPlane.targetIdentity = best.targetIdentity;
            }

            return best;
        }

        void updateWorldContactHaptics(
            auto& handState,
            bool isLeft,
            bool active,
            float approachSpeedGameUnits,
            float deltaSeconds)
        {
            soft_contact_math::HapticEdgeConfig config{};
            config.enabled = g_rockConfig.rockSoftContactWorldHapticsEnabled;
            config.baseIntensity = g_rockConfig.rockSoftContactWorldHapticBaseIntensity;
            config.maxIntensity = g_rockConfig.rockSoftContactWorldHapticMaxIntensity;
            config.speedScale = g_rockConfig.rockSoftContactWorldHapticSpeedScale;
            config.minApproachSpeed = g_rockConfig.rockSoftContactWorldHapticMinApproachSpeedGameUnits;
            config.cooldownSeconds = g_rockConfig.rockSoftContactWorldHapticCooldownSeconds;

            const auto decision = soft_contact_math::updateHapticEdge(handState.worldHaptic, active, approachSpeedGameUnits, deltaSeconds, config);
            const float duration = std::clamp(
                std::isfinite(g_rockConfig.rockSoftContactWorldHapticDurationSeconds) ? g_rockConfig.rockSoftContactWorldHapticDurationSeconds : 0.035f,
                0.0f,
                0.2f);
            if (decision.fire && duration > 0.0f && decision.intensity > 0.0f) {
                f4cf::vrcf::VRControllers.triggerHaptic(
                    isLeft ? f4cf::vrcf::Hand::Left : f4cf::vrcf::Hand::Right,
                    duration,
                    decision.intensity);
            }
        }

        void logContactTargetIdentity(bool isLeft, const Candidate& candidate)
        {
            if (!g_rockConfig.rockDebugContactTargetIdentityLogging || candidate.kind != ContactKind::WorldStatic) {
                return;
            }

            const auto& identity = candidate.targetIdentity;
            const auto sampleMilliseconds = std::max(1, g_rockConfig.rockDebugContactTargetIdentitySampleMilliseconds);
            const auto bodyId = contact_evidence::isValidBodyId(identity.bodyId) ? identity.bodyId : candidate.contact.targetId;
            const auto layer = identity.layer;
            const auto filterInfo = identity.filterInfo;
            const auto motionIndex = identity.motionIndex;
            const auto status = identity.status;
            const auto surfaceHint = identity.surfaceHint;
            const char* formType = identity.formType.empty() ? "???" : identity.formType.c_str();
            const char* displayName = identity.displayName.empty() ? "(unnamed)" : identity.displayName.c_str();
            const char* refEditorId = identity.refEditorId.empty() ? "(none)" : identity.refEditorId.c_str();
            const char* baseEditorId = identity.baseEditorId.empty() ? "(none)" : identity.baseEditorId.c_str();
            const auto& point = candidate.contact.targetPoint;
            const auto& normal = candidate.contact.normal;

            ROCK_LOG_SAMPLE_DEBUG(ContactTarget,
                sampleMilliseconds,
                "Contact target identity: hand={} source={} status={} endpoint={} body={} layer={} filter=0x{:08X} motion={} ref={:08X} base={:08X} type={} name='{}' refEditor='{}' baseEditor='{}' surface={} point=({:.2f},{:.2f},{:.2f}) normal=({:.3f},{:.3f},{:.3f})",
                isLeft ? "Left" : "Right",
                candidateSourceName(candidate.source),
                contact_target_identity::resolutionStatusName(status),
                contact_target_identity::endpointKindName(identity.endpointKind),
                bodyId,
                layer,
                filterInfo,
                motionIndex,
                identity.refFormId,
                identity.baseFormId,
                formType,
                displayName,
                refEditorId,
                baseEditorId,
                contact_target_identity::surfaceHintName(surfaceHint),
                point.x,
                point.y,
                point.z,
                normal.x,
                normal.y,
                normal.z);
        }

        void addDebugContact(SoftContactDebugSnapshot& snapshot, bool isLeft, ContactState state, const Candidate& candidate, const RE::NiPoint3& correction)
        {
            if (!candidate.valid || snapshot.contactCount >= snapshot.contacts.size()) {
                return;
            }

            auto& entry = snapshot.contacts[snapshot.contactCount++];
            entry.valid = true;
            entry.isLeft = isLeft;
            entry.suppressed = candidate.suppressed;
            entry.kind = candidate.kind;
            entry.state = state;
            entry.point = candidate.contact.movablePoint;
            entry.normalEnd = soft_contact_math::add(candidate.contact.movablePoint, soft_contact_math::mul(candidate.contact.normal, 8.0f));
            entry.correctionEnd = soft_contact_math::add(candidate.contact.movablePoint, correction);
            entry.penetration = candidate.contact.penetration;
            entry.responseScale = responseScaleForCandidate(candidate);
            entry.maxCorrection = maxCorrectionForContactKind(candidate.kind);
            entry.correctionLength = soft_contact_math::length(correction);
            entry.movableId = candidate.contact.movableId;
            entry.targetId = candidate.contact.targetId;
            entry.targetLayer = candidate.targetIdentity.layer;
            entry.targetFilterInfo = candidate.targetIdentity.filterInfo;
            entry.targetRefFormId = candidate.targetIdentity.refFormId;
            entry.targetBaseFormId = candidate.targetIdentity.baseFormId;
            entry.surfaceHint = candidate.targetIdentity.surfaceHint;
        }
    }

    void SoftContactRuntime::reset()
    {
        clearAllHands();
        _bodyReader.resetCache();
        _debugSnapshot = {};
        _wasEnabled = false;
        _logCounter = 0;
    }

    void SoftContactRuntime::clearHandForStrongerOwner(bool isLeft, const char* reason)
    {
        auto& handState = _hands[handIndex(isLeft)];
        const bool hadVisualAuthority =
            handState.externalTransformActive ||
            handState.state != ContactState::Inactive ||
            soft_contact_math::length(handState.correction) > kCorrectionClearDistance;
        clearHand(isLeft);
        _hands[handIndex(isLeft)].state = ContactState::Suppressed;
        if (hadVisualAuthority) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                500,
                "{} soft contact cleared for stronger hand owner ({})",
                isLeft ? "Left" : "Right",
                reason ? reason : "unknown");
        }
    }

    void SoftContactRuntime::clearHand(bool isLeft)
    {
        auto& handState = _hands[handIndex(isLeft)];
        if (auto* api = frik::api::FRIKApi::inst; api && api->clearExternalHandWorldTransform) {
            api->clearExternalHandWorldTransform(softContactTag(isLeft), handFromBool(isLeft));
        }
        handState = {};
    }

    void SoftContactRuntime::clearAllHands()
    {
        clearHand(false);
        clearHand(true);
    }

    bool SoftContactRuntime::getDebugSnapshot(SoftContactDebugSnapshot& outSnapshot) const
    {
        outSnapshot = _debugSnapshot;
        return outSnapshot.contactCount > 0 ||
               outSnapshot.rightState != ContactState::Inactive ||
               outSnapshot.leftState != ContactState::Inactive;
    }

    void SoftContactRuntime::update(const PhysicsFrameContext& frame,
        const Hand& rightHand,
        const Hand& leftHand,
        const WeaponCollision& weaponCollision,
        RE::NiAVObject* weaponNode,
        bool rightHandWeaponEquipped,
        bool leftSupportGripActive,
        const contact_evidence::NativeContactEvidenceSnapshot& nativeContactEvidence)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::SoftContact);

        _debugSnapshot = {};
        _debugSnapshot.rightState = _hands[0].state;
        _debugSnapshot.leftState = _hands[1].state;

        if (!g_rockConfig.rockSoftContactEnabled || !frame.worldReady || frame.menuBlocked) {
            if (_wasEnabled) {
                clearAllHands();
            }
            _debugSnapshot = {};
            _wasEnabled = false;
            return;
        }
        _wasEnabled = true;

        if (++_logCounter >= 360) {
            _logCounter = 0;
            const float loggedWorldContactPadding = worldContactPadding();
            const float loggedWorldQueryPadding = worldQueryRadiusPadding(loggedWorldContactPadding);
            const float loggedMaxCorrection = maxCorrectionForContactKind(ContactKind::Body);
            const float loggedWeaponMaxCorrection = maxCorrectionForContactKind(ContactKind::WeaponHand);
            const float loggedWorldMaxCorrection = maxCorrectionForContactKind(ContactKind::WorldStatic);
            ROCK_LOG_DEBUG(Hand,
                "SoftContact active: handHand={} weaponHand={} body={} world={} worldHaptics={} maxCorrection={:.2f} weaponMaxCorrection={:.2f} weaponRadiusPadding={:.2f} weaponCorrectionScale={:.2f} weaponHardStop={:.2f} worldMaxCorrection={:.2f} worldQueryPadding={:.2f} worldContactPadding={:.2f} worldPostReleaseReentryMinApproach={:.3f} worldCachedPlaneMaxTangentDrift={:.2f} priority={}",
                g_rockConfig.rockSoftContactHandHandEnabled ? "yes" : "no",
                g_rockConfig.rockSoftContactWeaponHandEnabled ? "yes" : "no",
                g_rockConfig.rockSoftContactBodyEnabled ? "yes" : "no",
                g_rockConfig.rockSoftContactWorldEnabled ? "yes" : "no",
                g_rockConfig.rockSoftContactWorldHapticsEnabled ? "yes" : "no",
                loggedMaxCorrection,
                loggedWeaponMaxCorrection,
                weaponHandRadiusPadding(),
                weaponHandCorrectionScale(),
                weaponHandHardStopPenetration(),
                loggedWorldMaxCorrection,
                loggedWorldQueryPadding,
                loggedWorldContactPadding,
                worldPostReleaseReentryMinApproachDistance(),
                worldCachedPlaneMaxTangentDrift(),
                g_rockConfig.rockSoftContactVisualPriority);
        }

        std::vector<RuntimeShape> rightShapes;
        std::vector<RuntimeShape> leftShapes;
        std::vector<RuntimeShape> bodyShapes;
        rightShapes.reserve(24);
        leftShapes.reserve(24);
        bodyShapes.reserve(32);

        buildHandShapes(false, frame.right, rightShapes);
        buildHandShapes(true, frame.left, leftShapes);

        if (g_rockConfig.rockSoftContactBodyEnabled) {
            DirectSkeletonBoneSnapshot bodySnapshot{};
            if (_bodyReader.capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::CoreBodyAndFingers,
                    skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                    bodySnapshot)) {
                buildBodyShapes(bodySnapshot, bodyShapes);
            }
        } else {
            _bodyReader.resetCache();
        }

        auto solveForHand = [&](bool isLeft, const HandFrameInput& handInput, const Hand& hand, const std::vector<RuntimeShape>& movableShapes,
                                const std::vector<RuntimeShape>& oppositeHandShapes) {
            auto& handState = _hands[handIndex(isLeft)];
            /*
             * ROCK disables generated hand contact while a held object or
             * two-handed weapon state owns the hand. Pull/locked-selection are
             * also grab ownership states: they can transition into a capture in
             * the same frame, so stale visual contact must not survive there and
             * poison the raw tracked hand frame used by grab setup.
             */
            const bool ownedByGrabFlow =
                suppressesGeneratedHandContactEvidence(hand.getState());
            const bool ownedByStrongerSystem =
                ownedByGrabFlow ||
                (!isLeft && rightHandWeaponEquipped) ||
                (isLeft && leftSupportGripActive);
            if (ownedByStrongerSystem) {
                clearHand(isLeft);
                handState.state = ContactState::Suppressed;
                return;
            }

            Candidate best{};
            const RE::NiPoint3 fallbackNormal = soft_contact_math::normalizeOr(handInput.palmNormalWorld, RE::NiPoint3(0.0f, 0.0f, 1.0f));
            const float radiusPadding = soft_contact_math::sanitizeNonNegative(g_rockConfig.rockSoftContactRadiusPaddingGameUnits, 0.75f);
            const float weaponRadiusPadding = weaponHandRadiusPadding();

            if (!handInput.disabled) {
                for (const auto& shape : movableShapes) {
                    if (g_rockConfig.rockSoftContactHandHandEnabled) {
                        keepStrongerCandidate(best,
                            solveShapeAgainstShapes(shape, oppositeHandShapes, fallbackNormal, ContactKind::HandHand, radiusPadding, isLeft, false));
                    }
                    if (g_rockConfig.rockSoftContactBodyEnabled) {
                        keepStrongerCandidate(best,
                            solveShapeAgainstShapes(shape, bodyShapes, fallbackNormal, ContactKind::Body, radiusPadding, isLeft, true));
                    }
                    if (g_rockConfig.rockSoftContactWeaponHandEnabled) {
                        keepStrongerCandidate(best,
                            solveShapeAgainstWeapon(shape, weaponCollision, weaponNode, fallbackNormal, weaponRadiusPadding));
                    }
                }

                keepStrongerCandidate(best,
                    solveWorldStaticContact(
                        frame.bhkWorld,
                        frame.hknpWorld,
                        nativeContactEvidence,
                        _hands[handIndex(isLeft)],
                        isLeft,
                        handInput,
                        fallbackNormal,
                        frame.deltaSeconds));
            }

            auto* api = frik::api::FRIKApi::inst;
            if (!api || !api->applyExternalHandWorldTransform || !api->clearExternalHandWorldTransform || handInput.disabled) {
                clearHand(isLeft);
                updateWorldContactHaptics(handState, isLeft, false, 0.0f, frame.deltaSeconds);
                return;
            }

            if (best.valid) {
                const bool worldContact = best.kind == ContactKind::WorldStatic;
                if (worldContact) {
                    logContactTargetIdentity(isLeft, best);
                }
                updateWorldContactHaptics(handState, isLeft, worldContact, best.approachSpeedGameUnits, frame.deltaSeconds);
                handState.correction = correctionForCandidate(best);
                handState.lastContactKind = best.kind;
                handState.state = best.contact.penetration >= 0.5f ? ContactState::Penetrating : ContactState::Touching;
                if (best.kind == ContactKind::WeaponHand) {
                    ROCK_LOG_SAMPLE_DEBUG(Hand,
                        500,
                        "{} weapon-hand soft contact: penetration={:.3f} correction={:.3f} responseScale={:.3f} maxCorrection={:.3f} radiusPadding={:.3f}",
                        isLeft ? "Left" : "Right",
                        best.contact.penetration,
                        soft_contact_math::length(handState.correction),
                        responseScaleForCandidate(best),
                        maxCorrectionForContactKind(best.kind),
                        weaponRadiusPadding);
                }

                RE::NiTransform target = handInput.rawHandWorld;
                target.translate = soft_contact_math::add(target.translate, handState.correction);
                if (api->applyExternalHandWorldTransform(softContactTag(isLeft), handFromBool(isLeft), target, g_rockConfig.rockSoftContactVisualPriority)) {
                    handState.externalTransformActive = true;
                } else {
                    ROCK_LOG_SAMPLE_WARN(Hand,
                        1000,
                        "{} soft contact external transform apply failed; clearing visual contact state",
                        isLeft ? "Left" : "Right");
                    clearHand(isLeft);
                    handState.state = ContactState::Inactive;
                    return;
                }
                addDebugContact(_debugSnapshot, isLeft, handState.state, best, handState.correction);
                return;
            }

            updateWorldContactHaptics(handState, isLeft, false, 0.0f, frame.deltaSeconds);
            if (handState.externalTransformActive || soft_contact_math::length(handState.correction) > kCorrectionClearDistance) {
                clearHand(isLeft);
                return;
            }

            handState.state = ContactState::Inactive;
        };

        solveForHand(false, frame.right, rightHand, rightShapes, leftShapes);
        solveForHand(true, frame.left, leftHand, leftShapes, rightShapes);

        _debugSnapshot.rightState = _hands[0].state;
        _debugSnapshot.leftState = _hands[1].state;
    }
}
