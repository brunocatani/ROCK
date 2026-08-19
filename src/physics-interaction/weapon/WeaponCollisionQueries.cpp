#include "physics-interaction/weapon/WeaponCollision.h"

/*
 * Everything other systems ASK WeaponCollision. Read-only, every one of them.
 *
 * Two different kinds of read live here and they have different contracts:
 *
 *  - SEQLOCK reads of the published body set (ids, part kinds, roles, sampled
 *    velocities, the evidence snapshots). These run on consumer threads while the
 *    physics thread may be republishing, so they all go through readUnderSeqlock()
 *    and every one of them fails CLOSED: an unreadable publication reads as "no
 *    bodies", never as a stale body set. WeaponCollisionBodies.cpp owns the writing
 *    side of that protocol; see the _weaponBodyPublicationVersion comment in the
 *    header for the contract itself.
 *
 *  - LIVE reads of the scene graph (bounds, compound child poses, release geometry,
 *    the two triangle-exact proximity scans). These resolve node transforms as they
 *    are right now and are only valid for the calling frame; nothing here retains
 *    an engine pointer.
 *
 * Both proximity entry points - findCurrentWeaponSurfaceNearPoints, which keeps the
 * nearest witness per supplied point, and tryFindInteractionContactNearPoint, which
 * ranks one best body - go through resolveWeaponSurfaceScanFrame. They rank
 * differently on purpose, but they must never disagree about which frame a body
 * lives in or which bodies are testable, or the same hand position produces a
 * surface witness and no interaction contact.
 *
 * Every query re-checks the weapon generation key it started with, so a body set
 * that is rebuilt mid-query yields nothing rather than a mixed answer.
 */

#include "physics-interaction/weapon/WeaponCollisionInternal.h"

#include "RockConfig.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSemantics.h"

#include "RE/Havok/hknpShape.h"
#include "RE/NetImmerse/NiNode.h"

#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace rock
{
    using namespace weapon_collision_detail;

    namespace
    {
        // Min/max corner pair -> the (center, half-extent) form the snapshot API
        // publishes. Kept in one place so the two snapshot builders cannot drift.
        struct AabbCenterExtents
        {
            RE::NiPoint3 center{};
            RE::NiPoint3 halfExtents{};
        };

        [[nodiscard]] AabbCenterExtents aabbCenterExtents(const RE::NiPoint3& minPoint, const RE::NiPoint3& maxPoint) noexcept
        {
            return AabbCenterExtents{
                RE::NiPoint3{
                    (minPoint.x + maxPoint.x) * 0.5f,
                    (minPoint.y + maxPoint.y) * 0.5f,
                    (minPoint.z + maxPoint.z) * 0.5f,
                },
                RE::NiPoint3{
                    (maxPoint.x - minPoint.x) * 0.5f,
                    (maxPoint.y - minPoint.y) * 0.5f,
                    (maxPoint.z - minPoint.z) * 0.5f,
                },
            };
        }
    }

    bool WeaponCollision::hasWeaponBody() const
    {
        return bankHasWeaponBody(activeWeaponBodies());
    }

    std::uint32_t WeaponCollision::getWeaponBodyCount() const
    {
        return _weaponBodyCountAtomic.load(std::memory_order_acquire);
    }

    bool WeaponCollision::getApproximateBoundsSnapshot(ApproximateBoundsSnapshot& outSnapshot) const
    {
        outSnapshot = {};
        outSnapshot.generationKey = getCurrentWeaponGenerationKey();
        if (outSnapshot.generationKey == 0) {
            return false;
        }

        bool sampled = false;
        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid() || !pointFinite(instance.generatedLocalMinGame) || !pointFinite(instance.generatedLocalMaxGame)) {
                continue;
            }
            if (instance.generatedLocalMaxGame.x < instance.generatedLocalMinGame.x ||
                instance.generatedLocalMaxGame.y < instance.generatedLocalMinGame.y ||
                instance.generatedLocalMaxGame.z < instance.generatedLocalMinGame.z) {
                continue;
            }

            if (!sampled) {
                outSnapshot.minWeaponLocal = instance.generatedLocalMinGame;
                outSnapshot.maxWeaponLocal = instance.generatedLocalMaxGame;
                sampled = true;
            } else {
                outSnapshot.minWeaponLocal.x = (std::min)(outSnapshot.minWeaponLocal.x, instance.generatedLocalMinGame.x);
                outSnapshot.minWeaponLocal.y = (std::min)(outSnapshot.minWeaponLocal.y, instance.generatedLocalMinGame.y);
                outSnapshot.minWeaponLocal.z = (std::min)(outSnapshot.minWeaponLocal.z, instance.generatedLocalMinGame.z);
                outSnapshot.maxWeaponLocal.x = (std::max)(outSnapshot.maxWeaponLocal.x, instance.generatedLocalMaxGame.x);
                outSnapshot.maxWeaponLocal.y = (std::max)(outSnapshot.maxWeaponLocal.y, instance.generatedLocalMaxGame.y);
                outSnapshot.maxWeaponLocal.z = (std::max)(outSnapshot.maxWeaponLocal.z, instance.generatedLocalMaxGame.z);
            }
            ++outSnapshot.sourceBodyCount;
        }

        if (!sampled || outSnapshot.sourceBodyCount == 0) {
            outSnapshot = {};
            return false;
        }
        const auto bounds = aabbCenterExtents(outSnapshot.minWeaponLocal, outSnapshot.maxWeaponLocal);
        outSnapshot.centerWeaponLocal = bounds.center;
        outSnapshot.halfExtentsWeaponLocal = bounds.halfExtents;
        outSnapshot.valid = pointFinite(outSnapshot.centerWeaponLocal) && pointFinite(outSnapshot.halfExtentsWeaponLocal);
        return outSnapshot.valid;
    }

    bool WeaponCollision::getCompoundGeometrySnapshot(CompoundGeometrySnapshot& outSnapshot) const
    {
        outSnapshot = {};
        outSnapshot.generationKey = getCurrentWeaponGenerationKey();
        if (outSnapshot.generationKey == 0) {
            outSnapshot.failure = CompoundGeometrySnapshotFailure::NoGeneration;
            return false;
        }

        const std::uint32_t expectedBodyCount = getWeaponBodyCount();
        if (expectedBodyCount == 0) {
            outSnapshot.failure = CompoundGeometrySnapshotFailure::NoActiveBodies;
            return false;
        }

        const auto& bank = activeWeaponBodies();
        const RE::NiAVObject* packageDriveNode = resolvePackageDriveNode(bank, nullptr);
        if (!packageDriveNode) {
            outSnapshot.failure = CompoundGeometrySnapshotFailure::SourceTransformUnavailable;
            return false;
        }

        outSnapshot.children.reserve(expectedBodyCount);
        auto fail = [&](const CompoundGeometrySnapshotFailure failure,
                        const std::uint32_t sourceIndex,
                        const std::uint32_t bodyId) {
            outSnapshot.failure = failure;
            outSnapshot.failedSourceIndex = sourceIndex;
            outSnapshot.failedBodyId = bodyId;
            return false;
        };

        bool sampledPoint = false;
        std::uint32_t sourceIndex = 0;
        for (const auto& instance : bank) {
            if (!instance.body.isValid()) {
                continue;
            }

            const std::uint32_t bodyId = instance.body.getBodyId().value;
            if (!instance.shape) {
                return fail(CompoundGeometrySnapshotFailure::MissingShape, sourceIndex, bodyId);
            }
            const auto& points = instance.generatedLocalPointsGame;
            if (points.empty()) {
                return fail(CompoundGeometrySnapshotFailure::MissingPointCloud, sourceIndex, bodyId);
            }
            for (const auto& point : points) {
                if (!pointFinite(point)) {
                    return fail(CompoundGeometrySnapshotFailure::NonFinitePoint, sourceIndex, bodyId);
                }
            }
            if (!pointCloudCanBuildHull(points)) {
                return fail(CompoundGeometrySnapshotFailure::DegeneratePointCloud, sourceIndex, bodyId);
            }

            CompoundGeometryChildSnapshot child{};
            child.shape = instance.shape;
            CompoundChildPoseSnapshot pose{};
            if (!resolveCompoundChildPose(instance, packageDriveNode, pose)) {
                return fail(CompoundGeometrySnapshotFailure::SourceTransformUnavailable, sourceIndex, bodyId);
            }
            child.shapeInWeapon = pose.shapeInWeapon;
            outSnapshot.sourcePointCount += points.size();

            for (const auto& point : points) {
                if (!sampledPoint) {
                    outSnapshot.minWeaponLocal = point;
                    outSnapshot.maxWeaponLocal = point;
                    sampledPoint = true;
                } else {
                    outSnapshot.minWeaponLocal = weapon_collision_geometry_math::pointMin(outSnapshot.minWeaponLocal, point);
                    outSnapshot.maxWeaponLocal = weapon_collision_geometry_math::pointMax(outSnapshot.maxWeaponLocal, point);
                }
            }

            outSnapshot.children.push_back(std::move(child));
            ++outSnapshot.sourceBodyCount;
            ++sourceIndex;
        }

        if (!sampledPoint || outSnapshot.sourceBodyCount == 0) {
            return fail(CompoundGeometrySnapshotFailure::NoActiveBodies, sourceIndex, INVALID_BODY_ID);
        }
        if (outSnapshot.sourceBodyCount != expectedBodyCount) {
            return fail(CompoundGeometrySnapshotFailure::BodyCountChanged, sourceIndex, INVALID_BODY_ID);
        }
        if (getCurrentWeaponGenerationKey() != outSnapshot.generationKey) {
            return fail(CompoundGeometrySnapshotFailure::GenerationChanged, sourceIndex, INVALID_BODY_ID);
        }

        const auto bounds = aabbCenterExtents(outSnapshot.minWeaponLocal, outSnapshot.maxWeaponLocal);
        outSnapshot.centerWeaponLocal = bounds.center;
        outSnapshot.halfExtentsWeaponLocal = bounds.halfExtents;
        if (!pointFinite(outSnapshot.centerWeaponLocal) || !pointFinite(outSnapshot.halfExtentsWeaponLocal) ||
            outSnapshot.halfExtentsWeaponLocal.x < 0.0f ||
            outSnapshot.halfExtentsWeaponLocal.y < 0.0f ||
            outSnapshot.halfExtentsWeaponLocal.z < 0.0f) {
            return fail(CompoundGeometrySnapshotFailure::InvalidBounds, sourceIndex, INVALID_BODY_ID);
        }

        outSnapshot.failure = CompoundGeometrySnapshotFailure::None;
        outSnapshot.valid = true;
        return true;
    }

    bool WeaponCollision::resolveCompoundChildPose(
        const WeaponBodyInstance& instance,
        const RE::NiAVObject* packageDriveNode,
        CompoundChildPoseSnapshot& outPose)
    {
        outPose = {};
        if (!packageDriveNode || !instance.shape) {
            return false;
        }

        RE::NiTransform sourceInWeapon = transform_math::makeIdentityTransform<RE::NiTransform>();
        RE::NiPoint3 shapeCenterWeaponLocal = instance.generatedLocalCenterGame;
        if (instance.sourceNode) {
            if (!tryResolveDescendantLocalTransform(packageDriveNode, instance.sourceNode, sourceInWeapon)) {
                return false;
            }
            if (!weaponTransformFinite(sourceInWeapon)) {
                return false;
            }
            shapeCenterWeaponLocal = transform_math::localPointToWorld(
                sourceInWeapon,
                instance.generatedSourceLocalCenterGame);
        }

        outPose.shapeInWeapon = sourceInWeapon;
        outPose.shapeInWeapon.translate = shapeCenterWeaponLocal;
        // Generated layer-44 shapes already bake their absolute source scale.
        // The dynamic compound supplies only the live relative pose, exactly
        // matching the per-part keyframed body convention.
        outPose.shapeInWeapon.scale = 1.0f;
        return weaponTransformFinite(outPose.shapeInWeapon);
    }

    bool WeaponCollision::getCompoundChildPoseSnapshot(
        const RE::NiAVObject* currentWeaponRoot,
        const std::uint64_t expectedGenerationKey,
        std::span<CompoundChildPoseSnapshot> outChildren,
        std::size_t& outChildCount) const
    {
        outChildCount = 0;
        if (!currentWeaponRoot || expectedGenerationKey == 0 ||
            getCurrentWeaponGenerationKey() != expectedGenerationKey) {
            return false;
        }

        const auto& bank = activeWeaponBodies();
        const std::uint32_t expectedBodyCount = getWeaponBodyCount();
        if (expectedBodyCount == 0 || outChildren.size() < expectedBodyCount) {
            return false;
        }

        for (const auto& instance : bank) {
            if (!instance.body.isValid()) {
                continue;
            }
            if (outChildCount >= outChildren.size() ||
                !resolveCompoundChildPose(instance, currentWeaponRoot, outChildren[outChildCount])) {
                outChildCount = 0;
                return false;
            }
            ++outChildCount;
        }

        if (outChildCount != expectedBodyCount ||
            getCurrentWeaponGenerationKey() != expectedGenerationKey) {
            outChildCount = 0;
            return false;
        }
        return true;
    }

    WeaponCollision::ReleaseGeometrySnapshot WeaponCollision::getCurrentWeaponReleaseGeometry(
        const RE::NiPoint3& gripWorldPoint,
        const RE::NiTransform& capturedWeaponWorld) const
    {
        ReleaseGeometrySnapshot result{};
        // The captured pose is inverted below to bring the grip point into weapon
        // space, so it has to pass the same gate as any other weapon transform.
        if (!weaponTransformFinite(capturedWeaponWorld)) {
            return result;
        }
        result.hasCapturedWeaponWorld = true;
        result.capturedWeaponWorld = capturedWeaponWorld;
        if (!std::isfinite(gripWorldPoint.x) || !std::isfinite(gripWorldPoint.y) || !std::isfinite(gripWorldPoint.z)) {
            return result;
        }

        const auto& bank = activeWeaponBodies();
        float maxDistanceSquared = 0.0f;
        bool sampledPoint = false;
        auto sampleLocalPoint = [&](const RE::NiPoint3& localPoint) {
            if (!std::isfinite(localPoint.x) || !std::isfinite(localPoint.y) || !std::isfinite(localPoint.z)) {
                return;
            }
            const auto worldPoint = weapon_collision_geometry_math::localPointToWorld(
                capturedWeaponWorld.rotate,
                capturedWeaponWorld.translate,
                capturedWeaponWorld.scale,
                localPoint);
            const float dx = worldPoint.x - gripWorldPoint.x;
            const float dy = worldPoint.y - gripWorldPoint.y;
            const float dz = worldPoint.z - gripWorldPoint.z;
            const float distanceSquared = dx * dx + dy * dy + dz * dz;
            if (!std::isfinite(distanceSquared) || distanceSquared < 0.0f) {
                return;
            }
            maxDistanceSquared = (std::max)(maxDistanceSquared, distanceSquared);
            sampledPoint = true;
        };

        for (const auto& instance : bank) {
            if (!instance.body.isValid()) {
                continue;
            }
            if (!instance.generatedLocalPointsGame.empty()) {
                for (const auto& point : instance.generatedLocalPointsGame) {
                    sampleLocalPoint(point);
                }
                continue;
            }

            // Bounds remain a safe release-only fallback for a generated body
            // whose reduced hull point cache is unexpectedly empty.
            for (int x = 0; x < 2; ++x) {
                for (int y = 0; y < 2; ++y) {
                    for (int z = 0; z < 2; ++z) {
                        sampleLocalPoint(RE::NiPoint3{
                            x == 0 ? instance.generatedLocalMinGame.x : instance.generatedLocalMaxGame.x,
                            y == 0 ? instance.generatedLocalMinGame.y : instance.generatedLocalMaxGame.y,
                            z == 0 ? instance.generatedLocalMinGame.z : instance.generatedLocalMaxGame.z,
                        });
                    }
                }
            }
        }

        result.leverGameUnits = sampledPoint ? std::sqrt(maxDistanceSquared) : 0.0f;
        return result;
    }

    RE::hknpBodyId WeaponCollision::getWeaponBodyId() const
    {
        for (const auto& instance : activeWeaponBodies()) {
            if (instance.body.isValid()) {
                return instance.body.getBodyId();
            }
        }
        return RE::hknpBodyId{ INVALID_BODY_ID };
    }

    std::uint32_t WeaponCollision::getWeaponBodyIdAtomic() const
    {
        return getWeaponBodyIdAtomic(0);
    }

    std::uint32_t WeaponCollision::getWeaponBodyIdAtomic(std::size_t index) const
    {
        const auto snapshot = getWeaponBodySnapshotAtomic();
        if (index >= snapshot.count || index >= MAX_WEAPON_BODIES) {
            return INVALID_BODY_ID;
        }
        return snapshot.bodyIds[index];
    }

    WeaponCollision::WeaponBodySnapshot WeaponCollision::getWeaponBodySnapshotAtomic() const
    {
        WeaponBodySnapshot candidate{};
        const bool stable = readUnderSeqlock([&] {
            candidate = {};
            candidate.bodyIds.fill(INVALID_BODY_ID);
            candidate.generationKey = _weaponBodySetKeyAtomic.load(std::memory_order_acquire);
            candidate.count = (std::min)(_weaponBodyCountAtomic.load(std::memory_order_acquire), static_cast<std::uint32_t>(MAX_WEAPON_BODIES));
            for (std::uint32_t i = 0; i < candidate.count; ++i) {
                candidate.bodyIds[i] = _weaponBodyIdsAtomic[i].load(std::memory_order_acquire);
            }
        });
        if (stable) {
            return candidate;
        }

        // Fail closed: an unreadable publication looks like "no bodies", never like
        // a stale body set.
        WeaponBodySnapshot empty{};
        empty.bodyIds.fill(INVALID_BODY_ID);
        return empty;
    }

    bool WeaponCollision::isWeaponBodyIdAtomic(std::uint32_t bodyId) const
    {
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        const auto snapshot = getWeaponBodySnapshotAtomic();
        for (std::uint32_t i = 0; i < snapshot.count && i < MAX_WEAPON_BODIES; ++i) {
            if (snapshot.bodyIds[i] == bodyId) {
                return true;
            }
        }
        return false;
    }

    bool WeaponCollision::tryGetWeaponContactAtomic(std::uint32_t bodyId, WeaponInteractionContact& outContact) const
    {
        outContact = {};
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        WeaponInteractionContact candidate{};
        bool found = false;
        const bool stable = readUnderSeqlock([&] {
            candidate = {};
            found = false;
            const std::uint32_t count = (std::min)(_weaponBodyCountAtomic.load(std::memory_order_acquire), static_cast<std::uint32_t>(MAX_WEAPON_BODIES));
            for (std::uint32_t i = 0; i < count; ++i) {
                if (_weaponBodyIdsAtomic[i].load(std::memory_order_acquire) != bodyId) {
                    continue;
                }

                candidate.valid = true;
                candidate.bodyId = bodyId;
                candidate.partKind = static_cast<WeaponPartKind>(_weaponBodyPartKindsAtomic[i].load(std::memory_order_acquire));
                candidate.reloadRole = static_cast<WeaponReloadRole>(_weaponBodyReloadRolesAtomic[i].load(std::memory_order_acquire));
                candidate.supportGripRole = static_cast<WeaponSupportGripRole>(_weaponBodySupportRolesAtomic[i].load(std::memory_order_acquire));
                candidate.socketRole = static_cast<WeaponSocketRole>(_weaponBodySocketRolesAtomic[i].load(std::memory_order_acquire));
                candidate.actionRole = static_cast<WeaponActionRole>(_weaponBodyActionRolesAtomic[i].load(std::memory_order_acquire));
                candidate.fallbackGripPose = static_cast<WeaponGripPoseId>(_weaponBodyGripPosesAtomic[i].load(std::memory_order_acquire));
                candidate.interactionRoot = reinterpret_cast<RE::NiAVObject*>(_weaponBodyInteractionRootsAtomic[i].load(std::memory_order_acquire));
                candidate.sourceRoot = reinterpret_cast<RE::NiAVObject*>(_weaponBodySourceRootsAtomic[i].load(std::memory_order_acquire));
                candidate.weaponGenerationKey = _weaponBodyGenerationKeysAtomic[i].load(std::memory_order_acquire);
                found = true;
                break;
            }
        });
        if (!stable) {
            return false;
        }
        if (found) {
            outContact = candidate;
        }
        return found;
    }

    bool WeaponCollision::tryGetWeaponBodySampledVelocityAtomic(std::uint32_t bodyId, float* outVelocityHavok) const
    {
        if (!outVelocityHavok) {
            return false;
        }
        outVelocityHavok[0] = 0.0f;
        outVelocityHavok[1] = 0.0f;
        outVelocityHavok[2] = 0.0f;
        outVelocityHavok[3] = 0.0f;
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        float vx = 0.0f;
        float vy = 0.0f;
        float vz = 0.0f;
        bool found = false;
        const bool stable = readUnderSeqlock([&] {
            vx = 0.0f;
            vy = 0.0f;
            vz = 0.0f;
            found = false;
            const std::uint32_t count = (std::min)(_weaponBodyCountAtomic.load(std::memory_order_acquire), static_cast<std::uint32_t>(MAX_WEAPON_BODIES));
            for (std::uint32_t i = 0; i < count; ++i) {
                if (_weaponBodyIdsAtomic[i].load(std::memory_order_acquire) != bodyId ||
                    _weaponBodySampledVelocityValidAtomic[i].load(std::memory_order_acquire) == 0) {
                    continue;
                }

                vx = _weaponBodySampledVelocityHavokXAtomic[i].load(std::memory_order_acquire);
                vy = _weaponBodySampledVelocityHavokYAtomic[i].load(std::memory_order_acquire);
                vz = _weaponBodySampledVelocityHavokZAtomic[i].load(std::memory_order_acquire);
                found = true;
                break;
            }
        });
        // A consistent read that found nothing, or found a non-finite sample, is a
        // final answer - only a torn read is worth another attempt, and
        // readUnderSeqlock has already spent those.
        if (!stable || !found || !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz)) {
            return false;
        }

        outVelocityHavok[0] = vx;
        outVelocityHavok[1] = vy;
        outVelocityHavok[2] = vz;
        return true;
    }

    bool WeaponCollision::tryGetWeaponContactDebugInfo(std::uint32_t bodyId, WeaponInteractionDebugInfo& outInfo) const
    {
        outInfo = {};
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }
        if (!isWeaponBodyIdAtomic(bodyId)) {
            return false;
        }

        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid() || instance.body.getBodyId().value != bodyId) {
                continue;
            }

            RE::NiAVObject* packageDriveRoot = resolvePackageDriveNode(activeWeaponBodies(), nullptr);
            outInfo.sourceName = instance.sourceName;
            outInfo.interactionRootName = packageDriveRoot ? safeNodeName(packageDriveRoot) : "";
            outInfo.sourceRootName = instance.sourceRootName;
            return true;
        }

        return false;
    }

    bool WeaponCollision::tryBuildSupportGripEvidenceView(
        const WeaponBodyInstance& instance,
        const RE::NiAVObject* currentWeaponRoot,
        SupportGripEvidenceView& outView) const
    {
        outView = {};
        if (!instance.body.isValid() ||
            instance.generatedLocalTrianglesGame.empty()) {
            return false;
        }

        RE::NiTransform localToWorld{};
        const bool sourceNodeCurrent = instance.sourceNode && currentWeaponRoot &&
            tryResolveDescendantWorldTransform(
                currentWeaponRoot,
                currentWeaponRoot->world,
                instance.sourceNode,
                localToWorld);
        if (!sourceNodeCurrent) {
            const RE::NiAVObject* fallbackRoot = currentWeaponRoot ?
                currentWeaponRoot :
                instance.driveNode;
            if (!fallbackRoot) {
                return false;
            }
            localToWorld = fallbackRoot->world;
        }
        if (!weaponTransformFinite(localToWorld)) {
            return false;
        }

        const auto& localTriangles =
            sourceNodeCurrent &&
                !instance.generatedSourceLocalTrianglesGame.empty() ?
            instance.generatedSourceLocalTrianglesGame :
            instance.generatedLocalTrianglesGame;
        if (localTriangles.empty() ||
            !std::isfinite(localToWorld.scale) ||
            std::abs(localToWorld.scale) <= 0.000001f) {
            return false;
        }

        outView.localTriangles = std::span<const TriangleData>(
            localTriangles.data(),
            localTriangles.size());
        outView.localToWorld = localToWorld;
        outView.sourceGroupId = instance.generatedSourceGroupId != 0 ?
            instance.generatedSourceGroupId :
            reinterpret_cast<std::uintptr_t>(instance.sourceNode);
        outView.bodyId = instance.body.getBodyId().value;
        outView.weaponGenerationKey = getCurrentWeaponGenerationKey();
        outView.sourceNodeCurrent = sourceNodeCurrent;
        return outView.weaponGenerationKey != 0;
    }

    bool WeaponCollision::tryGetSupportGripEvidenceView(
        std::uint32_t bodyId,
        const RE::NiAVObject* currentWeaponRoot,
        SupportGripEvidenceView& outView) const
    {
        outView = {};
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid() ||
                instance.body.getBodyId().value != bodyId) {
                continue;
            }
            return tryBuildSupportGripEvidenceView(
                instance,
                currentWeaponRoot,
                outView);
        }

        return false;
    }

    std::size_t WeaponCollision::findSupportGripEvidenceViews(
        const RE::NiAVObject* currentWeaponRoot,
        std::span<SupportGripEvidenceView> outViews) const
    {
        for (auto& view : outViews) {
            view = {};
        }
        if (outViews.empty()) {
            return 0;
        }

        std::array<std::uintptr_t, MAX_WEAPON_BODIES> seenSourceGroups{};
        std::size_t seenSourceCount = 0;
        std::size_t viewCount = 0;
        for (const auto& instance : activeWeaponBodies()) {
            SupportGripEvidenceView view{};
            if (!tryBuildSupportGripEvidenceView(
                    instance,
                    currentWeaponRoot,
                    view)) {
                continue;
            }

            const std::uintptr_t sourceGroup = view.sourceGroupId;
            const bool duplicate = sourceGroup != 0 &&
                std::find(
                    seenSourceGroups.begin(),
                    seenSourceGroups.begin() + seenSourceCount,
                    sourceGroup) !=
                    seenSourceGroups.begin() + seenSourceCount;
            if (duplicate) {
                continue;
            }
            if (sourceGroup != 0 &&
                seenSourceCount < seenSourceGroups.size()) {
                seenSourceGroups[seenSourceCount++] = sourceGroup;
            }

            outViews[viewCount++] = view;
            if (viewCount >= outViews.size()) {
                break;
            }
        }
        return viewCount;
    }

    bool WeaponCollision::tryFindCurrentWeaponSurfaceNearPoint(
        const RE::NiAVObject* currentWeaponRoot,
        const RE::NiPoint3& pointWorld,
        float maxDistanceGameUnits,
        WeaponSurfaceProximityWitness& outWitness) const
    {
        const std::array<RE::NiPoint3, 1> points{ pointWorld };
        std::array<WeaponSurfaceProximityWitness, 1> witnesses{};
        const std::size_t witnessCount = findCurrentWeaponSurfaceNearPoints(
            currentWeaponRoot,
            points,
            maxDistanceGameUnits,
            witnesses);
        outWitness = witnesses[0];
        return witnessCount == 1 && outWitness.valid;
    }

    /*
     * Resolve the frame one generated body is proximity-tested in, and reject the
     * bodies that cannot be tested at all.
     *
     * Shared by both triangle-exact entry points below:
     * findCurrentWeaponSurfaceNearPoints (batch, nearest witness per point) and
     * tryFindInteractionContactNearPoint (one point, best-ranked body). The two
     * rank differently on purpose, but they must agree on WHICH frame a body lives
     * in and WHICH bodies are testable - otherwise the same hand position yields a
     * surface witness and no interaction contact.
     */
    bool WeaponCollision::resolveWeaponSurfaceScanFrame(
        const RE::NiAVObject* scanRoot,
        const WeaponBodyInstance& instance,
        WeaponSurfaceScanFrame& outFrame)
    {
        outFrame = {};
        if (!scanRoot || !instance.body.isValid()) {
            return false;
        }

        // A body extracted with its own source-node triangles is tested in that
        // node's frame: pumps, bolts and magazines move independently of the weapon
        // root, so root-space triangles would put the surface in the wrong place
        // mid-animation.
        RE::NiTransform world = scanRoot->world;
        const bool sourceNodeCurrent = instance.sourceNode &&
            tryResolveDescendantWorldTransform(
                scanRoot,
                scanRoot->world,
                instance.sourceNode,
                world);
        outFrame.useSourceFrame = sourceNodeCurrent && !instance.generatedSourceLocalTrianglesGame.empty();
        if (!outFrame.useSourceFrame) {
            // Covers the failed-resolve case too: the resolver may have written a
            // partial transform, so the root frame is restored rather than trusted.
            world = scanRoot->world;
        }

        outFrame.world = world;
        outFrame.localTriangles = outFrame.useSourceFrame ?
            &instance.generatedSourceLocalTrianglesGame :
            &instance.generatedLocalTrianglesGame;
        outFrame.boundsMin = outFrame.useSourceFrame ?
            &instance.generatedSourceLocalMinGame :
            &instance.generatedLocalMinGame;
        outFrame.boundsMax = outFrame.useSourceFrame ?
            &instance.generatedSourceLocalMaxGame :
            &instance.generatedLocalMaxGame;

        // Fail closed: no triangles, an unusable frame, or an inverted/NaN AABB all
        // mean this body cannot answer a proximity question this frame.
        // weaponTransformFinite already rejects |scale| <= 0.0001, so the frame
        // scale needs no separate floor here.
        if (outFrame.localTriangles->empty() ||
            !weaponTransformFinite(outFrame.world) ||
            !pointFinite(*outFrame.boundsMin) ||
            !pointFinite(*outFrame.boundsMax) ||
            outFrame.boundsMin->x > outFrame.boundsMax->x ||
            outFrame.boundsMin->y > outFrame.boundsMax->y ||
            outFrame.boundsMin->z > outFrame.boundsMax->z) {
            return false;
        }

        outFrame.absoluteScale = std::abs(outFrame.world.scale);
        return true;
    }

    std::size_t WeaponCollision::findCurrentWeaponSurfaceNearPoints(
        const RE::NiAVObject* currentWeaponRoot,
        const std::span<const RE::NiPoint3> pointsWorld,
        const float maxDistanceGameUnits,
        const std::span<WeaponSurfaceProximityWitness> outWitnesses) const
    {
        constexpr std::size_t kMaximumPointCount = 16;
        for (auto& witness : outWitnesses) {
            witness = {};
        }
        const std::uint64_t currentGeneration = getCurrentWeaponGenerationKey();
        if (!currentWeaponRoot ||
            currentGeneration == 0 ||
            pointsWorld.empty() ||
            pointsWorld.size() > kMaximumPointCount ||
            outWitnesses.size() != pointsWorld.size() ||
            !std::isfinite(maxDistanceGameUnits) ||
            maxDistanceGameUnits < 0.0f) {
            return 0;
        }
        for (const auto& pointWorld : pointsWorld) {
            if (!pointFinite(pointWorld)) {
                return 0;
            }
        }

        /*
         * Authored-pose validity is a surface claim, not a part-AABB claim:
         * animation-zero/default hands can land inside a broad receiver box
         * while remaining nowhere near rendered weapon geometry. AABBs only
         * reject unrelated parts before the exact cached-triangle test.
         *
         * The bounded batch form is also used by the opt-in authored-grip
         * visualizer. It allocates nothing, scans each current body/triangle
         * bank once, and retains the true nearest witness per supplied point.
         */
        for (const auto& instance : activeWeaponBodies()) {
            WeaponSurfaceScanFrame frame{};
            if (!resolveWeaponSurfaceScanFrame(currentWeaponRoot, instance, frame)) {
                continue;
            }
            const RE::NiTransform& surfaceWorld = frame.world;
            const auto& localTriangles = *frame.localTriangles;
            const RE::NiPoint3& boundsMin = *frame.boundsMin;
            const RE::NiPoint3& boundsMax = *frame.boundsMax;

            // The search radius is in game units; the cached triangles are in the
            // frame's own local space, so the radius converts once per body.
            const float absoluteScale = frame.absoluteScale;
            const float localRadius = maxDistanceGameUnits / absoluteScale;
            std::array<RE::NiPoint3, kMaximumPointCount> localPoints{};
            std::array<bool, kMaximumPointCount> pointMayReachBody{};
            bool anyPointMayReachBody = false;
            for (std::size_t pointIndex = 0;
                 pointIndex < pointsWorld.size();
                 ++pointIndex) {
                localPoints[pointIndex] =
                    weapon_collision_geometry_math::worldPointToLocal(
                        surfaceWorld.rotate,
                        surfaceWorld.translate,
                        surfaceWorld.scale,
                        pointsWorld[pointIndex]);
                if (!pointFinite(localPoints[pointIndex])) {
                    continue;
                }
                const float boundsDistanceSquared =
                    weapon_interaction_probe_math::pointAabbDistanceSquared(
                        localPoints[pointIndex],
                        boundsMin,
                        boundsMax);
                pointMayReachBody[pointIndex] =
                    std::isfinite(boundsDistanceSquared) &&
                    weapon_interaction_probe_math::isWithinProbeRadiusSquared(
                        boundsDistanceSquared,
                        localRadius);
                anyPointMayReachBody =
                    anyPointMayReachBody || pointMayReachBody[pointIndex];
            }
            if (!anyPointMayReachBody) {
                continue;
            }

            for (const auto& triangle : localTriangles) {
                if (!pointFinite(triangle.v0) ||
                    !pointFinite(triangle.v1) ||
                    !pointFinite(triangle.v2)) {
                    continue;
                }

                for (std::size_t pointIndex = 0;
                     pointIndex < pointsWorld.size();
                     ++pointIndex) {
                    if (!pointMayReachBody[pointIndex]) {
                        continue;
                    }
                    float surfaceDistanceSquared =
                        (std::numeric_limits<float>::infinity)();
                    const RE::NiPoint3 closestPointLocal =
                        closestPointOnTriangleToPoint(
                            localPoints[pointIndex],
                            triangle,
                            surfaceDistanceSquared);
                    if (!std::isfinite(surfaceDistanceSquared) ||
                        surfaceDistanceSquared < 0.0f ||
                        !weapon_interaction_probe_math::isWithinProbeRadiusSquared(
                            surfaceDistanceSquared,
                            localRadius)) {
                        continue;
                    }

                    const float distanceGameUnits =
                        std::sqrt(surfaceDistanceSquared) * absoluteScale;
                    auto& witness = outWitnesses[pointIndex];
                    if (!std::isfinite(distanceGameUnits) ||
                        (witness.valid &&
                            distanceGameUnits >= witness.distanceGameUnits)) {
                        continue;
                    }
                    witness.closestPointWorld =
                        weapon_collision_geometry_math::localPointToWorld(
                            surfaceWorld.rotate,
                            surfaceWorld.translate,
                            surfaceWorld.scale,
                            closestPointLocal);
                    witness.distanceGameUnits = distanceGameUnits;
                    witness.bodyId = instance.body.getBodyId().value;
                    witness.weaponGenerationKey = currentGeneration;
                    witness.sourceNodeCurrent = frame.useSourceFrame;
                    witness.valid = pointFinite(witness.closestPointWorld);
                }
            }
        }

        if (getCurrentWeaponGenerationKey() != currentGeneration) {
            for (auto& witness : outWitnesses) {
                witness = {};
            }
            return 0;
        }

        std::size_t witnessCount = 0;
        for (const auto& witness : outWitnesses) {
            if (witness.valid &&
                witness.weaponGenerationKey == currentGeneration) {
                ++witnessCount;
            }
        }
        return witnessCount;
    }

    std::vector<WeaponCollisionProfileEvidenceDescriptor> WeaponCollision::buildProfileEvidenceSnapshot(
        const WeaponBodyBank& bank,
        WeaponCompositionSnapshot& outComposition) const
    {
        std::vector<WeaponCollisionProfileEvidenceDescriptor> descriptors;
        descriptors.reserve(bankWeaponBodyCount(bank));

        /*
         * Pair slot-classified parts with the installed OMOD occupying that
         * slot: resolve the equipped instance's active mods once and index
         * them by attach-point keyword FormID. Runs once per publication on
         * the main thread; ~a dozen form lookups.
         */
        const auto omodByAttachPointFormId =
            readEquippedOmodsByAttachPointFormId(&outComposition);
        outComposition.weaponGenerationKey = _cachedWeaponBodySetKey;
        outComposition.weaponFormId = _cachedWeaponFormID;

        auto copyLocalPoints = [](const std::vector<RE::NiPoint3>& points) {
            std::vector<WeaponEvidencePoint3> result;
            result.reserve(points.size());
            for (const auto& point : points) {
                result.push_back(makeWeaponEvidencePoint(point.x, point.y, point.z));
            }
            return result;
        };

        RE::NiAVObject* packageDriveRoot = resolvePackageDriveNode(bank, nullptr);
        for (const auto& instance : bank) {
            if (!instance.body.isValid()) {
                continue;
            }

            RE::NiAVObject* interactionRoot = packageDriveRoot ? packageDriveRoot : instance.driveNode;
            WeaponCollisionProfileEvidenceDescriptor descriptor{};
            descriptor.valid = true;
            descriptor.bodyId = instance.body.getBodyId().value;
            descriptor.weaponGenerationKey = _cachedWeaponBodySetKey;
            descriptor.sourceRootAddress = reinterpret_cast<std::uintptr_t>(instance.sourceNode);
            descriptor.geometryRootAddress = reinterpret_cast<std::uintptr_t>(interactionRoot);
            descriptor.sourceRootName = instance.sourceRootName;
            descriptor.geometryRootName = interactionRoot ? safeNodeName(interactionRoot) : "";
            descriptor.sourceName = instance.sourceName;
            descriptor.semantic = instance.semantic;
            descriptor.localBoundsGame = WeaponEvidenceBounds3{
                .min = makeWeaponEvidencePoint(instance.generatedLocalMinGame.x, instance.generatedLocalMinGame.y, instance.generatedLocalMinGame.z),
                .max = makeWeaponEvidencePoint(instance.generatedLocalMaxGame.x, instance.generatedLocalMaxGame.y, instance.generatedLocalMaxGame.z),
                .valid = true,
            };
            descriptor.localMeshPointsGame = copyLocalPoints(instance.generatedLocalPointsGame);
            descriptor.pointCount = instance.generatedPointCount;
            if (instance.semantic.attachPointFormId != 0) {
                const auto omodIt = omodByAttachPointFormId.find(instance.semantic.attachPointFormId);
                if (omodIt != omodByAttachPointFormId.end()) {
                    descriptor.omodFormId = omodIt->second;
                }
            }
            const auto partValue = static_cast<std::uint32_t>(
                instance.semantic.partKind);
            if (partValue < 64) {
                const auto coverageBit = 1ull << partValue;
                outComposition.semanticCoverageMask |= coverageBit;
                for (std::uint32_t compositionIndex = 0;
                     compositionIndex < outComposition.entryCount;
                     ++compositionIndex) {
                    auto& entry = outComposition.entries[compositionIndex];
                    if ((descriptor.omodFormId != 0 &&
                            entry.omodFormId == descriptor.omodFormId) ||
                        (instance.semantic.attachPointFormId != 0 &&
                            entry.attachPointFormId ==
                                instance.semantic.attachPointFormId)) {
                        entry.semanticCoverageMask |= coverageBit;
                        entry.flags |= 1u << 3;
                    }
                }
            }
            descriptors.push_back(std::move(descriptor));
        }

        constexpr std::uint64_t kFnvOffset = 1469598103934665603ull;
        constexpr std::uint64_t kFnvPrime = 1099511628211ull;
        auto signature = kFnvOffset;
        for (std::uint32_t i = 0; i < outComposition.entryCount; ++i) {
            const auto& entry = outComposition.entries[i];
            for (const auto value : {
                     entry.omodFormId,
                     entry.attachPointFormId,
                     entry.stableIndex,
                     entry.flags }) {
                signature ^= value;
                signature *= kFnvPrime;
            }
            if ((entry.flags & (1u << 0)) != 0 &&
                entry.semanticCoverageMask == 0 && i < 64) {
                outComposition.missingCoverageMask |= 1ull << i;
            }
        }
        outComposition.compositionSignature =
            outComposition.entryCount != 0 ? signature : 0;

        return descriptors;
    }

    std::vector<WeaponCollisionProfileEvidenceDescriptor> WeaponCollision::getProfileEvidenceDescriptors() const
    {
        std::vector<WeaponCollisionProfileEvidenceDescriptor> descriptors;
        if (readUnderSeqlock([&] {
                std::scoped_lock lock(_weaponEvidenceSnapshotMutex);
                descriptors = _profileEvidenceSnapshot;
            })) {
            return descriptors;
        }

        return {};
    }

    WeaponCollision::NativeScopeSightAnchorSnapshot WeaponCollision::getNativeScopeSightAnchorSnapshot() const
    {
        NativeScopeSightAnchorSnapshot snapshot{};
        if (readUnderSeqlock([&] {
                std::scoped_lock lock(_weaponEvidenceSnapshotMutex);
                snapshot = _nativeScopeSightAnchorSnapshot;
            })) {
            return snapshot;
        }

        return {};
    }

    WeaponCollision::WeaponCompositionSnapshot
    WeaponCollision::getWeaponCompositionSnapshot() const
    {
        WeaponCompositionSnapshot snapshot{};
        if (readUnderSeqlock([&] {
                std::scoped_lock lock(_weaponEvidenceSnapshotMutex);
                snapshot = _weaponCompositionSnapshot;
            })) {
            return snapshot;
        }
        return {};
    }

    bool WeaponCollision::tryGetProfileEvidenceDescriptorForBodyId(
        std::uint32_t bodyId,
        WeaponCollisionProfileEvidenceDescriptor& outDescriptor,
        RE::NiAVObject*& outSourceNode) const
    {
        outDescriptor = {};
        outSourceNode = nullptr;
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        const auto descriptors = getProfileEvidenceDescriptors();
        for (const auto& descriptor : descriptors) {
            if (!descriptor.valid || descriptor.bodyId != bodyId) {
                continue;
            }

            outDescriptor = descriptor;
            outSourceNode = reinterpret_cast<RE::NiAVObject*>(descriptor.sourceRootAddress);
            return true;
        }

        return false;
    }

    bool WeaponCollision::tryFindInteractionContactNearPoint(
        const RE::NiAVObject* weaponNode,
        const RE::NiPoint3& probeWorldPoint,
        float probeRadiusGame,
        WeaponInteractionContact& outContact) const
    {
        outContact = {};
        const std::uint64_t currentGeneration = getCurrentWeaponGenerationKey();
        if (!weaponNode || currentGeneration == 0 ||
            !pointFinite(probeWorldPoint) ||
            !std::isfinite(probeRadiusGame) || probeRadiusGame <= 0.0f) {
            return false;
        }

        weapon_interaction_probe_math::ProbeCandidateRank bestRank{};
        const WeaponBodyInstance* bestInstance = nullptr;
        int boundsCandidateCount = 0;
        int surfaceCandidateCount = 0;
        const RE::NiAVObject* packageDriveRoot = resolvePackageDriveNode(activeWeaponBodies(), const_cast<RE::NiAVObject*>(weaponNode));
        if (!packageDriveRoot) {
            return false;
        }

        for (const auto& instance : activeWeaponBodies()) {
            WeaponSurfaceScanFrame frame{};
            if (!resolveWeaponSurfaceScanFrame(packageDriveRoot, instance, frame)) {
                continue;
            }
            const RE::NiTransform& probeWorld = frame.world;
            const auto& localTriangles = *frame.localTriangles;
            const RE::NiPoint3& boundsMin = *frame.boundsMin;
            const RE::NiPoint3& boundsMax = *frame.boundsMax;

            const float absoluteScale = frame.absoluteScale;
            const float localRadius = probeRadiusGame / absoluteScale;
            const RE::NiPoint3 probeLocal = weapon_collision_geometry_math::worldPointToLocal(
                probeWorld.rotate,
                probeWorld.translate,
                probeWorld.scale,
                probeWorldPoint);
            if (!pointFinite(probeLocal)) {
                continue;
            }

            const float boundsDistanceSquared = weapon_interaction_probe_math::pointAabbDistanceSquared(
                probeLocal,
                boundsMin,
                boundsMax);
            if (!std::isfinite(boundsDistanceSquared) ||
                !weapon_interaction_probe_math::isWithinProbeRadiusSquared(
                    boundsDistanceSquared,
                    localRadius)) {
                continue;
            }

            ++boundsCandidateCount;
            float minimumSurfaceDistanceSquaredLocal =
                (std::numeric_limits<float>::infinity)();
            for (const auto& triangle : localTriangles) {
                if (!pointFinite(triangle.v0) ||
                    !pointFinite(triangle.v1) ||
                    !pointFinite(triangle.v2)) {
                    continue;
                }
                float surfaceDistanceSquaredLocal =
                    (std::numeric_limits<float>::infinity)();
                (void)closestPointOnTriangleToPoint(
                    probeLocal,
                    triangle,
                    surfaceDistanceSquaredLocal);
                if (std::isfinite(surfaceDistanceSquaredLocal) &&
                    surfaceDistanceSquaredLocal >= 0.0f) {
                    minimumSurfaceDistanceSquaredLocal = (std::min)(
                        minimumSurfaceDistanceSquaredLocal,
                        surfaceDistanceSquaredLocal);
                }
            }
            if (!std::isfinite(minimumSurfaceDistanceSquaredLocal) ||
                !weapon_interaction_probe_math::isWithinProbeRadiusSquared(
                    minimumSurfaceDistanceSquaredLocal,
                    localRadius)) {
                continue;
            }

            ++surfaceCandidateCount;
            const float scaleSquared = absoluteScale * absoluteScale;
            const weapon_interaction_probe_math::ProbeCandidateRank rank{
                .distanceSquaredGame =
                    minimumSurfaceDistanceSquaredLocal * scaleSquared,
                .aabbDiagonalSquaredGame =
                    weapon_interaction_probe_math::aabbDiagonalSquared(
                        boundsMin,
                        boundsMax) * scaleSquared,
                .semanticPriority = instance.semantic.priority,
            };
            if (bestInstance && !weapon_interaction_probe_math::isBetterProbeCandidate(rank, bestRank)) {
                continue;
            }

            bestRank = rank;
            bestInstance = &instance;
        }

        if (!bestInstance || getCurrentWeaponGenerationKey() != currentGeneration) {
            return false;
        }

        ROCK_LOG_SAMPLE_DEBUG(Weapon,
            g_rockConfig.rockLogSampleMilliseconds,
            "WeaponInteractionProbe exact surface ranked: part={} bodyId={} dist={:.2f} diag={:.1f} priority={} boundsCandidates={} surfaceCandidates={}",
            static_cast<int>(bestInstance->semantic.partKind),
            bestInstance->body.getBodyId().value,
            std::sqrt(bestRank.distanceSquaredGame),
            std::sqrt(bestRank.aabbDiagonalSquaredGame),
            bestInstance->semantic.priority,
            boundsCandidateCount,
            surfaceCandidateCount);

        outContact.valid = true;
        outContact.bodyId = bestInstance->body.getBodyId().value;
        outContact.partKind = bestInstance->semantic.partKind;
        outContact.reloadRole = bestInstance->semantic.reloadRole;
        outContact.supportGripRole = bestInstance->semantic.supportGripRole;
        outContact.socketRole = bestInstance->semantic.socketRole;
        outContact.actionRole = bestInstance->semantic.actionRole;
        outContact.fallbackGripPose = bestInstance->semantic.fallbackGripPose;
        outContact.interactionRoot = const_cast<RE::NiAVObject*>(packageDriveRoot);
        outContact.sourceRoot = bestInstance->sourceNode;
        outContact.weaponGenerationKey = currentGeneration;
        outContact.probeDistanceGame = std::sqrt(bestRank.distanceSquaredGame);
        return true;
    }
}
