#include "physics-interaction/weapon/WeaponCollisionInternal.h"

// Read-side snapshots and queries: bounds, compound geometry, contacts, support-grip evidence views, surface probes, profile evidence, and emitters.

namespace rock
{
    std::size_t WeaponCollision::collectAttachOnlyGripIndicators(
        const RE::NiAVObject* currentWeaponRoot,
        std::span<const weapon_part_runtime::Target> targets,
        std::span<const std::uint32_t> candidateBodyIds,
        std::span<RE::NiPoint3> outPositions) const
    {
        const auto generation = getCurrentWeaponGenerationKey();
        if (!currentWeaponRoot || generation == 0 || outPositions.empty() || candidateBodyIds.empty() ||
            !activeWeaponBodyRootMatches(currentWeaponRoot) ||
            !std::any_of(targets.begin(), targets.end(), [generation](const auto& target) {
                return weapon_part_runtime::targetAppliesToGeneration(target, generation) &&
                    target.grabMode == weapon_part_runtime::GrabMode::AttachOnly;
            })) {
            return 0;
        }

        std::array<std::uintptr_t, MAX_WEAPON_BODIES> markedSources{};
        std::size_t count = 0;
        for (const auto& instance : activeWeaponBodies()) {
            if (count == outPositions.size() || count == markedSources.size()) {
                break;
            }
            if (!instance.body.isValid() || !instance.geometry || !instance.sourceNode) {
                continue;
            }
            // Registration alone is not a hover. Only the exact body selected
            // by an available hand's current grab probe may show a marker.
            if (std::find(candidateBodyIds.begin(), candidateBodyIds.end(),
                    instance.body.getBodyId().value) == candidateBodyIds.end()) {
                continue;
            }
            const auto source = reinterpret_cast<std::uintptr_t>(instance.sourceNode);
            if (std::find(markedSources.begin(), markedSources.begin() + count, source) !=
                markedSources.begin() + count) {
                continue;
            }
            // Resolve the same identity and semantic fields as contact routing.
            // A higher-priority full-authority target must suppress an attach-only mark.
            const auto resolution = weapon_part_runtime::resolveTarget(targets, {
                .weaponGenerationKey = generation,
                .bodyId = instance.body.getBodyId().value,
                .sourceRoot = source,
                .sourceName = instance.sourceName,
                .partKind = instance.semantic.partKind,
                .reloadRole = instance.semantic.reloadRole,
                .supportRole = instance.semantic.supportGripRole,
                .socketRole = instance.semantic.socketRole,
                .actionRole = instance.semantic.actionRole,
            });
            if (!resolution.matched ||
                resolution.grabMode != weapon_part_runtime::GrabMode::AttachOnly) {
                continue;
            }
            RE::NiTransform sourceWorld{};
            if (!tryResolveDescendantWorldTransform(currentWeaponRoot,
                    currentWeaponRoot->world, instance.sourceNode, sourceWorld) ||
                !f4vr::isNodeVisible(instance.sourceNode) ||
                std::abs(sourceWorld.scale) <= 0.000001f) {
                continue;
            }
            const auto position = transform_math::localPointToWorld(
                sourceWorld, instance.generatedSourceLocalCenterGame);
            if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
                !std::isfinite(position.z)) {
                continue;
            }
            markedSources[count] = source;
            outPositions[count++] = position;
        }
        return generation == getCurrentWeaponGenerationKey() ? count : 0;
    }

    bool WeaponCollision::getApproximateBoundsSnapshot(ApproximateBoundsSnapshot& outSnapshot) const
    {
        outSnapshot = {};
        outSnapshot.generationKey = getCurrentWeaponGenerationKey();
        if (outSnapshot.generationKey == 0) {
            return false;
        }

        const auto finitePoint = [](const RE::NiPoint3& point) {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        };
        bool sampled = false;
        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid() || !instance.geometry || !finitePoint(instance.generatedLocalMinGame) || !finitePoint(instance.generatedLocalMaxGame)) {
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
        outSnapshot.centerWeaponLocal = RE::NiPoint3{
            (outSnapshot.minWeaponLocal.x + outSnapshot.maxWeaponLocal.x) * 0.5f,
            (outSnapshot.minWeaponLocal.y + outSnapshot.maxWeaponLocal.y) * 0.5f,
            (outSnapshot.minWeaponLocal.z + outSnapshot.maxWeaponLocal.z) * 0.5f,
        };
        outSnapshot.halfExtentsWeaponLocal = RE::NiPoint3{
            (outSnapshot.maxWeaponLocal.x - outSnapshot.minWeaponLocal.x) * 0.5f,
            (outSnapshot.maxWeaponLocal.y - outSnapshot.minWeaponLocal.y) * 0.5f,
            (outSnapshot.maxWeaponLocal.z - outSnapshot.minWeaponLocal.z) * 0.5f,
        };
        outSnapshot.valid = finitePoint(outSnapshot.centerWeaponLocal) && finitePoint(outSnapshot.halfExtentsWeaponLocal);
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
        const auto finitePoint = [](const RE::NiPoint3& point) {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        };
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
            if (!instance.body.isValid() || !instance.geometry) {
                continue;
            }

            const std::uint32_t bodyId = instance.body.getBodyId().value;
            if (!instance.shape) {
                return fail(CompoundGeometrySnapshotFailure::MissingShape, sourceIndex, bodyId);
            }
            const auto& points = instance.geometry->localPointsGame;
            if (points.empty()) {
                return fail(CompoundGeometrySnapshotFailure::MissingPointCloud, sourceIndex, bodyId);
            }
            for (const auto& point : points) {
                if (!finitePoint(point)) {
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

        outSnapshot.centerWeaponLocal = RE::NiPoint3{
            (outSnapshot.minWeaponLocal.x + outSnapshot.maxWeaponLocal.x) * 0.5f,
            (outSnapshot.minWeaponLocal.y + outSnapshot.maxWeaponLocal.y) * 0.5f,
            (outSnapshot.minWeaponLocal.z + outSnapshot.maxWeaponLocal.z) * 0.5f,
        };
        outSnapshot.halfExtentsWeaponLocal = RE::NiPoint3{
            (outSnapshot.maxWeaponLocal.x - outSnapshot.minWeaponLocal.x) * 0.5f,
            (outSnapshot.maxWeaponLocal.y - outSnapshot.minWeaponLocal.y) * 0.5f,
            (outSnapshot.maxWeaponLocal.z - outSnapshot.minWeaponLocal.z) * 0.5f,
        };
        if (!finitePoint(outSnapshot.centerWeaponLocal) || !finitePoint(outSnapshot.halfExtentsWeaponLocal) ||
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
        if (!activeWeaponBodyRootMatches(currentWeaponRoot)) {
            return false;
        }
        const std::uint32_t expectedBodyCount = getWeaponBodyCount();
        if (expectedBodyCount == 0 || outChildren.size() < expectedBodyCount) {
            return false;
        }

        for (const auto& instance : bank) {
            if (!instance.body.isValid() || !instance.geometry) {
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
        bool capturedTransformFinite =
            std::isfinite(capturedWeaponWorld.translate.x) &&
            std::isfinite(capturedWeaponWorld.translate.y) &&
            std::isfinite(capturedWeaponWorld.translate.z) &&
            std::isfinite(capturedWeaponWorld.scale) &&
            std::abs(capturedWeaponWorld.scale) > 0.0001f;
        for (int row = 0; capturedTransformFinite && row < 3; ++row) {
            for (int column = 0; capturedTransformFinite && column < 3; ++column) {
                capturedTransformFinite = std::isfinite(capturedWeaponWorld.rotate.entry[row][column]);
            }
        }
        if (!capturedTransformFinite) {
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
            if (!instance.body.isValid() || !instance.geometry) {
                continue;
            }
            if (!instance.geometry->localPointsGame.empty()) {
                for (const auto& point : instance.geometry->localPointsGame) {
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

    bool WeaponCollision::tryGetBodyTargetForDebug(
        const std::uint32_t bodyId,
        RE::NiTransform& outTarget) const
    {
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid() || !instance.geometry ||
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

    bool WeaponCollision::tryGetWeaponContactAtomic(std::uint32_t bodyId, WeaponInteractionContact& outContact) const
    {
        outContact = {};
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        for (int attempt = 0; attempt < 4; ++attempt) {
            const std::uint64_t startVersion = _published.version.load(std::memory_order_acquire);
            if ((startVersion & 1u) != 0) {
                continue;
            }

            WeaponInteractionContact candidate{};
            const std::uint32_t count = (std::min)(_published.count.load(std::memory_order_acquire), static_cast<std::uint32_t>(MAX_WEAPON_BODIES));
            bool found = false;
            for (std::uint32_t i = 0; i < count; ++i) {
                if (_published.ids[i].load(std::memory_order_acquire) != bodyId) {
                    continue;
                }

                candidate = readPublishedContact(i);
                found = true;
                break;
            }

            const std::uint64_t endVersion = _published.version.load(std::memory_order_acquire);
            if (startVersion == endVersion && (endVersion & 1u) == 0) {
                if (found) {
                    outContact = candidate;
                }
                return found;
            }
        }
        return false;
    }

    WeaponInteractionContact WeaponCollision::readPublishedContact(std::uint32_t i) const
    {
        // Only called inside a version-checked read with a validated index.
        WeaponInteractionContact contact{};
        contact.bodyId = _published.ids[i].load(std::memory_order_acquire);
        contact.valid = contact.bodyId != INVALID_BODY_ID;
        contact.partKind = static_cast<WeaponPartKind>(_published.partKinds[i].load(std::memory_order_acquire));
        contact.reloadRole = static_cast<WeaponReloadRole>(_published.reloadRoles[i].load(std::memory_order_acquire));
        contact.supportGripRole = static_cast<WeaponSupportGripRole>(_published.supportRoles[i].load(std::memory_order_acquire));
        contact.socketRole = static_cast<WeaponSocketRole>(_published.socketRoles[i].load(std::memory_order_acquire));
        contact.actionRole = static_cast<WeaponActionRole>(_published.actionRoles[i].load(std::memory_order_acquire));
        contact.fallbackGripPose = static_cast<WeaponGripPoseId>(_published.gripPoses[i].load(std::memory_order_acquire));
        contact.interactionRoot = reinterpret_cast<RE::NiAVObject*>(_published.interactionRoots[i].load(std::memory_order_acquire));
        contact.sourceRoot = reinterpret_cast<RE::NiAVObject*>(_published.sourceRoots[i].load(std::memory_order_acquire));
        contact.weaponGenerationKey = _published.generationKeys[i].load(std::memory_order_acquire);
        return contact;
    }

    std::size_t WeaponCollision::copyWeaponContactStatesAtomic(std::span<WeaponContactState> outStates) const
    {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const auto startVersion = _published.version.load(std::memory_order_acquire);
            if ((startVersion & 1u) != 0) {
                continue;
            }
            const auto count = (std::min)(_published.count.load(std::memory_order_acquire),
                static_cast<std::uint32_t>(MAX_WEAPON_BODIES));
            if (count > outStates.size()) {
                return 0;
            }
            for (std::uint32_t i = 0; i < count; ++i) {
                auto& state = outStates[i];
                state = {};
                state.contact = readPublishedContact(i);
                if (_published.sampledVelocityValid[i].load(std::memory_order_acquire) != 0) {
                    const float vx = _published.sampledVelocityHavokX[i].load(std::memory_order_acquire);
                    const float vy = _published.sampledVelocityHavokY[i].load(std::memory_order_acquire);
                    const float vz = _published.sampledVelocityHavokZ[i].load(std::memory_order_acquire);
                    if (std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz)) {
                        state.hasSampledVelocity = true;
                        state.sampledVelocityHavok = { vx, vy, vz, 0.0f };
                    }
                }
            }
            const auto endVersion = _published.version.load(std::memory_order_acquire);
            if (startVersion == endVersion && (endVersion & 1u) == 0) {
                return count;
            }
        }
        return 0;
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

        for (int attempt = 0; attempt < 4; ++attempt) {
            const std::uint64_t startVersion = _published.version.load(std::memory_order_acquire);
            if ((startVersion & 1u) != 0) {
                continue;
            }

            float vx = 0.0f;
            float vy = 0.0f;
            float vz = 0.0f;
            bool found = false;
            const std::uint32_t count = (std::min)(_published.count.load(std::memory_order_acquire), static_cast<std::uint32_t>(MAX_WEAPON_BODIES));
            for (std::uint32_t i = 0; i < count; ++i) {
                if (_published.ids[i].load(std::memory_order_acquire) != bodyId ||
                    _published.sampledVelocityValid[i].load(std::memory_order_acquire) == 0) {
                    continue;
                }

                vx = _published.sampledVelocityHavokX[i].load(std::memory_order_acquire);
                vy = _published.sampledVelocityHavokY[i].load(std::memory_order_acquire);
                vz = _published.sampledVelocityHavokZ[i].load(std::memory_order_acquire);
                found = true;
                break;
            }

            const std::uint64_t endVersion = _published.version.load(std::memory_order_acquire);
            if (startVersion != endVersion || (endVersion & 1u) != 0) {
                continue;
            }
            if (!found || !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz)) {
                return false;
            }

            outVelocityHavok[0] = vx;
            outVelocityHavok[1] = vy;
            outVelocityHavok[2] = vz;
            return true;
        }

        return false;
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
            if (!instance.body.isValid() || !instance.geometry || instance.body.getBodyId().value != bodyId) {
                continue;
            }

            outInfo.sourceName = instance.sourceName;
            outInfo.interactionRootName = instance.driveRootName;
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
        if (!instance.body.isValid() || !instance.geometry ||
            instance.geometry->mesh->localTrianglesGame.empty()) {
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
                !instance.geometry->mesh->sourceLocalTrianglesGame.empty() ?
            instance.geometry->mesh->sourceLocalTrianglesGame :
            instance.geometry->mesh->localTrianglesGame;
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
        if (getCurrentWeaponGenerationKey() == 0 ||
            !activeWeaponBodyRootMatches(currentWeaponRoot)) {
            return false;
        }

        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid() || !instance.geometry ||
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
        if (outViews.empty() || getCurrentWeaponGenerationKey() == 0 ||
            !activeWeaponBodyRootMatches(currentWeaponRoot)) {
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
        const auto pointFinite = [](const RE::NiPoint3& point) {
            return std::isfinite(point.x) &&
                   std::isfinite(point.y) &&
                   std::isfinite(point.z);
        };
        if (!currentWeaponRoot ||
            currentGeneration == 0 ||
            !activeWeaponBodyRootMatches(currentWeaponRoot) ||
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
            if (!instance.body.isValid() || !instance.geometry) {
                continue;
            }

            RE::NiTransform surfaceWorld = currentWeaponRoot->world;
            const bool sourceNodeCurrent = instance.sourceNode &&
                tryResolveDescendantWorldTransform(
                    currentWeaponRoot,
                    currentWeaponRoot->world,
                    instance.sourceNode,
                    surfaceWorld);
            const bool useSourceFrame =
                sourceNodeCurrent &&
                !instance.geometry->mesh->sourceLocalTrianglesGame.empty();
            if (!useSourceFrame) {
                surfaceWorld = currentWeaponRoot->world;
            }
            const auto& localTriangles =
                useSourceFrame ?
                instance.geometry->mesh->sourceLocalTrianglesGame :
                instance.geometry->mesh->localTrianglesGame;
            const RE::NiPoint3& boundsMin =
                useSourceFrame ?
                instance.generatedSourceLocalMinGame :
                instance.generatedLocalMinGame;
            const RE::NiPoint3& boundsMax =
                useSourceFrame ?
                instance.generatedSourceLocalMaxGame :
                instance.generatedLocalMaxGame;
            if (localTriangles.empty() ||
                !weaponTransformFinite(surfaceWorld) ||
                std::abs(surfaceWorld.scale) <= 0.000001f ||
                !pointFinite(boundsMin) ||
                !pointFinite(boundsMax) ||
                boundsMin.x > boundsMax.x ||
                boundsMin.y > boundsMax.y ||
                boundsMin.z > boundsMax.z) {
                continue;
            }

            const float absoluteScale = std::abs(surfaceWorld.scale);
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
                    witness.sourceNodeCurrent = useSourceFrame;
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
            readEquippedOmodsByAttachPointFormId(weaponSource(), &outComposition);
        outComposition.weaponGenerationKey = _identity.cachedBodySetKey;
        outComposition.weaponFormId = _identity.cachedFormID;

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
            if (!instance.body.isValid() || !instance.geometry) {
                continue;
            }

            RE::NiAVObject* interactionRoot = packageDriveRoot ? packageDriveRoot : instance.driveNode;
            WeaponCollisionProfileEvidenceDescriptor descriptor{};
            descriptor.valid = true;
            descriptor.bodyId = instance.body.getBodyId().value;
            descriptor.weaponGenerationKey = _identity.cachedBodySetKey;
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
            descriptor.localMeshPointsGame = copyLocalPoints(instance.geometry->localPointsGame);
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

    WeaponEmitterSnapshot WeaponCollision::buildWeaponEmitterSnapshot(
        RE::NiAVObject* weaponNode,
        std::uint64_t equippedWeaponKey,
        std::uint64_t weaponGenerationKey,
        std::uint64_t rootSetKey) const
    {
        const auto omodByAttachPointFormId = readEquippedOmodsByAttachPointFormId(weaponSource());
        auto snapshot = collectWeaponEmitterSnapshot(
            weaponNode,
            omodByAttachPointFormId,
            equippedWeaponKey,
            weaponGenerationKey,
            rootSetKey, !_physicalReference);

        ROCK_LOG_DEBUG(Weapon,
            "Weapon emitter snapshot discovered generation={:016X} emitters={} roots={:016X}",
            weaponGenerationKey,
            snapshot.count,
            rootSetKey);
        return snapshot;
    }

    void WeaponCollision::updateWeaponEmitterSnapshot(RE::NiAVObject* weaponNode, std::uint64_t equippedWeaponKey)
    {
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::WeaponEmitterRefresh);
        const std::uint64_t weaponGenerationKey = getCurrentWeaponGenerationKey();
        if (!weaponNode || equippedWeaponKey == 0 || weaponGenerationKey == 0 || _identity.cachedWeaponKey != equippedWeaponKey) {
            clearWeaponEmitterSnapshot();
            return;
        }

        const std::uint64_t rootSetKey = makeWeaponEmitterRootSetKey(weaponNode, !_physicalReference);
        WeaponEmitterSnapshot snapshot{};
        {
            std::scoped_lock lock(_evidence.mutex);
            snapshot = _evidence.emitters;
        }

        const bool discoveryRequired = snapshot.weaponGenerationKey != weaponGenerationKey ||
            snapshot.equippedWeaponKey != equippedWeaponKey ||
            snapshot.rootSetKey != rootSetKey ||
            snapshot.weaponRootAddress != reinterpret_cast<std::uintptr_t>(weaponNode);
        if (discoveryRequired) {
            snapshot = buildWeaponEmitterSnapshot(weaponNode, equippedWeaponKey, weaponGenerationKey, rootSetKey);
            for (auto& path : _emitterPaths) { path.transform.clear(); path.effect.clear(); }
        }
        std::array<RE::NiAVObject*, 4> roots{};
        std::size_t rootCount = 0;
        visitGeneratedWeaponMeshRootCandidates(weaponNode, [&](const WeaponMeshRootCandidate& candidate) {
            roots[rootCount++] = candidate.root;
        }, !_physicalReference);
        bool pathsValid = !discoveryRequired;
        for (std::size_t i = 0; i < snapshot.count; ++i) {
            auto& descriptor = snapshot.emitters[i];
            const auto& path = _emitterPaths[i];
            descriptor.active = false;
            descriptor.visible = false;
            auto* transform = path.transformRoot < rootCount ? path.transform.resolve(roots[path.transformRoot]) : nullptr;
            if (transform && reinterpret_cast<std::uintptr_t>(transform) == descriptor.transformNodeAddress) {
                descriptor.visible = weaponEmitterNodeEffectivelyVisible(transform);
                (void)updateWeaponEmitterTransform(descriptor, transform, weaponNode);
            } else {
                pathsValid = false;
            }
            if (descriptor.effectNodeAddress != 0) {
                auto* effect = path.effectRoot < rootCount ? path.effect.resolve(roots[path.effectRoot]) : nullptr;
                if (effect && reinterpret_cast<std::uintptr_t>(effect) == descriptor.effectNodeAddress)
                    descriptor.active = weaponEmitterNodeEffectivelyVisible(effect);
                else
                    pathsValid = false;
            }
        }
        if (!pathsValid && snapshot.count != 0) {
            // A changed path gets one bounded rediscovery pass for the whole
            // snapshot. No stored address is dereferenced, even during recovery.
            for (auto& path : _emitterPaths) { path.transform.clear(); path.effect.clear(); }
            for (std::size_t i = 0; i < snapshot.count; ++i) {
                snapshot.emitters[i].active = false;
                snapshot.emitters[i].visible = false;
            }
            for (std::size_t root = 0; root < rootCount; ++root) {
                auto remember = [&](std::size_t i, bool transform, RE::NiAVObject* node) {
                    auto& path = _emitterPaths[i];
                    if (transform) {
                        path.transform.capture(roots[root], node);
                        path.transformRoot = root;
                    } else {
                        path.effect.capture(roots[root], node);
                        path.effectRoot = root;
                    }
                };
                std::uint32_t visitedNodes = 0;
                refreshWeaponEmittersRecursive(roots[root], weaponNode, 0, visitedNodes, snapshot, remember);
            }
        }

        std::scoped_lock lock(_evidence.mutex);
        _evidence.emitters = snapshot;
    }

    void WeaponCollision::clearWeaponEmitterSnapshot()
    {
        for (auto& path : _emitterPaths) { path.transform.clear(); path.effect.clear(); }
        std::scoped_lock lock(_evidence.mutex);
        _evidence.emitters = {};
    }


    WeaponEvidenceSnapshot WeaponCollision::getProfileEvidenceDescriptors() const
    {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const std::uint64_t startVersion = _published.version.load(std::memory_order_acquire);
            if ((startVersion & 1u) != 0) {
                continue;
            }

            std::shared_ptr<const WeaponEvidenceSnapshot::Records> descriptors;
            {
                std::scoped_lock lock(_evidence.mutex);
                descriptors = _evidence.profileDescriptors;
            }

            const std::uint64_t endVersion = _published.version.load(std::memory_order_acquire);
            if (startVersion == endVersion && (endVersion & 1u) == 0) {
                return WeaponEvidenceSnapshot{ std::move(descriptors) };
            }
        }

        return {};
    }

    WeaponEmitterSnapshot WeaponCollision::getWeaponEmitterSnapshot() const
    {
        std::scoped_lock lock(_evidence.mutex);
        return _evidence.emitters;
    }

    WeaponCollision::NativeScopeSightAnchorSnapshot WeaponCollision::getNativeScopeSightAnchorSnapshot() const
    {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const std::uint64_t startVersion = _published.version.load(std::memory_order_acquire);
            if ((startVersion & 1u) != 0) {
                continue;
            }

            NativeScopeSightAnchorSnapshot snapshot{};
            {
                std::scoped_lock lock(_evidence.mutex);
                snapshot = _evidence.sightAnchor;
            }

            const std::uint64_t endVersion = _published.version.load(std::memory_order_acquire);
            if (startVersion == endVersion && (endVersion & 1u) == 0) {
                return snapshot;
            }
        }

        return {};
    }

    WeaponCollision::WeaponCompositionSnapshot
    WeaponCollision::getWeaponCompositionSnapshot() const
    {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const auto startVersion =
                _published.version.load(std::memory_order_acquire);
            if ((startVersion & 1u) != 0) {
                continue;
            }
            WeaponCompositionSnapshot snapshot{};
            {
                std::scoped_lock lock(_evidence.mutex);
                snapshot = _evidence.composition;
            }
            const auto endVersion =
                _published.version.load(std::memory_order_acquire);
            if (startVersion == endVersion && (endVersion & 1u) == 0) {
                return snapshot;
            }
        }
        return {};
    }

    bool WeaponCollision::tryFindInteractionContactNearPoint(
        const RE::NiAVObject* weaponNode,
        const RE::NiPoint3& probeWorldPoint,
        float probeRadiusGame,
        WeaponInteractionContact& outContact) const
    {
        InteractionQueryBatch batch;
        return tryFindInteractionContactNearPoint(weaponNode, probeWorldPoint, probeRadiusGame, outContact, batch);
    }

    bool WeaponCollision::tryFindInteractionContactNearPoint(
        const RE::NiAVObject* weaponNode,
        const RE::NiPoint3& probeWorldPoint,
        float probeRadiusGame,
        WeaponInteractionContact& outContact,
        InteractionQueryBatch& batch) const
    {
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::WeaponContactProbe);
        outContact = {};
        const auto currentGeneration = getCurrentWeaponGenerationKey();
        if (!weaponNode || currentGeneration == 0 || !activeWeaponBodyRootMatches(weaponNode) ||
            !weapon_interaction_query::finitePoint(probeWorldPoint) ||
            !std::isfinite(probeRadiusGame) || probeRadiusGame <= 0.0f) return false;

        if (batch.owner) {
            if (batch.owner != this || batch.root != weaponNode || batch.generation != currentGeneration) return false;
            performance_profiler::addCounter(performance_profiler::Counter::WeaponProbeBatchReuses);
        } else {
            performance_profiler::ScopedTimer prepareTimer(performance_profiler::Scope::WeaponProbePoseCapture);
            batch.owner = this;
            batch.root = weaponNode;
            batch.generation = currentGeneration;
            const RE::NiTransform rootWorld = weaponNode->world;
            for (const auto& instance : activeWeaponBodies()) {
                if (!instance.body.isValid() || !instance.geometry || !instance.indices) continue;
                RE::NiTransform probeWorld = rootWorld;
                const bool sourceCurrent = instance.sourceNode &&
                    tryResolveDescendantWorldTransform(weaponNode, rootWorld, instance.sourceNode, probeWorld);
                const bool useSourceFrame = sourceCurrent && !instance.geometry->mesh->sourceLocalTrianglesGame.empty();
                if (!useSourceFrame) probeWorld = rootWorld;
                const auto& triangles = useSourceFrame ? instance.geometry->mesh->sourceLocalTrianglesGame :
                    instance.geometry->mesh->localTrianglesGame;
                const auto& index = useSourceFrame ? instance.indices->sourceIndex : instance.indices->localIndex;
                const auto& boundsMin = useSourceFrame ? instance.generatedSourceLocalMinGame : instance.generatedLocalMinGame;
                const auto& boundsMax = useSourceFrame ? instance.generatedSourceLocalMaxGame : instance.generatedLocalMaxGame;
                if (weapon_interaction_query::prepare(triangles, index, probeWorld, boundsMin, boundsMax,
                        instance.semantic.priority, batch.parts[batch.count])) {
                    batch.instances[batch.count++] = &instance;
                }
            }
            performance_profiler::addCounter(performance_profiler::Counter::WeaponProbePoseBatches);
            performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponProbePoseParts, batch.count);
        }

        const auto selection = weapon_interaction_query::find(
            {batch.parts.data(), batch.count}, probeWorldPoint, probeRadiusGame);
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponProbeBoundsCandidates, selection.boundsCandidates);
        performance_profiler::observeValue(performance_profiler::ValueMetric::WeaponProbeSurfaceCandidates, selection.surfaceCandidates);
        if (!selection.valid() || getCurrentWeaponGenerationKey() != currentGeneration) return false;
        const auto* bestInstance = batch.instances[selection.part];
        if (!bestInstance->body.isValid()) return false;
        const auto& bestRank = selection.rank;
        const auto boundsCandidateCount = selection.boundsCandidates;
        const auto surfaceCandidateCount = selection.surfaceCandidates;
        const auto* packageDriveRoot = weaponNode;

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
