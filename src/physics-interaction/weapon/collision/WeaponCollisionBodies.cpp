#include "physics-interaction/weapon/WeaponCollisionInternal.h"
#include "physics-interaction/native/ShellCasingGrace.h"

// Weapon body banks: lifecycle, retirement, collision enablement, atomic body-id publication, drive queueing, and retired-body servicing.

namespace rock
{
    WeaponCollision::WeaponCollision() { clearAtomicBodyIds(); }

    WeaponCollision::WeaponBodyBank& WeaponCollision::activeWeaponBodies()
    {
        return _bodies.usingReplacementBank ? _bodies.replacementBank : _bodies.bank;
    }

    const WeaponCollision::WeaponBodyBank& WeaponCollision::activeWeaponBodies() const
    {
        return _bodies.usingReplacementBank ? _bodies.replacementBank : _bodies.bank;
    }

    WeaponCollision::WeaponBodyBank& WeaponCollision::inactiveWeaponBodies()
    {
        return _bodies.usingReplacementBank ? _bodies.bank : _bodies.replacementBank;
    }

    bool WeaponCollision::bankHasWeaponBody(const WeaponBodyBank& bank)
    {
        return std::any_of(bank.begin(), bank.end(), [](const WeaponBodyInstance& instance) {
            return instance.body.isValid();
        });
    }

    std::uint32_t WeaponCollision::bankWeaponBodyCount(const WeaponBodyBank& bank)
    {
        return static_cast<std::uint32_t>(std::count_if(bank.begin(), bank.end(), [](const WeaponBodyInstance& instance) {
            return instance.body.isValid();
        }));
    }

    RE::NiAVObject* WeaponCollision::resolvePackageDriveNode(const WeaponBodyBank& bank, RE::NiAVObject* fallbackWeaponNode)
    {
        if (fallbackWeaponNode) {
            return fallbackWeaponNode;
        }

        for (const auto& instance : bank) {
            if (instance.body.isValid() && instance.driveNode) {
                return instance.driveNode;
            }
        }
        return nullptr;
    }

    bool WeaponCollision::activeWeaponBodyRootMatches(const RE::NiAVObject* currentWeaponRoot) const
    {
        if (!currentWeaponRoot) {
            return false;
        }

        bool foundBody = false;
        for (const auto& instance : activeWeaponBodies()) {
            if (!instance.body.isValid()) {
                continue;
            }
            foundBody = true;
            if (instance.driveNode != currentWeaponRoot) {
                return false;
            }
        }
        return foundBody;
    }

    bool WeaponCollision::retireActiveWeaponBodiesForSceneTransition(RE::hknpWorld* world, const char* reason)
    {
        if (!world || getCurrentWeaponGenerationKey() == 0 || getWeaponBodyCount() == 0 ||
            !bankHasWeaponBody(activeWeaponBodies())) {
            return false;
        }

        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        const std::uint32_t retiredBodyCount = bankWeaponBodyCount(activeWeaponBodies());

        /*
         * The engine owns the equipped weapon scene graph. Once its identity,
         * instance, or root changes, every cached NiAVObject pointer from the
         * previous graph becomes invalid immediately. Remove the generated body
         * bank from all collision and publication paths before visual
         * stabilization or staged body creation can return.
         * Native body payloads still use the normal delayed reclamation path.
         */
        clearAtomicBodyIds();
        resetWeaponBodySetGeneration();
        destroyWeaponBodyBank(activeWeaponBodies(), true);
        clearGeneratedSourceCompletenessTracking();
        clearPendingWeaponVisualRebuild();
        clearGeneratedSourceCache();
        resetVisualSourceUnavailableRetention();
        _sources.detachedExclusionEquippedKey = 0;
        _sources.detachedExclusionGroups.clear();

        ROCK_LOG_INFO(Weapon,
            "Retired generated weapon bodies before scene transition reason={} bodies={} cached(identity/ownership/form)=({:016X}/{:016X}/{:08X}) observed(identity/ownership/form)=({:016X}/{:016X}/{:08X})",
            reason ? reason : "unknown",
            retiredBodyCount,
            _identity.cachedIdentityKey,
            _identity.cachedOwnershipKey,
            _identity.cachedFormID,
            _identity.observedIdentityKey,
            _identity.observedOwnershipKey,
            _identity.observedFormID);
        return true;
    }

    bool WeaponCollision::hasWeaponBody() const
    {
        return bankHasWeaponBody(activeWeaponBodies());
    }

    std::uint32_t WeaponCollision::getWeaponBodyCount() const
    {
        return _published.count.load(std::memory_order_acquire);
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
        WeaponBodySnapshot snapshot{};
        snapshot.bodyIds.fill(INVALID_BODY_ID);

        for (int attempt = 0; attempt < 4; ++attempt) {
            const std::uint64_t startVersion = _published.version.load(std::memory_order_acquire);
            if ((startVersion & 1u) != 0) {
                continue;
            }

            WeaponBodySnapshot candidate{};
            candidate.bodyIds.fill(INVALID_BODY_ID);
            candidate.generationKey = _published.setKey.load(std::memory_order_acquire);
            candidate.count = (std::min)(_published.count.load(std::memory_order_acquire), static_cast<std::uint32_t>(MAX_WEAPON_BODIES));
            for (std::uint32_t i = 0; i < candidate.count; ++i) {
                candidate.bodyIds[i] = _published.ids[i].load(std::memory_order_acquire);
            }

            const std::uint64_t endVersion = _published.version.load(std::memory_order_acquire);
            if (startVersion == endVersion && (endVersion & 1u) == 0) {
                return candidate;
            }
        }

        return snapshot;
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

    BethesdaPhysicsBody& WeaponCollision::getWeaponBody()
    {
        for (auto& instance : activeWeaponBodies()) {
            if (instance.body.isValid()) {
                return instance.body;
            }
        }
        return activeWeaponBodies()[0].body;
    }

    void WeaponCollision::destroyWeaponBody(RE::hknpWorld* world)
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        if (!bankHasWeaponBody(_bodies.bank) && !bankHasWeaponBody(_bodies.replacementBank)) {
            clearGeneratedSourceCompletenessTracking();
            clearPendingWeaponVisualRebuild();
            clearGeneratedSourceCache();
            clearPendingGeneratedWeaponBuild(world, false);
            _drive.rebuildRequested.store(false, std::memory_order_release);
            _drive.failureCount.store(0, std::memory_order_release);
            return;
        }

        clearAtomicBodyIds();
        resetWeaponBodySetGeneration();

        const auto activeDestroyed = bankWeaponBodyCount(activeWeaponBodies());
        const auto inactiveDestroyed = bankWeaponBodyCount(inactiveWeaponBodies());
        destroyWeaponBodyBank(activeWeaponBodies(), true);
        destroyWeaponBodyBank(inactiveWeaponBodies(), true);
        _bodies.usingReplacementBank = false;
        clearGeneratedSourceCompletenessTracking();
        clearPendingWeaponVisualRebuild();
        clearGeneratedSourceCache();
        clearPendingGeneratedWeaponBuild(world, false);
        _drive.rebuildRequested.store(false, std::memory_order_release);
        _drive.failureCount.store(0, std::memory_order_release);

        ROCK_LOG_INFO(Weapon, "Weapon collision bodies destroyed count={}", activeDestroyed + inactiveDestroyed);
    }

    void WeaponCollision::invalidateForScaleChange(RE::hknpWorld* world)
    {
        const bool hadWeaponBody = hasWeaponBody();
        if (hadWeaponBody) {
            ROCK_LOG_INFO(Weapon, "Generated weapon collision invalidated by physics scale change");
            destroyWeaponBody(world);
        } else {
            clearAtomicBodyIds();
            resetWeaponBodySetGeneration();
            ROCK_LOG_DEBUG(Weapon, "Generated weapon collision scale invalidation had no active bodies");
        }

        _identity.cachedWeaponKey = 0;
        _identity.cachedVisualKey = 0;
        _identity.cachedIdentityKey = 0;
        _identity.cachedOwnershipKey = 0;
        _identity.cachedFormID = 0;
        _identity.observedIdentityKey = 0;
        _identity.observedOwnershipKey = 0;
        _identity.observedFormID = 0;
        _identity.observedInstanceContentKey = 0;
        clearGeneratedSourceCompletenessTracking();
        clearPendingWeaponVisualRebuild();
        clearGeneratedSourceCache();
        clearPendingGeneratedWeaponBuild(world, true);
        resetVisualSourceUnavailableRetention();
        _drive.rebuildRequested.store(false, std::memory_order_release);
        _drive.failureCount.store(0, std::memory_order_release);
    }

    void WeaponCollision::destroyWeaponBodyBank(WeaponBodyBank& bank, bool releaseShapeRef)
    {
        for (auto& instance : bank) {
            retireWeaponBodyInstance(instance, releaseShapeRef);
        }
    }

    void WeaponCollision::retireWeaponBodyInstance(WeaponBodyInstance& instance, bool releaseShapeRef)
    {
        if (instance.body.isValid()) {
            RetiredBethesdaPhysicsBodyPayload payload{};
            if (instance.body.retireFromWorld(_cachedBhkWorld, payload) && payload.occupied()) {
                retireWeaponBodyPayload(payload);
            } else {
                /*
                 * Releasing a generated wrapper immediately after a rebuild was
                 * observed to leave native readers with stale collision-object
                 * pointers. If retirement cannot produce a payload, clear ROCK's
                 * ownership without calling the immediate destructor path.
                 */
                ROCK_LOG_ERROR(Weapon,
                    "Generated weapon body {} could not be retired; wrapper ownership cleared without immediate native release",
                    instance.body.getBodyId().value);
            }
        }
        clearWeaponBodyInstance(instance, releaseShapeRef);
    }

    void WeaponCollision::retireWeaponBodyPayload(RetiredBethesdaPhysicsBodyPayload& payload)
    {
        if (!payload.occupied()) {
            return;
        }

        std::scoped_lock lock(_bodies.retiredPayloadMutex);
        for (auto& retired : _bodies.retiredPayloads) {
            if (!retired.occupied()) {
                retired.bodyPayload = payload;
                retired.remainingPhysicsSteps = RETIRED_GENERATED_WEAPON_BODY_GRACE_STEPS;
                retired.processLifetimeHold = false;
                ++_bodies.retiredPayloadCount;
                ROCK_LOG_SAMPLE_DEBUG(Weapon,
                    1000,
                    "Generated weapon body {} payload retired for {} physics steps activeRetired={}",
                    payload.bodyId,
                    RETIRED_GENERATED_WEAPON_BODY_GRACE_STEPS,
                    _bodies.retiredPayloadCount);
                payload = {};
                return;
            }
        }

        BethesdaPhysicsBody::retainRetiredPayloadForProcessLifetime(
            payload,
            "weapon-body-queue",
            _bodies.retiredPayloads.size());
    }

    void WeaponCollision::setWeaponBodyBankCollisionEnabled(RE::hknpWorld* world, WeaponBodyBank& bank, bool enabled)
    {
        if (!world) {
            return;
        }

        const std::uint32_t filterInfo = generatedWeaponCollisionFilterInfo(enabled);
        for (auto& instance : bank) {
            if (instance.body.isValid()) {
                body_collision::setFilterInfo(world, instance.body.getBodyId(), filterInfo);
            }
        }
    }

    void WeaponCollision::clearWeaponBodyInstance(WeaponBodyInstance& instance, bool releaseShapeRef)
    {
        if (releaseShapeRef && instance.ownsShapeRef && instance.shape) {
            havok_ref_count::release(instance.shape);
        }
        instance.body.reset();
        instance.shape = nullptr;
        instance.driveNode = nullptr;
        instance.sourceNode = nullptr;
        instance.driveNodeRef.reset();
        instance.sourceNodeRef.reset();
        instance.sourceName.clear();
        instance.driveRootName.clear();
        instance.sourceRootName.clear();
        instance.generatedLocalCenterGame = {};
        instance.generatedSourceLocalCenterGame = {};
        instance.generatedLocalMinGame = {};
        instance.generatedLocalMaxGame = {};
        instance.generatedSourceLocalMinGame = {};
        instance.generatedSourceLocalMaxGame = {};
        instance.geometry.reset();
        instance.indices.reset();
        instance.generatedPointCount = 0;
        instance.generatedSourceGroupId = 0;
        instance.semantic = {};
        instance.ownsShapeRef = false;
        clearGeneratedKeyframedBodyDriveState(instance.driveState);
        instance.publicationIndex = INVALID_BODY_ID;
    }

    void WeaponCollision::beginWeaponBodyPublication()
    {
        const std::uint64_t version = _published.version.load(std::memory_order_relaxed);
        _published.version.store((version & ~1ull) + 1ull, std::memory_order_release);
    }

    void WeaponCollision::endWeaponBodyPublication()
    {
        const std::uint64_t version = _published.version.load(std::memory_order_relaxed);
        _published.version.store((version | 1ull) + 1ull, std::memory_order_release);
        const auto snapshot = getWeaponBodySnapshotAtomic();
        shell_casing_grace::publishWeapon(_identity.cachedFormID, snapshot.generationKey,
            std::span(snapshot.bodyIds).first(snapshot.count));
    }

    void WeaponCollision::clearAtomicBodyIds()
    {
        beginWeaponBodyPublication();
        _published.count.store(0, std::memory_order_release);
        _published.setKey.store(0, std::memory_order_release);
        for (auto& id : _published.ids) {
            id.store(INVALID_BODY_ID, std::memory_order_release);
        }
        for (auto& value : _published.partKinds) {
            value.store(static_cast<std::uint32_t>(WeaponPartKind::Other), std::memory_order_release);
        }
        for (auto& value : _published.reloadRoles) {
            value.store(static_cast<std::uint32_t>(WeaponReloadRole::None), std::memory_order_release);
        }
        for (auto& value : _published.supportRoles) {
            value.store(static_cast<std::uint32_t>(WeaponSupportGripRole::None), std::memory_order_release);
        }
        for (auto& value : _published.socketRoles) {
            value.store(static_cast<std::uint32_t>(WeaponSocketRole::None), std::memory_order_release);
        }
        for (auto& value : _published.actionRoles) {
            value.store(static_cast<std::uint32_t>(WeaponActionRole::None), std::memory_order_release);
        }
        for (auto& value : _published.gripPoses) {
            value.store(static_cast<std::uint32_t>(WeaponGripPoseId::None), std::memory_order_release);
        }
        for (auto& value : _published.interactionRoots) {
            value.store(0, std::memory_order_release);
        }
        for (auto& value : _published.sourceRoots) {
            value.store(0, std::memory_order_release);
        }
        for (auto& value : _published.generationKeys) {
            value.store(0, std::memory_order_release);
        }
        for (auto& value : _published.sampledVelocityHavokX) {
            value.store(0.0f, std::memory_order_release);
        }
        for (auto& value : _published.sampledVelocityHavokY) {
            value.store(0.0f, std::memory_order_release);
        }
        for (auto& value : _published.sampledVelocityHavokZ) {
            value.store(0.0f, std::memory_order_release);
        }
        for (auto& value : _published.sampledVelocityValid) {
            value.store(0, std::memory_order_release);
        }
        {
            std::scoped_lock lock(_evidence.mutex);
            _evidence.profileDescriptors.reset();
            _evidence.emitters = {};
            _evidence.sightAnchor = {};
            _evidence.composition = {};
        }
        endWeaponBodyPublication();
    }

    void WeaponCollision::unpublishAtomicBodyIds()
    {
        clearAtomicBodyIds();
    }

    void WeaponCollision::publishAtomicBodyIds(WeaponBodyBank& bank)
    {
        WeaponCompositionSnapshot weaponCompositionSnapshot{};
        auto evidenceSnapshot = buildProfileEvidenceSnapshot(
            bank,
            weaponCompositionSnapshot);
        weaponCompositionSnapshot.publicationSequence =
            ++_evidence.compositionPublicationSequence;
        RE::NiAVObject* packageDriveNode = resolvePackageDriveNode(bank, nullptr);
        NativeScopeSightAnchorSnapshot nativeScopeSightAnchorSnapshot = buildNativeScopeSightAnchorSnapshot(
            _identity.cachedBodySetKey,
            _identity.cachedOwnershipKey,
            _identity.cachedFormID,
            evidenceSnapshot);
        const auto manualScopeTarget = resolveEquippedManualScopeTarget(packageDriveNode);
        nativeScopeSightAnchorSnapshot.scopeWeaponIdentity = manualScopeTarget.weaponIdentity;
        nativeScopeSightAnchorSnapshot.scopeInstanceIdentity = manualScopeTarget.instanceIdentity;
        nativeScopeSightAnchorSnapshot.scopeEligible = manualScopeTarget.scopeEligible;
        nativeScopeSightAnchorSnapshot.nativeScopeOverlayValid = manualScopeTarget.overlayValid;
        nativeScopeSightAnchorSnapshot.nativeScopeOverlayIndex = manualScopeTarget.overlayIndex;
        nativeScopeSightAnchorSnapshot.manualDirectTransitionRequired =
            manualScopeTarget.directTransitionRequired;
        auto publishedEvidence = std::make_shared<const WeaponEvidenceSnapshot::Records>(std::move(evidenceSnapshot));
        std::uint32_t count = 0;
        beginWeaponBodyPublication();
        _published.count.store(0, std::memory_order_release);
        for (auto& id : _published.ids) {
            id.store(INVALID_BODY_ID, std::memory_order_release);
        }
        for (auto& value : _published.sampledVelocityValid) {
            value.store(0, std::memory_order_release);
        }
        _published.setKey.store(_identity.cachedBodySetKey, std::memory_order_release);
        {
            std::scoped_lock lock(_evidence.mutex);
            _evidence.profileDescriptors = std::move(publishedEvidence);
            _evidence.sightAnchor = nativeScopeSightAnchorSnapshot;
            _evidence.composition = weaponCompositionSnapshot;
        }
        for (auto& instance : bank) {
            if (instance.body.isValid() && count < MAX_WEAPON_BODIES) {
                instance.publicationIndex = count;
                _published.partKinds[count].store(static_cast<std::uint32_t>(instance.semantic.partKind), std::memory_order_release);
                _published.reloadRoles[count].store(static_cast<std::uint32_t>(instance.semantic.reloadRole), std::memory_order_release);
                _published.supportRoles[count].store(static_cast<std::uint32_t>(instance.semantic.supportGripRole), std::memory_order_release);
                _published.socketRoles[count].store(static_cast<std::uint32_t>(instance.semantic.socketRole), std::memory_order_release);
                _published.actionRoles[count].store(static_cast<std::uint32_t>(instance.semantic.actionRole), std::memory_order_release);
                _published.gripPoses[count].store(static_cast<std::uint32_t>(instance.semantic.fallbackGripPose), std::memory_order_release);
                _published.interactionRoots[count].store(reinterpret_cast<std::uintptr_t>(packageDriveNode), std::memory_order_release);
                _published.sourceRoots[count].store(reinterpret_cast<std::uintptr_t>(instance.sourceNode), std::memory_order_release);
                _published.generationKeys[count].store(_identity.cachedBodySetKey, std::memory_order_release);
                _published.ids[count].store(instance.body.getBodyId().value, std::memory_order_release);
                ++count;
            } else {
                instance.publicationIndex = INVALID_BODY_ID;
            }
        }
        _published.count.store(count, std::memory_order_release);
        endWeaponBodyPublication();
        dumpEquippedWeaponOmodEvidence(bank, packageDriveNode);
    }

    void WeaponCollision::publishSampledVelocityAtomic(std::uint32_t publicationIndex, const GeneratedKeyframedBodyDriveQueueResult& queueResult)
    {
        if (publicationIndex >= MAX_WEAPON_BODIES || publicationIndex >= _published.count.load(std::memory_order_acquire)) {
            return;
        }

        if (!queueResult.sampledVelocityValid) {
            _published.sampledVelocityValid[publicationIndex].store(0, std::memory_order_release);
            _published.sampledVelocityHavokX[publicationIndex].store(0.0f, std::memory_order_release);
            _published.sampledVelocityHavokY[publicationIndex].store(0.0f, std::memory_order_release);
            _published.sampledVelocityHavokZ[publicationIndex].store(0.0f, std::memory_order_release);
            return;
        }

        _published.sampledVelocityHavokX[publicationIndex].store(queueResult.sampledLinearVelocityHavok.x, std::memory_order_release);
        _published.sampledVelocityHavokY[publicationIndex].store(queueResult.sampledLinearVelocityHavok.y, std::memory_order_release);
        _published.sampledVelocityHavokZ[publicationIndex].store(queueResult.sampledLinearVelocityHavok.z, std::memory_order_release);
        _published.sampledVelocityValid[publicationIndex].store(1, std::memory_order_release);
    }

    void WeaponCollision::queueBodyTarget(WeaponBodyInstance& instance, const RE::NiTransform& weaponTransform, float sourceDeltaSeconds)
    {
        if (!instance.body.isValid()) {
            return;
        }

        const auto queueResult = queueGeneratedKeyframedBodyTarget(instance.driveState, weaponTransform, sourceDeltaSeconds, 1000.0f);
        publishSampledVelocityAtomic(instance.publicationIndex, queueResult);
    }

    void WeaponCollision::serviceRetiredWeaponBodies(
        RE::hknpWorld* currentWorld,
        std::uint32_t completedPhysicsSteps)
    {
        if (!currentWorld || completedPhysicsSteps == 0) {
            return;
        }

        std::scoped_lock lock(_bodies.retiredPayloadMutex);
        for (auto& retired : _bodies.retiredPayloads) {
            if (!retired.occupied()) {
                continue;
            }
            if (retired.processLifetimeHold) {
                continue;
            }
            if (retired.bodyPayload.retiredHknpWorld != currentWorld) {
                retired.processLifetimeHold = true;
                ROCK_LOG_SAMPLE_WARN(
                    Weapon,
                    1000,
                    "Retired generated weapon body {} belongs to a departed Havok world; retaining its complete native payload for process lifetime",
                    retired.bodyPayload.bodyId);
                continue;
            }

            retired.remainingPhysicsSteps =
                retired.remainingPhysicsSteps > completedPhysicsSteps ? retired.remainingPhysicsSteps - completedPhysicsSteps : 0;
            if (retired.remainingPhysicsSteps != 0) {
                continue;
            }

            const auto bodyId = retired.bodyPayload.bodyId;
            if (!BethesdaPhysicsBody::quarantineRetiredPayload(retired.bodyPayload)) {
                retired.processLifetimeHold = true;
                continue;
            }
            retired = {};
            if (_bodies.retiredPayloadCount > 0) {
                --_bodies.retiredPayloadCount;
            }
            ROCK_LOG_SAMPLE_DEBUG(Weapon,
                1000,
                "Retired generated weapon body {} transferred to collision-object quarantine activeRetired={}",
                bodyId,
                _bodies.retiredPayloadCount);
        }
    }
}
