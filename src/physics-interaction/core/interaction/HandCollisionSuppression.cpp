#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/native/NativePlayerCollisionFilter.h"

// Hand and native-player collision suppression: contact evidence ownership, dominant-weapon and weapon-support hand suppression, post-drop suppression, and native player physical-pair ownership.

namespace rock
{
    void PhysicsInteraction::clearLeftWeaponContact()
    {
        _weaponContact.left.bodyId.store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
        _weaponContact.left.partKind.store(static_cast<std::uint32_t>(WeaponPartKind::Other), std::memory_order_release);
        _weaponContact.left.reloadRole.store(static_cast<std::uint32_t>(WeaponReloadRole::None), std::memory_order_release);
        _weaponContact.left.supportRole.store(static_cast<std::uint32_t>(WeaponSupportGripRole::None), std::memory_order_release);
        _weaponContact.left.socketRole.store(static_cast<std::uint32_t>(WeaponSocketRole::None), std::memory_order_release);
        _weaponContact.left.actionRole.store(static_cast<std::uint32_t>(WeaponActionRole::None), std::memory_order_release);
        _weaponContact.left.gripPose.store(static_cast<std::uint32_t>(WeaponGripPoseId::None), std::memory_order_release);
        _weaponContact.left.missedFrames.store(WEAPON_CONTACT_TIMEOUT_FRAMES + 1, std::memory_order_release);
        _equipped.weaponInteractionAcquisitionStates[0] = {};
    }

    void PhysicsInteraction::clearRightWeaponContact()
    {
        _weaponContact.right.bodyId.store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
        _weaponContact.right.partKind.store(static_cast<std::uint32_t>(WeaponPartKind::Other), std::memory_order_release);
        _weaponContact.right.reloadRole.store(static_cast<std::uint32_t>(WeaponReloadRole::None), std::memory_order_release);
        _weaponContact.right.supportRole.store(static_cast<std::uint32_t>(WeaponSupportGripRole::None), std::memory_order_release);
        _weaponContact.right.socketRole.store(static_cast<std::uint32_t>(WeaponSocketRole::None), std::memory_order_release);
        _weaponContact.right.actionRole.store(static_cast<std::uint32_t>(WeaponActionRole::None), std::memory_order_release);
        _weaponContact.right.gripPose.store(static_cast<std::uint32_t>(WeaponGripPoseId::None), std::memory_order_release);
        _weaponContact.right.missedFrames.store(WEAPON_CONTACT_TIMEOUT_FRAMES + 1, std::memory_order_release);
        _equipped.weaponInteractionAcquisitionStates[1] = {};
    }

    bool PhysicsInteraction::isHandContactEvidenceSuppressed(bool isLeft) const
    {
        /*
         * Native hknp contact callbacks can run on the physics boundary while
         * game-frame ownership is changing. Use only atomic state here: the
         * physics thread needs ROCK's "hand collision disabled while owned"
         * answer without reading Hand::_state directly.
         */
        const Hand& hand = isLeft ? _leftHand : _rightHand;
        return hand.hasContactEvidenceSuppressedAtomic() ||
               (!isLeft && weaponHandSuppressedFor(false, WeaponHandReason::Dominant)) ||
               weaponHandSuppressedFor(isLeft, WeaponHandReason::Grip);
    }

    void PhysicsInteraction::clearContactEvidenceForHand(bool isLeft)
    {
        if (isLeft) {
            _leftHand.clearSemanticContactEvidence();
        } else {
            _rightHand.clearSemanticContactEvidence();
        }
    }

    void PhysicsInteraction::synchronizeContactEvidenceOwnership(bool rightHandWeaponAuthorityActive, bool leftSupportGripActive, bool rightPartGripActive)
    {
        /*
         * ROCK disables generated hand collision when a grab or two-hand/tool
         * owner has the hand. Clear semantic contact state at the same
         * authority transition so callbacks cannot leave a stale touch owner.
         */
        if (_rightHand.hasContactEvidenceSuppressedAtomic() || rightHandWeaponAuthorityActive || rightPartGripActive ||
            weaponHandSuppressedFor(false, WeaponHandReason::Dominant) ||
            weaponHandSuppressedFor(false, WeaponHandReason::Grip)) {
            clearContactEvidenceForHand(false);
        }

        if (_leftHand.hasContactEvidenceSuppressedAtomic() || leftSupportGripActive ||
            weaponHandSuppressedFor(true, WeaponHandReason::Grip)) {
            clearContactEvidenceForHand(true);
        }
    }

    /*
     * While a hand owns the equipped weapon (dominant firing hand, a part or
     * support grip, or the moment after it drops the weapon), its generated
     * collider bodies are switched to no-collide so they cannot become a
     * second physical owner of the weapon or feed stale touch contacts.
     * One lease set per hand serves every reason; the hand is restored when
     * no reason remains and any requested restore delay has elapsed.
     */
    void PhysicsInteraction::setWeaponHandSuppression(
        RE::hknpWorld* world, bool isLeft, WeaponHandReason reason, bool active, float restoreDelaySeconds)
    {
        auto& state = _suppression.weaponHands[isLeft ? 1 : 0];
        const auto bit = static_cast<std::uint8_t>(reason);
        if (active) {
            state.active |= bit;
            state.lingering &= static_cast<std::uint8_t>(~bit);
        } else if ((state.active & bit) != 0) {
            state.active &= static_cast<std::uint8_t>(~bit);
            if (restoreDelaySeconds > 0.0f) {
                state.lingering |= bit;
                state.restoreSeconds = (std::max)(state.restoreSeconds, restoreDelaySeconds);
            }
        }
        refreshWeaponHandSuppression(world, isLeft);
    }

    void PhysicsInteraction::clearWeaponHandSuppression(RE::hknpWorld* world, bool isLeft)
    {
        auto& state = _suppression.weaponHands[isLeft ? 1 : 0];
        state.active = 0;
        state.lingering = 0;
        state.restoreSeconds = 0.0f;
        refreshWeaponHandSuppression(world, isLeft);
    }

    void PhysicsInteraction::abandonWeaponHandSuppression()
    {
        for (auto& state : _suppression.weaponHands) {
            state.leases.clearTracking();
            state.active = 0;
            state.lingering = 0;
            state.restoreSeconds = 0.0f;
            state.published.store(0, std::memory_order_release);
        }
    }

    void PhysicsInteraction::updateWeaponHandSuppression(RE::hknpWorld* world, float deltaSeconds)
    {
        for (const bool isLeft : { false, true }) {
            auto& state = _suppression.weaponHands[isLeft ? 1 : 0];
            if (state.active == 0 && state.lingering != 0) {
                state.restoreSeconds -= deltaSeconds;
                if (state.restoreSeconds <= 0.0f) {
                    state.lingering = 0;
                    state.restoreSeconds = 0.0f;
                }
            }
            refreshWeaponHandSuppression(world, isLeft);
        }
    }

    void PhysicsInteraction::refreshWeaponHandSuppression(RE::hknpWorld* world, bool isLeft)
    {
        auto& state = _suppression.weaponHands[isLeft ? 1 : 0];
        const std::uint8_t reasons = state.active | state.lingering;
        state.published.store(reasons, std::memory_order_release);
        // Without a world the leases are kept until it can be restored.
        if (!world) {
            return;
        }

        if (reasons == 0) {
            if (!state.leases.empty() && !state.leases.releaseAll(world, "weapon-hand", [](std::uint32_t, const auto&) {})) {
                ROCK_LOG_WARN(Weapon, "{} hand weapon collision restore deferred; leases preserved", isLeft ? "Left" : "Right");
            }
            return;
        }

        Hand& hand = isLeft ? _leftHand : _rightHand;
        std::array<std::uint32_t, kGrabCollisionSuppressionBodyCountPerHand> bodies{};
        std::size_t count = 0;
        const std::uint32_t colliderCount = hand.getHandColliderBodyCount();
        if (colliderCount > 0) {
            for (std::uint32_t i = 0; i < colliderCount && count < bodies.size(); ++i) {
                bodies[count++] = hand.getHandColliderBodyIdAtomic(i);
            }
        } else if (hand.hasCollisionBody()) {
            bodies[count++] = hand.getCollisionBodyId().value;
        }
        if ((reasons & static_cast<std::uint8_t>(WeaponHandReason::Drop)) != 0 && count < bodies.size()) {
            count += _bodyBoneColliders.copyGrabSuppressionArmBodyIdsAtomic(isLeft, bodies.data() + count, bodies.size() - count);
        }

        // Recreated hand bodies get new IDs: drop leases on IDs that are gone.
        const auto current = std::span(bodies).first(count);
        state.leases.releaseWhere(world, "weapon-hand-stale",
            [&](std::uint32_t bodyId) { return std::find(current.begin(), current.end(), bodyId) == current.end(); },
            [](std::uint32_t, const auto&) {});
        for (const auto bodyId : current) {
            if (bodyId != INVALID_CONTACT_BODY_ID && !state.leases.contains(bodyId)) {
                (void)state.leases.acquire(world, bodyId, "weapon-hand");
            }
        }
    }

    void PhysicsInteraction::captureHandColliderBones()
    {
        performance_profiler::ScopedTimer captureTimer(performance_profiler::Scope::HandBoneCapture);
        (void)_handColliderBoneReader.capture(
            skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
            skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
            SkeletonBoneCaptureSpace::Controller,
            _handColliderBoneSnapshot);
    }

    void PhysicsInteraction::updateHandCollisions(const PhysicsFrameContext& frame)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::HandColliderUpdate);

        if (!runtime_state::isLocalSkeletonReady()) {
            return;
        }

        auto* world = frame.hknpWorld;

        if (!_rightHand.hasCollisionBody() || !_leftHand.hasCollisionBody()) {
            if (frame.reloadBoundaryActive) {
                return;
            }
            if (_lifecycle.handColliderCreateRetryFrames > 0) {
                --_lifecycle.handColliderCreateRetryFrames;
                return;
            }

            ROCK_LOG_WARN(Hand,
                "Hand collider runtime missing generated bodies; recreating rightBody={} leftBody={}",
                _rightHand.hasCollisionBody() ? "yes" : "no",
                _leftHand.hasCollisionBody() ? "yes" : "no");
            destroyHandCollisions(frame.bhkWorld);
            if (createHandCollisions(frame.hknpWorld, frame.bhkWorld)) {
                markGeneratedBodiesRebuilt(frame.bhkWorld, frame.hknpWorld);
            } else {
                _lifecycle.handColliderCreateRetryFrames = 120;
            }
            return;
        }

        _rightHand.updateDelayedGrabHandCollisionRestore(world, frame.deltaSeconds);
        _leftHand.updateDelayedGrabHandCollisionRestore(world, frame.deltaSeconds);
        updateWeaponHandSuppression(world, frame.deltaSeconds);

        if (frame.right.disabled && frame.left.disabled) {
            return;
        }
        captureHandColliderBones();
        // An invalid capture still reaches each owner's drive-failure/rebuild
        // handling; makeBoneLookup rejects it before any pose is queued.
        // Neither collider update writes the skeleton or hand-chain transport.
        // Both sides therefore consume the same freshly copied controller pose.
        if (!frame.right.disabled) {
            _rightHand.updateCollisionTransform(world, frame.right.rawHandWorld, frame.deltaSeconds, _handColliderBoneSnapshot);
        }
        if (!frame.left.disabled) {
            _leftHand.updateCollisionTransform(world, frame.left.rawHandWorld, frame.deltaSeconds, _handColliderBoneSnapshot);
        }
    }

    void PhysicsInteraction::updateBodyBoneCollisions(const PhysicsFrameContext& frame)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::BodyColliderUpdate);

        if (!runtime_state::isLocalSkeletonReady()) {
            return;
        }

        if (!g_rockConfig.rockBodyBoneCollidersEnabled) {
            if (_bodyBoneColliders.hasBodies()) {
                ROCK_LOG_INFO(Body, "Body bone collider config disabled; destroying generated body set");
                destroyBodyBoneCollisions(frame.bhkWorld);
                _contacts.bodyRuntime.reset();
            }
            return;
        }

        if (!_bodyBoneColliders.hasBodies()) {
            if (frame.reloadBoundaryActive) {
                return;
            }
            if (_lifecycle.bodyBoneColliderCreateRetryFrames > 0) {
                --_lifecycle.bodyBoneColliderCreateRetryFrames;
                return;
            }

            if (!createBodyBoneCollisions(frame.hknpWorld, frame.bhkWorld)) {
                _lifecycle.bodyBoneColliderCreateRetryFrames = 120;
            }
            return;
        }

        _bodyBoneColliders.update(frame.hknpWorld, frame.deltaSeconds);
    }

    bool PhysicsInteraction::isNativePlayerCollisionBody(RE::bhkWorld* bhk, RE::hknpWorld* hknp, std::uint32_t bodyId) const
    {
        if (!bhk || !hknp || !contact_pipeline_policy::isValidBodyId(bodyId)) {
            return false;
        }

        if (bodyId == _rightHand.getCollisionBodyId().value ||
            bodyId == _leftHand.getCollisionBodyId().value ||
            _rightHand.isHandColliderBodyId(bodyId) ||
            _leftHand.isHandColliderBodyId(bodyId) ||
            _rightHand.isHeldBodyId(bodyId) ||
            _leftHand.isHeldBodyId(bodyId) ||
            _weaponCollision.isWeaponBodyIdAtomic(bodyId) ||
            _secondaryWeaponCollision.isWeaponBodyIdAtomic(bodyId) ||
            _bodyBoneColliders.isColliderBodyIdAtomic(bodyId)) {
            return false;
        }

        std::uint32_t filterInfo = 0;
        if (!body_collision::tryReadFilterInfo(hknp, RE::hknpBodyId{ bodyId }, filterInfo)) {
            return false;
        }

        const std::uint32_t layer = filterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
        if (!collision_layer_policy::isNativePlayerCollisionBodyLayer(layer)) {
            return false;
        }

        auto* resolvedRef = resolveBodyToRef(bhk, hknp, RE::hknpBodyId{ bodyId });
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (resolvedRef) {
            return resolvedRef == player;
        }
        // Unresolved ownership is accepted only with positive player-tree
        // ancestry, never merely because reference resolution failed.
        const auto snapshot = havok_runtime::snapshotBodyIdentity(hknp, RE::hknpBodyId{ bodyId });
        auto* firstPerson = f4vr::getFirstPersonSkeleton();
        auto* thirdPerson = f4vr::getWorldRootNode();
        auto* node = havok_runtime::getOwnerNodeFromCollisionObject(snapshot.collisionObject);
        for (unsigned depth = 0; node && depth < 64; ++depth) {
            if (node == firstPerson || node == thirdPerson) {
                return true;
            }
            RE::NiNode* parent = nullptr;
            if (!native_memory::tryReadValue(&node->parent, parent) || parent == node) {
                ROCK_LOG_SAMPLE_WARN(PhysicsSafety, 5000,
                    "Native player body identity rejected: bodyId={} stage=owner-ancestry depth={}", bodyId, depth);
                return false;
            }
            node = parent;
        }
        return false;
    }

    void PhysicsInteraction::clearNativePlayerCollisionFilter(RE::hknpWorld* hknp)
    {
        native_player_collision::publish(hknp, {});
        _suppression.nativePlayerRefreshFrames = 0;
    }

    void PhysicsInteraction::updateNativePlayerCollisionFilter(RE::bhkWorld* bhk, RE::hknpWorld* hknp)
    {
        if (!g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled) {
            clearNativePlayerCollisionFilter(hknp);
            return;
        }

        if (!bhk || !hknp) {
            native_player_collision::abandon();
            return;
        }

        if (_suppression.nativePlayerRefreshFrames > 0) {
            --_suppression.nativePlayerRefreshFrames;
            return;
        }
        _suppression.nativePlayerRefreshFrames = 30;

        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::NativePlayerRefresh);
        auto cacheMutation = _generatedBodyStepDrive.callbackGate().pauseForMutation();

        struct NativePlayerBodyScanContext
        {
            PhysicsInteraction* self = nullptr;
            RE::bhkWorld* bhk = nullptr;
            RE::hknpWorld* hknp = nullptr;
            std::array<std::uint32_t, native_player_collision::kMaximumPlayerBodies> bodyIds{};
            std::uint32_t bodyCount = 0;
            bool overflow = false;
            RE::NiPoint3 playerPositionGameUnits{};
            bool playerPositionValid = false;
            RE::TESObjectREFR* rightHeldRef = nullptr;
            RE::TESObjectREFR* leftHeldRef = nullptr;
            std::array<DynamicWorldCarTarget, DynamicWorldCarCollisionRuntime::kMaxTrackedTargets> nearbyCars{};
            std::uint32_t nearbyCarCount = 0;
            bool nearbyCarOverflow = false;
            std::array<std::uint32_t, 512> visited{};

            bool alreadyVisited(std::uint32_t bodyId)
            {
                const auto key = bodyId + 1u; // Scanner admits only IDs <= 0xFFFFF.
                std::size_t slot = (bodyId * 2654435761u) & (visited.size() - 1);
                for (std::size_t probe = 0; probe < visited.size(); ++probe) {
                    if (visited[slot] == key) return true;
                    if (visited[slot] == 0) { visited[slot] = key; return false; }
                    slot = (slot + 1) & (visited.size() - 1);
                }
                return false; // Full scratch never prevents discovery.
            }

            bool contains(std::uint32_t bodyId) const
            {
                for (std::uint32_t i = 0; i < bodyCount && i < bodyIds.size(); ++i) {
                    if (bodyIds[i] == bodyId) {
                        return true;
                    }
                }
                return false;
            }

            void append(std::uint32_t bodyId)
            {
                if (alreadyVisited(bodyId)) return;
                appendNearbyCar(bodyId);
                if (!self || !self->isNativePlayerCollisionBody(bhk, hknp, bodyId) || contains(bodyId)) {
                    return;
                }
                if (bodyCount >= bodyIds.size()) {
                    overflow = true;
                    return;
                }
                bodyIds[bodyCount++] = bodyId;
            }

            void appendNearbyCar(std::uint32_t bodyId)
            {
                if (!self || !bhk || !hknp || !playerPositionValid ||
                    !contact_pipeline_policy::isValidBodyId(bodyId)) {
                    return;
                }

                std::uint32_t filterInfo = 0;
                if (!body_collision::tryReadFilterInfo(hknp, RE::hknpBodyId{ bodyId }, filterInfo)) {
                    return;
                }
                const auto layer = filterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
                if (layer != collision_layer_policy::FO4_LAYER_CLUTTER &&
                    layer != collision_layer_policy::FO4_LAYER_CLUTTER_LARGE &&
                    !collision_layer_policy::isDynamicWorldCarLayer(layer)) {
                    return;
                }

                RE::NiTransform bodyWorld{};
                if (!havok_runtime::tryGetBodyWorldTransform(hknp, RE::hknpBodyId{ bodyId }, bodyWorld)) {
                    return;
                }
                const float dx = bodyWorld.translate.x - playerPositionGameUnits.x;
                const float dy = bodyWorld.translate.y - playerPositionGameUnits.y;
                const float dz = bodyWorld.translate.z - playerPositionGameUnits.z;
                const float distanceSquared = dx * dx + dy * dy + dz * dz;
                constexpr float radiusSquared =
                    kNearbyCarCollisionRadiusGameUnits * kNearbyCarCollisionRadiusGameUnits;
                if (!std::isfinite(distanceSquared) || distanceSquared > radiusSquared) {
                    return;
                }

                auto* ref = resolveBodyToRef(bhk, hknp, RE::hknpBodyId{ bodyId });
                if (!ref || ref == rightHeldRef || ref == leftHeldRef || ref->IsDeleted() || ref->IsDisabled() ||
                    !fo4vr::isExplodableCar(ref->GetObjectReference())) {
                    return;
                }
                for (std::uint32_t index = 0; index < nearbyCarCount && index < nearbyCars.size(); ++index) {
                    if (nearbyCars[index].ref == ref) {
                        return;
                    }
                }
                if (nearbyCarCount >= nearbyCars.size()) {
                    nearbyCarOverflow = true;
                    return;
                }
                nearbyCars[nearbyCarCount++] = DynamicWorldCarTarget{
                    .ref = ref,
                    .seedBodyId = bodyId,
                };
            }
        } scanContext{ this, bhk, hknp };

        scanContext.playerPositionValid =
            character_controller_runtime::tryGetPlayerActorPositionGameUnits(scanContext.playerPositionGameUnits);
        scanContext.rightHeldRef = _rightHand.isHolding() ? _rightHand.getHeldRef() : nullptr;
        scanContext.leftHeldRef = _leftHand.isHolding() ? _leftHand.getHeldRef() : nullptr;

        auto visitBody = [](std::uint32_t bodyId, void* userData) {
            auto* context = static_cast<NativePlayerBodyScanContext*>(userData);
            if (!context) {
                return false;
            }
            context->append(bodyId);
            return true;
        };

        auto scanCollisionObject = [&](RE::NiCollisionObject* collisionObject) {
            havok_runtime::forEachPhysicsSystemBodyIdDetailed(collisionObject, hknp, 256, visitBody, &scanContext);
        };

        auto scanNode = [&](auto&& self, RE::NiAVObject* node, int depth) -> void {
            if (!node || depth <= 0) {
                return;
            }

            scanCollisionObject(node->collisionObject.get());
            if (auto* niNode = node->IsNode()) {
                auto& children = niNode->GetRuntimeData().children;
                for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                    if (auto* child = children[i].get()) {
                        self(self, child, depth - 1);
                    }
                }
            }
        };

        {
            // Freeze membership only during discovery; car synchronization below can mutate it.
            havok_world_lock::ScopedWorldReadLock readLock(hknp);
            if (auto* player = RE::PlayerCharacter::GetSingleton()) {
                if (player->currentProcess && player->currentProcess->middleHigh && player->currentProcess->middleHigh->poseBound) {
                    scanCollisionObject(player->currentProcess->middleHigh->poseBound.get());
                }
            }
            scanNode(scanNode, f4vr::getFirstPersonSkeleton(), 64);
            scanNode(scanNode, f4vr::getWorldRootNode(), 64);
        }

        if (scanContext.playerPositionValid) {
            _dynamicWorldCarCollision.synchronizeNearbyTargets(
                bhk,
                hknp,
                std::span<const DynamicWorldCarTarget>{ scanContext.nearbyCars.data(), scanContext.nearbyCarCount });
        }

        if (scanContext.nearbyCarOverflow) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                5000,
                "Nearby car collision target capacity exceeded; keeping first {} cars within {:.0f} game units",
                scanContext.nearbyCars.size(),
                kNearbyCarCollisionRadiusGameUnits);
        }

        if (scanContext.overflow && !_suppression.nativePlayerOverflowLogged) {
            _suppression.nativePlayerOverflowLogged = true;
            ROCK_LOG_WARN(Hand,
                "Native player contact filter capacity exceeded; additional bodies retain native collision (capacity={})",
                native_player_collision::kMaximumPlayerBodies);
        } else if (!scanContext.overflow) {
            _suppression.nativePlayerOverflowLogged = false;
        }

        std::array<native_player_collision::BodyIdentity, native_player_collision::kMaximumPlayerBodies> bodies{};
        std::size_t count = 0;
        for (std::uint32_t i = 0; i < scanContext.bodyCount; ++i) {
            const auto body = havok_runtime::snapshotBodyIdentity(hknp, RE::hknpBodyId{ scanContext.bodyIds[i] });
            auto* ownerNode = body.valid ? havok_runtime::getOwnerNodeFromCollisionObject(body.collisionObject) : nullptr;
            if (body.valid && body.collisionObject && ownerNode) {
                bodies[count++] = { body.bodyId.value, body.motionIndex,
                    reinterpret_cast<std::uintptr_t>(body.collisionObject),
                    reinterpret_cast<std::uintptr_t>(ownerNode) };
            }
        }
        native_player_collision::publish(hknp, { bodies.data(), count });
    }
}
