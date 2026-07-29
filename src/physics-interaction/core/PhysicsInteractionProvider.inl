/*
 * Provider snapshot and query glue is split from lifecycle/update orchestration because it exposes ROCK state to external consumers without owning physics behavior. It stays in this translation unit to preserve helper visibility and public API behavior.
 */
    void PhysicsInteraction::fillProviderFrameSnapshot(::rock::provider::RockProviderFrameSnapshot& outSnapshot) const
    {
        const auto& runtime = runtime_state::currentFrame();
        outSnapshot.providerReady = (_initialized.load(std::memory_order_acquire) && runtime.visualAuthorityAvailable) ? 1u : 0u;
        outSnapshot.frikSkeletonReady = runtime.localSkeletonReady ? 1u : 0u;
        outSnapshot.menuBlocking = runtime.localMenuBlocking ? 1u : 0u;
        outSnapshot.configBlocking = runtime.compatibilityConfigBlocking ? 1u : 0u;
        outSnapshot.bhkWorld = reinterpret_cast<std::uintptr_t>(_cachedBhkWorld);
        outSnapshot.hknpWorld = reinterpret_cast<std::uintptr_t>(_cachedHknpWorld);
        outSnapshot.gameToHavokScale = physics_scale::gameToHavok();
        outSnapshot.havokToGameScale = physics_scale::havokToGame();
        outSnapshot.physicsScaleRevision = physics_scale::revision();
        outSnapshot.lifecycleFlags = _lifecycleFlagsAtomic.load(std::memory_order_acquire);
        outSnapshot.lastLifecycleReason = static_cast<::rock::provider::RockProviderLifecycleReason>(
            _lastLifecycleReasonAtomic.load(std::memory_order_acquire));
        outSnapshot.worldGeneration = _worldGenerationAtomic.load(std::memory_order_acquire);
        outSnapshot.skeletonGeneration = _skeletonGenerationAtomic.load(std::memory_order_acquire);
        outSnapshot.providerGeneration = _providerGenerationAtomic.load(std::memory_order_acquire);
        outSnapshot.stableFrameCount = _stableFrameCountAtomic.load(std::memory_order_acquire);
        outSnapshot.deltaSeconds =
            std::isfinite(_deltaTime) && _deltaTime > 0.0f ?
                _deltaTime :
                (1.0f / 90.0f);
        outSnapshot.enrichmentFlags |= static_cast<std::uint32_t>(
            ::rock::provider::RockProviderFrameEnrichmentFlagV1::DeltaSecondsValid);
        if (auto* playerNodes = f4vr::getPlayerNodes();
            playerNodes && playerNodes->HmdNode &&
            finiteNiTransform(playerNodes->HmdNode->world)) {
            fillProviderTransform(
                playerNodes->HmdNode->world,
                outSnapshot.hmdTransform);
            outSnapshot.enrichmentFlags |= static_cast<std::uint32_t>(
                ::rock::provider::RockProviderFrameEnrichmentFlagV1::HmdTransformValid);
            const auto rawForward =
                playerNodes->HmdNode->world.rotate.Transpose() *
                RE::NiPoint3(0.0f, 1.0f, 0.0f);
            RE::NiPoint3 normalizedForward{};
            if (selection_query_policy::tryNormalizeVectorForHmdCone(
                    rawForward,
                    normalizedForward)) {
                outSnapshot.hmdForwardWorld[0] = normalizedForward.x;
                outSnapshot.hmdForwardWorld[1] = normalizedForward.y;
                outSnapshot.hmdForwardWorld[2] = normalizedForward.z;
                outSnapshot.enrichmentFlags |= static_cast<std::uint32_t>(
                    ::rock::provider::RockProviderFrameEnrichmentFlagV1::HmdForwardValid);
            }
        }
        outSnapshot.primaryHand = _twoHandedGrip.isFiringHandLeft() ?
            ::rock::provider::RockProviderHand::Left :
            ::rock::provider::RockProviderHand::Right;
        outSnapshot.offhandHand =
            outSnapshot.primaryHand == ::rock::provider::RockProviderHand::Left ?
                ::rock::provider::RockProviderHand::Right :
                ::rock::provider::RockProviderHand::Left;
        outSnapshot.enrichmentFlags |= static_cast<std::uint32_t>(
            ::rock::provider::RockProviderFrameEnrichmentFlagV1::CoherentHandRoles);
        outSnapshot.collisionGeneration =
            _collisionGenerationAtomic.load(std::memory_order_acquire);
        outSnapshot.enrichmentFlags |= static_cast<std::uint32_t>(
            ::rock::provider::RockProviderFrameEnrichmentFlagV1::CollisionGenerationValid);
        outSnapshot.equippedWeaponTransitionSequence =
            _equippedWeaponTransition.getPublicSnapshot().transitionSequence;
        outSnapshot.enrichmentFlags |= static_cast<std::uint32_t>(
            ::rock::provider::RockProviderFrameEnrichmentFlagV1::EquippedTransitionSequenceValid);

        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        outSnapshot.weaponNode = reinterpret_cast<std::uintptr_t>(weaponNode);
        outSnapshot.weaponFormId = currentEquippedWeaponFormId();
        for (auto& bodyId : outSnapshot.weaponBodyIds) {
            bodyId = 0x7FFF'FFFF;
        }
        const auto weaponSnapshot = _weaponCollision.getWeaponBodySnapshotAtomic();
        outSnapshot.weaponGenerationKey = weaponSnapshot.generationKey;
        outSnapshot.weaponBodyCount = (std::min)(weaponSnapshot.count, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_BODIES);
        for (std::uint32_t i = 0; i < outSnapshot.weaponBodyCount; ++i) {
            outSnapshot.weaponBodyIds[i] = weaponSnapshot.bodyIds[i];
        }

        if (_handBoneCache.isReady()) {
            fillProviderTransform(_handBoneCache.getWorldTransform(false), outSnapshot.rightHandTransform);
            fillProviderTransform(_handBoneCache.getWorldTransform(true), outSnapshot.leftHandTransform);
        }

        outSnapshot.rightHandBodyId = _rightHand.getCollisionBodyId().value;
        outSnapshot.leftHandBodyId = _leftHand.getCollisionBodyId().value;
        outSnapshot.rightHandState = providerHandStateFlags(_rightHand, false);
        outSnapshot.leftHandState = providerHandStateFlags(_leftHand, true);
        outSnapshot.offhandReservation = ::rock::provider::currentOffhandReservation();
    }

    bool PhysicsInteraction::queryProviderWorldRaycastV1(
        const ::rock::provider::RockProviderWorldRaycastRequestV1& request,
        ::rock::provider::RockProviderWorldRaycastResultV1& outResult) const
    {
        const RE::NiPoint3 start{
            request.startGame.x,
            request.startGame.y,
            request.startGame.z,
        };
        RE::NiPoint3 direction{
            request.directionGame.x,
            request.directionGame.y,
            request.directionGame.z,
        };
        const float directionLengthSquared =
            direction.x * direction.x +
            direction.y * direction.y +
            direction.z * direction.z;
        if (!std::isfinite(directionLengthSquared) ||
            directionLengthSquared <= 1.0e-8f) {
            return false;
        }
        const float inverseDirectionLength =
            1.0f / std::sqrt(directionLengthSquared);
        direction.x *= inverseDirectionLength;
        direction.y *= inverseDirectionLength;
        direction.z *= inverseDirectionLength;

        const RE::NiPoint3 end{
            start.x + direction.x * request.maxDistanceGame,
            start.y + direction.y * request.maxDistanceGame,
            start.z + direction.z * request.maxDistanceGame,
        };

        physics_ray_cast::ClosestSegmentResult rayResult{};
        if (!physics_ray_cast::castClosestSegment(
                _cachedBhkWorld,
                start,
                end,
                g_rockConfig.rockFarClipRayFilterInfo,
                rayResult)) {
            return false;
        }

        outResult.hit = rayResult.hit ? 1u : 0u;
        outResult.flags = rayResult.hit ?
            static_cast<std::uint32_t>(
                ::rock::provider::RockProviderWorldRaycastResultFlagV1::Hit) :
            0u;
        outResult.hitFraction =
            rayResult.hit ? rayResult.hitFraction : 1.0f;
        outResult.hitDistanceGame =
            outResult.hitFraction * request.maxDistanceGame;
        outResult.hitPointGame = {
            start.x + direction.x * outResult.hitDistanceGame,
            start.y + direction.y * outResult.hitDistanceGame,
            start.z + direction.z * outResult.hitDistanceGame,
        };
        if (rayResult.normalValid) {
            outResult.flags |= static_cast<std::uint32_t>(
                ::rock::provider::RockProviderWorldRaycastResultFlagV1::
                    NormalValid);
            outResult.hitNormalGame = {
                rayResult.normalGame.x,
                rayResult.normalGame.y,
                rayResult.normalGame.z,
            };
        }
        outResult.worldGeneration =
            _worldGenerationAtomic.load(std::memory_order_acquire);
        outResult.skeletonGeneration =
            _skeletonGenerationAtomic.load(std::memory_order_acquire);
        outResult.providerGeneration =
            _providerGenerationAtomic.load(std::memory_order_acquire);
        return true;
    }
    void PhysicsInteraction::fillProviderWeaponPartGripStates(
        std::array<::rock::provider::RockProviderWeaponPartGripStateV1, 2>& outStates) const
    {
        // Indexed [right, left] to match the getter's hand mapping. Published
        // after the grip update each frame, so consumers always read this
        // frame's grip decision alongside the frame snapshot.
        for (const bool isLeft : { false, true }) {
            auto& outState = outStates[isLeft ? 1u : 0u];
            outState = {};
            outState.hand = isLeft ? ::rock::provider::RockProviderHand::Left : ::rock::provider::RockProviderHand::Right;

            HandGripReport report{};
            _twoHandedGrip.getHandGripReport(isLeft, report);
            outState.gripKind = static_cast<::rock::provider::RockProviderWeaponPartGripKindV1>(report.kind);
            outState.active = report.active ? 1u : 0u;
            outState.attachOnly = report.attachOnly ? 1u : 0u;
            outState.authoredSupportGrip =
                report.authoredSupportGrip ? 1u : 0u;
            outState.gripSequence = report.gripSequence;
            outState.weaponGenerationKey = report.weaponGenerationKey;
            outState.bodyId = report.bodyId;
            outState.partKind = report.partKind;
            outState.reloadRole = report.reloadRole;
            outState.supportRole = report.supportRole;
            outState.socketRole = report.socketRole;
            outState.actionRole = report.actionRole;
            outState.sourceRoot = report.sourceRoot;
            outState.providerOwnerToken = report.providerOwnerToken;
            outState.providerGroupId = report.providerGroupId;
            outState.providerGrabMode = report.providerGrabMode;
            outState.hasHandPartLocal = report.hasHandPartLocal ? 1u : 0u;
            outState.handPartLocalSpace = report.handPartLocalIsSourceLocal ?
                ::rock::provider::RockProviderWeaponPartGripLocalSpaceV1::PartSourceLocal :
                ::rock::provider::RockProviderWeaponPartGripLocalSpaceV1::WeaponRootLocal;
            fillProviderTransform(report.handPartLocal, outState.handPartLocal);
            static_assert(sizeof(outState.sourceName) == std::tuple_size_v<decltype(report.sourceName)>);
            std::memcpy(outState.sourceName, report.sourceName.data(), sizeof(outState.sourceName));
            outState.sourceName[sizeof(outState.sourceName) - 1] = '\0';
            outState.omodFormId = report.omodFormId;
            outState.attachPointFormId = report.attachPointFormId;
            outState.classificationSource = report.classificationSource;
        }
    }

    bool PhysicsInteraction::queryProviderWeaponContactAtPoint(
        const ::rock::provider::RockProviderWeaponContactQuery& query,
        ::rock::provider::RockProviderWeaponContactResult& outResult) const
    {
        outResult = {};
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        if (!weaponNode) {
            return false;
        }

        WeaponInteractionContact contact{};
        const RE::NiPoint3 point{ query.pointGame[0], query.pointGame[1], query.pointGame[2] };
        if (!_weaponCollision.tryFindInteractionContactNearPoint(weaponNode, point, query.radiusGame, contact)) {
            return false;
        }

        outResult.valid = contact.valid ? 1u : 0u;
        outResult.bodyId = contact.bodyId;
        outResult.partKind = static_cast<std::uint32_t>(contact.partKind);
        outResult.reloadRole = static_cast<std::uint32_t>(contact.reloadRole);
        outResult.supportRole = static_cast<std::uint32_t>(contact.supportGripRole);
        outResult.socketRole = static_cast<std::uint32_t>(contact.socketRole);
        outResult.actionRole = static_cast<std::uint32_t>(contact.actionRole);
        outResult.interactionRoot = reinterpret_cast<std::uintptr_t>(contact.interactionRoot);
        outResult.sourceRoot = reinterpret_cast<std::uintptr_t>(contact.sourceRoot);
        outResult.weaponGenerationKey = contact.weaponGenerationKey;
        outResult.probeDistanceGame = contact.probeDistanceGame;
        return contact.valid;
    }

    bool PhysicsInteraction::queryProviderEquippedWeaponClassificationV1(::rock::provider::RockProviderWeaponClassificationV1& outResult) const
    {
        outResult = {};
        const auto identity = _weaponCollision.getEquippedWeaponClassification();
        outResult.valid = identity.hasEquippedWeapon ? 1u : 0u;
        outResult.formId = identity.formID;
        outResult.keywordFlags = identity.keywordFlags;
        outResult.sizeClass = static_cast<::rock::provider::RockProviderWeaponSizeClassV1>(identity.sizeClass);
        outResult.source = static_cast<::rock::provider::RockProviderWeaponClassificationSourceV1>(identity.classificationSource);
        outResult.weaponGenerationKey =
            _weaponCollision.getCurrentWeaponGenerationKey();
        using Source = WeaponClassificationSource;
        using Provenance =
            ::rock::provider::RockProviderWeaponClassificationProvenanceFlagV1;
        switch (identity.classificationSource) {
        case Source::Keyword:
            outResult.confidence = 1.0f;
            outResult.provenanceFlags |= static_cast<std::uint32_t>(
                Provenance::KeywordEvidence);
            break;
        case Source::WeightFallback:
            outResult.confidence = 0.65f;
            outResult.provenanceFlags |= static_cast<std::uint32_t>(
                Provenance::MeshBoundsFallback);
            break;
        case Source::Default:
            outResult.confidence = 0.35f;
            break;
        default:
            outResult.confidence = 0.0f;
            break;
        }
        if (outResult.weaponGenerationKey != 0) {
            outResult.provenanceFlags |= static_cast<std::uint32_t>(
                Provenance::GenerationBound);
        }
        return identity.hasEquippedWeapon;
    }

    bool PhysicsInteraction::queryProviderEquippedWeaponGripStateV1(
        ::rock::provider::RockProviderEquippedWeaponGripStateV1& outState) const
    {
        using Flag = ::rock::provider::RockProviderEquippedWeaponGripStateFlagV1;

        outState = {};
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        if (!_initialized.load(std::memory_order_acquire) || !weaponNode) {
            return false;
        }

        outState.flags = static_cast<std::uint32_t>(Flag::Valid);
        outState.weaponNode = reinterpret_cast<std::uintptr_t>(weaponNode);
        outState.weaponFormId = currentEquippedWeaponFormId();
        outState.weaponGenerationKey = _weaponCollision.getCurrentWeaponGenerationKey();

        if (_twoHandedGrip.getState() == TwoHandedState::Gripping) {
            outState.flags |= static_cast<std::uint32_t>(Flag::TwoHandGripActive);
        }
        if (_twoHandedGrip.isFiringHandLeft()) {
            outState.flags |= static_cast<std::uint32_t>(Flag::FiringHandLeft);
        }
        if (_twoHandedGrip.ownsWeaponTransform()) {
            outState.flags |= static_cast<std::uint32_t>(Flag::WeaponTransformOwned);
        }

        RE::NiTransform weaponWorld{};
        const bool solvedWeaponWorldValid =
            _twoHandedGrip.getSolvedWeaponTransform(weaponWorld) &&
            finiteNiTransform(weaponWorld);
        if (!solvedWeaponWorldValid && finiteNiTransform(weaponNode->world)) {
            weaponWorld = weaponNode->world;
        }
        if (solvedWeaponWorldValid || finiteNiTransform(weaponNode->world)) {
            fillProviderTransform(weaponWorld, outState.weaponWorld);
            outState.flags |= static_cast<std::uint32_t>(Flag::WeaponWorldValid);
        }

        RE::NiTransform rightHandInWeapon{};
        RE::NiTransform leftHandInWeapon{};
        if (_twoHandedGrip.getManualCycleRockGripBaselines(
                rightHandInWeapon,
                leftHandInWeapon)) {
            fillProviderTransform(rightHandInWeapon, outState.rightHandInWeapon);
            fillProviderTransform(leftHandInWeapon, outState.leftHandInWeapon);
            outState.flags |=
                static_cast<std::uint32_t>(Flag::RightHandInWeaponValid) |
                static_cast<std::uint32_t>(Flag::LeftHandInWeaponValid);
        }

        /*
         * EquippedWeaponData::fireNode is Bethesda's equip-time projectile
         * origin. It is valid before muzzle-flash presentation state exists,
         * so publishing it makes the muzzle snapshot available as soon as the
         * weapon is equipped instead of after its first shot.
         */
        if (const auto* projectileNode =
                getEquippedProjectileNode();
            projectileNode &&
            finiteNiTransform(projectileNode->world)) {
            const auto& projectileWorld = projectileNode->world;
            RE::NiPoint3 muzzleDirection{
                projectileWorld.rotate.entry[1][0],
                projectileWorld.rotate.entry[1][1],
                projectileWorld.rotate.entry[1][2],
            };
            const float directionLength = muzzleDirection.Length();
            if (std::isfinite(directionLength) &&
                directionLength > 0.000001f) {
                muzzleDirection /= directionLength;
                outState.muzzleOriginGame =
                    makeProviderPoint(projectileWorld.translate);
                outState.muzzleDirectionGame =
                    makeProviderPoint(muzzleDirection);
                outState.flags |=
                    static_cast<std::uint32_t>(Flag::MuzzleWorldValid);
            }
        }

        return true;
    }

    bool PhysicsInteraction::queryProviderEquippedWeaponHandlingStateV1(
        ::rock::provider::RockProviderEquippedWeaponHandlingStateV1& outState) const
    {
        using RuntimeFlag =
            ::rock::provider::RockProviderEquippedWeaponHandlingRuntimeFlagV1;

        outState = {};
        outState.fixedFiringHand = _fixedFiringHandIsLeft ?
            ::rock::provider::RockProviderHand::Left :
            ::rock::provider::RockProviderHand::Right;
        outState.currentFiringHand = _twoHandedGrip.isFiringHandLeft() ?
            ::rock::provider::RockProviderHand::Left :
            ::rock::provider::RockProviderHand::Right;
        outState.weaponGenerationKey =
            _weaponCollision.getCurrentWeaponGenerationKey();
        outState.weaponFormId = currentEquippedWeaponFormId();

        const auto setFlag = [&outState](const RuntimeFlag flag) {
            outState.runtimeFlags |= static_cast<std::uint32_t>(flag);
        };
        if (_fixedFiringHandIsLeft) {
            setFlag(RuntimeFlag::FixedHandLeft);
        }
        if (_twoHandedGrip.isFiringHandLeft()) {
            setFlag(RuntimeFlag::FiringHandLeft);
        }
        if (TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true)) {
            setFlag(RuntimeFlag::LeftFiringInfrastructureAvailable);
        }
        if (_twoHandedGrip.isManualOwnershipActive()) {
            setFlag(RuntimeFlag::ManualOwnershipActive);
        }
        if (_twoHandedGrip.isPartCarryActive()) {
            setFlag(RuntimeFlag::PartCarryActive);
        }
        if (_twoHandedGrip.isFiringGripOccupied()) {
            setFlag(RuntimeFlag::FiringGripOccupied);
        }
        if (resolveEquippedWeaponInteractionNode()) {
            setFlag(RuntimeFlag::WeaponPresent);
        }
        return _initialized.load(std::memory_order_acquire);
    }

    std::uint32_t PhysicsInteraction::getProviderWeaponEvidenceDetailCountV1() const
    {
        return static_cast<std::uint32_t>(_weaponCollision.getProfileEvidenceDescriptors().size());
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponEvidenceDetailsV1(
        ::rock::provider::RockProviderWeaponEvidenceDetailV1* outDetails,
        std::uint32_t maxDetails) const
    {
        if (!outDetails || maxDetails == 0) {
            return 0;
        }

        const auto descriptors = _weaponCollision.getProfileEvidenceDescriptors();
        const std::uint32_t count = (std::min)(maxDetails, static_cast<std::uint32_t>(descriptors.size()));
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto& descriptor = descriptors[i];
            auto& out = outDetails[i];
            out = {};
            out.size = sizeof(::rock::provider::RockProviderWeaponEvidenceDetailV1);
            out.bodyId = descriptor.bodyId;
            out.partKind = static_cast<std::uint32_t>(descriptor.semantic.partKind);
            out.reloadRole = static_cast<std::uint32_t>(descriptor.semantic.reloadRole);
            out.supportRole = static_cast<std::uint32_t>(descriptor.semantic.supportGripRole);
            out.socketRole = static_cast<std::uint32_t>(descriptor.semantic.socketRole);
            out.actionRole = static_cast<std::uint32_t>(descriptor.semantic.actionRole);
            out.fallbackGripPose = static_cast<std::uint32_t>(descriptor.semantic.fallbackGripPose);
            out.interactionRoot = descriptor.geometryRootAddress;
            out.sourceRoot = descriptor.sourceRootAddress;
            out.weaponGenerationKey = descriptor.weaponGenerationKey;
            out.localBoundsGame.min = makeProviderPoint(descriptor.localBoundsGame.min);
            out.localBoundsGame.max = makeProviderPoint(descriptor.localBoundsGame.max);
            out.localBoundsGame.valid = descriptor.localBoundsGame.valid ? 1u : 0u;
            out.pointCount = descriptor.pointCount;
            copyProviderString(out.sourceName, sizeof(out.sourceName), descriptor.sourceName);
            out.omodFormId = descriptor.omodFormId;
            out.attachPointFormId = descriptor.semantic.attachPointFormId;
            out.classificationSource = static_cast<std::uint32_t>(descriptor.semantic.classificationSource);
        }

        return count;
    }

    std::uint32_t PhysicsInteraction::getProviderWeaponEvidenceDetailPointCountV1(std::uint32_t bodyId) const
    {
        WeaponCollisionProfileEvidenceDescriptor descriptor{};
        RE::NiAVObject* sourceNode = nullptr;
        if (!_weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(bodyId, descriptor, sourceNode)) {
            return 0;
        }

        return descriptor.pointCount;
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponEvidenceDetailPointsV1(
        std::uint32_t bodyId,
        ::rock::provider::RockProviderPoint3* outPoints,
        std::uint32_t maxPoints) const
    {
        if (!outPoints || maxPoints == 0) {
            return 0;
        }

        WeaponCollisionProfileEvidenceDescriptor descriptor{};
        RE::NiAVObject* sourceNode = nullptr;
        if (!_weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(bodyId, descriptor, sourceNode)) {
            return 0;
        }

        const std::uint32_t copied = (std::min)(maxPoints, static_cast<std::uint32_t>(descriptor.localMeshPointsGame.size()));
        for (std::uint32_t i = 0; i < copied; ++i) {
            outPoints[i] = makeProviderPoint(descriptor.localMeshPointsGame[i]);
        }

        return copied;
    }

    std::uint32_t PhysicsInteraction::getProviderWeaponEmitterCountV1() const
    {
        const auto snapshot = _weaponCollision.getWeaponEmitterSnapshot();
        return static_cast<std::uint32_t>((std::min)(snapshot.count, snapshot.emitters.size()));
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponEmittersV1(
        ::rock::provider::RockProviderWeaponEmitterV1* outEmitters,
        std::uint32_t maxEmitters) const
    {
        if (!outEmitters || maxEmitters == 0) {
            return 0;
        }

        static_assert(MAX_WEAPON_EMITTERS == ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_EMITTERS_V1);
        const auto snapshot = _weaponCollision.getWeaponEmitterSnapshot();
        const std::uint32_t count = (std::min)(maxEmitters,
            static_cast<std::uint32_t>((std::min)(snapshot.count, snapshot.emitters.size())));
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto& descriptor = snapshot.emitters[i];
            auto& out = outEmitters[i];
            out = {};
            out.size = sizeof(::rock::provider::RockProviderWeaponEmitterV1);
            out.version = ::rock::provider::ROCK_PROVIDER_API_VERSION;
            out.kind = static_cast<::rock::provider::RockProviderWeaponEmitterKindV1>(descriptor.kind);
            out.source = static_cast<::rock::provider::RockProviderWeaponEmitterSourceV1>(descriptor.source);
            out.flags = static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponEmitterFlagV1::TransformValid) |
                static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponEmitterFlagV1::DirectionValid);
            if (descriptor.effectStateKnown) {
                out.flags |= static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponEmitterFlagV1::EffectStateKnown);
            }
            if (descriptor.hasAddOnNodeValue) {
                out.flags |= static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponEmitterFlagV1::HasAddOnNodeValue);
            }
            if (descriptor.omodFormId != 0) {
                out.flags |= static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponEmitterFlagV1::HasOmod);
            }
            if (descriptor.attachPointFormId != 0) {
                out.flags |= static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponEmitterFlagV1::HasAttachPoint);
            }
            out.active = descriptor.active ? 1u : 0u;
            out.visible = descriptor.visible ? 1u : 0u;
            out.addOnNodeValue = descriptor.addOnNodeValue;
            out.omodFormId = descriptor.omodFormId;
            out.attachPointFormId = descriptor.attachPointFormId;
            out.weaponGenerationKey = descriptor.weaponGenerationKey;
            std::copy(descriptor.rotate.begin(), descriptor.rotate.end(), out.weaponLocalTransform.rotate);
            std::copy(descriptor.translate.begin(), descriptor.translate.end(), out.weaponLocalTransform.translate);
            out.weaponLocalTransform.scale = descriptor.scale;
            out.forwardWeaponLocal = {
                descriptor.forwardWeaponLocal[0],
                descriptor.forwardWeaponLocal[1],
                descriptor.forwardWeaponLocal[2],
            };
            static_assert(sizeof(out.sourceName) == std::tuple_size_v<decltype(descriptor.sourceName)>);
            std::memcpy(out.sourceName, descriptor.sourceName.data(), sizeof(out.sourceName));
            out.sourceName[sizeof(out.sourceName) - 1] = '\0';
        }
        return count;
    }

    std::uint32_t PhysicsInteraction::copyProviderBodyContacts(
        ::rock::provider::RockProviderBodyContactV1* outContacts,
        std::uint32_t maxContacts) const
    {
        if (!outContacts || maxContacts == 0) {
            return 0;
        }

        std::array<body_contact_runtime::BodyContactRecord, body_contact_runtime::kMaxBodyContactRecords> records{};
        const auto requested = (std::min)(static_cast<std::size_t>(maxContacts), records.size());
        const auto copied = _bodyContactRuntime.snapshot(records.data(), requested);
        for (std::size_t i = 0; i < copied; ++i) {
            const auto& record = records[i];
            auto& out = outContacts[i];
            out = {};
            out.size = sizeof(::rock::provider::RockProviderBodyContactV1);
            out.version = ::rock::provider::ROCK_PROVIDER_API_VERSION;
            out.frameIndex = record.frame;
            out.bodyId = record.bodyId;
            out.targetBodyId = record.targetBodyId;
            out.bodyLayer = record.bodyLayer;
            out.targetLayer = record.targetLayer;
            out.zone = static_cast<::rock::provider::RockProviderBodyZoneKind>(record.zone);
            out.side = static_cast<::rock::provider::RockProviderBodyZoneSide>(record.side);
            out.role = static_cast<std::uint32_t>(record.role);
            out.descriptorIndex = record.descriptorIndex;
            out.targetKind = providerBodyContactTargetKind(record.targetKind);
            out.targetZone = static_cast<::rock::provider::RockProviderBodyZoneKind>(record.targetZone);
            out.targetSide = static_cast<::rock::provider::RockProviderBodyZoneSide>(record.targetSide);
            out.targetRole = static_cast<std::uint32_t>(record.targetRole);
            out.targetDescriptorIndex = record.targetDescriptorIndex;
            out.inPowerArmor = record.inPowerArmor ? 1u : 0u;
            out.targetInPowerArmor = record.targetInPowerArmor ? 1u : 0u;
            out.hasContactPointGame = record.hasContactPointGame ? 1u : 0u;
            out.contactPointGame = makeProviderPoint(record.contactPointGame);
        }

        return static_cast<std::uint32_t>(copied);
    }

    void PhysicsInteraction::fillProviderHandInteractionStates(
        std::array<::rock::provider::RockProviderHandInteractionStateV1, 2>& outStates) const
    {
        using Phase =
            ::rock::provider::RockProviderHandInteractionPhaseV1;
        using Flag =
            ::rock::provider::RockProviderHandInteractionFlagV1;
        using TargetKind =
            ::rock::provider::RockProviderBodyContactTargetKind;

        const auto phaseForState = [](const HandState state) {
            switch (state) {
            case HandState::SelectedClose:
            case HandState::SelectedFar:
                return Phase::Selecting;
            case HandState::PrePullItem:
            case HandState::Pulled:
                return Phase::Pulling;
            case HandState::SelectionLocked:
            case HandState::PreGrabItem:
            case HandState::HeldInit:
            case HandState::GrabFromOtherHand:
            case HandState::GrabExternal:
                return Phase::Catching;
            case HandState::HeldBody:
            case HandState::SelectedTwoHand:
            case HandState::HeldTwoHanded:
            case HandState::LootOtherHand:
                return Phase::Holding;
            case HandState::StashCandidate:
                return Phase::StashCandidate;
            case HandState::ConsumeCandidate:
                return Phase::ConsumeCandidate;
            default:
                return Phase::Idle;
            }
        };
        const auto targetKindForSelection = [](const SelectedObject& selection) {
            return grab_target::isActorDriven(selection.targetKind) ?
                TargetKind::Actor :
                TargetKind::DynamicProp;
        };

        const auto worldGeneration =
            _worldGenerationAtomic.load(std::memory_order_acquire);
        const auto skeletonGeneration =
            _skeletonGenerationAtomic.load(std::memory_order_acquire);
        const auto providerGeneration =
            _providerGenerationAtomic.load(std::memory_order_acquire);
        const auto collisionGeneration =
            _collisionGenerationAtomic.load(std::memory_order_acquire);
        const bool primaryIsLeft = _twoHandedGrip.isFiringHandLeft();

        for (const bool isLeft : { false, true }) {
            const Hand& hand = isLeft ? _leftHand : _rightHand;
            auto& state = outStates[isLeft ? 1u : 0u];
            state = {};
            state.hand = isLeft ?
                ::rock::provider::RockProviderHand::Left :
                ::rock::provider::RockProviderHand::Right;
            state.phase = phaseForState(hand.getState());
            if (state.phase == Phase::Idle && hand.isTouching()) {
                state.phase = Phase::Touching;
            }
            state.flags = static_cast<std::uint32_t>(Flag::Valid);
            if (isLeft == primaryIsLeft) {
                state.flags |= static_cast<std::uint32_t>(Flag::Primary);
            } else {
                state.flags |= static_cast<std::uint32_t>(Flag::Offhand);
            }

            if (hand.isHolding()) {
                auto* heldRef = hand.getHeldRef();
                state.targetFormId = heldRef ? heldRef->GetFormID() : 0;
                state.primaryBodyId = hand.getSavedObjectState().bodyId.value;
                state.targetKind = TargetKind::HeldObject;
                state.flags |= static_cast<std::uint32_t>(Flag::LooseObject);
                if (hand.isHoldingLooseWeapon()) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::LooseWeapon);
                }
            } else if (hand.hasSelection()) {
                const auto& selection = hand.getSelection();
                state.targetFormId =
                    selection.refr ? selection.refr->GetFormID() : 0;
                state.primaryBodyId = selection.bodyId.value;
                state.targetKind = targetKindForSelection(selection);
                if (grab_target::isPhysicalRockObject(selection.targetKind)) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::LooseObject);
                }
            } else if (hand.isTouching()) {
                state.targetFormId = hand.getLastTouchedFormID();
                state.targetKind = TargetKind::DynamicProp;
            }

            const auto& heldBodies = hand.getHeldBodyIds();
            state.heldBodyCount = static_cast<std::uint32_t>((std::min)(
                heldBodies.size(),
                static_cast<std::size_t>(
                    ::rock::provider::ROCK_PROVIDER_MAX_HAND_HELD_BODIES_V1)));
            for (std::uint32_t i = 0; i < state.heldBodyCount; ++i) {
                state.heldBodyIds[i] = heldBodies[i];
            }
            if (heldBodies.size() > state.heldBodyCount) {
                state.flags |= static_cast<std::uint32_t>(
                    Flag::HeldBodyListTruncated);
            }

            HandGripReport gripReport{};
            _twoHandedGrip.getHandGripReport(isLeft, gripReport);
            if (gripReport.active) {
                switch (gripReport.kind) {
                case weapon_part_grip_report_policy::HandGripKind::FiringGrip:
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::FiringGrip);
                    break;
                case weapon_part_grip_report_policy::HandGripKind::PartCarry:
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::PartCarry);
                    break;
                default:
                    state.flags |= static_cast<std::uint32_t>(Flag::PartGrip);
                    break;
                }
            }

            state.effectiveInputSuppressionFlags =
                ::rock::provider::currentHandInputSuppressionFlagsV1(
                    state.hand);
            if (state.effectiveInputSuppressionFlags != 0) {
                state.flags |= static_cast<std::uint32_t>(
                    Flag::InputSuppressed);
            }
            ::rock::provider::RockProviderHandCollisionAvailabilityV1
                collisionState{};
            if (queryProviderHandCollisionAvailabilityV1(
                    state.hand,
                    collisionState)) {
                state.collisionAvailabilityFlags = collisionState.flags;
                if ((collisionState.flags & static_cast<std::uint32_t>(
                         ::rock::provider::RockProviderHandCollisionAvailabilityFlagV1::CollisionAvailable)) != 0) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::CollisionAvailable);
                }
                if ((collisionState.flags & static_cast<std::uint32_t>(
                         ::rock::provider::RockProviderHandCollisionAvailabilityFlagV1::TransitionSuppressed)) != 0) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::TransitionSuppressed);
                }
            }
            state.worldGeneration = worldGeneration;
            state.skeletonGeneration = skeletonGeneration;
            state.providerGeneration = providerGeneration;
            state.collisionGeneration = collisionGeneration;
        }
    }

    bool PhysicsInteraction::queryProviderEquippedWeaponStateV1(
        ::rock::provider::RockProviderEquippedWeaponStateV1& outState) const
    {
        using Flag =
            ::rock::provider::RockProviderEquippedWeaponStateFlagV1;
        outState = {};
        const auto transition = _equippedWeaponTransition.getPublicSnapshot();
        outState.weaponFormId = transition.weaponFormID != 0 ?
            transition.weaponFormID :
            currentEquippedWeaponFormId();
        outState.weaponGenerationKey =
            _weaponCollision.getCurrentWeaponGenerationKey();
        outState.transitionSequence = transition.transitionSequence;
        outState.terminalSequence = transition.terminalSequence;
        switch (transition.source) {
        case EquippedWeaponTransitionCoordinator::Source::ObservedEquip:
            outState.transitionSource =
                ::rock::provider::RockProviderEquippedWeaponTransitionSourceV1::ObservedEquip;
            break;
        case EquippedWeaponTransitionCoordinator::Source::HeldTriggerEquip:
            outState.transitionSource =
                ::rock::provider::RockProviderEquippedWeaponTransitionSourceV1::HeldTriggerEquip;
            break;
        case EquippedWeaponTransitionCoordinator::Source::HeldGripZoneEquip:
            outState.transitionSource =
                ::rock::provider::RockProviderEquippedWeaponTransitionSourceV1::HeldGripZoneEquip;
            break;
        case EquippedWeaponTransitionCoordinator::Source::MenuExit:
            outState.transitionSource =
                ::rock::provider::RockProviderEquippedWeaponTransitionSourceV1::MenuExit;
            break;
        case EquippedWeaponTransitionCoordinator::Source::WorkbenchExit:
            outState.transitionSource =
                ::rock::provider::RockProviderEquippedWeaponTransitionSourceV1::WorkbenchExit;
            break;
        }
        outState.terminalResult =
            static_cast<::rock::provider::RockProviderEquippedWeaponTransitionResultV1>(
                transition.terminalResult);
        if (outState.weaponFormId != 0 || outState.weaponGenerationKey != 0) {
            outState.flags |= static_cast<std::uint32_t>(Flag::Valid);
        }
        const auto setFlag = [&outState](const Flag flag, const bool enabled) {
            if (enabled) {
                outState.flags |= static_cast<std::uint32_t>(flag);
            }
        };
        setFlag(Flag::IdentityPending, transition.identityPending);
        setFlag(Flag::DrawPending, transition.drawPending);
        setFlag(Flag::BridgePresented, transition.bridgePresented);
        setFlag(Flag::NativeRenderable, transition.nativeRenderable);
        setFlag(
            Flag::HandPoseHandoffComplete,
            transition.handPoseHandoffComplete);
        setFlag(Flag::RecoveryExhausted, transition.recoveryExhausted);
        setFlag(Flag::TransitionActive, transition.active);
        outState.worldGeneration =
            _worldGenerationAtomic.load(std::memory_order_acquire);
        outState.skeletonGeneration =
            _skeletonGenerationAtomic.load(std::memory_order_acquire);
        outState.providerGeneration =
            _providerGenerationAtomic.load(std::memory_order_acquire);
        return (outState.flags & static_cast<std::uint32_t>(Flag::Valid)) != 0 ||
               transition.transitionSequence != 0 ||
               transition.terminalSequence != 0;
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponPartPosesV1(
        ::rock::provider::RockProviderWeaponPartPoseV1* outParts,
        const std::uint32_t maxParts) const
    {
        if (!outParts || maxParts == 0) {
            return 0;
        }
        using Flag = ::rock::provider::RockProviderWeaponPartPoseFlagV1;
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        const auto currentGeneration =
            _weaponCollision.getCurrentWeaponGenerationKey();
        if (!weaponNode || currentGeneration == 0 ||
            !finiteNiTransform(weaponNode->world)) {
            return 0;
        }

        const auto descriptors =
            _weaponCollision.getProfileEvidenceDescriptors();
        const auto count = static_cast<std::uint32_t>((std::min)(
            static_cast<std::size_t>(maxParts),
            descriptors.size()));
        const auto weaponInverse =
            transform_math::invertTransform(weaponNode->world);
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto& descriptor = descriptors[i];
            auto& out = outParts[i];
            out = {};
            out.frameIndex =
                _palmClockGameFrameIndex.load(std::memory_order_acquire);
            out.weaponGenerationKey = descriptor.weaponGenerationKey;
            out.bodyId = descriptor.bodyId;
            out.partKind = static_cast<std::uint32_t>(
                descriptor.semantic.partKind);
            out.omodFormId = descriptor.omodFormId;
            out.attachPointFormId = descriptor.semantic.attachPointFormId;
            copyProviderString(
                out.sourceName,
                sizeof(out.sourceName),
                descriptor.sourceName);
            if (!descriptor.valid ||
                descriptor.weaponGenerationKey != currentGeneration) {
                continue;
            }
            out.flags |= static_cast<std::uint32_t>(Flag::Valid);
            out.actionRole = static_cast<std::uint32_t>(
                descriptor.semantic.actionRole);
            auto* sourceNode = reinterpret_cast<RE::NiAVObject*>(
                descriptor.sourceRootAddress);
            if (!sourceNode || !finiteNiTransform(sourceNode->world)) {
                continue;
            }
            if (sourceNode->parent && finiteNiTransform(sourceNode->local)) {
                fillProviderTransform(
                    sourceNode->local,
                    out.sourceParentLocal);
                out.flags |= static_cast<std::uint32_t>(
                    Flag::SourceParentLocalValid);
            }
            const auto weaponRootLocal = transform_math::composeTransforms(
                weaponInverse,
                sourceNode->world);
            if (finiteNiTransform(weaponRootLocal)) {
                fillProviderTransform(
                    weaponRootLocal,
                    out.weaponRootLocal);
                out.flags |= static_cast<std::uint32_t>(
                    Flag::WeaponRootLocalValid);
            }
        }
        return count;
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponPartDriveResultsV1(
        const std::uint64_t ownerToken,
        ::rock::provider::RockProviderWeaponPartDriveApplicationResultV1* outResults,
        const std::uint32_t maxResults) const
    {
        if (ownerToken == 0 || !outResults || maxResults == 0) {
            return 0;
        }
        std::uint32_t copied = 0;
        const auto count = (std::min)(
            _providerWeaponPartDriveResultCount,
            static_cast<std::uint32_t>(
                _providerWeaponPartDriveResults.size()));
        for (std::uint32_t i = 0; i < count && copied < maxResults; ++i) {
            const auto& result = _providerWeaponPartDriveResults[i];
            if (result.ownerToken == ownerToken) {
                outResults[copied++] = result;
            }
        }
        return copied;
    }

    bool PhysicsInteraction::queryProviderScopeSightStateV1(
        ::rock::provider::RockProviderScopeSightStateV1& outState) const
    {
        using Flag = ::rock::provider::RockProviderScopeSightFlagV1;
        outState = {};
        const auto anchor =
            _weaponCollision.getNativeScopeSightAnchorSnapshot();
        if (anchor.weaponGenerationKey == 0 || anchor.weaponFormID == 0) {
            return false;
        }

        const NativeScopeResolvedAnchorSnapshot resolvedAnchor =
            _twoHandedGrip.getNativeScopeResolvedAnchorSnapshot();
        const native_scope_sight_anchor_policy::PublicationIdentity
            resolvedIdentity{
                .weaponGenerationKey = resolvedAnchor.weaponGenerationKey,
                .equippedWeaponOwnershipKey =
                    resolvedAnchor.equippedWeaponOwnershipKey,
                .weaponFormID = resolvedAnchor.weaponFormID,
            };
        const native_scope_sight_anchor_policy::PublicationIdentity
            publishedIdentity{
                .weaponGenerationKey = anchor.weaponGenerationKey,
                .equippedWeaponOwnershipKey =
                    anchor.equippedWeaponOwnershipKey,
                .weaponFormID = anchor.weaponFormID,
            };
        const bool resolvedAnchorMatchesPublication =
            resolvedAnchor.valid &&
            native_scope_sight_anchor_policy::matchesCurrentEquippedWeapon(
                resolvedIdentity,
                publishedIdentity);

        outState.weaponGenerationKey = anchor.weaponGenerationKey;
        outState.weaponFormId = anchor.weaponFormID;
        outState.nativeScopeOverlayIndex = anchor.nativeScopeOverlayIndex;
        outState.sightBodyCount = anchor.sightBodyCount;
        outState.anchorWeaponLocal = makeProviderPoint(
            resolvedAnchorMatchesPublication ?
                resolvedAnchor.anchorWeaponLocal :
                anchor.anchorWeaponLocal);
        outState.sightBoundsWeaponLocal.min = makeProviderPoint(
            anchor.sightBoundsMinWeaponLocal);
        outState.sightBoundsWeaponLocal.max = makeProviderPoint(
            anchor.sightBoundsMaxWeaponLocal);
        outState.sightBoundsWeaponLocal.valid = anchor.valid ? 1u : 0u;
        outState.flags |= static_cast<std::uint32_t>(Flag::Available);
        if (resolvedAnchorMatchesPublication || anchor.valid) {
            outState.flags |= static_cast<std::uint32_t>(Flag::AnchorValid);
        }
        if (anchor.valid) {
            outState.flags |= static_cast<std::uint32_t>(Flag::BoundsValid);
        }
        if (anchor.nativeScopeOverlayValid) {
            outState.flags |= static_cast<std::uint32_t>(
                Flag::NativeOverlayValid);
        }
        if (anchor.manualDirectTransitionRequired &&
            resolvedAnchorMatchesPublication) {
            outState.flags |= static_cast<std::uint32_t>(
                Flag::ManualDirectTransitionRequired);
        }

        const bool menuOpen = _twoHandedGrip.isScopeMenuOpenThisFrame();
        if (menuOpen) {
            outState.flags |=
                static_cast<std::uint32_t>(Flag::MenuOpen) |
                static_cast<std::uint32_t>(Flag::Active);
        }
        const auto activation =
            _twoHandedGrip.getNativeScopeActivationDebugSnapshot();
        outState.publicationSequence = activation.evaluationSequence;
        if (menuOpen &&
            activation.weaponGenerationKey == anchor.weaponGenerationKey) {
            if (activation.nativeScopeAlreadyActive ||
                activation.nativeGeometryDecision) {
                outState.activationSource =
                    ::rock::provider::RockProviderScopeActivationSourceV1::NativeGeometry;
            } else if (activation.rockGeometryDecision) {
                outState.activationSource =
                    ::rock::provider::RockProviderScopeActivationSourceV1::RockGeometry;
            } else {
                outState.activationSource =
                    ::rock::provider::RockProviderScopeActivationSourceV1::ManualInput;
            }
        }

        const auto descriptors =
            _weaponCollision.getProfileEvidenceDescriptors();
        for (const auto& descriptor : descriptors) {
            if (!descriptor.valid ||
                descriptor.weaponGenerationKey != anchor.weaponGenerationKey ||
                (descriptor.semantic.partKind != WeaponPartKind::Scope &&
                    descriptor.semantic.partKind != WeaponPartKind::Sight)) {
                continue;
            }
            outState.sightBodyId = descriptor.bodyId;
            outState.omodFormId = descriptor.omodFormId;
            outState.attachPointFormId =
                descriptor.semantic.attachPointFormId;
            if (descriptor.semantic.partKind == WeaponPartKind::Scope) {
                break;
            }
        }
        const auto composition =
            _weaponCollision.getWeaponCompositionSnapshot();
        if (outState.publicationSequence == 0) {
            outState.publicationSequence = composition.publicationSequence;
        }
        outState.frameIndex =
            _palmClockGameFrameIndex.load(std::memory_order_acquire);
        outState.worldGeneration =
            _worldGenerationAtomic.load(std::memory_order_acquire);
        outState.skeletonGeneration =
            _skeletonGenerationAtomic.load(std::memory_order_acquire);
        outState.providerGeneration =
            _providerGenerationAtomic.load(std::memory_order_acquire);
        return true;
    }

    bool PhysicsInteraction::queryProviderWeaponCompositionStateV1(
        ::rock::provider::RockProviderWeaponCompositionStateV1& outState) const
    {
        outState = {};
        const auto snapshot =
            _weaponCollision.getWeaponCompositionSnapshot();
        if (snapshot.weaponGenerationKey == 0) {
            return false;
        }
        outState.weaponGenerationKey = snapshot.weaponGenerationKey;
        outState.compositionSignature = snapshot.compositionSignature;
        outState.weaponFormId = snapshot.weaponFormId;
        outState.entryCount = snapshot.entryCount;
        outState.semanticCoverageMask = snapshot.semanticCoverageMask;
        outState.missingCoverageMask = snapshot.missingCoverageMask;
        outState.publicationSequence = snapshot.publicationSequence;
        return true;
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponCompositionEntriesV1(
        ::rock::provider::RockProviderWeaponCompositionEntryV1* outEntries,
        const std::uint32_t maxEntries) const
    {
        if (!outEntries || maxEntries == 0) {
            return 0;
        }
        const auto snapshot =
            _weaponCollision.getWeaponCompositionSnapshot();
        const auto count = (std::min)(
            maxEntries,
            static_cast<std::uint32_t>((std::min)(
                static_cast<std::size_t>(snapshot.entryCount),
                snapshot.entries.size())));
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto& source = snapshot.entries[i];
            auto& destination = outEntries[i];
            destination = {};
            destination.omodFormId = source.omodFormId;
            destination.attachPointFormId = source.attachPointFormId;
            destination.stableIndex = source.stableIndex;
            destination.flags = source.flags;
            destination.semanticCoverageMask = source.semanticCoverageMask;
        }
        return count;
    }

    bool PhysicsInteraction::queryProviderSelectedAuthoredGripPoseV1(
        ::rock::provider::RockProviderAuthoredGripPoseV1& outPose) const
    {
        using Flag = ::rock::provider::RockProviderAuthoredGripPoseFlagV1;
        outPose = {};
        SelectedAuthoredGripPoseSnapshot snapshot{};
        if (!_twoHandedGrip.getSelectedAuthoredGripPoseSnapshot(snapshot) ||
            !snapshot.valid) {
            return false;
        }
        outPose.weaponGenerationKey = snapshot.weaponGenerationKey;
        outPose.weaponFormId = currentEquippedWeaponFormId();
        outPose.source =
            static_cast<::rock::provider::RockProviderAuthoredGripSourceV1>(
                snapshot.source);
        outPose.variantKey = snapshot.variantKey;
        outPose.captureSequence = snapshot.captureSequence;
        outPose.flags = static_cast<std::uint32_t>(Flag::Valid);
        if (snapshot.rightHandValid) {
            fillProviderTransform(
                snapshot.rightHandWeaponLocal,
                outPose.rightHandInWeapon);
            outPose.flags |= static_cast<std::uint32_t>(
                Flag::RightHandValid);
        }
        if (snapshot.leftHandValid) {
            fillProviderTransform(
                snapshot.leftHandWeaponLocal,
                outPose.leftHandInWeapon);
            outPose.flags |= static_cast<std::uint32_t>(
                Flag::LeftHandValid);
        }
        outPose.rightFingerLocalTransformMask =
            snapshot.rightFingerLocalTransformMask;
        outPose.leftFingerLocalTransformMask =
            snapshot.leftFingerLocalTransformMask;
        for (std::size_t i = 0;
             i < snapshot.rightFingerLocalTransforms.size();
             ++i) {
            if ((snapshot.rightFingerLocalTransformMask & (1u << i)) != 0) {
                fillProviderTransform(
                    snapshot.rightFingerLocalTransforms[i],
                    outPose.rightFingerLocalTransforms[i]);
            }
            if ((snapshot.leftFingerLocalTransformMask & (1u << i)) != 0) {
                fillProviderTransform(
                    snapshot.leftFingerLocalTransforms[i],
                    outPose.leftFingerLocalTransforms[i]);
            }
        }
        if (snapshot.rightFingerLocalTransformMask != 0) {
            outPose.flags |= static_cast<std::uint32_t>(
                Flag::RightFingersValid);
        }
        if (snapshot.leftFingerLocalTransformMask != 0) {
            outPose.flags |= static_cast<std::uint32_t>(
                Flag::LeftFingersValid);
        }
        outPose.worldGeneration =
            _worldGenerationAtomic.load(std::memory_order_acquire);
        outPose.skeletonGeneration =
            _skeletonGenerationAtomic.load(std::memory_order_acquire);
        outPose.providerGeneration =
            _providerGenerationAtomic.load(std::memory_order_acquire);
        return true;
    }

    bool PhysicsInteraction::queryProviderPresentedHandPoseV1(
        const ::rock::provider::RockProviderHand hand,
        ::rock::provider::RockProviderPresentedHandPoseV1& outPose) const
    {
        using Flag = ::rock::provider::RockProviderPresentedHandPoseFlagV1;
        outPose = {};
        const bool isLeft =
            hand == ::rock::provider::RockProviderHand::Left;
        if (!frik_visual_authority::isAvailable() ||
            !frik_visual_authority::isSkeletonReadyHint()) {
            return false;
        }
        const auto handWorld = frik_visual_authority::getHandWorldTransform(
            isLeft ? frik_visual_authority::Hand::Left :
                     frik_visual_authority::Hand::Right);
        if (!finiteNiTransform(handWorld)) {
            return false;
        }
        outPose.hand = hand;
        outPose.flags =
            static_cast<std::uint32_t>(Flag::Valid) |
            static_cast<std::uint32_t>(Flag::HandWorldValid);
        fillProviderTransform(handWorld, outPose.handWorld);

        DirectSkeletonBoneSnapshot skeleton{};
        if (_providerPresentedPoseReader.capture(
                skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                skeleton)) {
            const auto findBone = [&skeleton](const char* name) ->
                const DirectSkeletonBoneEntry* {
                if (!name) {
                    return nullptr;
                }
                for (const auto& bone : skeleton.bones) {
                    if (bone.name == name) {
                        return &bone;
                    }
                }
                return nullptr;
            };
            for (std::size_t finger = 0; finger < 5; ++finger) {
                for (std::size_t segment = 0; segment < 3; ++segment) {
                    const auto poseIndex = finger * 3 + segment;
                    const auto* bone = findBone(
                        root_flattened_finger_skeleton_runtime::fingerBoneName(
                            isLeft,
                            finger,
                            segment));
                    if (!bone || bone->drawableParentSnapshotIndex < 0 ||
                        static_cast<std::size_t>(
                            bone->drawableParentSnapshotIndex) >=
                            skeleton.bones.size()) {
                        continue;
                    }
                    const auto& parent = skeleton.bones[
                        static_cast<std::size_t>(
                            bone->drawableParentSnapshotIndex)];
                    const auto local = transform_math::composeTransforms(
                        transform_math::invertTransform(parent.world),
                        bone->world);
                    if (!finiteNiTransform(local)) {
                        continue;
                    }
                    fillProviderTransform(
                        local,
                        outPose.fingerLocalTransforms[poseIndex]);
                    outPose.fingerLocalTransformMask |=
                        static_cast<std::uint16_t>(1u << poseIndex);
                }
            }
            if (outPose.fingerLocalTransformMask != 0) {
                outPose.flags |=
                    static_cast<std::uint32_t>(Flag::FingerLocalsValid) |
                    static_cast<std::uint32_t>(Flag::RootFlattenedReadback);
            }
        }
        outPose.worldGeneration =
            _worldGenerationAtomic.load(std::memory_order_acquire);
        outPose.skeletonGeneration =
            _skeletonGenerationAtomic.load(std::memory_order_acquire);
        outPose.providerGeneration =
            _providerGenerationAtomic.load(std::memory_order_acquire);
        return true;
    }

    std::uint32_t PhysicsInteraction::copyProviderSemanticHandContactsV1(
        const ::rock::provider::RockProviderHand handValue,
        const std::uint32_t maxFramesSinceContact,
        ::rock::provider::RockProviderSemanticHandContactV1* outContacts,
        const std::uint32_t maxContacts) const
    {
        if (!outContacts || maxContacts == 0) {
            return 0;
        }
        using Flag =
            ::rock::provider::RockProviderSemanticHandContactFlagV1;
        const bool isLeft =
            handValue == ::rock::provider::RockProviderHand::Left;
        const Hand& hand = isLeft ? _leftHand : _rightHand;
        ::rock::provider::RockProviderHandCollisionAvailabilityV1
            availability{};
        (void)queryProviderHandCollisionAvailabilityV1(
            handValue,
            availability);
        const bool collisionAvailable =
            (availability.flags & static_cast<std::uint32_t>(
                 ::rock::provider::RockProviderHandCollisionAvailabilityFlagV1::CollisionAvailable)) != 0;
        const bool transitionSuppressed =
            (availability.flags & static_cast<std::uint32_t>(
                 ::rock::provider::RockProviderHandCollisionAvailabilityFlagV1::TransitionSuppressed)) != 0;

        std::uint32_t copied = 0;
        for (const auto role : hand_collider_semantics::kHandColliderRoles) {
            if (copied >= maxContacts) {
                break;
            }
            hand_semantic_contact_state::SemanticContactRecord record{};
            if (!hand.getFreshSemanticContactForRole(
                    role,
                    maxFramesSinceContact,
                    record)) {
                continue;
            }
            auto& out = outContacts[copied++];
            out = {};
            out.frameIndex =
                _palmClockGameFrameIndex.load(std::memory_order_acquire);
            out.hand = handValue;
            out.role = static_cast<std::uint32_t>(record.role);
            out.finger = static_cast<std::uint32_t>(record.finger);
            out.segment = static_cast<std::uint32_t>(record.segment);
            out.handBodyId = record.handBodyId;
            out.targetBodyId = record.otherBodyId;
            if (record.framesSinceContact != 0) {
                out.contactState =
                    ::rock::provider::RockProviderSemanticContactStateV1::End;
            } else if (record.contactRunStartFrame == record.contactFrame) {
                out.contactState =
                    ::rock::provider::RockProviderSemanticContactStateV1::Begin;
            } else {
                out.contactState =
                    ::rock::provider::RockProviderSemanticContactStateV1::Continued;
            }
            out.framesSinceContact = record.framesSinceContact;
            out.contactSequence = record.sequence;
            if (record.hasContactPointGame) {
                out.flags |= static_cast<std::uint32_t>(
                    Flag::ContactPointValid);
                out.contactPointGame = {
                    record.contactPointGame.x,
                    record.contactPointGame.y,
                    record.contactPointGame.z,
                };
            }
            if (record.hasContactNormalGame) {
                out.flags |= static_cast<std::uint32_t>(
                    Flag::ContactNormalValid);
                out.contactNormalGame = {
                    record.contactNormalGame.x,
                    record.contactNormalGame.y,
                    record.contactNormalGame.z,
                };
            }
            if (_cachedBhkWorld && _cachedHknpWorld) {
                if (auto* target = resolveBodyToRef(
                        _cachedBhkWorld,
                        _cachedHknpWorld,
                        RE::hknpBodyId{ record.otherBodyId })) {
                    out.targetFormId = target->GetFormID();
                    out.flags |= static_cast<std::uint32_t>(
                        Flag::TargetFormResolved);
                }
            }
            if (hand.isHeldBodyId(record.otherBodyId)) {
                out.flags |= static_cast<std::uint32_t>(
                    Flag::HeldObjectRelation);
            }
            if (collisionAvailable) {
                out.flags |= static_cast<std::uint32_t>(
                    Flag::CollisionAvailable);
            }
            if (transitionSuppressed) {
                out.flags |= static_cast<std::uint32_t>(
                    Flag::TransitionSuppressed);
            }
            out.collisionGeneration =
                _collisionGenerationAtomic.load(std::memory_order_acquire);
        }
        return copied;
    }

    std::uint32_t PhysicsInteraction::copyProviderPlayerColliderDescriptorsV1(
        ::rock::provider::RockProviderPlayerColliderDescriptorV1* outDescriptors,
        const std::uint32_t maxDescriptors) const
    {
        if (!outDescriptors || maxDescriptors == 0) {
            return 0;
        }
        using Kind = ::rock::provider::RockProviderPlayerColliderKindV1;
        using Flag = ::rock::provider::RockProviderPlayerColliderFlagV1;
        const auto frameIndex =
            _palmClockGameFrameIndex.load(std::memory_order_acquire);
        const auto collisionGeneration =
            _collisionGenerationAtomic.load(std::memory_order_acquire);
        const auto lifecycleFlags =
            _lifecycleFlagsAtomic.load(std::memory_order_acquire);
        const bool enabled =
            ::rock::provider::hasLifecycleFlag(
                lifecycleFlags,
                ::rock::provider::RockProviderLifecycleFlag::GeneratedBodiesValid) &&
            ::rock::provider::hasLifecycleFlag(
                lifecycleFlags,
                ::rock::provider::RockProviderLifecycleFlag::PhysicsWriteAllowed);
        std::uint32_t copied = 0;

        const auto copyHand = [&](const Hand& hand, const bool isLeft) {
            const auto bodyCount = hand.getHandColliderBodyCount();
            for (std::uint32_t i = 0;
                 i < bodyCount && copied < maxDescriptors;
                 ++i) {
                const auto bodyId = hand.getHandColliderBodyIdAtomic(i);
                HandColliderBodyMetadata metadata{};
                if (!hand.tryGetHandColliderMetadata(bodyId, metadata)) {
                    continue;
                }
                auto& out = outDescriptors[copied++];
                out = {};
                out.frameIndex = frameIndex;
                out.kind = Kind::Hand;
                out.hand = isLeft ?
                    ::rock::provider::RockProviderHand::Left :
                    ::rock::provider::RockProviderHand::Right;
                out.bodyId = bodyId;
                out.role = static_cast<std::uint32_t>(metadata.role);
                out.descriptorIndex = i;
                out.flags = static_cast<std::uint32_t>(Flag::Valid);
                if (enabled) {
                    out.flags |= static_cast<std::uint32_t>(Flag::Enabled);
                }
                if (metadata.primaryPalmAnchor) {
                    out.flags |= static_cast<std::uint32_t>(
                        Flag::PrimaryPalmAnchor);
                    RE::NiTransform target{};
                    if (hand.tryGetPalmAnchorTarget(target) &&
                        finiteNiTransform(target)) {
                        fillProviderTransform(target, out.transform);
                        out.flags |= static_cast<std::uint32_t>(
                            Flag::TransformValid);
                    }
                }
                out.collisionGeneration = collisionGeneration;
            }
        };
        copyHand(_rightHand, false);
        copyHand(_leftHand, true);

        const auto bodyCount = _bodyBoneColliders.getBodyCount();
        for (std::uint32_t i = 0;
             i < bodyCount && copied < maxDescriptors;
             ++i) {
            const auto bodyId = _bodyBoneColliders.getBodyIdAtomic(i);
            BodyBoneColliderMetadata metadata{};
            if (!_bodyBoneColliders.tryGetBodyMetadataAtomic(
                    bodyId,
                    metadata)) {
                continue;
            }
            auto& out = outDescriptors[copied++];
            out = {};
            out.frameIndex = frameIndex;
            out.kind = Kind::Body;
            out.bodyId = bodyId;
            out.role = static_cast<std::uint32_t>(metadata.role);
            out.zone = static_cast<::rock::provider::RockProviderBodyZoneKind>(
                metadata.zone);
            out.side = static_cast<::rock::provider::RockProviderBodyZoneSide>(
                metadata.side);
            out.descriptorIndex = metadata.descriptorIndex;
            out.lengthGameUnits = metadata.lengthGameUnits;
            out.radiusGameUnits = metadata.radiusGameUnits;
            out.flags = static_cast<std::uint32_t>(Flag::Valid);
            if (enabled) {
                out.flags |= static_cast<std::uint32_t>(Flag::Enabled);
            }
            if (metadata.inPowerArmor) {
                out.flags |= static_cast<std::uint32_t>(
                    Flag::InPowerArmor);
            }
            out.collisionGeneration = collisionGeneration;
        }
        return copied;
    }

    bool PhysicsInteraction::queryProviderHandCollisionAvailabilityV1(
        const ::rock::provider::RockProviderHand handValue,
        ::rock::provider::RockProviderHandCollisionAvailabilityV1& outState) const
    {
        using Flag =
            ::rock::provider::RockProviderHandCollisionAvailabilityFlagV1;
        outState = {};
        const bool isLeft =
            handValue == ::rock::provider::RockProviderHand::Left;
        const Hand& hand = isLeft ? _leftHand : _rightHand;
        outState.hand = handValue;
        outState.frameIndex =
            _palmClockGameFrameIndex.load(std::memory_order_acquire);
        outState.collisionGeneration =
            _collisionGenerationAtomic.load(std::memory_order_acquire);
        outState.handBodyCount = hand.getHandColliderBodyCount();
        if (outState.handBodyCount != 0) {
            outState.flags |= static_cast<std::uint32_t>(Flag::BodiesReady);
        }

        dynamic_hand_collision_telemetry::Snapshot telemetry{};
        if (_dynamicHandCollision.getTelemetrySnapshot(telemetry)) {
            const auto& handTelemetry = telemetry.hands[isLeft ? 1u : 0u];
            outState.collisionSequence = telemetry.updateSequence;
            for (const auto& twin : handTelemetry.twins) {
                if (twin.bodyCreated) {
                    ++outState.dynamicTwinCount;
                }
            }
            if (outState.dynamicTwinCount == handTelemetry.twins.size()) {
                outState.flags |= static_cast<std::uint32_t>(
                    Flag::DynamicTwinsReady);
            }
            if (telemetry.physicsWritesAllowed) {
                outState.flags |= static_cast<std::uint32_t>(
                    Flag::PhysicsWritesAllowed);
            }
            if (telemetry.menuBlocked) {
                outState.flags |= static_cast<std::uint32_t>(
                    Flag::MenuSuppressed);
            }
            if (telemetry.transitionCollisionSuppressed) {
                outState.flags |= static_cast<std::uint32_t>(
                    Flag::TransitionSuppressed);
            }
            if (handTelemetry.handDisabled) {
                outState.flags |= static_cast<std::uint32_t>(
                    Flag::HandDisabled);
            }
            if (outState.handBodyCount != 0 && telemetry.worldReady &&
                telemetry.physicsWritesAllowed && !telemetry.menuBlocked &&
                !telemetry.transitionCollisionSuppressed &&
                !handTelemetry.handDisabled) {
                outState.flags |= static_cast<std::uint32_t>(
                    Flag::CollisionAvailable);
            }
        }
        outState.worldGeneration =
            _worldGenerationAtomic.load(std::memory_order_acquire);
        outState.skeletonGeneration =
            _skeletonGenerationAtomic.load(std::memory_order_acquire);
        outState.providerGeneration =
            _providerGenerationAtomic.load(std::memory_order_acquire);
        return true;
    }
