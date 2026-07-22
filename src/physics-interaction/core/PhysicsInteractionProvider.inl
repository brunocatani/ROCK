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
