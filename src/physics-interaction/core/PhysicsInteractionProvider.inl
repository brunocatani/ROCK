/*
 * Provider snapshot and query glue is split from lifecycle/update orchestration because it exposes ROCK state to external consumers without owning physics behavior. It stays in this translation unit to preserve helper visibility and public API behavior.
 */
    void PhysicsInteraction::fillProviderFrameSnapshot(::rock::provider::RockProviderFrameSnapshot& outSnapshot) const
    {
        auto* api = frik::api::FRIKApi::inst;
        outSnapshot.providerReady = _initialized.load(std::memory_order_acquire) ? 1u : 0u;
        outSnapshot.frikSkeletonReady = api && api->isSkeletonReady() ? 1u : 0u;
        outSnapshot.menuBlocking = ((api && api->isAnyMenuOpen()) || input_remap_runtime::isMenuInputActive()) ? 1u : 0u;
        outSnapshot.configBlocking = api && (api->isConfigOpen() || api->isWristPipboyOpen()) ? 1u : 0u;
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

    std::uint32_t PhysicsInteraction::copyProviderWeaponEvidenceDescriptors(
        ::rock::provider::RockProviderWeaponEvidenceDescriptor* outDescriptors,
        std::uint32_t maxDescriptors) const
    {
        if (!outDescriptors || maxDescriptors == 0) {
            return 0;
        }

        const auto descriptors = _weaponCollision.getProfileEvidenceDescriptors();
        const std::uint32_t count = (std::min)(maxDescriptors, static_cast<std::uint32_t>(descriptors.size()));
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto& descriptor = descriptors[i];
            auto& out = outDescriptors[i];
            out = {};
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
            copyProviderString(out.sourceName, sizeof(out.sourceName), descriptor.sourceName);
        }

        return count;
    }

    std::uint32_t PhysicsInteraction::getProviderWeaponEvidenceDetailCountV3() const
    {
        return static_cast<std::uint32_t>(_weaponCollision.getProfileEvidenceDescriptors().size());
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponEvidenceDetailsV3(
        ::rock::provider::RockProviderWeaponEvidenceDetailV3* outDetails,
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
            out.size = sizeof(::rock::provider::RockProviderWeaponEvidenceDetailV3);
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
        }

        return count;
    }

    std::uint32_t PhysicsInteraction::getProviderWeaponEvidenceDetailPointCountV3(std::uint32_t bodyId) const
    {
        WeaponCollisionProfileEvidenceDescriptor descriptor{};
        RE::NiAVObject* sourceNode = nullptr;
        if (!_weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(bodyId, descriptor, sourceNode)) {
            return 0;
        }

        return descriptor.pointCount;
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponEvidenceDetailPointsV3(
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

    std::uint32_t PhysicsInteraction::copyProviderBodyContactsV6(
        ::rock::provider::RockProviderBodyContactV6* outContacts,
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
            out.size = sizeof(::rock::provider::RockProviderBodyContactV6);
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
