// Private Collision service operations. Included after shared owner/lifecycle helpers.

    void ROCK_PROVIDER_CALL apiClearExternalBodies(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return;
        }
        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        if (!consumerHasCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::ExternalBodies)) {
            return;
        }
        s_externalBodies.clearOwner(ownerToken);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetBodyContactSnapshotV1(
        RockProviderBodyContactV1* outContacts,
        std::uint32_t maxContacts)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderBodyContacts(outContacts, maxContacts);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterExternalBodiesForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const RockProviderExternalBodyRegistration* bodies,
        const std::uint32_t bodyCount)
    {
        if (ownerToken == 0 || scopeToken == 0 ||
            (bodyCount != 0 && !bodies)) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::ExternalBodyScopes);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        switch (s_externalBodies.registerBodiesForScopeDetailed(
            ownerToken,
            scopeToken,
            bodies,
            bodyCount)) {
        case ExternalBodyRegistry::RegistrationResult::Ok:
            return RockProviderResultV1::Ok;
        case ExternalBodyRegistry::RegistrationResult::CapacityFull:
            return RockProviderResultV1::CapacityFull;
        case ExternalBodyRegistry::RegistrationResult::OwnerConflict:
            return RockProviderResultV1::OwnerConflict;
        case ExternalBodyRegistry::RegistrationResult::InvalidArgument:
        default:
            return RockProviderResultV1::InvalidArgument;
        }
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearExternalBodiesForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken)
    {
        if (ownerToken == 0 || scopeToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::ExternalBodyScopes);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        return s_externalBodies.clearScope(ownerToken, scopeToken) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyExternalContactsSinceV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const std::uint64_t afterSequence,
        RockProviderExternalContactRecordV1* outContacts,
        const std::uint32_t maxContacts,
        RockProviderExternalContactStreamStateV1* outStreamState)
    {
        provider_state_policy::clearQueryOutput(outStreamState);
        if (ownerToken == 0 || !outStreamState ||
            outStreamState->size < sizeof(RockProviderExternalContactStreamStateV1) ||
            (maxContacts != 0 && !outContacts)) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_externalBodyMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::ExternalBodyScopes);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (!consumerHasCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::ExternalContacts)) {
            return RockProviderResultV1::PermissionDenied;
        }
        (void)s_externalBodies.copyContactsSinceV1(
            ownerToken,
            scopeToken,
            afterSequence,
            outContacts,
            maxContacts,
            *outStreamState);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopySemanticHandContactsV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        const std::uint32_t maxFramesSinceContact,
        RockProviderSemanticHandContactV1* outContacts,
        const std::uint32_t maxContacts,
        std::uint32_t* outContactCount)
    {
        if (outContactCount) *outContactCount = 0;
        if (ownerToken == 0 || !outContactCount ||
            (hand != RockProviderHand::Right && hand != RockProviderHand::Left) ||
            (maxContacts != 0 && !outContacts)) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::SemanticHandContacts);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        *outContactCount = pi->copyProviderSemanticHandContactsV1(
            hand,
            maxFramesSinceContact,
            outContacts,
            maxContacts);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyPlayerColliderDescriptorsV1(
        const std::uint64_t ownerToken,
        RockProviderPlayerColliderDescriptorV1* outDescriptors,
        const std::uint32_t maxDescriptors,
        std::uint32_t* outDescriptorCount)
    {
        if (outDescriptorCount) *outDescriptorCount = 0;
        if (ownerToken == 0 || !outDescriptorCount ||
            (maxDescriptors != 0 && !outDescriptors)) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::PlayerColliderDescriptors);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        *outDescriptorCount = pi->copyProviderPlayerColliderDescriptorsV1(
            outDescriptors,
            maxDescriptors);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetHandCollisionAvailabilityV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        RockProviderHandCollisionAvailabilityV1* outState)
    {
        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            provider_state_policy::clearQueryOutput(outState);
            return RockProviderResultV1::HandUnavailable;
        }
        return queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::PlayerColliderDescriptors,
            outState,
            [hand](PhysicsInteraction& pi, RockProviderHandCollisionAvailabilityV1& state) {
                return pi.queryProviderHandCollisionAvailabilityV1(hand, state);
            },
            true);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiQueryWorldRaycastV1(
        const std::uint64_t ownerToken,
        const RockProviderWorldRaycastRequestV1* request,
        RockProviderWorldRaycastResultV1* outResult)
    {
        provider_state_policy::clearQueryOutput(outResult);
        if (ownerToken == 0 || !request || !outResult) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->size != sizeof(RockProviderWorldRaycastRequestV1) ||
            outResult->size != sizeof(RockProviderWorldRaycastResultV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version != ROCK_PROVIDER_API_VERSION ||
            outResult->version != ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (!finiteProviderPoint(request->startGame) ||
            !finiteProviderPoint(request->directionGame) ||
            !std::isfinite(request->maxDistanceGame) ||
            request->maxDistanceGame <= 0.0f ||
            request->maxDistanceGame >
                ROCK_PROVIDER_MAX_WORLD_RAYCAST_DISTANCE_GAME_V1) {
            return RockProviderResultV1::InvalidArgument;
        }
        const float directionLengthSquared =
            request->directionGame.x * request->directionGame.x +
            request->directionGame.y * request->directionGame.y +
            request->directionGame.z * request->directionGame.z;
        if (!std::isfinite(directionLengthSquared) ||
            directionLengthSquared <= 1.0e-8f) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }
        if (!apiIsProviderReady()) {
            return RockProviderResultV1::NotReady;
        }

        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }

        const auto frameIndex = currentGameFrameIndex();
        {
            std::scoped_lock lock(s_consumerMutex);
            auto* slot = findConsumerSlotLocked(ownerToken);
            if (!slot) {
                return RockProviderResultV1::OwnerNotRegistered;
            }
            if (!hasConsumerCapabilityV1(
                    slot->grantedCapabilities,
                    RockProviderConsumerCapabilityV1::WorldRaycasts)) {
                return RockProviderResultV1::PermissionDenied;
            }
            if (slot->worldRaycastFrameIndex != frameIndex) {
                slot->worldRaycastFrameIndex = frameIndex;
                slot->worldRaycastCount = 0;
            }
            if (slot->worldRaycastCount >=
                ROCK_PROVIDER_MAX_WORLD_RAYCASTS_PER_OWNER_PER_FRAME_V1) {
                return RockProviderResultV1::CapacityFull;
            }
            ++slot->worldRaycastCount;
        }

        *outResult = {};
        outResult->frameIndex = frameIndex;
        if (!pi->queryProviderWorldRaycastV1(*request, *outResult)) {
            return RockProviderResultV1::WorldNotReady;
        }
        return RockProviderResultV1::Ok;
    }
