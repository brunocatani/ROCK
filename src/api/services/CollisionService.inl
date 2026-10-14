// Private Collision service operations. Included after shared owner/lifecycle helpers.

    std::uint32_t apiGetBodyContactSnapshotV1(
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

    api::Status apiCopySemanticHandContactsV1(
        const std::uint64_t ownerToken,
        const api::Hand hand,
        const std::uint32_t maxFramesSinceContact,
        RockProviderSemanticHandContactV1* outContacts,
        const std::uint32_t maxContacts,
        std::uint32_t* outContactCount)
    {
        if (outContactCount) *outContactCount = 0;
        if (ownerToken == 0 || !outContactCount ||
            (hand != api::Hand::Right && hand != api::Hand::Left) ||
            (maxContacts != 0 && !outContacts)) {
            return api::Status::InvalidArgument;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::SemanticHandContacts);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        if (!onAnimationOwnerThread()) {
            return api::Status::WrongThread;
        }
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        *outContactCount = pi->copyProviderSemanticHandContactsV1(
            hand,
            maxFramesSinceContact,
            outContacts,
            maxContacts);
        return api::Status::Ok;
    }

    api::Status apiCopyPlayerColliderDescriptorsV1(
        const std::uint64_t ownerToken,
        RockProviderPlayerColliderDescriptorV1* outDescriptors,
        const std::uint32_t maxDescriptors,
        std::uint32_t* outDescriptorCount)
    {
        if (outDescriptorCount) *outDescriptorCount = 0;
        if (ownerToken == 0 || !outDescriptorCount ||
            (maxDescriptors != 0 && !outDescriptors)) {
            return api::Status::InvalidArgument;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::PlayerColliderDescriptors);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        if (!onAnimationOwnerThread()) {
            return api::Status::WrongThread;
        }
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        *outDescriptorCount = pi->copyProviderPlayerColliderDescriptorsV1(
            outDescriptors,
            maxDescriptors);
        return api::Status::Ok;
    }

    api::Status apiGetHandCollisionAvailabilityV1(
        const std::uint64_t ownerToken,
        const api::Hand hand,
        RockProviderHandCollisionAvailabilityV1* outState)
    {
        if (hand != api::Hand::Right && hand != api::Hand::Left) {
            provider_state_policy::clearQueryOutput(outState);
            return api::Status::HandUnavailable;
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

    api::Status apiQueryWorldRaycastV1(
        const std::uint64_t ownerToken,
        const RockProviderWorldRaycastRequestV1* request,
        RockProviderWorldRaycastResultV1* outResult)
    {
        provider_state_policy::clearQueryOutput(outResult);
        if (ownerToken == 0 || !request || !outResult) {
            return api::Status::InvalidArgument;
        }
        if (request->size != sizeof(RockProviderWorldRaycastRequestV1) ||
            outResult->size != sizeof(RockProviderWorldRaycastResultV1)) {
            return api::Status::InvalidSize;
        }
        if (request->version != ROCK_PROVIDER_API_VERSION ||
            outResult->version != ROCK_PROVIDER_API_VERSION) {
            return api::Status::UnsupportedVersion;
        }
        if (!finiteProviderPoint(request->startGame) ||
            !finiteProviderPoint(request->directionGame) ||
            !std::isfinite(request->maxDistanceGame) ||
            request->maxDistanceGame <= 0.0f ||
            request->maxDistanceGame >
                ROCK_PROVIDER_MAX_WORLD_RAYCAST_DISTANCE_GAME_V1) {
            return api::Status::InvalidArgument;
        }
        const float directionLengthSquared =
            request->directionGame.x * request->directionGame.x +
            request->directionGame.y * request->directionGame.y +
            request->directionGame.z * request->directionGame.z;
        if (!std::isfinite(directionLengthSquared) ||
            directionLengthSquared <= 1.0e-8f) {
            return api::Status::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return api::Status::WrongThread;
        }
        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != api::Status::Ok) {
            return generationResult;
        }
        if (!apiIsProviderReady()) {
            return api::Status::NotReady;
        }

        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }

        const auto frameIndex = currentGameFrameIndex();
        {
            std::scoped_lock lock(s_consumerMutex);
            auto* slot = findConsumerSlotLocked(ownerToken);
            if (!slot) {
                return api::Status::OwnerNotRegistered;
            }
            if (!hasConsumerCapabilityV1(
                    slot->grantedCapabilities,
                    RockProviderConsumerCapabilityV1::WorldRaycasts)) {
                return api::Status::PermissionDenied;
            }
            if (slot->worldRaycastFrameIndex != frameIndex) {
                slot->worldRaycastFrameIndex = frameIndex;
                slot->worldRaycastCount = 0;
            }
            if (slot->worldRaycastCount >=
                ROCK_PROVIDER_MAX_WORLD_RAYCASTS_PER_OWNER_PER_FRAME_V1) {
                return api::Status::CapacityFull;
            }
            ++slot->worldRaycastCount;
        }

        *outResult = {};
        outResult->frameIndex = frameIndex;
        if (!pi->queryProviderWorldRaycastV1(*request, *outResult)) {
            return api::Status::WorldNotReady;
        }
        return api::Status::Ok;
    }
