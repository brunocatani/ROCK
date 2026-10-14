// Private Collision service operations. Included after shared owner/lifecycle helpers.
// They fill provider API major-2 Collision records directly; the endpoint has
// already authorized the owner and validated the caller's records.

    std::uint32_t copyBodyContacts(
        api::collision::BodyContact* outContacts,
        const std::uint32_t capacity)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return 0;
        }
        return pi->copyProviderBodyContacts(outContacts, capacity);
    }

    api::Status copySemanticHandContacts(
        const api::Hand hand,
        const std::uint32_t maxFramesSinceContact,
        api::collision::SemanticHandContact* outContacts,
        const std::uint32_t capacity,
        std::uint32_t& outCount)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        outCount = pi->copyProviderSemanticHandContacts(hand, maxFramesSinceContact, outContacts, capacity);
        return api::Status::Ok;
    }

    api::Status copyPlayerColliderDescriptors(
        api::collision::PlayerColliderDescriptor* outDescriptors,
        const std::uint32_t capacity,
        std::uint32_t& outCount)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        outCount = pi->copyProviderPlayerColliderDescriptors(outDescriptors, capacity);
        return api::Status::Ok;
    }

    api::Status getHandCollisionAvailability(
        const api::Hand hand,
        api::collision::HandCollisionAvailability& outState)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        return pi->queryProviderHandCollisionAvailability(hand, outState) ?
            api::Status::Ok :
            api::Status::TargetUnavailable;
    }

    api::Status queryWorldRaycast(
        const std::uint64_t ownerToken,
        const api::collision::WorldRaycastRequest& request,
        api::collision::WorldRaycastResult& outResult)
    {
        if (!finiteProviderPoint(request.startGame) ||
            !finiteProviderPoint(request.directionGame) ||
            !std::isfinite(request.maxDistanceGame) ||
            request.maxDistanceGame <= 0.0f ||
            request.maxDistanceGame > api::collision::kMaxRaycastDistanceGame) {
            return api::Status::InvalidArgument;
        }
        const float directionLengthSquared =
            request.directionGame.x * request.directionGame.x +
            request.directionGame.y * request.directionGame.y +
            request.directionGame.z * request.directionGame.z;
        if (!std::isfinite(directionLengthSquared) ||
            directionLengthSquared <= 1.0e-8f) {
            return api::Status::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            request.worldGeneration,
            request.skeletonGeneration,
            request.providerGeneration);
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

        // Each owner gets a bounded number of casts per game frame.
        const auto frameIndex = currentGameFrameIndex();
        {
            std::scoped_lock lock(s_consumerMutex);
            auto* slot = findConsumerSlotLocked(ownerToken);
            if (!slot) {
                return api::Status::OwnerNotRegistered;
            }
            if (slot->worldRaycastFrameIndex != frameIndex) {
                slot->worldRaycastFrameIndex = frameIndex;
                slot->worldRaycastCount = 0;
            }
            if (slot->worldRaycastCount >= api::collision::kMaxRaycastsPerOwnerPerFrame) {
                return api::Status::CapacityFull;
            }
            ++slot->worldRaycastCount;
        }

        outResult.frameIndex = frameIndex;
        if (!pi->queryProviderWorldRaycast(request, outResult)) {
            return api::Status::WorldNotReady;
        }
        return api::Status::Ok;
    }
