// Private Diagnostics service operations. Included after shared owner/lifecycle helpers.

    RockProviderResultV1 ROCK_PROVIDER_CALL apiPublishDebugOverlayV1(
        const std::uint64_t ownerToken,
        const RockProviderDebugOverlayPublicationV1* publication)
    {
        if (ownerToken == 0 || !publication) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        if (publication->size !=
            sizeof(RockProviderDebugOverlayPublicationV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (publication->version == 0 ||
            publication->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (publication->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            publication->worldGeneration,
            publication->skeletonGeneration,
            publication->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }
        if (!apiIsProviderReady()) {
            return RockProviderResultV1::NotReady;
        }
        {
            std::scoped_lock lock(s_consumerMutex);
            const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::DebugOverlayPublication);
            if (ownerResult != RockProviderResultV1::Ok) {
                return ownerResult;
            }
        }
        return provider_debug_overlay::publish(
            ownerToken,
            *publication,
            currentProviderFrameIndex());
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearDebugOverlayV1(
        const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        {
            std::scoped_lock lock(s_consumerMutex);
            const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::DebugOverlayPublication);
            if (ownerResult != RockProviderResultV1::Ok) {
                return ownerResult;
            }
        }
        provider_debug_overlay::clear(ownerToken);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
        apiSetColliderVisualizationOverrideV1(
            const std::uint64_t ownerToken,
            const RockProviderColliderVisualizationRequestV1* request)
    {
        if (ownerToken == 0 || !request) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->size !=
            sizeof(RockProviderColliderVisualizationRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version != ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->weaponGenerationKey == 0 ||
            request->bodyId == 0x7FFF'FFFF ||
            request->leaseFrames == 0) {
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
        if (!pi) {
            return RockProviderResultV1::NotReady;
        }
        const auto capabilityResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::
                ColliderVisualizationOverride);
        if (capabilityResult != RockProviderResultV1::Ok) {
            return capabilityResult;
        }

        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot ||
                request->weaponGenerationKey !=
                    s_lastSnapshot.weaponGenerationKey) {
                return RockProviderResultV1::TargetUnavailable;
            }
        }
        if (!pi->isProviderWeaponBodyCurrentV1(
                request->weaponGenerationKey,
                request->bodyId)) {
            return RockProviderResultV1::TargetInvalid;
        }

        return provider_collider_visualization::set(
            ownerToken,
            *request,
            currentProviderFrameIndex());
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
        apiClearColliderVisualizationOverrideV1(
            const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        const auto capabilityResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::
                ColliderVisualizationOverride);
        if (capabilityResult != RockProviderResultV1::Ok) {
            return capabilityResult;
        }
        provider_collider_visualization::clear(ownerToken);
        return RockProviderResultV1::Ok;
    }
