// Private Touch service operations. Included after shared owner/lifecycle helpers.

    RockProviderResultV1 ROCK_PROVIDER_CALL
    apiSetTouchGrabTargetsForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const RockProviderTouchGrabTargetV1* targets,
        const std::uint32_t targetCount)
    {
        if (ownerToken == 0 || scopeToken == 0 ||
            targetCount > ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGETS_V1 ||
            (targetCount != 0 && !targets)) {
            return RockProviderResultV1::InvalidArgument;
        }
        for (std::uint32_t index = 0; index < targetCount; ++index) {
            if (targets[index].size !=
                sizeof(RockProviderTouchGrabTargetV1)) {
                return RockProviderResultV1::InvalidSize;
            }
            if (targets[index].version == 0 ||
                targets[index].version > ROCK_PROVIDER_API_VERSION) {
                return RockProviderResultV1::UnsupportedVersion;
            }
            const auto generationResult = validateGenerationGuards(
                targets[index].worldGeneration,
                targets[index].skeletonGeneration,
                targets[index].providerGeneration);
            if (generationResult != RockProviderResultV1::Ok) {
                return generationResult;
            }
        }

        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::TouchGrabTargets);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        switch (s_touchGrabTargets.setScope(
            ownerToken,
            scopeToken,
            targets,
            targetCount,
            currentProviderFrameIndex())) {
        case TouchGrabRegistry::RegistrationResult::Ok:
            return RockProviderResultV1::Ok;
        case TouchGrabRegistry::RegistrationResult::CapacityFull:
            return RockProviderResultV1::CapacityFull;
        case TouchGrabRegistry::RegistrationResult::OwnerConflict:
            return RockProviderResultV1::OwnerConflict;
        case TouchGrabRegistry::RegistrationResult::InvalidArgument:
        default:
            return RockProviderResultV1::InvalidArgument;
        }
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
    apiClearTouchGrabTargetsForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken)
    {
        if (ownerToken == 0 || scopeToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::TouchGrabTargets);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        return s_touchGrabTargets.clearScope(ownerToken, scopeToken) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
    apiCopyTouchGrabStatesForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        RockProviderTouchGrabStateV1* outStates,
        const std::uint32_t maxStates,
        std::uint32_t* outStateCount)
    {
        if (outStateCount) *outStateCount = 0;
        if (ownerToken == 0 || scopeToken == 0 || !outStateCount ||
            (maxStates != 0 && !outStates)) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::TouchGrabTargets);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        *outStateCount = s_touchGrabTargets.copyStates(
            ownerToken,
            scopeToken,
            outStates,
            maxStates,
            currentProviderFrameIndex());
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL
    apiRequestTouchGrabYieldV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const std::uint64_t targetId,
        const std::uint32_t targetGeneration)
    {
        if (ownerToken == 0 || scopeToken == 0 || targetId == 0 ||
            targetGeneration == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::TouchGrabTargets);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        return s_touchGrabTargets.requestYield(
                   ownerToken,
                   scopeToken,
                   targetId,
                   targetGeneration,
                   currentProviderFrameIndex()) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
    }
