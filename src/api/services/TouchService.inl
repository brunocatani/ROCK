// Private Touch service operations. Included after shared owner/lifecycle helpers.

    api::Status 
    apiSetTouchGrabTargetsForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const ::rock::api::touch::TouchGrabTarget* targets,
        const std::uint32_t targetCount)
    {
        if (ownerToken == 0 || scopeToken == 0 ||
            targetCount > ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGETS_V1 ||
            (targetCount != 0 && !targets)) {
            return api::Status::InvalidArgument;
        }
        for (std::uint32_t index = 0; index < targetCount; ++index) {
            if (targets[index].size !=
                sizeof(::rock::api::touch::TouchGrabTarget)) {
                return api::Status::InvalidSize;
            }
            if (targets[index].version == 0 ||
                targets[index].version > ROCK_PROVIDER_API_VERSION) {
                return api::Status::UnsupportedVersion;
            }
            const auto generationResult = validateGenerationGuards(
                targets[index].worldGeneration,
                targets[index].skeletonGeneration,
                targets[index].providerGeneration);
            if (generationResult != api::Status::Ok) {
                return generationResult;
            }
        }

        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        switch (s_touchGrabTargets.setScope(
            ownerToken,
            scopeToken,
            targets,
            targetCount,
            currentProviderFrameIndex())) {
        case TouchGrabRegistry::RegistrationResult::Ok:
            return api::Status::Ok;
        case TouchGrabRegistry::RegistrationResult::CapacityFull:
            return api::Status::CapacityFull;
        case TouchGrabRegistry::RegistrationResult::OwnerConflict:
            return api::Status::OwnerConflict;
        case TouchGrabRegistry::RegistrationResult::InvalidArgument:
        default:
            return api::Status::InvalidArgument;
        }
    }

    api::Status 
    apiClearTouchGrabTargetsForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken)
    {
        if (ownerToken == 0 || scopeToken == 0) {
            return api::Status::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        return s_touchGrabTargets.clearScope(ownerToken, scopeToken) ?
            api::Status::Ok :
            api::Status::TargetUnavailable;
    }

    api::Status 
    apiCopyTouchGrabStatesForScopeV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        ::rock::api::touch::TouchGrabState* outStates,
        const std::uint32_t maxStates,
        std::uint32_t* outStateCount)
    {
        if (outStateCount) *outStateCount = 0;
        if (ownerToken == 0 || scopeToken == 0 || !outStateCount ||
            (maxStates != 0 && !outStates)) {
            return api::Status::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        *outStateCount = s_touchGrabTargets.copyStates(
            ownerToken,
            scopeToken,
            outStates,
            maxStates,
            currentProviderFrameIndex());
        return api::Status::Ok;
    }

    api::Status 
    apiRequestTouchGrabYieldV1(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const std::uint64_t targetId,
        const std::uint32_t targetGeneration)
    {
        if (ownerToken == 0 || scopeToken == 0 || targetId == 0 ||
            targetGeneration == 0) {
            return api::Status::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_touchGrabMutex);
        const auto ownerResult =
            validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        return s_touchGrabTargets.requestYield(
                   ownerToken,
                   scopeToken,
                   targetId,
                   targetGeneration,
                   currentProviderFrameIndex()) ?
            api::Status::Ok :
            api::Status::TargetUnavailable;
    }
