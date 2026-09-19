// Private Core service operations. Included after shared owner/lifecycle helpers.

    const char* ROCK_PROVIDER_CALL apiGetModVersion()
    {
        return Version::NAME.data();
    }

    bool ROCK_PROVIDER_CALL apiIsProviderReady()
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        return pi && pi->isProviderReady();
    }

    bool ROCK_PROVIDER_CALL apiGetFrameSnapshot(RockProviderFrameSnapshot* outSnapshot)
    {
        provider_state_policy::clearQueryOutput(outSnapshot, ROCK_PROVIDER_FRAME_SNAPSHOT_V1_SIZE);
        if (!outSnapshot || outSnapshot->size < ROCK_PROVIDER_FRAME_SNAPSHOT_V1_SIZE) {
            return false;
        }

        const auto requestedSize = outSnapshot->size;
        std::scoped_lock lock(s_snapshotMutex);
        if (!s_hasSnapshot) {
            return false;
        }

        const auto copySize = (std::min<std::size_t>)(requestedSize, sizeof(RockProviderFrameSnapshot));
        std::memcpy(outSnapshot, &s_lastSnapshot, copySize);
        outSnapshot->size = static_cast<std::uint32_t>(copySize);
        return true;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterConsumerV1(
        const RockProviderConsumerRegistrationV1* registration,
        RockProviderConsumerHandleV1* outHandle)
    {
        if (!registration || !outHandle) {
            return RockProviderResultV1::InvalidArgument;
        }

        if (registration->size != sizeof(RockProviderConsumerRegistrationV1) || outHandle->size != sizeof(RockProviderConsumerHandleV1)) {
            return RockProviderResultV1::InvalidSize;
        }

        if (registration->version == 0 || registration->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }

        const auto modNameLength = boundedStringLength(registration->modName, sizeof(registration->modName));
        if (modNameLength == 0 || modNameLength >= sizeof(registration->modName)) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto grantedCapabilities = registration->requestedCapabilities & kImplementedConsumerCapabilitiesV1;
        const auto providerGeneration = currentProviderGenerationForRegistration();

        std::scoped_lock lock(s_consumerMutex);
        for (const auto& slot : s_consumers) {
            if (modNameEquals(slot, registration->modName, modNameLength)) {
                return RockProviderResultV1::OwnerConflict;
            }
        }

        for (auto& slot : s_consumers) {
            if (slot.token != 0) {
                continue;
            }

            slot = {};
            slot.token = nextConsumerToken();
            slot.grantedCapabilities = grantedCapabilities;
            slot.interfaces[0] = {1,1};
            events::bind(slot.token, rock::api::InterfaceId::Core);
            slot.providerGeneration = providerGeneration;
            std::memcpy(slot.modName, registration->modName, modNameLength);

            *outHandle = {};
            outHandle->size = sizeof(RockProviderConsumerHandleV1);
            outHandle->version = ROCK_PROVIDER_API_VERSION;
            outHandle->ownerToken = slot.token;
            outHandle->grantedCapabilities = slot.grantedCapabilities;
            outHandle->providerGeneration = slot.providerGeneration;
            return RockProviderResultV1::Ok;
        }

        return RockProviderResultV1::CapacityFull;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiUnregisterConsumerV1(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        {
            std::scoped_lock lock(
                s_consumerMutex,
                s_interactionCommandMutex,
                s_handInputSuppressionMutex,
                s_weaponPartMutex,
                s_nativeAnimationAuthorityMutex,
                s_equippedWeaponHandlingAuthorityMutex);
            auto* slot = findConsumerSlotLocked(ownerToken);
            if (!slot) {
                return RockProviderResultV1::OwnerNotRegistered;
            }
            *slot = {};
            clearInteractionCommandsForOwnerLocked(ownerToken, RockProviderInteractionFailureV1::OwnerNotRegistered);
            clearHandInputSuppressionsForOwnerLocked(
                ownerToken,
                RockProviderHand::None,
                RockProviderSuppressionInvalidationReasonV1::OwnerUnregistered);
            clearWeaponPartTargetsForOwnerLocked(ownerToken);
            clearWeaponPartDrivesForOwnerLocked(ownerToken);
            clearNativeAnimationAuthorityForOwnerLocked(ownerToken);
            clearEquippedWeaponHandlingAuthorityForOwnerLocked(ownerToken);
        }

        {
            std::scoped_lock lock(s_externalBodyMutex);
            s_externalBodies.clearOwner(ownerToken);
        }
        {
            std::scoped_lock lock(s_touchGrabMutex);
            s_touchGrabTargets.clearOwner(ownerToken);
        }

        {
            std::scoped_lock lock(s_offhandReservationMutex);
            if (s_offhandReservationSlot.ownerToken == ownerToken) {
                clearOffhandReservationLocked(
                    RockProviderSuppressionInvalidationReasonV1::OwnerUnregistered);
            }
        }

        {
            std::scoped_lock lock(s_callbackMutex);
            for (auto& callback : s_callbacks) {
                if (callback.ownerToken == ownerToken) {
                    callback = {};
                }
            }
        }

        {
            std::scoped_lock lock(s_animationPhaseCallbackMutex);
            clearAnimationPhaseCallbacksForOwnerLocked(ownerToken);
        }
        (void)clearHandVisualAuthorityForOwner(
            ownerToken,
            RockProviderHand::None,
            true);
        clearNativeAnimationRuntimePublicationForOwner(ownerToken);
        provider_debug_overlay::clear(ownerToken);
        provider_collider_visualization::clear(ownerToken);

        { const auto access=s_physicsInteraction.borrow(); if (auto* pi=access.get()) pi->releaseProviderPowerArmorGrabs(ownerToken); }
        events::remove(ownerToken);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterAnimationPhaseCallbackV1(
        const std::uint64_t ownerToken,
        RockProviderAnimationPhaseCallbackV1 callback,
        void* userData,
        std::uint64_t* outCallbackToken)
    {
        if (!callback || !outCallbackToken) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCallbackToken = 0;

        std::scoped_lock lock(s_consumerMutex, s_animationPhaseCallbackMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::AnimationPhases);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        AnimationPhaseCallbackSlot* available = nullptr;
        for (auto& slot : s_animationPhaseCallbacks) {
            if (slot.ownerToken == ownerToken) {
                return RockProviderResultV1::OwnerConflict;
            }
            if (!slot.callback && !available) {
                available = &slot;
            }
        }
        if (!available) {
            return RockProviderResultV1::CapacityFull;
        }

        auto token = s_nextAnimationPhaseCallbackToken.fetch_add(
            1,
            std::memory_order_acq_rel);
        if (token == 0) {
            token = s_nextAnimationPhaseCallbackToken.fetch_add(
                1,
                std::memory_order_acq_rel);
        }
        *available = AnimationPhaseCallbackSlot{
            .token = token,
            .ownerToken = ownerToken,
            .callback = callback,
            .userData = userData,
        };
        *outCallbackToken = token;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiUnregisterAnimationPhaseCallbackV1(
        const std::uint64_t ownerToken,
        const std::uint64_t callbackToken)
    {
        if (ownerToken == 0 || callbackToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_animationPhaseCallbackMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::AnimationPhases);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_animationPhaseCallbacks) {
            if (slot.token == callbackToken && slot.ownerToken == ownerToken) {
                slot = {};
                return RockProviderResultV1::Ok;
            }
        }
        return RockProviderResultV1::TargetUnavailable;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRegisterFrameCallbackForOwnerV1(
        const std::uint64_t ownerToken,
        RockProviderFrameCallback callback,
        void* userData,
        std::uint64_t* outCallbackToken)
    {
        if (ownerToken == 0 || !callback || !outCallbackToken) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCallbackToken = 0;

        std::scoped_lock lock(s_consumerMutex, s_callbackMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::FrameSnapshots);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_callbacks) {
            if (!slot.callback) {
                auto token = s_nextCallbackToken.fetch_add(
                    1,
                    std::memory_order_acq_rel);
                if (token == 0) {
                    token = s_nextCallbackToken.fetch_add(
                        1,
                        std::memory_order_acq_rel);
                }
                slot = CallbackSlot{
                    .token = token,
                    .ownerToken = ownerToken,
                    .callback = callback,
                    .userData = userData,
                };
                *outCallbackToken = token;
                return RockProviderResultV1::Ok;
            }
        }
        return RockProviderResultV1::CapacityFull;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiUnregisterFrameCallbackForOwnerV1(
        const std::uint64_t ownerToken,
        const std::uint64_t callbackToken)
    {
        if (ownerToken == 0 || callbackToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_callbackMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::FrameSnapshots);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_callbacks) {
            if (slot.token != callbackToken) {
                continue;
            }
            if (slot.ownerToken != ownerToken) {
                return RockProviderResultV1::OwnerConflict;
            }
            slot = {};
            return RockProviderResultV1::Ok;
        }
        return RockProviderResultV1::TargetUnavailable;
    }
