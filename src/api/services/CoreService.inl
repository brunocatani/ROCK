// Private Core service operations. Included after shared owner/lifecycle helpers.

    const char* apiGetModVersion()
    {
        return Version::NAME.data();
    }

    bool apiIsProviderReady()
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        return pi && pi->isProviderReady();
    }

    bool copyPublication(Publication& outPublication)
    {
        std::scoped_lock lock(s_snapshotMutex);
        if (!s_hasSnapshot) {
            return false;
        }
        outPublication = s_lastSnapshot;
        return true;
    }

    api::Status registerOwner(
        const api::core::Registration& registration,
        api::core::Owner& outOwner)
    {
        const auto modNameLength = boundedStringLength(registration.modName, sizeof(registration.modName));
        if (modNameLength == 0 || modNameLength >= sizeof(registration.modName)) {
            return api::Status::InvalidArgument;
        }
        const auto providerGeneration = currentProviderGenerationForRegistration();

        std::scoped_lock lock(s_consumerMutex);
        for (const auto& slot : s_consumers) {
            if (modNameEquals(slot, registration.modName, modNameLength)) {
                return api::Status::OwnerConflict;
            }
        }

        for (auto& slot : s_consumers) {
            if (slot.token != 0) {
                continue;
            }

            slot = {};
            slot.token = nextConsumerToken();
            slot.interfaces[0] = {rock::api::core::kMajor, 1};
            const auto eventStatus = events::bind(slot.token, rock::api::InterfaceId::Core);
            if (eventStatus != rock::api::Status::Ok) {
                slot = {};
                return eventStatus;
            }
            slot.providerGeneration = providerGeneration;
            std::memcpy(slot.modName, registration.modName, modNameLength);
            outOwner.ownerToken = slot.token;
            outOwner.providerGeneration = slot.providerGeneration;
            return api::Status::Ok;
        }

        return api::Status::CapacityFull;
    }

    api::Status unregisterOwner(std::uint64_t ownerToken)
    {
        if(const auto status=native_interaction_service::canUnregisterOwner(ownerToken);status!=rock::api::Status::Ok)
            return static_cast<api::Status>(status);
        native_interaction_service::clearOwner(ownerToken);
        loose_weapon_state::clear(ownerToken);
        physical_reload_input::clearOwner(ownerToken);
        support_trigger_input::clearOwner(ownerToken);
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
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
                return api::Status::OwnerNotRegistered;
            }
            // Registration can run concurrently. Retire all event slots before
            // this owner slot becomes available to another registration.
            events::remove(ownerToken);
            *slot = {};
            clearInteractionCommandsForOwnerLocked(ownerToken, api::grab::InteractionFailure::OwnerNotRegistered);
            clearHandInputSuppressionsForOwnerLocked(
                ownerToken,
                api::Hand::None,
                ::rock::api::input::SuppressionInvalidationReason::OwnerUnregistered);
            clearWeaponPartTargetsForOwnerLocked(ownerToken);
            clearWeaponPartDrivesForOwnerLocked(ownerToken);
            clearNativeAnimationAuthorityForOwnerLocked(ownerToken);
            clearEquippedWeaponHandlingAuthorityForOwnerLocked(ownerToken);
        }

        {
            std::scoped_lock lock(s_touchGrabMutex);
            s_touchGrabTargets.clearOwner(ownerToken);
        }

        {
            std::scoped_lock lock(s_offhandReservationMutex);
            if (s_offhandReservationSlot.ownerToken == ownerToken) {
                clearOffhandReservationLocked(
                    ::rock::api::input::SuppressionInvalidationReason::OwnerUnregistered);
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
            api::Hand::None,
            true);
        clearNativeAnimationRuntimePublicationForOwner(ownerToken);
        saved_grab_offset::clearDefaultGrabPosesForOwner(ownerToken);

        { const auto access=s_physicsInteraction.borrow(); if (auto* pi=access.get()) pi->releaseProviderPowerArmorGrabs(ownerToken); }
        return api::Status::Ok;
    }

    api::Status registerAnimationPhaseCallback(
        const std::uint64_t ownerToken,
        api::animation::PhaseCallback callback,
        void* userData,
        std::uint64_t* outCallbackToken)
    {
        if (!callback || !outCallbackToken) {
            return api::Status::InvalidArgument;
        }
        *outCallbackToken = 0;

        std::scoped_lock lock(s_consumerMutex, s_animationPhaseCallbackMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }

        AnimationPhaseCallbackSlot* available = nullptr;
        for (auto& slot : s_animationPhaseCallbacks) {
            if (slot.ownerToken == ownerToken) {
                return api::Status::OwnerConflict;
            }
            if (!slot.callback && !available) {
                available = &slot;
            }
        }
        if (!available) {
            return api::Status::CapacityFull;
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
        return api::Status::Ok;
    }

    api::Status unregisterAnimationPhaseCallback(
        const std::uint64_t ownerToken,
        const std::uint64_t callbackToken)
    {
        if (ownerToken == 0 || callbackToken == 0) {
            return api::Status::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_animationPhaseCallbackMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_animationPhaseCallbacks) {
            if (slot.token == callbackToken && slot.ownerToken == ownerToken) {
                slot = {};
                return api::Status::Ok;
            }
        }
        return api::Status::TargetUnavailable;
    }

    api::Status registerFrameCallback(
        const std::uint64_t ownerToken,
        api::core::FrameCallback callback,
        void* userData,
        std::uint64_t* outCallbackToken)
    {
        if (ownerToken == 0 || !callback || !outCallbackToken) {
            return api::Status::InvalidArgument;
        }
        *outCallbackToken = 0;

        std::scoped_lock lock(s_consumerMutex, s_callbackMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
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
                return api::Status::Ok;
            }
        }
        return api::Status::CapacityFull;
    }

    api::Status unregisterFrameCallback(
        const std::uint64_t ownerToken,
        const std::uint64_t callbackToken)
    {
        if (ownerToken == 0 || callbackToken == 0) {
            return api::Status::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_callbackMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_callbacks) {
            if (slot.token != callbackToken) {
                continue;
            }
            if (slot.ownerToken != ownerToken) {
                return api::Status::OwnerConflict;
            }
            slot = {};
            return api::Status::Ok;
        }
        return api::Status::TargetUnavailable;
    }
