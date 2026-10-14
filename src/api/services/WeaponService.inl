// Private Weapon service operations. Included after shared owner/lifecycle helpers.

    api::Hand apiGetPrimaryHandV1()
    {
        // ROCK's firing role is the only primary/offhand authority. Native
        // Fallout/FRIK handedness never changes ROCK controller identity.
        const bool primaryIsLeft =
            s_equippedWeaponFiringHandIsLeft.load(std::memory_order_acquire);
        return primaryIsLeft ? api::Hand::Left : api::Hand::Right;
    }

    api::Hand apiGetOffhandHandV1()
    {
        return apiGetPrimaryHandV1() == api::Hand::Left ? api::Hand::Right : api::Hand::Left;
    }

    bool apiQueryEquippedWeaponClassificationV1(RockProviderWeaponClassificationV1* outResult)
    {
        provider_state_policy::clearQueryOutput(outResult);
        if (!outResult || outResult->size != sizeof(RockProviderWeaponClassificationV1)) {
            return false;
        }

        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return false;
        }

        return pi->queryProviderEquippedWeaponClassificationV1(*outResult);
    }

    std::uint32_t apiGetWeaponEmitterCountV1()
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return 0;
        }
        return pi->getProviderWeaponEmitterCountV1();
    }

    std::uint32_t apiCopyWeaponEmittersV1(
        RockProviderWeaponEmitterV1* outEmitters,
        std::uint32_t maxEmitters)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return 0;
        }
        return pi->copyProviderWeaponEmittersV1(outEmitters, maxEmitters);
    }

    bool apiGetEquippedWeaponGripStateV1(
        const std::uint64_t ownerToken,
        RockProviderEquippedWeaponGripStateV1* outState)
    {
        provider_state_policy::clearQueryOutput(outState);
        if (!outState ||
            outState->size != sizeof(RockProviderEquippedWeaponGripStateV1)) {
            return false;
        }
        if (!onAnimationOwnerThread()) {
            return false;
        }
        {
            std::scoped_lock lock(s_consumerMutex);
            if (!consumerHasCapabilityLocked(
                    ownerToken,
                    RockProviderConsumerCapabilityV1::EquippedWeaponGripState)) {
                return false;
            }
        }

        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        return pi && pi->isInitialized() &&
               pi->queryProviderEquippedWeaponGripStateV1(*outState);
    }

    api::Status apiSetEquippedWeaponHandlingAuthorityV1(
        const std::uint64_t ownerToken,
        const RockProviderEquippedWeaponHandlingRequestV1* request)
    {
        if (ownerToken == 0 || !request) {
            return api::Status::InvalidArgument;
        }
        if (request->size != sizeof(RockProviderEquippedWeaponHandlingRequestV1)) {
            return api::Status::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return api::Status::UnsupportedVersion;
        }
        constexpr std::uint32_t implementedFlags =
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::AmbidextrousHandoff) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::GripZoneEquip) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::GripZoneHoverHaptics) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::FiringGripProximitySupport) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::EquippedWeaponShoulderStash) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::PipboyTriggerHandEquip) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::EquipVisualBridge);
        const auto ownershipFlag = static_cast<std::uint32_t>(
            RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership);
        const auto ownershipDependentFlags =
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::AmbidextrousHandoff) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::GripZoneEquip) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::EquippedWeaponShoulderStash) |
            static_cast<std::uint32_t>(RockProviderEquippedWeaponHandlingFlagV1::PipboyTriggerHandEquip);
        const auto primaryDetachFlag = static_cast<std::uint32_t>(
            RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach);
        const auto shoulderStashFlag = static_cast<std::uint32_t>(
            RockProviderEquippedWeaponHandlingFlagV1::EquippedWeaponShoulderStash);
        if (request->flags == 0 ||
            (request->flags & ~implementedFlags) != 0 ||
            request->leaseFrames == 0 ||
            ((request->flags & ownershipDependentFlags) != 0 &&
                (request->flags & ownershipFlag) == 0) ||
            ((request->flags & shoulderStashFlag) != 0 &&
                (request->flags & primaryDetachFlag) == 0) ||
            !equippedWeaponHandlingRequestValuesValid(*request)) {
            return api::Status::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != api::Status::Ok) {
            return generationResult;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(
            s_consumerMutex,
            s_equippedWeaponHandlingAuthorityMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::EquippedWeaponHandlingAuthority);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        pruneExpiredEquippedWeaponHandlingAuthorityLocked(frameIndex);
        if (s_equippedWeaponHandlingAuthority.active &&
            s_equippedWeaponHandlingAuthority.ownerToken != ownerToken) {
            return api::Status::OwnerConflict;
        }

        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request->leaseFrames,
            ROCK_PROVIDER_MAX_EQUIPPED_WEAPON_HANDLING_LEASE_FRAMES_V1);
        s_equippedWeaponHandlingAuthority.active = true;
        s_equippedWeaponHandlingAuthority.ownerToken = ownerToken;
        s_equippedWeaponHandlingAuthority.expiresAfterFrame =
            provider_lease_policy::exclusiveExpiryFrame(
                frameIndex,
                leaseFrames);
        s_equippedWeaponHandlingAuthority.request = *request;
        s_equippedWeaponHandlingAuthority.request.size =
            sizeof(RockProviderEquippedWeaponHandlingRequestV1);
        s_equippedWeaponHandlingAuthority.request.version =
            ROCK_PROVIDER_API_VERSION;
        s_equippedWeaponHandlingAuthority.request.leaseFrames = leaseFrames;
        return api::Status::Ok;
    }

    api::Status apiClearEquippedWeaponHandlingAuthorityV1(
        const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        std::scoped_lock lock(
            s_consumerMutex,
            s_equippedWeaponHandlingAuthorityMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::EquippedWeaponHandlingAuthority);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        if (s_equippedWeaponHandlingAuthority.active &&
            s_equippedWeaponHandlingAuthority.ownerToken != ownerToken) {
            return api::Status::OwnerConflict;
        }
        clearEquippedWeaponHandlingAuthorityForOwnerLocked(ownerToken);
        return api::Status::Ok;
    }

    api::Status getHandlingAuthority(api::weapon::HandlingAuthority& output)
    {
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_equippedWeaponHandlingAuthorityMutex);
        pruneExpiredEquippedWeaponHandlingAuthorityLocked(frameIndex);
        if (s_equippedWeaponHandlingAuthority.active) {
            output.active = 1;
            output.features = s_equippedWeaponHandlingAuthority.request.flags;
            output.owner = s_equippedWeaponHandlingAuthority.ownerToken;
            output.expiresAfterFrame = s_equippedWeaponHandlingAuthority.expiresAfterFrame;
        }
        return api::Status::Ok;
    }

    bool apiGetEquippedWeaponHandlingStateV1(
        RockProviderEquippedWeaponHandlingStateV1* outState)
    {
        provider_state_policy::clearQueryOutput(outState);
        if (!outState ||
            outState->size != sizeof(RockProviderEquippedWeaponHandlingStateV1)) {
            return false;
        }
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized() ||
            !pi->queryProviderEquippedWeaponHandlingStateV1(*outState)) {
            return false;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_equippedWeaponHandlingAuthorityMutex);
        pruneExpiredEquippedWeaponHandlingAuthorityLocked(frameIndex);
        if (s_equippedWeaponHandlingAuthority.active) {
            outState->authorityFlags =
                s_equippedWeaponHandlingAuthority.request.flags;
            outState->ownerToken =
                s_equippedWeaponHandlingAuthority.ownerToken;
            outState->expiresAfterFrame =
                s_equippedWeaponHandlingAuthority.expiresAfterFrame;
            outState->runtimeFlags |= static_cast<std::uint32_t>(
                RockProviderEquippedWeaponHandlingRuntimeFlagV1::AuthorityActive);
        }
        return true;
    }

    api::Status apiGetEquippedWeaponStateV1(
        const std::uint64_t ownerToken,
        RockProviderEquippedWeaponStateV1* outState)
    {
        provider_state_policy::clearQueryOutput(outState);
        if (ownerToken == 0 || !outState) {
            return api::Status::InvalidArgument;
        }
        if (outState->size < sizeof(RockProviderEquippedWeaponStateV1)) {
            return api::Status::InvalidSize;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::FrameSnapshots);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        std::scoped_lock lock(s_snapshotMutex);
        if (!s_hasSnapshot) {
            return api::Status::NotReady;
        }
        *outState = s_lastEquippedWeaponState;
        return api::Status::Ok;
    }

    api::Status apiGetScopeSightStateV1(
        const std::uint64_t ownerToken,
        RockProviderScopeSightStateV1* outState)
    {
        return queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::ScopeSightState,
            outState,
            [](PhysicsInteraction& pi, RockProviderScopeSightStateV1& state) {
                return pi.queryProviderScopeSightStateV1(state);
            },
            true);
    }

    api::Status apiGetWeaponCompositionStateV1(
        const std::uint64_t ownerToken,
        RockProviderWeaponCompositionStateV1* outState)
    {
        return queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::WeaponComposition,
            outState,
            [](PhysicsInteraction& pi, RockProviderWeaponCompositionStateV1& state) {
                return pi.queryProviderWeaponCompositionStateV1(state);
            });
    }

    api::Status apiCopyWeaponCompositionEntriesV1(
        const std::uint64_t ownerToken,
        RockProviderWeaponCompositionEntryV1* outEntries,
        const std::uint32_t maxEntries,
        std::uint32_t* outEntryCount)
    {
        if (outEntryCount) *outEntryCount = 0;
        if (ownerToken == 0 || !outEntryCount ||
            (maxEntries != 0 && !outEntries)) {
            return api::Status::InvalidArgument;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::WeaponComposition);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        *outEntryCount = pi->copyProviderWeaponCompositionEntriesV1(
            outEntries,
            maxEntries);
        return api::Status::Ok;
    }

    api::Status apiGetSelectedAuthoredGripPoseV1(
        const std::uint64_t ownerToken,
        RockProviderAuthoredGripPoseV1* outPose)
    {
        return queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::PoseReadback,
            outPose,
            [](PhysicsInteraction& pi, RockProviderAuthoredGripPoseV1& pose) {
                return pi.queryProviderSelectedAuthoredGripPoseV1(pose);
            },
            true);
    }
