// Private Weapon service operations. Included after shared owner/lifecycle helpers.

    api::Hand primaryHand()
    {
        // ROCK's firing role is the only primary/offhand authority. Native
        // Fallout/FRIK handedness never changes ROCK controller identity.
        const bool primaryIsLeft =
            s_equippedWeaponFiringHandIsLeft.load(std::memory_order_acquire);
        return primaryIsLeft ? api::Hand::Left : api::Hand::Right;
    }

    api::Hand offhandHand()
    {
        return primaryHand() == api::Hand::Left ? api::Hand::Right : api::Hand::Left;
    }

    api::Status getWeaponClassification(api::weapon::Classification& outClassification)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        return pi->queryProviderWeaponClassification(outClassification) ?
            api::Status::Ok :
            api::Status::NotReady;
    }

    std::uint32_t weaponEmitterCount()
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return 0;
        }
        return pi->getProviderWeaponEmitterCount();
    }

    std::uint32_t copyWeaponEmitters(
        api::weapon::Emitter* outEmitters,
        const std::uint32_t capacity)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return 0;
        }
        return pi->copyProviderWeaponEmitters(outEmitters, capacity);
    }

    api::Status getWeaponGrip(api::weapon::Grip& outGrip)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        return pi && pi->isInitialized() && pi->queryProviderWeaponGrip(outGrip) ?
            api::Status::Ok :
            api::Status::NotReady;
    }

    api::Status setHandlingAuthority(
        const std::uint64_t ownerToken,
        const api::weapon::HandlingRequest& request)
    {
        using Feature = api::weapon::HandlingFeature;
        constexpr std::uint32_t implementedFeatures =
            static_cast<std::uint32_t>(Feature::FiringGripOwnership) |
            static_cast<std::uint32_t>(Feature::PrimaryDetach) |
            static_cast<std::uint32_t>(Feature::AmbidextrousHandoff) |
            static_cast<std::uint32_t>(Feature::GripZoneEquip) |
            static_cast<std::uint32_t>(Feature::GripZoneHoverHaptics) |
            static_cast<std::uint32_t>(Feature::FiringGripProximitySupport);
        // Detach, handoff and grip-zone equip act on the firing-grip owner.
        constexpr std::uint32_t ownershipDependentFeatures =
            static_cast<std::uint32_t>(Feature::PrimaryDetach) |
            static_cast<std::uint32_t>(Feature::AmbidextrousHandoff) |
            static_cast<std::uint32_t>(Feature::GripZoneEquip);
        constexpr auto ownershipFeature = static_cast<std::uint32_t>(Feature::FiringGripOwnership);
        if (request.features == 0 ||
            (request.features & ~implementedFeatures) != 0 ||
            request.leaseFrames == 0 ||
            ((request.features & ownershipDependentFeatures) != 0 &&
                (request.features & ownershipFeature) == 0) ||
            !handlingRequestValuesValid(request)) {
            return api::Status::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            request.worldGeneration,
            request.skeletonGeneration,
            request.providerGeneration);
        if (generationResult != api::Status::Ok) {
            return generationResult;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_equippedWeaponHandlingAuthorityMutex);
        pruneExpiredEquippedWeaponHandlingAuthorityLocked(frameIndex);
        if (s_equippedWeaponHandlingAuthority.active &&
            s_equippedWeaponHandlingAuthority.ownerToken != ownerToken) {
            return api::Status::OwnerConflict;
        }

        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request.leaseFrames,
            api::weapon::kMaxHandlingLeaseFrames);
        s_equippedWeaponHandlingAuthority.active = true;
        s_equippedWeaponHandlingAuthority.ownerToken = ownerToken;
        s_equippedWeaponHandlingAuthority.expiresAfterFrame =
            provider_lease_policy::exclusiveExpiryFrame(
                frameIndex,
                leaseFrames);
        s_equippedWeaponHandlingAuthority.request = request;
        s_equippedWeaponHandlingAuthority.request.leaseFrames = leaseFrames;
        return api::Status::Ok;
    }

    api::Status clearHandlingAuthority(const std::uint64_t ownerToken)
    {
        std::scoped_lock lock(s_equippedWeaponHandlingAuthorityMutex);
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
            output.features = s_equippedWeaponHandlingAuthority.request.features;
            output.owner = s_equippedWeaponHandlingAuthority.ownerToken;
            output.expiresAfterFrame = s_equippedWeaponHandlingAuthority.expiresAfterFrame;
        }
        return api::Status::Ok;
    }

    bool queryWeaponHandling(provider_state_policy::WeaponHandling& outHandling)
    {
        outHandling = {};
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        return pi && pi->isInitialized() && pi->queryProviderWeaponHandling(outHandling);
    }

    api::Status copyPublishedWeaponState(api::weapon::WeaponState& outState)
    {
        std::scoped_lock lock(s_snapshotMutex);
        if (!s_hasSnapshot) {
            return api::Status::NotReady;
        }
        outState = s_lastEquippedWeaponState;
        return api::Status::Ok;
    }

    api::Status getScopeSightState(api::weapon::ScopeSightState& outState)
    {
        const auto published = sample();
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        if (!pi->queryProviderScopeSightState(outState)) {
            return api::Status::TargetUnavailable;
        }
        outState.sample.collisionGeneration = published.collisionGeneration;
        return api::Status::Ok;
    }

    api::Status getWeaponCompositionState(api::weapon::CompositionState& outState)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        return pi->queryProviderWeaponCompositionState(outState) ?
            api::Status::Ok :
            api::Status::TargetUnavailable;
    }

    api::Status copyWeaponCompositionEntries(
        api::weapon::CompositionEntry* outEntries,
        const std::uint32_t capacity,
        std::uint32_t& outCount)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        outCount = pi->copyProviderWeaponCompositionEntries(outEntries, capacity);
        return api::Status::Ok;
    }

    api::Status getAuthoredGripPose(api::weapon::AuthoredGripPose& outPose)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        return pi->queryProviderAuthoredGripPose(outPose) ?
            api::Status::Ok :
            api::Status::TargetUnavailable;
    }

