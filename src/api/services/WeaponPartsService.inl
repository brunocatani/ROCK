// Private WeaponParts service operations. Included after shared owner/lifecycle helpers.

    bool queryWeaponContact(
        const api::weaponparts::ContactQuery& query,
        api::weaponparts::ContactResult& outResult)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        return pi && pi->isInitialized() && pi->queryProviderWeaponContactAtPoint(query, outResult);
    }

    std::uint32_t copyWeaponEvidenceDetails(
        api::weaponparts::EvidenceDetail* outDetails,
        std::uint32_t capacity)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderWeaponEvidenceDetails(outDetails, capacity);
    }

    std::uint32_t copyWeaponEvidencePoints(
        std::uint32_t bodyId,
        api::Point3* outPoints,
        std::uint32_t capacity)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderWeaponEvidencePoints(bodyId, outPoints, capacity);
    }

    api::Status setWeaponPartTargets(
        std::uint64_t ownerToken,
        std::span<const PartTarget> targets)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        if (targets.size() > api::weaponparts::kMaxTargets) {
            return api::Status::CapacityFull;
        }

        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }

        for (const auto& target : targets) {
            if (!isValidWeaponPartGrabMode(target.grabMode) ||
                !hasValidWeaponPartMatcher(target.flags, target.bodyId, target.sourceRoot, target.sourceName) ||
                !hasValidWeaponPartTargetSemantics(target)) {
                return api::Status::InvalidArgument;
            }
        }

        if (targets.size() > availableWeaponPartTargetSlotsForOwnerLocked(ownerToken)) {
            return api::Status::CapacityFull;
        }

        clearWeaponPartTargetsForOwnerLocked(ownerToken);
        for (const auto& target : targets) {
            bool stored = false;
            for (auto& slot : s_weaponPartTargets) {
                if (!slot.active) {
                    slot.active = true;
                    slot.ownerToken = ownerToken;
                    slot.target = target;
                    slot.target.sourceName[std::size(slot.target.sourceName) - 1] = '\0';
                    stored = true;
                    break;
                }
            }
            if (!stored) {
                clearWeaponPartTargetsForOwnerLocked(ownerToken);
                return api::Status::CapacityFull;
            }
        }
        return api::Status::Ok;
    }

    api::Status clearWeaponPartTargets(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        clearWeaponPartTargetsForOwnerLocked(ownerToken);
        return api::Status::Ok;
    }

    api::Status setWeaponPartDrives(
        std::uint64_t ownerToken,
        std::span<const PartDrive> drives)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        if (drives.size() > api::weaponparts::kMaxDrives) {
            return api::Status::CapacityFull;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }

        for (const auto& drive : drives) {
            if (!isValidWeaponPartDriveSpace(drive.driveSpace) ||
                drive.leaseFrames == 0 ||
                !isFiniteProviderTransform(drive.targetTransform) ||
                !hasConcreteWeaponPartDriveMatcher(drive.flags, drive.bodyId, drive.sourceRoot, drive.sourceName)) {
                return api::Status::InvalidArgument;
            }
        }

        pruneExpiredWeaponPartDrivesLocked(frameIndex);
        if (drives.size() > availableWeaponPartDriveSlotsForOwnerLocked(ownerToken)) {
            return api::Status::CapacityFull;
        }
        clearWeaponPartDrivesForOwnerLocked(ownerToken);
        for (const auto& drive : drives) {
            const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
                drive.leaseFrames,
                api::weaponparts::kMaxDriveLeaseFrames);
            const auto expiresAfterFrame =
                provider_lease_policy::exclusiveExpiryFrame(
                    frameIndex,
                    leaseFrames);
            bool stored = false;
            for (auto& slot : s_weaponPartDrives) {
                if (!slot.active) {
                    slot.active = true;
                    slot.ownerToken = ownerToken;
                    slot.expiresAfterFrame = expiresAfterFrame;
                    slot.target = drive;
                    slot.target.sourceName[std::size(slot.target.sourceName) - 1] = '\0';
                    stored = true;
                    break;
                }
            }
            if (!stored) {
                clearWeaponPartDrivesForOwnerLocked(ownerToken);
                return api::Status::CapacityFull;
            }
        }
        return api::Status::Ok;
    }

    api::Status clearWeaponPartDrives(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        clearWeaponPartDrivesForOwnerLocked(ownerToken);
        return api::Status::Ok;
    }

    bool copyPartGripState(api::Hand hand, api::weaponparts::GripState& outState)
    {
        if (hand != api::Hand::Right && hand != api::Hand::Left) {
            return false;
        }

        std::scoped_lock lock(s_snapshotMutex);
        if (!isProviderReady() || !s_hasSnapshot || s_lastSnapshot.providerReady == 0) {
            return false;
        }
        outState = s_lastPartGripStates[hand == api::Hand::Left ? 1u : 0u];
        return true;
    }

    api::Status queryWeaponPartTargetResolution(
        const provider::PartQuery& query,
        api::weaponparts::ResolutionResult& outResult)
    {
        weapon_part_runtime::Resolution resolution{};
        (void)resolveWeaponPartTarget(query, resolution);
        outResult = {};
        outResult.whitelistActive = resolution.whitelistActive ? 1u : 0u;
        outResult.matched = resolution.matched ? 1u : 0u;
        outResult.grabMode = static_cast<api::weaponparts::GrabMode>(resolution.grabMode);
        outResult.groupId = resolution.groupId;
        outResult.priority = resolution.priority;
        outResult.winningOwner = resolution.ownerToken;
        outResult.weaponGenerationKey = query.weaponGenerationKey;
        outResult.frameIndex = currentGameFrameIndex();
        return api::Status::Ok;
    }

    api::Status copyWeaponPartPoses(
        api::weaponparts::PartPose* outPoses,
        const std::uint32_t capacity,
        std::uint32_t& outCount)
    {
        outCount = 0;
        if (!onAnimationOwnerThread()) {
            return api::Status::WrongThread;
        }
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        outCount = pi->copyProviderWeaponPartPoses(outPoses, capacity);
        return api::Status::Ok;
    }

    api::Status copyWeaponPartDriveResults(
        const std::uint64_t ownerToken,
        api::weaponparts::DriveResult* outResults,
        const std::uint32_t capacity,
        std::uint32_t& outCount)
    {
        outCount = 0;
        if (!onAnimationOwnerThread()) {
            return api::Status::WrongThread;
        }
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return api::Status::NotReady;
        }
        outCount = pi->copyProviderWeaponPartDriveResults(ownerToken, outResults, capacity);
        return api::Status::Ok;
    }

    api::Status setWeaponPartGrabPoints(api::OwnerToken owner,std::span<const api::weaponparts::GrabPoint> points) {
        for(const auto& p:points)if(!resolveSourceKey(p.weaponGenerationKey,p.sourceKey))return api::Status::GenerationMismatch;
        std::scoped_lock lock(s_weaponPartMutex);
        return s_weaponPartGrabPoints.replace(owner,points,[&](const auto& p){
            return std::any_of(s_weaponPartTargets.begin(),s_weaponPartTargets.end(),[&](const auto& s){
                return s.active&&s.ownerToken==owner&&s.target.groupId==p.groupId&&s.target.weaponGenerationKey==p.weaponGenerationKey&&
                    s.target.grabMode==api::weaponparts::GrabMode::AttachOnly;
            });
        });
    }
    api::Status clearWeaponPartGrabPoints(api::OwnerToken owner) {
        std::scoped_lock lock(s_weaponPartMutex);s_weaponPartGrabPoints.clear(owner);return api::Status::Ok;
    }
    bool hasGrabPoints(std::uint64_t generation) {
        std::scoped_lock lock(s_weaponPartMutex);return s_weaponPartGrabPoints.any(generation);
    }
    std::size_t copyWeaponPartPointSelection(std::span<weapon_part_runtime::Target> targets,std::span<weapon_part_points::OwnedPoint> points,std::size_t& pointCount) {
        std::scoped_lock lock(s_weaponPartMutex);pointCount=s_weaponPartGrabPoints.copy(points);std::size_t count=0;
        if(pointCount)for(const auto& slot:s_weaponPartTargets)if(slot.active){if(count==targets.size())break;targets[count++]=toRuntimeTarget(slot);}
        return count;
    }
