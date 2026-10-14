// Private WeaponParts service operations. Included after shared owner/lifecycle helpers.

    bool apiQueryWeaponContactAtPoint(
        const RockProviderWeaponContactQuery* query,
        RockProviderWeaponContactResult* outResult)
    {
        provider_state_policy::clearQueryOutput(outResult);
        if (!query || !outResult ||
            query->size != sizeof(RockProviderWeaponContactQuery) ||
            outResult->size != sizeof(RockProviderWeaponContactResult)) {
            return false;
        }

        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return false;
        }

        return pi->queryProviderWeaponContactAtPoint(*query, *outResult);
    }

    std::uint32_t apiCopyWeaponEvidenceDetailsV1(
        RockProviderWeaponEvidenceDetailV1* outDetails,
        std::uint32_t maxDetails)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderWeaponEvidenceDetailsV1(outDetails, maxDetails);
    }

    std::uint32_t apiCopyWeaponEvidenceDetailPointsV1(
        std::uint32_t bodyId,
        api::Point3* outPoints,
        std::uint32_t maxPoints)
    {
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isInitialized()) {
            return 0;
        }

        return pi->copyProviderWeaponEvidenceDetailPointsV1(bodyId, outPoints, maxPoints);
    }

    api::Status apiSetWeaponPartTargetsV1(
        std::uint64_t ownerToken,
        const RockProviderWeaponPartTargetV1* targets,
        std::uint32_t targetCount)
    {
        if (ownerToken == 0 || (targetCount > 0 && !targets)) {
            return api::Status::InvalidArgument;
        }
        if (targetCount > ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1) {
            return api::Status::CapacityFull;
        }

        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }

        for (std::uint32_t i = 0; i < targetCount; ++i) {
            const auto& target = targets[i];
            if (target.size != sizeof(RockProviderWeaponPartTargetV1)) {
                return api::Status::InvalidSize;
            }
            if (target.version == 0 || target.version > ROCK_PROVIDER_API_VERSION) {
                return api::Status::UnsupportedVersion;
            }
            if (!isValidWeaponPartGrabMode(target.grabMode) ||
                !hasValidWeaponPartMatcher(target.flags, target.bodyId, target.sourceRoot, target.sourceName) ||
                !hasValidWeaponPartTargetSemantics(target)) {
                return api::Status::InvalidArgument;
            }
        }

        if (targetCount > availableWeaponPartTargetSlotsForOwnerLocked(ownerToken)) {
            return api::Status::CapacityFull;
        }

        clearWeaponPartTargetsForOwnerLocked(ownerToken);
        for (std::uint32_t i = 0; i < targetCount; ++i) {
            bool stored = false;
            for (auto& slot : s_weaponPartTargets) {
                if (!slot.active) {
                    slot.active = true;
                    slot.ownerToken = ownerToken;
                    slot.target = targets[i];
                    slot.target.sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1] = '\0';
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

    api::Status apiClearWeaponPartTargetsV1(std::uint64_t ownerToken)
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

    api::Status apiSetWeaponPartDriveTargetsV1(
        std::uint64_t ownerToken,
        const RockProviderWeaponPartDriveTargetV1* targets,
        std::uint32_t targetCount)
    {
        if (ownerToken == 0 || (targetCount > 0 && !targets)) {
            return api::Status::InvalidArgument;
        }
        if (targetCount > ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1) {
            return api::Status::CapacityFull;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }

        for (std::uint32_t i = 0; i < targetCount; ++i) {
            const auto& target = targets[i];
            if (target.size != sizeof(RockProviderWeaponPartDriveTargetV1)) {
                return api::Status::InvalidSize;
            }
            if (target.version == 0 || target.version > ROCK_PROVIDER_API_VERSION) {
                return api::Status::UnsupportedVersion;
            }
            if (!isValidWeaponPartDriveSpace(target.driveSpace) ||
                target.leaseFrames == 0 ||
                !isFiniteProviderTransform(target.targetTransform) ||
                !hasConcreteWeaponPartDriveMatcher(target.flags, target.bodyId, target.sourceRoot, target.sourceName)) {
                return api::Status::InvalidArgument;
            }
        }

        pruneExpiredWeaponPartDrivesLocked(frameIndex);
        if (targetCount > availableWeaponPartDriveSlotsForOwnerLocked(ownerToken)) {
            return api::Status::CapacityFull;
        }
        clearWeaponPartDrivesForOwnerLocked(ownerToken);
        for (std::uint32_t i = 0; i < targetCount; ++i) {
            const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
                targets[i].leaseFrames,
                ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_LEASE_FRAMES_V1);
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
                    slot.target = targets[i];
                    slot.target.sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1] = '\0';
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

    api::Status apiClearWeaponPartDriveTargetsV1(std::uint64_t ownerToken)
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

    // Resolves the opaque source key while the frame owner holds the live
    // catalog; task readers copy the published value.
    api::weaponparts::GripState publicGripState(const RockProviderWeaponPartGripStateV1& in)
    {
        api::weaponparts::GripState out{};
        out.hand = static_cast<api::Hand>(in.hand);
        out.gripKind = static_cast<api::weaponparts::GripKind>(in.gripKind);
        out.active = in.active;
        out.attachOnly = in.attachOnly;
        out.gripSequence = in.gripSequence;
        out.weaponGenerationKey = in.weaponGenerationKey;
        out.weaponSlot = 0;
        out.bodyId = in.bodyId;
        out.partKind = in.partKind;
        out.reloadRole = in.reloadRole;
        out.supportRole = in.supportRole;
        out.socketRole = in.socketRole;
        out.actionRole = in.actionRole;
        out.omodFormId = in.omodFormId;
        out.sourceKey = sourceKey(in.weaponGenerationKey, in.sourceRoot);
        out.targetOwner = in.providerOwnerToken;
        out.targetGroupId = in.providerGroupId;
        out.targetGrabMode = static_cast<api::weaponparts::GrabMode>(in.providerGrabMode);
        out.attachPointFormId = in.attachPointFormId;
        out.classificationSource = in.classificationSource;
        out.authoredSupportGrip = in.authoredSupportGrip;
        out.hasHandPartLocal = in.hasHandPartLocal;
        out.handPartLocalSpace = static_cast<api::weaponparts::GripLocalSpace>(in.handPartLocalSpace);
        out.handPartLocal = in.handPartLocal;
        std::copy_n(in.sourceName, std::size(out.sourceName), out.sourceName);
        return out;
    }

    bool apiGetWeaponPartGripStateV1(api::Hand hand, api::weaponparts::GripState* outState)
    {
        provider_state_policy::clearQueryOutput(outState);
        if (!outState || outState->size != sizeof(api::weaponparts::GripState)) {
            return false;
        }
        if (hand != api::Hand::Right && hand != api::Hand::Left) {
            return false;
        }

        std::scoped_lock lock(s_snapshotMutex);
        if (!apiIsProviderReady() || !s_hasSnapshot || s_lastSnapshot.providerReady == 0) {
            return false;
        }
        *outState = s_lastPartGripStates[hand == api::Hand::Left ? 1u : 0u];
        return true;
    }

    api::Status apiQueryWeaponPartTargetResolutionV1(
        const std::uint64_t ownerToken,
        const RockProviderWeaponPartResolutionQueryV1* query,
        RockProviderWeaponPartResolutionResultV1* outResolution)
    {
        provider_state_policy::clearQueryOutput(outResolution);
        if (ownerToken == 0 || !query || !outResolution) {
            return api::Status::InvalidArgument;
        }
        if (query->size < sizeof(RockProviderWeaponPartResolutionQueryV1) ||
            outResolution->size < sizeof(RockProviderWeaponPartResolutionResultV1)) {
            return api::Status::InvalidSize;
        }
        const auto ownerResult = validateRegisteredOwner(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }

        RockProviderWeaponPartTargetQueryV1 internal{};
        internal.weaponGenerationKey = query->weaponGenerationKey;
        internal.bodyId = query->bodyId;
        internal.partKind = query->partKind;
        internal.reloadRole = query->reloadRole;
        internal.supportRole = query->supportRole;
        internal.socketRole = query->socketRole;
        internal.actionRole = query->actionRole;
        internal.sourceRoot = query->sourceRoot;
        std::memcpy(internal.sourceName, query->sourceName,
            sizeof(internal.sourceName));
        internal.sourceName[sizeof(internal.sourceName) - 1] = '\0';
        RockProviderWeaponPartTargetResolutionV1 resolution{};
        (void)resolveWeaponPartTargetV1(internal, resolution);

        *outResolution = {};
        outResolution->whitelistActive = resolution.whitelistActive;
        outResolution->matched = resolution.matched;
        outResolution->grabMode = resolution.grabMode;
        outResolution->groupId = resolution.groupId;
        outResolution->priority = resolution.priority;
        outResolution->winningOwnerToken = resolution.ownerToken;
        outResolution->weaponGenerationKey = query->weaponGenerationKey;
        outResolution->frameIndex = currentGameFrameIndex();
        return api::Status::Ok;
    }

    api::Status apiCopyWeaponPartPoseSnapshotV1(
        const std::uint64_t ownerToken,
        RockProviderWeaponPartPoseV1* outParts,
        const std::uint32_t maxParts,
        std::uint32_t* outPartCount)
    {
        if (outPartCount) *outPartCount = 0;
        if (ownerToken == 0 || !outPartCount ||
            (maxParts != 0 && !outParts)) {
            return api::Status::InvalidArgument;
        }
        const auto ownerResult = validateRegisteredOwner(ownerToken);
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
        *outPartCount = pi->copyProviderWeaponPartPosesV1(
            outParts,
            maxParts);
        return api::Status::Ok;
    }

    api::Status apiCopyWeaponPartDriveApplicationResultsV1(
        const std::uint64_t ownerToken,
        RockProviderWeaponPartDriveApplicationResultV1* outResults,
        const std::uint32_t maxResults,
        std::uint32_t* outResultCount)
    {
        if (outResultCount) *outResultCount = 0;
        if (ownerToken == 0 || !outResultCount ||
            (maxResults != 0 && !outResults)) {
            return api::Status::InvalidArgument;
        }
        const auto ownerResult = validateRegisteredOwner(ownerToken);
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
        *outResultCount = pi->copyProviderWeaponPartDriveResultsV1(
            ownerToken,
            outResults,
            maxResults);
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
