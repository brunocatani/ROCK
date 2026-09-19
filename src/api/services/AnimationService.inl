// Private Animation service operations. Included after shared owner/lifecycle helpers.

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetNativeAnimationAuthorityV1(
        std::uint64_t ownerToken,
        const RockProviderNativeAnimationAuthorityRequestV1* request)
    {
        if (!request || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->size != sizeof(RockProviderNativeAnimationAuthorityRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        constexpr auto implementedFlags = static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityFlagV1::ReloadPose);
        if (request->flags == 0 ||
            (request->flags & ~implementedFlags) != 0 ||
            request->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
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

        const auto frameIndex = currentProviderFrameIndex();
        const auto boundedLeaseFrames = provider_lease_policy::clampLeaseFrames(
            request->leaseFrames,
            ROCK_PROVIDER_MAX_NATIVE_ANIMATION_AUTHORITY_LEASE_FRAMES_V1);
        const auto expiresAtFrame = provider_lease_policy::exclusiveExpiryFrame(
            frameIndex,
            boundedLeaseFrames);

        std::scoped_lock lock(s_consumerMutex, s_nativeAnimationAuthorityMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::NativeAnimationAuthority);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        pruneExpiredNativeAnimationAuthorityLocked(frameIndex);
        NativeAnimationAuthoritySlot* available = nullptr;
        for (auto& slot : s_nativeAnimationAuthoritySlots) {
            if (slot.active && slot.ownerToken == ownerToken) {
                available = &slot;
                break;
            }
            if (!slot.active && !available) {
                available = &slot;
            }
        }
        if (!available) {
            return RockProviderResultV1::CapacityFull;
        }

        *available = NativeAnimationAuthoritySlot{
            .active = true,
            .ownerToken = ownerToken,
            .flags = request->flags,
            .expiresAtFrame = expiresAtFrame,
            .worldGeneration = request->worldGeneration,
            .skeletonGeneration = request->skeletonGeneration,
            .providerGeneration = request->providerGeneration,
        };
        publishNativeAnimationAuthorityAggregateLocked();
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearNativeAnimationAuthorityV1(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_nativeAnimationAuthorityMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::NativeAnimationAuthority);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        clearNativeAnimationAuthorityForOwnerLocked(ownerToken);
        return RockProviderResultV1::Ok;
    }

    bool ROCK_PROVIDER_CALL apiGetNativeAnimationAuthorityStateV1(
        RockProviderNativeAnimationAuthorityStateV1* outState)
    {
        provider_state_policy::clearQueryOutput(outState);
        if (!outState || outState->size != sizeof(RockProviderNativeAnimationAuthorityStateV1)) {
            return false;
        }

        RockProviderNativeAnimationRuntimePublicationV1 publication{};
        bool hasRuntimeProvider = false;
        {
            std::scoped_lock lock(s_nativeAnimationRuntimePublicationMutex);
            pruneExpiredNativeAnimationRuntimePublicationLocked(
                currentProviderFrameIndex());
            hasRuntimeProvider = s_hasNativeAnimationRuntimePublication;
            if (hasRuntimeProvider) {
                publication = s_nativeAnimationRuntimePublication;
            }
        }
        *outState = {};
        outState->size = sizeof(RockProviderNativeAnimationAuthorityStateV1);
        outState->version = ROCK_PROVIDER_API_VERSION;
        outState->activeFlags = s_nativeAnimationAuthorityFlags.load(std::memory_order_acquire);
        outState->statusFlags = hasRuntimeProvider ?
            publication.statusFlags |
                static_cast<std::uint32_t>(
                    RockProviderNativeAnimationAuthorityStatusFlagV1::RuntimeProviderAvailable) :
            0;
        outState->activeOwnerCount = s_nativeAnimationAuthorityOwnerCount.load(std::memory_order_acquire);
        outState->capturedTransformCount = hasRuntimeProvider ?
            publication.capturedTransformCount : 0;
        outState->captureSequence = hasRuntimeProvider ?
            publication.captureSequence : 0;
        return true;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetHandVisualAuthorityV1(
        const std::uint64_t ownerToken,
        const RockProviderHandVisualAuthorityRequestV1* request)
    {
        if (!request || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        if (request->size != sizeof(RockProviderHandVisualAuthorityRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->hand != RockProviderHand::Right &&
            request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::InvalidArgument;
        }

        constexpr std::uint32_t worldFlag =
            static_cast<std::uint32_t>(RockProviderHandVisualAuthorityFlagV1::WorldTransform);
        constexpr std::uint32_t fingerFlag =
            static_cast<std::uint32_t>(RockProviderHandVisualAuthorityFlagV1::FingerLocalTransforms);
        constexpr std::uint32_t implementedFlags = worldFlag | fingerFlag;
        if (request->flags == 0 || (request->flags & ~implementedFlags) != 0 ||
            request->priority < -10000 || request->priority > 10000 ||
            request->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & worldFlag) != 0 &&
            !finiteProviderTransform(request->worldTransform)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & fingerFlag) != 0) {
            if (request->fingerLocalTransformMask == 0 ||
                (request->fingerLocalTransformMask &
                    ~ROCK_PROVIDER_ALL_FINGER_LOCAL_TRANSFORMS_V1) != 0) {
                return RockProviderResultV1::InvalidArgument;
            }
            for (std::size_t index = 0; index < 15; ++index) {
                const auto bit = static_cast<std::uint16_t>(1u << index);
                if ((request->fingerLocalTransformMask & bit) != 0 &&
                    !finiteProviderTransform(request->fingerLocalTransforms[index])) {
                    return RockProviderResultV1::InvalidArgument;
                }
            }
        }

        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }

        {
            std::scoped_lock lock(s_consumerMutex);
            const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::HandVisualAuthority);
            if (ownerResult != RockProviderResultV1::Ok) {
                return ownerResult;
            }
        }
        if (!frik_visual_authority::isAvailable() ||
            !frik_visual_authority::isSkeletonReadyHint()) {
            return RockProviderResultV1::NotReady;
        }

        std::scoped_lock lock(s_handVisualAuthorityMutex);
        auto* slot = findHandVisualAuthoritySlotLocked(ownerToken, request->hand);
        if (!slot) {
            return RockProviderResultV1::CapacityFull;
        }
        if (slot->ownerToken != 0 && slot->publishedFlags != request->flags &&
            !clearHandVisualAuthoritySlotLocked(*slot, false)) {
            return RockProviderResultV1::TargetUnavailable;
        }
        if (slot->ownerToken == 0) {
            slot->ownerToken = ownerToken;
            slot->hand = request->hand;
            const int length = std::snprintf(
                slot->tag,
                sizeof(slot->tag),
                "ROCK_API_%016llX",
                static_cast<unsigned long long>(ownerToken));
            if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(slot->tag)) {
                *slot = {};
                return RockProviderResultV1::InvalidArgument;
            }
        }

        const auto hand = toVisualHand(request->hand);
        bool published = true;
        if ((request->flags & fingerFlag) != 0) {
            frik_visual_authority::FingerLocalTransformOverride fingerLocals{};
            fingerLocals.enabledMask = request->fingerLocalTransformMask;
            for (std::size_t index = 0; index < 15; ++index) {
                const auto bit = static_cast<std::uint16_t>(1u << index);
                if ((fingerLocals.enabledMask & bit) != 0) {
                    fingerLocals.localTransforms[index] =
                        toNiTransform(request->fingerLocalTransforms[index]);
                }
            }
            published = frik_visual_authority::setHandPoseCustom(
                            slot->tag,
                            hand,
                            frik_visual_authority::HandPoseData{},
                            request->priority) &&
                        frik_visual_authority::setHandPoseCustomLocalTransforms(
                            slot->tag,
                            hand,
                            &fingerLocals,
                            request->priority);
        }
        if (published && (request->flags & worldFlag) != 0) {
            published = frik_visual_authority::publishHandWorld(
                slot->tag,
                hand,
                toNiTransform(request->worldTransform),
                request->priority);
        }
        if (!published) {
            slot->publishedFlags = request->flags;
            (void)clearHandVisualAuthoritySlotLocked(*slot, true);
            return RockProviderResultV1::TargetUnavailable;
        }

        slot->publishedFlags = request->flags;
        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request->leaseFrames,
            ROCK_PROVIDER_MAX_HAND_VISUAL_AUTHORITY_LEASE_FRAMES_V1);
        slot->expiresAfterFrame = provider_lease_policy::exclusiveExpiryFrame(
            currentProviderFrameIndex(),
            leaseFrames);
        slot->worldGeneration = request->worldGeneration;
        slot->skeletonGeneration = request->skeletonGeneration;
        slot->providerGeneration = request->providerGeneration;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearHandVisualAuthorityV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand)
    {
        if (ownerToken == 0 ||
            (hand != RockProviderHand::None &&
                hand != RockProviderHand::Right &&
                hand != RockProviderHand::Left)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        {
            std::scoped_lock lock(s_consumerMutex);
            const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
                ownerToken,
                RockProviderConsumerCapabilityV1::HandVisualAuthority);
            if (ownerResult != RockProviderResultV1::Ok) {
                return ownerResult;
            }
        }

        return clearHandVisualAuthorityForOwner(ownerToken, hand, false) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::TargetUnavailable;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiPublishNativeAnimationRuntimeV1(
        const std::uint64_t ownerToken,
        const RockProviderNativeAnimationRuntimePublicationV1* publication)
    {
        if (!publication || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (publication->size !=
            sizeof(RockProviderNativeAnimationRuntimePublicationV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (publication->version == 0 ||
            publication->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        constexpr std::uint32_t implementedStatusFlags =
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::HookInstalled) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::RuntimeEnabled) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::CaptureValid) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::LocalReloadTestLeaseActive) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::HookInstallFailed) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::ThreadMismatch) |
            static_cast<std::uint32_t>(RockProviderNativeAnimationAuthorityStatusFlagV1::CaptureFault);
        if ((publication->statusFlags & ~implementedStatusFlags) != 0) {
            return RockProviderResultV1::InvalidArgument;
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

        std::scoped_lock lock(
            s_consumerMutex,
            s_nativeAnimationRuntimePublicationMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::NativeAnimationRuntimeProvider);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (s_nativeAnimationRuntimeProviderOwner != 0 &&
            s_nativeAnimationRuntimeProviderOwner != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }

        s_nativeAnimationRuntimeProviderOwner = ownerToken;
        s_nativeAnimationRuntimePublication = *publication;
        s_nativeAnimationRuntimePublication.size =
            sizeof(RockProviderNativeAnimationRuntimePublicationV1);
        s_nativeAnimationRuntimePublication.version = ROCK_PROVIDER_API_VERSION;
        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            publication->leaseFrames,
            ROCK_PROVIDER_MAX_NATIVE_ANIMATION_RUNTIME_LEASE_FRAMES_V1);
        s_nativeAnimationRuntimePublication.leaseFrames = leaseFrames;
        s_nativeAnimationRuntimeExpiresAfterFrame =
            provider_lease_policy::exclusiveExpiryFrame(
                currentProviderFrameIndex(),
                leaseFrames);
        s_hasNativeAnimationRuntimePublication = true;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearNativeAnimationRuntimeV1(
        const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(
            s_consumerMutex,
            s_nativeAnimationRuntimePublicationMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::NativeAnimationRuntimeProvider);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (s_nativeAnimationRuntimeProviderOwner != 0 &&
            s_nativeAnimationRuntimeProviderOwner != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }
        s_nativeAnimationRuntimeProviderOwner = 0;
        s_nativeAnimationRuntimePublication = {};
        s_nativeAnimationRuntimeExpiresAfterFrame = 0;
        s_hasNativeAnimationRuntimePublication = false;
        return RockProviderResultV1::Ok;
    }
