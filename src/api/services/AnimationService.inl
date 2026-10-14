// Private Animation service operations. Included after shared owner/lifecycle helpers.

    /*
     * Selective Bethesda animation authority. ROCK preserves the authored
     * relationship between both arms, hands and weapon, then rigidly anchors
     * both skeleton trees to the visible first-person weapon/controller world
     * frame while retaining the weapon's authored motion since lease
     * acquisition. Arms covers only the two collarbone-to-hand chains; Hands
     * adds the hand roots and finger/thumb descendants; Weapon covers only
     * Weapon and WeaponLeft. The character root, COM, torso, head and legs
     * remain owned by the live VR body.
     */
    api::Status setNativeAnimationAuthority(
        std::uint64_t ownerToken,
        const ::rock::api::animation::NativeAnimationAuthorityRequest& request)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        constexpr auto implementedFlags = static_cast<std::uint32_t>(::rock::api::animation::NativeAnimationAuthorityFlag::ReloadPose);
        if (request.flags == 0 ||
            (request.flags & ~implementedFlags) != 0 ||
            request.leaseFrames == 0) {
            return api::Status::InvalidArgument;
        }

        const auto generationResult = validateGenerationGuards(
            request.worldGeneration,
            request.skeletonGeneration,
            request.providerGeneration);
        if (generationResult != api::Status::Ok) {
            return generationResult;
        }
        if (!apiIsProviderReady()) {
            return api::Status::NotReady;
        }

        const auto frameIndex = currentProviderFrameIndex();
        const auto boundedLeaseFrames = provider_lease_policy::clampLeaseFrames(
            request.leaseFrames,
            api::animation::kMaxNativeAuthorityLeaseFrames);
        const auto expiresAtFrame = provider_lease_policy::exclusiveExpiryFrame(
            frameIndex,
            boundedLeaseFrames);

        std::scoped_lock lock(s_consumerMutex, s_nativeAnimationAuthorityMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
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
            return api::Status::CapacityFull;
        }

        *available = NativeAnimationAuthoritySlot{
            .active = true,
            .ownerToken = ownerToken,
            .flags = request.flags,
            .expiresAtFrame = expiresAtFrame,
            .worldGeneration = request.worldGeneration,
            .skeletonGeneration = request.skeletonGeneration,
            .providerGeneration = request.providerGeneration,
        };
        publishNativeAnimationAuthorityAggregateLocked();
        return api::Status::Ok;
    }

    api::Status clearNativeAnimationAuthority(std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_nativeAnimationAuthorityMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        clearNativeAnimationAuthorityForOwnerLocked(ownerToken);
        return api::Status::Ok;
    }

    bool getNativeAnimationAuthorityState(
        ::rock::api::animation::NativeAnimationAuthorityState& outState)
    {
        outState = {};

        ::rock::api::animation::NativeAnimationRuntimePublication publication{};
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
        outState.activeFlags = s_nativeAnimationAuthorityFlags.load(std::memory_order_acquire);
        // Only slot 0's presentation can be under authority until the runtime
        // keeps per-slot weapon state.
        outState.weaponSlotMask =
            (outState.activeFlags & static_cast<std::uint32_t>(::rock::api::animation::NativeAnimationAuthorityFlag::Weapon)) ? 1u : 0u;
        outState.statusFlags = hasRuntimeProvider ?
            publication.statusFlags |
                static_cast<std::uint32_t>(
                    ::rock::api::animation::NativeAnimationAuthorityStatusFlag::RuntimeProviderAvailable) :
            0;
        outState.activeOwnerCount = s_nativeAnimationAuthorityOwnerCount.load(std::memory_order_acquire);
        outState.capturedTransformCount = hasRuntimeProvider ?
            publication.capturedTransformCount : 0;
        outState.captureSequence = hasRuntimeProvider ?
            publication.captureSequence : 0;
        return true;
    }

    api::Status setHandVisualAuthority(
        const std::uint64_t ownerToken,
        const ::rock::api::animation::HandVisualAuthorityRequest& request)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return api::Status::WrongThread;
        }
        if (s_presentedReadbackPhase) {
            return api::Status::NotReady;
        }
        const auto requestHand = static_cast<api::Hand>(request.hand);
        if (requestHand != api::Hand::Right &&
            requestHand != api::Hand::Left) {
            return api::Status::InvalidArgument;
        }

        constexpr std::uint32_t worldFlag =
            static_cast<std::uint32_t>(::rock::api::animation::HandVisualAuthorityFlag::WorldTransform);
        constexpr std::uint32_t fingerFlag =
            static_cast<std::uint32_t>(::rock::api::animation::HandVisualAuthorityFlag::FingerLocalTransforms);
        constexpr std::uint32_t implementedFlags = worldFlag | fingerFlag;
        if (request.flags == 0 || (request.flags & ~implementedFlags) != 0 ||
            request.priority < -10000 || request.priority > 10000 ||
            request.leaseFrames == 0) {
            return api::Status::InvalidArgument;
        }
        if ((request.flags & worldFlag) != 0 &&
            !finiteProviderTransform(request.worldTransform)) {
            return api::Status::InvalidArgument;
        }
        if ((request.flags & fingerFlag) != 0) {
            if (request.fingerLocalTransformMask == 0 ||
                (request.fingerLocalTransformMask &
                    ~kAllFingerLocalTransforms) != 0) {
                return api::Status::InvalidArgument;
            }
            for (std::size_t index = 0; index < 15; ++index) {
                const auto bit = 1u << index;
                if ((request.fingerLocalTransformMask & bit) != 0 &&
                    !finiteProviderTransform(request.fingerLocalTransforms[index])) {
                    return api::Status::InvalidArgument;
                }
            }
        }

        const auto generationResult = validateGenerationGuards(
            request.worldGeneration,
            request.skeletonGeneration,
            request.providerGeneration);
        if (generationResult != api::Status::Ok) {
            return generationResult;
        }

        {
            std::scoped_lock lock(s_consumerMutex);
            const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
            if (ownerResult != api::Status::Ok) {
                return ownerResult;
            }
        }
        if (!frik_visual_authority::isAvailable() ||
            !frik_visual_authority::isSkeletonReadyHint()) {
            return api::Status::NotReady;
        }

        std::scoped_lock lock(s_handVisualAuthorityMutex);
        auto* slot = findHandVisualAuthoritySlotLocked(ownerToken, requestHand);
        if (!slot) {
            return api::Status::CapacityFull;
        }
        if (slot->ownerToken != 0 && slot->publishedFlags != request.flags &&
            !clearHandVisualAuthoritySlotLocked(*slot, false)) {
            return api::Status::TargetUnavailable;
        }
        if (slot->ownerToken == 0) {
            slot->ownerToken = ownerToken;
            slot->hand = requestHand;
            const int length = std::snprintf(
                slot->tag,
                sizeof(slot->tag),
                "ROCK_API_%016llX",
                static_cast<unsigned long long>(ownerToken));
            if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(slot->tag)) {
                *slot = {};
                return api::Status::InvalidArgument;
            }
        }

        const auto hand = toVisualHand(requestHand);
        bool published = true;
        if ((request.flags & fingerFlag) != 0) {
            frik_visual_authority::FingerLocalTransformOverride fingerLocals{};
            fingerLocals.enabledMask = request.fingerLocalTransformMask;
            for (std::size_t index = 0; index < 15; ++index) {
                const auto bit = 1u << index;
                if ((fingerLocals.enabledMask & bit) != 0) {
                    fingerLocals.localTransforms[index] =
                        toNiTransform(request.fingerLocalTransforms[index]);
                }
            }
            published = frik_visual_authority::setHandPoseCustom(
                            slot->tag,
                            hand,
                            frik_visual_authority::HandPoseData{},
                            request.priority) &&
                        frik_visual_authority::setHandPoseCustomLocalTransforms(
                            slot->tag,
                            hand,
                            &fingerLocals,
                            request.priority);
        }
        if (published && (request.flags & worldFlag) != 0) {
            published = frik_visual_authority::publishHandWorld(
                slot->tag,
                hand,
                toNiTransform(request.worldTransform),
                request.priority);
        }
        if (!published) {
            slot->publishedFlags = request.flags;
            (void)clearHandVisualAuthoritySlotLocked(*slot, true);
            return api::Status::TargetUnavailable;
        }

        slot->publishedFlags = request.flags;
        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request.leaseFrames,
            api::animation::kMaxHandVisualLeaseFrames);
        slot->expiresAfterFrame = provider_lease_policy::exclusiveExpiryFrame(
            currentProviderFrameIndex(),
            leaseFrames);
        slot->worldGeneration = request.worldGeneration;
        slot->skeletonGeneration = request.skeletonGeneration;
        slot->providerGeneration = request.providerGeneration;
        return api::Status::Ok;
    }

    api::Status clearHandVisualAuthority(
        const std::uint64_t ownerToken,
        const api::Hand hand)
    {
        if (ownerToken == 0 ||
            (hand != api::Hand::None &&
                hand != api::Hand::Right &&
                hand != api::Hand::Left)) {
            return api::Status::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return api::Status::WrongThread;
        }
        {
            std::scoped_lock lock(s_consumerMutex);
            const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
            if (ownerResult != api::Status::Ok) {
                return ownerResult;
            }
        }

        return clearHandVisualAuthorityForOwner(ownerToken, hand, false) ?
            api::Status::Ok :
            api::Status::TargetUnavailable;
    }

    api::Status publishNativeAnimationRuntime(
        const std::uint64_t ownerToken,
        const ::rock::api::animation::NativeAnimationRuntimePublication& publication)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        constexpr std::uint32_t implementedStatusFlags =
            static_cast<std::uint32_t>(::rock::api::animation::NativeAnimationAuthorityStatusFlag::HookInstalled) |
            static_cast<std::uint32_t>(::rock::api::animation::NativeAnimationAuthorityStatusFlag::RuntimeEnabled) |
            static_cast<std::uint32_t>(::rock::api::animation::NativeAnimationAuthorityStatusFlag::CaptureValid) |
            static_cast<std::uint32_t>(::rock::api::animation::NativeAnimationAuthorityStatusFlag::LocalReloadTestLeaseActive) |
            static_cast<std::uint32_t>(::rock::api::animation::NativeAnimationAuthorityStatusFlag::HookInstallFailed) |
            static_cast<std::uint32_t>(::rock::api::animation::NativeAnimationAuthorityStatusFlag::ThreadMismatch) |
            static_cast<std::uint32_t>(::rock::api::animation::NativeAnimationAuthorityStatusFlag::CaptureFault);
        if ((publication.statusFlags & ~implementedStatusFlags) != 0) {
            return api::Status::InvalidArgument;
        }
        if (publication.leaseFrames == 0) {
            return api::Status::InvalidArgument;
        }
        const auto generationResult = validateGenerationGuards(
            publication.worldGeneration,
            publication.skeletonGeneration,
            publication.providerGeneration);
        if (generationResult != api::Status::Ok) {
            return generationResult;
        }

        std::scoped_lock lock(
            s_consumerMutex,
            s_nativeAnimationRuntimePublicationMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        if (s_nativeAnimationRuntimeProviderOwner != 0 &&
            s_nativeAnimationRuntimeProviderOwner != ownerToken) {
            return api::Status::OwnerConflict;
        }

        s_nativeAnimationRuntimeProviderOwner = ownerToken;
        s_nativeAnimationRuntimePublication = publication;
        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            publication.leaseFrames,
            api::animation::kMaxRuntimePublicationLeaseFrames);
        s_nativeAnimationRuntimePublication.leaseFrames = leaseFrames;
        s_nativeAnimationRuntimeExpiresAfterFrame =
            provider_lease_policy::exclusiveExpiryFrame(
                currentProviderFrameIndex(),
                leaseFrames);
        s_hasNativeAnimationRuntimePublication = true;
        return api::Status::Ok;
    }

    api::Status clearNativeAnimationRuntime(
        const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        std::scoped_lock lock(
            s_consumerMutex,
            s_nativeAnimationRuntimePublicationMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        if (s_nativeAnimationRuntimeProviderOwner != 0 &&
            s_nativeAnimationRuntimeProviderOwner != ownerToken) {
            return api::Status::OwnerConflict;
        }
        s_nativeAnimationRuntimeProviderOwner = 0;
        s_nativeAnimationRuntimePublication = {};
        s_nativeAnimationRuntimeExpiresAfterFrame = 0;
        s_hasNativeAnimationRuntimePublication = false;
        return api::Status::Ok;
    }
