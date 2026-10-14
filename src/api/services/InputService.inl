// Private Input service operations. Included after shared owner/lifecycle helpers.

    api::Status getPlacementClickState(api::input::PlacementClickState& output)
    {
        const auto access=s_physicsInteraction.borrow();
        const auto* pi=access.get();
        if (!pi || !pi->isProviderReady()) return api::Status::NotReady;
        pi->queryPlacementClickState(output);
        return api::Status::Ok;
    }

    api::Status setHandInputSuppression(
        std::uint64_t ownerToken,
        const ::rock::api::input::HandInputSuppressionRequest& request)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        const auto requestHand = static_cast<api::Hand>(request.hand);
        if (requestHand != api::Hand::Right && requestHand != api::Hand::Left) {
            return api::Status::HandUnavailable;
        }
        if (request.flags == 0 || (request.flags & ~kImplementedHandInputSuppressionFlags) != 0) {
            return api::Status::InvalidArgument;
        }
        if (request.leaseFrames == 0) {
            return api::Status::InvalidArgument;
        }

        const auto leftChord = request.chordButtons[0];
        const auto rightChord = request.chordButtons[1];
        constexpr auto allowedButtons = (1ull << 1) | (1ull << 2) | (1ull << 7) | (1ull << 32) | (1ull << 33);
        if (hasHandInputSuppressionFlag(request.flags, ::rock::api::input::HandInputSuppressionFlag::ReserveButtonChord) &&
            (((leftChord | rightChord) & ~allowedButtons) != 0 ||
             !(requestHand == api::Hand::Left ? leftChord : rightChord))) {
            return api::Status::InvalidArgument;
        }

        const auto generationResult = validateGenerationGuards(
            request.worldGeneration,
            request.skeletonGeneration,
            request.providerGeneration);
        if (generationResult != api::Status::Ok) {
            return generationResult;
        }

        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request.leaseFrames,
            api::input::kMaxSuppressionLeaseFrames);
        const auto frameIndex = currentProviderFrameIndex();
        const auto expiresAfterFrame =
            provider_lease_policy::exclusiveExpiryFrame(frameIndex, leaseFrames);

        std::scoped_lock lock(s_consumerMutex, s_handInputSuppressionMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }

        pruneExpiredHandInputSuppressionsLocked(frameIndex);
        for (auto& slot : s_handInputSuppressions) {
            if (slot.active && slot.ownerToken == ownerToken && slot.hand == requestHand) {
                slot.flags = request.flags;
                slot.leftChord = leftChord; slot.rightChord = rightChord;
                slot.expiresAfterFrame = expiresAfterFrame;
                slot.worldGeneration = request.worldGeneration;
                slot.skeletonGeneration = request.skeletonGeneration;
                slot.providerGeneration = request.providerGeneration;
                slot.lastInvalidationReason =
                    ::rock::api::input::SuppressionInvalidationReason::None;
                slot.lastInvalidatedFrame = 0;
                return api::Status::Ok;
            }
        }

        for (auto& slot : s_handInputSuppressions) {
            if (!slot.active) {
                slot = HandInputSuppressionSlot{
                    .active = true,
                    .ownerToken = ownerToken,
                    .hand = requestHand,
                    .flags = request.flags,
                    .leftChord = leftChord, .rightChord = rightChord,
                    .expiresAfterFrame = expiresAfterFrame,
                    .worldGeneration = request.worldGeneration,
                    .skeletonGeneration = request.skeletonGeneration,
                    .providerGeneration = request.providerGeneration,
                    .lastInvalidationReason =
                        ::rock::api::input::SuppressionInvalidationReason::None,
                    .lastInvalidatedFrame = 0,
                };
                return api::Status::Ok;
            }
        }

        return api::Status::CapacityFull;
    }

    api::Status clearHandInputSuppression(
        std::uint64_t ownerToken,
        api::Hand hand)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        if (hand != api::Hand::None && hand != api::Hand::Right && hand != api::Hand::Left) {
            return api::Status::HandUnavailable;
        }

        std::scoped_lock lock(s_consumerMutex, s_handInputSuppressionMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }

        clearHandInputSuppressionsForOwnerLocked(ownerToken, hand);
        return api::Status::Ok;
    }

    bool getRawWandButtonState(api::Hand hand, std::uint32_t buttonId, ::rock::api::input::RawWandButtonState& outState)
    {
        outState = {};
        if (hand != api::Hand::Left && hand != api::Hand::Right) {
            return false;
        }
        if (!rock::input_remap_policy::isValidButtonId(static_cast<int>(buttonId))) {
            return false;
        }

        // Level state only by design: ROCK consumes its press/release edge queues internally each frame, so exposing them would race consumers.
        const auto raw = rock::input_remap_runtime::peekRawButtonState(hand == api::Hand::Left, static_cast<int>(buttonId));
        outState.available = raw.available ? 1u : 0u;
        outState.held = raw.held ? 1u : 0u;
        outState.sampleSequence = raw.sampleSequence;
        outState.sampleAgeMilliseconds = raw.sampleAgeMilliseconds;
        outState.availabilityReason =
            static_cast<::rock::api::input::InputAvailabilityReason>(
                raw.availabilityReason);
        return true;
    }

    bool isNativePipboyInputSuppressed()
    {
        return rock::input_remap_runtime::isNativePipboyInputSuppressionActive();
    }

    api::Status getHandInputSuppressionState(
        const std::uint64_t ownerToken,
        const api::Hand hand,
        ::rock::api::input::HandInputSuppressionState& outState)
    {
        outState = {};
        if (ownerToken == 0 ||
            (hand != api::Hand::Right && hand != api::Hand::Left)) {
            return api::Status::InvalidArgument;
        }
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_handInputSuppressionMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        pruneExpiredHandInputSuppressionsLocked(frameIndex);

        outState.leaseFrameIndex = frameIndex;
        outState.hand = static_cast<api::Hand>(hand);
        if (rock::input_remap_runtime::ownsBareFistInput()) {
            outState.effectiveFlags = static_cast<std::uint32_t>(
                ::rock::api::input::HandInputSuppressionFlag::SuppressConfigModeChord);
        }
        for (const auto& slot : s_handInputSuppressions) {
            if (slot.hand != hand) {
                continue;
            }
            if (slot.active) {
                outState.effectiveFlags |= effectiveHandInputSuppressionFlags(hand, slot.flags, slot.leftChord, slot.rightChord);
            }
            if (slot.ownerToken != ownerToken) {
                continue;
            }
            if (slot.active) {
                outState.callerFlags = slot.flags;
                outState.callerLeaseActive = 1;
                outState.callerExpiresAfterFrame = slot.expiresAfterFrame;
                outState.callerRemainingFrames =
                    provider_lease_policy::remainingFrames(
                        frameIndex,
                        slot.expiresAfterFrame);
            } else if (outState.lastInvalidationReason ==
                       ::rock::api::input::SuppressionInvalidationReason::None) {
                outState.lastInvalidationReason =
                    slot.lastInvalidationReason;
            }
        }
        {
            std::scoped_lock snapshotLock(s_snapshotMutex);
            if (s_hasSnapshot) {
                outState.sample = s_lastSnapshot.sample();
            }
        }
        return api::Status::Ok;
    }

    api::Status getLogicalInputActionState(
        const std::uint64_t ownerToken,
        const ::rock::api::input::LogicalInputAction action,
        ::rock::api::input::LogicalInputActionState& outState)
    {
        outState = {};
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        if (action != ::rock::api::input::LogicalInputAction::Jump) {
            return api::Status::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return api::Status::WrongThread;
        }
        const auto ownerResult = validateRegisteredOwner(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }

        Publication snapshot{};
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot) {
                return api::Status::NotReady;
            }
            snapshot = s_lastSnapshot;
        }

        const auto input = rock::input_remap_runtime::readLogicalJumpState();
        outState.action = action;
        outState.available = input.available ? 1u : 0u;
        outState.held = input.held ? 1u : 0u;
        outState.availabilityReason =
            static_cast<::rock::api::input::InputAvailabilityReason>(
                input.availabilityReason);
        outState.sampleSequence = input.sampleSequence;
        outState.pressSequence = input.pressSequence;
        outState.sampleAgeMilliseconds = input.sampleAgeMilliseconds;
        outState.sample = snapshot.sample();
        return api::Status::Ok;
    }

    bool getRawWandThumbstick(api::Hand hand, float& outX, float& outY)
    {
        outX = 0.0f;
        outY = 0.0f;
        if (hand != api::Hand::Left && hand != api::Hand::Right) return false;
        return rock::input_remap_runtime::peekRawThumbstick(hand == api::Hand::Left, outX, outY);
    }

    std::uint32_t getNativeInputContext()
    {
        try {
            using Flag = ::rock::api::input::NativeInputContextFlag;
            if (!rock::input_remap_runtime::isInputRemapHookInstalled()) return 0;
            auto flags = static_cast<std::uint32_t>(Flag::Available);
            if (rock::input_remap_runtime::isMenuInputActive())
                return flags | static_cast<std::uint32_t>(Flag::MenuActive);
            if (rock::input_remap_runtime::hasNativeActivationTarget(true))
                flags |= static_cast<std::uint32_t>(Flag::PrimaryActivationTarget);
            return flags;
        } catch (...) {
            static std::atomic_bool logged{ false };
            if (!logged.exchange(true)) logger::error("Native input context query failed; consumer input denied");
            return 0; // Unavailable: consumers must not claim input on a failed query.
        }
    }
