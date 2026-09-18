// Private Input service operations. Included after shared owner/lifecycle helpers.

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetHandInputSuppressionV1(
        std::uint64_t ownerToken,
        const RockProviderHandInputSuppressionRequestV1* request)
    {
        if (!request || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->size != sizeof(RockProviderHandInputSuppressionRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if (request->flags == 0 || (request->flags & ~kImplementedHandInputSuppressionFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->leaseFrames == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto leftChord = (static_cast<std::uint64_t>(request->chordButtonsHigh[0]) << 32) | request->chordButtonsLow[0];
        const auto rightChord = (static_cast<std::uint64_t>(request->chordButtonsHigh[1]) << 32) | request->chordButtonsLow[1];
        constexpr auto allowedButtons = (1ull << 1) | (1ull << 2) | (1ull << 7) | (1ull << 32) | (1ull << 33);
        if (hasHandInputSuppressionFlagV1(request->flags, RockProviderHandInputSuppressionFlagV1::ReserveButtonChord) &&
            (((leftChord | rightChord) & ~allowedButtons) != 0 ||
             !(request->hand == RockProviderHand::Left ? leftChord : rightChord))) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto generationResult = validateGenerationGuards(
            request->worldGeneration,
            request->skeletonGeneration,
            request->providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return generationResult;
        }

        const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
            request->leaseFrames,
            ROCK_PROVIDER_MAX_HAND_INPUT_SUPPRESSION_LEASE_FRAMES_V1);
        const auto frameIndex = currentProviderFrameIndex();
        const auto expiresAfterFrame =
            provider_lease_policy::exclusiveExpiryFrame(frameIndex, leaseFrames);

        std::scoped_lock lock(s_consumerMutex, s_handInputSuppressionMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::HandInputSuppression);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        pruneExpiredHandInputSuppressionsLocked(frameIndex);
        for (auto& slot : s_handInputSuppressions) {
            if (slot.active && slot.ownerToken == ownerToken && slot.hand == request->hand) {
                slot.flags = request->flags;
                slot.leftChord = leftChord; slot.rightChord = rightChord;
                slot.expiresAfterFrame = expiresAfterFrame;
                slot.worldGeneration = request->worldGeneration;
                slot.skeletonGeneration = request->skeletonGeneration;
                slot.providerGeneration = request->providerGeneration;
                slot.lastInvalidationReason =
                    RockProviderSuppressionInvalidationReasonV1::None;
                slot.lastInvalidatedFrame = 0;
                return RockProviderResultV1::Ok;
            }
        }

        for (auto& slot : s_handInputSuppressions) {
            if (!slot.active) {
                slot = HandInputSuppressionSlot{
                    .active = true,
                    .ownerToken = ownerToken,
                    .hand = request->hand,
                    .flags = request->flags,
                    .leftChord = leftChord, .rightChord = rightChord,
                    .expiresAfterFrame = expiresAfterFrame,
                    .worldGeneration = request->worldGeneration,
                    .skeletonGeneration = request->skeletonGeneration,
                    .providerGeneration = request->providerGeneration,
                    .lastInvalidationReason =
                        RockProviderSuppressionInvalidationReasonV1::None,
                    .lastInvalidatedFrame = 0,
                };
                return RockProviderResultV1::Ok;
            }
        }

        return RockProviderResultV1::CapacityFull;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearHandInputSuppressionV1(
        std::uint64_t ownerToken,
        RockProviderHand hand)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (hand != RockProviderHand::None && hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }

        std::scoped_lock lock(s_consumerMutex, s_handInputSuppressionMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::HandInputSuppression);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        clearHandInputSuppressionsForOwnerLocked(ownerToken, hand);
        return RockProviderResultV1::Ok;
    }

    bool ROCK_PROVIDER_CALL apiGetRawWandButtonStateV1(RockProviderHand hand, std::uint32_t buttonId, RockProviderRawWandButtonStateV1* outState)
    {
        provider_state_policy::clearQueryOutput(outState);
        if (!outState || outState->size != sizeof(RockProviderRawWandButtonStateV1)) {
            return false;
        }
        if (hand != RockProviderHand::Left && hand != RockProviderHand::Right) {
            return false;
        }
        if (!rock::input_remap_policy::isValidButtonId(static_cast<int>(buttonId))) {
            return false;
        }

        // Level state only by design: ROCK consumes its press/release edge queues internally each frame, so exposing them would race consumers.
        const auto raw = rock::input_remap_runtime::peekRawButtonState(hand == RockProviderHand::Left, static_cast<int>(buttonId));
        *outState = {};
        outState->size = sizeof(RockProviderRawWandButtonStateV1);
        outState->version = ROCK_PROVIDER_API_VERSION;
        outState->available = raw.available ? 1u : 0u;
        outState->held = raw.held ? 1u : 0u;
        outState->sampleSequence = raw.sampleSequence;
        outState->sampleAgeMilliseconds = raw.sampleAgeMilliseconds;
        outState->availabilityReason =
            static_cast<RockProviderInputAvailabilityReasonV1>(
                raw.availabilityReason);
        return true;
    }

    bool ROCK_PROVIDER_CALL apiIsNativePipboyInputSuppressedV1()
    {
        return rock::input_remap_runtime::isNativePipboyInputSuppressionActive();
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetHandInputSuppressionStateV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        RockProviderHandInputSuppressionStateV1* outState)
    {
        provider_state_policy::clearQueryOutput(outState);
        if (ownerToken == 0 || !outState ||
            (hand != RockProviderHand::Right && hand != RockProviderHand::Left)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (outState->size < sizeof(RockProviderHandInputSuppressionStateV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_handInputSuppressionMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::InputObservability);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        pruneExpiredHandInputSuppressionsLocked(frameIndex);

        *outState = {};
        outState->frameIndex = frameIndex;
        outState->hand = hand;
        if (rock::input_remap_runtime::ownsBareFistInput()) {
            outState->effectiveFlags = static_cast<std::uint32_t>(
                RockProviderHandInputSuppressionFlagV1::SuppressConfigModeChord);
        }
        for (const auto& slot : s_handInputSuppressions) {
            if (slot.hand != hand) {
                continue;
            }
            if (slot.active) {
                outState->effectiveFlags |= effectiveHandInputSuppressionFlags(hand, slot.flags, slot.leftChord, slot.rightChord);
            }
            if (slot.ownerToken != ownerToken) {
                continue;
            }
            if (slot.active) {
                outState->callerFlags = slot.flags;
                outState->callerLeaseActive = 1;
                outState->callerExpiresAfterFrame = slot.expiresAfterFrame;
                outState->callerRemainingFrames =
                    provider_lease_policy::remainingFrames(
                        frameIndex,
                        slot.expiresAfterFrame);
            } else if (outState->lastInvalidationReason ==
                       RockProviderSuppressionInvalidationReasonV1::None) {
                outState->lastInvalidationReason =
                    slot.lastInvalidationReason;
            }
        }
        {
            std::scoped_lock snapshotLock(s_snapshotMutex);
            if (s_hasSnapshot) {
                outState->worldGeneration = s_lastSnapshot.worldGeneration;
                outState->skeletonGeneration = s_lastSnapshot.skeletonGeneration;
                outState->providerGeneration = s_lastSnapshot.providerGeneration;
            }
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetLogicalInputActionStateV1(
        const std::uint64_t ownerToken,
        const RockProviderLogicalInputActionV1 action,
        RockProviderLogicalInputActionStateV1* outState)
    {
        provider_state_policy::clearQueryOutput(outState);
        if (ownerToken == 0 || !outState) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (outState->size < sizeof(RockProviderLogicalInputActionStateV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (outState->version == 0 ||
            outState->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (action != RockProviderLogicalInputActionV1::Jump) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::InputObservability);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        RockProviderFrameSnapshot snapshot{};
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot) {
                return RockProviderResultV1::NotReady;
            }
            snapshot = s_lastSnapshot;
        }

        const auto input = rock::input_remap_runtime::readLogicalJumpState();
        *outState = {};
        outState->action = action;
        outState->available = input.available ? 1u : 0u;
        outState->held = input.held ? 1u : 0u;
        outState->availabilityReason =
            static_cast<RockProviderInputAvailabilityReasonV1>(
                input.availabilityReason);
        outState->sampleSequence = input.sampleSequence;
        outState->pressSequence = input.pressSequence;
        outState->sampleAgeMilliseconds = input.sampleAgeMilliseconds;
        outState->frameIndex = snapshot.frameIndex;
        outState->worldGeneration = snapshot.worldGeneration;
        outState->skeletonGeneration = snapshot.skeletonGeneration;
        outState->providerGeneration = snapshot.providerGeneration;
        return RockProviderResultV1::Ok;
    }

    bool ROCK_PROVIDER_CALL apiGetRawWandThumbstickV1(RockProviderHand hand, float* outX, float* outY)
    {
        if (outX) *outX = 0.0f;
        if (outY) *outY = 0.0f;
        if (!outX || !outY || (hand != RockProviderHand::Left && hand != RockProviderHand::Right)) return false;
        return rock::input_remap_runtime::peekRawThumbstick(hand == RockProviderHand::Left, *outX, *outY);
    }

    std::uint32_t ROCK_PROVIDER_CALL apiGetNativeInputContextV1()
    {
        try {
            using Flag = RockProviderNativeInputContextFlagV1;
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
