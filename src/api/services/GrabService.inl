// Held placement operations are coordinated by the frame owner.
    api::Status getHeldPlacementState(api::grab::v1_1::HeldPlacementState& output) {
        const auto access=s_physicsInteraction.borrow();
        const auto* pi=access.get();
        return pi ? pi->queryHeldPlacementState(output) : api::Status::NotReady;
    }
    api::Status submitHeldPlacementIntent(api::OwnerToken owner, const api::grab::v1_1::HeldPlacementIntent& request) {
        const auto access=s_physicsInteraction.borrow();
        auto* pi=access.get();
        return pi ? pi->submitHeldPlacementIntent(owner,request) : api::Status::NotReady;
    }
    api::Status clearHeldPlacementIntent(api::OwnerToken owner) {
        const auto access=s_physicsInteraction.borrow();
        if (auto* pi=access.get()) pi->clearHeldPlacementIntent(owner);
        return api::Status::Ok;
    }

// Private Grab service operations. Included after shared owner/lifecycle helpers.

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestForceGrabV1(
        std::uint64_t ownerToken,
        const RockProviderForceGrabRequestV1* request,
        std::uint64_t* outCommandId)
    {
        if (!request || !outCommandId || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCommandId = 0;

        if (request->size != sizeof(RockProviderForceGrabRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        const bool fromInventory = (request->flags & static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::FromPlayerInventory)) != 0;
        if (fromInventory ? request->hand != RockProviderHand::None :
            (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left)) {
            return RockProviderResultV1::HandUnavailable;
        }
        if (fromInventory && (request->flags != static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::FromPlayerInventory) ||
            request->targetBodyId != 0x7FFF'FFFF || request->maxDistanceGame != 0.0f)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->targetFormId == 0 || (request->flags & ~kImplementedForceGrabFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!std::isfinite(request->maxDistanceGame)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::UsePreferredGrabPointGame)) != 0 &&
            !isFiniteVector3(request->preferredGrabPointGame)) {
            return RockProviderResultV1::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = RockProviderInteractionCommandKindV1::ForceGrab;
        command.forceGrab = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetInteractionCommandResultV1(
        std::uint64_t ownerToken,
        std::uint64_t commandId,
        RockProviderInteractionCommandResultV1* outResult)
    {
        if (!outResult || ownerToken == 0 || commandId == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        const auto requestedSize = outResult->size;
        if (requestedSize <
            ROCK_PROVIDER_INTERACTION_COMMAND_RESULT_V1_PREFIX_SIZE) {
            return RockProviderResultV1::InvalidSize;
        }
        const auto copyResult = [outResult, requestedSize](
                                    RockProviderInteractionCommandResultV1 result) {
            const auto copySize = (std::min<std::size_t>)(
                requestedSize,
                sizeof(result));
            result.size = static_cast<std::uint32_t>(copySize);
            std::memcpy(outResult, &result, copySize);
        };

        std::scoped_lock lock(s_consumerMutex, s_interactionCommandMutex);
        const auto ownerResult = validateInteractionCommandOwnerLocked(ownerToken);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        for (const auto& slot : s_interactionResults) {
            if (slot.active && slot.result.ownerToken == ownerToken && slot.result.commandId == commandId) {
                copyResult(slot.result);
                return RockProviderResultV1::Ok;
            }
        }

        for (const auto& slot : s_interactionCommands) {
            if (slot.active && slot.command.ownerToken == ownerToken && slot.command.commandId == commandId) {
                copyResult(makeCommandResult(
                    slot.command,
                    RockProviderInteractionCommandStateV1::Queued,
                    RockProviderInteractionFailureV1::None));
                return RockProviderResultV1::Ok;
            }
        }

        return RockProviderResultV1::RequestNotFound;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestForceReleaseV1(
        std::uint64_t ownerToken,
        const RockProviderForceReleaseRequestV1* request,
        std::uint64_t* outCommandId)
    {
        if (!request || !outCommandId || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCommandId = 0;

        if (request->size != sizeof(RockProviderForceReleaseRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if ((request->flags & ~kImplementedForceReleaseFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::RequireMatchingTarget)) != 0 &&
            !hasInteractionTargetIdentity(request->targetFormId, request->targetBodyId)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::UseVelocityHavok)) != 0 &&
            (!isFiniteVector3(request->linearVelocityHavok) || !isFiniteVector3(request->angularVelocityRadiansPerSecond))) {
            return RockProviderResultV1::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = RockProviderInteractionCommandKindV1::ForceRelease;
        command.forceRelease = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestThrownDropV1(
        std::uint64_t ownerToken,
        const RockProviderThrownDropRequestV1* request,
        std::uint64_t* outCommandId)
    {
        if (!request || !outCommandId || ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        *outCommandId = 0;

        if (request->size != sizeof(RockProviderThrownDropRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (request->hand != RockProviderHand::Right && request->hand != RockProviderHand::Left) {
            return RockProviderResultV1::HandUnavailable;
        }
        if ((request->flags & ~kImplementedThrownDropFlagsV1) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::RequireMatchingTarget)) != 0 &&
            !hasInteractionTargetIdentity(request->targetFormId, request->targetBodyId)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::UseVelocityHavok)) != 0 &&
            (!isFiniteVector3(request->linearVelocityHavok) || !isFiniteVector3(request->angularVelocityRadiansPerSecond))) {
            return RockProviderResultV1::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = RockProviderInteractionCommandKindV1::ThrownDrop;
        command.thrownDrop = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetHandInteractionStateV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        RockProviderHandInteractionStateV1* outState)
    {
        provider_state_policy::clearQueryOutput(outState);
        if (!outState || ownerToken == 0 ||
            (hand != RockProviderHand::Right && hand != RockProviderHand::Left)) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (outState->size < sizeof(RockProviderHandInteractionStateV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::HandInteractionState);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        std::scoped_lock lock(s_snapshotMutex);
        if (!apiIsProviderReady() || !s_hasSnapshot || s_lastSnapshot.providerReady == 0) {
            return RockProviderResultV1::NotReady;
        }
        *outState = s_lastHandInteractionStates[
            hand == RockProviderHand::Left ? 1u : 0u];
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCancelInteractionCommandV1(
        const std::uint64_t ownerToken,
        const std::uint64_t commandId)
    {
        if (ownerToken == 0 || commandId == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_interactionCommandMutex);
        const auto ownerResult = validateInteractionCommandOwnerLocked(ownerToken);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_interactionCommands) {
            if (!slot.active || slot.command.ownerToken != ownerToken ||
                slot.command.commandId != commandId) {
                continue;
            }
            completeInteractionCommandLocked(
                slot.command,
                RockProviderInteractionCommandStateV1::Cancelled,
                RockProviderInteractionFailureV1::None);
            slot = {};
            return RockProviderResultV1::Ok;
        }
        for (const auto& slot : s_interactionResults) {
            if (!slot.active || slot.result.ownerToken != ownerToken ||
                slot.result.commandId != commandId) {
                continue;
            }
            return interaction_command_policy::isTerminal(slot.result.state) ?
                RockProviderResultV1::RequestNotFound :
                RockProviderResultV1::AlreadyCommitted;
        }
        return RockProviderResultV1::RequestNotFound;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiAcquireOffhandReservationV1(
        const std::uint64_t ownerToken,
        const RockProviderOffhandReservationRequestV1* request)
    {
        return setOffhandReservationLeaseV1(ownerToken, request, false);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRenewOffhandReservationV1(
        const std::uint64_t ownerToken,
        const RockProviderOffhandReservationRequestV1* request)
    {
        return setOffhandReservationLeaseV1(ownerToken, request, true);
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiReleaseOffhandReservationV1(
        const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_offhandReservationMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::OffhandReservation);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        pruneExpiredOffhandReservationLocked(currentProviderFrameIndex());
        if (s_offhandReservationSlot.ownerToken != 0 &&
            s_offhandReservationSlot.ownerToken != ownerToken) {
            return RockProviderResultV1::OwnerConflict;
        }
        clearOffhandReservationLocked(
            RockProviderSuppressionInvalidationReasonV1::ExplicitClear);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetOffhandReservationStateV1(
        const std::uint64_t ownerToken,
        RockProviderOffhandReservationStateV1* outState)
    {
        provider_state_policy::clearQueryOutput(outState);
        if (ownerToken == 0 || !outState) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (outState->size < sizeof(RockProviderOffhandReservationStateV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_offhandReservationMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(
            ownerToken,
            RockProviderConsumerCapabilityV1::OffhandReservation);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        pruneExpiredOffhandReservationLocked(frameIndex);
        *outState = {};
        outState->reservation = s_offhandReservationSlot.reservation;
        outState->active = s_offhandReservationSlot.ownerToken != 0 ? 1u : 0u;
        outState->ownerToken = s_offhandReservationSlot.ownerToken;
        outState->expiresAfterFrame =
            s_offhandReservationSlot.expiresAfterFrame;
        outState->remainingFrames = provider_lease_policy::remainingFrames(
            frameIndex,
            outState->expiresAfterFrame);
        {
            std::scoped_lock snapshotLock(s_snapshotMutex);
            if (s_hasSnapshot) {
                outState->worldGeneration = s_lastSnapshot.worldGeneration;
                outState->skeletonGeneration =
                    s_lastSnapshot.skeletonGeneration;
                outState->providerGeneration =
                    s_lastSnapshot.providerGeneration;
            }
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetHandTargetDetailsV1(std::uint64_t ownerToken,
        RockProviderHand hand, RockProviderHandTargetDetailsV1* out)
    {
        provider_state_policy::clearQueryOutput(out);
        if (!out || (hand != RockProviderHand::Left && hand != RockProviderHand::Right)) return RockProviderResultV1::InvalidArgument;
        if (out->size != sizeof(*out)) return RockProviderResultV1::InvalidSize;
        if (out->version != ROCK_PROVIDER_API_VERSION) return RockProviderResultV1::UnsupportedVersion;
        const auto permission = validateReadCapability(ownerToken, RockProviderConsumerCapabilityV1::TargetDetails);
        if (permission != RockProviderResultV1::Ok) return permission;
        if (!onAnimationOwnerThread()) return RockProviderResultV1::WrongThread;
        RockProviderHandInteractionStateV1 handState{};
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot || !s_lastSnapshot.providerReady) return RockProviderResultV1::NotReady;
            handState = s_lastHandInteractionStates[hand == RockProviderHand::Left ? 1u : 0u];
        }
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isProviderReady()) return RockProviderResultV1::NotReady;
        pi->getProviderHandTargetDetailsV1(handState, *out);
        out->reference.frameIndex = out->handState.frameIndex;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestPowerArmorGrabV1(std::uint64_t ownerToken,
        const RockProviderPowerArmorGrabRequestV1* request, std::uint64_t* outCommandId)
    {
        if (outCommandId) *outCommandId = 0;
        if (!request || !outCommandId || !ownerToken) return RockProviderResultV1::InvalidArgument;
        if (request->size != sizeof(*request) || request->target.size != sizeof(request->target)) return RockProviderResultV1::InvalidSize;
        if (request->version != ROCK_PROVIDER_API_VERSION || request->target.version != ROCK_PROVIDER_API_VERSION) return RockProviderResultV1::UnsupportedVersion;
        if ((request->hand != RockProviderHand::Left && request->hand != RockProviderHand::Right) ||
            !rock::reference_interaction::pointName(request->point) || !request->target.referenceFormId ||
            !request->target.worldGeneration || !request->target.skeletonGeneration || !request->target.providerGeneration ||
            !std::isfinite(request->maxDistanceGame) || request->maxDistanceGame < 0 || request->maxDistanceGame > 32) return RockProviderResultV1::InvalidArgument;
        const auto permission = validateReadCapability(ownerToken, RockProviderConsumerCapabilityV1::PowerArmor);
        if (permission != RockProviderResultV1::Ok) return permission;
        QueuedInteractionCommandV1 command{};
        command.kind = RockProviderInteractionCommandKindV1::ForceGrab;
        command.ownerToken = ownerToken;
        command.powerArmorPoint = request->point;
        command.powerArmorReferenceNativeHandle = request->target.referenceNativeHandle;
        auto& grab = command.forceGrab;
        grab.hand = request->hand;
        grab.targetFormId = request->target.referenceFormId;
        grab.worldGeneration = request->target.worldGeneration;
        grab.skeletonGeneration = request->target.skeletonGeneration;
        grab.providerGeneration = request->target.providerGeneration;
        grab.maxDistanceGame = request->maxDistanceGame;
        return enqueueInteractionCommand(command, outCommandId);
    }
