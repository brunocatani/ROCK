// Held placement operations are coordinated by the frame owner.
    api::Status getHeldObjectPose(api::OwnerToken owner,api::Hand hand,api::grab::HeldObjectPose& output) {
        const auto access=s_physicsInteraction.borrow();const auto* pi=access.get();
        return pi?pi->getHeldObjectPose(owner,hand,output):api::Status::NotReady;
    }
    api::Status setHeldObjectTarget(api::OwnerToken owner,const api::grab::HeldObjectTarget& request) {
        const auto access=s_physicsInteraction.borrow();
        auto* pi=access.get();return pi?pi->setHeldObjectTarget(owner,request):api::Status::NotReady;
    }
    api::Status clearHeldObjectTarget(api::OwnerToken owner,api::Hand hand) {
        const auto access=s_physicsInteraction.borrow();
        auto* pi=access.get();return pi?pi->clearHeldObjectTarget(owner,hand):api::Status::Ok;
    }
    api::Status getHeldPlacementState(api::grab::HeldPlacementState& output) {
        const auto access=s_physicsInteraction.borrow();
        const auto* pi=access.get();
        return pi ? pi->queryHeldPlacementState(output) : api::Status::NotReady;
    }
    api::Status submitHeldPlacementIntent(api::OwnerToken owner, const api::grab::HeldPlacementIntent& request) {
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

    api::Status apiRequestForceGrabV1(
        std::uint64_t ownerToken,
        const RockProviderForceGrabRequestV1* request,
        std::uint64_t* outCommandId)
    {
        if (!request || !outCommandId || ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        *outCommandId = 0;

        if (request->size != sizeof(RockProviderForceGrabRequestV1)) {
            return api::Status::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return api::Status::UnsupportedVersion;
        }
        const bool fromInventory = (request->flags & static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::FromPlayerInventory)) != 0;
        if (fromInventory ? request->hand != api::Hand::None :
            (request->hand != api::Hand::Right && request->hand != api::Hand::Left)) {
            return api::Status::HandUnavailable;
        }
        if (fromInventory && (request->flags != static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::FromPlayerInventory) ||
            request->targetBodyId != 0x7FFF'FFFF || request->maxDistanceGame != 0.0f)) {
            return api::Status::InvalidArgument;
        }
        if (request->targetFormId == 0 || (request->flags & ~kImplementedForceGrabFlagsV1) != 0) {
            return api::Status::InvalidArgument;
        }
        if (!std::isfinite(request->maxDistanceGame)) {
            return api::Status::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::UsePreferredGrabPointGame)) != 0 &&
            !isFiniteVector3(request->preferredGrabPointGame)) {
            return api::Status::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = api::grab::InteractionCommandKind::ForceGrab;
        command.forceGrab = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    api::Status apiGetInteractionCommandResultV1(
        std::uint64_t ownerToken,
        std::uint64_t commandId,
        RockProviderInteractionCommandResultV1* outResult)
    {
        if (!outResult || ownerToken == 0 || commandId == 0) {
            return api::Status::InvalidArgument;
        }
        const auto requestedSize = outResult->size;
        if (requestedSize <
            ROCK_PROVIDER_INTERACTION_COMMAND_RESULT_V1_PREFIX_SIZE) {
            return api::Status::InvalidSize;
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
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }

        for (const auto& slot : s_interactionResults) {
            if (slot.active && slot.result.ownerToken == ownerToken && slot.result.commandId == commandId) {
                copyResult(slot.result);
                return api::Status::Ok;
            }
        }

        for (const auto& slot : s_interactionCommands) {
            if (slot.active && slot.command.ownerToken == ownerToken && slot.command.commandId == commandId) {
                copyResult(makeCommandResult(
                    slot.command,
                    api::grab::InteractionCommandState::Queued,
                    api::grab::InteractionFailure::None));
                return api::Status::Ok;
            }
        }

        return api::Status::RequestNotFound;
    }

    api::Status apiRequestForceReleaseV1(
        std::uint64_t ownerToken,
        const RockProviderForceReleaseRequestV1* request,
        std::uint64_t* outCommandId)
    {
        if (!request || !outCommandId || ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        *outCommandId = 0;

        if (request->size != sizeof(RockProviderForceReleaseRequestV1)) {
            return api::Status::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return api::Status::UnsupportedVersion;
        }
        if (request->hand != api::Hand::Right && request->hand != api::Hand::Left) {
            return api::Status::HandUnavailable;
        }
        if ((request->flags & ~kImplementedForceReleaseFlagsV1) != 0) {
            return api::Status::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(api::grab::ForceReleaseFlag::RequireMatchingTarget)) != 0 &&
            !hasInteractionTargetIdentity(request->targetFormId, request->targetBodyId)) {
            return api::Status::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(api::grab::ForceReleaseFlag::UseVelocityHavok)) != 0 &&
            (!isFiniteVector3(request->linearVelocityHavok) || !isFiniteVector3(request->angularVelocityRadiansPerSecond))) {
            return api::Status::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = api::grab::InteractionCommandKind::ForceRelease;
        command.forceRelease = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    api::Status apiRequestThrownDropV1(
        std::uint64_t ownerToken,
        const RockProviderThrownDropRequestV1* request,
        std::uint64_t* outCommandId)
    {
        if (!request || !outCommandId || ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        *outCommandId = 0;

        if (request->size != sizeof(RockProviderThrownDropRequestV1)) {
            return api::Status::InvalidSize;
        }
        if (request->version == 0 || request->version > ROCK_PROVIDER_API_VERSION) {
            return api::Status::UnsupportedVersion;
        }
        if (request->hand != api::Hand::Right && request->hand != api::Hand::Left) {
            return api::Status::HandUnavailable;
        }
        if ((request->flags & ~kImplementedThrownDropFlagsV1) != 0) {
            return api::Status::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(api::grab::ThrownDropFlag::RequireMatchingTarget)) != 0 &&
            !hasInteractionTargetIdentity(request->targetFormId, request->targetBodyId)) {
            return api::Status::InvalidArgument;
        }
        if ((request->flags & static_cast<std::uint32_t>(api::grab::ThrownDropFlag::UseVelocityHavok)) != 0 &&
            (!isFiniteVector3(request->linearVelocityHavok) || !isFiniteVector3(request->angularVelocityRadiansPerSecond))) {
            return api::Status::InvalidArgument;
        }

        QueuedInteractionCommandV1 command{};
        command.ownerToken = ownerToken;
        command.kind = api::grab::InteractionCommandKind::ThrownDrop;
        command.thrownDrop = *request;
        return enqueueInteractionCommand(command, outCommandId);
    }

    api::Status apiGetHandInteractionStateV1(
        const std::uint64_t ownerToken,
        const api::Hand hand,
        RockProviderHandInteractionStateV1* outState)
    {
        provider_state_policy::clearQueryOutput(outState);
        if (!outState || ownerToken == 0 ||
            (hand != api::Hand::Right && hand != api::Hand::Left)) {
            return api::Status::InvalidArgument;
        }
        if (outState->size < sizeof(RockProviderHandInteractionStateV1)) {
            return api::Status::InvalidSize;
        }
        const auto ownerResult = validateRegisteredOwner(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        std::scoped_lock lock(s_snapshotMutex);
        if (!apiIsProviderReady() || !s_hasSnapshot || s_lastSnapshot.providerReady == 0) {
            return api::Status::NotReady;
        }
        *outState = s_lastHandInteractionStates[
            hand == api::Hand::Left ? 1u : 0u];
        return api::Status::Ok;
    }

    api::Status apiCancelInteractionCommandV1(
        const std::uint64_t ownerToken,
        const std::uint64_t commandId)
    {
        if (ownerToken == 0 || commandId == 0) {
            return api::Status::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_interactionCommandMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        for (auto& slot : s_interactionCommands) {
            if (!slot.active || slot.command.ownerToken != ownerToken ||
                slot.command.commandId != commandId) {
                continue;
            }
            completeInteractionCommandLocked(
                slot.command,
                api::grab::InteractionCommandState::Cancelled,
                api::grab::InteractionFailure::None);
            slot = {};
            return api::Status::Ok;
        }
        for (const auto& slot : s_interactionResults) {
            if (!slot.active || slot.result.ownerToken != ownerToken ||
                slot.result.commandId != commandId) {
                continue;
            }
            return interaction_command_policy::isTerminal(slot.result.state) ?
                api::Status::RequestNotFound :
                api::Status::AlreadyCommitted;
        }
        return api::Status::RequestNotFound;
    }

    api::Status apiAcquireOffhandReservationV1(
        const std::uint64_t ownerToken,
        const RockProviderOffhandReservationRequestV1* request)
    {
        return setOffhandReservationLeaseV1(ownerToken, request, false);
    }

    api::Status apiRenewOffhandReservationV1(
        const std::uint64_t ownerToken,
        const RockProviderOffhandReservationRequestV1* request)
    {
        return setOffhandReservationLeaseV1(ownerToken, request, true);
    }

    api::Status apiReleaseOffhandReservationV1(
        const std::uint64_t ownerToken)
    {
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        std::scoped_lock lock(s_consumerMutex, s_offhandReservationMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        pruneExpiredOffhandReservationLocked(currentProviderFrameIndex());
        if (s_offhandReservationSlot.ownerToken != 0 &&
            s_offhandReservationSlot.ownerToken != ownerToken) {
            return api::Status::OwnerConflict;
        }
        clearOffhandReservationLocked(
            ::rock::api::input::SuppressionInvalidationReason::ExplicitClear);
        return api::Status::Ok;
    }

    api::Status apiGetOffhandReservationStateV1(
        const std::uint64_t ownerToken,
        RockProviderOffhandReservationStateV1* outState)
    {
        provider_state_policy::clearQueryOutput(outState);
        if (ownerToken == 0 || !outState) {
            return api::Status::InvalidArgument;
        }
        if (outState->size < sizeof(RockProviderOffhandReservationStateV1)) {
            return api::Status::InvalidSize;
        }
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_offhandReservationMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        pruneExpiredOffhandReservationLocked(frameIndex);
        *outState = {};
        outState->hand = s_offhandReservationSlot.hand;
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
        return api::Status::Ok;
    }

    api::Status apiGetHandTargetDetailsV1(std::uint64_t ownerToken,
        api::Hand hand, RockProviderHandTargetDetailsV1* out)
    {
        provider_state_policy::clearQueryOutput(out);
        if (!out || (hand != api::Hand::Left && hand != api::Hand::Right)) return api::Status::InvalidArgument;
        if (out->size != sizeof(*out)) return api::Status::InvalidSize;
        if (out->version != ROCK_PROVIDER_API_VERSION) return api::Status::UnsupportedVersion;
        const auto permission = validateRegisteredOwner(ownerToken);
        if (permission != api::Status::Ok) return permission;
        if (!onAnimationOwnerThread()) return api::Status::WrongThread;
        RockProviderHandInteractionStateV1 handState{};
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot || !s_lastSnapshot.providerReady) return api::Status::NotReady;
            handState = s_lastHandInteractionStates[hand == api::Hand::Left ? 1u : 0u];
        }
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isProviderReady()) return api::Status::NotReady;
        pi->getProviderHandTargetDetailsV1(handState, *out);
        out->reference.frameIndex = out->handState.frameIndex;
        return api::Status::Ok;
    }

    api::Status apiRequestPowerArmorGrabV1(std::uint64_t ownerToken,
        const RockProviderPowerArmorGrabRequestV1* request, std::uint64_t* outCommandId)
    {
        if (outCommandId) *outCommandId = 0;
        if (!request || !outCommandId || !ownerToken) return api::Status::InvalidArgument;
        if (request->size != sizeof(*request) || request->target.size != sizeof(request->target)) return api::Status::InvalidSize;
        if (request->version != ROCK_PROVIDER_API_VERSION || request->target.version != ROCK_PROVIDER_API_VERSION) return api::Status::UnsupportedVersion;
        if ((request->hand != api::Hand::Left && request->hand != api::Hand::Right) ||
            !rock::reference_interaction::pointName(request->point) || !request->target.referenceFormId ||
            !request->target.worldGeneration || !request->target.skeletonGeneration || !request->target.providerGeneration ||
            !std::isfinite(request->maxDistanceGame) || request->maxDistanceGame < 0 || request->maxDistanceGame > 32) return api::Status::InvalidArgument;
        const auto permission = validateRegisteredOwner(ownerToken);
        if (permission != api::Status::Ok) return permission;
        QueuedInteractionCommandV1 command{};
        command.kind = api::grab::InteractionCommandKind::ForceGrab;
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
