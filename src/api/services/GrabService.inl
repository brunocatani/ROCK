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

    api::Status requestForceGrab(
        std::uint64_t ownerToken,
        const api::grab::ForceGrabRequest& request,
        std::uint64_t& outCommandId)
    {
        outCommandId = 0;
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        const bool fromInventory = (request.flags & kForceGrabFromPlayerInventory) != 0;
        if (fromInventory ? request.hand != api::Hand::None :
            (request.hand != api::Hand::Right && request.hand != api::Hand::Left)) {
            return api::Status::HandUnavailable;
        }
        if (fromInventory && (request.flags != kForceGrabFromPlayerInventory ||
            request.targetBodyId != 0x7FFF'FFFF || request.maxDistanceGame != 0.0f)) {
            return api::Status::InvalidArgument;
        }
        if (request.targetFormId == 0 || (request.flags & ~kImplementedForceGrabFlags) != 0) {
            return api::Status::InvalidArgument;
        }
        if (!std::isfinite(request.maxDistanceGame)) {
            return api::Status::InvalidArgument;
        }
        if ((request.flags & static_cast<std::uint32_t>(api::grab::ForceGrabFlag::UsePreferredGrabPointGame)) != 0 &&
            !isFiniteVector3(request.preferredGrabPointGame)) {
            return api::Status::InvalidArgument;
        }

        QueuedInteractionCommand command{};
        command.ownerToken = ownerToken;
        command.kind = api::grab::InteractionCommandKind::ForceGrab;
        command.forceGrab = request;
        return enqueueInteractionCommand(command, &outCommandId);
    }

    api::Status getInteractionCommandResult(
        std::uint64_t ownerToken,
        std::uint64_t commandId,
        api::grab::InteractionCommandResult& outResult)
    {
        if (ownerToken == 0 || commandId == 0) {
            return api::Status::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_interactionCommandMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }

        for (const auto& slot : s_interactionResults) {
            if (slot.active && slot.result.ownerToken == ownerToken && slot.result.commandId == commandId) {
                outResult = slot.result;
                return api::Status::Ok;
            }
        }

        for (const auto& slot : s_interactionCommands) {
            if (slot.active && slot.command.ownerToken == ownerToken && slot.command.commandId == commandId) {
                outResult = makeCommandResult(
                    slot.command,
                    api::grab::InteractionCommandState::Queued,
                    api::grab::InteractionFailure::None);
                return api::Status::Ok;
            }
        }

        return api::Status::RequestNotFound;
    }

    // Force release and thrown drop share one request shape and its checks.
    template <class Request>
    api::Status validateReleaseRequest(const Request& request, std::uint32_t implementedFlags,
        std::uint32_t requireMatchingTarget, std::uint32_t useVelocity)
    {
        if (request.hand != api::Hand::Right && request.hand != api::Hand::Left) {
            return api::Status::HandUnavailable;
        }
        if ((request.flags & ~implementedFlags) != 0) {
            return api::Status::InvalidArgument;
        }
        if ((request.flags & requireMatchingTarget) != 0 &&
            !hasInteractionTargetIdentity(request.targetFormId, request.targetBodyId)) {
            return api::Status::InvalidArgument;
        }
        if ((request.flags & useVelocity) != 0 &&
            (!isFiniteVector3(request.linearVelocityHavok) || !isFiniteVector3(request.angularVelocityRadiansPerSecond))) {
            return api::Status::InvalidArgument;
        }
        return api::Status::Ok;
    }

    api::Status requestForceRelease(
        std::uint64_t ownerToken,
        const api::grab::ForceReleaseRequest& request,
        std::uint64_t& outCommandId)
    {
        outCommandId = 0;
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        using Flag = api::grab::ForceReleaseFlag;
        if (const auto status = validateReleaseRequest(request, kImplementedForceReleaseFlags,
                static_cast<std::uint32_t>(Flag::RequireMatchingTarget), static_cast<std::uint32_t>(Flag::UseVelocityHavok));
            status != api::Status::Ok) {
            return status;
        }

        QueuedInteractionCommand command{};
        command.ownerToken = ownerToken;
        command.kind = api::grab::InteractionCommandKind::ForceRelease;
        command.forceRelease = request;
        return enqueueInteractionCommand(command, &outCommandId);
    }

    api::Status requestThrownDrop(
        std::uint64_t ownerToken,
        const api::grab::ThrownDropRequest& request,
        std::uint64_t& outCommandId)
    {
        outCommandId = 0;
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        using Flag = api::grab::ThrownDropFlag;
        if (const auto status = validateReleaseRequest(request, kImplementedThrownDropFlags,
                static_cast<std::uint32_t>(Flag::RequireMatchingTarget), static_cast<std::uint32_t>(Flag::UseVelocityHavok));
            status != api::Status::Ok) {
            return status;
        }

        QueuedInteractionCommand command{};
        command.ownerToken = ownerToken;
        command.kind = api::grab::InteractionCommandKind::ThrownDrop;
        command.thrownDrop = request;
        return enqueueInteractionCommand(command, &outCommandId);
    }

    api::Status getHandInteractionState(
        const std::uint64_t ownerToken,
        const api::Hand hand,
        api::grab::HandInteractionState& outState)
    {
        outState = {};
        if (ownerToken == 0 ||
            (hand != api::Hand::Right && hand != api::Hand::Left)) {
            return api::Status::InvalidArgument;
        }
        const auto ownerResult = validateRegisteredOwner(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        std::scoped_lock lock(s_snapshotMutex);
        if (!isProviderReady() || !s_hasSnapshot || s_lastSnapshot.providerReady == 0) {
            return api::Status::NotReady;
        }
        outState = s_lastHandInteractionStates[
            hand == api::Hand::Left ? 1u : 0u];
        return api::Status::Ok;
    }

    api::Status cancelInteractionCommand(
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

    api::Status acquireHandReservation(
        const std::uint64_t ownerToken,
        const api::grab::HandReservationRequest& request)
    {
        return setHandReservationLease(ownerToken, request, false);
    }

    api::Status renewHandReservation(
        const std::uint64_t ownerToken,
        const api::grab::HandReservationRequest& request)
    {
        return setHandReservationLease(ownerToken, request, true);
    }

    api::Status releaseHandReservation(
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

    api::Status getHandReservationState(
        const std::uint64_t ownerToken,
        api::grab::HandReservationState& outState)
    {
        outState = {};
        if (ownerToken == 0) {
            return api::Status::InvalidArgument;
        }
        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_offhandReservationMutex);
        const auto ownerResult = validateRegisteredOwnerLocked(ownerToken);
        if (ownerResult != api::Status::Ok) {
            return ownerResult;
        }
        pruneExpiredOffhandReservationLocked(frameIndex);
        outState.hand = s_offhandReservationSlot.hand;
        outState.reservation = s_offhandReservationSlot.reservation;
        outState.active = s_offhandReservationSlot.ownerToken != 0 ? 1u : 0u;
        outState.ownerToken = s_offhandReservationSlot.ownerToken;
        outState.expiresAfterFrame =
            s_offhandReservationSlot.expiresAfterFrame;
        outState.remainingFrames = provider_lease_policy::remainingFrames(
            frameIndex,
            outState.expiresAfterFrame);
        {
            std::scoped_lock snapshotLock(s_snapshotMutex);
            if (s_hasSnapshot) {
                outState.worldGeneration = s_lastSnapshot.worldGeneration;
                outState.skeletonGeneration =
                    s_lastSnapshot.skeletonGeneration;
                outState.providerGeneration =
                    s_lastSnapshot.providerGeneration;
            }
        }
        return api::Status::Ok;
    }

    api::Status getHandTargetDetails(std::uint64_t ownerToken,
        api::Hand hand, api::grab::HandTargetDetails& out)
    {
        out = {};
        if (hand != api::Hand::Left && hand != api::Hand::Right) return api::Status::InvalidArgument;
        const auto permission = validateRegisteredOwner(ownerToken);
        if (permission != api::Status::Ok) return permission;
        if (!onAnimationOwnerThread()) return api::Status::WrongThread;
        HandInteractionPublication handState{};
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot || !s_lastSnapshot.providerReady) return api::Status::NotReady;
            handState = s_lastHandInteractionStates[hand == api::Hand::Left ? 1u : 0u];
        }
        const auto instanceAccess = s_physicsInteraction.borrow();
        auto* pi = instanceAccess.get();
        if (!pi || !pi->isProviderReady()) return api::Status::NotReady;
        pi->getProviderHandTargetDetails(handState, out);
        return api::Status::Ok;
    }

    api::Status requestPowerArmorGrab(std::uint64_t ownerToken,
        const api::grab::PowerArmorGrabRequest& request, std::uint64_t& outCommandId)
    {
        outCommandId = 0;
        if (!ownerToken) return api::Status::InvalidArgument;
        if ((request.hand != api::Hand::Left && request.hand != api::Hand::Right) ||
            !rock::reference_interaction::pointName(request.point) || !request.target.referenceFormId ||
            !request.target.worldGeneration || !request.target.skeletonGeneration || !request.target.providerGeneration ||
            !std::isfinite(request.maxDistanceGame) || request.maxDistanceGame < 0 || request.maxDistanceGame > 32) return api::Status::InvalidArgument;
        const auto permission = validateRegisteredOwner(ownerToken);
        if (permission != api::Status::Ok) return permission;
        QueuedInteractionCommand command{};
        command.kind = api::grab::InteractionCommandKind::ForceGrab;
        command.ownerToken = ownerToken;
        command.powerArmorPoint = request.point;
        command.powerArmorReferenceNativeHandle = request.target.referenceNativeHandle;
        auto& grab = command.forceGrab;
        grab.hand = request.hand;
        grab.targetFormId = request.target.referenceFormId;
        grab.worldGeneration = request.target.worldGeneration;
        grab.skeletonGeneration = request.target.skeletonGeneration;
        grab.providerGeneration = request.target.providerGeneration;
        grab.maxDistanceGame = request.maxDistanceGame;
        return enqueueInteractionCommand(command, &outCommandId);
    }
