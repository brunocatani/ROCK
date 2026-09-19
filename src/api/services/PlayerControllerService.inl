// Private PlayerController service operations. Included after shared owner/lifecycle helpers.

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetPlayerControllerStateV1(
        const std::uint64_t ownerToken,
        const std::uint32_t queryFlags,
        RockProviderPlayerControllerStateV1* outState)
    {
        provider_state_policy::clearQueryOutput(outState);
        if (ownerToken == 0 || !outState) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (outState->size < sizeof(RockProviderPlayerControllerStateV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (outState->version == 0 ||
            outState->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        constexpr std::uint32_t implementedQueryFlags =
            static_cast<std::uint32_t>(
                RockProviderPlayerControllerQueryFlagV1::CheckPenetration);
        if ((queryFlags & ~implementedQueryFlags) != 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::PlayerController);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        if (!apiIsProviderReady()) {
            return RockProviderResultV1::NotReady;
        }

        RockProviderFrameSnapshot snapshot{};
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot || !hasLifecycleFlag(
                    s_lastSnapshot.lifecycleFlags,
                    RockProviderLifecycleFlag::ProviderReady)) {
                return RockProviderResultV1::NotReady;
            }
            snapshot = s_lastSnapshot;
        }

        rock::character_controller_runtime::PlayerControllerState state{};
        // The legacy CheckPenetration bit is accepted for ABI compatibility
        // but intentionally produces no flags. FO4VR's controller virtual at
        // +0x1E8 reports blocking-layer contact, not penetration depth.
        static_cast<void>(queryFlags);
        if (!rock::character_controller_runtime::tryGetPlayerControllerState(
                state)) {
            return RockProviderResultV1::NotReady;
        }

        std::uint32_t flags = static_cast<std::uint32_t>(
            RockProviderPlayerControllerStateFlagV1::Valid);
        const auto addFlag = [&flags](
                                 const bool enabled,
                                 const RockProviderPlayerControllerStateFlagV1 flag) {
            if (enabled) {
                flags |= static_cast<std::uint32_t>(flag);
            }
        };
        addFlag(state.positionValid,
            RockProviderPlayerControllerStateFlagV1::PositionValid);
        addFlag(state.velocityValid,
            RockProviderPlayerControllerStateFlagV1::VelocityValid);
        addFlag(state.shapeValid,
            RockProviderPlayerControllerStateFlagV1::ShapeValid);
        addFlag(state.supportNormalValid,
            RockProviderPlayerControllerStateFlagV1::SupportNormalValid);
        addFlag(
            state.supportState ==
                rock::character_controller_runtime::PlayerSupportState::Supported,
            RockProviderPlayerControllerStateFlagV1::Supported);
        addFlag(
            state.supportState ==
                rock::character_controller_runtime::PlayerSupportState::Sliding,
            RockProviderPlayerControllerStateFlagV1::Sliding);
        addFlag(
            state.implementation == rock::character_controller_runtime::
                PlayerControllerImplementation::Proxy,
            RockProviderPlayerControllerStateFlagV1::Proxy);
        addFlag(
            state.implementation == rock::character_controller_runtime::
                PlayerControllerImplementation::RigidBody,
            RockProviderPlayerControllerStateFlagV1::RigidBody);

        *outState = {};
        outState->frameIndex = snapshot.frameIndex;
        outState->flags = flags;
        outState->implementation =
            static_cast<RockProviderPlayerControllerImplementationV1>(
                state.implementation);
        outState->supportState =
            static_cast<RockProviderPlayerSupportStateV1>(state.supportState);
        outState->positionGame = toProviderPoint(state.positionGame);
        outState->velocityGame = toProviderPoint(state.velocityGame);
        outState->supportNormalGame = toProviderPoint(state.supportNormal);
        outState->radiusGame = state.radiusGame;
        outState->heightGame = state.heightGame;
        outState->worldGeneration = snapshot.worldGeneration;
        outState->skeletonGeneration = snapshot.skeletonGeneration;
        outState->providerGeneration = snapshot.providerGeneration;
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiRequestPlayerControllerJumpV1(
        const std::uint64_t ownerToken,
        const RockProviderPlayerControllerJumpRequestV1* request)
    {
        if (ownerToken == 0 || !request) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (request->size !=
            sizeof(RockProviderPlayerControllerJumpRequestV1)) {
            return RockProviderResultV1::InvalidSize;
        }
        if (request->version == 0 ||
            request->version > ROCK_PROVIDER_API_VERSION) {
            return RockProviderResultV1::UnsupportedVersion;
        }
        if (!std::isfinite(request->heightGameUnits) ||
            request->heightGameUnits <= 0.0f ||
            request->heightGameUnits > 256.0f ||
            request->worldGeneration == 0 ||
            request->skeletonGeneration == 0 ||
            request->providerGeneration == 0) {
            return RockProviderResultV1::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return RockProviderResultV1::WrongThread;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::PlayerController);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
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

        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot || !hasLifecycleFlag(
                    s_lastSnapshot.lifecycleFlags,
                    RockProviderLifecycleFlag::ProviderReady) ||
                !hasLifecycleFlag(
                    s_lastSnapshot.lifecycleFlags,
                    RockProviderLifecycleFlag::PhysicsWriteAllowed)) {
                return RockProviderResultV1::NotReady;
            }
        }

        rock::character_controller_runtime::PlayerControllerState state{};
        if (!rock::character_controller_runtime::tryGetPlayerControllerState(
                state)) {
            return RockProviderResultV1::NotReady;
        }
        return rock::character_controller_runtime::requestPlayerJump(
                   request->heightGameUnits) ?
            RockProviderResultV1::Ok :
            RockProviderResultV1::NotReady;
    }
