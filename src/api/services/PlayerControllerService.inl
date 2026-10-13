// Private PlayerController service operations. Included after shared owner/lifecycle helpers.
// They fill provider API major-2 PlayerController records directly; the
// endpoint has already validated the caller's records.

    api::Status apiGetPlayerControllerState(
        const std::uint64_t ownerToken,
        api::playercontroller::PlayerControllerState& outState)
    {
        if (!onAnimationOwnerThread()) {
            return api::Status::WrongThread;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::PlayerController);
        if (ownerResult != RockProviderResultV1::Ok) {
            return static_cast<api::Status>(ownerResult);
        }
        if (!apiIsProviderReady()) {
            return api::Status::NotReady;
        }

        RockProviderFrameSnapshot snapshot{};
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot || !hasLifecycleFlag(
                    s_lastSnapshot.lifecycleFlags,
                    RockProviderLifecycleFlag::ProviderReady)) {
                return api::Status::NotReady;
            }
            snapshot = s_lastSnapshot;
        }

        rock::character_controller_runtime::PlayerControllerState state{};
        if (!rock::character_controller_runtime::tryGetPlayerControllerState(
                state)) {
            return api::Status::NotReady;
        }

        using Flag = api::playercontroller::PlayerControllerStateFlag;
        std::uint32_t flags = static_cast<std::uint32_t>(Flag::Valid);
        const auto addFlag = [&flags](const bool enabled, const Flag flag) {
            if (enabled) {
                flags |= static_cast<std::uint32_t>(flag);
            }
        };
        addFlag(state.positionValid, Flag::PositionValid);
        addFlag(state.velocityValid, Flag::VelocityValid);
        addFlag(state.shapeValid, Flag::ShapeValid);
        addFlag(state.supportNormalValid, Flag::SupportNormalValid);
        addFlag(
            state.supportState ==
                rock::character_controller_runtime::PlayerSupportState::Supported,
            Flag::Supported);
        addFlag(
            state.supportState ==
                rock::character_controller_runtime::PlayerSupportState::Sliding,
            Flag::Sliding);
        addFlag(
            state.implementation == rock::character_controller_runtime::
                PlayerControllerImplementation::Proxy,
            Flag::Proxy);
        addFlag(
            state.implementation == rock::character_controller_runtime::
                PlayerControllerImplementation::RigidBody,
            Flag::RigidBody);

        const auto point = [](const RE::NiPoint3& value) {
            return api::Point3{ value.x, value.y, value.z };
        };
        outState.sample = { snapshot.frameIndex, snapshot.stateSequence, snapshot.worldGeneration,
            snapshot.skeletonGeneration, snapshot.providerGeneration, snapshot.collisionGeneration };
        outState.flags = flags;
        outState.implementation =
            static_cast<api::playercontroller::PlayerControllerImplementation>(state.implementation);
        outState.supportState =
            static_cast<api::playercontroller::PlayerSupportState>(state.supportState);
        outState.positionGame = point(state.positionGame);
        outState.velocityGame = point(state.velocityGame);
        outState.supportNormalGame = point(state.supportNormal);
        outState.radiusGame = state.radiusGame;
        outState.heightGame = state.heightGame;
        return api::Status::Ok;
    }

    api::Status apiRequestPlayerControllerJump(
        const std::uint64_t ownerToken,
        const api::playercontroller::PlayerControllerJumpRequest& request)
    {
        if (!std::isfinite(request.heightGameUnits) ||
            request.heightGameUnits <= 0.0f ||
            request.heightGameUnits > api::playercontroller::kMaxNativeJumpHeightGame ||
            request.worldGeneration == 0 ||
            request.skeletonGeneration == 0 ||
            request.providerGeneration == 0) {
            return api::Status::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return api::Status::WrongThread;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::PlayerController);
        if (ownerResult != RockProviderResultV1::Ok) {
            return static_cast<api::Status>(ownerResult);
        }
        const auto generationResult = validateGenerationGuards(
            request.worldGeneration,
            request.skeletonGeneration,
            request.providerGeneration);
        if (generationResult != RockProviderResultV1::Ok) {
            return static_cast<api::Status>(generationResult);
        }
        if (!apiIsProviderReady()) {
            return api::Status::NotReady;
        }

        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot || !hasLifecycleFlag(
                    s_lastSnapshot.lifecycleFlags,
                    RockProviderLifecycleFlag::ProviderReady) ||
                !hasLifecycleFlag(
                    s_lastSnapshot.lifecycleFlags,
                    RockProviderLifecycleFlag::PhysicsWriteAllowed)) {
                return api::Status::NotReady;
            }
        }

        rock::character_controller_runtime::PlayerControllerState state{};
        if (!rock::character_controller_runtime::tryGetPlayerControllerState(
                state)) {
            return api::Status::NotReady;
        }
        return rock::character_controller_runtime::requestPlayerJump(
                   request.heightGameUnits) ?
            api::Status::Ok :
            api::Status::NotReady;
    }
