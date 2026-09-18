Status ROCK_CALL getSample(OwnerToken owner, SampleV1* outSample) noexcept {
    if (!outSample) return Status::InvalidArgument;
    *outSample = {};
    return invoke(owner, kInterfaceId, 1, false, [&]() {
        *outSample = provider::runtime::sample();
        return Status::Ok;
    });
}

Status ROCK_CALL registerConsumerV1(const RegistrationV1* registration, OwnerV1* outHandle) noexcept {
    if (provider::events::inSynchronousCallback()) return Status::Busy;
    if (const auto s=checkInput(registration); s!=Status::Ok) return s;
    if (const auto s=checkOutput(outHandle); s!=Status::Ok) return s;
    try {
        provider::RockProviderConsumerRegistrationV1 request{};
        provider::RockProviderConsumerHandleV1 handle{};
        convert(request, *registration);
        request.requestedCapabilities = 0xFFFFFFFFu;
        const auto status = provider::runtime::apiRegisterConsumerV1(&request, &handle);
        convert(*outHandle, handle);
        return static_cast<Status>(status);
    } catch (...) { return Status::InternalError; }
}
Status ROCK_CALL unregisterConsumerV1(std::uint64_t ownerToken) noexcept {
    return invoke(ownerToken, kInterfaceId, 0, true, [&]() {
        return static_cast<Status>(provider::runtime::apiUnregisterConsumerV1(ownerToken));
    });
}
Status ROCK_CALL registerAnimationPhaseCallbackV1(std::uint64_t ownerToken, PhaseCallbackV1 callback, void* userData, std::uint64_t* outCallbackToken) noexcept {
    if (outCallbackToken) *outCallbackToken=0;
    return invoke(ownerToken, kInterfaceId, 4, true, [&]() {
        return static_cast<Status>(provider::runtime::apiRegisterAnimationPhaseCallbackV1(ownerToken, callback, userData, outCallbackToken));
    });
}
Status ROCK_CALL unregisterAnimationPhaseCallbackV1(std::uint64_t ownerToken, std::uint64_t callbackToken) noexcept {
    return invoke(ownerToken, kInterfaceId, 4, true, [&]() {
        return static_cast<Status>(provider::runtime::apiUnregisterAnimationPhaseCallbackV1(ownerToken, callbackToken));
    });
}
Status ROCK_CALL registerFrameCallbackForOwnerV1(std::uint64_t ownerToken, FrameCallbackV1 callback, void* userData, std::uint64_t* outCallbackToken) noexcept {
    if (outCallbackToken) *outCallbackToken=0;
    return invoke(ownerToken, kInterfaceId, 4, true, [&]() {
        return static_cast<Status>(provider::runtime::apiRegisterFrameCallbackForOwnerV1(ownerToken, callback, userData, outCallbackToken));
    });
}
Status ROCK_CALL unregisterFrameCallbackForOwnerV1(std::uint64_t ownerToken, std::uint64_t callbackToken) noexcept {
    return invoke(ownerToken, kInterfaceId, 4, true, [&]() {
        return static_cast<Status>(provider::runtime::apiUnregisterFrameCallbackForOwnerV1(ownerToken, callbackToken));
    });
}
Status ROCK_CALL bindInterface(OwnerToken owner, InterfaceId family, std::uint32_t major, std::uint32_t permissions) noexcept {
    try { return provider::runtime::bind(owner, family, major, permissions); }
    catch (...) { return Status::InternalError; }
}

Status ROCK_CALL copyEvents(OwnerToken owner,std::uint64_t after,EventV1* events,std::uint32_t capacity,StreamV1* state) noexcept {
    if (!state) return Status::InvalidArgument;
    *state={};
    if (capacity && !events) return Status::InvalidArgument;
    if (capacity>256) return Status::CapacityFull;
    for (std::uint32_t i=0;i<capacity;++i) if (const auto status=checkOutput(events+i);status!=Status::Ok) return status;
    return invoke(owner,kInterfaceId,1,false,[&]() { return provider::events::copy(owner,after,events,capacity,*state); });
}
