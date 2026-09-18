Status ROCK_CALL getSample(OwnerToken owner, SampleV1* outSample) noexcept {
    if (!outSample) return Status::InvalidArgument;
    *outSample = {};
    return invoke(owner, kInterfaceId, 1, false, [&]() {
        *outSample = provider::runtime::sample();
        return Status::Ok;
    });
}

Status ROCK_CALL getEnvironment(OwnerToken owner, EnvironmentV1* result) noexcept {
    if (const auto s=checkOutput(result); s!=Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&]() {
        provider::RockProviderFrameSnapshot frame{};
        if (!provider::runtime::apiGetFrameSnapshot(&frame)) return Status::NotReady;
        result->sample=provider::runtime::sample();
        result->externalBodyCount=frame.externalBodyCount;
        result->gameToHavokScale=frame.gameToHavokScale;
        result->havokToGameScale=frame.havokToGameScale;
        result->physicsScaleRevision=frame.physicsScaleRevision;
        result->weaponGenerationKey=frame.weaponGenerationKey;
        result->weaponBodyCount=frame.weaponBodyCount;
        std::copy_n(frame.weaponBodyIds,8,result->weaponBodyIds);
        return Status::Ok;
    });
}

Status ROCK_CALL copyEvents(OwnerToken owner,std::uint64_t after,EventV1* events,std::uint32_t capacity,StreamV1* state) noexcept {
    if (!state) return Status::InvalidArgument;
    *state={};
    if (capacity && !events) return Status::InvalidArgument;
    if (capacity>256) return Status::CapacityFull;
    for (std::uint32_t i=0;i<capacity;++i) if (const auto status=checkOutput(events+i);status!=Status::Ok) return status;
    return invoke(owner,kInterfaceId,1,false,[&]() { return provider::events::copy(owner,after,events,capacity,*state); });
}
