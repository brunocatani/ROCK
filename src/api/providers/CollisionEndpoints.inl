Status ROCK_CALL getSample(OwnerToken owner, SampleV1* outSample) noexcept {
    return boundary::readSample(owner, kInterfaceId, outSample);
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
    return boundary::copyEventStream(owner,kInterfaceId,after,events,capacity,kEventCapacityPerOwner,state);
}
