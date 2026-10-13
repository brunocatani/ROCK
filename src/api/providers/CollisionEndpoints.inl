Status ROCK_CALL getSample(OwnerToken owner, Sample* outSample) noexcept {
    return boundary::readSample(owner, kInterfaceId, outSample);
}


Status ROCK_CALL copyEvents(OwnerToken owner,std::uint64_t after,EventV1* events,std::uint32_t capacity,Stream* state) noexcept {
    return boundary::copyEventStream(owner,kInterfaceId,after,events,capacity,kEventCapacityPerOwner,state);
}
