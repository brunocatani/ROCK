Status ROCK_CALL getSample(OwnerToken owner, SampleV1* outSample) noexcept {
    if (!outSample) return Status::InvalidArgument;
    *outSample = {};
    return invoke(owner, kInterfaceId, 1, false, [&]() {
        *outSample = provider::runtime::sample();
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
