Status ROCK_CALL getSample(OwnerToken owner, SampleV1* outSample) noexcept {
    if (!outSample) return Status::InvalidArgument;
    *outSample = {};
    return invoke(owner, kInterfaceId, 1, false, [&]() {
        *outSample = provider::runtime::sample();
        return Status::Ok;
    });
}

Status ROCK_CALL requestInventoryGrab(OwnerToken owner, const InventoryGrabRequestV1* request, std::uint64_t* command) noexcept {
    if (!command) return Status::InvalidArgument;
    *command=0;
    if (const auto s=checkInput(request); s!=Status::Ok) return s;
    return invoke(owner,kInterfaceId,2,true,[&]() {
        provider::RockProviderForceGrabRequestV1 native{};
        native.hand=static_cast<provider::RockProviderHand>(request->hand);
        native.targetFormId=request->baseFormId;
        native.worldGeneration=request->worldGeneration;
        native.skeletonGeneration=request->skeletonGeneration;
        native.providerGeneration=request->providerGeneration;
        native.flags=static_cast<std::uint32_t>(provider::RockProviderForceGrabFlagV1::FromPlayerInventory);
        return static_cast<Status>(provider::runtime::apiRequestForceGrabV1(owner,&native,command));
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

Status ROCK_CALL setEventCallback(OwnerToken owner,EventCallbackV1 callback,void* user) noexcept {
    if (!callback) return Status::InvalidArgument;
    return invoke(owner,kInterfaceId,1,true,[&]() { return provider::events::setGrabCallback(owner,callback,user); });
}
Status ROCK_CALL clearEventCallback(OwnerToken owner) noexcept {
    return invoke(owner,kInterfaceId,1,true,[&]() { provider::events::clearGrabCallback(owner); return Status::Ok; });
}
