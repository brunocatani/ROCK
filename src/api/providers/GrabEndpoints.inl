Status ROCK_CALL getSample(OwnerToken owner, SampleV1* outSample) noexcept {
    return boundary::readSample(owner, kInterfaceId, outSample);
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
    return boundary::copyEventStream(owner,kInterfaceId,after,events,capacity,kEventCapacityPerOwner,state);
}

Status ROCK_CALL setEventCallback(OwnerToken owner,EventCallbackV1 callback,void* user) noexcept {
    if (!callback) return Status::InvalidArgument;
    return invoke(owner,kInterfaceId,1,true,[&]() { return provider::events::setGrabCallback(owner,callback,user); },provider::OwnerAccess::Active);
}
Status ROCK_CALL clearEventCallback(OwnerToken owner) noexcept {
    return invoke(owner,kInterfaceId,1,true,[&]() { provider::events::clearGrabCallback(owner); return Status::Ok; });
}
