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

Status ROCK_CALL copySources(OwnerToken owner,std::uint64_t generation,SourceV1* output,std::uint32_t capacity,std::uint32_t* copied,std::uint32_t* total) noexcept {
    if (!copied || !total) return Status::InvalidArgument;
    *copied=0; *total=0;
    if (capacity>4096) return Status::CapacityFull;
    if (capacity && !output) return Status::InvalidArgument;
    for (std::uint32_t i=0;i<capacity;++i) if (const auto status=boundary::checkOutput(output+i); status!=Status::Ok) return status;
    return invoke(owner,kInterfaceId,1,true,[&]() {
        std::array<provider::WeaponSourceRecord,64> records{};
        std::uint32_t chunkCount=0;
        auto status=provider::runtime::copySources(generation,0,records.data(),0,chunkCount,*total);
        if (status!=Status::Ok) return status;
        const auto requested=std::min(capacity,*total);
        while (*copied<requested) {
            status=provider::runtime::copySources(generation,*copied,records.data(),std::min<std::uint32_t>(64,requested-*copied),chunkCount,*total);
            if (status!=Status::Ok || !chunkCount) {
                for (std::uint32_t i=0;i<*copied;++i) output[i]={};
                *copied=0; return status==Status::Ok?Status::GenerationMismatch:status;
            }
            for (std::uint32_t i=0;i<chunkCount;++i) {
                auto& out=output[*copied+i];
                out.bodyId=records[i].bodyId; out.weaponGenerationKey=records[i].weaponGenerationKey;
                out.sourceKey=records[i].sourceKey; out.parentKey=records[i].parentKey;
                out.sourceParentLocal=records[i].sourceParentLocal; std::copy_n(records[i].name,64,out.name);
            }
            *copied+=chunkCount;
        }
        return Status::Ok;
    });
}

Status ROCK_CALL querySourcePose(OwnerToken owner,std::uint64_t generation,std::uint64_t key,SourcePoseV1* output) noexcept {
    if (const auto status=checkOutput(output);status!=Status::Ok) return status;
    return invoke(owner,kInterfaceId,1,true,[&]() {
        provider::WeaponSourcePose pose{};
        const auto status=provider::runtime::querySourcePose(generation,key,pose);
        if (status!=Status::Ok) return status;
        output->flags=1; output->sample=provider::runtime::sample();
        output->weaponGenerationKey=pose.weaponGenerationKey; output->sourceKey=pose.sourceKey;
        output->sourceParentLocal=pose.sourceParentLocal; output->weaponRootLocal=pose.weaponRootLocal; output->world=pose.world;
        return Status::Ok;
    });
}

Status ROCK_CALL querySourcePath(OwnerToken owner,std::uint64_t generation,std::uint64_t key,std::uint64_t* parentKey,std::uint32_t* childIndex) noexcept {
    if (!parentKey || !childIndex) return Status::InvalidArgument;
    *parentKey=0; *childIndex=0;
    return invoke(owner,kInterfaceId,1,true,[&]() { return provider::runtime::querySourcePath(generation,key,*parentKey,*childIndex); });
}
