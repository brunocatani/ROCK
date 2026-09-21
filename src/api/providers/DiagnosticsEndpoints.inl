Status ROCK_CALL getSample(OwnerToken owner, SampleV1* outSample) noexcept {
    return boundary::readSample(owner, kInterfaceId, outSample);
}

Status ROCK_CALL publishDebugOverlayV1(std::uint64_t ownerToken, const DebugOverlayPublicationV1* publication) noexcept {
    if (const auto s=checkInput(publication); s!=Status::Ok) return s;
    return invoke(ownerToken,kInterfaceId,2,true,[&]() {
        if (publication->lineCount>kMaxLinesPerPublisher || publication->textCount>kMaxTextPerPublisher) return Status::CapacityFull;
        if ((publication->lineCount && !publication->lines) || (publication->textCount && !publication->textEntries)) return Status::InvalidArgument;
        std::array<provider::RockProviderDebugOverlayLineV1,kMaxLinesPerPublisher> lines{};
        std::array<provider::RockProviderDebugOverlayTextV1,kMaxTextPerPublisher> texts{};
        for (std::uint32_t i=0;i<publication->lineCount;++i) {
            if (const auto s=checkInput(publication->lines+i); s!=Status::Ok) return s;
            convert(lines[i],publication->lines[i]);
        }
        for (std::uint32_t i=0;i<publication->textCount;++i) {
            if (const auto s=checkInput(publication->textEntries+i); s!=Status::Ok) return s;
            convert(texts[i],publication->textEntries[i]);
        }
        provider::RockProviderDebugOverlayPublicationV1 native{};
        convert(native,*publication);
        native.lines=lines.data(); native.textEntries=texts.data();
        return static_cast<Status>(provider::runtime::apiPublishDebugOverlayV1(ownerToken,&native));
    });
}

Status ROCK_CALL copyEvents(OwnerToken owner,std::uint64_t after,EventV1* events,std::uint32_t capacity,StreamV1* state) noexcept {
    return boundary::copyEventStream(owner,kInterfaceId,after,events,capacity,kEventCapacityPerOwner,state);
}
